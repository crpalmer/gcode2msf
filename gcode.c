#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <assert.h>
#include "gcode.h"
#include "printer.h"
#include "transition-block.h"

#define EXTRA_FILAMENT	75

typedef struct {
    enum {
	MOVE,
	START_TOWER,
	END_TOWER,
	SET_E,
	TOOL,
	PING,
	START,
	OTHER,
	DONE
    } t;
    union {
	struct {
	    double x, y, z, e;
	} move;
	struct {
	    int num, step;
	} ping;
	double e;
	int tool;
    } x;
    long pos;
} token_t;

static FILE *f, *o;

static char buf[1024*1024];

int extrusions = 0;
int gcode_trace = 0;
int validate_only = 0;

static double last_x = 0, last_y = 0, last_z = 0, start_e = 0, last_e = 0, last_e_z = 0, acc_e = 0, last_reported_z = -1;
static bb_t bb;
static int tool = 0;
static int seen_tool = 0;
static int n_used_tools = 0;
static double tower_z = -1;
static int in_tower = 0;
static int seen_ping = 0;
static int started = 0;

run_t runs[MAX_RUNS];
int n_runs = 0;
int used_tool[N_DRIVES] = { 0, };

splice_t splices[MAX_RUNS];
int n_splices = 0;
ping_t pings[MAX_RUNS];
int n_pings = 0;

static double retract_mm = 0;
static double retract_mm_per_min = 0;
static double z_hop = 0;
static double travel_mm_per_min = 0;
static double s3d_default_speed = 0;
static double infill_mm_per_min = 0;
static double first_layer_mm_per_min = 0;

static double mm_per_sec_to_per_min(double mm_per_sec)
{
    return mm_per_sec * 60;
}

static double s3d_speed(double pct)
{
    return s3d_default_speed * pct;
}

struct {
    const char *key;
    double *value;
    double (*normalize)(double);
} gcode_params[] = {
    /* KISSlicer */
    { "; destring_length = ", &retract_mm, NULL },
    { "; destring_speed_mm_per_s = ", &retract_mm_per_min, mm_per_sec_to_per_min },
    { "; Z_lift_mm = ", &z_hop, NULL },
    { "; travel_speed_mm_per_s = ", &travel_mm_per_min, mm_per_sec_to_per_min },
    { "; Sparse Speed = ", &infill_mm_per_min, mm_per_sec_to_per_min },
    { "; first_layer_speed_mm_per_s = ", &first_layer_mm_per_min, mm_per_sec_to_per_min },
    /* S3D */
    { ";   extruderRetractionDistance,", &retract_mm, NULL },
    { ";   extruderRetractionZLift,", &z_hop, NULL },
    { ";   extruderRetractionSpeed,", &retract_mm_per_min, NULL },
    { ";   rapidXYspeed,", &travel_mm_per_min, NULL },
    { ";   defaultSpeed,", &s3d_default_speed, NULL },
    { ";   outlineUnderspeed,", &infill_mm_per_min, s3d_speed },
};

#define N_GCODE_PARAMS (sizeof(gcode_params) / sizeof(gcode_params[0]))

#define STRNCMP(a, b) strncmp(a, b, strlen(b))

static int
find_arg(const char *buf, char arg, double *val)
{
    size_t i;

    for (i = 0; buf[i]; i++) {
	if (buf[i] == ' ' && buf[i+1] == arg) {
	     if (sscanf(&buf[i+2], "%lf", val) > 0) {
		return 1;
	     }
	     return 0;
	}
    }
}

static void
check_for_gcode_params()
{
    int i;

    if (buf[0] != ';') return;

    for (i = 0; i < N_GCODE_PARAMS; i++) {
	if (STRNCMP(buf, gcode_params[i].key) == 0) {
	    double v = atof(buf + strlen(gcode_params[i].key));
	    if (gcode_params[i].normalize) v = gcode_params[i].normalize(v);
	    *gcode_params[i].value = v;
	    break;
	}
    }
}

static token_t
get_next_token_wrapped()
{
    token_t t;

    while (fgets(buf, sizeof(buf), f) != NULL) {
	t.pos = ftell(f);
	if (STRNCMP(buf, "G1 ") == 0) {
	    t.t = MOVE;
	    if (! find_arg(buf, 'X', &t.x.move.x)) t.x.move.x = last_x;
	    if (! find_arg(buf, 'Y', &t.x.move.y)) t.x.move.y = last_y;
	    if (! find_arg(buf, 'Z', &t.x.move.z)) t.x.move.z = last_z;
	    if (! find_arg(buf, 'E', &t.x.move.e)) t.x.move.e = last_e;
	    return t;
	}
	if (STRNCMP(buf, "; finishing tower layer") == 0 ||
	    STRNCMP(buf, "; finishing sparse tower layer") == 0) {
	    t.t = START_TOWER;
	    return t;
	}
	if (STRNCMP(buf, "; leaving transition tower") == 0) {
	    t.t = END_TOWER;
	    return t;
	}
	if (STRNCMP(buf, "; ping ") == 0) {
	    t.t = PING;
	    sscanf(buf, "; ping %d pause %d", &t.x.ping.num, &t.x.ping.step);
	    return t;
	}
	if (STRNCMP(buf, "G92 ") == 0) {
	    t.t = SET_E;
	    if (find_arg(buf, 'E', &t.x.e)) return t;
	}
	if (STRNCMP(buf, "; *** Main G-code ***") == 0 ||
	    STRNCMP(buf, "; layer 1, ") == 0) {
	    t.t = START;
	    return t;
	}
	if (buf[0] == 'T' && isdigit(buf[1])) {
	    t.t = TOOL;
	    t.x.tool = atoi(&buf[1]);
	    return t;
	}
	check_for_gcode_params();
	t.t = OTHER;
	return t;
    }

    t.t = DONE;
    return t;
}

static token_t
get_next_token()
{
    token_t t = get_next_token_wrapped();
    if (gcode_trace) {
	printf("%8ld ", t.pos);
	switch (t.t) {
	case MOVE: printf("MOVE (%f,%f,%f) e=%f\n", t.x.move.x, t.x.move.y, t.x.move.z, t.x.move.e); break;
	case START_TOWER: printf("START_TOWER\n"); break;
	case END_TOWER: printf("END_TOWER\n"); break;
	case PING: printf("PING %d.%d\n", t.x.ping.num, t.x.ping.step); break;
	case SET_E: printf("SET_E %f\n", t.x.e); break;
	case START: printf("START\n"); break;
	case TOOL: printf("TOOL %d\n", t.x.tool); break;
	case OTHER: break;
	default: printf("*** UNKNOWN TOKEN ****\n");
        }
    }
    return t;
}

static void
accumulate()
{
    acc_e += (last_e - start_e);
}

static void
reset_state()
{
    start_e = last_e;
    acc_e = 0;
    seen_ping = 0;
    bb_init(&bb);
}

static void
show_extrusion(char chr, int force)
{
    int bad = in_tower && tower_z != last_e_z && acc_e > 0;

    last_reported_z = last_e_z;
    if (bad || acc_e != 0 || force) {
	if (validate_only && ! bad) return;
	if (! validate_only && ! extrusions) return;

	printf("%c", chr);
	if (seen_tool) printf(" T%d", tool);
	printf(" Z %f", last_e_z);
	if (acc_e != 0) printf(" E %f", acc_e);
	if (bad) printf(" *********** z delta = %f", last_e_z - tower_z);
	if (seen_ping) printf(" [ping %d]", seen_ping);
	printf("\n");
    }
}

static void
add_run()
{
    if (acc_e != 0) {
	if (! used_tool[tool]) {
	    used_tool[tool] = 1;
	    n_used_tools++;
	}

	runs[n_runs].bb = bb;
	runs[n_runs].z = last_e_z;
	runs[n_runs].e = acc_e;
	runs[n_runs].t = tool;
	runs[n_runs].pre_transition = -1;
	runs[n_runs].post_transition = -1;
	n_runs++;
    }
}

static void
preprocess()
{
    reset_state();
    while (1) {
	token_t t = get_next_token();
	switch(t.t) {
	case MOVE:
	    if (t.x.move.e != last_e && t.x.move.z != last_e_z) {
		accumulate();
		if (started) {
		    add_run();
		    show_extrusion('+', 0);
		}
		last_e_z = t.x.move.z;
		reset_state();
	    }
	    bb_add_point(&bb, t.x.move.x, t.x.move.y);
	    last_e = t.x.move.e;
	    last_x = t.x.move.x;
	    last_y = t.x.move.y;
	    last_z = t.x.move.z;
	    break;
	case START_TOWER:
	    accumulate();
	    show_extrusion('+', 0);
	    reset_state();
	    tower_z = last_e_z;
	    in_tower = 1;
	    show_extrusion('>', 1);
	    break;
	case END_TOWER:
	    accumulate();
	    show_extrusion('<', 1);
	    in_tower = 0;
	    reset_state();
	    break;
	case SET_E:
	    accumulate();
	    start_e = t.x.e;
	    last_e = t.x.e;
	    break;
	case TOOL:
	    if (tool != t.x.tool) {
		if (! started) {
		    fprintf(stderr, "** ERROR *** Tool change before in prefix gcode\n");
		    exit(1);
		}
		accumulate();
		add_run();
		if (validate_only && acc_e == 0) printf("Z %.02f ******* Tool change with no extrusion, chroma may screw this up\n", last_z);
		show_extrusion('+', 0);
		reset_state();
		tool = t.x.tool;
		seen_tool = 1;
	    }
	    break;
	case PING:
	    seen_ping = t.x.ping.num;
	    break;
	case START:
	    started = 1;
	    accumulate();
	    reset_state();
	    break;
	case DONE:
	    return;
	case OTHER:
	    break;
	}
    }
}

static double transition_e, transition_starting_e;
static double transition_pct;

static double
extrusion_speed()
{
    // TODO: Handle layer 1 speed
    // TODO: Handle perimeter speed

    if (infill_mm_per_min > 0) return infill_mm_per_min;
    if (printer->print_speed > 0) return printer->print_speed;
}

static void
move_to_and_extrude(double x, double y, double z, double e)
{
    fprintf(o, "G1");
    if (isfinite(x)) fprintf(o, " X%f", x);
    if (isfinite(y)) fprintf(o, " Y%f", y);
    if (isfinite(z)) fprintf(o, " Z%f", z);
    if (isfinite(e)) {
	fprintf(o, " E%f F%f\n", e, extrusion_speed());
	transition_e = e;
    } else {
        fprintf(o, " F%f\n", travel_mm_per_min);
    }
}

static void
move_to(double x, double y, double z)
{
    move_to_and_extrude(x, y, z, NAN);
}

static void
do_retraction()
{
    if (retract_mm) {
	transition_e -= retract_mm;
	fprintf(o, "G1 E%f F%f\n", transition_e, retract_mm_per_min);
    }
}

static void
undo_retraction()
{
    if (retract_mm) {
	transition_e += retract_mm;
	fprintf(o, "G1 E%f F%f\n", transition_e, retract_mm_per_min);
    }
}

static void
add_splice(int drive, double mm)
{
    splices[n_splices].drive = drive;
    splices[n_splices].mm = mm;
    n_splices++;
}

static double
transition_block_corner_x(int corner, double early)
{
    corner = corner % 4;
    switch(corner) {
    case 0: return transition_block.x;
    case 1: return transition_block.x + transition_block.w - early;
    case 2: return transition_block.x + transition_block.w;
    case 3: return transition_block.x + early;
    }
}

static double
transition_block_corner_y(int corner, double early)
{
    corner = corner % 4;
    switch(corner) {
    case 0: return transition_block.y + early;
    case 1: return transition_block.y;
    case 2: return transition_block.y + transition_block.h - early;
    case 3: return transition_block.y + transition_block.h;
    }
}

static double
transition_block_start_x(layer_t *l)
{
    return transition_block_corner_x(l->num % 4, 0);
}

static double
transition_block_start_y(layer_t *l)
{
    return transition_block_corner_y(l->num % 4, 0);
}

static double
extrusion_mm(layer_t *l, double x0, double y0, double x1, double y1)
{
   double len = sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
   double mm3 = len * printer->nozzle * l->h;
   return transition_e + filament_mm3_to_length(mm3);
}

static void
draw_perimeter(layer_t *l, transition_t *t)
{
    int i;
    double last_x = transition_block_start_x(l);
    double last_y = transition_block_start_y(l);

    fprintf(o, "; Drawing the tower perimeter\n");
    for (i = 1; i <= 4; i++) {
	double x = transition_block_corner_x(l->num+i, i == 4 ? printer->nozzle / 2 : 0);
	double y = transition_block_corner_y(l->num+i, i == 4 ? printer->nozzle / 2 : 0);
        move_to_and_extrude(x, y, NAN, extrusion_mm(l, last_x, last_y, x, y));
	last_x = x;
	last_y = y;
    }
}

static double
xy_to_pct(double *xy)
{
    return (xy[0] - transition_block.w + xy[1] - transition_block.h) / (transition_block.w + transition_block.h);
}

static void
pct_to_xy(int x_first, double pct, double *xy)
{
    double dist = pct * (transition_block.w + transition_block.h);
    if (x_first) {
	xy[0] = transition_block.x + dist;
	xy[1] = transition_block.y;
	if (xy[0] > transition_block.x + transition_block.w) {
	    xy[0] = transition_block.x + transition_block.w;
	    xy[1] = transition_block.y + (dist - transition_block.w);
	}
    } else {
	xy[0] = transition_block.x;
	xy[1] = transition_block.y + dist;
	if (xy[1] > transition_block.y + transition_block.h) {
	    xy[0] = transition_block.x + (dist - transition_block.h);
	    xy[1] = transition_block.y + transition_block.h;
	}
    }
}

static void
clamp_xy_to_perimeter(int x_first, double *xy)
{
    double pct = xy_to_pct(xy);
    pct_to_xy(x_first, pct, xy);
}

static void
transition_fill(layer_t *l, transition_t *t)
{
    double stride = printer->nozzle / l->density;
    double xy[2], next_xy[2];
    int x_first = 1;

    fprintf(o, "; Filling in the tower portion\n");

    while (transition_e - transition_start_e < t->mm) {
	pct_to_xy(x_first, transition_pct, xy);
	next_xy[0] = xy[0] + stride;
	next_xy[1] = xy[1];
	clamp_xy_to_perimeter(x_first, next_xy);

	move_to_and_extrude(next_xy[0], next_xy[1], NAN, extrusion_mm(l, xy[0], xy[1], next_xy[0], next_xy[1]);

	next_x += printer->nozzle;
	transition_next_y += printer->nozzle;
	if (transition_next_x > transition_block.x + transition_block.w) break;
	if (transition_next_y > 
#endif
}

static void
generate_transition(layer_t *l, transition_t *t, double *total_e)
{
    double actual_e;

    transition_e = last_e;

    fprintf(o, "; Transition: %d->%d with %f mm\n", t->from, t->to, t->mm);
    // assume retraction was done just before tool change: do_retraction();
    move_to(NAN, NAN, l->z + z_hop);

    if (l->transition0 == t->num) {
	transition_pct = 0;
printf("layer %d (%d transitions): density %f\n", l->num, l->n_transitions, l->density);
    }
    move_to(transition_block_start_x(l), transition_block_start_y(l), NAN);
    if (z_hop) move_to(NAN, NAN, l->z);

    undo_retraction();

    transition_starting_e = transition_e;

    if (0 && t->num == 0) draw_perimeter(l, t);

    transition_fill(l, t);

    actual_e = transition_e - transition_starting_e;

    do_retraction();

    move_to(NAN, NAN, last_z+z_hop);
    move_to(last_x, last_y, NAN);
    if (z_hop) move_to(NAN, NAN, last_z);
    // assume unretraction will done just immediately after tool change: undo_retraction();

    if (t->ping) {
	// TODO: actually report the correct extrusion for the ping
	pings[n_pings].mm = *total_e + actual_e;
	n_pings++;
    }
    fprintf(o, "; Done transition: %d->%d actually used %fmm for %fmm\n", t->from, t->to, actual_e, t->mm);
    if (t->from != t->to) add_splice(t->from, *total_e + actual_e * printer->transition_target);

    *total_e += actual_e;
}

static void
produce_gcode()
{
    int l = 0;
    int t = 0;
    double last_e_z = 0, last_e_tool;
    int seen_tool = 0;
    int tool = 0;
    int started = 0;
    double total_e;

    last_e = last_x = last_y = last_z = 0;

    rewind(f);
    while (1) {
	token_t token = get_next_token();

	switch(token.t) {
	case MOVE: {
	    int may_need_transition = 0;

	    if (tool != last_e_tool) may_need_transition = 1;
	    if (token.x.move.z != last_e_z && layers[l].n_transitions == 1 && transitions[t].from == transitions[t].to) may_need_transition = 1;

	    if (t < n_transitions && started && token.x.move.e != last_e && may_need_transition) {
		last_e_z = token.x.move.z;
		last_e_tool = tool;
		generate_transition(&layers[l], &transitions[t], &total_e);
		assert(token.x.move.z == layers[l].z);
		t++;
		if (layers[l].transition0 + layers[l].n_transitions == t) l++;
		assert(layers[l].transition0 <= t && t < layers[l].transition0 + layers[l].n_transitions);
	    }
	    total_e += token.x.move.e - last_e;
	    last_e = token.x.move.e;
	    last_x = token.x.move.x;
	    last_y = token.x.move.y;
	    last_z = token.x.move.z;
	    fprintf(o, "%s", buf);
	    break;
	}
	case START:
	    started = 1;
	    break;
	case START_TOWER:
	case END_TOWER:
	case PING:
	case OTHER:
	    fprintf(o, "%s", buf);
	    break;
	case SET_E:
	    last_e = 0;
	    fprintf(o, "%s", buf);
	    break;
	case TOOL:
	    if (! seen_tool) {
		seen_tool = 1;
		last_e_tool = token.x.tool;
	    }
	    tool = token.x.tool;
	    fprintf(o, "; Switching to tool %d\n", tool);
	    break;
	case DONE:
	    total_e += printer->bowden_len > 0 ? printer->bowden_len : EXTRA_FILAMENT;
	    add_splice(tool, total_e);
	    return;
	}
    }
}

void gcode_to_runs(const char *fname)
{
    if ((f = fopen(fname, "r")) == NULL) {
	perror(fname);
	return;
    }

    preprocess();
}

void gcode_to_msf_gcode(const char *output_fname)
{
    if ((o = fopen(output_fname, "w")) == NULL) {
	perror(output_fname);
	return;
    }

    produce_gcode();
    fclose(o);
    o = NULL;
}
