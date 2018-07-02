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

#define EPSILON 0.0000001

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
	case OTHER: printf("%s", buf); break;
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

static double layer_transition_e;
static double transition_e, transition_starting_e;
static double transition_pct;
static double ping_complete_e;

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
add_splice(int drive, double mm, double waste)
{
    splices[n_splices].drive = drive;
    splices[n_splices].mm = mm;
    splices[n_splices].waste = waste;
    n_splices++;
}

static void
generate_pause(int ms)
{
    while (ms > 4000) {
	fprintf(o, "G4 P4000\n");
	ms -= 4000;
    }
    if (ms > 0) fprintf(o, "G4 P%d\n", ms);
}

static void
move_off_tower(double x, double y)
{
    double new_x, new_y;

    if (fabs(transition_block.x - x) < fabs(transition_block.x + transition_block.w - x)) {
	new_x = transition_block.x - printer->nozzle;
    } else {
	new_x = transition_block.x + transition_block.w + printer->nozzle;
    }

    if (fabs(transition_block.y - y) < fabs(transition_block.y + transition_block.h - y)) {
	new_y = transition_block.y - printer->nozzle;
    } else {
	new_y = transition_block.y + transition_block.h + printer->nozzle;
    }

    if (fabs(x  - new_x) < fabs(y - new_y)) move_to(new_x, NAN, NAN);
    else move_to(NAN, new_y, NAN);
}

static void
check_ping_complete(double x, double y)
{
    if (ping_complete_e > 0 && transition_e > ping_complete_e) {
	fprintf(o, "; Ping extrusion complete at %f, doing second pause\n", transition_e);
	ping_complete_e = 0;
	do_retraction();
	if (printer->ping_off_tower) move_off_tower(x, y);
	generate_pause(7000);
        if (printer->ping_off_tower) move_to(x, y, NAN);
	undo_retraction();
	fprintf(o, "; Resuming transition block\n");
    }
}

typedef enum {
    BOTTOM_LEFT = 0, BOTTOM_RIGHT, TOP_RIGHT, TOP_LEFT
} corner_t;

static corner_t
layer_to_corner(layer_t *l)
{
    switch(l->num % 4) {
    case 0: return BOTTOM_LEFT;
    case 1: return TOP_LEFT;
    case 2: return BOTTOM_RIGHT;
    case 3: return TOP_RIGHT;
    }
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
    corner_t corner = layer_to_corner(l);
    double last_x = transition_block_corner_x(corner, 0);
    double last_y = transition_block_corner_y(corner, 0);

    fprintf(o, "; Drawing the tower perimeter\n");
    for (i = 1; i <= 4; i++) {
	double x = transition_block_corner_x(corner+i, i == 4 ? printer->nozzle / 2 : 0);
	double y = transition_block_corner_y(corner+i, i == 4 ? printer->nozzle / 2 : 0);
        move_to_and_extrude(x, y, NAN, extrusion_mm(l, last_x, last_y, x, y));
	check_ping_complete(x, y);
	last_x = x;
	last_y = y;
    }
}

typedef struct {
    double x, y;
} xy_t;

static double
transition_block_adjusted_x(layer_t *l)
{
    return transition_block.x + (l->use_perimeter ? printer->nozzle/2 : 0);
}

static double
transition_block_adjusted_y(layer_t *l)
{
    return transition_block.y + (l->use_perimeter ? printer->nozzle/2 : 0);
}

static double
transition_block_adjusted_w(layer_t *l)
{
    return transition_block.w - 2*(l->use_perimeter ? printer->nozzle/2 : 0);
}

static double
transition_block_adjusted_h(layer_t *l)
{
    return transition_block.h - 2*(l->use_perimeter ? printer->nozzle/2 : 0);
}

static double
xy_to_pct(layer_t *l, xy_t *xy)
{
    corner_t corner = layer_to_corner(l);
    double x = transition_block_adjusted_x(l);
    double y = transition_block_adjusted_y(l);
    double w = transition_block_adjusted_w(l);
    double h = transition_block_adjusted_h(l);
    double used;

    switch(corner) {
    case BOTTOM_LEFT:  used = xy->x - x     + xy->y - y;     break;
    case TOP_LEFT:     used = xy->x - x     + y + h - xy->y; break;
    case TOP_RIGHT:    used = x + w - xy->x + y + h - xy->y; break;
    case BOTTOM_RIGHT: used = x + w - xy->x + xy->y - y;     break;
    }

    assert(used >= 0);

    return used / (transition_block_adjusted_w(l) + transition_block_adjusted_h(l));
}

static void
pct_to_xy(layer_t *l, int is_y_first, double pct, xy_t *xy)
{
    corner_t corner = layer_to_corner(l);
    double dist, dist_x, dist_y;
    double x = transition_block_adjusted_x(l);
    double y = transition_block_adjusted_y(l);
    double w = transition_block_adjusted_w(l);
    double h = transition_block_adjusted_h(l);
    int dir_x, dir_y;

    dist = pct * (transition_block_adjusted_w(l) + transition_block_adjusted_h(l));

    switch(corner) {
    case BOTTOM_LEFT:  dir_x =  1; dir_y =  1; break;
    case TOP_LEFT:     dir_x =  1; dir_y = -1; y += h; break;
    case TOP_RIGHT:    dir_x = -1; dir_y = -1; x += w; y += h; break;
    case BOTTOM_RIGHT: dir_x = -1; dir_y =  1; x += w; break;
    }

    if (is_y_first == 0) {
	dist_x = dist > w ? w : dist;
	dist_y = dist > w ? dist - w : 0;
    } else {
	dist_x = dist > h ? dist - h : 0;
	dist_y = dist > h ? h : dist;
    }

    xy->x = x + dir_x * dist_x;
    xy->y = y + dir_y * dist_y;
}

static void
clamp_xy_to_perimeter(layer_t *l, int is_y_first, xy_t *xy)
{
    double pct = xy_to_pct(l, xy);
    if (pct > 1) pct = 1;
    pct_to_xy(l, is_y_first, pct, xy);
}

static void
transition_fill(layer_t *l, transition_t *t)
{
    double stride0 = (1 / l->density) * printer->nozzle;
    double stride = sqrt(2 * stride0 * stride0);
    double x_stride = stride;
    double y_stride = stride;
    xy_t xy, next_xy;
    int is_last = t->num == l->transition0 + l->n_transitions - 1;
    corner_t corner = layer_to_corner(l);

    fprintf(o, "; Filling in the tower portion, density = %f\n", l->density);

    if (corner == TOP_RIGHT || corner == BOTTOM_RIGHT) x_stride = -x_stride;
    if (corner == TOP_LEFT || corner == TOP_RIGHT) y_stride = -y_stride;

    pct_to_xy(l, 0, transition_pct, &xy);
    while ((is_last || transition_e - transition_starting_e < t->mm + t->extra_mm) && transition_pct < 1 - EPSILON) {
	int is_y_first;

	for (is_y_first = 0; is_y_first < 2 && transition_pct < 1 - EPSILON; is_y_first++) {
	    /* move along the first side */
	    next_xy = xy;
	    if (is_y_first) next_xy.x += x_stride;
	    else next_xy.y += y_stride;
	    clamp_xy_to_perimeter(l, is_y_first, &next_xy);
	    move_to_and_extrude(next_xy.x, next_xy.y, NAN, extrusion_mm(l, xy.x, xy.y, next_xy.x, next_xy.y));
	    check_ping_complete(next_xy.x, next_xy.y);
	    transition_pct = xy_to_pct(l, &next_xy);

	    if (transition_pct >= 1 - EPSILON) break;

	    /* cross over to the other side */
	    xy = next_xy;
	    pct_to_xy(l, ! is_y_first, transition_pct, &next_xy);
	    move_to_and_extrude(next_xy.x, next_xy.y, NAN, extrusion_mm(l, xy.x, xy.y, next_xy.x, next_xy.y));
	    check_ping_complete(next_xy.x, next_xy.y);
	    transition_pct = xy_to_pct(l, &next_xy);
	    xy = next_xy;
	}
    }

    if (transition_pct > 1) {
	if (is_last) {} // fprintf(stderr, "WARNING: layer %d generated pct=%f extra transition\n", l->num + 1, transition_pct);
	else fprintf(stderr, "WARNING: generated less transition than needed on layer %d, pct=%f\n", l->num + 1, transition_pct);
    }
}

static void
generate_transition(layer_t *l, transition_t *t, double *total_e)
{
    double original_e;
    double actual_e;
    double waste;
    xy_t start_xy;

    original_e = transition_e = last_e;

    waste =  t->mm * printer->transition_target + t->extra_mm;
    if (t->from != t->to) {
	if (n_splices > 0) splices[n_splices-1].waste += (1-printer->transition_target) * t->mm;
	add_splice(t->from, *total_e + waste, waste);
    }

    fprintf(o, "; Transition: %d->%d with %f + %f mm\n", t->from, t->to, t->mm, t->extra_mm);
    // assume retraction was done just before tool change: do_retraction();
    move_to(NAN, NAN, l->z + z_hop);

    if (l->transition0 == t->num) {
	transition_pct = 0;
	layer_transition_e = 0;
    }

    pct_to_xy(l, 0, transition_pct, &start_xy);  // TODO: alternate corners
    move_to(start_xy.x, start_xy.y, NAN);
    if (z_hop) move_to(NAN, NAN, l->z);

    if (t->ping) {
	ping_complete_e = transition_e + 20;
	fprintf(o, "; Starting initial ping pause at %f complete at %f\n", transition_e, ping_complete_e);
	if (printer->ping_off_tower) move_off_tower(start_xy.x, start_xy.y);
	generate_pause(13000);
	if (printer->ping_off_tower) move_to(start_xy.x, start_xy.y, NAN);
    } else {
	ping_complete_e = 0;
    }

    undo_retraction();

    transition_starting_e = transition_e;

    if (l->transition0 == t->num && l->use_perimeter) draw_perimeter(l, t);

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

    fprintf(o, "G92 E%f\n", original_e);
    fprintf(o, "; Done transition: %d->%d actually used %f mm for %f + %f mm\n", t->from, t->to, actual_e, t->mm, t->extra_mm);

    *total_e += actual_e;
    layer_transition_e += actual_e;

    assert(t->num != l->transition0 + l->n_transitions - 1 || fabs(transition_pct - 1) < 0.001);
    assert(t->num != l->transition0 + l->n_transitions - 1 || l->mm - 1 <= layer_transition_e <= l->mm+5);
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
	    add_splice(tool, total_e, 0);
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
