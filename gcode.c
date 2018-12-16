#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <assert.h>
#include "bed-usage.h"
#include "gcode.h"
#include "printer.h"
#include "transition-block.h"

#define EPSILON 0.0000001

typedef struct {
    enum {
	MOVE,
	SET_E,
	TOOL,
	START,
	OTHER,
	FAN,
	KISS_EXT,
	DONE
    } t;
    union {
	struct {
	    double x, y, z, e, f;
	    int changes_position;
	} move;
	struct {
	    int num, step;
	} ping;
	double e;
	int tool;
	double fan;
    } x;
    long pos;
} token_t;

static const char *path_names[] = { "normal", "infill", "support", "interface" };
typedef enum { UNKNOWN = 0, KISSLICER, SIMPLIFY3D, SLIC3R } slicer_t;
const char *slicer_names[] = { "Unknown", "KISSlicer", "Simplify3D", "Slic3r" };

static slicer_t slicer;
static int has_started;

typedef struct {
    double total_e;
    double acc_transition;
    double acc_waste;
    int    next_move_full;
} extrusion_state_t;

static FILE *f, *o;

static char buf[1024*1024];

int extrusions = 0;
int gcode_trace = 0;
int validate_only = 0;
int debug_tool_changes = 0;
int stop_at_ping = -1;
int squash_interface = 0;

static double last_x = 0, last_y = 0, last_z = 0, last_e = 0, last_f = 0;
static double start_e = 0, start_z = NAN;
static int e_is_absolute = 1;
static int in_slic3r_crap = 0;
static path_t start_path = NORMAL, cur_path = NORMAL;
static double last_fan = 0;
static int tool = 0;
static int seen_tool = 0;
static int n_used_tools = 0;

run_t runs[MAX_RUNS];
int n_runs = 0;
int used_tool[N_DRIVES] = { 0, };
double tool_mm[N_DRIVES] = { 0, };
bed_usage_t *bed_usage;

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
static double flow_max_mm3_per_sec = DBL_MAX;

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
    { "; flow_max_mm3_per_s = ", &flow_max_mm3_per_sec, NULL },
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
    return 0;
}

static int
has_arg(const char *buf, char arg)
{
    double v;

    return find_arg(buf, arg, &v);
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

static void
check_for_kisslicer_path_types()
{
    const char *p;
    double feed, head;

    if (slicer != KISSLICER) return;

    if (STRNCMP(buf, "; '") != 0) return;
    if ((p = strchr(buf+3, '\'')) == NULL) return;
    if (sscanf(p, "', %lf [feed mm/s], %lf [head mm/s]", &feed, &head) != 2) return;

    /* Ignore destrings */
    if (STRNCMP(buf, "; 'Destring/Wipe/Jump Path'") == 0) return;

    if (STRNCMP(buf, "; 'Support (may Stack) Path'") == 0) cur_path = SUPPORT;
    else if (STRNCMP(buf, "; 'Support Interface Path'") == 0) cur_path = INTERFACE;
    else if (STRNCMP(buf, "; 'Sparse Infill Path'") == 0) cur_path = INFILL;
    else if (STRNCMP(buf, "; 'Stacked Sparse Infill Path'") == 0) cur_path = INFILL;
    else cur_path = NORMAL;
}

static void
check_for_simplify3d_path_types()
{
    const char *p;

    if (slicer != SIMPLIFY3D) return;

    if (buf[0] != ';') return;
    for (p = buf+1; *p && (isspace(*p) || islower(*p)); p++) {}
    if (*p) return;

    if (STRNCMP(buf, "; infill") == 0 || STRNCMP(buf, "; feature infill") == 0) cur_path = INFILL;
    else if (STRNCMP(buf, "; support") == 0 || STRNCMP(buf, "; feature support") == 0) cur_path = SUPPORT;
    else if (STRNCMP(buf, "; dense support") == 0 || STRNCMP(buf, "; feature dense support") == 0) cur_path = SUPPORT;
    else cur_path = NORMAL;
}

static long next_pos = 0;

static void
rewind_input()
{
    rewind(f);
    next_pos = 0;
    has_started = 0;
    e_is_absolute = 1;
    in_slic3r_crap = 0;
    tool = 0;
}

static token_t
get_next_token_wrapped()
{
    token_t t;

    while (fgets(buf, sizeof(buf), f) != NULL) {
	t.t = OTHER;
	t.pos = next_pos;
	next_pos = ftell(f);
	if (in_slic3r_crap && (
	    STRNCMP(buf, "G1 E-15.0000") == 0 ||
	    STRNCMP(buf, "G1 E10.5000 F5400") == 0 ||
	    STRNCMP(buf, "G1 E3.0000 F2700") == 0 ||
	    STRNCMP(buf, "G1 E1.5000 F1620") == 0)
	) {
	    continue;
	}

	if (STRNCMP(buf, "G1 ") == 0) {
	    t.t = MOVE;
	    t.x.move.changes_position = has_arg(buf, 'X') || has_arg(buf, 'Y') || has_arg(buf, 'Z');
	    if (! find_arg(buf, 'X', &t.x.move.x)) t.x.move.x = last_x;
	    if (! find_arg(buf, 'Y', &t.x.move.y)) t.x.move.y = last_y;
	    if (! find_arg(buf, 'E', &t.x.move.e)) t.x.move.e = last_e;
	    else if (! e_is_absolute) t.x.move.e += last_e;
	    if (! find_arg(buf, 'F', &t.x.move.f)) t.x.move.f = last_f;
	    if (! find_arg(buf, 'Z', &t.x.move.z)) {
		t.x.move.z = last_z;
	    } else {
		if (slicer == SLIC3R && ! has_started) {
		    next_pos = t.pos;
		    fseek(f, next_pos, SEEK_SET);
		    t.t = START;
		    has_started = 1;
		    return t;
		}
	    }
	    return t;
	}

	if (STRNCMP(buf, "M82 ") == 0 || strcmp(buf, "M82") == 0) {
	    e_is_absolute = 1;
	    continue;
	}

	if (STRNCMP(buf, "M83 ") == 0 || strcmp(buf, "M83") == 0) {
	    e_is_absolute = 0;
	    continue;
	}

	if (STRNCMP(buf, "; CP TOOLCHANGE UNLOAD") == 0) {
	    in_slic3r_crap = 1;
	}

	if (STRNCMP(buf, "; G-Code generated by Simplify3D(R)") == 0) {
	    slicer = SIMPLIFY3D;
	    return t;
	}

	if (STRNCMP(buf, "; KISSlicer") == 0) {
	    slicer = KISSLICER;
	    return t;
	}

	if (STRNCMP(buf, "; generated by Slic3r") == 0) {
	    slicer = SLIC3R;
	    return t;
	}

	if (STRNCMP(buf, "G92 ") == 0) {
	    t.t = SET_E;
	    if (find_arg(buf, 'E', &t.x.e)) return t;
	}
	if (STRNCMP(buf, "M106 ") == 0) {
	    t.t = FAN;
	    if (find_arg(buf, 'S', &t.x.fan)) return t;
	}
	if (STRNCMP(buf, "M107") == 0) {
	    t.t = FAN;
	    t.x.fan = 0;
	    return  t;
	}
	if (STRNCMP(buf, "; *** Main G-code ***") == 0 ||
	    STRNCMP(buf, "; layer 1, ") == 0) {
	    has_started = 1;
	    t.t = START;
	    return t;
	}
	if (STRNCMP(buf, ";    Ext ") == 0 && sscanf(buf, ";    Ext %d =  %*f mm", &t.x.tool) == 1) {
	    t.t = KISS_EXT;
	    t.x.tool--;
	    return t;
	}
	if (buf[0] == 'T' && isdigit(buf[1])) {
	    t.t = TOOL;
	    t.x.tool = atoi(&buf[1]);
	    in_slic3r_crap = 0;
	    return t;
	} check_for_gcode_params();
	check_for_kisslicer_path_types();
	check_for_simplify3d_path_types();
	return t;
    }

    t.t = DONE;
    return t;
}

static void
update_last_state(token_t *t)
{
    last_f = t->x.move.f;
    last_e = t->x.move.e;
    last_x = t->x.move.x;
    last_y = t->x.move.y;
    last_z = t->x.move.z;
}

static token_t
get_next_token()
{
    token_t t = get_next_token_wrapped();
    if (gcode_trace) {
	printf("%8ld ", t.pos);
	switch (t.t) {
	case MOVE: printf("MOVE (%f,%f,%f) e=%f path=%s%s\n", t.x.move.x, t.x.move.y, t.x.move.z, t.x.move.e, path_names[cur_path], t.x.move.changes_position ? " changes-pos" : ""); break;
	case SET_E: printf("SET_E %f\n", t.x.e); break;
	case START: printf("START\n"); break;
	case TOOL: printf("TOOL %d\n", t.x.tool); break;
	case FAN: printf("FAN %f\n", t.x.fan); break;
	case OTHER: printf("%s", buf); break;
	case KISS_EXT: printf("KISS_EXT %d\n", t.x.tool); break;
	default: printf("*** UNKNOWN TOKEN ****\n");
        }
    }
    return t;
}

static void
add_run(long offset)
{
    double delta_e = last_e - start_e;

    if (start_path == INTERFACE && squash_interface) {
	return;
    }

    if (delta_e > 0 && offset >= 0) {
	if (! used_tool[tool]) {
	    used_tool[tool] = 1;
	    n_used_tools++;
	}

	if (n_runs > 0 && runs[n_runs-1].t == tool && runs[n_runs-1].z == start_z && runs[n_runs-1].path == start_path) {
	    runs[n_runs-1].e += delta_e;
	} else {
	    runs[n_runs].z = start_z;
	    runs[n_runs].e = delta_e;
	    runs[n_runs].t = tool;
	    runs[n_runs].path = start_path;
	    runs[n_runs].offset = offset;
	    runs[n_runs].next_move_no_extrusion = 0;
	    n_runs++;
	}
    }
    tool_mm[tool] += delta_e;
}

static int
compatible_runs(run_t *r1, run_t *r2)
{
    return r1->z >= r2->z && r1->t == r2->t;
}

static void
merge_consecutive_runs()
{
    int i, j, k;
    int next_run;

    for (i = 0, next_run = 0; next_run < n_runs; i++) {
	runs[i] = runs[next_run];
	runs[i].leading_support_mm = 0;
	for (j = next_run; j < n_runs && compatible_runs(&runs[i], &runs[j]) && runs[j].path == SUPPORT; j++) {
	    runs[i].leading_support_mm += runs[j].e;
	}
	for (j = next_run+1; j < n_runs && compatible_runs(&runs[i], &runs[j]); j++) {
	    runs[i].e += runs[j].e;
	}
	runs[i].trailing_infill_mm = 0;
	for (k = next_run-1; k >= next_run && runs[k].path == INFILL; k--)  {
	    runs[i].trailing_infill_mm += runs[k].e;
	}
	next_run = j;
    }

    n_runs = i;
}

static void merge_run(run_t *pre, run_t *next)
{
    pre->z = next->z;
    pre->trailing_infill_mm = next->trailing_infill_mm;
    pre->e += next->e;
}

static void
merge_negative_height_runs()
{
    int i;
    int next_run;
    int n_merged = 0;

    for (i = 0, next_run = 0; next_run < n_runs; i++) {
	double last_z = i > 0 ? runs[i-1].z : 0;

	runs[i] = runs[next_run++];
	while (next_run < n_runs &&
		runs[i].t == runs[next_run].t &&
		runs[i].z < last_z &&
		runs[next_run].z - last_z < printer->max_layer_height)
	{
	    merge_run(&runs[i], &runs[next_run++]);
	    n_merged++;
	}
    }

    n_runs = i;

    if (n_merged > 0) fprintf(stderr, "Warning: %d runs were merged due to negative layer heights\n", n_merged);
}
    
static void
merge_compatible_runs()
{
    int i;
    int next_run;

    for (i = 0, next_run = 0; next_run < n_runs; i++) {
	double last_z = i > 0 ? runs[i-1].z : 0;

	runs[i] = runs[next_run++];
	while (next_run < n_runs &&
	    runs[i].t == runs[next_run].t &&
	    last_z < runs[i].z &&
	    runs[i].z < runs[next_run].z &&
	    runs[i].t == runs[next_run].t &&
	    runs[next_run].z - last_z < printer->max_layer_height)
	{
	    merge_run(&runs[i], &runs[next_run]);
	    next_run++;
	}

	if (runs[i].z < last_z) {
	    fprintf(stderr, "Cannot complete gcode processing.\nExtrusion at height %f occurs before extrusion at height %f\n", last_z, runs[i].z);
	    exit(1);
	}
    }

    n_runs = i;
}

static void
prune_runs()
{
    merge_consecutive_runs();
    merge_negative_height_runs();
    merge_compatible_runs();
}

static void
preprocess()
{
    long start_offset = 0;
    int check_next_move = 0;

    if (bed_usage) bed_usage_destroy(bed_usage);
    bed_usage = bed_usage_new();

    while (1) {
	token_t t = get_next_token();
	switch(t.t) {
	case MOVE:
	    if (check_next_move && t.x.move.changes_position) {
		if (n_runs > 0) runs[n_runs-1].next_move_no_extrusion = t.x.move.e == last_e;
		check_next_move = 0;
	    }

	    if (t.x.move.e > last_e && t.x.move.changes_position) {
		if (start_z != t.x.move.z) bed_usage_new_layer(bed_usage, t.x.move.z);
	        if (t.x.move.z != start_z || start_path != cur_path) {
		    add_run(start_offset);
		    start_z = t.x.move.z;
		    start_e = last_e;
		    start_path = cur_path;
		    check_next_move = 1;
	        }
	        bed_usage_extrude(bed_usage, last_x, last_y, t.x.move.x, t.x.move.y);
		if (start_offset < 0) start_offset = ftell(f);
	    }

	    update_last_state(&t);
	    break;
	case SET_E:
	    add_run(start_offset);
	    start_e = last_e = t.x.e;
	    start_offset = -1;
	    break;
	case FAN:
	    last_fan = t.x.fan;
	    break;
	case TOOL:
	    if (tool != t.x.tool) {
		if (! has_started) {
		    fprintf(stderr, "** ERROR *** Tool change before in prefix gcode\n");
		    exit(1);
		}
		add_run(t.pos);
		start_e = last_e;
		check_next_move = 1;
		tool = t.x.tool;
		seen_tool = 1;
	    }
	    break;
	case START:
	    start_z = last_z;;
	    break;
	case DONE:
    	    prune_runs();
	    return;
	case KISS_EXT:
	case OTHER:
	    break;
	}
    }
}

static int is_first_layer = 1;
static double layer_transition_e;
static double transition_e, transition_starting_e;
static double transition_pct;
static double ping_complete_e;
static double total_ext[N_DRIVES];

static double
base_extrusion_speed(double layer_height)
{
    if (infill_mm_per_min > 0) return infill_mm_per_min;
    if (printer->print_speed > 0) return printer->print_speed;
    return 30*60;
}

static double
extrusion_speed(double layer_height)
{
    double mm_per_min = base_extrusion_speed(layer_height);
    if (is_first_layer && mm_per_min > first_layer_mm_per_min && first_layer_mm_per_min > 0) mm_per_min = first_layer_mm_per_min;
    if (speed_to_flow_rate(mm_per_min, layer_height) > flow_max_mm3_per_sec && flow_max_mm3_per_sec > 0) mm_per_min = flow_rate_to_speed(flow_max_mm3_per_sec, layer_height);
    return mm_per_min;
}


static void
move_to_and_extrude_common(double x, double y, double z, double e, double speed_multiplier, double layer_height)
{
    fprintf(o, "G1");
    if (isfinite(x)) fprintf(o, " X%f", x);
    if (isfinite(y)) fprintf(o, " Y%f", y);
    if (isfinite(z)) fprintf(o, " Z%f", z);
    if (isfinite(e)) {
	fprintf(o, " E%f F%f\n", e_is_absolute ? e : e - transition_e, extrusion_speed(layer_height) * speed_multiplier);
	transition_e = e;
    } else {
        fprintf(o, " F%f\n", travel_mm_per_min);
    }
}

static void
move_to_and_extrude(double x, double y, double z, double e, double layer_height)
{
    return move_to_and_extrude_common(x, y, z, e, 1, layer_height);
}

static void
move_to_and_extrude_perimeter(double x, double y, double z, double e, double layer_height)
{
    return move_to_and_extrude_common(x, y, z, e, printer->perimeter_speed_multiplier, layer_height);
}

static void
move_to(double x, double y, double z)
{
    move_to_and_extrude(x, y, z, NAN, NAN);
}

static void
do_retraction()
{
    if (retract_mm) {
	transition_e -= retract_mm;
	fprintf(o, "G1 E%f F%f\n", e_is_absolute ? transition_e : -retract_mm, retract_mm_per_min);
    }
}

static void
undo_retraction()
{
    if (retract_mm) {
	transition_e += retract_mm;
	fprintf(o, "G1 E%f F%f\n", e_is_absolute ? transition_e : retract_mm, retract_mm_per_min);
    }
}

static void
add_splice(int drive, double mm, double pre_mm, extrusion_state_t *e)
{
    splices[n_splices].drive = drive;
    splices[n_splices].mm = mm + pre_mm;
    splices[n_splices].waste = e->acc_waste + pre_mm;
    splices[n_splices].transition_mm = e->acc_transition + pre_mm;
    total_ext[drive] += splices[n_splices].mm - (n_splices == 0 ? 0 : splices[n_splices-1].mm);
    n_splices++;

    e->acc_waste = e->acc_transition = -pre_mm;
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
    case 2: return TOP_RIGHT;
    case 3: return BOTTOM_RIGHT;
    }
    assert(0);
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
    assert(0);
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
    assert(0);
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
        move_to_and_extrude_perimeter(x, y, NAN, extrusion_mm(l, last_x, last_y, x, y), l->h);
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

typedef struct {
    double x,y,e;
} xye_t;

static void
record_move_and_extrude(xye_t *xye, xy_t *xy, double e)
{
    xye->x = xy->x;
    xye->y = xy->y;
    xye->e = e;
    transition_e = e;
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
    static xye_t xye[10000];
    int n_xye = 0;
    double scale, start;
    int i;

    if (corner == TOP_RIGHT || corner == BOTTOM_RIGHT) x_stride = -x_stride;
    if (corner == TOP_LEFT || corner == TOP_RIGHT) y_stride = -y_stride;

    start = transition_e;

    pct_to_xy(l, 0, transition_pct, &xy);
    while ((is_last || transition_e - transition_starting_e < t->pre_mm + t->post_mm) && transition_pct < 1 - EPSILON) {
	int is_y_first;

	for (is_y_first = 0; is_y_first < 2 && transition_pct < 1 - EPSILON; is_y_first++) {
	    /* move along the first side */
	    next_xy = xy;
	    if (is_y_first) next_xy.x += x_stride;
	    else next_xy.y += y_stride;
	    clamp_xy_to_perimeter(l, is_y_first, &next_xy);
	    record_move_and_extrude(&xye[n_xye++], &next_xy, extrusion_mm(l, xy.x, xy.y, next_xy.x, next_xy.y));
	    transition_pct = xy_to_pct(l, &next_xy);

	    if (transition_pct >= 1 - EPSILON) break;

	    /* cross over to the other side */
	    xy = next_xy;
	    pct_to_xy(l, ! is_y_first, transition_pct, &next_xy);
	    record_move_and_extrude(&xye[n_xye++], &next_xy, extrusion_mm(l, xy.x, xy.y, next_xy.x, next_xy.y));
	    transition_pct = xy_to_pct(l, &next_xy);
	    xy = next_xy;
	}
    }

    scale = ((t->pre_mm + t->post_mm) - (start - transition_starting_e)) / (xye[n_xye-1].e - start);

    fprintf(o, "; Filling in the tower portion, density = %f, extrusion-width = %f\n", l->density, printer->nozzle * scale);

    for (i = 0; i < n_xye; i++) {
	move_to_and_extrude(xye[i].x, xye[i].y, NAN, (xye[i].e - start) * scale + start, l->h);
	check_ping_complete(xye[i].x, xye[i].y);
    }

    assert(t->pre_mm + t->post_mm - 0.01 < transition_e - transition_starting_e &&
	   transition_e - transition_starting_e < t->pre_mm + t->post_mm + 0.01);

    if (transition_pct > 1) {
	if (is_last) {} // fprintf(stderr, "WARNING: layer %d generated pct=%f extra transition\n", l->num + 1, transition_pct);
	else fprintf(stderr, "WARNING: generated less transition than needed on layer %d, pct=%f\n", l->num + 1, transition_pct);
    }
}

static void
report_speed(FILE *o, layer_t *l, double mm_per_min)
{
    double s = mm_per_min / 60;
    double flow = speed_to_flow_rate(mm_per_min, l->h);

    fprintf(o, "%.0f mm/sec", s);
    fprintf(o, " = %.2f filament mm/sec", filament_mm3_to_length(flow));
    fprintf(o, " (flow %.1f mm^3/sec)", flow);
}

static void
generate_transition(layer_t *l, transition_t *t, extrusion_state_t *e)
{
    double original_e;
    double actual_e;
    xy_t start_xy;
    double start_total_e;

    original_e = last_e;
    transition_e = transition_starting_e = last_e;

    start_total_e = e->total_e + t->mm_from_runs;

    fprintf(o, "; Transition: %d->%d with %f || %f mm %f mm since splice || total_e=%f acc_trans=%f acc_waste=%f || %f\n", t->from, t->to, t->pre_mm, t->post_mm, n_splices > 0 ? e->total_e - splices[n_splices-1].mm : e->total_e, e->total_e, e->acc_transition, e->acc_waste, t->mm_pre_transition);
    fprintf(o, "; Speed: ");
    report_speed(o, l, extrusion_speed(l->h));
    if (l->transition0 == t->num && l->use_perimeter) {
	fprintf(o, ", perimeter: ");
	report_speed(o, l, extrusion_speed(l->h) * printer->perimeter_speed_multiplier);
    }
    fprintf(o, "\n");

    if (last_fan > 0) fprintf(o, "M107\n");

    if (t->from != t->to) {
	e->total_e += t->mm_from_runs;
	add_splice(t->from, e->total_e, t->pre_mm, e);
    }

    // assume retraction was done just before tool change: do_retraction();
    move_to(NAN, NAN, l->z + z_hop);

    if (l->transition0 == t->num) {
	transition_pct = 0;
	layer_transition_e = 0;
    }

    pct_to_xy(l, 0, transition_pct, &start_xy);
    move_to(start_xy.x, start_xy.y, NAN);
    if (z_hop) move_to(NAN, NAN, l->z);

    if (t->ping) {
	ping_complete_e = transition_starting_e + 20;

	pings[n_pings].mm = start_total_e;
	n_pings++;

	fprintf(o, "; Starting initial ping pause at %f complete at %f (from %f)\n", pings[n_pings-1].mm, ping_complete_e, transition_starting_e);
	if (stop_at_ping == n_pings) {
		fclose(o);
		exit(0);
	}
	if (printer->ping_off_tower) move_off_tower(start_xy.x, start_xy.y);
	generate_pause(13000);
	if (printer->ping_off_tower) move_to(start_xy.x, start_xy.y, NAN);
    } else {
	ping_complete_e = 0;
    }

    undo_retraction();

    if (l->transition0 == t->num && l->use_perimeter) draw_perimeter(l, t);
    transition_fill(l, t);

    do_retraction();

    actual_e = transition_e - transition_starting_e;

    if (z_hop) move_to(NAN, NAN, l->z+z_hop);

    if (t->next_move_no_extrusion) {
	e->next_move_full = 1;
    } else {
	move_to(last_x, last_y, NAN);
	if (z_hop) move_to(NAN, NAN, last_z);
    }

    // assume unretraction will done just immediately after tool change: undo_retraction();

    if (e_is_absolute) fprintf(o, "G92 E%f\n", original_e);
    if (last_fan > 0) fprintf(o, "M106 S%f\n", last_fan);
    fprintf(o, "; Done transition: %d->%d actually used %f mm for %f + %f mm\n", t->from, t->to, actual_e, t->pre_mm, t->post_mm);

    e->acc_transition  += t->infill_mm + t->pre_mm + t->post_mm + t->support_mm;
    e->acc_transition  += actual_e - (t->pre_mm + t->post_mm);
    e->acc_waste       += actual_e;
    e->total_e         += actual_e;
    layer_transition_e += actual_e;

    assert(t->num != l->transition0 + l->n_transitions - 1 || fabs(transition_pct - 1) < 0.001);
}

static void
produce_prime(extrusion_state_t *e)
{
    double start_x = last_x, start_y = last_y, start_z = last_z, start_e = last_e;
    int i;

    if (prime_info.len > 0) {
	fprintf(o, "; Priming %d line%s of length %f starting at %f,%f,%f\n", prime_info.n, prime_info.n > 1 ? "s" : "", prime_info.len, prime_info.x, prime_info.y, layers[0].h);
	move_to(prime_info.x, prime_info.y, layers[0].h);
	for (i = 1; i <= prime_info.n; i++) {
	    if (i > 1) move_to(NAN, prime_info.y + (i-1)*printer->nozzle, NAN);
	    move_to_and_extrude(prime_info.x + (i % 2 == 1 ? prime_info.len : 0), NAN, NAN, last_e + prime_info.e * i, layers[0].h);
	}
	if (retract_mm > 0) {
	    fprintf(o, "G1 E%f F%f\n", e_is_absolute ? last_e + prime_info.e * prime_info.n - retract_mm : -retract_mm, retract_mm_per_min);
	}
	if (e_is_absolute) fprintf(o, "G92 E%f\n", start_e);
	fprintf(o, "; Priming complete\n");
	e->next_move_full = 1;
	last_e = start_e;
    }

    last_x = start_x;
    last_y = start_y;
    last_z = start_z;
}

static void
produce_gcode()
{
    int l = 0;
    int t = 0;
    extrusion_state_t e = { 0, };
    double squash_e = NAN;

    last_e = last_x = last_y = last_z = 0;

    rewind_input();
    while (1) {
	token_t token = get_next_token();

	if (t < n_transitions && token.pos >= transitions[t].offset) {
	    generate_transition(&layers[l], &transitions[t], &e);
	    t++;
	    if (layers[l].transition0 + layers[l].n_transitions == t) {
		l++;
		is_first_layer = 0;
	    }
	    assert(l >= n_layers || (layers[l].transition0 <= t && t < layers[l].transition0 + layers[l].n_transitions));
	}

	switch(token.t) {
	case MOVE:
	    //assert(t == 0 || l >= n_layers || token.x.move.e == last_e || token.x.move.z == layers[l].z);
	    update_last_state(&token);
	    if (cur_path == INTERFACE && squash_interface) {
		e.next_move_full = 1;
		fprintf(o, "; SI: %s", buf);
		squash_e = token.x.move.e;
	    } else {
		if (isfinite(squash_e)) {
		    fprintf(o, "; Squash Interface complete\n");
		    if (e_is_absolute) fprintf(o, "G92 E%f\n", squash_e);
		    squash_e = NAN;
		}
		if (e.next_move_full && token.x.move.changes_position) {
		    e.next_move_full = 0;
		    fprintf(o, "G1 X%f Y%f Z%f E%f F%f\n", token.x.move.x, token.x.move.y, token.x.move.z, token.x.move.e - (e_is_absolute ? 0 : last_e), token.x.move.f);
		} else {
		    fprintf(o, "%s", buf);
		}
	    }
	    break;
	case FAN:
	    last_fan = token.x.fan;
	    fprintf(o, "%s", buf);
	    break;
	case TOOL:
	    fprintf(o, "; Switching to tool %d\n", token.x.tool);
	    if (debug_tool_changes) fprintf(o, "T%d\n", token.x.tool);
	    break;
	case DONE:
	    e.total_e += transition_final_mm + transition_final_waste;
	    add_splice(tool, e.total_e, 0, &e);
	    splices[n_splices-1].waste += transition_final_waste;
	    return;
	case KISS_EXT: {
	    double mm = total_ext[token.x.tool];
	    if (token.x.tool == tool) mm += transition_final_mm + transition_final_waste;
	    if (transition_block.area == 0) mm += printer->prime_mm;
	    fprintf(o, ";    Ext %d = %8.2f mm  (%.03f cm^3)\n", token.x.tool+1, mm, filament_length_to_mm3(mm)/10/10/10);
	    break;
	}
	case SET_E:
	    last_e = token.x.e;
	    fprintf(o, "%s", buf);
	    break;
	case START:
	    produce_prime(&e);
	    break;
	default:
	    fprintf(o, "%s", buf);
	    break;
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
