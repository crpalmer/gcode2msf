#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

#define MAX_RUNS	100000
#define N_DRIVES	4

typedef struct {
    double x[2];
    double y[2];
} bb_t;

typedef struct {
    enum {
	MOVE,
	START_TOWER,
	END_TOWER,
	SET_E,
	TOOL,
	PING,
	START,
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

typedef struct {
    bb_t   bb;
    double z;
    double e;
    int	   t;
    int    pre_transition, post_transition;
} run_t;

typedef struct {
    bb_t   bb;
    double z;
    double h;
    int	transition0;
    int n_transitions;
    double mm;
} layer_t;

typedef struct {
    int from, to;
    double mm;
} transition_t;

static FILE *f;

static char buf[1024*1024];

static int validate_only = 0;
static int summary = 0;
static int extrusions = 0;
static int trace = 0;

static double last_x = 0, last_y = 0, last_z = 0, start_e = 0, last_e = 0, last_e_z = 0, acc_e = 0, last_reported_z = -1;
static bb_t bb;
static int tool = 0;
static int seen_tool = 0;
static int used_tool[N_DRIVES] = { 0, };
static double tower_z = -1;
static int in_tower = 0;
static int seen_ping = 0;
static int started = 0;

static run_t runs[MAX_RUNS];
static int n_runs = 0;

static layer_t layers[MAX_RUNS];
static int n_layers;
static transition_t transitions[MAX_RUNS];
static int n_transitions = 0;
static bb_t model_bb;

struct {
    int  id;
    const char *name;
} materials[N_DRIVES] = {
    { 1, "Orange PLA" },
    { -1, NULL },
    { -1, NULL },
    { 2, "White Soluble" }
};

#define N_SPLICING (N_DRIVES*N_DRIVES)
struct {
   int from, to;
   double heat, compressing;
   int reverse;
} splicing[N_SPLICING] = {
   { 1, 1, 9, 2, 0 },
   { 1, 2, 6, 4, 0 },
   { 2, 1, 6, 4, 1 },
};

typedef struct {
    const char *name;
    unsigned lv;
    double ppm;
    double nozzle;
    double filament;
    double radius;
    double size[2];
} printer_t;

static printer_t printers[] = {
    { "TLM", 25821, 30.099659, 0.4, 1.75, 150, { NAN, NAN} },
    { "Taz6", 25821, 30.099659, 0.4, 1.75, NAN, { 280, 280} }
};

static printer_t *printer = &printers[1];

struct {
    double target_pct;
    double min_density;
    double transition_mm;
} t_config = { 0.2, 0.05, 100 };

#define BB_PRINTF_ARGS(bb) (bb)->x[0], (bb)->y[0], (bb)->x[1], (bb)->y[1]

static void
bb_init(bb_t *bb)
{
    bb->x[0] = bb->y[0] = DBL_MAX;
    bb->x[1] = bb->y[1] = -DBL_MAX;
}

static void
bb_add_point(bb_t *bb, double x, double y)
{
    if (x < bb->x[0]) bb->x[0] = x;
    if (x > bb->x[1]) bb->x[1] = x;
    if (y < bb->y[0]) bb->y[0] = y;
    if (y > bb->y[1]) bb->y[1] = y;
}

static void
bb_add_bb(bb_t *dest, const bb_t *other)
{
    bb_add_point(dest, other->x[0], other->y[0]);
    bb_add_point(dest, other->x[1], other->y[1]);
}

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

#define STRNCMP(a, b) strncmp(a, b, strlen(b))

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
    }

    t.t = DONE;
    return t;
}

static token_t
get_next_token()
{
    token_t t = get_next_token_wrapped();
    if (trace) {
	printf("%8ld ", t.pos);
	switch (t.t) {
	case MOVE: printf("MOVE (%f,%f,%f) e=%f\n", t.x.move.x, t.x.move.y, t.x.move.z, t.x.move.e); break;
	case START_TOWER: printf("START_TOWER\n"); break;
	case END_TOWER: printf("END_TOWER\n"); break;
	case PING: printf("PING %d.%d\n", t.x.ping.num, t.x.ping.step); break;
	case SET_E: printf("SET_E %f\n", t.x.e); break;
	case START: printf("START\n"); break;
	case TOOL: printf("TOOL %d\n", t.x.tool); break;
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
	used_tool[tool] = 1;

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
	    bb_add_point(&bb, t.x.move.x, t.x.move.y);

	    if (t.x.move.e != last_e && t.x.move.z != last_e_z) {
		accumulate();
		if (started) {
		    add_run();
		    show_extrusion('+', 0);
		}
		last_e_z = t.x.move.z;
		reset_state();
	    }
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
	}
    }
}

/* pre_transition is the amount of filament to consume for the transition prior to printing anything
 * post_transition is the amount of filament to consume for the transition after printing everything
 */

static transition_t *
get_pre_transition(int j)
{
    if (runs[j].pre_transition >= 0) return &transitions[runs[j].pre_transition];
    else return NULL;
}

static double
get_pre_transition_mm(int j)
{
    transition_t *t = get_pre_transition(j);
    return t ? t_config.target_pct * t->mm : 0;
}

static transition_t *
get_post_transition(int j)
{
    if (runs[j].post_transition >= 0) return &transitions[runs[j].post_transition];
    else return NULL;
}

static double
get_post_transition_mm(int j)
{
    transition_t *t = get_post_transition(j);
    return t ? (1-t_config.target_pct) * t->mm : 0;
}

static double
transition_block_layer_area(int layer)
{
    layer_t *l = &layers[layer];
    return M_PI*printer->filament/2.0*printer->filament/2.0*l->mm / l->h;
}

static double
transition_block_area()
{
    int i;
    double area = 0;

    for (i = 0; i < n_layers; i++) {
	double la = transition_block_layer_area(i);
	if (la > area) area = la;
    }
    return area;
}

static void
transition_block_size(double xy[2])
{
    /* Make y = 2*x (or visa versa) then you get
       area = x * 2*x = 2*x^2
       x = sqrt(area) / sqrt(2)
     */
    double area = transition_block_area();
    double x = sqrt(area) / sqrt(2);
    xy[0] = x;
    xy[1] = 2*x;
}

static void
output_summary()
{
    int i, j;
    double tool_mm[N_DRIVES] = { 0, };
    double tool_waste[N_DRIVES] = { 0, };

    printf("Layer by layer extrusions\n");
    printf("-------------------------\n");
    for (i = 0; i < n_runs; i = j) {
	printf("%6.02f", runs[i].z);
	for (j = i; j < n_runs && runs[i].z == runs[j].z; j++) {
	    printf(" %10.2f [%d]", runs[j].e, runs[j].t);
	}
	printf("\n");
    }
    printf("\n");
    printf("Extruder by extruder extrusions\n");
    printf("-------------------------------\n");
    for (i = 0; i < n_runs; i = j) {
	double mm = 0, waste = 0;
	for (j = i; j < n_runs && runs[i].t == runs[j].t; j++) {
	    mm += runs[j].e;
	    waste += get_pre_transition_mm(j) + get_post_transition_mm(j);
	}
	tool_mm[runs[i].t] += mm;
	tool_waste[runs[i].t] += waste;
	printf("T%d %10.4f mm %10.4f waste\n", runs[i].t, mm, waste);
    }
    for (i = 0; i < N_DRIVES; i++) {
	if (tool_mm[i] + tool_waste[i] != 0) printf("   TOTAL: T%d %10.2f mm %10.2f waste\n", i, tool_mm[i], tool_waste[i]);
    }
    printf("\n");
    double t_xy[2];
    transition_block_size(t_xy);
    printf("Transition block: area %f dimensions %fx%f model_bb=(%f,%f),(%f,%f)\n", transition_block_area(), t_xy[0], t_xy[1], BB_PRINTF_ARGS(&model_bb));
    printf("----------------\n");
    for (i = 0; i < n_layers; i++) {
	printf("z=%-6.2f height=%-4.2f mm=%-6.2f n-transitions=%d area=%f bb=(%f,%f),(%f,%f)\n", layers[i].z, layers[i].h, layers[i].mm, layers[i].n_transitions, transition_block_layer_area(i), BB_PRINTF_ARGS(&layers[i].bb));
    }
}

static double
transition_length(int from, int to)
{
    if (from == to) return 17.7;	// TODO: Can I make this smaller unless we are doing a ping?
    else return t_config.transition_mm;
}

static void
add_transition(int from, int to, double z, run_t *run, run_t *pre_run)
{
    layer_t *layer;

    if (n_layers == 0 || z > layers[n_layers-1].z) {
	if (n_layers > 0) {
	    layers[n_layers-1].h = z - layers[n_layers-1].z;
	}
	layer = &layers[n_layers];
	bb_init(&layer->bb);
	layer->z = z;
	layer->h = -1;
	layer->transition0 = n_transitions;
	layer->n_transitions = 0;
	layer->mm = 0;
	n_layers++;
    }

    pre_run->post_transition = n_transitions;
    run->pre_transition = n_transitions;
    transitions[n_transitions].from = from;
    transitions[n_transitions].to = to;
    transitions[n_transitions].mm = transition_length(from, to);
    n_transitions++;

    layer = &layers[n_layers-1];
    layer->n_transitions++;
    layer->mm += transitions[n_transitions-1].mm;
    bb_add_bb(&layer->bb, &run->bb);
}

static void
compute_transition_tower()
{
    int i;

    for (i = 1; i < n_runs; i++) {
	if (runs[i-1].t != runs[i].t) {
	    add_transition(runs[i-1].t, runs[i].t, runs[i].z, &runs[i], &runs[i-1]);
	} else if (runs[i-1].z != runs[i].z && (n_layers == 0 || layers[n_layers-1].z != runs[i-1].z)) {
	    add_transition(runs[i-1].t, runs[i-1].t, runs[i-1].z, &runs[i-1], &runs[i-1]);
	}
    }
}

static void
prune_transition_tower()
{
    int i;
    int last_transition;

    while (n_layers > 0) {
	transition_t *t = &transitions[layers[n_layers-1].transition0];

	if (layers[n_layers-1].n_transitions > 1 || t->from != t->to) break;
	n_layers--;
    }
    n_transitions = layers[n_layers-1].transition0 + layers[n_layers-1].n_transitions - 1;
    for (i = 0; i < n_runs; i++) {
	if (runs[i].pre_transition > n_transitions) runs[i].pre_transition = -1;
	if (runs[i].post_transition > n_transitions) runs[i].post_transition = -1;
    }
}

static void
find_model_bb_to_tower_height()
{
    int i;

    bb_init(&model_bb);

    for (i = 0; i < n_layers; i++) {
	bb_add_bb(&model_bb, &layers[i].bb);
    }
}

static char *
float_to_hex(double f, char *buf)
{
    unsigned v;
    if (f == 0) strcpy(buf, "0");
    else {
	int shift = 0;

	if (f < 0) v = 1 << 31;
	else v = 0;
	f = fabs(f);

	while (f >= 2) {
	   f /= 2;
	   shift++;
	}
	while (f < 1) {
	   f *= 2;
	   shift--;
	}
	f = f - 1;
	v |= (unsigned) (f * (0x800000 + 0.5));
	v |= (shift + 0x7f) << 23;

	sprintf(buf, "%x", v);
    }

    return buf;
}

static void
produce_msf_colours(FILE *o)
{
    int i;
    int first = 1;

    fprintf(o, "cu:");
    for (i = 0; i < N_DRIVES; i++) {
	if (used_tool[i]) {
	    if (! first) fprintf(o, ";");
	    first = 0;
	    fprintf(o, "%d%s", materials[i].id, materials[i].name);
	}
    }
    fprintf(o, "\n");
}

static void
count_or_produce_splices(FILE *o, int *n_out)
{
    int i, j, n = 0;
    char buf[20];
    double mm = 0;

    for (i = 0; i < n_runs; i = j) {
	for (j = i; j < n_runs && runs[i].t == runs[j].t; j++) {
	    mm += get_pre_transition_mm(j);
	    mm += runs[j].e;
	    mm += get_post_transition_mm(j);
	}
	if (o) fprintf(o, "(%02x,%s)\n", runs[i].t, float_to_hex(mm, buf));
	n++;
    }

    if (n_out) *n_out = n;
}

static void
produce_msf_splices(FILE *o)
{
    count_or_produce_splices(o, NULL);
}

static int
msf_n_splices()
{
    int n;

    count_or_produce_splices(NULL, &n);
    return n;
}

static void
produce_msf(const char *fname)
{
    FILE *o = stdout;
    char buf[20];

    fprintf(o, "MSF1.4\n");
    produce_msf_colours(o);
    fprintf(o, "ppm:%s\n", float_to_hex(printer->ppm, buf));
    fprintf(o, "lo:%s\n", float_to_hex(printer->lv, buf));
    fprintf(o, "ns:%x\n", msf_n_splices());
    fprintf(o, "np:0\n");
    fprintf(o, "nh:0\n");
    // TODO na:
    produce_msf_splices(o);
    // TODO pings
    // produce_msf_splice_configuration(o);
}

static void
produce_gcode(const char *gcode)
{
}

static void process(const char *fname)
{
    if ((f = fopen(fname, "r")) == NULL) {
	perror(fname);
	return;
    }

    preprocess();
    compute_transition_tower();
    prune_transition_tower();
    find_model_bb_to_tower_height();
    rewind(f);
    produce_msf(fname);
    rewind(f);
    produce_gcode(fname);
    fclose(f);
    if (summary) output_summary();
}

int main(int argc, char **argv)
{
    int i;

    while (argc > 1) {
	    if (strcmp(argv[1], "--validate") == 0) validate_only = 1;
	    else if (strcmp(argv[1], "--summary") == 0) summary = 1;
	    else if (strcmp(argv[1], "--trace") == 0) trace = 1;
	    else if (strcmp(argv[1], "--extrusions") == 0) extrusions = 1;
	    else break;
	    argc--;
	    argv++;
    }

    if (argc > 2) {
	fprintf(stderr, "May only process 1 file per run!\n");
	exit(0);
    }

    process(argv[1]);
}
