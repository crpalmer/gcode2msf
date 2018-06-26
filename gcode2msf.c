#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_RUNS	100000

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
	    double z, e;
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
    double z;
    double e;
    int	   t;
} run_t;

static FILE *f;

static char buf[1024*1024];

static int validate_only = 0;
static int summary = 0;
static int extrusions = 0;
static int trace = 0;

static double last_z = 0, start_e = 0, last_e = 0, last_e_z = 0, acc_e = 0, last_reported_z = -1;
static int tool = 0;
static int seen_tool = 0;
static double tower_z = -1;
static int in_tower = 0;
static int seen_ping = 0;
static int started = 0;

static run_t runs[MAX_RUNS];
static int n_runs = 0;

struct {
    int  id;
    const char *name;
} materials[4] = {
    { 1, "Orange PLA" },
    { -1, NULL },
    { -1, NULL },
    { 1, "White PLA" }
};

struct {
    unsigned lv;
    double ppm;
} calibration = { 25821, 30.099659 };

struct {
    double nozzle;
} printer = { 0.4 };

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
	case MOVE: printf("MOVE z=%f, e=%f\n", t.x.move.z, t.x.move.e); break;
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
	runs[n_runs].z = last_e_z;
	runs[n_runs].e = acc_e;
	runs[n_runs].t = tool;
	n_runs++;
    }
}

static void
preprocess()
{
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
	    last_e = t.x.move.e;
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

static void
output_summary()
{
    int i, j;
    double tool_total[10] = { 0, };

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
	double total = 0;
	for (j = i; j < n_runs && runs[i].t == runs[j].t; j++) {
	    total += runs[j].e;
	}
	tool_total[runs[i].t] += total;
	printf("T%d %10.4f mm\n", runs[i].t, total);
    }
    for (i = 0; i < 10; i++) {
	if (tool_total[i] != 0) printf("   TOTAL: T%d %10.2f mm\n", i, tool_total[i]);
    }
}

static void
compute_transition_tower()
{
}

static void
produce_msf(const char *fname)
{
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
