#ifndef __GCODE_H__
#define __GCODE_H__

#include "bed-usage.h"

#define MAX_RUNS        100000
#define N_DRIVES 4

typedef enum { NORMAL = 0, INFILL, SUPPORT, INTERFACE, UNKNOWN_PATH } path_t;

typedef struct {
    int    t;
    double z;
    double e;
    long   offset;
    path_t path;
    int    next_move_no_extrusion;
    int    ends_with_retraction;
    double trailing_infill_mm, leading_support_mm;

// temp
    int    pre_transition, post_transition;

} run_t;

typedef struct {
    int drive;
    double mm;
    double waste;
    double transition_mm;
} splice_t;

typedef struct {
    double mm;
} ping_t;

extern run_t runs[MAX_RUNS];
extern int n_runs;
extern int used_tool[N_DRIVES];
extern bed_usage_t *bed_usage;

extern splice_t splices[MAX_RUNS];
extern int n_splices;
extern ping_t pings[MAX_RUNS];
extern int n_pings;
extern double retract_mm;

extern int extrusions;
extern int gcode_trace;
extern int validate_only;
extern int debug_tool_changes;
extern int stop_at_ping;
extern int squash_interface;

void gcode_to_runs(const char *fname);
void gcode_to_msf_gcode(const char *output_fname);

#endif
