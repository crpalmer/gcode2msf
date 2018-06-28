#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include "printer.h"
#include "yaml-wrapper.h"

static struct {
    const char *key;
    int offset;
    enum { BOOLEAN, DOUBLE, INT, STRING } type;
    int max_depth;
} keys[] = {
    { "name", offsetof(printer_t, name), STRING, -1 },
    { "circular", offsetof(printer_t, circular), BOOLEAN, -1 },
    { "diameter", offsetof(printer_t, diameter), DOUBLE, -1 },
    { "x", offsetof(printer_t, bed_x), DOUBLE, 2 },
    { "y", offsetof(printer_t, bed_y), DOUBLE, 2 },
    { "filamentDiameter", offsetof(printer_t, filament), DOUBLE, -1 },
    { "nozzleDiameter", offsetof(printer_t, nozzle), DOUBLE, -1 },
    { "purgeLength", offsetof(printer_t, transition_len), DOUBLE, -1 },
    { "minPurgeLength", offsetof(printer_t, min_transition_len), DOUBLE, -1 },
    { "initialPurgeLength", offsetof(printer_t, early_transition_len), DOUBLE, -1 },
    { "purgeTarget", offsetof(printer_t, transition_target), DOUBLE, -1 },
    { "printSpeed", offsetof(printer_t, print_speed), DOUBLE, -1 },
    { "minDensity", offsetof(printer_t, min_density), DOUBLE, -1 },
    { "perimeterSpeedMultiplier", offsetof(printer_t, perimeter_speed), DOUBLE, -1 },
    { "loadingOffset", offsetof(printer_t, loading_offset), INT, -1 },
    { "printValue", offsetof(printer_t, pv), DOUBLE, -1 },
    { "calibrationGCodeLength", offsetof(printer_t, calibration_len), DOUBLE, -1 },
};

#define N_KEYS (sizeof(keys) / sizeof(keys[0]))

printer_t *
printer_load(const char *fname)
{
    printer_t *printer;
    yaml_wrapper_t *p;
    yaml_event_t event, event2;
    int ki;
    int depth = 0;

    if ((p = yaml_wrapper_new(fname)) == NULL) return 0;

    printer = calloc(sizeof(*printer), 1);

    for (;;) {
	if (! yaml_wrapper_event(p, &event)) break;

process_event:
	if (event.type == YAML_MAPPING_START_EVENT) depth++;
	else if (event.type == YAML_MAPPING_END_EVENT) depth--;
	else if (event.type == YAML_SCALAR_EVENT) {
	    const char *key = event.data.scalar.value;
	    const char *value;

	    if (! yaml_wrapper_event(p, &event2)) {
		yaml_event_delete(&event);
		break;
	    }

	    if (event2.type != YAML_SCALAR_EVENT) {
		yaml_event_delete(&event);
		event = event2;
		goto process_event;
	    }

	    value = event2.data.scalar.value;

	    for (ki = 0; ki < N_KEYS; ki++) {
		if (strcmp(key, keys[ki].key) == 0) {
		    void *p = ((unsigned char *) printer) + keys[ki].offset;
		    char **s = (char **) p;
		    double *d = (double *) p;
		    int *i = (int *) p;

		    if (keys[ki].max_depth  < 0 || depth <= keys[ki].max_depth) {
			switch(keys[ki].type) {
			case BOOLEAN:
			    *i = strcmp(value, "true") == 0;
			    break;
			case DOUBLE:
			    *d = atof(value);
			    break;
			case INT:
			    *i = atoi(value);
			    break;
			case STRING:
			    if (*s) free(*s);
			    *s = strdup(value);
			    break;
			}
		    }
		}
	    }

	    yaml_event_delete(&event2);
	}
	yaml_event_delete(&event);
    }

    return printer;
}
