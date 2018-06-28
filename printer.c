#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include "printer.h"
#include "yaml-wrapper.h"

static struct {
    const char *key;
    int offset;
    enum { BOOLEAN, DOUBLE, INT, STRING } type;

} keys[] = {
    { "name", offsetof(printer_t, name), STRING },
    { "circular", offsetof(printer_t, circular), BOOLEAN },
    { "diameter", offsetof(printer_t, diameter), DOUBLE },
    { "x", offsetof(printer_t, bed_x), DOUBLE },
    { "y", offsetof(printer_t, bed_y), DOUBLE },
    { "filamentDiameter", offsetof(printer_t, filament), DOUBLE },
    { "nozzleDiameter", offsetof(printer_t, nozzle), DOUBLE },
    { "purgeLength", offsetof(printer_t, transition_len), DOUBLE },
    { "minPurgeLength", offsetof(printer_t, min_transition_len), DOUBLE },
    { "initialPurgeLength", offsetof(printer_t, early_transition_len), DOUBLE },
    { "purgeTarget", offsetof(printer_t, transition_target), DOUBLE },
    { "printSpeed", offsetof(printer_t, print_speed), DOUBLE },
    { "minDensity", offsetof(printer_t, min_density), DOUBLE },
    { "perimeterSpeedMultiplier", offsetof(printer_t, perimeter_speed), DOUBLE },
    { "loadingOffset", offsetof(printer_t, loading_offset), INT },
    { "printValue", offsetof(printer_t, pv), DOUBLE },
    { "calibrationGCodeLength", offsetof(printer_t, calibration_len), DOUBLE },
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
	else if (event.type == YAML_SCALAR_EVENT && depth < 3) {
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

	    yaml_event_delete(&event2);
	}
	yaml_event_delete(&event);
    }

    return printer;
}
