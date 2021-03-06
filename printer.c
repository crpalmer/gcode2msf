#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <math.h>
#include "printer.h"
#include "yaml-wrapper.h"

static struct {
    const char *key;
    size_t offset;
    enum { BOOLEAN, DOUBLE, INT, STRING } type;
    int max_depth;
} keys[] = {
    { "name", offsetof(printer_t, name), STRING, -1 },
    { "circular", offsetof(printer_t, circular), BOOLEAN, -1 },
    { "diameter", offsetof(printer_t, diameter), DOUBLE, -1 },
    { "x", offsetof(printer_t, bed_x), DOUBLE, 2 },
    { "y", offsetof(printer_t, bed_y), DOUBLE, 2 },
    { "bowdenTube", offsetof(printer_t, bowden_len), DOUBLE, -1 },
    { "filamentDiameter", offsetof(printer_t, filament), DOUBLE, -1 },
    { "nozzleDiameter", offsetof(printer_t, nozzle), DOUBLE, -1 },
    { "max_layer_height", offsetof(printer_t, max_layer_height), DOUBLE, -1 },
    { "purgeLength", offsetof(printer_t, transition_len), DOUBLE, -1 },
    { "minPurgeLength", offsetof(printer_t, min_transition_len), DOUBLE, -1 },
    { "initialPurgeLength", offsetof(printer_t, early_transition_len), DOUBLE, -1 },
    { "purgeTarget", offsetof(printer_t, transition_target), DOUBLE, -1 },
    { "transitionInInfill", offsetof(printer_t, transition_in_infill), BOOLEAN, -1} ,
    { "transitionInSupport", offsetof(printer_t, transition_in_support), BOOLEAN, -1} ,
    { "printSpeed", offsetof(printer_t, print_speed_mm_per_min), DOUBLE, -1 },
    { "perimeterSpeedMultiplier", offsetof(printer_t, perimeter_speed_multiplier), DOUBLE, -1},
    { "minDensity", offsetof(printer_t, min_density), DOUBLE, -1 },
    { "perimeterSpeedMultiplier", offsetof(printer_t, perimeter_speed), DOUBLE, -1 },
    { "loadingOffset", offsetof(printer_t, loading_offset), INT, -1 },
    { "printValue", offsetof(printer_t, pv), DOUBLE, -1 },
    { "calibrationGCodeLength", offsetof(printer_t, calibration_len), DOUBLE, -1 },
    { "minBottomDensity", offsetof(printer_t, min_bottom_density), DOUBLE, -1 },
    { "pingOffTower", offsetof(printer_t, ping_off_tower), BOOLEAN, -1 },
    { "prime_mm", offsetof(printer_t, prime_mm), DOUBLE, -1 },
    { "pings_ignore_retraction", offsetof(printer_t, pings_ignore_retraction), BOOLEAN, -1 },
    { "ping_stabilize_mm", offsetof(printer_t, ping_stabilize_mm), DOUBLE, -1 },
};

#define N_KEYS (sizeof(keys) / sizeof(keys[0]))

printer_t *printer;

int
printer_load(const char *fname)
{
    yaml_wrapper_t *p;
    yaml_event_t event, event2;
    int ki;
    int depth = 0;

    if ((p = yaml_wrapper_new(fname)) == NULL) return 0;

    printer = calloc(sizeof(*printer), 1);
    printer->ping_stabilize_mm = 5000;

    for (;;) {
	if (! yaml_wrapper_event(p, &event)) break;

process_event:
	if (event.type == YAML_MAPPING_START_EVENT) depth++;
	else if (event.type == YAML_MAPPING_END_EVENT) depth--;
	else if (event.type == YAML_SCALAR_EVENT) {
	    const char *key = (char *) event.data.scalar.value;
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

	    value = (char *) event2.data.scalar.value;

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

    if (printer->max_layer_height <= 0) printer->max_layer_height = printer->nozzle * 0.8;
    printer->print_speed_mm_per_min *= 60;

    return 1;
}


static double
filament_cross_section_area()
{
    double radius = printer->filament/2;

    return M_PI*radius*radius;
}

double
filament_length_to_mm3(double len)
{
    return filament_cross_section_area() * len;
}

double
filament_mm3_to_length(double mm3)
{
    return mm3 / filament_cross_section_area();
}

double
speed_to_flow_rate(double mm_per_min, double layer_height)
{
    return mm_per_min / 60 * printer->nozzle * layer_height;
}

double
flow_rate_to_speed(double mm3_per_sec, double layer_height)
{
    return mm3_per_sec * 60 / printer->nozzle / layer_height;
}
