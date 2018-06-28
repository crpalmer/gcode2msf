#include <stdio.h>
#include <ctype.h>
#include <malloc.h>
#include <string.h>
#include <yaml.h>
#include "gcode.h"
#include "materials.h"
#include "yaml-wrapper.h"

#define MAX_MATERIALS	1000
#define MAX_SPLICES	10000

static material_t materials[MAX_MATERIALS];
static int n_materials;
static material_splice_t material_splices[MAX_SPLICES];
static int n_material_splices;

static active_material_t active_materials[N_DRIVES];

static char buf[100*1024];

static struct {
    const char *colour;
    colour_strength_t strength;
} default_colour_strengths[] = {
    { "Black",		STRONG },
    { "Yellow",		WEAK },
    { "White",		WEAK },
    { "Transparent",	WEAK },
    { "Orange",		MEDIUM },
    { "Green",		STRONG },
};

#define N_DEFAULT_COLOUR_STRENGTHS (sizeof(default_colour_strengths) / sizeof(default_colour_strengths[0]))

static material_t *
find_or_create_material(const char *name)
{
     int i;

     for (i = 0; i < n_materials; i++) {
	if (strcmp(materials[i].name, name) == 0) return &materials[i];
     }

     materials[n_materials].id = n_materials;
     materials[n_materials].name = strdup(name);
     n_materials++;

     return &materials[n_materials-1];
}

static material_splice_t *
find_or_create_splice(material_t *incoming, material_t *outgoing)
{
     int i;

    for (i = 0; i < n_material_splices; i++) {
	if (material_splices[i].incoming == incoming->id && material_splices[i].outgoing == outgoing->id) return &material_splices[i];
    }

    material_splices[n_material_splices].incoming = incoming->id;
    material_splices[n_material_splices].outgoing = outgoing->id;
    n_material_splices++;

    return &material_splices[n_material_splices-1];
}

static void
process_combination(yaml_wrapper_t *p, material_splice_t *splice)
{
    yaml_event_t event, event2;

    while (yaml_wrapper_event(p, &event)) {
	if (event.type == YAML_MAPPING_END_EVENT) {
	    yaml_event_delete(&event);
	    break;
	}
	if (event.type == YAML_SCALAR_EVENT && yaml_wrapper_event(p, &event2)) {
	    if (event2.type == YAML_SCALAR_EVENT) {
		const char *key = event.data.scalar.value;
		const char *value = event2.data.scalar.value;

		if (strcmp(key, "heatFactor") == 0) splice->heat = atof(value);
		else if (strcmp(key, "compressionFactor") == 0) splice->compression = atof(value);
		else if (strcmp(key, "reverse") == 0) splice->reverse = atoi(value);
		else printf("unknown combination parameter: %s = %s\n", key, value);
	    }
	    yaml_event_delete(&event2);
	}
	yaml_event_delete(&event);
    }
}
static void
process_combinations(yaml_wrapper_t *p, material_t *incoming)
{
    material_t *outgoing;
    yaml_event_t event;

    while (yaml_wrapper_event(p, &event)) {
	if (event.type == YAML_MAPPING_END_EVENT) {
	    yaml_event_delete(&event);
	    break;
	}
	if (event.type == YAML_SCALAR_EVENT) {
	    outgoing = find_or_create_material(event.data.scalar.value);
	    process_combination(p, find_or_create_splice(incoming, outgoing));
	}
	yaml_event_delete(&event);
    }
}

static void
process_material(yaml_wrapper_t *p, material_t *m)
{
    yaml_event_t event, event2;

    for (;;) {
	if (! yaml_wrapper_event(p, &event)) break;
	if (event.type == YAML_SCALAR_EVENT) {
	    if (strcmp(event.data.scalar.value, "type") == 0) {
		if (! yaml_wrapper_event(p, &event2)) {
		    yaml_event_delete(&event);
		    break;
		}
		m->type = strdup(event2.data.scalar.value);
		yaml_event_delete(&event2);
	    } else if (strcmp(event.data.scalar.value, "combinations") == 0) {
		if (! yaml_wrapper_event(p, &event2)) {
		    yaml_event_delete(&event);
		    break;
		}
		if (event2.type == YAML_MAPPING_START_EVENT) process_combinations(p, m);
		yaml_event_delete(&event2);
	    }
	} else if (event.type == YAML_MAPPING_END_EVENT) {
	    yaml_event_delete(&event);
	    break;
	}
	yaml_event_delete(&event);
    }
}

static void
init_active_materials()
{
    int i;

    for (i = 0; i < N_DRIVES; i++) {
	if (active_materials[i].m == NULL) {
	    active_materials[i].m = find_or_create_material("Default PLA");
	    active_materials[i].colour = NULL;
	    active_materials[i].strength = MEDIUM;
	}
    }
}

int
materials_load(const char *fname)
{
    yaml_wrapper_t *p;
    yaml_event_t event;
    int had_error;

    if ((p = yaml_wrapper_new(fname)) == NULL) return 0;

    while (yaml_wrapper_event(p, &event)) {
	if (event.type == YAML_SCALAR_EVENT) {
	    material_t *m = find_or_create_material(event.data.scalar.value);
	    process_material(p, m);
	}
	yaml_event_delete(&event);
    }

    had_error = yaml_wrapper_had_error(p);

    yaml_wrapper_delete(p);

    init_active_materials();

    return ! had_error;
}

material_t *const
materials_find(const char *name)
{
    return find_or_create_material(name);
}

material_splice_t *const
materials_find_splice(int incoming, int outgoing)
{
    return find_or_create_splice(&materials[incoming], &materials[outgoing]);
}

void
set_active_material(int drive, const char *name, const char *colour, colour_strength_t strength)
{
    if (strength == UNKNOWN) {
	int i;

	for (i = 0; i < N_DEFAULT_COLOUR_STRENGTHS; i++) {
	    if (strcasecmp(colour, default_colour_strengths[i].colour) == 0) {
		strength = default_colour_strengths[i].strength;
		break;
	    }
	}
	if (strength == UNKNOWN) strength = MEDIUM;
    }

    active_materials[drive].m = find_or_create_material(name);
    if (active_materials[drive].colour) free(active_materials[drive].colour);
    active_materials[drive].colour = strdup(colour);
    active_materials[drive].strength = strength;
}

active_material_t *
get_active_material(int drive)
{
    return &active_materials[drive];
}
