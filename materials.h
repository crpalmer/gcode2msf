#ifndef __MATERIALS_H__
#define __MATERIALS_H__

typedef struct {
    int id;
    const char *name;
    const char *type;
} material_t;

typedef struct {
    int incoming, outgoing;
    double heat;
    double compression;
    int reverse;
} material_splice_t;

typedef enum {
   WEAK, MEDIUM, STRONG, UNKNOWN
} colour_strength_t;

typedef struct {
    material_t *m;
    char *colour;
    colour_strength_t strength;
} active_material_t;

int
materials_load(const char *fname);

material_t *const
materials_find(const char *name);

material_splice_t *const
materials_find_splice(int incoming, int outgoing);

active_material_t *
get_active_material(int drive);

void
set_active_material(int drive, const char *name, const char *colour, colour_strength_t strength);

#endif


