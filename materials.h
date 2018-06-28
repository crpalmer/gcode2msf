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

int
materials_load(const char *fname);

material_t *const
materials_find(const char *name);

material_splice_t *const
materials_find_splice(int incoming, int outgoing);

#endif


