#ifndef __BED_USAGE_H__
#define __BED_USAGE_H__

typedef struct bed_usageS bed_usage_t;

bed_usage_t *bed_usage_new(void);

void bed_usage_new_layer(bed_usage_t *, double z);

void bed_usage_extrude(bed_usage_t *, double x0, double y0, double x1, double y1);

int bed_usage_place_object(bed_usage_t *b, double w, double h, double to_z, double *x_res, double *y_res);

void bed_usage_add_object(bed_usage_t *b, double x, double y, double w, double h, char usage);

int bed_usage_place_and_add_object(bed_usage_t *b, double w, double h, double to_z, char usage, double *x_res, double *y_res);

void bed_usage_print(bed_usage_t *, FILE *);

void bed_usage_destroy(bed_usage_t *);

#endif
