#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <malloc.h>
#include "printer.h"
#include "bed-usage.h"

/* This code is based on running a simple line plotting algorithm (Bresenham's line algorithm).
 *
 * Treat the bed as a grid of fixed sized cells and try to mark the cells in which extrusion
 * takes place.
 *
 * Once we've mapped all the extrusions we can place objects on the build area (e.g. transition
 * towers and purge lines/blocks).
 *
 * There are two inaccuracies resulting from using a simple approach:
 *  + The algorithm will can miss cells.
 *  + Extrusion width is not accounted for
 *
 * Rather than trying to deal with that added complexity, we'll simply require one full cell
 * be empty around the area in which we place an object.  That should buffer the inaccuracies.
 */

#define CELL_SIZE	5
#define USED		'*'

typedef struct {
    double	z;
    int		n_used;
    unsigned char *used;
} layer_t;

struct bed_usageS {
    int n_layers;
    int a_layers;
    layer_t *l;
    layer_t *cur;
    int w, h;
};

#define CELL(b, array, x, y) ((array)[(y) * (b)->w + (x)])

bed_usage_t *bed_usage_new(void)
{
    bed_usage_t *b;

    b = malloc(sizeof(*b));

    b->n_layers = 0;
    b->a_layers = 16;
    b->l = malloc(sizeof(*b->l) * b->a_layers);

    if (printer->circular) {
	b->w = b->h = ceil(printer->diameter / CELL_SIZE);
    } else {
	b->w = ceil(printer->bed_x / CELL_SIZE);
	b->h = ceil(printer->bed_y / CELL_SIZE);
    }

    return b;

}

void bed_usage_new_layer(bed_usage_t *b, double z)
{
    if (b->n_layers && b->cur->n_used == 0) return;

    if (b->n_layers >= b->a_layers) {
	b->a_layers *= 2;
	b->l = realloc(b->l, sizeof(*b->l) * b->a_layers);
    }

    b->cur = &b->l[b->n_layers++];
    b->cur->z = z;
    b->cur->n_used = 0;
    b->cur->used = calloc(sizeof(*b->cur->used), b->w * b->h);
}

static void mark(bed_usage_t *b, double x0, double y0)
{
    int x = x0, y = y0;

    if (printer->circular) {
	x += printer->diameter / 2.0 / CELL_SIZE;
	y += printer->diameter / 2.0 / CELL_SIZE;
    }

    if (! CELL(b, b->cur->used, x, y)) {
         b->cur->n_used++;
         CELL(b, b->cur->used, x, y) = USED;
    }
}

void bed_usage_extrude(bed_usage_t *b, double x0, double y0, double x1, double y1)
{
     double delta_x;
     double delta_y;
     double delta_err;
     int sign_x, sign_y;
     int x, y, x_end, y_end;

     x0 /= CELL_SIZE;
     x1 /= CELL_SIZE;
     y0 /= CELL_SIZE;
     y1 /= CELL_SIZE;

     delta_x = x1 - x0;
     delta_y = y1 - y0;
     delta_err = abs(delta_y / delta_x);

     sign_x = delta_x > 0 ? 1 : -1;
     sign_y = delta_y > 0 ? 1 : -1;

     x = (int) x0;
     y = (int) y0;
     x_end = (int) x1;
     y_end = (int) y1;

     if (x == x_end) { /* vertical line */
	while (y != y_end + sign_y) {
	    mark(b, x0, y);
	    y += sign_y;
	}
    } else {
        double error = 0;

	while (x != x_end + sign_x) {
	    mark(b, x, y);
	    error += delta_err;
	    if (error >= 0.5) {
		y += sign_y;
		//if (error >= 0.5) mark(b, x, y);
		error -= 1;
	    }
	    x += sign_x;
	}
    }
}

static inline int xy_to_bed_xy(double xy)
{
    if (printer->circular) return (xy + printer->diameter/2.0) / CELL_SIZE;
    else return xy / CELL_SIZE;
}

static inline double bed_xy_to_xy(int xy)
{
    if (printer->circular) return (xy * CELL_SIZE) - printer->diameter/2.0;
    else return xy * CELL_SIZE;
}

static void
print_bed(bed_usage_t *b, unsigned char *used, FILE *f)
{
    int x, y;

    for (y = b->h-1; y >= 0; y--) {
	for (x = 0; x < b->w; x++) {
	    char sym = CELL(b, used, x, y) ? CELL(b, used, x, y) : ' ';
	    if (printer->circular) {
		double dx = bed_xy_to_xy(x);
		double dy = bed_xy_to_xy(y);
		if (sqrt(dx*dx + dy*dy) > printer->diameter/2) sym = '.';
	    }
	    fprintf(f, "%c", sym);
	}
        fprintf(f, "\n");
    }
}

static unsigned char *
get_usage_to_z(bed_usage_t *b, double z)
{
    unsigned char *used, *expanded;
    int i, j, k;

    used = calloc(sizeof(*used), b->w * b->h);
    expanded = calloc(sizeof(*expanded), b->w * b->h);

    for (i = 0; i < b->n_layers; i++) {
	if (i == 0 || b->l[i].z <= z) {
	    for (j = 0; j < b->w * b->h; j++) used[j] |= b->l[i].used[j];
	}
    }

    for (i = 0; i < b->w; i++) {
	for (j = 0; j < b->h; j++) {
	    for (k = 0; k < 9; k++) {
		int x = i + k % 3 - 1;
		int y = j + k / 3 - 1;
		if (x >= 0 && y >= 0 && x < b->w && y < b->h) CELL(b, expanded, i, j) |= CELL(b, used, x, y);
	    }
	}
    }

    free(used);
    return expanded;
}

static int
is_valid(int x, int y)
{
    double fixed_x = bed_xy_to_xy(x);
    double fixed_y = bed_xy_to_xy(y);

    return printer_is_valid(fixed_x, fixed_y);
}

int bed_usage_place_object(bed_usage_t *b, double w0, double h0, double to_z, double *x_res, double *y_res)
{
    unsigned char *l = get_usage_to_z(b, to_z);
    int w, h;
    int x, y, dx, dy;
    double best_d = NAN;

    w = ceil(w0 / CELL_SIZE);
    h = ceil(h0 / CELL_SIZE);

    for (x = 0; x < b->w - w; x++) {
	for (y = 0; y < b->h - h; y++) {
	    double this_d;

	    for (dx = 0; dx < w; dx++) {
		for (dy = 0; dy < h; dy++) {
		    if (! is_valid(x + dx, y+ dy) || CELL(b, l, x + dx, y + dy)) goto next_location;
		}
	    }
	    this_d = sqrt((x + w/2 - b->w/2)*(x + w/2 - b->w/2) + (y + h/2 - b->h/2)*(y + h/2 - b->h/2));
	    if (! isfinite(best_d) || this_d < best_d) {
		best_d = this_d;
		*x_res = bed_xy_to_xy(x) + (w*CELL_SIZE - w0) / 2;
		*y_res = bed_xy_to_xy(y) + (h*CELL_SIZE - h0) / 2;
	    }
next_location:
	    ;
	}
    }

    free(l);

    return isfinite(best_d);
}

void bed_usage_add_object(bed_usage_t *b, double x0, double y0, double w0, double h0, char usage)
{
    int x = xy_to_bed_xy(x0);
    int y = xy_to_bed_xy(y0);
    int w = w0 / CELL_SIZE;
    int h = h0 / CELL_SIZE;
    int dx, dy;
    
    for (dx = 0; dx < w; dx++) {
	for (dy = 0; dy < h; dy++) {
	    CELL(b, b->l[0].used, x + dx, y + dy) = usage;
	}
    }

}

int bed_usage_place_and_add_object(bed_usage_t *b, double w, double h, double to_z, char usage, double *x, double *y)
{
    if (! bed_usage_place_object(b, w, h, to_z, x, y)) return 0;
    bed_usage_add_object(b, *x, *y, w, h, usage);
    return 1;
}

void bed_usage_print(bed_usage_t *b, FILE *f)
{
    print_bed(b, b->l[0].used, f);
}

void bed_usage_destroy(bed_usage_t *b)
{
    free(b->l);
    free(b);
}

