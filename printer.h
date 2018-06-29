#ifndef __PRINTER_H__
#define __PRINTER_H__

typedef struct {
    char *name;
    int    circular;
    double diameter;
    double bed_x, bed_y;
    double bowden_len;
    double filament;
    double nozzle;
    double transition_len;
    double min_transition_len;
    double early_transition_len;
    double transition_target;
    double print_speed;
    double min_density;
    double perimeter_speed;
    unsigned loading_offset;
    double pv;
    double calibration_len;
} printer_t;

printer_t *
printer_load(const char *fname);

#endif
