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
    int    transition_in_infill;
    int    transition_in_support;
    double print_speed;
    double perimeter_speed_multiplier;
    double min_density;
    double perimeter_speed;
    unsigned loading_offset;
    double pv;
    double calibration_len;
    double min_bottom_density;
    int ping_off_tower;
} printer_t;

extern printer_t *printer;

int
printer_load(const char *fname);

double filament_length_to_mm3(double len);
double filament_mm3_to_length(double len);
double speed_to_flow_rate(double mm_per_min, double layer_height);
double flow_rate_to_speed(double mm3_per_sec, double layer_height);

#endif
