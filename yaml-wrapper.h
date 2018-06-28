#ifndef __YAML_WRAPPER_H__
#define __YAML_WRAPPER_H__

#include <yaml.h>

typedef struct yaml_wrapperS yaml_wrapper_t;

yaml_wrapper_t *
yaml_wrapper_new(const char *fname);

int
yaml_wrapper_event(yaml_wrapper_t *p, yaml_event_t *event);

int
yaml_wrapper_had_error(yaml_wrapper_t *p);

void
yaml_wrapper_delete(yaml_wrapper_t *p);

#endif
