#include <stdio.h>
#include <malloc.h>
#include "yaml-wrapper.h"

struct yaml_wrapperS {
    yaml_parser_t parser;
    FILE *f;
    int had_error;
    int had_eof;
};

yaml_wrapper_t *
yaml_wrapper_new(const char *fname)
{
    FILE *f;
    yaml_parser_t parser;
    yaml_wrapper_t *p;

    if ((f = fopen(fname, "r")) == NULL) return NULL;
    if (! yaml_parser_initialize(&parser)) {
        fclose(f);
        return NULL;
    }

    p = malloc(sizeof(*p));
    p->parser = parser;
    p->f = f;
    p->had_error = 0;
    p->had_eof = 0;

    yaml_parser_set_input_file(&p->parser, p->f);

    return p;
}

int
yaml_wrapper_event(yaml_wrapper_t *p, yaml_event_t *event)
{
    if (p->had_error || p->had_eof) return 0;

    if (! yaml_parser_parse(&p->parser, event)) {
	p->had_error = 1;
	fprintf(stderr, "YAML parser error: %d\n", p->parser.error);
	return 0;
    }

    if (event->type == YAML_STREAM_END_EVENT) {
	p->had_eof = 1;
	return 0;
    }

    return 1;
}

int
yaml_wrapper_had_error(yaml_wrapper_t *p)
{
    return p->had_error;
}

void
yaml_wrapper_delete(yaml_wrapper_t *p)
{
    yaml_parser_delete(&p->parser);
    fclose(p->f);
    free(p);
}
