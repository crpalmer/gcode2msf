#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define STRNCMP(a, b) strncmp(a, b, strlen(b))

static char buf[10*1024*1024];

static int
decode_hex(const char *s, unsigned *v)
{
    return sscanf(s, "%x", v) == 1;
}

static unsigned
decode_hex_def(const char *s)
{
    unsigned v;
    return decode_hex(s, &v) ? v : -1;
}

static double
decode_float(const char *s)
{
    unsigned v;
    double d;

    if (! s) return NAN;
    if (! decode_hex(s, &v)) return NAN;
    d = (v & 0x7fffff) / ((double) 0x800000) + 1;
    d = d * pow(2.0, ((v >> 23) & 0xff) - 127);
    if (v & (1 << 31)) d = -d;

    return d;
}

static void
report_drive(unsigned cmd)
{
    static double last = 0;
    double v = decode_float(&buf[4]);

    printf("DRIVE %d PRODUCE %f DELTA %f\n", cmd, v, v - last);
    last = v;
}

static void
report_ping()
{
    static int n_pings = 0;
    static double last = 0;
    double v = decode_float(&buf[4]);

    printf("PING %4d @ %f DELTA %f\n", ++n_pings, v, v - last);
    last = v;
}

static const char *
get_next_value(const char *s)
{
    const char *f = strchr(s, ',');
    if (f) return f+1;
    else return NULL;
}

static void
report_splice(int from, int to)
{
    const char *heat = &buf[4];
    const char *comp = get_next_value(heat);
    const char *reverse = get_next_value(comp);

    printf("SPLICE %d -> %d: heat %f compression %f%s\n", from, to, decode_float(heat), decode_float(comp), reverse && atoi(reverse) ? " reverse" : "");
}

int
main(int argc, char **argv)
{
    while (fgets(buf, sizeof(buf), stdin) != NULL) {
	if (STRNCMP(buf, "MSF") == 0) {
	    printf("MSF Version %s", &buf[3]);
	} else if (STRNCMP(buf, "cu:") == 0) {
	} else if (STRNCMP(buf, "ppm:") == 0) {
	    printf("Pulses per mm: %f\n", decode_float(&buf[4]));
	} else if (STRNCMP(buf, "lo:") == 0) {
	    printf("Loading offset: %d\n", decode_hex_def(&buf[3]));
	} else if (STRNCMP(buf, "ns:") == 0) {
	    printf("# splices: %d\n", decode_hex_def(&buf[3]));
	} else if (STRNCMP(buf, "np:") == 0) {
	    printf("# pings: %d\n", decode_hex_def(&buf[3]));
	} else if (STRNCMP(buf, "nh:") == 0) {
	    printf("I don't know what nh is\n");
	} else if (STRNCMP(buf, "na:") == 0) {
	    printf("# splice settings: %d\n", decode_hex_def(&buf[3]));
	} else if (buf[0] == '(') {
	    unsigned cmd;

	    if (! decode_hex(&buf[1], &cmd)) printf("ERROR in cmd %s\n", &buf[1]);
	    else if (cmd < 4) report_drive(cmd);
	    else if (cmd == 0x64) report_ping(cmd);
	    else report_splice((cmd >> 4), cmd & 0xf);
	}
    }
    return 0;
}
