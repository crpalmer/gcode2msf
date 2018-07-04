EXECS =gcode2msf msf2text

all:	$(EXECS)

GCODE2MSF_OBJS = \
	bb.o \
	gcode.o \
	gcode2msf.o \
	materials.o \
	printer.o \
	transition-block.o \
	yaml-wrapper.o

MSF2TEXT_OBJS = msf2text.o

# pull in dependency info for *existing* .o files
OBJS = $(GCODE2MSF_OBJS) $(MSF2TEXT_OBJS)
-include $(OBJS:.o=.d)

CFLAGS=-g -Wall -Werror

gcode2msf: $(GCODE2MSF_OBJS)
	$(CC) $(GCODE2MSF_OBJS) -o gcode2msf -lm -lyaml

msf2text: $(MSF2TEXT_OBJS)
	$(CC) $(MSF2TEXT_OBJS) -o msf2text -lm

# compile and generate dependency info
%.o: %.c
	@echo "Building: $*.c"
	@gcc -c $(CFLAGS) $*.c -o $*.o
	@gcc -MM $(CFLAGS) $*.c > $*.d

clean:
	-rm *.o *.d $(EXECS)
