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

CFLAGS=-g

gcode2msf: $(GCODE2MSF_OBJS)
	$(CC) $(GCODE2MSF_OBJS) -o gcode2msf -lm -lyaml

msf2text: msf2text.o
	$(CC) msf2text.o -o msf2text -lm

# compile and generate dependency info
%.o: %.c
	@echo "Building: $*.c"
	@gcc -c $(CFLAGS) $*.c -o $*.o
	@gcc -MM $(CFLAGS) $*.c > $*.d

clean:
	-rm *.o *.d $(EXECS)
