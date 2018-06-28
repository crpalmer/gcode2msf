all:	gcode2msf msf2text

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

bb.o: bb.h
gcode.o: gcode.h
gcode.h: bb.h
gcode2msf.o: bb.h gcode.h materials.h printer.h transition-block.h
materials.o: materials.h yaml-wrapper.h
transition-block.o: transition-block.h bb.h gcode.h
printer.o: printer.h yaml-wrapper.h

msf2text: msf2text.c
	$(CC) msf2text.c -o msf2text -lm
