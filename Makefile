all:	gcode2msf msf2text

GCODE2MSF_OBJS = gcode2msf.o materials.o yaml-wrapper.o

CFLAGS=-g

gcode2msf: $(GCODE2MSF_OBJS)
	$(CC) $(GCODE2MSF_OBJS) -o gcode2msf -lm -lyaml

gcode2msf.o: materials.h
materials.o: materials.h

msf2text: msf2text.c
	$(CC) msf2text.c -o msf2text -lm
