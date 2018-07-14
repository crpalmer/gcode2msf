all:	gcode2msf msf2text

gcode2msf: gcode2msf.c
	$(CC) gcode2msf.c -o gcode2msf -lm

msf2text: msf2text.c
	$(CC) msf2text.c -o msf2text -lm
