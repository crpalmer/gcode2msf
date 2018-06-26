all:	grid-show hex2ascii extrusion msf2text

grid-show: grid-show.c
	$(CC) grid-show.c -o grid-show -lm

hex2ascii: hex2ascii.c
	$(CC) hex2ascii.c -o hex2ascii

extrusion: extrusion.c
	$(CC) extrusion.c -o extrusion

msf2text: msf2text.c
	$(CC) msf2text.c -o msf2text -lm
