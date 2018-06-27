all:	grid-show hex2ascii

grid-show: grid-show.c
	$(CC) grid-show.c -o grid-show -lm

hex2ascii: hex2ascii.c
	$(CC) hex2ascii.c -o hex2ascii
