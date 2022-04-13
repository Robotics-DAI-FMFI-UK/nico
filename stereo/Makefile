LIBS=-lpng

CCOPT=-std=c11 -g

all: zapis_obrazky 

zapis_obrazky:	zapis_obrazky.c pngwriter.c
	gcc -o zapis_obrazky $^ $(LIBS) $(CCOPT)

clean:
	rm -f zapis_obrazky image*.png
