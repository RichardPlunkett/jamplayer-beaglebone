HEADERS = iolib.h jamarray.h jamcomp.h jamdefs.h jamexec.h jamexp.h jamexprt.h jamheap.h jamjtag.h jamport.h jamstack.h jamsym.h jamutil.h jamytab.h
OBJECTS = iolib.o jamarray.o jamcomp.o jamcrc.o jamexec.o jamexp.o jamheap.o jamjtag.o jamnote.o jamstack.o jamstub.o jamsym.o jamutil.o

default: jp

%.o: %.c $(HEADERS)
	gcc -c $< -o $@

jp: $(OBJECTS)
	gcc $(OBJECTS) -o $@

clean:
	-rm -f $(OBJECTS)
	-rm -f a.out
