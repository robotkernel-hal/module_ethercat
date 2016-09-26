CFLAGS=-Wall -g3 -O0
OBJECTS=hw.o datagram.o datagram_pool.o ec.o mbx.o coe.o
PROGRAM=main
CC=gcc 

LIBRARY=libethercat.a

$(PROGRAM): $(LIBRARY)
	gcc main.c $(LIBRARY) -o main -lpthread -lrt

$(LIBRARY): $(OBJECTS)
	ar cr $(LIBRARY) $(OBJECTS)

clean:
	rm -f $(OBJECTS)
	rm -f $(LIBRARY)
	rm -f $(PROGRAM)

scp: $(PROGRAM)
	scp $(PROGRAM) l_mobil@rmc-lx0071:

