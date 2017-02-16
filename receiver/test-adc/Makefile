#CPPFLAGS=-g $(shell root-config --cflags) -std=c++0x
CPPFLAGS+= -I /usr/local/include
#LDFLAGS=-g $(shell root-config --ldflags)
#~ LDLIBS=$(shell root-config --libs)
LDLIBS=-L/usr/lib/x86_64-linux-gnu -L/usr/local/lib -lrt -lm

SRCS= main.c
OBJS=$(subst .c,.o,$(SRCS))

main: $(OBJS)
	gcc $(LDFLAGS) -o main main.o $(LDLIBS) 

main.o: main.c
	gcc $(CPPFLAGS) -c main.c
	
clean:

	rm *.o
