#CPPFLAGS=-g $(shell root-config --cflags) -std=c++0x
CPPFLAGS+= -I /usr/local/include
#LDFLAGS=-g $(shell root-config --ldflags)
#~ LDLIBS=$(shell root-config --libs)
LDLIBS=-L/usr/lib/x86_64-linux-gnu -L/usr/local/lib -lfftw3 -lrt -lm

SRCS= unetlink.c functions.c
OBJS=$(subst .c,.o,$(SRCS))

unetlink: $(OBJS)
	gcc $(LDFLAGS) -o unetlink unetlink.o functions.o $(LDLIBS) 

unetlink.o: unetlink.c functions.c functions.h
	gcc $(CPPFLAGS) -c unetlink.c
    
functions.o: functions.c
	gcc $(CPPFLAGS) -c functions.c
	
clean:

	rm *.o
