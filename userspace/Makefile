CFLAGS += -Wall -I../kernel -O0 -g
LDFLAGS += 

#MAKEFLAGS += -rR --no-print-directory

.PHONY: clean all

all: h264enc thumb

h264enc_objects = h264enc.o

h264enc: $(h264enc_objects)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

thumb_objects = thumb.o

thumb: $(thumb_objects)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

clean:
	rm -f *.o
	rm -f *.P
	rm -f h264enc
	rm -f thumb

%.o: %.c
	$(CC) $(CFLAGS) -MD -MP -MQ $@ -c $< -o $@

-include $(h264enc_objects:%.o=%.P)
-include $(thumb_objects:%.o=%.P)
