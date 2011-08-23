.phony: all clean wipe

all:
	mkdir -p build && cd build && cmake .. && make

clean:
	cd build && make Makefile clean

wipe:
	rm -fr build && rm -fr lib
