MAKE_DIR = $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

.phony: all clean wipe

all:
	mkdir -p $(MAKE_DIR)/build && cd $(MAKE_DIR)/build && cmake .. && make

clean:
	cd $(MAKE_DIR)/build && make clean

wipe:
	rm -fr $(MAKE_DIR)/build && rm -fr $(MAKE_DIR)/lib
