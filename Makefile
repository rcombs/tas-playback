CFLAGS += -std=c11 -g

all: serial arduino
serial: serial.c

.PHONY: arduino upload clean clean-arduino

arduino:
	$(MAKE) -C serial-n64

upload:
	$(MAKE) -C serial-n64 upload

clean: clean-arduino
	rm serial

clean-arduino:
	$(MAKE) -C serial-n64 clean
