CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -D_GNU_SOURCE
TARGET = bin/alps-stick
SRC = src/alps_stick_daemon.c
SERVICE = alps-stick.service
INSTALL_BIN = /usr/local/bin/alps-stick
INSTALL_SERVICE = /etc/systemd/system/$(SERVICE)

.PHONY: all clean install uninstall

all: $(TARGET)

$(TARGET): $(SRC)
	@mkdir -p bin
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -rf bin

install: $(TARGET)
	install -D -m 755 $(TARGET) $(INSTALL_BIN)
	install -D -m 644 $(SERVICE) $(INSTALL_SERVICE)
	systemctl daemon-reload
	systemctl enable --now $(SERVICE)

uninstall:
	systemctl stop $(SERVICE) || true
	systemctl disable $(SERVICE) || true
	rm -f $(INSTALL_BIN)
	rm -f $(INSTALL_SERVICE)
	systemctl daemon-reload
