CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -Iinclude -D_GNU_SOURCE

TARGET = bin/alps-stick
TEST_TAP_TARGET = bin/test_tap_gesture
TEST_SCROLL_TARGET = bin/test_scroll_wheel

DAEMON_SRCS = \
	src/main.c \
	src/core/config.c \
	src/features/tap_gesture.c \
	src/features/scroll_wheel.c \
	src/io/device.c \
	src/io/emitter.c

TEST_TAP_SRCS = \
	tests/test_tap_gesture.c \
	src/core/config.c \
	src/features/tap_gesture.c

TEST_SCROLL_SRCS = \
	tests/test_scroll_wheel.c \
	src/core/config.c \
	src/features/scroll_wheel.c

HEADERS = \
	include/core/event.h \
	include/core/action.h \
	include/core/config.h \
	include/features/tap_gesture.h \
	include/features/scroll_wheel.h \
	include/io/device.h \
	include/io/emitter.h

SERVICE = alps-stick.service
INSTALL_BIN = /usr/local/bin/alps-stick
INSTALL_SERVICE = /etc/systemd/system/$(SERVICE)

.PHONY: all test sanitize clean install uninstall

all: $(TARGET)

$(TARGET): $(DAEMON_SRCS) $(HEADERS)
	@mkdir -p bin
	$(CC) $(CFLAGS) -o $@ $(DAEMON_SRCS)

$(TEST_TAP_TARGET): $(TEST_TAP_SRCS) $(HEADERS)
	@mkdir -p bin
	$(CC) $(CFLAGS) -o $@ $(TEST_TAP_SRCS)

$(TEST_SCROLL_TARGET): $(TEST_SCROLL_SRCS) $(HEADERS)
	@mkdir -p bin
	$(CC) $(CFLAGS) -o $@ $(TEST_SCROLL_SRCS)

test: $(TEST_TAP_TARGET) $(TEST_SCROLL_TARGET)
	./$(TEST_TAP_TARGET)
	./$(TEST_SCROLL_TARGET)

sanitize: CFLAGS += -fsanitize=address,undefined -fno-omit-frame-pointer -g
sanitize: clean all test

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
