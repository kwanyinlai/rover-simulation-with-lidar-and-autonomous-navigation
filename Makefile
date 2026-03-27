CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c99 -MMD -MP -Isrc

BUILD_DIR = build
SRCS_DIR = src

UNAME := $(shell uname)

ifeq ($(UNAME), Darwin)
	CFLAGS += -DGL_SILENCE_DEPRECATION -I/opt/homebrew/include
	LIBS = -framework OpenCL -framework GLUT -framework OpenGL -lGLEW -L/opt/homebrew/lib -lm
else
	CFLAGS += -I/usr/include
	LIBS = -lGLEW -lGL -lGLU -lglut -lm
endif

TARGET = $(BUILD_DIR)/lidar_sim

SRCS = $(shell find $(SRCS_DIR) -name '*.c')

OBJS = $(patsubst $(SRCS_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))

DEPS = $(OBJS:.o=.d)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $^ -o $@ $(LIBS)

$(BUILD_DIR)/%.o: $(SRCS_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

-include $(DEPS)

clean:
	rm -rf $(BUILD_DIR)