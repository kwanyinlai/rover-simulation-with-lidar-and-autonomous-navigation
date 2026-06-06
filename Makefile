CFLAGS = -Wall -Wextra -O3 -MMD -MP -Isrc
EXTRAFLAGS = -g
# -03 for heavy optimisation since our program is pretty intensive

BUILD_DIR = build
SRCS_DIR = src

# detect OS for appropriate linking
UNAME := $(shell uname)

# compile differently for Linux vs MacOS since OpenGL linking is different
ifeq ($(UNAME), Darwin)
# silence deprecation warnings for OpenGL on MacOS, and add homebrew include path for GL headers
	CFLAGS += -DGL_SILENCE_DEPRECATION -I/opt/homebrew/include
	LIBS = -framework GLUT -framework OpenGL -L/opt/homebrew/lib -lm
else
# for Linux, we can just link against the system OpenGL libraries
	CFLAGS += -I/usr/include
	LIBS = -lGL -lGLU -lglut -lm
endif

CFLAGS += $(EXTRAFLAGS)

TARGET = $(BUILD_DIR)/lidar_sim

# run shell command to find all .c files in src dir and subdirs
SRCS = $(shell find $(SRCS_DIR) -name '*.c')

# use pattern substitution to create corresponding .o files in build dir
OBJS = $(patsubst $(SRCS_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))

DEPS = $(OBJS:.o=.d)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	gcc $(CFLAGS) $^ -o $@ $(LIBS)

$(BUILD_DIR)/%.o: $(SRCS_DIR)/%.c
	@mkdir -p $(dir $@)
	gcc $(CFLAGS) -c $< -o $@

-include $(DEPS)

clean:
	rm -rf $(BUILD_DIR)