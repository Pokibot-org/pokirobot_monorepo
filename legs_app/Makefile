.PHONY: all test clean build

SRC_FILES := $(shell find ./src -name '*.c' -o -name '*.h')

help: # show help for each of the Makefile recipes.
	@grep -E '^[a-zA-Z0-9 -]+:.*#'  Makefile | sort | while read -r l; do printf "\033[1;32m$$(echo $$l | cut -f 1 -d':')\033[00m:$$(echo $$l | cut -f 2- -d'#')\n"; done

build: # build target
	west build -b pokimotor . --build-dir ../build

build-debug: # build debug target
	west build -b pokimotor . --build-dir ../build  -- -DOVERLAY_CONFIG="debug.conf"

rebuild: # rebuild target
	west build -b pokimotor . --pristine --build-dir ../build

clean: # clean project
	west build -t clean --build-dir ../build

flash: # flash target
	west flash --build-dir ../build

dbg: # debug target and connect with gdb
	west debug --build-dir ../build

dbgserv: # debug target and expose gdb server
	west debugserver --build-dir ../build

format: # format all files in the project
	clang-format -i $(SRC_FILES)
	tools/format_eof.sh $(SRC_FILES)
