.PHONY: help build test

SRC_FILES := $(shell find ./src -name '*.c' -o -name '*.h')

help: # show help for each of the Makefile recipes.
	@grep -E '^[a-zA-Z0-9 -]+:.*#'  Makefile | sort | while read -r l; do printf "\033[1;32m$$(echo $$l | cut -f 1 -d':')\033[00m:$$(echo $$l | cut -f 2- -d'#')\n"; done

build: # build target
	west build -b mini_stm32h743 . --build-dir ../build

rebuild: # rebuild target
	west build -b mini_stm32h743 . --pristine --build-dir ../build

build-pc-app: # build strat pc app
	west build -b native_sim_64 . --build-dir ../build

rebuild-pc-app: # build strat pc app
	west build -b native_sim_64 . --pristine --build-dir ../build

run-pc-app: # run strat pc app
	../build/zephyr/zephyr.exe

run-pc-gui: # run start pc ui
	python ../tools/strat_visualizer/src/main.py

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
