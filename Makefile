UC_CMD = make -f teensy.mk INCLUDE_FLAGS="-Iinclude -IADC -DTEENSY_ASM" TEENSY=36 hex TEENSY_CORE_SPEED=240000000
MC_CMD = g++ -Iinclude -g

UC_CPP := $(wildcard ADC/*.cpp)
UC_C :=

all: uc mc

uc: bin/uc_phase_shift.hex bin/uc_raw.hex bin/uc_sim.hex

mc: bin/mc_sim bin/mc_std bin/mc_byte

bin/uc_phase_shift.hex: src/main.cpp src/input_adc.cpp src/process_phase_shift.cpp src/output_usb.cpp
	${UC_CMD} CPP_FILES="$^ $(UC_CPP)" C_FILES="$(UC_C)" TARGET=bin/uc_phase_shift

bin/uc_raw.hex: src/main.cpp src/input_adc.cpp src/process_byte.cpp src/output_usb.cpp
	${UC_CMD}  CPP_FILES="$^ $(UC_CPP)" C_FILES="$(UC_C)" TARGET=bin/uc_raw

bin/uc_sim.hex: src/main.cpp src/input_sim.cpp src/process_phase_shift.cpp src/output_usb.cpp
	${UC_CMD}  CPP_FILES="$^ $(UC_CPP)" C_FILES="$(UC_C)" TARGET=bin/uc_sim

bin/mc_sim: src/main.cpp src/input_sim.cpp src/process_phase_shift.cpp src/output_stdout.cpp
	${MC_CMD} $^ -o $@

bin/mc_std: src/main.cpp src/input_stdin.cpp src/process_phase_shift.cpp src/output_stdout.cpp
	${MC_CMD} $^ -o $@

bin/mc_byte: src/main.cpp src/input_byte.cpp src/process_phase_shift.cpp src/output_stdout.cpp
	${MC_CMD} $^ -o $@

teensy_loader_cli/teensy_loader_cli: teensy_loader_cli/teensy_loader_cli.c
	cd teensy_loader_cli && make

EXE=uc_phase_shift
flash: bin/${EXE}.hex teensy_loader_cli/teensy_loader_cli
	teensy_loader_cli/teensy_loader_cli --mcu=mk66fx1m0 -s bin/${EXE}.hex

test_flash: teensy_loader_cli/blink_slow_Teensy36.hex
	teensy_loader_cli/teensy_loader_cli --mcu=mk66fx1m0 -s teensy_loader_cli/blink_slow_Teensy36.hex

reboot: teensy_loader_cli/teensy_loader_cli
	teensy_loader_cli/teensy_loader_cli --mcu=mk66fx1m0 -b

.PHONY: clean flash reboot

clean:
	make -f teensy.mk clean
	rm -f bin/*

