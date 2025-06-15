# Variables
DOCKER_IMAGE := topher217/smartphone-robot-firmware
DOCKER_TAG := latest
JOBS ?= 4
DOCKER_DEBUG_CONTAINER := smartphone-robot-debug
DOCKER_RUN := docker run --rm -it \
    --device /dev/bus/usb:/dev/bus/usb \
    -v $(PWD):/project \
    -w /project \
    $(DOCKER_IMAGE):$(DOCKER_TAG)
DOCKER_EXEC := docker exec -it $(DOCKER_DEBUG_CONTAINER)

# Default target
.PHONY: all
all: help

# Help target
.PHONY: help
help:
	@echo "Available targets (run from project root):"
	@echo "  make firmware    - Build firmware (in Docker)"
	@echo "  make openocd     - Start OpenOCD GDB server in Docker container (leave running in one terminal)" 
	@echo "  make debug       - Start GDB in the same Docker container (run in a second terminal)"
	@echo "  make docker-clean - Stop and remove the debug container if needed"
	@echo "  make flash       - Flash firmware to device (in Docker)"
	@echo "  make clean       - Remove build artifacts"
	@echo "  make docker      - Build or rebuild the Docker image (must be in project root)"
	@echo "  make shell       - Start an interactive shell in the Docker container"
	@echo ""
	@echo "Assumptions:"
	@echo "  - All commands must be run from the project root directory (where this Makefile is located)."
	@echo "  - All builds and operations are performed inside Docker. Local builds are not supported."
	@echo "  - USB debug probe access is provided via --device /dev/bus/usb:/dev/bus/usb."
	@echo ""
	@echo "Options:"
	@echo "  JOBS=N                - Number of parallel build jobs (default: 4)"
	@echo "  Example: make flash DOCKER_USB_DEVICE=/dev/ttyACM0"

# Build firmware
.PHONY: firmware
firmware:
	@echo "Building firmware in Docker with $(JOBS) jobs..."
	$(DOCKER_RUN) bash -c "rm -rf build && mkdir build && cd build && cmake .. && make -j$(JOBS)"

# Name for the persistent debug container
DEBUG_CONTAINER := smartphone-robot-debug

# Start OpenOCD GDB server in a persistent, named Docker container (detached)
.PHONY: openocd
openocd:
	@echo "Starting OpenOCD GDB server in Docker container ($(DEBUG_CONTAINER))..."
	docker rm -f $(DEBUG_CONTAINER) 2>/dev/null || true
	docker run --name $(DEBUG_CONTAINER) \
	    --device /dev/bus/usb:/dev/bus/usb \
	    -v $(PWD):/project \
	    -w /project \
	    $(DOCKER_IMAGE):$(DOCKER_TAG) \
	    bash -c 'openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000"'
	@echo "OpenOCD exited. If you want to debug again, re-run this target."

# Start GDB in the same running container as OpenOCD
.PHONY: debug
debug:
	@echo "Starting GDB in running debug container ($(DEBUG_CONTAINER))..."
	docker exec -it $(DEBUG_CONTAINER) bash -c 'gdb-multiarch build/robot.elf -x .gdbinit'

# Stop and remove the persistent debug container
.PHONY: docker-clean
docker-clean:
	@echo "Stopping and removing debug container ($(DEBUG_CONTAINER))..."
	docker rm -f $(DEBUG_CONTAINER) 2>/dev/null || true

# Flash firmware
.PHONY: flash
flash:
	@echo "Flashing firmware in Docker..."
	$(DOCKER_RUN) bash -c "openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg  -c 'adapter speed 5000' -c 'program build/robot.elf verify reset exit'" || \
		(echo '\n[ERROR] OpenOCD could not access the CMSIS-DAP device. If you see errors like "could not claim interface" or "Input/Output Error", try unplugging and replugging your debug probe, and ensure no other process is using it.\n'; exit 1)

# Clean build artifacts
.PHONY: clean
clean:
	rm -rf build

# Build (or rebuild) the Docker image
.PHONY: docker
docker:
	cd docker && docker build $(DOCKER_BUILD_ARGS) -t $(DOCKER_IMAGE):$(DOCKER_TAG) .

# Interactive shell in the Docker container
.PHONY: shell
shell:
	@echo "Starting interactive shell in Docker..."
	$(DOCKER_RUN) bash