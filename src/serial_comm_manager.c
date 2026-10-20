#include "pico/types.h"
#include "serial_comm_manager.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include <string.h>
#include <stdio.h>
#include "robot.h"
#include "rp2040_log.h"

static IncomingPacketFromAndroid incoming_packet_from_android;
static OutgoingPacketToAndroid outgoing_packet_to_android;
static OutgoingLogPacketToAndroid outgoing_log_packet_to_android;
void handle_packet(IncomingPacketFromAndroid *packet);
static RP2040_STATE rp2040_state_;

static void send_state_response(uint8_t packet_type)
{
    outgoing_packet_to_android.packet_type = packet_type;
    health_fill(&rp2040_state_.Health);
    outgoing_packet_to_android.data = rp2040_state_;

    uint8_t* bytes = (uint8_t*)&outgoing_packet_to_android;
    for (int i = 0; i < sizeof(outgoing_packet_to_android); i++){
        putchar(bytes[i]);
    }
}

// The DRV8830s hold their last command until told otherwise, so a host that
// stops talking (app crash, USB unplugged, phone asleep) would leave the
// wheels driving. Stop them if no motor command arrives for this long.
#define MOTOR_COMMAND_TIMEOUT_MS 250
// While the host is silent, keep re-sending the stop: one I2C write can be
// lost to motor noise, and a single stop that failed left one wheel spinning
// with the app closed (2026-10-06).
#define MOTOR_STOP_REPEAT_MS 50
static absolute_time_t motor_command_deadline;
static bool motors_driven = false;
// Silent from boot until the first command: drv8830_init() zeroes the motors
// once, and if that write is lost a wheel spins from power-up until a host
// connects (2026-10-06).
static bool host_silent = true;

void motor_command_watchdog(void){
    if (!time_reached(motor_command_deadline)){
        return;
    }
    if (motors_driven){
        rp2040_log_w("No motor command for %d ms; stopping motors\n", MOTOR_COMMAND_TIMEOUT_MS);
        motors_driven = false;
        host_silent = true;
    }
    if (host_silent){
        rp2040_state_.MotorsState.ControlValues.left = 0;
        rp2040_state_.MotorsState.ControlValues.right = 0;
        set_motor_levels(&rp2040_state_);
        motor_command_deadline = make_timeout_time_ms(MOTOR_STOP_REPEAT_MS);
    }
}

void serial_comm_manager_init(RP2040_STATE* rp2040_state){
    rp2040_state_ = *rp2040_state;
    incoming_packet_from_android.start_marker = START_MARKER;
    incoming_packet_from_android.end_marker = END_MARKER;
    outgoing_packet_to_android.start_marker = START_MARKER;
    outgoing_packet_to_android.end_marker = END_MARKER;
    outgoing_packet_to_android.data_size = sizeof(rp2040_state_);
    outgoing_log_packet_to_android.start_marker = START_MARKER;
    outgoing_log_packet_to_android.packet_type = GET_LOG;
    outgoing_log_packet_to_android.end_marker = END_MARKER;
}

static void reset_packet_and_send_nack(int8_t *start_idx, int8_t *end_idx, uint16_t *buffer_index) {
    *start_idx = -1;
    *end_idx = -1;
    *buffer_index = 0;
    memset(&incoming_packet_from_android, 0, sizeof(IncomingPacketFromAndroid));
    outgoing_packet_to_android.packet_type = NACK;
    uint8_t* nack_bytes = (uint8_t*)&outgoing_packet_to_android;
    for (int j = 0; j < sizeof(outgoing_packet_to_android); j++){
        putchar(nack_bytes[j]);
    }
}

// reads data from the UART and stores it in buffer. If no data is available, returns immediately.
// if new data is available, reads it until the buffer is full or both start and stop markers detected
// calls handle_block to process the data if both markers are detected
bool get_block(void) {
    // initialize as -1 as a way of detecting the absence of each marker in the buffer
    static int8_t start_idx = -1;
    static int8_t end_idx = -1;
    uint16_t buffer_index= 0;
    uint8_t i = 0;
    uint8_t MAX_SERIAL_GET_COUNT = 100;
    
    int c = getchar_timeout_us(100);
    // Only process data after finding the START_MARKER
    if (c != PICO_ERROR_TIMEOUT && c == START_MARKER){
        start_idx = buffer_index;
	// After finding the start marker get the rest of the packet or until MAX_SERIAL_GET_COUNT
	// to prevent an infinite loop
	// + 1 to accomodate for the packet_type
        while (buffer_index < (ANDROID_BUFFER_LENGTH_IN + 1) && i < MAX_SERIAL_GET_COUNT) {
            c = getchar_timeout_us(100);
    
    	    if (c != PICO_ERROR_TIMEOUT){
    	        if (buffer_index == 0){
    	    	    // First byte after start marker is the command
    	    	    incoming_packet_from_android.packet_type = (c & 0xFF);
    	    	    buffer_index++;
    	        }else{
		    // -2 to accomodate for the start marker and packet_type
    	    	    incoming_packet_from_android.data[buffer_index - 1] = (c & 0xFF);
    	    	    buffer_index++;
    	        }
    	    }else {
                rp2040_log_w("Timeout while reading packet. Resetting state.\n");
                reset_packet_and_send_nack(&start_idx, &end_idx, &buffer_index);
                return true;
	    }
    	    i++;
        }

        if (buffer_index < (ANDROID_BUFFER_LENGTH_IN + 1)) {
            rp2040_log_w("Received incomplete packet. Resetting state.\n");
            reset_packet_and_send_nack(&start_idx, &end_idx, &buffer_index);
            return true;
        }

	c = getchar_timeout_us(100);
        if (c != PICO_ERROR_TIMEOUT && c == END_MARKER){
            // Call the handle_block function with the packet data
            handle_packet(&incoming_packet_from_android);
            // Reset the values of start and end idx to detect the next block
            start_idx = -1;
            end_idx = -1;
            buffer_index = 0;
            // Reset the packet
            memset(&incoming_packet_from_android, 0, sizeof(IncomingPacketFromAndroid));
        }else{
            rp2040_log_w("Received packet with no end marker. Resetting state.\n");
            reset_packet_and_send_nack(&start_idx, &end_idx, &buffer_index);
            return true;
        }

        return true;
    }

    return false;
}

void handle_packet(IncomingPacketFromAndroid *packet){
    // Assign the same packet type to the outgoing packet for verification on Android end
    switch (packet->packet_type){
    	case GET_LOG:
	    outgoing_log_packet_to_android.packet_type = packet->packet_type;

	    putchar(outgoing_log_packet_to_android.start_marker);
	    putchar(outgoing_log_packet_to_android.packet_type);

	    rp2040_log_acquire_lock();
	    outgoing_log_packet_to_android.data_size = rp2040_get_byte_count();

	    // put uint16_t data_size via putchar
	    putchar(outgoing_log_packet_to_android.data_size & 0xFF);
	    putchar((outgoing_log_packet_to_android.data_size >> 8) & 0xFF);

	    rp2040_log_flush();
	    rp2040_log_release_lock();

	    putchar(outgoing_log_packet_to_android.end_marker);
	    break;
	case SET_MOTOR_LEVEL:
            memcpy(&rp2040_state_.MotorsState.ControlValues.left, &packet->data[0], sizeof(uint8_t));
	    memcpy(&rp2040_state_.MotorsState.ControlValues.right, &packet->data[1], sizeof(uint8_t));
	    set_motor_levels(&rp2040_state_);
	    // Bits 7..2 are the voltage; zero voltage is coast or brake, nothing to time out.
	    motors_driven = (packet->data[0] & 0xFC) || (packet->data[1] & 0xFC);
	    host_silent = false;
	    motor_command_deadline = make_timeout_time_ms(MOTOR_COMMAND_TIMEOUT_MS);
            get_fast_motor_state(&rp2040_state_);
            send_state_response(packet->packet_type);
	    break;
	case GET_STATE:
	    // Clear the state
            memset(&rp2040_state_, 0, sizeof(rp2040_state_));
            // Add STATE to response
            get_state(&rp2040_state_);
            send_state_response(packet->packet_type);
	    break;
	case RESET_STATE:
		// TODO
		break;
	default:
		outgoing_packet_to_android.packet_type = NACK;
		break;
    }
}
