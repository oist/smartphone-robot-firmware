#include "pico/types.h"
#include "serial_comm_manager.h"
#include "pico/stdio.h"
#include <string.h>
#include "robot.h"
#include "custom_printf.h"

static IncomingPacketFromAndroid incoming_packet_from_android;
static OutgoingPacketToAndroid outgoing_packet_to_android;
static RP2040_STATE rp2040_state;
void handle_packet(IncomingPacketFromAndroid *packet);

void serial_comm_manager_init(){
    incoming_packet_from_android.start_marker = START_MARKER;
    incoming_packet_from_android.end_marker = END_MARKER;
    outgoing_packet_to_android.start_marker = START_MARKER;
    outgoing_packet_to_android.end_marker = END_MARKER;
    
}
// reads data from the UART and stores it in buffer. If no data is available, returns immediately.
// if new data is available, reads it until the buffer is full or both start and stop markers detected
// calls handle_block to process the data if both markers are detected
void get_block() {
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
        while (buffer_index < (ANDROID_BUFFER_LENGTH_IN + 1) || i == MAX_SERIAL_GET_COUNT) {
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
    	        assert(false);
	    }
    	    i++;
        }
	
	c = getchar_timeout_us(100);

        if (c != PICO_ERROR_TIMEOUT && c == END_MARKER){
            // Calculate the length of the packet
            uint16_t packet_length = end_idx - start_idx;
            if (packet_length >= sizeof(IncomingPacketFromAndroid)) {
                // Call the handle_block function with the packet data
                handle_packet(&incoming_packet_from_android);
                // Reset the values of start and end idx to detect the next block
                start_idx = -1;
                end_idx = -1;
                buffer_index = 0;
	        // Reset the packet
	        memset(&incoming_packet_from_android, 0, sizeof(IncomingPacketFromAndroid)); } else {
                //synchronized_printf("Received incomplete packet.\n");
		assert(false);
            }
        }else{
	    //synchronized_printf("Received packet with no end marker.\n");
	    assert(false);
	}

    }
}

void handle_packet(IncomingPacketFromAndroid *packet){
// clear the outgoing packet
    outgoing_packet_to_android.packet_type = 0;
    memset(outgoing_packet_to_android.data, 0, sizeof(outgoing_packet_to_android.data));
    // Assign the same packet type to the outgoing packet for verification on Android end
    outgoing_packet_to_android.packet_type = packet->packet_type;
    switch (packet->packet_type){
    	case GET_LOG:
	        // TODO
	        break;
	case SET_MOTOR_LEVEL:
	    // Copy the motor levels from the packet to the rp2040_state
            memcpy(&rp2040_state.MotorsState.ControlValues.left, &packet->data[0], sizeof(uint8_t));
	    memcpy(&rp2040_state.MotorsState.ControlValues.right, &packet->data[1], sizeof(uint8_t));
	    process_motor_levels(&rp2040_state);
            // Add STATE to response
            get_state(&rp2040_state); 
            memcpy(&outgoing_packet_to_android.data[0], &rp2040_state, sizeof(RP2040_STATE));
	    break;
	case RESET_STATE:
		// TODO
		break;
	default:
		outgoing_packet_to_android.packet_type = NACK;
		break;
    }
    uint8_t* bytes = (uint8_t*)&outgoing_packet_to_android;
    lock_printf_synchronization();
    for (int i = 0; i < sizeof(OutgoingPacketToAndroid); i++){
        putchar(bytes[i]);
    }
    unlock_printf_synchronization();
}
