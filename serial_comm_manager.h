#ifndef SERIAL_COMM_MANAGER_H
#define SERIAL_COMM_MANAGER_H

#include "hardware/platform_defs.h"
#include "pico/types.h"

#define RESPONSE_BUFFER_LENGTH _u(64)

#define GET_LOG 0x00
#define SET_MOTOR_LEVEL 0x01
#define SET_MOTOR_BRAKE 0x02
#define RESET_STATE 0x03

#define ON_WIRELESS_ATTACHED 0x00
#define ON_WIRELESS_DETACHED 0x01
#define ON_MOTOR_FAULT 0x02
#define ON_USB_ERROR 0x03

#define NACK 0xFC
#define ACK 0xFD
#define START_MARKER 0xFE
#define END_MARKER 0xFF

#define ANDROID_BUFFER_LENGTH_IN _u(2)
#define ANDROID_BUFFER_LENGTH_OUT _u(61) // +3 for start, command, and end marks for a 64 byte packet

typedef struct 
{
    uint8_t start_marker;
    uint8_t packet_type;
    uint8_t data[ANDROID_BUFFER_LENGTH_IN];
    uint8_t end_marker;
} IncomingPacketFromAndroid;

typedef struct 
{
    uint8_t start_marker;
    uint8_t packet_type;
    uint8_t data[ANDROID_BUFFER_LENGTH_OUT];
    uint8_t end_marker;
} OutgoingPacketToAndroid;

#pragma pack(1) // Set packing alignment to 1 byte
typedef struct
{
    struct
    {
        struct
        {
            uint8_t left;
            uint8_t right;
        } ControlValues;

        struct
        {
            // See drv8830 datasheet Table 8.
            uint8_t left;
            uint8_t right;
        } Faults;
        
        struct
        {
            uint32_t left;
            uint32_t right;
        } EncoderCounts;
    } MotorsState;

    struct
    {
	uint16_t voltage;
	uint8_t safety_status;
	uint16_t temperature;
	uint8_t state_of_health;
	uint16_t flags;
    } BatteryDetails;
    
    struct
    {
	uint32_t max77976_chg_details;
        uint8_t wireless_charger_attached;	
	uint16_t usb_charger_voltage;
    } ChargeSideUSB;
    

} RP2040_STATE;
#pragma pack() // Reset packing alignment to default

void get_block();
void serial_comm_manager_init();

#endif
