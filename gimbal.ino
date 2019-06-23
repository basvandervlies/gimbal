/******************************************************************************
	This is example sketch for Arduino.
	Shows how to control SimpleBGC-driven gimbal via Serial API.
	API specs are available at http://www.basecamelectronics.com/serialapi/

	Demo:  control camera angles by Serial API direct control and by
	emulating various RC input methods;

	Arduino hardware:
	- analog joystick on the pins A1, A2  (connect GND, +5V to the side outputs of its potentiometers)

	Gimbal settings:
	- RC control in SPEED mode, RC signal should come from active RC source
	- RC SPEED is set to about 30..100

	Copyright (c) 2014-2015 Aleksey Moskalenko
*******************************************************************************/
#include <inttypes.h>
#include <SBGC.h>
#include <SBGC_Arduino.h>


// Serial baud rate should match with the rate, configured for the SimpleBGC controller
#define SERIAL_SPEED 115200

// delay between commands, ms
#define SBGC_CMD_DELAY 20

/*****************************************************************************/

// Set serial port where SBGC32 is connectedM
#define serial Serial

// global
static SBGC_cmd_realtime_data_t rt_data;
int	is_connected = 0;
int	result = 0;
int	first = 1;

void setup() {
	serial.begin(SERIAL_SPEED);
	SBGC_Demo_setup(&serial);

  Serial.println("Bas en Mike");
	// Take a pause to let gimbal controller to initialize
	delay(3000);
     		SerialCommand cmd;
            cmd.init(SBGC_CMD_REALTIME_DATA_4);
		    result = sbgc_parser.send_cmd(cmd, 0);
	delay(100);
}

void process_in_queue() {
  // Serial.println("Yes");
	while(sbgc_parser.read_cmd()) {
		SerialCommand &cmd = sbgc_parser.in_cmd;

        if (!is_connected) {
            first = 0;
            is_connected = 1;
        }
    
        Serial.println("Yes while");
        // Serial.println(cmd.id);
		uint8_t error = 0;
		switch(cmd.id) {
		// Receive realtime data
		case SBGC_CMD_REALTIME_DATA_3:
		case SBGC_CMD_REALTIME_DATA_4:
			error = SBGC_cmd_realtime_data_unpack(rt_data, cmd);
			if(!error) {
                // Serial.println("Yes DATA");
                Serial.println(rt_data.imu_angle[YAW]);
                Serial.println(rt_data.target_angle[YAW]);
				// Extract some usefull data
				// Average stabilization error (0.001 degree)
				uint32_t err = (uint32_t)(abs(rt_data.imu_angle[ROLL] - rt_data.target_angle[ROLL])
					+ abs(rt_data.imu_angle[PITCH] - rt_data.target_angle[PITCH])
					+ abs(rt_data.imu_angle[YAW] - rt_data.target_angle[YAW])) * (uint32_t)(SBGC_ANGLE_DEGREE_SCALE*1000);
			
			
			} else {
				sbgc_parser.onParseError(error);
			}
			break;
		}
	}
}

void loop() {
	SBGC_cmd_control_t c = { 0, 0, 0, 0, 0, 0, 0 };
    uint8_t error = 0;


    process_in_queue();

	// Move camera to initial position (all angles are zero)
	// Set speed 30 degree/sec
	// c.mode = SBGC_CONTROL_MODE_ANGLE;
	// c.speedROLL = c.speedPITCH = c.speedYAW = 30 * SBGC_SPEED_SCALE;
	// SBGC_cmd_control_send(c, sbgc_parser);
    // SBGC_cmd_execute_menu_send(SBGC_MENU_RESET_IMU, sbgc_parser);
    // delay(3000);
    SerialCommand cmd;
    SBGC_cmd_execute_menu_send(35, sbgc_parser);
    delay(3000);
    if ( is_connected ) {
            cmd.init(SBGC_CMD_REALTIME_DATA_4);
		    result = sbgc_parser.send_cmd(cmd, 0);
            is_connected = 0;
            // Serial.println(result);
		} else { // Set version request to init connection
            if ( first ) {
                cmd.init(SBGC_CMD_BOARD_INFO);
                sbgc_parser.send_cmd(cmd, 0);
                delay(100);
            }
    }
  // exit(0);
}
