/*
    This is the configuration file
    for the firmware of the 2D Printer.
*/

// general:

#define SOFTWARE_VERSION "0.1.3"

// Connection with the PC:

#define BAUD_RATE 115200

//protocol Symbols:
#define TERMINATOR ';'
#define SEPERATOR ","
// commands: 
#define HOMING "h"
#define GO_TO "g"
#define DISENGAGE_TOOLHEAD "u"  
#define ENGAGE_TOOLHEAD "d"
#define CHANGE_TOOL "c"

// Hardware Configuration:

    // Led Pins (Bitshift register):
    #define TX = 0
    #define RX = 1
    #define Status top = 2
    #define Status mid = 3
    #define Satus bot = 4

    // Bitshift register Pins;
    #define SER 13
    #define SRCLK A4
    #define RCLK A5

    // Motors:
    //Pins:
    #define MOTOR_A_EN_PIN 2
    #define MOTOR_A_DIR_PIN 4
    #define MOTOR_A_STEP_PIN 3
    #define MOTOR_A_RX_PIN 5
    #define MOTOR_A_TX_PIN 6
    #define MOTOR_B_EN_PIN 7
    #define MOTOR_B_DIR_PIN 9
    #define MOTOR_B_STEP_PIN 8
    #define MOTOR_B_RX_PIN 10
    #define MOTOR_B_TX_PIN 11
    //other:
    #define MAX_MOTOR_CURRENT 400 // milli Ampere
    #define STEPPER_BAUD_RATE 115200
    #define FULLSTEP_TO_MICROMETER_RATIO 0.48
    #define MICROSTEPPING 1
    // function name of the used algorithm for step by step moving:
//    #define STEPS_ALGORITHM move_steps
//   #define STEPS_ALGORITHM move_steps_linear_interpolation_time // not working at the moment (same symptom as move_steps)
   #define STEPS_ALGORITHM move_steps_linear_interpolation_slope

    // for use with own implementation:
    // for x axis: WORKING_SPEED_DELAY >= 134
    // for y axis: WORKING_SPEED_DELAY >= 155
    // use WORKING_SPEED_DELAY >= 170 for safety
    #define WORKING_SPEED_DELAY 600
    #define HOMING_SPEED_DELAY 1200
    // homing offsets in 10 000 micrometer
    #define HOMING_OFFSET_X 0
    #define HOMING_OFFSET_Y 0
    #define HOMING_MOVEBACK 500
    #define HIGH_DELAY 10
    //debug:
    #define DISABLE_MOTORS false

    // Axis-Endswitches:
    #define X_AXIS_END_SWITCH_0_PIN A0
    #define X_AXIS_END_SWITCH_1_PIN A1
    #define Y_AXIS_END_SWITCH_0_PIN A2
    #define Y_AXIS_END_SWITCH_1_PIN A3

    // Toolhead:
    #define SERVO_PIN 12
    #define SERVO_UP_POSITION 10
    #define SERVO_DOWN_POSITION 0
    #define SERVO_CHANGE_TOOL_POSITION 80
    #define SERVO_TEST_POSITION 20

    // calculated values:
    #define STEP_TO_MICROMETER_RATIO (FULLSTEP_TO_MICROMETER_RATIO/MICROSTEPPING)
