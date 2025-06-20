/*
    This is the configuration file
    for the firmware of the 2D Printer.
*/

// general:

#define SOFTWARE_VERSION "Alpha"

// Connection with the PC:

#define TOGGLE_X_AXIS true
#define TOGGLE_Y_AXIS false
#define TOGGLE_X_Y_AXIS false

#define BAUD_RATE 115200
//protocol Symbols:
#define TERMINATOR ";"
#define SEPERATOR ","
// commands: 
#define HOMING "h"
#define GO_TO "g"
#define DISENGAGE_TOOLHEAD "u"  
#define ENGAGE_TOOLHEAD "d"


// Hardware Configuration:

    // Led Pins:
    #define TRX_LED_PIN 1
    #define STATUS_LED_TOP_PIN A5
    #define STATUS_LED_MID_PIN 3
    #define STATUS_LED_BOT_PIN 4

    // Button pins
    #define HOMING_BUTTON_PIN 5
    #define WORKLIGHT_BUTTON_PIN 6

    // Motors:
    //Pins:
    #define MOTOR_A_EN_PIN 2
    #define MOTOR_A_DIR_PIN 3
    #define MOTOR_A_STEP_PIN 4
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
    #define FULLSTEP_TO_MICROMETER_RATIO 0.05
    #define MICROSTEPPING 256
    #define USE_ACCELSTEPPER true
    #define MAX_ACCELERATION 3000 // for use with AccelStepper
    #define MAX_SPEED 14000     // for use with AccelStepper
    #define WORKING_SPEED 14000  // for use with AccelStepper
    #define WORKING_SPEED_DELAY 160 // for use with own implementation

    // Axis-Endswitches:
    #define X_AXIS_END_SWITCH_0_PIN A1
    #define X_AXIS_END_SWITCH_1_PIN A0
    #define Y_AXIS_END_SWITCH_0_PIN A2
    #define Y_AXIS_END_SWITCH_1_PIN A3

    // Toolhead:
    #define SERVO_PIN 13
    #define SERVO_UP_POSITION 10
    #define SERVO_DOWN_POSITION 0

    // calculated values:
    #define STEP_TO_MICROMETER_RATIO (FULLSTEP_TO_MICROMETER_RATIO/MICROSTEPPING)
    #define INVERSE_STEP_TO_MICROMETER_RATIO (1*STEP_TO_MICROMETER_RATIO)
