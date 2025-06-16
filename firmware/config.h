/*
    This is the configuration file
    for the firmware of the 2D Printer.
*/

// Connection with the PC:

#define BAUD_RATE 9600


// Hardware Configuration:

    // Led Pins:
    #define TRX_LED_PIN 1
    #define STATUS_LED_TOP_PIN 2
    #define STATUS_LED_MID_PIN 3
    #define STATUS_LED_BOT_PIN 4

    // Button pins
    #define HOMING_BUTTON_PIN 5
    #define WORKLIGHT_BUTTON_PIN 6

    // Motors:
    //Pins:
    #define MOTOR_A_EN_PIN 7
    #define MOTOR_A_DIR_PIN 8
    #define MOTOR_A_STEP_PIN 9
    #define MOTOR_B_EN_PIN 10
    #define MOTOR_B_DIR_PIN 11
    #define MOTOR_B_STEP_PIN 12
    //other:
    #define FULLSTEP_TO_MICROMETER_RATIO 1
    #define MICROSTEPPING 16
    #define MAX_ACCELERATION 3000
    #define MAX_SPEED 14000
    #define WORKING_SPEED 1000

    // Axis-Endswitches:
    #define X_AXIS_END_SWITCH_0_PIN 13
    #define X_AXIS_END_SWITCH_1_PIN 14
    #define Y_AXIS_END_SWITCH_0_PIN 15
    #define Y_AXIS_END_SWITCH_1_PIN 16

    // Toolhead:
    #define SERVO_PIN 17
    #define WORKLIGHT_PIN 18