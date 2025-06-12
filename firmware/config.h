/*
    This is the configuration file
    for the firmware of the 2D Printer.
*/

// Connection with the PC:

#define BAUD_RATE 9600


// Hardware Configuration:

    // Led Pins:
    #define RX_LED_PIN 1
    #define TX_LED_PIN 2
    #define STATUS_LED_TOP_PIN 3
    #define STATUS_LED_MID_PIN 4
    #define STATUS_LED_BOT_PIN 5

    // Button pins
    #define HOMING_BUTTON_PIN 6
    #define WORKLIGHT_BUTTON_PIN 7

    // Motors:
    #define MOTOR_A_EN_PIN 8
    #define MOTOR_A_DIR_PIN 9
    #define MOTOR_A_STEP_PIN 10
    #define MOTOR_A_CS_PIN 11
    #define MOTOR_B_EN_PIN 12
    #define MOTOR_B_DIR_PIN 13
    #define MOTOR_B_STEP_PIN 14
    #define MOTOR_B_CS_PIN 15

    // Axis-Endswitches:
    #define X_AXIS_END_SWITCH_PIN 16
    #define Y_AXIS_END_SWITCH_PIN 17

    // Toolhead:
    #define SERVO_PIN 18
    #define WORKLIGHT_PIN 19