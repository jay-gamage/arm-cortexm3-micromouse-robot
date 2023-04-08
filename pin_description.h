// Header file contains pin description for MICROMOUSE - MATRIXBOT

#ifndef MBED_PIN_DESC
#define MBED_PIN_DESC

// MBED Pin Description:
#define RIGHT_STEERING_PIN              p5          // Front panel - right steering sensor
#define LEFT_STEERING_PIN               p7          // Front panel - left steering sensor

#define MOTOR_LEFT_CLOCK_PIN            p14         // Left motor clock generate
#define MOTOR_LEFT_DIR_PIN              p12         // Left motor direction control
#define MOTOR_RIGHT_CLOCK_PIN           p11         // Right motor clock generate
#define MOTOR_RIGHT_DIR_PIN             p10         // Right motor direction

#define LEFT_PROXIMITY_PIN              p9          // Back panel - left proximity sensor
#define CENTER_PROXIMITY_PIN            p6          // Front panel - centre proximity sensor
#define RIGHT_PROXIMITY_PIN             p8          // Back panel - right proximity sensor

#define SET_SWITCH_PIN                  p27         // Set switch
#define BEGIN_SWITCH_PIN                p28         // Reset switch

// PC1602F 16x2 Text LCD Pins:
#define LCD_RS_PIN                      p21         // Register select
#define LCD_E_PIN                       p22         // Enable pin
#define LCD_D0_PIN                      p23         // Data pin 0
#define LCD_D1_PIN                      p24         // Data pin 1
#define LCD_D2_PIN                      p25         // Data pin 2
#define LCD_D3_PIN                      p26         // Data pin e

// HC-SR04 Ultrasonic Sensor Pins:
#define ULTRASONIC_TX_PIN               p29         // Transmit
#define ULTRASONIC_RX_PIN               p30         // Receive

#endif