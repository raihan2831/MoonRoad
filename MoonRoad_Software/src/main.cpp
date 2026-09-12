#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "BMI088.h"
#include "PID.h"

// The BMI088 consists of two separate sensors internally.
// Instantiate accel and gyro separately with their respective I2C addresses.
Bmi088Accel accel(Wire, 0x19);
Bmi088Gyro gyro(Wire, 0x69);

Servo tvcServo;
#define SERVO_PIN 15

PID myPID(1.0f, 0.0f, 0.05f, -15.0f, 15.0f, 0.01f);

repeating_timer_t timer;
const float DT = 0.005f; // 5 milliseconds

volatile float current_measurement = 0.0f;
float target_setpoint = 0.0f;

bool control_loop_callback(struct repeating_timer *t) {
    float measurement = current_measurement;
    float control_output = myPID.compute(target_setpoint, measurement, DT);
    tvcServo.write(control_output);
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Wire.setSDA(4);
    Wire.setSCL(5);
    Wire.begin();

    // The begin() method returns an integer status; negative values indicate an error
    // Initialize BMI088 Accelerometer
    if (accel.begin() < 0) {
        Serial.println("ERROR: BMI088 Accelerometer initialization failed!");
        while (true) { delay(100); }
    }
    
    // Initialize BMI088 Gyroscope
    if (gyro.begin() < 0) {
        Serial.println("ERROR: BMI088 Gyroscope initialization failed!");
        while (true) { delay(100); }
    }
    
    Serial.println("BMI088 Initialized successfully!");

    tvcServo.attach(SERVO_PIN);
    tvcServo.write(0);

    // Start the repeating hardware timer to call control_loop_callback every 5ms (-5 means exact intervals)
    add_repeating_timer_ms(-5, control_loop_callback, nullptr, &timer);
}

void loop() {
    // Read sensor data using standard Bolder Flight methods
    accel.readSensor();
    gyro.readSensor();

    // Fetch the data from the sensor objects
    float ax = accel.getAccelX_mss();
    float ay = accel.getAccelY_mss();
    float az = accel.getAccelZ_mss();
    float gx = gyro.getGyroX_rads();
    float gy = gyro.getGyroY_rads();
    float gz = gyro.getGyroZ_rads();

    // Update the volatile variable used by the PID timer loop
    current_measurement = gy; 
    
    // Small delay to prevent the loop from spinning too aggressively
    delay(2);
}