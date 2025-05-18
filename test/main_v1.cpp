#include <Arduino.h>
#include "motor.h"
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

// float vertical_p = 10.7;
// float vertical_p = 20;
// float vertical_i = 10;
// float vertical_d = 8;

// best values
// float vertical_p = 25*0.7;
// float vertical_i = 1;
// float vertical_d = 75*0.7;

// low
// float vertical_p = 25*0.6;
// float vertical_i = 1;
// float vertical_d = 40*0.6;


float vertical_p = 15;
float vertical_i = 0;
float vertical_d = 10;


float vertical_integral = 0;
float vertical_last_error = 0;
uint32_t vertical_last_time = 0;

int vertical_control(float angle_error) {
    uint32_t current_time = millis();
    Serial.print("pass time: ");Serial.print(current_time - vertical_last_time);
    angle_error = angle_error * -1; // invert the angle error to make it positive when tilting forward
    
    vertical_integral += angle_error;
    // limit the integral -60 ~ 60
    if(vertical_integral > 60) vertical_integral = 60;
    if(vertical_integral < -60) vertical_integral = -60;

    
    float p_term = vertical_p * angle_error;
    float d_term = vertical_d * ((angle_error - vertical_last_error) );
    float i_term = vertical_i * (vertical_integral);
    float pwm = p_term + d_term + i_term;
    
    vertical_last_time = current_time;
    vertical_last_error = angle_error;

    // debug print all values
    Serial.print(" p: ");Serial.print(p_term);
    Serial.print(", d: ");Serial.print(d_term);
    Serial.print(", angle_error: ");Serial.print(angle_error);
    Serial.print(", l e: ");Serial.print(vertical_last_error);
    Serial.print(", l t: ");Serial.print(vertical_last_time);
    Serial.print(", c t: ");Serial.print(current_time);
    Serial.print(", pwm: ");Serial.println(pwm);
    // Serial.print(", pass time: ");Serial.println(current_time - vertical_last_time);    


    return pwm;    
}

float angle_bias = 0;

void setup() {
    Serial.begin(115200);
    
    Wire.begin(33, 32);
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true, 1000, 500);
    
    motor_init();
    // read angle bias
    // mpu6050.update();
    // angle_bias = mpu6050.getAngleY();
    // Serial.print("angle bias: ");Serial.println(angle_bias);

    float avg = 0;
    for(int i = 0; i < 100; i++){
        mpu6050.update();
        avg += mpu6050.getAngleY();
        delay(10);
    }
    avg /= 100;
    angle_bias = avg;
    Serial.print("angle bias: ");Serial.println(angle_bias);


}

uint32_t last_time = 0;

void loop() {

    if(millis() - last_time > 10){
        last_time = millis();
        mpu6050.update();
        float angle_error = mpu6050.getAngleY() - angle_bias;
        
        int pwm = vertical_control(angle_error);
        // Serial.print("angleY: ");Serial.print(angle_error);
        if(abs(angle_error) < 50){
                
            // Serial.print(", pwm: ");Serial.println(pwm);
            
            motor_set_speed(pwm, pwm);
        } else {
            motor_set_speed(0, 0);
        }
    }
}

