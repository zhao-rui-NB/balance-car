#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <Arduino.h>



// define hbridge pins

#define MOTOR_LEFT_EN 4
#define MOTOR_LEFT_IN1 17
#define MOTOR_LEFT_IN2 16

#define MOTOR_RIGHT_EN 19
#define MOTOR_RIGHT_IN1 5
#define MOTOR_RIGHT_IN2 18


// set motor speed function


void motor_init(){
    pinMode(MOTOR_LEFT_EN, OUTPUT);
    pinMode(MOTOR_LEFT_IN1, OUTPUT);
    pinMode(MOTOR_LEFT_IN2, OUTPUT);

    pinMode(MOTOR_RIGHT_EN, OUTPUT);
    pinMode(MOTOR_RIGHT_IN1, OUTPUT);
    pinMode(MOTOR_RIGHT_IN2, OUTPUT);

    ledcSetup(0, 10000, 8);
    ledcAttachPin(MOTOR_LEFT_EN, 0);

    ledcSetup(1, 10000, 8);
    ledcAttachPin(MOTOR_RIGHT_EN, 1);


    ledcWrite(0, 0);
    ledcWrite(1, 0);
}

void motor_set_speed(int leftSpeed, int rightSpeed) {
    ledcWrite(0, min(abs(leftSpeed ), 255));
    ledcWrite(1, min(abs(rightSpeed ), 255));

    digitalWrite(MOTOR_LEFT_IN1, (leftSpeed > 0) ? HIGH : LOW);
    digitalWrite(MOTOR_LEFT_IN2, (leftSpeed > 0) ? LOW : HIGH);
    digitalWrite(MOTOR_RIGHT_IN1, (rightSpeed > 0) ? HIGH : LOW);
    digitalWrite(MOTOR_RIGHT_IN2, (rightSpeed > 0) ? LOW : HIGH);
}

#endif // __MOTOR_H__