#include <Arduino.h>
#include "motor.h"
// #include <MPU6050_tockn.h>
#include <Wire.h>



#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050_6Axis_MotionApps612.h" // Uncomment this library to work with DMP 6.12 and comment on the above library.


MPU6050 mpu;

uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint8_t FIFOBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorFloat gravity;    // [x, y, z]            Gravity vector
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector



// float vertical_p = 15*0.7;
// float vertical_i = 1;
// float vertical_d = 130*0.7;


// float vertical_p = 14*0.7;
// float vertical_i = 1.2;
// float vertical_d = 125*0.7;

// float vertical_p = 15*0.7;
// float vertical_i = 1;
// float vertical_d = 130*0.7;

float percent = 0.7;
float vertical_p = 15.5*percent;
float vertical_i = 1;
float vertical_d = 110*percent;

float vertical_integral = 0;
float vertical_last_error = 0;
uint32_t vertical_last_time = 0;

// #define INTEGRAL_BUFFER_SIZE 350
// uint16_t integral_index = 0;
// float integral_buffer[INTEGRAL_BUFFER_SIZE] = {0};

int vertical_control(float angle_error) {
    uint32_t current_time = millis();
    Serial.print("pass time: ");Serial.print(current_time - vertical_last_time);
    
    vertical_integral += angle_error;
    // integral_buffer[integral_index] = angle_error;
    // integral_index = (integral_index + 1) % INTEGRAL_BUFFER_SIZE;
    // // average the integral over the buffer size
    // vertical_integral = 0;
    // for(int i = 0; i < INTEGRAL_BUFFER_SIZE; i++) {
    //     vertical_integral += integral_buffer[i];
    // }
    // Serial.print("\tintegral: ");Serial.print(vertical_integral);
    if(vertical_integral > 180) vertical_integral = 180;
    if(vertical_integral < -180) vertical_integral = -180;

    
    float p_term = vertical_p * angle_error;
    float d_term = vertical_d * ((angle_error - vertical_last_error) );
    float i_term = vertical_i * (vertical_integral);
    float pwm = p_term + d_term + i_term;
    
    vertical_last_time = current_time;
    vertical_last_error = angle_error;

    // Serial.print(" p: ");Serial.print(p_term);
    // Serial.print(", i: ");Serial.print(i_term);
    // Serial.print(", d: ");Serial.print(d_term);
    // Serial.print(", angle_error: ");Serial.print(angle_error);
    // Serial.print(", l e: ");Serial.print(vertical_last_error);
    // Serial.print(", l t: ");Serial.print(vertical_last_time);
    // Serial.print(", c t: ");Serial.print(current_time);
    // Serial.print(", pwm: ");Serial.println(pwm);
    // // Serial.print(", pass time: ");Serial.println(current_time - vertical_last_time);    

    Serial.println();

    return pwm;    
}

float angle_bias = 0;

void mpu6050_init() {
    mpu.initialize();
    Serial.println(F("Testing MPU6050 connection..."));
    if(mpu.testConnection() == false){
        Serial.println("MPU6050 connection failed");
        while(true);
    }
    else {
        Serial.println("MPU6050 connection successful");
    }

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);


    if (devStatus != 0) {
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
      while(1); // Loop forever if DMP initialization fails
    } else {
      Serial.println(F("DMP Initialization successful!"));
    }


    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);


    Serial.println(F("DMP ready! Waiting for first interrupt..."));
}


void setup() {
    Serial.begin(115200);
    
    Wire.begin(33, 32);
    
    motor_init();
    mpu6050_init();
}

uint32_t last_time = 0;

void loop() {



    static uint32_t lastUpdate = 0;
    uint32_t now = millis();
    
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
        //  10 ms
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        float yaw = ypr[0] * 180/M_PI;
        float pitch = ypr[1] * 180/M_PI;
        float roll = ypr[2] * 180/M_PI;

        int pwm = vertical_control(pitch);
        // Serial.print("angleY: ");Serial.print(angle_error);
        if(abs(pitch) < 50){
                
            // Serial.print(", pwm: ");Serial.println(pwm);
            
            motor_set_speed(pwm, pwm);
        } else {
            motor_set_speed(0, 0);
        }


        // Serial.printf("passtime: %d\t ypr: %.3f\t%.3f\t%.3f\n", now - lastUpdate, yaw, pitch, roll);
        lastUpdate = now;

    } 
}

