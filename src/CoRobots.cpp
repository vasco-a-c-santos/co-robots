/**
 * Co-Robots
 *
 * Main source code file
 * 
 */

#include "CoRobots.h"
#include "TouchSensorDriver.h"
#include "EncoderDriver.h"
#include "MotorDriver.h"
#include "MotionController.h"

bool run = false;

// instantiate Touch Sensor class
TouchSensorDriver touchSensorDriver(TOUCH_PIN_NUMBER, VALUE_THRESHOLD);

// instantiate Encoder class
EncoderDriver encoderA(ENCODER_A_PIN);
EncoderDriver encoderB(ENCODER_B_PIN);

void IRAM_ATTR counterA () {
	encoderA.incrementInterruptCounter(); // Increase the impulses 
}
void IRAM_ATTR counterB () {
	encoderB.incrementInterruptCounter(); // Increase the impulses 
}

// instantiate MOTOR class
MotorDriver motorA(MOTOR_A_ENABLE_PIN, MOTOR_A_IN_1, MOTOR_A_IN_2, PWM_FREQUENCY,
	PWM_MOTOR_A_CHANNEL, PWM_RESOLUTION, PWM_MOTOR_A_DUTY_CYCLE);
MotorDriver motorB(MOTOR_B_ENABLE_PIN, MOTOR_B_IN_3, MOTOR_B_IN_4, PWM_FREQUENCY,
	PWM_MOTOR_B_CHANNEL, PWM_RESOLUTION, PWM_MOTOR_B_DUTY_CYCLE);

void setup() {
	Serial.begin(115200);
	delay(1000);
	encoderA.begin();
	encoderB.begin();
	attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), counterA, FALLING); // Configuration of interruption 0, where the HC-020K encoder is connected. FALLING = the interruption acts when the pin signal falls: it goes from HIGH to LOW.
	attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), counterB, FALLING); // Configuration of interruption 0, where the HC-020K encoder is connected. FALLING = the interruption acts when the pin signal falls: it goes from HIGH to LOW.
	motorA.begin();
	motorB.begin();
}

void loop() {
	//Serial.println(1000*60*15);
	if (touchSensorDriver.touched()) {
		//Serial.println(touchSensorDriver.getTouchSensorValue());
		run = true;
		delay(2000);
	}
	if (run) {
		Serial.printf("%d  %d\n", encoderA.getInterruptCounterValue(), encoderB.getInterruptCounterValue());
		delay(1000);
	}
}