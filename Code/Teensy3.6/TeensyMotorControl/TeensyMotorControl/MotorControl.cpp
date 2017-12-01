// 
// 
// 

//Pin Numbers for Motor Control Pins
const int MotorEnable = 7;
const int MotorDirection = 8;
const int PWMSpeedControl = 9;
const int PWMCurrentControl = 10;
const int MotorSpeed = 15;
const int MotorCurrent = 16;

#include "MotorControl.h"
void Init_MotorInterface() {
	pinMode(MotorEnable, OUTPUT); //Set up Motor Enable as an Output
	pinMode(MotorDirection, OUTPUT); //Set up motor direction as an output
	digitalWrite(MotorEnable, LOW); //Init to low
	digitalWrite(MotorDirection, LOW); //Init to low
	pinMode(PWMCurrentControl, OUTPUT); //Set up current control as an output
	digitalWrite(PWMCurrentControl, LOW); //Set output to low.  This may eventually be changed to a PWM signal, 
				    						//but now it will be left as a GPIO with a low outputl
	Init_PWM(); //Initiliaze the PWM Signal
	Init_ADC();

}

void Init_ADC() {
	//Initliaze ADC
}

void Init_PWM() {
	//Just the Speed PWM signal will be initialized.  If time permits the Current PWM signal will be set up as well

	SIM_SOPT2 &= ~0x3000000; //Disable Clock
	SIM_SOPT2 |= 0x1000000; //Set up IRC48M as Input Clock to TPM
	TPM1_C1SC = 0x80; //Clear TPM1_C1SC
	TPM1_SC = 0x80; //Clear TPM1_SC
	TPM1_MOD = 20000; //Setup Timer to count to 20,000
	TPM1_C1SC = 0x28; //Setup PWM output to create pulse in that goes high on first, C1V, and low on second
	TPM1_C1V = 0; //Initialize to 0
	TPM1_SC = 0x28; // Set up for Center aligned, Increments on TPM clock, Divide clock by 1
	SIM_SOPT2 |= 0x1000000; //Set up IRC48M as Input Clock to TPM
	PORTB_PCR1 = 0x600; //Set up PTB1 (Teensy Pin 17) as output for TPM1_CH1
	Serial.println("PWM Successfully Initialized");
}