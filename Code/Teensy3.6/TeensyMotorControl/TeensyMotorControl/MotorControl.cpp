// 
// 
// 

//Pin Numbers for Motor Control Pins
const int MotorEnable = 7;
const int MotorDirection = 8;
const int PWMCurrentControl = 10;
const int MotorSpeed_Feedback = 15;
const int MotorCurrent_Feedback = 16;
const int MotorControl_Input = 20;

#include "MotorControl.h"

void Init_MotorInterface() {
	pinMode(MotorControl_Input, INPUT);//Set up Pin 20 to Input to Control Turning on Motor from Host MSP430
	pinMode(MotorEnable, OUTPUT); //Set up Motor Enable as an Output
	pinMode(MotorDirection, OUTPUT); //Set up motor direction as an output
	digitalWrite(MotorEnable, LOW); //Init to low
	digitalWrite(MotorDirection, LOW); //Init to low
	pinMode(MotorSpeed_Feedback, INPUT); //Motor Speed Feedback as Input
	pinMode(PWMCurrentControl, OUTPUT); //Set up current control as an output
	digitalWrite(PWMCurrentControl, LOW); //Set output to low.  This may eventually be changed to a PWM signal, 
				    						//but now it will be left as a GPIO with a low outputl
	Init_PWM(); //Initiliaze the PWM Signal
	Init_ADC(); //Initialize the ADC for speed measurements back from Motor.

}

void Init_ADC() {
	//Initliaze ADC
	if ((ADC0_SC3 & 0x40) == 0x00) { //Check if Calibration Passed
	ADC0_CFG1 = 0x0C; //Sets up 16 bit Resolution
	ADC0_SC2 = 0x00; //Set Reference to VREFH = 2.5V and VREFL = 0V
	ADC0_SC3 = 0x05; //Sets up single conversion, takes 8 samples and averages them.
	Serial.println("ADC Successfully Initialized");
	}
	else {
		Serial.println("ADC Failed to Initialize");
	}
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
	TPM1_SC = 0x2C; // Set up for Center aligned, Increments on TPM clock, Divide clock by 16
	SIM_SOPT2 |= 0x1000000; //Set up IRC48M as Input Clock to TPM
	PORTB_PCR1 = 0x600; //Set up PTB1 (Teensy Pin 17) as output for TPM1_CH1
	Serial.println("PWM Successfully Initialized");
}

void BeginADCConversion() { //Need to set up interrupt
	ADC0_SC1A = 0x0E; //Set up for correct channel, writing to this register starts a conversion
	//ADC0_SC1A = 0x4E; //Set up for correct channel, Enables Interrupt upon Completion
	//Write to ADC0_SC1A
}

void ADC_Calibration() {
	ADC0_SC3 = 0xC0; //Begins calibration
}