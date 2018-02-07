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
int ADCData;

#include "MotorControl.h"

void Init_MotorInterface() {
	pinMode(MotorControl_Input, INPUT);//Set up Pin 20 to Input to Control Turning on Motor from Host MSP430
	pinMode(MotorEnable, OUTPUT); //Set up Motor Enable as an Output
	pinMode(MotorDirection, OUTPUT); //Set up motor direction as an output
	digitalWrite(MotorEnable, LOW); //Init to low
	digitalWrite(MotorDirection, LOW); //Init to low (CCW) High is CW
	pinMode(MotorSpeed_Feedback, INPUT); //Motor Speed Feedback as Input
	pinMode(PWMCurrentControl, OUTPUT); //Set up current control as an output
	digitalWrite(PWMCurrentControl, LOW); //Set output to low.  This may eventually be changed to a PWM signal, 
				    						//but now it will be left as a GPIO with a low output
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

void BeginADCConversion() {
	NVIC_ISER1 |= 0x80; //Enable Interrupt for ADC0
	ADC0_SC1A = 0x4E; //Set up for correct channel, Enables Interrupt upon Completion
}

void ADC_Calibration() {
	ADC0_SC3 = 0xC0; //Begins calibration
}

void adc0_isr() {
	ADCData = ADC0_RA; //Reading of this register clears interrupt
}

void TurnMotorOn() {
	TPM1_C1V = 1000; //Set PWM Duty Cycle to 5%.  This is below the threshold.  If you set duty cycle to 0 you have to toggle the Enable line to turn the motor on.
					 //By setting the duty cycle to 5% you dont need to toggle the line, and it wont be turning.
	digitalWrite(MotorEnable, HIGH); //Enable the motor
}

void TurnMotorOff() {
	digitalWrite(MotorEnable, LOW); //Disable the motor
	TPM1_C1V = 0x00; //Set PWM Duty Cycle to 0%
}

double EMA(double GyroData, double OldEMA) { 
	/*
	EMA is exponential weighted average.  It does a running average of the data (in this case the z-gyro data).  This is used to calculate the reaction wheel speed.
	This is done because an outlier point won't ruin the flow of data.  The data is weighted based off of a parameter.  For our use the most recent data point will
	have a weight of 0.8, and the old calculated EMA will have a weight of 0.2.
	More information can be found here: https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
	*/
	const double weight = 0.8;
	return (weight * GyroData) + ((1 - weight) * OldEMA);

}

void Motor_Direction(byte Direction) {
	switch (Direction) {
	case 0: 
		digitalWrite(MotorDirection, LOW); //CCW
		break;
	case 1: 
		digitalWrite(MotorDirection, HIGH); //CW
		break;
	}
}

void MotorSpeed(double MotorRPM) {
	int DutyCycle;
	// Motor speed input ranges (RPM)
	const int in_min = 0;
	const int in_max = 2590;

	// motor controller output ranges (Duty Cycle)
	const int out_min = 2000;
	const int out_max = 18000;

	DutyCycle = (MotorRPM - in_min) * ((out_max - out_min) / (in_max - in_min)) + out_min;

	if (DutyCycle < 2000) { //Is it less than the min
		TPM1_C1V = 2000;
	}
	else if (DutyCycle > 18000) { //Is it more than the max
		TPM1_C1V = 18000;
	}
	else {
		TPM1_C1V = DutyCycle; //Set it to calculated value
	}
}

void NoMotorSpeed() {
	TPM1_C1V = 1000; //This is the 5% that is no spinning but not off
	//This is done so that the enable line doesn't need to be toggled1
}