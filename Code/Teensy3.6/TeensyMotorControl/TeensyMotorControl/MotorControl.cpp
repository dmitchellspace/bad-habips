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
const int Main_Batt_En = 28;
const int Backup_Batt_En = 6;

//ADC Variables
const int ADC0_ChSelect[3] = { 0x11, 0x12, 0x17 };
const int ADC1_ChSelect[6] = { 0x04, 0x05, 0x06, 0x07, 0x11, 0x17 };
int ADC0_Select = 0, ADC1_Select = 0;
int ADCData[9]; //This array is [S1 S2 S3 S4 S5 S6 M1 M2 M3].  This is what gets written to the SD Card
float BatteryCheckData[9], RunningMainVoltage, RunningMotorVoltage;

//The battery divider values are taken from the resistors used to scale down the voltages coming back from the board
const float BatteryDivider[9] = { 0.59523, 0.29752, 0.19844, 0.148766, 0.11841, 0.09921, 0.59523, 0.29752, 0.19844 };
const float ADC_LSB = 0.00005035400390625; //3.3V scale, 16 bit resolution
bool MainBatteryBad, MotorBatteryBad;
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
	Init_ADC(); //Initialize the ADC for speed measurements back from Motor, as well as battery measurements.

}

void Init_ADC() {
	short CalData;
	//Initliaze ADC0
	pinMode(Backup_Batt_En, OUTPUT);
	digitalWrite(Backup_Batt_En, HIGH); //Init the backup as on
	pinMode(Main_Batt_En, OUTPUT);
	digitalWrite(Main_Batt_En, LOW); //Init the main as off
	
	if ((ADC0_SC3 & 0x40) == 0x00) { //Check if Calibration Passed
		NVIC_ISER1 |= 0x80; //Enable Interrupt for ADC0
		CalData = ADC0_CLP0 + ADC0_CLP1 + ADC0_CLP2 + ADC0_CLP3 + ADC0_CLP4 + ADC0_CLPS;
		CalData = CalData / 2;
		CalData |= 0x8000;
		ADC0_PG = CalData;
		CalData = ADC0_CLM0 + ADC0_CLM1 + ADC0_CLM2 + ADC0_CLM3 + ADC0_CLM4 + ADC0_CLMS;
		CalData = CalData / 2;
		CalData |= 0x8000;
		ADC0_MG = CalData;
		ADC0_CFG1 = 0x0C; //Sets up 16 bit Resolution
		ADC0_CFG2 = 0x00;
		ADC0_SC2 = 0x00; //Set Reference to VREFH = 2.5V and VREFL = 0V
		ADC0_SC3 = 0x05; //Set up averaging
		Serial.println("ADC0 Successfully Initialized");
	}
	else {
		Serial.println("ADC0 Failed to Initialize");
	}

	if ((ADC1_SC3 & 0x40) == 0x00) { //Check if Calibration Passed
		NVIC_ISER2 |= 0x200; //Enable Interrupt for ADC1
		CalData = ADC1_CLP0 + ADC1_CLP1 + ADC1_CLP2 + ADC1_CLP3 + ADC1_CLP4 + ADC1_CLPS;
		CalData = CalData / 2;
		CalData |= 0x8000;
		ADC1_PG = CalData;
		CalData = ADC1_CLM0 + ADC1_CLM1 + ADC1_CLM2 + ADC1_CLM3 + ADC1_CLM4 + ADC1_CLMS;
		CalData = CalData / 2;
		CalData |= 0x8000;
		ADC1_MG = CalData;
		ADC1_CFG1 = 0x0C; //Sets up 16 bit Resolution
		ADC0_CFG2 = 0x10; //Set up pin set b
		ADC1_SC2 = 0x00; //Set Reference to VREFH = 2.5V and VREFL = 0V
		ADC1_SC3 = 0x05; //Set up averaging
		Serial.println("ADC1 Successfully Initialized");
	}
	else {
		Serial.println("ADC1 Failed to Initialize");
	}

	ADC0_Select = 0;
	ADC1_Select = 0;
	BeginADCConversion(); //Kick off the first set of conversions
	
	while ((ADC0_Select <= 2) && (ADC1_Select <= 5)) { //Wait for it to finish the first series of conversions
		delay(0);
	}
	
	CheckBatteryLevel(); //Turn on the correct battery
	ADC0_Select = 0;
	ADC1_Select = 0;
	BeginADCConversion(); //Start the next series of conversions
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
	ADC0_SC1A = (0x40 | ADC0_ChSelect[ADC0_Select]); //Set up ADC0 for correct channel, Enables Interrupt upon Completion
	ADC1_SC1A = (0x40 | ADC1_ChSelect[ADC1_Select]); //Set up ADC1
}

void ADC_Calibration() {
	SIM_SCGC6 |= 0x8000000; //Enable Module
	ADC0_SC3 = 0xC0; //Begins ADC0 Calibration
	SIM_SCGC3 |= 0x8000000; //Enable Module
	ADC1_SC3 = 0xC0; //Begins ADC0 Calibration
}

void CheckBatteryLevel() {
	RunningMainVoltage = 0; //Reset this to 0
	RunningMotorVoltage = 0; //Reset this to 0
	MainBatteryBad = 0; //Reset this to 0
	MotorBatteryBad = 0; //Reset this to 0

	for (int counter = 0; counter < 6; counter++) { //Loop for the Motor battery
		//Serial.println(ADCData[counter]);
		BatteryCheckData[counter] = ADCData[counter] * ADC_LSB; //Scale it to 3.3V
		//Serial.println(BatteryCheckData[counter]);
		BatteryCheckData[counter] = (BatteryCheckData[counter] / BatteryDivider[counter]) - RunningMotorVoltage; //Calculate the voltage of the cell
		RunningMotorVoltage = BatteryCheckData[counter] + RunningMotorVoltage;
		if (BatteryCheckData[counter] < 3.3) { //Is it below the acceptable level?
			MotorBatteryBad = 1; //Mark it as a fail
		} //End if
	} //End for

	if (MotorBatteryBad) { //If it's bad turn the motor off
		//digitalWrite(MotorEnable, LOW);
		//DEBUG PUT THIS BACK IN
	}

	for (int counter = 6; counter < 9; counter++) { //Loop for the main battery
		//Serial.println(ADCData[counter]);
		BatteryCheckData[counter] = ADCData[counter] * ADC_LSB;
		//Serial.println(BatteryCheckData[counter]);
		BatteryCheckData[counter] = (BatteryCheckData[counter] * BatteryDivider[counter]) - RunningMainVoltage;
		RunningMainVoltage = BatteryCheckData[counter] + RunningMainVoltage;

		if (BatteryCheckData[counter] < 3.3) { //Is it below the acceptable level?
			MainBatteryBad = 1; //Mark it as a fail
		} //End if
	} //End for

	if (~MainBatteryBad) { //Turn on the backup battery and turn off the main battery
		//It is very important that this is done in this order.  You don't want both of the batteries off at the same time.
		digitalWrite(Main_Batt_En, HIGH);
		digitalWrite(Backup_Batt_En, LOW);
	}
	else {
		//It is very important that this is done in this order.  You don't want both of the batteries off at the same time.
		digitalWrite(Backup_Batt_En, HIGH);
		digitalWrite(Main_Batt_En, LOW);
	}

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

void adc0_isr() {
	//In the ISR it gets the data and then begins the next conversion
	switch (ADC0_Select) {
	case 0:
		ADCData[5] = ADC0_RA;
		ADC0_Select++;
		ADC0_SC1A = (0x40 | ADC0_ChSelect[ADC0_Select]); //Set up for correct channel, Enables Interrupt upon Completion
		break;
	case 1:
		ADCData[4] = ADC0_RA;
		ADC0_Select++;
		ADC0_SC1A = (0x40 | ADC0_ChSelect[ADC0_Select]); //Set up for correct channel, Enables Interrupt upon Completion
		break;
	case 2:
		ADCData[7] = ADC0_RA;
		ADC0_Select++;
		break;
	}
}

void adc1_isr() {
	//In the ISR it gets the data and then begins the next conversion
	switch(ADC1_Select) {
	case 0:
		ADCData[3] = ADC1_RA;
		ADC1_Select++;
		ADC1_SC1A = (0x40 | ADC1_ChSelect[ADC1_Select]); //Set up for correct channel, Enables Interrupt upon Completion
		break;
	case 1:
		ADCData[2] = ADC1_RA;
		ADC1_Select++;
		ADC1_SC1A = (0x40 | ADC1_ChSelect[ADC1_Select]); //Set up for correct channel, Enables Interrupt upon Completion
		break;
	case 2:
		ADCData[1] = ADC1_RA;
		ADC1_Select++;
		ADC1_SC1A = (0x40 | ADC1_ChSelect[ADC1_Select]); //Set up for correct channel, Enables Interrupt upon Completion
		break;
	case 3:
		ADCData[0] = ADC1_RA;
		ADC1_Select++;
		ADC1_SC1A = (0x40 | ADC1_ChSelect[ADC1_Select]); //Set up for correct channel, Enables Interrupt upon Completion
		break;
	case 4:
		ADCData[6] = ADC1_RA;
		ADC1_Select++;
		ADC1_SC1A = (0x40 | ADC1_ChSelect[ADC1_Select]); //Set up for correct channel, Enables Interrupt upon Completion
		break;
	case 5:
		ADCData[8] = ADC1_RA;
		ADC1_Select++;
		break;
	}
}