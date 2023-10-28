/*
 * battery.c
 *
 *  Created on: Apr 27, 2017
 *      Author: Lincster
 */
#include <msp430.h>
#include <driverlib.h>
#include "battery.h"
#include "common.h"


uint16_t adc_val;
int adc_software_fg = 0;
int cellNum = 1;

// Error raised if an individual cell is too low for a little while
int low_bat_cell_error = 0;

// Keep track of how many times the average voltage for a cell was too low
int low_cell_count[6]={0};

//*********************************************************************************************************//

void getCellVoltages(double adc_cell_averages[], int first_sample_flag){
    int i = 1;
	double current_cells_divided_voltage;
	double current_cells_voltage;
	double multiple_cell_voltages[7];
	double individual_cell_voltage;
	double individual_cell_average;

	// Have scale factors to reverse voltage dividers
	// Vs=((R1+R2)/R2)xVout where R1 is the upper resistors & R2 is the lower
	double voltage_scale_factors[6] = {10.08,8.44526,6.72193,5.03922,3.36111,1.68};

	// Loop through values for 6 cells, then 5 cells, ... , then 1 cell
	for (i=1; i<7; i++){
		cellNum = i;
		readBattADC(i);
		current_cells_divided_voltage = (double) adc_val/4096*3.3;
		current_cells_voltage = current_cells_divided_voltage*voltage_scale_factors[i-1];
		multiple_cell_voltages[i-1] = current_cells_voltage;
	}
    multiple_cell_voltages[6] = 0.0; // Just have this spot so can use a for loop for every iteration in the next part

	// Determine the values of individual cells and add to adc_cell_data based on sampleNum
	//     First cell is multiple_cell_voltages[0]-multiple_cell_voltages[1] (6-5)
	//     Second cell is multiple_cell_voltages[1]-multiple_cell_voltages[2] (5-4)
	//     Third cell is multiple_cell_voltages[2]-multiple_cell_voltages[3] (4-3)
	//     Fourth cell is multiple_cell_voltages[3]-multiple_cell_voltages[4] (3-2)
	//     Fifth cell is multiple_cell_voltages[4]-multiple_cell_voltages[5] (2-1)
	//     Sixth cell is multiple_cell_voltages[5] (1)
    if (first_sample_flag){ // No averaging first sample since no previous value
        for (i=0; i<6; i++){
        	individual_cell_voltage = multiple_cell_voltages[i]-multiple_cell_voltages[i+1];
        	adc_cell_averages[i] = individual_cell_voltage;
        }
    }
    else{
	    for (i=0; i<6; i++){
		    individual_cell_voltage = multiple_cell_voltages[i]-multiple_cell_voltages[i+1];
			individual_cell_average = EMA_Batt(individual_cell_voltage,adc_cell_averages[i]);

			if (individual_cell_average < CELL_THRESHOLD){ // Increment low cell counter if cell voltage is too low
				low_cell_count[i]++;
				if (low_cell_count[i] >= LOW_CELL_COUNT_THRESHOLD){
					low_bat_cell_error = 1;
				}
			}

			adc_cell_averages[i] = individual_cell_average;
		}
    }

	return;
}

void setupBattADC(void){
	// VBAT_MOD_CELL_1 is for all 6 cells and is pin P3.0_A12_C12
	// VBAT_MOD_CELL_2 is for 5 cells and is pin P3.1_A13_C13
	// VBAT_MOD_CELL_3 is for 4 cells and is pin P3.2_A14_C14
	// VBAT_MOD_CELL_4 is for 3 cells and is pin P7.5_A17
	// VBAT_MOD_CELL_5 is for 2 cells and is pin P7.6_A18
	// VBAT_MOD_CELL_6 is for 1 cell and is pin P7.7_A19

	// Configure P3.0 for ADC, A12
	P3SEL1 |= BIT0;
	P3SEL0 |= BIT0;

	// Configure P3.1 for ADC, A13
    P3SEL1 |= BIT1;
    P3SEL0 |= BIT1;

    // Configure P3.2 for ADC, A14
    P3SEL1 |= BIT2;
    P3SEL0 |= BIT2;

    // Configure P7.5 for ADC, A17
    P7SEL1 |= BIT5;
    P7SEL0 |= BIT5;

    // Configure P7.6 for ADC, A18
    P7SEL1 |= BIT6;
    P7SEL0 |= BIT6;

    // Configure P7.7 for ADC, A19
    P7SEL1 |= BIT7;
    P7SEL0 |= BIT7;

	// Clear and use 16clk cycles, turn ADC on
	ADC12CTL0 = 0;
	ADC12CTL0 |= ADC12SHT0_2 + ADC12ON;

	// Clear and turn on sampling timer
	ADC12CTL1 = 0;
	ADC12CTL1 = ADC12SHP; // Use sampling timer

	// Set 12bit resolution
	ADC12CTL2 = 0;
	ADC12CTL2 |= ADC12RES_2;

	// Clear and setup A12 as input source for MEM12
	ADC12MCTL12 = 0;
	ADC12MCTL12 |= ADC12_B_INPUT_A12;

	// Clear and setup A13 as input source for MEM13
	ADC12MCTL13 = 0;
	ADC12MCTL13 |= ADC12_B_INPUT_A13;

	// Clear and setup A14 as input source for MEM14
	ADC12MCTL14 = 0;
	ADC12MCTL14 |= ADC12_B_INPUT_A14;

	// Clear and setup A17 as input source for MEM17
	ADC12MCTL17 = 0;
	ADC12MCTL17 |= ADC12_B_INPUT_A17;

	// Clear and setup A18 as input source for MEM18
	ADC12MCTL18 = 0;
	ADC12MCTL18 |= ADC12_B_INPUT_A18;

	// Clear and setup A19 as input source for MEM19
	ADC12MCTL19 = 0;
	ADC12MCTL19 |= ADC12_B_INPUT_A19;

    // Enable interrupts for A12
    ADC12IER0 = 0;
    ADC12IER0 |= ADC12IE12;

    // Enable interrutps for A13
    ADC12IER0 |= ADC12IE13;

    // Enable interrupts for A14
    ADC12IER0 |= ADC12IE14;

    // Enable interrupts for A17
    ADC12IER1 = 0;
    ADC12IER1 |= ADC12IE17;

    // Enable interrupts for A18
    ADC12IER1 |= ADC12IE18;

    // Enable interrupts for A19
    ADC12IER1 |= ADC12IE19;

    activate_GPIO_config();
    return;
}

void readBattADC(int adc_mem_reg){
	// Start acquisitons for the MEM for the current battery cell
	ADC12CTL3 = 0;
	switch (adc_mem_reg){
	case 1:
		ADC12CTL3 |= ADC12CSTARTADD_12; // MEM12
		break;
	case 2:
		ADC12CTL3 |= ADC12CSTARTADD_13; // MEM13
		break;
	case 3:
		ADC12CTL3 |= ADC12CSTARTADD_14; // MEM14
		break;
	case 4:
		ADC12CTL3 |= ADC12CSTARTADD_17; // MEM17
		break;
	case 5:
		ADC12CTL3 |= ADC12CSTARTADD_18; // MEM18
		break;
	case 6:
		ADC12CTL3 |= ADC12CSTARTADD_19; // MEM19
		break;
	default:
		break;
	}

	// Enable ADC and start acquisition
   ADC12CTL0 |= ADC12ENC;
   ADC12CTL0 |= ADC12SC;

   //enable general interrupts
   __bis_SR_register(LPM0_bits | GIE);

   //wait for the adc software flag to be set
   while(adc_software_fg == 0);

   //disable the ADC
   ADC12CTL0 &= ~ADC12ENC;

   //clear the adc software flag
   adc_software_fg = 0;

   return;
}
// Exponential moving average
// https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
double EMA_Batt(double new_sample, double ma_old){
	// A lower alpha discounts older observations faster
	double alpha = 0.3;
	return alpha * new_sample + (1-alpha) * ma_old; // also have seen online: newValue = oldValue + alpha * (value - oldValue)
}
