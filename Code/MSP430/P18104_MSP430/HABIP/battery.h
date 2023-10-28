/*
 * battery.h
 *
 *  Created on: Apr 27, 2017
 *      Author: Lincster
 */

#ifndef HABIP_BATTERY_H_
#define HABIP_BATTERY_H_

void setupBattADC(void);
void readBattADC(int adc_mem_reg);
void getCellVoltages(double adc_cell_averages[], int first_sample_flag);
double EMA_Batt(double new_sample, double ma_old);

#define CELL_THRESHOLD 3.2
#define LOW_CELL_COUNT_THRESHOLD 20

#endif /* HABIP_BATTERY_H_ */
