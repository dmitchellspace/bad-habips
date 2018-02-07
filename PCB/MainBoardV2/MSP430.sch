EESchema Schematic File Version 2
LIBS:mainBoardV2-rescue
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:msp430fr4994
LIBS:asemb-24mhz-xy-t
LIBS:pct2075d,118
LIBS:ms560702ba03-50
LIBS:lsm9ds1
LIBS:adg704brmz
LIBS:lt3481
LIBS:az2117h
LIBS:lt8609
LIBS:ltc2992
LIBS:TVS_Small
LIBS:tvsSmall
LIBS:mmqa5v6t1g
LIBS:led_x2_3pin
LIBS:bme280
LIBS:aoz8318di
LIBS:switch_dpdt
LIBS:SWITCH_3PDT
LIBS:teensy3_6-OuterHeadersOnly
LIBS:mounting_hole_eighth_inch
LIBS:mainBoardV2-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 6 6
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MSP430FR4994 U8
U 1 1 5A00F820
P 4500 3700
F 0 "U8" H 4500 6450 60  0000 C CNN
F 1 "MSP430FR5994" H 4500 1050 60  0000 C CNN
F 2 "Housings_QFP:LQFP-80_12x12mm_Pitch0.5mm" H 4500 3450 60  0001 C CNN
F 3 "" H 4500 3450 60  0001 C CNN
	1    4500 3700
	1    0    0    -1  
$EndComp
$Comp
L R R49
U 1 1 5A00F82D
P 1050 1100
F 0 "R49" V 1130 1100 50  0000 C CNN
F 1 "10" V 1050 1100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 980 1100 50  0001 C CNN
F 3 "" H 1050 1100 50  0000 C CNN
	1    1050 1100
	0    1    1    0   
$EndComp
$Comp
L C C23
U 1 1 5A00F834
P 1300 1250
F 0 "C23" H 1325 1350 50  0000 L CNN
F 1 "100nF" H 1325 1150 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1338 1100 50  0001 C CNN
F 3 "" H 1300 1250 50  0000 C CNN
	1    1300 1250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR79
U 1 1 5A00F83B
P 1300 1400
F 0 "#PWR79" H 1300 1150 50  0001 C CNN
F 1 "GND" H 1300 1250 50  0000 C CNN
F 2 "" H 1300 1400 50  0000 C CNN
F 3 "" H 1300 1400 50  0000 C CNN
	1    1300 1400
	1    0    0    -1  
$EndComp
$Comp
L R R50
U 1 1 5A00F841
P 1800 1100
F 0 "R50" V 1880 1100 50  0000 C CNN
F 1 "0" V 1800 1100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1730 1100 50  0001 C CNN
F 3 "" H 1800 1100 50  0000 C CNN
	1    1800 1100
	0    1    1    0   
$EndComp
Text GLabel 2050 1900 0    39   Input ~ 0
MSP_LED_GREEN
Text GLabel 2050 2000 0    39   Input ~ 0
MSP_LED_BLUE
Text GLabel 2050 3700 0    39   Input ~ 0
COMMS_MOSI
Text GLabel 2050 3800 0    39   Input ~ 0
COMMS_MISO
Text GLabel 2050 3900 0    39   Input ~ 0
COMMS_SCLK
Text GLabel 2050 4000 0    39   Input ~ 0
COMMS_CS
Text GLabel 2050 2200 0    39   Input ~ 0
DAQCS_CS
Text GLabel 2050 2300 0    39   Input ~ 0
MULTIPLEX_A0
Text GLabel 2050 2400 0    39   Input ~ 0
MULTIPLEX_A1
Text GLabel 2050 2500 0    39   Input ~ 0
DAQCS_MOSI
Text GLabel 2050 2600 0    39   Input ~ 0
DAQCS_MISO
Text GLabel 2050 4100 0    39   Input ~ 0
UART_3_TX
Text GLabel 2050 4200 0    39   Input ~ 0
UART_3_RX
Text GLabel 2050 4600 0    39   Input ~ 0
HOST_SDA
Text GLabel 2050 4700 0    39   Input ~ 0
HOST_SCL
$Comp
L Crystal Y1
U 1 1 5A00F864
P 9450 1400
F 0 "Y1" H 9450 1550 50  0000 C CNN
F 1 "32.768 kHz" H 9450 1250 50  0000 C CNN
F 2 "Dans_Parts:32.768_Crystal_ThroughHole" H 9450 1400 50  0001 C CNN
F 3 "" H 9450 1400 50  0000 C CNN
	1    9450 1400
	-1   0    0    -1  
$EndComp
$Comp
L C C31
U 1 1 5A00F86B
P 9800 1600
F 0 "C31" H 9825 1700 50  0000 L CNN
F 1 "22pF" H 9825 1500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9838 1450 50  0001 C CNN
F 3 "" H 9800 1600 50  0000 C CNN
	1    9800 1600
	1    0    0    -1  
$EndComp
$Comp
L C C30
U 1 1 5A00F872
P 9100 1600
F 0 "C30" H 9000 1700 50  0000 L CNN
F 1 "22pF" H 8850 1500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9138 1450 50  0001 C CNN
F 3 "" H 9100 1600 50  0000 C CNN
	1    9100 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR85
U 1 1 5A00F879
P 9450 1800
F 0 "#PWR85" H 9450 1550 50  0001 C CNN
F 1 "GND" H 9450 1650 50  0000 C CNN
F 2 "" H 9450 1800 50  0000 C CNN
F 3 "" H 9450 1800 50  0000 C CNN
	1    9450 1800
	1    0    0    -1  
$EndComp
NoConn ~ 2050 5700
NoConn ~ 2050 5800
NoConn ~ 2050 5600
NoConn ~ 2050 5500
NoConn ~ 2050 6200
Text GLabel 2050 6100 0    39   Input ~ 0
HF_IN_1
Text GLabel 2050 5900 0    39   Input ~ 0
LF_IN
Text GLabel 2050 6000 0    39   Input ~ 0
LF_OUT
Text GLabel 8950 1400 0    39   Input ~ 0
LF_IN
Text GLabel 9950 1400 2    39   Input ~ 0
LF_OUT
Text GLabel 6950 1100 2    39   Input ~ 0
UART_1_TX
Text GLabel 6950 1200 2    39   Input ~ 0
UART_1_RX
Text GLabel 6950 1300 2    39   Input ~ 0
DAQCS_SCLK
Text GLabel 6950 1600 2    39   Input ~ 0
UART_2_TX
Text GLabel 6950 2000 2    39   Input ~ 0
CUTDOWN_EN
Text GLabel 6950 2400 2    39   Input ~ 0
RESET_PI_1
Text GLabel 6950 2500 2    39   Input ~ 0
RESET_PI_2
Text GLabel 6950 2600 2    39   Input ~ 0
RESET_PI_3
Text GLabel 6950 2700 2    39   Input ~ 0
RESET_PI_4
Text GLabel 6950 2900 2    39   Input ~ 0
UART_4_TX
Text GLabel 6950 3000 2    39   Input ~ 0
UART_4_RX
Text GLabel 6950 3800 2    39   Input ~ 0
MSP_GPIO_0
Text GLabel 6950 3900 2    39   Input ~ 0
MSP_GPIO_1
Text GLabel 6950 4000 2    39   Input ~ 0
MSP_GPIO_2
Text GLabel 6950 4100 2    39   Input ~ 0
MSP_GPIO_3
Text GLabel 6950 4450 2    39   Input ~ 0
SBWTCK_HOST
Text GLabel 6950 4800 2    39   Input ~ 0
RST_SBWTDIO_HOST
$Comp
L GND #PWR83
U 1 1 5A00F8A8
P 7050 6250
F 0 "#PWR83" H 7050 6000 50  0001 C CNN
F 1 "GND" H 7050 6100 50  0000 C CNN
F 2 "" H 7050 6250 50  0000 C CNN
F 3 "" H 7050 6250 50  0000 C CNN
	1    7050 6250
	1    0    0    -1  
$EndComp
Text Notes 1050 950  0    59   ~ 0
Layout: Place decoupling capacitor \nas close to pins as possible
Text Notes 2950 850  0    197  ~ 0
Host MSP430 Pinout
$Comp
L C C25
U 1 1 5A00F8DB
P 4900 7200
F 0 "C25" H 4925 7300 50  0000 L CNN
F 1 "1uF" H 4925 7100 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4938 7050 50  0001 C CNN
F 3 "" H 4900 7200 50  0000 C CNN
	1    4900 7200
	1    0    0    -1  
$EndComp
$Comp
L C C26
U 1 1 5A00F8E2
P 5300 7200
F 0 "C26" H 5325 7300 50  0000 L CNN
F 1 "10uF" H 5325 7100 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5338 7050 50  0001 C CNN
F 3 "" H 5300 7200 50  0000 C CNN
	1    5300 7200
	1    0    0    -1  
$EndComp
$Comp
L C C27
U 1 1 5A00F8E9
P 5700 7200
F 0 "C27" H 5725 7300 50  0000 L CNN
F 1 "100nF" H 5725 7100 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5738 7050 50  0001 C CNN
F 3 "" H 5700 7200 50  0000 C CNN
	1    5700 7200
	1    0    0    -1  
$EndComp
$Comp
L C C28
U 1 1 5A00F8F0
P 6100 7200
F 0 "C28" H 6125 7300 50  0000 L CNN
F 1 "100nF" H 6125 7100 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 6138 7050 50  0001 C CNN
F 3 "" H 6100 7200 50  0000 C CNN
	1    6100 7200
	1    0    0    -1  
$EndComp
$Comp
L C C29
U 1 1 5A00F8F7
P 6500 7200
F 0 "C29" H 6525 7300 50  0000 L CNN
F 1 "100nF" H 6525 7100 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 6538 7050 50  0001 C CNN
F 3 "" H 6500 7200 50  0000 C CNN
	1    6500 7200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR82
U 1 1 5A00F8FE
P 5700 7450
F 0 "#PWR82" H 5700 7200 50  0001 C CNN
F 1 "GND" H 5700 7300 50  0000 C CNN
F 2 "" H 5700 7450 50  0000 C CNN
F 3 "" H 5700 7450 50  0000 C CNN
	1    5700 7450
	1    0    0    -1  
$EndComp
Text Notes 4800 6850 0    197  ~ 0
DC Filtering
Text Notes 7900 1150 0    197  ~ 0
Low-Freq. Oscillator
Text Notes 7800 2450 0    197  ~ 0
High-Freq. Oscillator
$Comp
L ASEMB-24MHZ-XY-T U9
U 1 1 5A00F90A
P 9350 2850
F 0 "U9" H 9350 3100 60  0000 C CNN
F 1 "ASEMB-24MHZ-XY-T" H 9400 2600 60  0000 C CNN
F 2 "Dans_Parts:ASEMB-24MHZ" H 9350 2850 60  0001 C CNN
F 3 "" H 9350 2850 60  0001 C CNN
	1    9350 2850
	1    0    0    -1  
$EndComp
$Comp
L R R59
U 1 1 5A00F911
P 10250 2950
F 0 "R59" V 10330 2950 50  0000 C CNN
F 1 "10" V 10250 2950 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 10180 2950 50  0001 C CNN
F 3 "" H 10250 2950 50  0000 C CNN
	1    10250 2950
	0    1    1    0   
$EndComp
$Comp
L C C32
U 1 1 5A00F918
P 10000 3200
F 0 "C32" H 10025 3300 50  0000 L CNN
F 1 "10000pF" H 10025 3100 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 10038 3050 50  0001 C CNN
F 3 "" H 10000 3200 50  0000 C CNN
	1    10000 3200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR86
U 1 1 5A00F91F
P 10000 3350
F 0 "#PWR86" H 10000 3100 50  0001 C CNN
F 1 "GND" H 10000 3200 50  0000 C CNN
F 2 "" H 10000 3350 50  0000 C CNN
F 3 "" H 10000 3350 50  0000 C CNN
	1    10000 3350
	1    0    0    -1  
$EndComp
NoConn ~ 8800 2750
$Comp
L GND #PWR84
U 1 1 5A00F92C
P 8700 3050
F 0 "#PWR84" H 8700 2800 50  0001 C CNN
F 1 "GND" H 8700 2900 50  0000 C CNN
F 2 "" H 8700 3050 50  0000 C CNN
F 3 "" H 8700 3050 50  0000 C CNN
	1    8700 3050
	1    0    0    -1  
$EndComp
Text GLabel 9900 2750 2    39   Input ~ 0
HF_IN_1
Wire Wire Line
	1550 1100 1550 1700
Connection ~ 1550 1100
Connection ~ 1550 1300
Connection ~ 1550 1500
Connection ~ 1300 1100
Wire Wire Line
	2050 1100 1950 1100
Wire Wire Line
	1550 1700 2050 1700
Wire Wire Line
	1550 1500 2050 1500
Wire Wire Line
	1550 1300 2050 1300
Wire Wire Line
	8950 1400 9300 1400
Wire Wire Line
	9800 1450 9800 1400
Wire Wire Line
	9100 1750 9100 1800
Wire Wire Line
	9800 1800 9800 1750
Connection ~ 9800 1400
Connection ~ 9100 1400
Wire Wire Line
	9600 1400 9950 1400
Wire Wire Line
	9100 1450 9100 1400
Wire Wire Line
	6950 5200 7050 5200
Wire Wire Line
	7050 5200 7050 6250
Wire Wire Line
	6950 6000 7050 6000
Connection ~ 7050 6000
Wire Wire Line
	6950 5800 7050 5800
Connection ~ 7050 5800
Wire Wire Line
	6950 5600 7050 5600
Connection ~ 7050 5600
Wire Wire Line
	6950 5400 7050 5400
Connection ~ 7050 5400
Connection ~ 7050 6200
Wire Wire Line
	7050 6200 6950 6200
Wire Notes Line
	7800 850  11050 850 
Wire Notes Line
	7800 2050 7800 850 
Wire Notes Line
	550  550  7600 550 
Wire Notes Line
	7600 550  7600 6450
Wire Notes Line
	7600 6450 550  6450
Wire Notes Line
	550  6450 550  550 
Wire Wire Line
	6500 6950 6500 7050
Wire Wire Line
	6100 6950 6100 7050
Connection ~ 6100 6950
Wire Wire Line
	5700 6950 5700 7050
Connection ~ 5700 6950
Wire Wire Line
	5300 6950 5300 7050
Connection ~ 5300 6950
Wire Wire Line
	4900 7050 4900 6950
Connection ~ 4900 6950
Wire Wire Line
	4900 7350 4900 7450
Wire Wire Line
	4900 7450 6500 7450
Wire Wire Line
	6500 7450 6500 7350
Wire Wire Line
	6100 7350 6100 7450
Connection ~ 6100 7450
Wire Wire Line
	5700 7350 5700 7450
Connection ~ 5700 7450
Wire Wire Line
	5300 7350 5300 7450
Connection ~ 5300 7450
Wire Notes Line
	4550 6550 6800 6550
Wire Notes Line
	11050 850  11050 2050
Wire Notes Line
	11050 2050 7800 2050
Wire Notes Line
	7800 2150 11050 2150
Wire Notes Line
	11050 2150 11050 3600
Wire Notes Line
	11050 3600 7800 3600
Wire Notes Line
	7800 3600 7800 2150
Wire Wire Line
	9900 2950 10100 2950
Wire Wire Line
	10000 3050 10000 2950
Connection ~ 10000 2950
Wire Wire Line
	8700 3050 8700 2950
Wire Wire Line
	8700 2950 8800 2950
Wire Notes Line
	4550 6550 4550 7700
Wire Notes Line
	4550 7700 6800 7700
Wire Notes Line
	6800 7700 6800 6550
$Comp
L R R51
U 1 1 5A0AA561
P 2350 7100
F 0 "R51" V 2430 7100 50  0000 C CNN
F 1 "10k" V 2350 7100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2280 7100 50  0001 C CNN
F 3 "" H 2350 7100 50  0000 C CNN
	1    2350 7100
	-1   0    0    1   
$EndComp
$Comp
L R R52
U 1 1 5A0AA729
P 2750 7100
F 0 "R52" V 2830 7100 50  0000 C CNN
F 1 "10k" V 2750 7100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2680 7100 50  0001 C CNN
F 3 "" H 2750 7100 50  0000 C CNN
	1    2750 7100
	-1   0    0    1   
$EndComp
Wire Wire Line
	2350 6950 2750 6950
Wire Wire Line
	2550 6850 2550 6950
Connection ~ 2550 6950
Text GLabel 2150 7300 0    39   Input ~ 0
HOST_SCL
Wire Wire Line
	2150 7300 2350 7300
Wire Wire Line
	2350 7300 2350 7250
Text GLabel 2150 7400 0    39   Input ~ 0
HOST_SDA
Wire Wire Line
	2150 7400 2750 7400
Wire Wire Line
	2750 7400 2750 7250
Text Notes 600  7100 0    197  ~ 0
I2C Pullup
Wire Notes Line
	550  6700 2850 6700
Wire Notes Line
	2850 6700 2850 7500
Wire Notes Line
	2850 7500 550  7500
Wire Notes Line
	550  7500 550  6700
NoConn ~ 6950 2100
$Comp
L R R53
U 1 1 5A376B38
P 3800 7000
F 0 "R53" V 3700 7000 50  0000 C CNN
F 1 "10k" V 3800 7000 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3730 7000 50  0001 C CNN
F 3 "" H 3800 7000 50  0000 C CNN
	1    3800 7000
	0    1    1    0   
$EndComp
$Comp
L SW_PUSH_SMALL_H SW1
U 1 1 5A376B45
P 3300 7150
F 0 "SW1" H 3100 6650 50  0000 C CNN
F 1 "PTS645SM43SMTR92 LFS" H 3750 6650 50  0000 C CNN
F 2 "Dans_Parts:SWITCH-TACTILE-PTS645SM43SMTR92" H 3300 7350 50  0001 C CNN
F 3 "" H 3300 7350 50  0000 C CNN
	1    3300 7150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR80
U 1 1 5A376B4C
P 3100 7200
F 0 "#PWR80" H 3100 6950 50  0001 C CNN
F 1 "GND" H 3100 7050 50  0000 C CNN
F 2 "" H 3100 7200 50  0000 C CNN
F 3 "" H 3100 7200 50  0000 C CNN
	1    3100 7200
	1    0    0    -1  
$EndComp
Text GLabel 3650 7150 2    39   Input ~ 0
RST_SBWTDIO_HOST
Text Notes 3150 6850 0    197  ~ 0
Reset
$Comp
L C C24
U 1 1 5A376B54
P 3550 7300
F 0 "C24" H 3575 7400 50  0000 L CNN
F 1 "1000pF" H 3200 7400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3588 7150 50  0001 C CNN
F 3 "" H 3550 7300 50  0000 C CNN
	1    3550 7300
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR81
U 1 1 5A376B5B
P 3550 7450
F 0 "#PWR81" H 3550 7200 50  0001 C CNN
F 1 "GND" H 3700 7400 50  0000 C CNN
F 2 "" H 3550 7450 50  0000 C CNN
F 3 "" H 3550 7450 50  0000 C CNN
	1    3550 7450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 7150 3100 7150
Wire Wire Line
	3100 7150 3100 7200
Wire Notes Line
	2950 6550 4300 6550
Wire Notes Line
	4300 6550 4300 7750
Wire Notes Line
	4300 7750 2950 7750
Wire Notes Line
	2950 7750 2950 6550
Wire Wire Line
	3450 7150 3650 7150
Connection ~ 3550 7150
Wire Wire Line
	3650 7000 3550 7000
Text GLabel 4800 6950 0    39   Input ~ 0
+3.3V
Wire Wire Line
	4800 6950 6500 6950
Text GLabel 6950 1700 2    39   Input ~ 0
UART_2_RX
NoConn ~ 6950 1400
NoConn ~ 6950 1500
NoConn ~ 6950 1800
NoConn ~ 6950 2200
NoConn ~ 6950 2300
NoConn ~ 6950 3100
NoConn ~ 6950 3200
NoConn ~ 6950 3300
NoConn ~ 6950 3400
NoConn ~ 6950 3500
NoConn ~ 6950 3600
NoConn ~ 2050 5300
NoConn ~ 2050 5200
NoConn ~ 2050 5100
NoConn ~ 2050 4800
NoConn ~ 2050 4900
NoConn ~ 2050 5000
NoConn ~ 2050 4400
NoConn ~ 2050 2800
NoConn ~ 2050 2900
NoConn ~ 2050 3000
NoConn ~ 2050 3100
NoConn ~ 2050 3200
NoConn ~ 2050 3300
NoConn ~ 2050 3400
NoConn ~ 2050 3500
Text GLabel 10400 2950 2    39   Input ~ 0
+3.3V
Text GLabel 3950 7000 2    39   Input ~ 0
+3.3V
Text GLabel 2500 6850 0    39   Input ~ 0
+3.3V
Wire Wire Line
	2500 6850 2550 6850
Text GLabel 850  1100 0    39   Input ~ 0
+3.3V
Wire Wire Line
	850  1100 900  1100
Wire Wire Line
	1200 1100 1650 1100
Wire Notes Line
	7800 3850 7800 5700
Wire Notes Line
	7800 5700 11050 5700
Wire Notes Line
	11050 5700 11050 3850
Text GLabel 9900 5000 0    39   Input ~ 0
MSP_LED_GREEN
Text GLabel 9900 4400 0    39   Input ~ 0
MSP_LED_RED
$Comp
L LED D31
U 1 1 5A7577E6
P 9300 4100
F 0 "D31" H 9350 4000 50  0000 C CNN
F 1 "LTST-C171KRKT" H 9400 4200 50  0000 C CNN
F 2 "LEDs:LED_0805_HandSoldering" H 9300 4100 50  0001 C CNN
F 3 "" H 9300 4100 50  0000 C CNN
	1    9300 4100
	-1   0    0    1   
$EndComp
Wire Notes Line
	11050 3850 7800 3850
Text Notes 8000 4850 0    197  ~ 0
LEDs
Text GLabel 9100 4700 0    39   Input ~ 0
+5V
Text GLabel 9900 5550 0    39   Input ~ 0
MSP_LED_BLUE
$Comp
L Q_NMOS_GSD Q15
U 1 1 5A75781A
P 10000 5350
F 0 "Q15" V 10200 5450 50  0000 R CNN
F 1 "NTR4003NT3G" V 9900 5150 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 10200 5450 50  0001 C CNN
F 3 "" H 10000 5350 50  0000 C CNN
	1    10000 5350
	0    -1   -1   0   
$EndComp
Text GLabel 9100 5250 0    39   Input ~ 0
+5V
$Comp
L Q_NMOS_GSD Q14
U 1 1 5A757855
P 10000 4200
F 0 "Q14" V 10200 4300 50  0000 R CNN
F 1 "NTR4003NT3G" V 9950 4050 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 10200 4300 50  0001 C CNN
F 3 "" H 10000 4200 50  0000 C CNN
	1    10000 4200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9900 4400 10100 4400
Text GLabel 10200 4100 2    39   Input ~ 0
GND
$Comp
L R R57
U 1 1 5A75785E
P 9650 4100
F 0 "R57" V 9550 4100 50  0000 C CNN
F 1 "150" V 9650 4100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9580 4100 50  0001 C CNN
F 3 "" H 9650 4100 50  0000 C CNN
	1    9650 4100
	0    1    1    0   
$EndComp
Text GLabel 9100 4100 0    39   Input ~ 0
+5V
Text GLabel 2050 2100 0    39   Input ~ 0
MSP_LED_RED
Wire Wire Line
	3550 7000 3550 7150
Text GLabel 2050 4300 0    39   Input ~ 0
RESET_VIDEO_PI
Wire Wire Line
	9100 1800 9800 1800
Connection ~ 9450 1800
$Comp
L Q_NMOS_GSD Q12
U 1 1 5A7577EF
P 10000 4800
F 0 "Q12" V 10200 4850 50  0000 R CNN
F 1 "NTR4003NT3G" V 9950 4650 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 10200 4900 50  0001 C CNN
F 3 "" H 10000 4800 50  0000 C CNN
	1    10000 4800
	0    -1   -1   0   
$EndComp
$Comp
L LED D7
U 1 1 5A76FE62
P 9300 4700
F 0 "D7" H 9350 4600 50  0000 C CNN
F 1 "APT2012LZGCK" H 9400 4800 50  0000 C CNN
F 2 "LEDs:LED_0805_HandSoldering" H 9300 4700 50  0001 C CNN
F 3 "" H 9300 4700 50  0000 C CNN
	1    9300 4700
	-1   0    0    1   
$EndComp
$Comp
L R R54
U 1 1 5A770570
P 9650 4700
F 0 "R54" V 9550 4700 50  0000 C CNN
F 1 "1.2k" V 9650 4700 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9580 4700 50  0001 C CNN
F 3 "" H 9650 4700 50  0000 C CNN
	1    9650 4700
	0    1    1    0   
$EndComp
Wire Wire Line
	9900 5000 10000 5000
Text GLabel 10200 4700 2    39   Input ~ 0
GND
Text GLabel 10200 5250 2    39   Input ~ 0
GND
Wire Wire Line
	9900 5550 10000 5550
$Comp
L R R55
U 1 1 5A771691
P 9650 5250
F 0 "R55" V 9550 5250 50  0000 C CNN
F 1 "80" V 9650 5250 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9580 5250 50  0001 C CNN
F 3 "" H 9650 5250 50  0000 C CNN
	1    9650 5250
	0    1    1    0   
$EndComp
$Comp
L LED D8
U 1 1 5A771799
P 9300 5250
F 0 "D8" H 9350 5150 50  0000 C CNN
F 1 "HSMR-C170" H 9350 5350 50  0000 C CNN
F 2 "LEDs:LED_0805_HandSoldering" H 9300 5250 50  0001 C CNN
F 3 "" H 9300 5250 50  0000 C CNN
	1    9300 5250
	-1   0    0    1   
$EndComp
$Comp
L R R46
U 1 1 5A782298
P 10250 4400
F 0 "R46" V 10330 4400 50  0000 C CNN
F 1 "10k" V 10250 4400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 10180 4400 50  0001 C CNN
F 3 "" H 10250 4400 50  0000 C CNN
	1    10250 4400
	0    1    1    0   
$EndComp
Text GLabel 10500 4400 2    39   Input ~ 0
+5V
Wire Wire Line
	10500 4400 10400 4400
Connection ~ 10000 4400
$EndSCHEMATC
