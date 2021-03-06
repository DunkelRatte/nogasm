EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
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
L teensy:Teensy-LC U14
U 1 1 591740E4
P 3200 2700
F 0 "U14" H 3200 3850 60  0000 C CNN
F 1 "Teensy-LC" H 3200 1550 60  0000 C CNN
F 2 "nogasm:Teensy30_31_32_LC" H 3000 1600 60  0000 C CNN
F 3 "" H 3200 2150 60  0000 C CNN
	1    3200 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 59175358
P 5300 2500
F 0 "C6" H 5325 2600 50  0000 L CNN
F 1 "1uF" H 5325 2400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5338 2350 50  0001 C CNN
F 3 "" H 5300 2500 50  0000 C CNN
	1    5300 2500
	-1   0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR01
U 1 1 591757F7
P 4750 2800
F 0 "#PWR01" H 4750 2650 50  0001 C CNN
F 1 "+3.3V" H 4750 2940 50  0000 C CNN
F 2 "" H 4750 2800 50  0000 C CNN
F 3 "" H 4750 2800 50  0000 C CNN
	1    4750 2800
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR02
U 1 1 59175932
P 5550 2100
F 0 "#PWR02" H 5550 1950 50  0001 C CNN
F 1 "+5V" H 5550 2240 50  0000 C CNN
F 2 "" H 5550 2100 50  0000 C CNN
F 3 "" H 5550 2100 50  0000 C CNN
	1    5550 2100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 59175996
P 5550 2850
F 0 "#PWR03" H 5550 2600 50  0001 C CNN
F 1 "GND" H 5550 2700 50  0000 C CNN
F 2 "" H 5550 2850 50  0000 C CNN
F 3 "" H 5550 2850 50  0000 C CNN
	1    5550 2850
	1    0    0    -1  
$EndComp
Text Label 1400 3150 0    60   ~ 0
5VIO
Text Label 5100 3550 2    60   ~ 0
V_PRES
Text Label 1550 2150 0    60   ~ 0
ENC_Bl
Text Label 1550 2250 0    60   ~ 0
ENC_Gr
Text Label 1550 2350 0    60   ~ 0
ENC_Rd
Text Label 1550 2450 0    60   ~ 0
ENC_SW
Text Label 1550 2550 0    60   ~ 0
ENC_A
Text Label 1550 2650 0    60   ~ 0
ENC_B
$Comp
L Switch:SW_DIP_x04 SW1
U 1 1 59176017
P 1050 2950
F 0 "SW1" H 1050 3300 50  0000 C CNN
F 1 "SW_DIP_x04" H 1050 2700 50  0000 C CNN
F 2 "nogasm:SW_DIP_4_SMD" H 1050 2950 50  0001 C CNN
F 3 "" H 1050 2950 50  0001 C CNN
	1    1050 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 591763E6
P 600 3400
F 0 "#PWR04" H 600 3150 50  0001 C CNN
F 1 "GND" H 600 3250 50  0000 C CNN
F 2 "" H 600 3400 50  0000 C CNN
F 3 "" H 600 3400 50  0000 C CNN
	1    600  3400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 59176653
P 6000 3100
F 0 "R2" V 6080 3100 50  0000 C CNN
F 1 "750" V 6000 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5930 3100 50  0001 C CNN
F 3 "" H 6000 3100 50  0000 C CNN
	1    6000 3100
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 591766C6
P 5750 3350
F 0 "R1" V 5830 3350 50  0000 C CNN
F 1 "10k" V 5750 3350 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5680 3350 50  0001 C CNN
F 3 "" H 5750 3350 50  0000 C CNN
	1    5750 3350
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR05
U 1 1 59176A06
P 6950 3650
F 0 "#PWR05" H 6950 3400 50  0001 C CNN
F 1 "GND" H 6950 3500 50  0000 C CNN
F 2 "" H 6950 3650 50  0000 C CNN
F 3 "" H 6950 3650 50  0000 C CNN
	1    6950 3650
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR06
U 1 1 59176BC4
P 6900 2150
F 0 "#PWR06" H 6900 2000 50  0001 C CNN
F 1 "+12V" H 6900 2290 50  0000 C CNN
F 2 "" H 6900 2150 50  0000 C CNN
F 3 "" H 6900 2150 50  0000 C CNN
	1    6900 2150
	1    0    0    -1  
$EndComp
$Comp
L Connector:Barrel_Jack J1
U 1 1 59176E53
P 6400 2350
F 0 "J1" H 6400 2560 50  0000 C CNN
F 1 "DC Jack PJ-031DH 3.5mm" H 6400 2175 50  0000 C CNN
F 2 "nogasm:Barreljack_3mm" H 6450 2310 50  0001 C CNN
F 3 "" H 6450 2310 50  0001 C CNN
	1    6400 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:D D1
U 1 1 59176F85
P 7550 2500
F 0 "D1" H 7550 2600 50  0000 C CNN
F 1 "MRA4004T3G" H 7550 2700 50  0000 C CNN
F 2 "nogasm:D_SMA_0202" H 7550 2500 50  0001 C CNN
F 3 "" H 7550 2500 50  0001 C CNN
	1    7550 2500
	0    1    1    0   
$EndComp
$Comp
L Connector:Barrel_Jack J2
U 1 1 59177E2D
P 6500 950
F 0 "J2" H 6500 1160 50  0000 C CNN
F 1 "DC Jack PRT-10811" H 6500 775 50  0000 C CNN
F 2 "nogasm:Barreljack_5.4mm" H 6550 910 50  0001 C CNN
F 3 "" H 6550 910 50  0001 C CNN
	1    6500 950 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5917804D
P 7100 1100
F 0 "#PWR07" H 7100 850 50  0001 C CNN
F 1 "GND" H 7100 950 50  0000 C CNN
F 2 "" H 7100 1100 50  0001 C CNN
F 3 "" H 7100 1100 50  0001 C CNN
	1    7100 1100
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR08
U 1 1 591780C9
P 7100 800
F 0 "#PWR08" H 7100 650 50  0001 C CNN
F 1 "+12V" H 7100 940 50  0000 C CNN
F 2 "" H 7100 800 50  0001 C CNN
F 3 "" H 7100 800 50  0001 C CNN
	1    7100 800 
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C13
U 1 1 5917813D
P 7800 1050
F 0 "C13" H 7825 1150 50  0000 L CNN
F 1 "10uF" H 7825 950 50  0000 L CNN
F 2 "Capacitors_SMD:C_1210_HandSoldering" H 7838 900 50  0001 C CNN
F 3 "" H 7800 1050 50  0001 C CNN
	1    7800 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 591881A1
P 8500 1450
F 0 "#PWR09" H 8500 1200 50  0001 C CNN
F 1 "GND" H 8500 1300 50  0000 C CNN
F 2 "" H 8500 1450 50  0001 C CNN
F 3 "" H 8500 1450 50  0001 C CNN
	1    8500 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR011
U 1 1 5918878B
P 8150 2000
F 0 "#PWR011" H 8150 1850 50  0001 C CNN
F 1 "+3.3V" H 8150 2140 50  0000 C CNN
F 2 "" H 8150 2000 50  0001 C CNN
F 3 "" H 8150 2000 50  0001 C CNN
	1    8150 2000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C15
U 1 1 59188822
P 8150 2550
F 0 "C15" H 8175 2650 50  0000 L CNN
F 1 "1uF" H 8175 2450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 8188 2400 50  0001 C CNN
F 3 "" H 8150 2550 50  0001 C CNN
	1    8150 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C17
U 1 1 59188A47
P 8500 2550
F 0 "C17" H 8525 2650 50  0000 L CNN
F 1 "0.01uF" H 8525 2450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 8538 2400 50  0001 C CNN
F 3 "" H 8500 2550 50  0001 C CNN
	1    8500 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 59188C8F
P 8150 3100
F 0 "#PWR012" H 8150 2850 50  0001 C CNN
F 1 "GND" H 8150 2950 50  0000 C CNN
F 2 "" H 8150 3100 50  0001 C CNN
F 3 "" H 8150 3100 50  0001 C CNN
	1    8150 3100
	1    0    0    -1  
$EndComp
$Comp
L nogasm:MP3V5050GP U16
U 1 1 59189421
P 9950 2450
F 0 "U16" H 9950 2650 60  0000 C CNN
F 1 "MP3V5050GP" H 9950 2400 60  0000 C CNN
F 2 "nogasm:MP3V5050GP" H 9950 2650 60  0001 C CNN
F 3 "" H 9950 2650 60  0001 C CNN
	1    9950 2450
	-1   0    0    1   
$EndComp
$Comp
L Device:R R3
U 1 1 591896D0
P 9450 3550
F 0 "R3" V 9530 3550 50  0000 C CNN
F 1 "750" V 9450 3550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9380 3550 50  0001 C CNN
F 3 "" H 9450 3550 50  0001 C CNN
	1    9450 3550
	0    1    1    0   
$EndComp
$Comp
L Device:C C21
U 1 1 59189786
P 9850 3700
F 0 "C21" H 9875 3800 50  0000 L CNN
F 1 "0.33uF" H 9875 3600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9888 3550 50  0001 C CNN
F 3 "" H 9850 3700 50  0001 C CNN
	1    9850 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 59189B24
P 9850 3950
F 0 "#PWR013" H 9850 3700 50  0001 C CNN
F 1 "GND" H 9850 3800 50  0000 C CNN
F 2 "" H 9850 3950 50  0001 C CNN
F 3 "" H 9850 3950 50  0001 C CNN
	1    9850 3950
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:MCP6001-OT U17
U 1 1 59189D24
P 10750 3450
F 0 "U17" H 10800 3650 50  0000 C CNN
F 1 "MCP6001 RT-I/OT" H 10950 3250 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5_HandSoldering" H 10700 3550 50  0001 C CNN
F 3 "" H 10800 3650 50  0001 C CNN
	1    10750 3450
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5918B788
P 10650 3750
F 0 "#PWR014" H 10650 3500 50  0001 C CNN
F 1 "GND" H 10650 3600 50  0000 C CNN
F 2 "" H 10650 3750 50  0001 C CNN
F 3 "" H 10650 3750 50  0001 C CNN
	1    10650 3750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5918BA09
P 9800 3050
F 0 "#PWR015" H 9800 2800 50  0001 C CNN
F 1 "GND" H 9800 2900 50  0000 C CNN
F 2 "" H 9800 3050 50  0001 C CNN
F 3 "" H 9800 3050 50  0001 C CNN
	1    9800 3050
	1    0    0    -1  
$EndComp
Text Label 11500 3450 0    60   ~ 0
V_PRES
$Comp
L power:+3.3V #PWR016
U 1 1 5918C10D
P 10650 3150
F 0 "#PWR016" H 10650 3000 50  0001 C CNN
F 1 "+3.3V" H 10650 3290 50  0000 C CNN
F 2 "" H 10650 3150 50  0001 C CNN
F 3 "" H 10650 3150 50  0001 C CNN
	1    10650 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:Rotary_Encoder_Switch SW2
U 1 1 5919010E
P 10500 4600
F 0 "SW2" H 10200 4400 50  0000 C CNN
F 1 "Rotary_Encoder_Switch_RGB" H 10500 4340 50  0000 C CNN
F 2 "nogasm:COM-10982" H 10400 4760 50  0001 C CNN
F 3 "" H 10500 4860 50  0001 C CNN
	1    10500 4600
	1    0    0    1   
$EndComp
$Comp
L Device:C C20
U 1 1 591911C9
P 9700 4900
F 0 "C20" H 9725 5000 50  0000 L CNN
F 1 "0.01uF" H 9725 4800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9738 4750 50  0001 C CNN
F 3 "" H 9700 4900 50  0001 C CNN
	1    9700 4900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C19
U 1 1 59191348
P 9350 4900
F 0 "C19" H 9375 5000 50  0000 L CNN
F 1 "0.01uF" H 9375 4800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9388 4750 50  0001 C CNN
F 3 "" H 9350 4900 50  0001 C CNN
	1    9350 4900
	1    0    0    -1  
$EndComp
Text Label 9050 4500 2    60   ~ 0
ENC_B
Text Label 9050 4700 2    60   ~ 0
ENC_A
$Comp
L power:GND #PWR017
U 1 1 5919181E
P 10000 5300
F 0 "#PWR017" H 10000 5050 50  0001 C CNN
F 1 "GND" H 10000 5150 50  0000 C CNN
F 2 "" H 10000 5300 50  0001 C CNN
F 3 "" H 10000 5300 50  0001 C CNN
	1    10000 5300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR018
U 1 1 59193379
P 11200 4500
F 0 "#PWR018" H 11200 4350 50  0001 C CNN
F 1 "+5V" H 11200 4640 50  0000 C CNN
F 2 "" H 11200 4500 50  0001 C CNN
F 3 "" H 11200 4500 50  0001 C CNN
	1    11200 4500
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:MMBT3904 Q2
U 1 1 591935AD
P 10300 5700
F 0 "Q2" H 10500 5775 50  0000 L CNN
F 1 "MMBT2222A" H 10500 5700 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 10500 5625 50  0001 L CIN
F 3 "" H 10300 5700 50  0001 L CNN
	1    10300 5700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 591938E5
P 12150 5450
F 0 "R7" V 12230 5450 50  0000 C CNN
F 1 "10k" V 12150 5450 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 12080 5450 50  0001 C CNN
F 3 "" H 12150 5450 50  0001 C CNN
	1    12150 5450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 59193AA4
P 12150 6200
F 0 "R8" V 12230 6200 50  0000 C CNN
F 1 "15k" V 12150 6200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 12080 6200 50  0001 C CNN
F 3 "" H 12150 6200 50  0001 C CNN
	1    12150 6200
	1    0    0    -1  
$EndComp
Text Label 12350 5800 0    60   ~ 0
ENC_SW
$Comp
L Transistor_BJT:MMBT3904 Q3
U 1 1 59197563
P 10800 6050
F 0 "Q3" H 11000 6125 50  0000 L CNN
F 1 "MMBT2222A" H 11000 6050 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 11000 5975 50  0001 L CIN
F 3 "" H 10800 6050 50  0001 L CNN
	1    10800 6050
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:MMBT3904 Q4
U 1 1 591975DD
P 11350 6400
F 0 "Q4" H 11550 6475 50  0000 L CNN
F 1 "MMBT2222A" H 11550 6400 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 11550 6325 50  0001 L CIN
F 3 "" H 11350 6400 50  0001 L CNN
	1    11350 6400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 59197A68
P 10400 6200
F 0 "R4" V 10480 6200 50  0000 C CNN
F 1 "300" V 10400 6200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 10330 6200 50  0001 C CNN
F 3 "" H 10400 6200 50  0001 C CNN
	1    10400 6200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 59197B48
P 10900 6550
F 0 "R5" V 10980 6550 50  0000 C CNN
F 1 "180" V 10900 6550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 10830 6550 50  0001 C CNN
F 3 "" H 10900 6550 50  0001 C CNN
	1    10900 6550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 59197C23
P 11450 6900
F 0 "R6" V 11530 6900 50  0000 C CNN
F 1 "180" V 11450 6900 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 11380 6900 50  0001 C CNN
F 3 "" H 11450 6900 50  0001 C CNN
	1    11450 6900
	1    0    0    -1  
$EndComp
Text Label 9800 5700 2    60   ~ 0
ENC_Rd
Text Label 9800 6050 2    60   ~ 0
ENC_Gr
Text Label 9800 6400 2    60   ~ 0
ENC_Bl
$Comp
L power:GND #PWR019
U 1 1 591992D2
P 11450 7400
F 0 "#PWR019" H 11450 7150 50  0001 C CNN
F 1 "GND" H 11450 7250 50  0000 C CNN
F 2 "" H 11450 7400 50  0001 C CNN
F 3 "" H 11450 7400 50  0001 C CNN
	1    11450 7400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5919BED0
P 4100 4750
F 0 "C2" H 4125 4850 50  0000 L CNN
F 1 "1uF" H 4125 4650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4138 4600 50  0001 C CNN
F 3 "" H 4100 4750 50  0001 C CNN
	1    4100 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5919C166
P 4500 4750
F 0 "C3" H 4525 4850 50  0000 L CNN
F 1 "1uF" H 4525 4650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4538 4600 50  0001 C CNN
F 3 "" H 4500 4750 50  0001 C CNN
	1    4500 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5919C464
P 4900 4750
F 0 "C4" H 4925 4850 50  0000 L CNN
F 1 "1uF" H 4925 4650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4938 4600 50  0001 C CNN
F 3 "" H 4900 4750 50  0001 C CNN
	1    4900 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5919C6ED
P 5250 4750
F 0 "C5" H 5275 4850 50  0000 L CNN
F 1 "1uF" H 5275 4650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5288 4600 50  0001 C CNN
F 3 "" H 5250 4750 50  0001 C CNN
	1    5250 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5919C6F3
P 5650 4750
F 0 "C7" H 5675 4850 50  0000 L CNN
F 1 "1uF" H 5675 4650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5688 4600 50  0001 C CNN
F 3 "" H 5650 4750 50  0001 C CNN
	1    5650 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 5919C6F9
P 6050 4750
F 0 "C8" H 6075 4850 50  0000 L CNN
F 1 "1uF" H 6075 4650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 6088 4600 50  0001 C CNN
F 3 "" H 6050 4750 50  0001 C CNN
	1    6050 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 5919C8D3
P 6450 4750
F 0 "C9" H 6475 4850 50  0000 L CNN
F 1 "1uF" H 6475 4650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 6488 4600 50  0001 C CNN
F 3 "" H 6450 4750 50  0001 C CNN
	1    6450 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 5919C8D9
P 6850 4750
F 0 "C10" H 6875 4850 50  0000 L CNN
F 1 "1uF" H 6875 4650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 6888 4600 50  0001 C CNN
F 3 "" H 6850 4750 50  0001 C CNN
	1    6850 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 5919C8DF
P 7250 4750
F 0 "C11" H 7275 4850 50  0000 L CNN
F 1 "1uF" H 7275 4650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7288 4600 50  0001 C CNN
F 3 "" H 7250 4750 50  0001 C CNN
	1    7250 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C12
U 1 1 5919C8E5
P 7600 4750
F 0 "C12" H 7625 4850 50  0000 L CNN
F 1 "1uF" H 7625 4650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7638 4600 50  0001 C CNN
F 3 "" H 7600 4750 50  0001 C CNN
	1    7600 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C14
U 1 1 5919C8EB
P 8000 4750
F 0 "C14" H 8025 4850 50  0000 L CNN
F 1 "1uF" H 8025 4650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 8038 4600 50  0001 C CNN
F 3 "" H 8000 4750 50  0001 C CNN
	1    8000 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C16
U 1 1 5919C8F1
P 8400 4750
F 0 "C16" H 8425 4850 50  0000 L CNN
F 1 "1uF" H 8425 4650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 8438 4600 50  0001 C CNN
F 3 "" H 8400 4750 50  0001 C CNN
	1    8400 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5919DE7B
P 3750 4750
F 0 "C1" H 3775 4850 50  0000 L CNN
F 1 "1uF" H 3775 4650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3788 4600 50  0001 C CNN
F 3 "" H 3750 4750 50  0001 C CNN
	1    3750 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 2450 5100 2450
Wire Wire Line
	4300 2550 5100 2550
Wire Wire Line
	5100 2450 5100 2350
Wire Wire Line
	5100 2350 5300 2350
Wire Wire Line
	5100 2550 5100 2650
Wire Wire Line
	5100 2650 5300 2650
Wire Wire Line
	5550 2650 5550 2850
Connection ~ 5300 2650
Wire Wire Line
	5550 2350 5550 2100
Connection ~ 5300 2350
Wire Wire Line
	1400 3150 2150 3150
Wire Wire Line
	4300 2650 4550 2650
Wire Wire Line
	4550 2650 4550 2800
Wire Wire Line
	4550 2800 4750 2800
Wire Wire Line
	4300 3550 5100 3550
Wire Wire Line
	2150 2150 1550 2150
Wire Wire Line
	2150 2250 1550 2250
Wire Wire Line
	2150 2350 1550 2350
Wire Wire Line
	2150 2450 1550 2450
Wire Wire Line
	2150 2550 1550 2550
Wire Wire Line
	2150 2650 1550 2650
Wire Wire Line
	1350 3050 2150 3050
Wire Wire Line
	2150 2950 1350 2950
Wire Wire Line
	1350 2850 2150 2850
Wire Wire Line
	2150 2750 1350 2750
Wire Wire Line
	750  2750 600  2750
Wire Wire Line
	600  2750 600  2850
Wire Wire Line
	750  2850 600  2850
Connection ~ 600  2850
Wire Wire Line
	750  2950 600  2950
Connection ~ 600  2950
Wire Wire Line
	750  3050 600  3050
Connection ~ 600  3050
Wire Wire Line
	4300 2750 4450 2750
Wire Wire Line
	4450 2750 4450 3100
Wire Wire Line
	4450 3100 5750 3100
Wire Wire Line
	5750 3200 5750 3100
Connection ~ 5750 3100
Wire Wire Line
	6150 3100 6650 3100
Wire Wire Line
	5750 3500 6950 3500
Wire Wire Line
	6950 3250 6950 3500
Connection ~ 6950 3500
Wire Wire Line
	6700 2350 6950 2350
Wire Wire Line
	6950 2350 6950 2450
Wire Wire Line
	6700 2450 6950 2450
Wire Wire Line
	6700 2250 6900 2250
Wire Wire Line
	6900 2150 6900 2250
Connection ~ 6900 2250
Wire Wire Line
	7550 2250 7550 2350
Connection ~ 6950 2450
Wire Wire Line
	7350 2450 7350 2650
Wire Wire Line
	7350 2650 7550 2650
Wire Wire Line
	6800 1050 7100 1050
Wire Wire Line
	6800 950  7100 950 
Wire Wire Line
	7100 950  7100 1050
Connection ~ 7100 1050
Wire Wire Line
	6800 850  7100 850 
Wire Wire Line
	7100 800  7100 850 
Connection ~ 7100 850 
Wire Wire Line
	7800 900  7800 850 
Connection ~ 7800 850 
Wire Wire Line
	7800 1300 8500 1300
Wire Wire Line
	7800 1300 7800 1200
Wire Wire Line
	8500 1150 8500 1300
Connection ~ 8500 1300
Wire Wire Line
	8150 2000 8150 2400
Wire Wire Line
	8150 2400 8500 2400
Connection ~ 8500 2400
Wire Wire Line
	8150 2700 8500 2700
Wire Wire Line
	8150 2700 8150 3100
Wire Wire Line
	9100 2500 8950 2500
Wire Wire Line
	8950 2500 8950 2700
Connection ~ 8500 2700
Wire Wire Line
	9100 2600 9100 3550
Wire Wire Line
	9600 3550 9850 3550
Wire Wire Line
	9850 3850 9850 3950
Connection ~ 9850 3550
Wire Wire Line
	10450 3350 10150 3350
Wire Wire Line
	10150 3350 10150 3050
Wire Wire Line
	10300 2900 11300 2900
Wire Wire Line
	11300 2900 11300 3450
Wire Wire Line
	11050 3450 11300 3450
Wire Wire Line
	10000 2900 9800 2900
Wire Wire Line
	9800 2900 9800 3050
Connection ~ 11300 3450
Wire Wire Line
	9100 3550 9300 3550
Wire Wire Line
	9050 4500 9350 4500
Wire Wire Line
	10200 4600 10000 4600
Wire Wire Line
	9050 4700 9700 4700
Wire Wire Line
	9350 4500 9350 4750
Connection ~ 9350 4500
Wire Wire Line
	9700 4750 9700 4700
Connection ~ 9700 4700
Wire Wire Line
	9350 5050 9700 5050
Wire Wire Line
	10000 4600 10000 5050
Connection ~ 9700 5050
Connection ~ 10000 5050
Wire Wire Line
	10800 4500 11200 4500
Wire Wire Line
	10800 4700 12150 4700
Wire Wire Line
	12150 4700 12150 5300
Wire Wire Line
	12150 5600 12150 5800
Wire Wire Line
	12150 5800 12350 5800
Connection ~ 12150 5800
Wire Wire Line
	10400 4900 10400 5500
Wire Wire Line
	10400 5900 10400 6050
Wire Wire Line
	10900 6250 10900 6400
Wire Wire Line
	11450 6600 11450 6750
Wire Wire Line
	10600 6050 9800 6050
Wire Wire Line
	9800 5700 10100 5700
Wire Wire Line
	11150 6400 9800 6400
Wire Wire Line
	10400 6350 10400 7250
Wire Wire Line
	10400 7250 10900 7250
Wire Wire Line
	11450 7050 11450 7250
Wire Wire Line
	10900 6700 10900 7250
Connection ~ 10900 7250
Connection ~ 11450 7250
Wire Wire Line
	10500 4900 10500 5400
Wire Wire Line
	10500 5400 10900 5400
Wire Wire Line
	10900 5400 10900 5850
Wire Wire Line
	10600 4900 10600 5300
Wire Wire Line
	10600 5300 11450 5300
Wire Wire Line
	11450 5300 11450 6200
Wire Wire Line
	12150 7250 12150 6350
Wire Wire Line
	3750 4600 3750 4450
Wire Wire Line
	3750 4450 4100 4450
Wire Wire Line
	7600 4450 7600 4600
Wire Wire Line
	4100 4600 4100 4450
Connection ~ 4100 4450
Wire Wire Line
	4500 4600 4500 4450
Connection ~ 4500 4450
Wire Wire Line
	4900 4600 4900 4450
Connection ~ 4900 4450
Wire Wire Line
	5250 4600 5250 4450
Connection ~ 5250 4450
Wire Wire Line
	5650 4200 5650 4450
Connection ~ 5650 4450
Wire Wire Line
	6050 4600 6050 4450
Connection ~ 6050 4450
Wire Wire Line
	6450 4600 6450 4450
Connection ~ 6450 4450
Wire Wire Line
	6850 4600 6850 4450
Connection ~ 6850 4450
Wire Wire Line
	7250 4600 7250 4450
Connection ~ 7250 4450
Wire Wire Line
	7600 5100 7600 4900
Wire Wire Line
	3750 5100 4100 5100
Wire Wire Line
	3750 5100 3750 4900
Wire Wire Line
	4100 4900 4100 5100
Connection ~ 4100 5100
Wire Wire Line
	4500 4900 4500 5100
Connection ~ 4500 5100
Wire Wire Line
	4900 4900 4900 5100
Connection ~ 4900 5100
Wire Wire Line
	5250 4900 5250 5100
Connection ~ 5250 5100
Wire Wire Line
	5650 4900 5650 5100
Connection ~ 5650 5100
Wire Wire Line
	6050 4900 6050 5100
Connection ~ 6050 5100
Wire Wire Line
	6450 4900 6450 5100
Connection ~ 6450 5100
Wire Wire Line
	6850 4900 6850 5100
Connection ~ 6850 5100
Wire Wire Line
	7250 4900 7250 5100
Connection ~ 7250 5100
$Comp
L power:+5V #PWR020
U 1 1 5919F0F8
P 5650 4200
F 0 "#PWR020" H 5650 4050 50  0001 C CNN
F 1 "+5V" H 5650 4340 50  0000 C CNN
F 2 "" H 5650 4200 50  0001 C CNN
F 3 "" H 5650 4200 50  0001 C CNN
	1    5650 4200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5919F162
P 5650 5300
F 0 "#PWR021" H 5650 5050 50  0001 C CNN
F 1 "GND" H 5650 5150 50  0000 C CNN
F 2 "" H 5650 5300 50  0001 C CNN
F 3 "" H 5650 5300 50  0001 C CNN
	1    5650 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 4450 8000 4600
Connection ~ 7600 4450
Wire Wire Line
	8400 4450 8400 4600
Connection ~ 8000 4450
Wire Wire Line
	8400 5100 8400 4900
Connection ~ 7600 5100
Wire Wire Line
	8000 4900 8000 5100
Connection ~ 8000 5100
$Comp
L ws2812:WS2812B U1
U 1 1 591A240D
P 2950 6550
F 0 "U1" H 2950 6650 60  0000 C CNN
F 1 "WS2812B" H 2950 6550 60  0000 C CNN
F 2 "nogasm:WS2812B" H 2950 6550 60  0000 C CNN
F 3 "" H 2950 6550 60  0000 C CNN
	1    2950 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 6800 2400 6800
$Comp
L ws2812:WS2812B U2
U 1 1 591A3038
P 4250 6550
F 0 "U2" H 4250 6650 60  0000 C CNN
F 1 "WS2812B" H 4250 6550 60  0000 C CNN
F 2 "nogasm:WS2812B" H 4250 6550 60  0000 C CNN
F 3 "" H 4250 6550 60  0000 C CNN
	1    4250 6550
	1    0    0    -1  
$EndComp
$Comp
L ws2812:WS2812B U3
U 1 1 591A3113
P 5550 6550
F 0 "U3" H 5550 6650 60  0000 C CNN
F 1 "WS2812B" H 5550 6550 60  0000 C CNN
F 2 "nogasm:WS2812B" H 5550 6550 60  0000 C CNN
F 3 "" H 5550 6550 60  0000 C CNN
	1    5550 6550
	1    0    0    -1  
$EndComp
$Comp
L ws2812:WS2812B U4
U 1 1 591A31D7
P 6850 6550
F 0 "U4" H 6850 6650 60  0000 C CNN
F 1 "WS2812B" H 6850 6550 60  0000 C CNN
F 2 "nogasm:WS2812B" H 6850 6550 60  0000 C CNN
F 3 "" H 6850 6550 60  0000 C CNN
	1    6850 6550
	1    0    0    -1  
$EndComp
$Comp
L ws2812:WS2812B U6
U 1 1 591A34EE
P 4250 7150
F 0 "U6" H 4250 7250 60  0000 C CNN
F 1 "WS2812B" H 4250 7150 60  0000 C CNN
F 2 "nogasm:WS2812B" H 4250 7150 60  0000 C CNN
F 3 "" H 4250 7150 60  0000 C CNN
	1    4250 7150
	1    0    0    -1  
$EndComp
$Comp
L ws2812:WS2812B U7
U 1 1 591A34F4
P 5550 7150
F 0 "U7" H 5550 7250 60  0000 C CNN
F 1 "WS2812B" H 5550 7150 60  0000 C CNN
F 2 "nogasm:WS2812B" H 5550 7150 60  0000 C CNN
F 3 "" H 5550 7150 60  0000 C CNN
	1    5550 7150
	1    0    0    -1  
$EndComp
$Comp
L ws2812:WS2812B U8
U 1 1 591A34FA
P 6850 7150
F 0 "U8" H 6850 7250 60  0000 C CNN
F 1 "WS2812B" H 6850 7150 60  0000 C CNN
F 2 "nogasm:WS2812B" H 6850 7150 60  0000 C CNN
F 3 "" H 6850 7150 60  0000 C CNN
	1    6850 7150
	1    0    0    -1  
$EndComp
$Comp
L ws2812:WS2812B U9
U 1 1 591A366B
P 2950 7700
F 0 "U9" H 2950 7800 60  0000 C CNN
F 1 "WS2812B" H 2950 7700 60  0000 C CNN
F 2 "nogasm:WS2812B" H 2950 7700 60  0000 C CNN
F 3 "" H 2950 7700 60  0000 C CNN
	1    2950 7700
	1    0    0    -1  
$EndComp
$Comp
L ws2812:WS2812B U10
U 1 1 591A3671
P 4250 7700
F 0 "U10" H 4250 7800 60  0000 C CNN
F 1 "WS2812B" H 4250 7700 60  0000 C CNN
F 2 "nogasm:WS2812B" H 4250 7700 60  0000 C CNN
F 3 "" H 4250 7700 60  0000 C CNN
	1    4250 7700
	1    0    0    -1  
$EndComp
$Comp
L ws2812:WS2812B U11
U 1 1 591A3677
P 5550 7700
F 0 "U11" H 5550 7800 60  0000 C CNN
F 1 "WS2812B" H 5550 7700 60  0000 C CNN
F 2 "nogasm:WS2812B" H 5550 7700 60  0000 C CNN
F 3 "" H 5550 7700 60  0000 C CNN
	1    5550 7700
	1    0    0    -1  
$EndComp
$Comp
L ws2812:WS2812B U12
U 1 1 591A367D
P 6850 7700
F 0 "U12" H 6850 7800 60  0000 C CNN
F 1 "WS2812B" H 6850 7700 60  0000 C CNN
F 2 "nogasm:WS2812B" H 6850 7700 60  0000 C CNN
F 3 "" H 6850 7700 60  0000 C CNN
	1    6850 7700
	1    0    0    -1  
$EndComp
$Comp
L ws2812:WS2812B U13
U 1 1 591A36F1
P 2950 8250
F 0 "U13" H 2950 8350 60  0000 C CNN
F 1 "WS2812B" H 2950 8250 60  0000 C CNN
F 2 "nogasm:WS2812B" H 2950 8250 60  0000 C CNN
F 3 "" H 2950 8250 60  0000 C CNN
	1    2950 8250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 6800 3700 6800
Wire Wire Line
	4800 6800 5000 6800
Wire Wire Line
	6100 6800 6300 6800
Wire Wire Line
	2150 7000 7700 7000
Wire Wire Line
	2150 7000 2150 7400
Wire Wire Line
	2150 7400 2400 7400
Wire Wire Line
	3500 7400 3700 7400
Wire Wire Line
	4800 7400 5000 7400
Wire Wire Line
	6100 7400 6300 7400
Wire Wire Line
	2150 7600 7700 7600
Wire Wire Line
	2150 7600 2150 7950
Wire Wire Line
	2150 7950 2400 7950
Wire Wire Line
	3500 7950 3700 7950
Wire Wire Line
	4800 7950 5000 7950
Wire Wire Line
	6100 7950 6300 7950
Wire Wire Line
	2150 8150 7700 8150
Wire Wire Line
	2150 8150 2150 8500
Wire Wire Line
	2150 8500 2400 8500
Wire Wire Line
	3500 5850 3500 6300
Connection ~ 3500 7300
Connection ~ 3500 7850
Wire Wire Line
	4800 6300 4800 6700
Connection ~ 4800 7300
Wire Wire Line
	6100 6300 6100 6700
Connection ~ 6100 7300
Wire Wire Line
	7700 7600 7700 7400
Wire Wire Line
	7700 7400 7400 7400
Wire Wire Line
	7400 6800 7700 6800
Wire Wire Line
	7700 6800 7700 7000
Wire Wire Line
	7400 6300 7400 6700
Connection ~ 7400 7300
Wire Wire Line
	7700 8150 7700 7950
Wire Wire Line
	7700 7950 7400 7950
Connection ~ 3500 6700
Connection ~ 4800 6700
Connection ~ 6100 6700
Connection ~ 7400 6700
Wire Wire Line
	3500 6300 4800 6300
Connection ~ 6100 6300
Connection ~ 4800 6300
Connection ~ 3500 6300
$Comp
L power:GND #PWR022
U 1 1 591A5E31
P 3500 5850
F 0 "#PWR022" H 3500 5600 50  0001 C CNN
F 1 "GND" H 3500 5700 50  0000 C CNN
F 2 "" H 3500 5850 50  0001 C CNN
F 3 "" H 3500 5850 50  0001 C CNN
	1    3500 5850
	0    1    1    0   
$EndComp
$Comp
L ws2812:WS2812B U5
U 1 1 591A34E8
P 2950 7150
F 0 "U5" H 2950 7250 60  0000 C CNN
F 1 "WS2812B" H 2950 7150 60  0000 C CNN
F 2 "nogasm:WS2812B" H 2950 7150 60  0000 C CNN
F 3 "" H 2950 7150 60  0000 C CNN
	1    2950 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 6700 2400 6400
Connection ~ 2400 6400
Wire Wire Line
	3700 6400 3700 6700
Connection ~ 3700 6400
Wire Wire Line
	5000 6400 5000 6700
Connection ~ 5000 6400
Wire Wire Line
	6300 6400 6300 6700
Wire Wire Line
	1700 7500 2400 7500
Wire Wire Line
	2400 7850 2400 7500
Connection ~ 2400 7500
Wire Wire Line
	2400 7300 2400 7100
Connection ~ 2400 7100
Wire Wire Line
	3700 7100 3700 7300
Connection ~ 3700 7100
Wire Wire Line
	3700 7850 3700 7500
Connection ~ 3700 7500
Wire Wire Line
	5000 7100 5000 7300
Connection ~ 5000 7100
Wire Wire Line
	6300 7100 6300 7300
Wire Wire Line
	1700 6400 2400 6400
Wire Wire Line
	6300 7500 6300 7850
Wire Wire Line
	5000 7850 5000 7500
Connection ~ 5000 7500
$Comp
L power:+5V #PWR023
U 1 1 591A855B
P 1700 6400
F 0 "#PWR023" H 1700 6250 50  0001 C CNN
F 1 "+5V" H 1700 6540 50  0000 C CNN
F 2 "" H 1700 6400 50  0001 C CNN
F 3 "" H 1700 6400 50  0001 C CNN
	1    1700 6400
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR024
U 1 1 591A8692
P 1700 7100
F 0 "#PWR024" H 1700 6950 50  0001 C CNN
F 1 "+5V" H 1700 7240 50  0000 C CNN
F 2 "" H 1700 7100 50  0001 C CNN
F 3 "" H 1700 7100 50  0001 C CNN
	1    1700 7100
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR025
U 1 1 591A8745
P 1700 7500
F 0 "#PWR025" H 1700 7350 50  0001 C CNN
F 1 "+5V" H 1700 7640 50  0000 C CNN
F 2 "" H 1700 7500 50  0001 C CNN
F 3 "" H 1700 7500 50  0001 C CNN
	1    1700 7500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 8400 1700 8400
$Comp
L power:+5V #PWR026
U 1 1 591A896B
P 1700 8400
F 0 "#PWR026" H 1700 8250 50  0001 C CNN
F 1 "+5V" H 1700 8540 50  0000 C CNN
F 2 "" H 1700 8400 50  0001 C CNN
F 3 "" H 1700 8400 50  0001 C CNN
	1    1700 8400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 7100 2400 7100
Text Label 1700 6800 0    60   ~ 0
5VIO
$Comp
L Regulator_Linear:LM7805_TO220 U15
U 1 1 591DDDA6
P 8500 850
F 0 "U15" H 8300 1050 50  0000 C CNN
F 1 "LM7805CT" H 8500 1050 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-220_Horizontal" H 8500 950 50  0000 C CIN
F 3 "" H 8500 850 50  0000 C CNN
	1    8500 850 
	1    0    0    -1  
$EndComp
$Comp
L nogasm:BUK92150-55A Q1
U 1 1 593084D1
P 6850 3050
F 0 "Q1" H 7050 3125 50  0000 L CNN
F 1 "BUK92150-55A" H 7050 3050 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-3_TabPin2" H 7050 2975 50  0000 L CIN
F 3 "" H 6850 3050 50  0000 L CNN
	1    6850 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 2650 5550 2650
Wire Wire Line
	5300 2350 5550 2350
Wire Wire Line
	600  2850 600  2950
Wire Wire Line
	600  2950 600  3050
Wire Wire Line
	600  3050 600  3400
Wire Wire Line
	5750 3100 5850 3100
Wire Wire Line
	6950 3500 6950 3650
Wire Wire Line
	6900 2250 7550 2250
Wire Wire Line
	6950 2450 6950 2850
Wire Wire Line
	6950 2450 7350 2450
Wire Wire Line
	7100 1050 7100 1100
Wire Wire Line
	7100 850  7800 850 
Wire Wire Line
	7800 850  8200 850 
Wire Wire Line
	8500 1300 8500 1450
Wire Wire Line
	8500 2400 9100 2400
Wire Wire Line
	8500 2700 8950 2700
Wire Wire Line
	9850 3550 10450 3550
Wire Wire Line
	11300 3450 11500 3450
Wire Wire Line
	9350 4500 10200 4500
Wire Wire Line
	9700 4700 10200 4700
Wire Wire Line
	9700 5050 10000 5050
Wire Wire Line
	10000 5050 10000 5300
Wire Wire Line
	12150 5800 12150 6050
Wire Wire Line
	10900 7250 11450 7250
Wire Wire Line
	11450 7250 11450 7400
Wire Wire Line
	11450 7250 12150 7250
Wire Wire Line
	4100 4450 4500 4450
Wire Wire Line
	4500 4450 4900 4450
Wire Wire Line
	4900 4450 5250 4450
Wire Wire Line
	5250 4450 5650 4450
Wire Wire Line
	5650 4450 5650 4600
Wire Wire Line
	5650 4450 6050 4450
Wire Wire Line
	6050 4450 6450 4450
Wire Wire Line
	6450 4450 6850 4450
Wire Wire Line
	6850 4450 7250 4450
Wire Wire Line
	7250 4450 7600 4450
Wire Wire Line
	4100 5100 4500 5100
Wire Wire Line
	4500 5100 4900 5100
Wire Wire Line
	4900 5100 5250 5100
Wire Wire Line
	5250 5100 5650 5100
Wire Wire Line
	5650 5100 5650 5300
Wire Wire Line
	5650 5100 6050 5100
Wire Wire Line
	6050 5100 6450 5100
Wire Wire Line
	6450 5100 6850 5100
Wire Wire Line
	6850 5100 7250 5100
Wire Wire Line
	7250 5100 7600 5100
Wire Wire Line
	7600 4450 8000 4450
Wire Wire Line
	8000 4450 8400 4450
Wire Wire Line
	7600 5100 8000 5100
Wire Wire Line
	8000 5100 8400 5100
Wire Wire Line
	3500 7300 3500 7850
Wire Wire Line
	3500 7850 3500 8400
Wire Wire Line
	4800 7300 4800 7850
Wire Wire Line
	6100 7300 6100 7850
Wire Wire Line
	7400 7300 7400 7850
Wire Wire Line
	3500 6700 3500 7300
Wire Wire Line
	4800 6700 4800 7300
Wire Wire Line
	6100 6700 6100 7300
Wire Wire Line
	7400 6700 7400 7300
Wire Wire Line
	6100 6300 7400 6300
Wire Wire Line
	4800 6300 6100 6300
Wire Wire Line
	3500 6300 3500 6700
Wire Wire Line
	2400 6400 3700 6400
Wire Wire Line
	3700 6400 5000 6400
Wire Wire Line
	5000 6400 6300 6400
Wire Wire Line
	2400 7500 3700 7500
Wire Wire Line
	2400 7100 3700 7100
Wire Wire Line
	3700 7100 5000 7100
Wire Wire Line
	3700 7500 5000 7500
Wire Wire Line
	5000 7100 6300 7100
Wire Wire Line
	5000 7500 6300 7500
$Comp
L Device:R_POT RV1
U 1 1 5918B842
P 10150 2900
F 0 "RV1" H 10150 2800 50  0000 C CNN
F 1 "10k" H 10150 2900 50  0000 C CNN
F 2 "Potentiometers:Potentiometer_Trimmer_Vishay_TS53YL" H 10150 2900 50  0001 C CNN
F 3 "" H 10150 2900 50  0001 C CNN
	1    10150 2900
	0    -1   1    0   
$EndComp
Wire Wire Line
	8500 1300 8900 1300
Wire Wire Line
	8900 1300 8900 1200
$Comp
L Device:CP C18
U 1 1 59187FB4
P 8900 1050
F 0 "C18" H 8925 1150 50  0000 L CNN
F 1 "10uF" H 8925 950 50  0000 L CNN
F 2 "Capacitors_SMD:C_1210_HandSoldering" H 8938 900 50  0001 C CNN
F 3 "" H 8900 1050 50  0001 C CNN
	1    8900 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 900  8900 850 
$Comp
L power:+5V #PWR010
U 1 1 5918822B
P 9300 850
F 0 "#PWR010" H 9300 700 50  0001 C CNN
F 1 "+5V" H 9300 990 50  0000 C CNN
F 2 "" H 9300 850 50  0001 C CNN
F 3 "" H 9300 850 50  0001 C CNN
	1    9300 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 850  9300 850 
Wire Wire Line
	8800 850  8900 850 
Connection ~ 8900 850 
$EndSCHEMATC
