EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
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
L Connector:Conn_01x15_Female J2
U 1 1 6188FF26
P 2550 2600
F 0 "J2" H 2578 2626 50  0000 L CNN
F 1 "Conn" H 2350 3450 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x15_P2.54mm_Vertical" H 2550 2600 50  0001 C CNN
F 3 "~" H 2550 2600 50  0001 C CNN
	1    2550 2600
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x15_Female J3
U 1 1 618907DF
P 2900 2600
F 0 "J3" H 2950 2600 50  0000 C CNN
F 1 "Conn" H 2850 1750 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x15_P2.54mm_Vertical" H 2900 2600 50  0001 C CNN
F 3 "~" H 2900 2600 50  0001 C CNN
	1    2900 2600
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x03_Male J7
U 1 1 6189190A
P 4600 1500
F 0 "J7" H 4572 1524 50  0000 R CNN
F 1 "Conn_01x03_Male" H 4572 1433 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 4600 1500 50  0001 C CNN
F 3 "~" H 4600 1500 50  0001 C CNN
	1    4600 1500
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male J6
U 1 1 618921C3
P 4550 2050
F 0 "J6" H 4522 2074 50  0000 R CNN
F 1 "Conn_01x03_Male" H 4522 1983 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 4550 2050 50  0001 C CNN
F 3 "~" H 4550 2050 50  0001 C CNN
	1    4550 2050
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 61894086
P 1300 1400
F 0 "J1" H 1408 1581 50  0000 C CNN
F 1 "Power" H 1100 1400 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Horizontal" H 1300 1400 50  0001 C CNN
F 3 "~" H 1300 1400 50  0001 C CNN
	1    1300 1400
	1    0    0    1   
$EndComp
$Comp
L Device:R_Small R1
U 1 1 61899807
P 1450 2900
F 0 "R1" H 1509 2946 50  0000 L CNN
F 1 "220" H 1509 2855 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P2.54mm_Vertical" H 1450 2900 50  0001 C CNN
F 3 "~" H 1450 2900 50  0001 C CNN
	1    1450 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Small D1
U 1 1 6189A3D3
P 1450 3200
F 0 "D1" V 1496 3130 50  0000 R CNN
F 1 "LED_Small" V 1405 3130 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm" V 1450 3200 50  0001 C CNN
F 3 "~" V 1450 3200 50  0001 C CNN
	1    1450 3200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3350 1900 3100 1900
$Comp
L Connector:Conn_01x03_Male J4
U 1 1 618A749E
P 4500 2500
F 0 "J4" H 4472 2524 50  0000 R CNN
F 1 "Conn_01x03_Male" H 4472 2433 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 4500 2500 50  0001 C CNN
F 3 "~" H 4500 2500 50  0001 C CNN
	1    4500 2500
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male J5
U 1 1 618A797F
P 4500 3000
F 0 "J5" H 4472 3024 50  0000 R CNN
F 1 "Conn_01x03_Male" H 4472 2933 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 4500 3000 50  0001 C CNN
F 3 "~" H 4500 3000 50  0001 C CNN
	1    4500 3000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4350 2150 4050 2150
Wire Wire Line
	4300 2600 4050 2600
Wire Wire Line
	4300 3100 4050 3100
Wire Wire Line
	4350 2100 4350 2050
Wire Wire Line
	4050 1600 4400 1600
Wire Wire Line
	4300 2500 4050 2500
Wire Wire Line
	4300 3000 4050 3000
Wire Wire Line
	4300 2900 3550 2900
Wire Wire Line
	3550 2900 3550 3550
Wire Wire Line
	3550 3550 2100 3550
Wire Wire Line
	2100 3550 2100 3300
Wire Wire Line
	2100 3300 2350 3300
Wire Wire Line
	4300 2400 3450 2400
Wire Wire Line
	3450 2400 3450 3500
Wire Wire Line
	3450 3500 2050 3500
Wire Wire Line
	2050 3500 2050 3200
Wire Wire Line
	2050 3200 2350 3200
Wire Wire Line
	4350 1950 3400 1950
Wire Wire Line
	3400 1950 3400 3450
Wire Wire Line
	3400 3450 2000 3450
Wire Wire Line
	2000 3450 2000 3100
Wire Wire Line
	2000 3100 2350 3100
Text GLabel 4150 1400 0    50   Input ~ 0
channel1
Wire Wire Line
	4150 1400 4400 1400
Text GLabel 2100 3000 0    50   Input ~ 0
channel1
Wire Wire Line
	2350 2800 1450 2800
Wire Wire Line
	1450 3000 1450 3100
Wire Wire Line
	1450 3300 1450 3700
$Comp
L power:GND #PWR01
U 1 1 618C89CE
P 1450 3700
F 0 "#PWR01" H 1450 3450 50  0001 C CNN
F 1 "GND" H 1455 3527 50  0000 C CNN
F 2 "" H 1450 3700 50  0001 C CNN
F 3 "" H 1450 3700 50  0001 C CNN
	1    1450 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 618C90F5
P 4050 3100
F 0 "#PWR013" H 4050 2850 50  0001 C CNN
F 1 "GND" H 3900 3050 50  0000 C CNN
F 2 "" H 4050 3100 50  0001 C CNN
F 3 "" H 4050 3100 50  0001 C CNN
	1    4050 3100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR08
U 1 1 618CBE0F
P 4050 2100
F 0 "#PWR08" H 4050 1950 50  0001 C CNN
F 1 "+3.3V" H 3900 2150 50  0000 C CNN
F 2 "" H 4050 2100 50  0001 C CNN
F 3 "" H 4050 2100 50  0001 C CNN
	1    4050 2100
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR02
U 1 1 618CDC16
P 1700 1300
F 0 "#PWR02" H 1700 1150 50  0001 C CNN
F 1 "+5V" H 1715 1473 50  0000 C CNN
F 2 "" H 1700 1300 50  0001 C CNN
F 3 "" H 1700 1300 50  0001 C CNN
	1    1700 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 1300 1550 1300
$Comp
L power:GND #PWR03
U 1 1 618D1600
P 1700 1400
F 0 "#PWR03" H 1700 1150 50  0001 C CNN
F 1 "GND" H 1705 1227 50  0000 C CNN
F 2 "" H 1700 1400 50  0001 C CNN
F 3 "" H 1700 1400 50  0001 C CNN
	1    1700 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 1400 1550 1400
Wire Wire Line
	3100 2200 3250 2200
$Comp
L power:+5V #PWR06
U 1 1 618D5439
P 3350 1900
F 0 "#PWR06" H 3350 1750 50  0001 C CNN
F 1 "+5V" H 3400 1950 50  0000 C CNN
F 2 "" H 3350 1900 50  0001 C CNN
F 3 "" H 3350 1900 50  0001 C CNN
	1    3350 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 618D59EE
P 3250 2200
F 0 "#PWR05" H 3250 1950 50  0001 C CNN
F 1 "GND" H 3255 2027 50  0000 C CNN
F 2 "" H 3250 2200 50  0001 C CNN
F 3 "" H 3250 2200 50  0001 C CNN
	1    3250 2200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 618D8111
P 4050 2600
F 0 "#PWR011" H 4050 2350 50  0001 C CNN
F 1 "GND" H 3900 2550 50  0000 C CNN
F 2 "" H 4050 2600 50  0001 C CNN
F 3 "" H 4050 2600 50  0001 C CNN
	1    4050 2600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 618D8613
P 4050 2150
F 0 "#PWR09" H 4050 1900 50  0001 C CNN
F 1 "GND" H 3900 2100 50  0000 C CNN
F 2 "" H 4050 2150 50  0001 C CNN
F 3 "" H 4050 2150 50  0001 C CNN
	1    4050 2150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 618D8AB5
P 4050 1600
F 0 "#PWR07" H 4050 1350 50  0001 C CNN
F 1 "GND" H 4055 1427 50  0000 C CNN
F 2 "" H 4050 1600 50  0001 C CNN
F 3 "" H 4050 1600 50  0001 C CNN
	1    4050 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 2100 4350 2100
$Comp
L power:+3.3V #PWR012
U 1 1 618DFFB5
P 4050 3000
F 0 "#PWR012" H 4050 2850 50  0001 C CNN
F 1 "+3.3V" H 3900 3050 50  0000 C CNN
F 2 "" H 4050 3000 50  0001 C CNN
F 3 "" H 4050 3000 50  0001 C CNN
	1    4050 3000
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR014
U 1 1 618E4EC7
P 4250 1500
F 0 "#PWR014" H 4250 1350 50  0001 C CNN
F 1 "+3.3V" H 4100 1500 50  0000 C CNN
F 2 "" H 4250 1500 50  0001 C CNN
F 3 "" H 4250 1500 50  0001 C CNN
	1    4250 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 1500 4400 1500
$Comp
L power:+3.3V #PWR04
U 1 1 618E5582
P 3250 2100
F 0 "#PWR04" H 3250 1950 50  0001 C CNN
F 1 "+3.3V" H 3450 2150 50  0000 C CNN
F 2 "" H 3250 2100 50  0001 C CNN
F 3 "" H 3250 2100 50  0001 C CNN
	1    3250 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 2100 3200 2100
$Comp
L power:+3.3V #PWR010
U 1 1 618F5024
P 4050 2500
F 0 "#PWR010" H 4050 2350 50  0001 C CNN
F 1 "+3.3V" H 3900 2550 50  0000 C CNN
F 2 "" H 4050 2500 50  0001 C CNN
F 3 "" H 4050 2500 50  0001 C CNN
	1    4050 2500
	1    0    0    -1  
$EndComp
Connection ~ 3250 2100
Wire Wire Line
	3250 2100 3300 2100
Connection ~ 3250 2200
Wire Wire Line
	3250 2200 3300 2200
Wire Wire Line
	2100 3000 2350 3000
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 61900E11
P 1600 1300
F 0 "#FLG0101" H 1600 1375 50  0001 C CNN
F 1 "PWR_FLAG" H 2000 1350 50  0000 C CNN
F 2 "" H 1600 1300 50  0001 C CNN
F 3 "~" H 1600 1300 50  0001 C CNN
	1    1600 1300
	1    0    0    -1  
$EndComp
Connection ~ 1600 1300
Wire Wire Line
	1600 1300 1700 1300
Connection ~ 1600 1400
Wire Wire Line
	1600 1400 1700 1400
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 6190156D
P 1600 1400
F 0 "#FLG0102" H 1600 1475 50  0001 C CNN
F 1 "PWR_FLAG" H 2000 1400 50  0000 C CNN
F 2 "" H 1600 1400 50  0001 C CNN
F 3 "~" H 1600 1400 50  0001 C CNN
	1    1600 1400
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 61902BEB
P 3200 2100
F 0 "#FLG0103" H 3200 2175 50  0001 C CNN
F 1 "PWR_FLAG" H 3450 2200 50  0000 C CNN
F 2 "" H 3200 2100 50  0001 C CNN
F 3 "~" H 3200 2100 50  0001 C CNN
	1    3200 2100
	1    0    0    -1  
$EndComp
Connection ~ 3200 2100
Wire Wire Line
	3200 2100 3250 2100
NoConn ~ 3100 2000
NoConn ~ 3100 2300
NoConn ~ 3100 2400
NoConn ~ 2350 1900
NoConn ~ 2350 2000
NoConn ~ 2350 2100
NoConn ~ 2350 2200
NoConn ~ 2350 2300
NoConn ~ 2350 2400
NoConn ~ 2350 2500
NoConn ~ 2350 2600
NoConn ~ 2350 2700
NoConn ~ 2350 2900
NoConn ~ 3100 2500
NoConn ~ 3100 2600
NoConn ~ 3100 2700
NoConn ~ 3100 2800
NoConn ~ 3100 2900
NoConn ~ 3100 3000
NoConn ~ 3100 3100
NoConn ~ 3100 3200
NoConn ~ 3100 3300
NoConn ~ 100  2200
Connection ~ 1550 1400
Wire Wire Line
	1550 1400 1600 1400
Connection ~ 1550 1300
Wire Wire Line
	1550 1300 1600 1300
$EndSCHEMATC