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
Text GLabel 3200 2350 0    50   Input ~ 0
1
Text GLabel 3200 2450 0    50   Input ~ 0
2
Text GLabel 3200 2550 0    50   Input ~ 0
3
Text GLabel 3200 2650 0    50   Input ~ 0
4
Text GLabel 3200 2750 0    50   Input ~ 0
5
Text GLabel 3200 2850 0    50   Input ~ 0
6
Text GLabel 3200 2950 0    50   Input ~ 0
7
Text GLabel 3200 3050 0    50   Input ~ 0
8
Text GLabel 3200 3150 0    50   Input ~ 0
9
Text GLabel 3200 3250 0    50   Input ~ 0
10
Text GLabel 3200 3350 0    50   Input ~ 0
11
Text GLabel 3200 3450 0    50   Input ~ 0
12
Text GLabel 3200 3550 0    50   Input ~ 0
13
Text GLabel 3200 3650 0    50   Input ~ 0
14
Text GLabel 3200 3750 0    50   Input ~ 0
15
Text GLabel 3200 3850 0    50   Input ~ 0
16
Text GLabel 3200 3950 0    50   Input ~ 0
17
Text GLabel 3200 4050 0    50   Input ~ 0
18
$Comp
L TMC6140-EVAL~v1.1-rescue:TMC6140-Trinamic IC1
U 1 1 623CDFA4
P 5700 3350
F 0 "IC1" H 5700 4491 39  0000 C CNN
F 1 "TMC6140-Trinamic" H 5700 4416 39  0000 C CNN
F 2 "Package_DFN_QFN:QFN-36-1EP_5x6mm_P0.5mm_EP3.6x4.1mm_ThermalVias" H 5700 4450 39  0001 C CNN
F 3 "" H 5700 3950 39  0001 C CNN
	1    5700 3350
	1    0    0    -1  
$EndComp
Text GLabel 6200 3750 2    50   Input ~ 0
1
Text GLabel 6200 2750 2    50   Input ~ 0
2
Text GLabel 6200 2850 2    50   Input ~ 0
3
Text GLabel 6200 2950 2    50   Input ~ 0
4
Text GLabel 6200 3550 2    50   Input ~ 0
5
Text GLabel 6200 3350 2    50   Input ~ 0
6
Text GLabel 6200 2650 2    50   Input ~ 0
7
Text GLabel 6200 2550 2    50   Input ~ 0
8
Text GLabel 6200 2450 2    50   Input ~ 0
9
Text GLabel 5200 2650 0    50   Input ~ 0
10
Text GLabel 5200 2450 0    50   Input ~ 0
11
Text GLabel 5200 2850 0    50   Input ~ 0
12
Text GLabel 5200 3350 0    50   Input ~ 0
13
Text GLabel 5200 3650 0    50   Input ~ 0
14
Text GLabel 5200 3450 0    50   Input ~ 0
15
Text GLabel 5200 3750 0    50   Input ~ 0
16
Text GLabel 5200 3550 0    50   Input ~ 0
17
Text GLabel 5200 3850 0    50   Input ~ 0
18
Text GLabel 6800 2350 0    50   Input ~ 0
19
Text GLabel 6800 2450 0    50   Input ~ 0
20
Text GLabel 6800 2550 0    50   Input ~ 0
21
Text GLabel 6800 2650 0    50   Input ~ 0
22
Text GLabel 6800 2750 0    50   Input ~ 0
23
Text GLabel 6800 2850 0    50   Input ~ 0
24
Text GLabel 6800 2950 0    50   Input ~ 0
25
Text GLabel 6800 3050 0    50   Input ~ 0
26
Text GLabel 6800 3150 0    50   Input ~ 0
27
Text GLabel 6800 3250 0    50   Input ~ 0
28
Text GLabel 6800 3350 0    50   Input ~ 0
29
Text GLabel 6800 3450 0    50   Input ~ 0
30
Text GLabel 6800 3550 0    50   Input ~ 0
31
Text GLabel 6800 3650 0    50   Input ~ 0
32
Text GLabel 6800 3750 0    50   Input ~ 0
33
Text GLabel 6800 3850 0    50   Input ~ 0
34
Text GLabel 6800 3950 0    50   Input ~ 0
35
Text GLabel 6800 4050 0    50   Input ~ 0
36
Text GLabel 5200 3150 0    50   Input ~ 0
19
Text GLabel 5200 3250 0    50   Input ~ 0
20
Text GLabel 5200 2750 0    50   Input ~ 0
21
Text GLabel 5200 4050 0    50   Input ~ 0
22
Text GLabel 5200 4150 0    50   Input ~ 0
23
Text GLabel 6200 3950 2    50   Input ~ 0
24
Text GLabel 5200 4250 0    50   Input ~ 0
25
Text GLabel 6200 3450 2    50   Input ~ 0
26
Text GLabel 6200 3650 2    50   Input ~ 0
27
Text GLabel 6200 3850 2    50   Input ~ 0
28
Text GLabel 5200 3050 0    50   Input ~ 0
29
Text GLabel 5200 3950 0    50   Input ~ 0
30
Text GLabel 5200 2950 0    50   Input ~ 0
31
Text GLabel 5200 2550 0    50   Input ~ 0
32
Text GLabel 6200 3050 2    50   Input ~ 0
34
Text GLabel 6200 3150 2    50   Input ~ 0
35
Text GLabel 6200 3250 2    50   Input ~ 0
36
Text GLabel 6200 4150 2    50   Input ~ 0
33
Wire Wire Line
	6200 4250 6350 4250
$Comp
L TMC6140-EVAL~v1.1-cache:power_GND #PWR0101
U 1 1 6234FC47
P 6350 4250
F 0 "#PWR0101" H 6350 4000 50  0001 C CNN
F 1 "power_GND" H 6355 4077 50  0000 C CNN
F 2 "" H 6350 4250 50  0001 C CNN
F 3 "" H 6350 4250 50  0001 C CNN
	1    6350 4250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x19 J1
U 1 1 6236EBD8
P 3400 3250
F 0 "J1" H 3480 3292 50  0000 L CNN
F 1 "Conn_01x19" H 3480 3201 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x19_P2.54mm_Vertical" H 3400 3250 50  0001 C CNN
F 3 "~" H 3400 3250 50  0001 C CNN
	1    3400 3250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x19 J2
U 1 1 62377F6E
P 7000 3250
F 0 "J2" H 6918 2125 50  0000 C CNN
F 1 "Conn_01x19" H 6918 2216 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x19_P2.54mm_Vertical" H 7000 3250 50  0001 C CNN
F 3 "~" H 7000 3250 50  0001 C CNN
	1    7000 3250
	1    0    0    1   
$EndComp
Wire Wire Line
	6350 4250 6800 4250
Wire Wire Line
	6800 4250 6800 4150
Connection ~ 6350 4250
Wire Wire Line
	3200 4150 3200 4650
Wire Wire Line
	3200 4650 6800 4650
Wire Wire Line
	6800 4650 6800 4250
Connection ~ 6800 4250
$EndSCHEMATC
