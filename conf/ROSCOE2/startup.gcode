M0
M10 Z0 T1
M3 Z0 P8 C1 D24 E0 W68
M3 Z0 P10 C2 D22 E0 W69
; Config LED driver Type 4 control in slot 1
M10 Z1 T4
; Now config LED driver slot 1 to PWM pin 13 on channel1 enable pin 2
M9 Z1 P13 C1 D2
;M301 P36
;M33 P36 D60 E1
;M303 P64
; M39 removes analog pin, M303 enables it
;M39 P64
;M1

