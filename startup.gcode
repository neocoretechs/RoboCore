; M10 Controller types - Type 0=Smart controller, Type 1=HBridge, Type 2=Split Bridge, Type 3=Switch Bridge
; Type 4=PWM driver which uses slots separate from controller types
; After M10 definition, these codes configure the defined driver:
; M2 - define smart controller. M2 [Z<slot>] [C<channel> W<encoder pin> E<default dir>]
; M3 - define H-bridge. M3 [Z<slot>] P<pin> C<channel> D<direction pin> E<default dir> W<encoder pin> [R<resolution 8,9,10 bits>] [X<prescale 0-7>]
; M4 - define Split Bridge. M4 [Z<slot>] P<pin> Q<pin> C<channel> D<enable pin> E<default dir> [W<encoder pin>] [R<resolution 8,9,10 bits>] [X<prescale 0-7>]
; M5 - define Switch Bridge. M5 Z<slot> P<pin> Q<pin> C<channel> D<enable pin> E<default dir> [W<encoder>]
; M9 - define PWM driver. M9 [Z<slot>] P<pin> C<channel> D<enable pin> [R<resolution 8,9,10 bits>] [X<prescale 0-7>
M0
; Define diff driver for traction in slot 0, Type 1 is HBridge driver in motor driver slots
M10 Z0 T1
; Config H-bridge diff driver slot 0, PWM pin 8 channel 1, Direction pin digital 24, encoder pin 68
M3 Z0 P8 C1 D24 E0 W68
; Config H-bridge diff driver slot 0 PWM pin 10 channel 2, Dir pin 22, encoder 69 default start dir 0
M3 Z0 P10 C2 D22 E0 W69
; Define H-bridge driver slot 1 for boom actuator, PWM pin 9 channel 1, Dir pin 26, no encoder
M10 Z1 T1
; config H-bridge actuator slot 1, PWM pin 9 channel 1, Dir pin 26, default dir 0
M3 Z1 P9 C1 D26 E0
; Define PWM LED driver Type 4 control in slot 0, Type 4 is a PWM driver in seperate slots from motor drivers
M10 Z0 T4
; Now config LED driver slot 0 to PWM pin 13 on channel 1 enable pin 30
M9 Z0 P13 C1 D30
; Now Define SplitBridge lift actuator slot 2 to Type 2 split bridge driver in motor driver slot
M10 Z2 T2
; Config SplitBridge lift actuator slot 2, digital pins 2 and 3 for forward/reverse channel 1, direction pin 28, default dir 0
M4 Z2 P2 Q3 C1 D32 E0
;M301 P36
;M33 P36 D60 E1
;M303 P64
; M39 removes analog pin, M303 enables it
;M39 P64
;M1

