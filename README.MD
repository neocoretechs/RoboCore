<h1>RoboCore</h1>
The RoboCore is the interface between real time devices and ROSJavaLite for the purpose of collecting real time data and presenting it on the ROS bus under various topics, and for receiving control signals as a subscriber and delivering those
to the intended device.
Modules for reading Kinect device, PS3 controller, Laser based range finders, Video, and other external devices is provided. The bulk of the code is devoted to communication with the 'WireOO repository' output from the Mega2560 controller using custom M and G codes to effect functions on that board.
A series of asynchronous serial readers and writers handle the different types of messages coming from the WireOO 2560 including low level GPIO, ultrasonic sensor,and motor controller propulsion, configuration, and monitoring codes (smart controllers and H bridge as well).
The 'MegaPubs' class provides the primary entry point for most of the Mega250 communication and motor control which is handled via the USB port and Java RxTx library. Using combinations of M ang G codes sent over the serial interface various
GPIO functions can be controlled, motor controllers of different types may be configured and linked to pins that
process ultrasonic and encoder inputs to handle low level motor lockout during proxmimity or preventing runaway of the robot. Notifications and alarms are presented in real time via the Mega2560 WireOO interface and multiplexed on the
ROSJavaLite bus.
Notifications can be delivered via a speech subsystem using the 'VoxHumana' repository by activating the speech ROS node and publishing to the speech topic.
In general classes ending with 'pubs' are publishers of data to the ROS bus and those with 'subs' are subscribers, although some provide dual functionality.


