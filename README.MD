<h1>RoboCore</h1>
Provides core functions for field robotics.<p/>
RoboCore acts as an interface between real time devices and ROSJavaLite for the purpose of collecting real time data and presenting it on the ROSJavaLite messaging bus.<p/>
A node can handle messages under various topics as a publisher, and receive control signals as a subscriber, and deliver messages to the intended subsystem on distributed nodes on the robot.<p/>
ROS Services are also employed as tools for diagnostic reporting and ad hoc control of PWM and GPIO low level functions for prototyping and testing of various subsystems.<p/> 
In addition to presenting the robot as a logical collection of configurations and process setpoints, modules for reading low level hardware devices, PS3 style controllers, Laser based range finders, video, and other external devices are provided. 
A portion of the code is devoted to communication with the <b>MarlinSpike realtime subsystem</b>, which uses a Single Board Computer GPIO or a microcontroller such as the Mega2560 controller.<p/>
A platform independant subsystem of custom M and G codes passes messages to effect functions on those realtime boards.<p/>
A series of asynchronous serial readers and writers handle the different types of messages coming from the MarlinSpike, including low level GPIO, ultrasonic sensor,and motor controller propulsion, configuration, and monitoring codes.<p/>
Virtually any type of smart motor controller and various types of Half bridge, Split bridge, and H bridge motor controller can be configured
with a few lines of M code sent to the MarlinSpike subsystem.<p/>
The 'MegaPubs' class provides the primary entry point for most of the MarlinSpike communication and motor control. 
In the case of a detached microcontroller, communication is handled via the USB port and Java RxTx library.<p/> 
Using combinations of M and G codes sent to the MarlinSpike, various hardware functions can be controlled, and motor controllers of different types may be configured, as previously mentioned, and subsequently the necessary control commands or PWM levels and enable and disable pins and functions can be described via the M and G code processing that takes place from the higher level messages on the ROSJavaLite bus down to the low level real time processing on the MarlinSpike<p/>
Some of the hardware functions may process ultrasonic range finders or read encoder inputs to handle low level motor lockout during proximity, or read hall odometry sensors on the wheels thus preventing runaway of the robot.<p/> 
Notifications and alarms from attached hardware controllers and low-level sensing devices are presented in real time via the MarlinSpike subsystem and multiplexed on the ROSJavaLite bus.<p/>
The robots may communicate with end users by notifications which can be delivered via a speech subsystem using the 'VoxHumana' module in a repository by the same name, and by activating the speech ROSJavaLite subscriber node with the source of the notification publishing to the ROSJavaLite message bus speech topic.<p/>
These are just some of the functions provided by way of example, many others are present in the operational code base in the various repositories. This repository contains what might be described as the 'nervous system' and 'autonomic functions' of the various robots that run under its operational code. If you peruse the files you may notice configurations for 2 robots that currently are fully operational and under ongoing further research and development.<p/>
The 2 example robots; ROSCOE1 and ROSCOE2 both use the exact same code base save for some minor changes to a few configurations for the propulsion controls, motor types, and peripherals they both have. ROSCOE1 uses a smart controller and brushless DC motor drive housed in a 20mm ammo case, while ROSCOE2 uses gear motors and H bridge drivers using an old mobility wheelchair chassis stripped to its battery carrier and gear motor/suspension elements, then augmented with custom housings for electronics and boom crane style effectors.<p/>
Both robots have steroscopic vision systems using conventional webcams, and run on a series of networked single board computers such as Raspberry Pis and Odroids with various wireless access point options for external facing controls and user interface and maintenance.<p/>
Although autonomous control is the ultimate goal, there is a wearable device that uses a single board computer and ruggedized power system along with controller and display that is also used to control the robots in real time.<p/>
In general, classes ending with 'pubs' are publishers of data to the ROSJavaLite bus and those with 'subs' are subscribers, although some provide dual functionality.


