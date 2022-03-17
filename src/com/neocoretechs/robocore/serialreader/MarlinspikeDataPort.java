package com.neocoretechs.robocore.serialreader;

import java.io.IOException;

import com.neocoretechs.robocore.SynchronizedFixedThreadPoolManager;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.robocore.serialreader.marlinspikeport.PWM;
import com.neocoretechs.robocore.serialreader.marlinspikeport.Pins;
import com.neocoretechs.robocore.serialreader.marlinspikeport.control.AbstractMotorControl;
import com.neocoretechs.robocore.serialreader.marlinspikeport.control.RoboteqDevice;
import com.neocoretechs.robocore.serialreader.marlinspikeport.control.SplitBridgeDriver;
import com.neocoretechs.robocore.serialreader.marlinspikeport.control.SwitchBridgeDriver;
import com.neocoretechs.robocore.serialreader.marlinspikeport.pwmcontrol.AbstractPWMControl;
import com.neocoretechs.robocore.serialreader.marlinspikeport.pwmcontrol.HBridgeDriver;
import com.neocoretechs.robocore.serialreader.marlinspikeport.pwmcontrol.VariablePWMDriver;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.GpioPinDigitalInput;
import com.pi4j.io.gpio.GpioPinAnalogInput;
import com.pi4j.io.gpio.OdroidC1Pin;
import com.pi4j.io.gpio.Pin;
import com.pi4j.io.gpio.PinMode;
import com.pi4j.io.gpio.GpioPin;
import com.pi4j.io.gpio.PinState;

/**
 * RoboCore robotic controller platform simulator.
 * Implementation of arduino type microcontroller firmware in PI4J Java asynchronous process.<p/>
 *
 * Like the original firmware, processes a variation of M and G code from CNC and 3D printing to control a range of motor controllers and drivers
 * and GPIO pins for smart machine and robotics hardware functionality.
 *
 * Previously geared toward the Mega2560 microcontroller chip, this code unifies the microcontroller platform and allows it to be easily accessed through
 * the standard CNC-derived 'language'. 
 *
 * As an example, it unifies the PWM pins to deliver consistent PWM functions across all available pins, and allows
 * various motor controllers like H-bridge, half bridge, and smart controllers to be controlled through UART, PWM, and GPIO functions.
 *
 * It contains a main processing loop that receives M and G code commands through a deque,
 * then uses dequeued entries for the main CNC-derived M and G code processing loop.
 *
 * motor controllers are all unified and
 * accessible through the main processing loop. In the primary loop, M and G code commands are processed.
 * 
 * Another example is the way in which ultrasonic sensors can be attached to any motor driver through main loop processing commands and used to
 * set up a minimum distance to detect an object before issuing a command to shut down the motor driver.
 *
 * To continue the examples, through the processing loop, hall effect sensors can be attached to any motor driver through pins designated in the main
 * processing loop M and G code commands and used to detect wheel rotations, then set up pin change interrupts for those sensors through other
 * M and G code commands, then perform actions based on the wheel rotations and the interrupts generated and processed through other M and G code commands.
 *
 *
* When 'stopped' is true the Gcodes G0-G5 are ignored as a safety interlock.
* Implemented Codes
*-------------------
* G0  -> G1
* G4  - Dwell S<seconds> or P<milliseconds>
* G5  - Command motor or PWM control C<Channel> [P<Motor Power -1000,1000>] [X<PWM level -1000,1000 scaled 0-2000]

* M Codes
* M0   - Real time output off
* M1   - Real time output on - (default)
* M2	- Set smart controller (default) with optional encoder pin [C<channel> E<encoder pin>] per channel.
* M3	- Configure H-bridge Motor driver with optional encoder pin
* M4	- Configure BLDC Motor driver with optional encoder pin
* M5	- Set maximum motor power
* M6	- Set motor scaling [S<scale>] divisor to divide power level
* M7	- Override and set motor stop mode until M8 is issued
* M8	- Set motor to normal run mode after M7
* M10	- Central configuration directive for creating all types of motor and PWM controls and drivers
* M11	- Set duration of encoder in interrupt ticks C<channel> D<duration>
* M12	- Set base power lever for motor C<channel> P<power>, will add this value to each motor command
* M33	- Link ultrasonic distance sensor to motor channel for safety interlock. P<pin> D<min. distance in cm> [E<direction 1- forward facing, 0 - reverse facing sensor>] 
* M35	- Clear all persistent digital pins
* M36	- Clear all persistent analog pins
* M37	- Clear all persistent PWM pins
* M38	- Remove persistent PWM pin
* M39	- Remove persistent analog pin
* M40	- Remove persistent digital pin
* M41	- Create persistent digital pin P<pin> set HIGH. (gives you a +5v pin)
* M42  - Create persistent digital pin P<pin> set LOW. (gives you a grounded pin)
* M43	- Read temporary one shot digital pin P<pin>, publish <digitalpin> 1 - pin, 2 - value
* M44	- Read temporary one shot digital pin with pullup, publish <digitalpin> 1 - pin, 2 - value
* M45	- Activate persistent PWM P<pin> S<power val 0-255> [T<timer mode 0-3>] [R<resolution 8,9,10 bits>] [X<prescale 0-7>]
* M80  - Turn on Power Supply
* M81  - Turn off Power Supply
* M82  - 
* M83  - 
* M84  - 
* M85  - 
* M92  - 
* M104 - 
* M105 - 
* M106 - 
* M107 - 
* M109 - 
* M114 - 
* M115 - Capabilities string
* M117 - display message
* M119 - 
* M126 -
* M127 - 
* M128 - 
* M129 - 
* M140 - 
* M150 -
* M190 - 
*       
* M200 
* M201 - 
* M202 - 
* M203 - 
* M204 - 
* M205 - 
* M206 - 
* M207 - 
* M208 - 
* M209 - 
* M218 - 
* M220 
* M221 
* M226	
* M227 - Same as M226 with INPUT_PULLUP
* M240 - Trigger a camera to take a photograph
* M250 - 
* M280 - 
* M300 - Emit one-shot ultrasonic pulse on given pin and return duration P<pin number>
* M301 - ultrasonic output P<pin>, publish with <ultrasonic> 1 - pin, 2 - reading in cm
* M302 - Disable ultrasonic 
* M303 - analog read for P<pin>, with exclusion range 0-1024 optional L<min> H<max> . Publish under <analogpin>
* M304 - Configure analog read with optional INPUT_PULLUP for P<pin>, with optional exclusion range 0-1024 L<min> H<max> .
* M305 - Configure digital read upon state 0 or 1, publish <digitalpin> 1 - pin, 2 - value
* M306 - Activate digital read with optional INPUT_PULLUP upon state  0 or 1, publish <digitalpin> 1 - pin, 2 - value
* M349 - 
* M400 - 
* M401 -
* M402 - 
* M445 - Disable PWM pin (compliment to M45), sets motor power to 0, removes PWM pin
* M500 - Stores parameters in EEPROM
* M501 - Reads parameters from EEPROM (if you need reset them after you changed them temporarily).
* M502 - Reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
* M503 - Print the current settings (from memory not from eeprom)
* M540 - 
* M600 - 
* M666 - 
* M605 - 
* M700 - RoboCore - Retrieve startup params
* M701 - Display digital pins in use
* M702 - Display analog pins in use
* M703 - Display ultrasonic pins in use
* M704 - Display PWM pins in use
* M705 - Display Motor controller and channel attributes
* M706 - Report all pins currently assigned
* M798 - Report status of attached controller
* M799 - Command emergency stop of motor controller
* M800 - 
* M802	- Acquire analog pin data M802 Pnn Sxxx Mxxx P=Pin number, S=number readings, M=microseconds per reading
* M810 - RoboCore - IMU readings
* M908	- Control digipot
* M999 - Restart after being stopped by error, clears 'stopped' flag
* 
* @author: Jonathan Neville Groff  Copyright (C) NeoCoreTechs 2020
*/
public class MarlinspikeDataPort implements Runnable, DataPortInterface {
	public static boolean DEBUG = true;
	public static int DEFAULT_PWM_FREQUENCY = 10000;
	public static int MAX_MOTOR_POWER = 1000;
	CircularBlockingDeque<String> inDeque = new CircularBlockingDeque<String>(1);
	CircularBlockingDeque<String> outDeque = new CircularBlockingDeque<String>(1024);
	static int gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

	static boolean realtime_output = true; // Determines whether real time data from inactive period is streamed

	static String cmdbuffer;
	static String[] cmdtokens = new String[25];
	static String outbuffer;
	char serial_char;

	static int serial_read;
	static int serial_count = 0;
	static boolean comment_mode = false;

	//Inactivity shutdown variables
	static long previous_millis_cmd = 0;
	static long max_inactive_time = 0;

	long starttime = 0;
	long stoptime = 0;

	boolean Stopped=false;
	boolean target_direction;
	
	PWM pwm = null;
	GpioPinDigitalOutput dpin;
	int nread = 0;
	long micros = 0;
	int[] values;
	String motorCntrlResp;
	int status;
	int fault = 0;
	// Dynamically defined ultrasonic rangers
	Ultrasonic[] psonics = new Ultrasonic[10];
	// Last distance published per sensor
	float[] sonicDist = new float[]{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
	// Dynamically defined analog pins
	double[][] analogRanges = new double[2][16];
	PWM[] panalogs = new PWM[10];//{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	// Dynamically defined digital pins
	int[] digitalTarget = new int[32];
	GpioPinDigitalOutput[] pdigitals= new GpioPinDigitalOutput[32];
	// PWM control block
	PWM[] ppwms = new PWM[12];//{0,0,0,0,0,0,0,0,0,0,0,0};
	static int pwm_freq = DEFAULT_PWM_FREQUENCY;
	
	private int MAX_CMD_SIZE = 1024;
	private volatile boolean shouldRun = true;

	// &roboteqDevice, new HBridgeDriver, new SplitBridgeDriver...
	AbstractMotorControl[] motorControl = new AbstractMotorControl[10];
	AbstractPWMControl[] pwmControl= new AbstractPWMControl[10];
	int channel;
	  
	int digitarg;
	static int uspin = 0;
	long t;
	int pin_number, pin_numberB;
	int dir_pin, dir_default, enable_pin;
	int encode_pin = 0;
	int dir_face;
	int dist;
	
	int result;
	int motorController = 0;
	int PWMDriver;
	int motorChannel; 
	int motorPower;
	int PWMLevel;
	int codenum;
	
	// Messages outputting data
	final static String FIRMWARE_URL = "http://www.neocoretechs.com";
	final static String PROTOCOL_VERSION = "Marlinspike Java 1.0";
	final static String MACHINE_NAME = "";
	final static String MACHINE_UUID = "";
	final static String errormagic= "Error:";
	final static String echomagic= "echo:";
	final static String datasetHdr= "dataset";
	final static String motorCntrlHdr= "motorcontrol";
	final static String sonicCntrlHdr ="ultrasonic";
	final static String timeCntrlHdr = "time";
	final static String posCntrlHdr=  "position";
	final static String motorFaultCntrlHdr= "motorfault";
	final static String PWMFaultCntrlHdr= "pwmfault";
	final static String batteryCntrlHdr= "battery";
	final static String digitalPinHdr= "digitalpin";
	final static String analogPinHdr="analogpin";
	final static String digitalPinSettingHdr ="digitalpinsetting";
	final static String analogPinSettingHdr= "analogpinsetting";
	final static String ultrasonicPinSettingHdr= "ultrasonicpinsetting";
	final static String pwmPinSettingHdr= "pwmpinsetting";
	final static String motorControlSettingHdr= "motorcontrolsetting";
	final static String pwmControlSettingHdr ="pwmcontrolsetting";
	final static String pinSettingHdr ="assignedpins";
	final static String controllerStatusHdr= "controllerstatus";
	final static String eepromHdr ="eeprom";
		
	// Message delimiters, quasi XML
	final static String MSG_BEGIN ="<";
	final static String MSG_DELIMIT= ">";
	final static String MSG_END ="</";
	final static String MSG_TERMINATE ="/>";

	// Serial Console informational Messages
	final static String MSG_STATUS ="status";
	final static String MSG_POWERUP = "PowerUp";
	final static String MSG_EXTERNAL_RESET= "External Reset";
	final static String MSG_BROWNOUT_RESET= "Brown out Reset";
	final static String MSG_WATCHDOG_RESET ="Watchdog Reset";
	final static String MSG_SOFTWARE_RESET ="Software Reset";
	final static String MSG_AUTHOR = " | Author: ";
	final static String MSG_CONFIGURATION_VER= " Last Updated: ";
	final static String MSG_FREE_MEMORY= " Free Memory: ";
	final static String MSG_ERR_LINE_NO ="Line Number is not Last Line Number+1, Last Line: ";
	final static String MSG_ERR_CHECKSUM_MISMATCH ="checksum mismatch, Last Line: ";
	final static String MSG_ERR_NO_CHECKSUM= "No Checksum with line number, Last Line: ";
	final static String MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM= "No Line Number with checksum, Last Line: ";
	final static String MSG_M115_REPORT ="FIRMWARE_NAME:Marlinspike RoboCore";
	final static String MSG_115_REPORT2 ="FIRMWARE_URL:"+ FIRMWARE_URL+ "\r\nPROTOCOL_VERSION:" +PROTOCOL_VERSION +"\r\nMACHINE_TYPE:"+ MACHINE_NAME +"\r\nUUID:"+ MACHINE_UUID;
	final static String MSG_ERR_KILLED ="Controller halted. kill() called!";
	final static String MSG_ERR_STOPPED ="Controller stopped due to errors";
	final static String MSG_RESEND= "Resend: ";
	final static String MSG_UNKNOWN_COMMAND= "Neither G nor M code found ";
	final static String MSG_UNKNOWN_GCODE= "Unknown G code ";
	final static String MSG_UNKNOWN_MCODE= "Unknown M code ";
	final static String MSG_BAD_MOTOR ="Bad Motor command ";
	final static String MSG_BAD_PWM= "Bad PWM Driver command ";
	
	// These correspond to the controller faults return by 'queryFaultCode'
	final static String MSG_MOTORCONTROL_1= "Overheat";
	final static String MSG_MOTORCONTROL_2= "Overvoltage";
	final static String MSG_MOTORCONTROL_3= "Undervoltage";
	final static String MSG_MOTORCONTROL_4= "Short circuit";
	final static String MSG_MOTORCONTROL_5= "Emergency stop";
	final static String MSG_MOTORCONTROL_6= "Sepex excitation fault";
	final static String MSG_MOTORCONTROL_7= "MOSFET failure";
	final static String MSG_MOTORCONTROL_8= "Startup configuration fault";
	final static String MSG_MOTORCONTROL_9= "Stall";

	
	@Override
	/**
	 * initialize the fixed thread pool manager, start thread running main loop
	 */
	public void connect(boolean writeable) throws IOException {
		SynchronizedFixedThreadPoolManager.init(3, Integer.MAX_VALUE, new String[]{getPortName()});
		SynchronizedFixedThreadPoolManager.spin(this, getPortName());
	}

	@Override
	public int read() throws IOException {
		try {
			return inDeque.take().charAt(0);
		} catch (InterruptedException e) {
			return -1;
		}
	}

	@Override
	public void write(int c) throws IOException {
		inDeque.addLast(String.valueOf((char)c));
	}

	@Override
	public void close() {
		try {
			stop();
		} catch (IOException e) {
			e.printStackTrace();
		}
		shouldRun = false;
	}

	@Override
	public String readLine() {
		try {
			return outDeque.take();
		} catch (InterruptedException e) {
			return null;
		}
	}

	@Override
	public int bytesToRead() throws IOException {
		if(inDeque.peek() == null)
			return 0;
		return inDeque.peek().length();
	}

	@Override
	public void writeLine(String output) throws IOException {
		if(DEBUG)
			System.out.printf("%s queuing command %s%n", this.getClass().getName(), output);
		inDeque.addLast(output);
	}

	@Override
	public String getPortName() {
		return "MarlinspikeDataPort";
	}

	@Override
	public String stringSettings() {
		return getPortName();
	}

	@Override
	public void run() {
		while(shouldRun) {
		  try {
			get_command();
			if(!comment_mode) {
				try {
					process_commands();
					manage_inactivity();
				} catch(IOException ioe) {
					throw new RuntimeException(ioe);
				}
			}
		  } catch (InterruptedException e) {
			shouldRun = false;
		  }
		}
	}
	 
	void get_command() throws InterruptedException {
	  comment_mode = true;
	  cmdbuffer = (String) inDeque.takeFirst();
	  serial_count = 0;
	  serial_char = (char)cmdbuffer.charAt(0);
	  if(serial_char == '\n' || serial_char == '\r' || serial_char == '#' ||  serial_char == ';' || serial_count >= (MAX_CMD_SIZE - 1) ) {
	        comment_mode = true; //for new command
	        return;
	  }
	  comment_mode = false;
	  // Determine if an outstanding error caused safety shutdown. If so respond with header
	  if(serial_char == 'G'){
	          switch(cmdbuffer.charAt(1)) {
				case '0':
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
					if(Stopped) { // If robot is stopped by an error the G[0-5] codes are ignored.
						serial_count = -1;
						return;
					}
					break;
				default:
					break;
	          }
	     }
	  serial_count = 0;
	  cmdtokens = cmdbuffer.split(" ");
	  if(DEBUG)
			System.out.printf("%s get_command %d tokens%n", this.getClass().getName(), cmdtokens.length);
	}

	float code_value() {
		  return Float.parseFloat(cmdtokens[serial_count].substring(1));
	}

	long code_value_long() {
		  return Integer.parseInt(cmdtokens[serial_count].substring(1));
	}
	/**
	 * Determine if parameter exists in command line token sequence
	 * @param code
	 * @return
	 */
	boolean code_seen(char code) {
		serial_count = -1;
		for(int i = 0; i < cmdtokens.length; i++) {
		  if( cmdtokens[i].charAt(0) == code ) {
			  serial_count = i;
			  return true;
		  }
		}
		return false;
	}

	/**
	 * Process command line
	 */
	void process_commands() throws IOException { 
		if(code_seen('G')) {
			  int cval = (int)code_value();
			  processGCode(cval);
		} else {
			 if(code_seen('M') ) {
				  int cval = (int)code_value();
				  processMCode(cval);
			 } else { // if neither G nor M code
				 System.out.println("Neither G nor M code encountered in command:"+cmdbuffer);
			 }
		}
	}

	/**
	 * Processing of G-code command sequence
	 * @param cval
	 */
	void processGCode(int cval) throws IOException {
		if(DEBUG)
			System.out.printf("%s processGCode %s%n", this.getClass().getName(), String.valueOf(cval));
		switch(cval) {    
		    case 4: // G4 dwell
		      codenum = 0;
		      if(code_seen('P')) codenum = (int) code_value(); // milliseconds to wait
		      if(code_seen('S')) codenum = (int) (code_value() * 1000); // seconds to wait
		      //codenum += millis();  // keep track of when we started waiting
		      previous_millis_cmd = 0;//millis();
		      while(++previous_millis_cmd  < codenum ) {
		        //manage_inactivity();
				try {
					Thread.sleep(1);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
		      }
			  outDeque.add(String.format("%sG4%s%n",MSG_BEGIN,MSG_TERMINATE));
		      break;
			  
			case 5: // G5 - Absolute command motor [Z<controller>] C<Channel> [P<motor power -1000 to 1000>] [X<PWM power -1000 to 1000>(scaled 0-2000)]
			     if(!Stopped) {
					 if(code_seen('Z')) {
						 motorController = (int) code_value();
					 }
				     if(code_seen('C')) {
						motorChannel = (int) code_value(); // channel 1,2
						if(code_seen('P')) {
							motorPower = (int) code_value(); // motor power -1000,1000
							fault = 0; // clear fault flag
							if( (status=motorControl[motorController].commandMotorPower(motorChannel, motorPower)) != 0) {
								outDeque.add(String.format("%s%s%d %d %d%s%n",MSG_BEGIN,MSG_BAD_MOTOR,status,motorChannel,motorPower,MSG_TERMINATE));
							} else {
								outDeque.add(String.format("%sG5%s%n",MSG_BEGIN,MSG_TERMINATE));
							}
						} else {// code P or X
							if(code_seen('X')) {
								PWMLevel = (int) code_value(); // PWM level -1000,1000, scaled to 0-2000 in PWM controller, as no reverse
								fault = 0; // clear fault flag
								// use motor related index and value, as we have them
								if( (status=pwmControl[motorController].commandPWMLevel(motorChannel, PWMLevel)) != 0) {
									outDeque.add(String.format("%s%s%d %d %d%s%n",MSG_BEGIN,MSG_BAD_PWM,status,motorChannel,PWMLevel,MSG_TERMINATE));
								} else {
									outDeque.add(String.format("%sG5%s%n",MSG_BEGIN,MSG_TERMINATE));
								}
							} // code X
						}
					 } // code C
			     } // stopped
			     break;
			  
			case 99: // G99 start watchdog timer. G99 T<time_in_millis> values are 15,30,60,120,250,500,1000,4000,8000 default 4000
				if( code_seen('T') ) {
					//int time_val = (int) code_value();
					//watchdog_timer = new WatchdogTimer();
					outDeque.add(String.format("%sG99%s%n",MSG_BEGIN,MSG_TERMINATE));
					//watchdog_timer.watchdog_init(time_val);
				}
				break;
				
			case 100: // G100 reset watchog timer before time interval is expired, otherwise a reset occurs
				//if( watchdog_timer != null ) {
				//	watchdog_timer.watchdog_reset();
				//}
				outDeque.add(String.format("%sG100%s%n",MSG_BEGIN,MSG_TERMINATE));
				break;
				
			default:
				outDeque.add(String.format("%s%s%s%s%n",MSG_BEGIN,MSG_UNKNOWN_GCODE,cmdbuffer,MSG_TERMINATE));
				break;		
		} // switch
	}
	  
	/**
	 * Process M codes
	 * @param cval
	 */
	void processMCode(int cval) {
		if(DEBUG)
			System.out.printf("%s processMCode %s%n", this.getClass().getName(), String.valueOf(cval));
		int motorController = 0; 
		int PWMDriver = 0;  
		switch( cval ) {
			case 0: // M0 - Set real time output off
				realtime_output = false;
				outDeque.add(String.format("%sM0%s%n",MSG_BEGIN,MSG_TERMINATE));
				break;
				
			case 1: // M1 - Set real time output on 
				realtime_output = true;
				outDeque.add(String.format("%sM1%s%n",MSG_BEGIN,MSG_TERMINATE));
				break;
				
			//CHANNEL 1-10, NO CHANNEL ZERO!	
			case 2: // M2 [Z<slot>] [C<channel> W<encoder pin> E<default dir>] - set smart controller (default) with optional encoder pin per channel, can be issued multiple times
				 if(code_seen('Z')) {
					 motorController = (int) code_value();
				 }
				if(code_seen('C')) {
					channel = (int) code_value();
					if(channel <= 0) {
						break;
					}
					if(code_seen('W')) {
						pin_number = (int) code_value();
						motorControl[motorController].createEncoder(channel, pin_number);
					}
					if(code_seen('E')) {
						motorControl[motorController].setDefaultDirection(channel, (int) code_value());
					}
					outDeque.add(String.format("%sM2%s%n",MSG_BEGIN,MSG_TERMINATE));
	
				}
				break;
				
			// Set HBridge PWM motor driver, map pin to channel, this will check to prevent free running motors during inactivity
			// For a PWM motor control subsequent G5 commands are affected here.
			// and then D a direction pin that determines forward/backward , then E, the default value of the direction pin.
			// The D and E pins determine whether a HIGH or LOW determines FORWARD/BACK on the given controller channel. This is
			// to compensate for 'backward' motors mounted left/right, or differences in controller design. Using a combination of
			// these 2 parameters you can tune any controller/motor setup properly for forward/back.
			// Finally, W<encoder pin>  to receive hall wheel sensor signals and
			// optionally PWM timer setup [R<resolution 8,9,10 bits>] [X<prescale 0-7>].
			// The Timer mode (0-3) is preset to 2 in the individual driver. Page 129 in datasheet. Technically we are using a 'non PWM'
			// where the 'compare output mode' is defined by 3 operating modes. Since we are unifying all the timers to use all available PWM
			// pins, the common mode among them all is the 'non PWM', within which the 3 available operating modes can be chosen from.
			// There are essentially three main operating modes:
			// 0 - Stop
			// 1 - Toggle on compare match
			// 2 - Clear on match
			// 3 - Set on match
			// For motor operation and general purpose PWM, mode 2 the most universally applicable.
			case 3: // M3 [Z<slot>] P<pin> C<channel> D<direction pin> E<default dir> W<encoder pin> [F PWM frequency]
				pin_number = -1;
				encode_pin = 0;
				pwm_freq = DEFAULT_PWM_FREQUENCY;
				if(code_seen('Z')) {
					motorController = (int) code_value();
				}
			 // motorControl = (AbstractMotorControl*)&hBridgeDriver;
			 if(motorControl[motorController] != null) {
			  ((HBridgeDriver)motorControl[motorController]).setMotors(ppwms);
			  ((HBridgeDriver)motorControl[motorController]).setDirectionPins(pdigitals);
			  if(code_seen('P')) {
		          pin_number = (int) code_value();
			  } else {
				 break;
			  }
		      if(code_seen('C')) {
		        channel = (int) code_value();
				if(channel <= 0) {
					break;
				}
				if( code_seen('D')) {
					dir_pin = (int) code_value();
				} else {
					break;
				}
				if( code_seen('E')) {
					dir_default = (int) code_value();
				} else {
					break;
				}
				if( code_seen('W')) {
					encode_pin = (int) code_value();
				}
				if(code_seen('F')) {
					pwm_freq = (int) code_value();
				}
				try {
					((HBridgeDriver)motorControl[motorController]).createPWM(channel, pin_number, dir_pin, dir_default, pwm_freq);
				} catch (IOException e) {
					e.printStackTrace();
				}
				if(encode_pin != 0) {
					motorControl[motorController].createEncoder(channel, encode_pin);
				}
				outDeque.add(String.format("%sM3%s%n",MSG_BEGIN,MSG_TERMINATE));
			   } // code_seen['C']
		      } // if motorControl[motorController]
			  break;
			  
			// Split bridge or 2 half bridge motor controller. Takes 2 inputs: one for forward,called P, one for backward,called Q, then motor channel, 
			// and then D, an enable pin. Finally, W<encoder pin>  to receive hall wheel sensor signals and 
			// optionally PWM timer setup [R<resolution 8,9,10 bits>] [X<prescale 0-7>].
			// Everything derived from HBridgeDriver can be done here.
			case 4:// M4 [Z<slot>] P<pin> Q<pin> C<channel> D<enable pin> B<enable pin b> E<default dir> [W<encoder pin>] [F<frequency 1-1000000>]
			  pwm_freq = DEFAULT_PWM_FREQUENCY; // frequency
			  pin_number = -1;
			  pin_numberB = -1;
			  int dir_pinb = -1;
			  encode_pin = 0;
			  if(code_seen('Z')) {
				motorController = (int)code_value();
			  }
			  if(motorControl[motorController]  != null) {
			  //motorControl = (AbstractMotorControl*)&splitBridgeDriver;
			  ((SplitBridgeDriver)motorControl[motorController]).setMotors(ppwms);
			  ((SplitBridgeDriver)motorControl[motorController]).setDirectionPins(pdigitals);
			  if(code_seen('P')) {
				pin_number = (int) code_value();
			  } else {
				 break;
			  }
			  if(code_seen('Q')) {
				pin_numberB = (int) code_value();
			 } else {
				break;
			 }
			  if(code_seen('C')) {
				  channel = (int) code_value();
				  if(channel <= 0) {
					break;
				  }
				  if( code_seen('D')) {
					dir_pin = (int) code_value();
				  } else {
					break;
				  }
				  if( code_seen('B')) {
						dir_pinb = (int) code_value();
				  } else {
						break;
				  }
				  if( code_seen('E')) {
					dir_default = (int) code_value();
				  } else {
					break;
				  }
				  if( code_seen('W')) {
					encode_pin = (int) code_value();
				  }
				  if( code_seen('F')) {
						pwm_freq = (int) code_value();
				  }
				  try {
					((SplitBridgeDriver)motorControl[motorController]).createPWM(channel, pin_number, pin_numberB, dir_pin, dir_pinb, dir_default, pwm_freq);
				} catch (IOException e) {
					e.printStackTrace();
				}
				  if(encode_pin != 0) {
					motorControl[motorController].createEncoder(channel, encode_pin);
				  }
				  outDeque.add(String.format("%sM4%s%n",MSG_BEGIN,MSG_TERMINATE));
				} // code C
				} //motorcontrol[motorcontroller]
				break;
				
			// Switch bridge or 2 digital motor controller. Takes 2 inputs: one digital pin for forward,called P, one for backward,called Q, then motor channel,
			// and then D, an enable pin, and E default dir, with optional encoder
			case 5: //M5 Z<slot> P<pin> Q<pin> C<channel> D<enable pin> E<default dir> [W<encoder>]- Create switch bridge Z slot, P forward pin, Q reverse pin, D enable, E default state of enable for dir
				pin_number = -1;
				pin_numberB = -1;
				encode_pin = 0;
				  if(code_seen('Z')) {
					  motorController = (int)code_value();
				  }
				  if(motorControl[motorController] != null) {
					  if(code_seen('P')) {
						  pin_number = (int) code_value();
					  } else {
						  break;
					  }
					  if(code_seen('Q')) {
						  pin_numberB = (int) code_value();
					  } else {
						  break;
					  }
					  if(code_seen('C')) {
						  channel = (int) code_value();
						  if(channel <= 0) {
							  break;
						  }
						  if( code_seen('D')) {
							  dir_pin = (int) code_value();
						  } else {
							  break;
						  }
						  if( code_seen('B')) {
							  dir_pinb = (int) code_value();
						  } else {
							  break;
						  }
						  if( code_seen('E')) {
							  dir_default = (int) code_value();
						  } else {
							  break;
						  }
						  if( code_seen('W')) {
							 encode_pin = (int) code_value();
						  }
						  ((SwitchBridgeDriver)motorControl[motorController]).createDigital(channel, pin_number, pin_numberB, dir_pin, dir_pinb, dir_default);
						  if(encode_pin != 0) {
							  motorControl[motorController].createEncoder(channel, encode_pin);
						  }
						  outDeque.add(String.format("%sM5%s%n",MSG_BEGIN,MSG_TERMINATE));
					  } // code C
				  } //motorcontrol[motorcontroller]
				break;
				
			case 6: //M6 [Z<slot>] [S<scale>] [X<scale>] - Set motor or PWM scaling, divisor for final power to limit speed or level, set to 0 to cancel. If X, slot is PWM
				if(code_seen('Z')) {
					motorController = (int) code_value();
				}
				if( code_seen('S') ) {
					if(motorControl[motorController] != null) {
						motorControl[motorController].setMotorPowerScale((int) code_value());
						outDeque.add(String.format("%sM6%s%n",MSG_BEGIN,MSG_TERMINATE));
					}
				} else {
					if(code_seen('X')) {
						if(pwmControl[motorController] != null) {
							pwmControl[motorController].setPWMPowerScale((int) code_value());
							outDeque.add(String.format("%sM6%s%n",MSG_BEGIN,MSG_TERMINATE));
						}
					}
				}
				break;
				
			case 7: // M7 [Z<slot>] [X]- Set motor override to stop motor operation, or optionally PWM operation, if X, slot is PWM
				if(code_seen('Z')) {
					motorController = (int) code_value();
				}
				if(code_seen('X')) {
					if(pwmControl[motorController] != null) {
						try {
							pwmControl[motorController].setPWMShutdown();
						} catch (IOException e) {
							e.printStackTrace();
						}
						outDeque.add(String.format("%sM7%s%n",MSG_BEGIN,MSG_TERMINATE));
					}
				} else {
					if(motorControl[motorController] != null) {
						try {
							motorControl[motorController].setMotorShutdown();
						} catch (IOException e) {
							e.printStackTrace();
						}
						outDeque.add(String.format("%sM7%s%n",MSG_BEGIN,MSG_TERMINATE));
					}
				}
				break;
				
			case 8: // M8 [Z<slot>][X] - Set motor override to start motor operation after stop override M7. If X, slot is PWM
				if(code_seen('Z')) {
					motorController = (int) code_value();
				}
				if(code_seen('X')) {
					if(pwmControl[motorController] != null) {
						try {
							pwmControl[motorController].setPWMRun();
						} catch (IOException e) {
							e.printStackTrace();
						}
						outDeque.add(String.format("%sM8%s%n",MSG_BEGIN,MSG_TERMINATE));
					}
				} else {
					if(motorControl[motorController] != null ) {
						try {
							motorControl[motorController].setMotorRun();
						} catch (IOException e) {
							e.printStackTrace();
						}
						outDeque.add(String.format("%sM8%s%n",MSG_BEGIN,MSG_TERMINATE));
					}
				}
				break;
				
			// Activate a previously created PWM controller of type AbstractPWMControl - a non propulsion PWM device such as LED or pump
			// Note there is no encoder or direction pin, and no possibility of reverse. What would be reverse in a motor control is the first
			// half of the power scale instead.
			case 9: // M9 [Z<slot>] P<pin> C<channel> D<enable pin> E<dir default> [F<PWM frequency>] - PWM control
				pwm_freq = DEFAULT_PWM_FREQUENCY; // resolution in bits
				pin_number = -1;
				encode_pin = 0;
				if(code_seen('Z')) {
					  PWMDriver = (int) code_value();
				}
				if(pwmControl[PWMDriver] != null) {
				 if(code_seen('P')) {
					  pin_number = (int)code_value();
				 } else {
					  break;
				 }
				 if(code_seen('C')) {
					channel = (int) code_value();
					if(channel <= 0) {
						break;
					}
					if( code_seen('D')) {
						enable_pin = (int) code_value();
					} else {
						break;
					}
					if( code_seen('E')) {
						  dir_default = (int) code_value();
					} else {
						  break;
					}
					if(code_seen('F')) {
						pwm_freq = (int) code_value();
					}
					try {
						((VariablePWMDriver)pwmControl[PWMDriver]).createPWM(channel, pin_number, enable_pin, dir_default, pwm_freq);
					} catch (IOException e) {
						e.printStackTrace();
					}
					outDeque.add(String.format("%sM9%s%n",MSG_BEGIN,MSG_TERMINATE));
				 }
				}
				break;
				
			// Dynamically allocate a controller to a control slot. The slot parameter is used to refer
			// to the dynamically allocated controller in other M codes that relate to motor control functions.
			// The M10 code merely creates the instance of the proper controller and assigns the slot. Other M codes
			// refer to the slot and provide further configuration. when creating new type of controllers, this is the code
			// that can be expanded to instantiate those controllers
			case 10: // M10 Z<controller slot> T<controller type>
				if( code_seen('Z') ) {
					motorController = (int) code_value();
					if( code_seen('T') ) {
						int controllerType = (int) code_value();		 
						switch(controllerType) {
							case 0: // type 0 smart controller
								if( motorControl[motorController]  != null) {
									//delete motorControl[motorController];
									motorControl[motorController] = null; // in case assignment below fails
								}
								try {
									motorControl[motorController] = new RoboteqDevice(MAX_MOTOR_POWER);
								} catch (IOException e) {
									e.printStackTrace();
								}
								outDeque.add(String.format("%sM10%s%n",MSG_BEGIN,MSG_TERMINATE));
								break;
							case 1: // type 1 Hbridge
								// up to 10 channels, each channel has a direction pin (1), and a PWM pin (0)
								if(motorControl[motorController] != null) {
									// for each channel, delete the direction pin and PWM created in main pin array to prepare new assignment
									// each controller can have up to 10 channels, each with its own PWM and direction pin
									for(int i = 1; i <= motorControl[motorController].getChannels(); i++) {
											int pMotor1 = ((HBridgeDriver)motorControl[motorController]).getMotorPWMPin(i);
											int pMotor2 = ((HBridgeDriver)motorControl[motorController]).getMotorEnablePin(i);
											if(pMotor2 != 255 && pdigitals[pMotor2] != null) {
												pdigitals[pMotor2] = null;
											}
											if(pMotor1 != 255 && ppwms[pMotor1] != null) {
												ppwms[pMotor1] = null;
											}
									}
									//delete motorControl[motorController];
									motorControl[motorController] = null; // in case assignment below fails
								}
								motorControl[motorController] = new HBridgeDriver(MAX_MOTOR_POWER);
								outDeque.add(String.format("%sM10%s%n",MSG_BEGIN,MSG_TERMINATE));
								break;
							case 2: // type 2 Split bridge, each channel has 2 PWM pins and an enable pin, so up to 5 channels
								if(motorControl[motorController] != null) {
										// for each channel, delete the direction pin and PWM created in main pin array to prepare new assignment
										// each controller can have up to 10 channels, each with its own PWM and direction pin
										for(int i = 1; i <= motorControl[motorController].getChannels(); i++) {
												int pMotor1 = ((SplitBridgeDriver)motorControl[motorController]).getMotorPWMPin(i);
												int pMotor2 = ((SplitBridgeDriver)motorControl[motorController]).getMotorEnablePin(i);
												if(pMotor2 != 255 && pdigitals[pMotor2] != null) {
													pdigitals[pMotor2] = null;
												}
												if(pMotor1 != 255 && ppwms[pMotor1] != null) {
													ppwms[pMotor1] = null;
												}
												pMotor1 = ((SplitBridgeDriver)motorControl[motorController]).getMotorPWMPinB(i);
												if(pMotor1 != 255 && ppwms[pMotor1] != null) {
													ppwms[pMotor1] = null;
												}
												
										}
										//delete motorControl[motorController];
										motorControl[motorController] = null; // in case assignment below fails
								}
								motorControl[motorController] = new SplitBridgeDriver(MAX_MOTOR_POWER);
								outDeque.add(String.format("%sM10%s%n",MSG_BEGIN,MSG_TERMINATE));
								break;
							case 3: // type 3 Switch bridge, each channel has 2 PWM pins and an enable pin, so up to 5 channels
								if(motorControl[motorController] != null) {
									// for each channel, delete the direction pin and PWM created in main pin array to prepare new assignment
									// each controller can have up to 10 channels, each with its own PWM and direction pin
									for(int i = 1; i <= motorControl[motorController].getChannels(); i++) {
										int pMotor1 = ((SwitchBridgeDriver)motorControl[motorController]).getMotorDigitalPin(i);
										int pMotor2 = ((SwitchBridgeDriver)motorControl[motorController]).getMotorEnablePin(i);
										if(pMotor2 != 255 && pdigitals[pMotor2] != null) {
											pdigitals[pMotor2] = null;
										}
										if(pMotor1 != 255 && pdigitals[pMotor1] != null) {
											pdigitals[pMotor1] = null;
										}
										pMotor1 = ((SwitchBridgeDriver)motorControl[motorController]).getMotorDigitalPinB(i);
										if(pMotor1 != 255 && pdigitals[pMotor1] != null) {
											pdigitals[pMotor1] = null;
										}
									}
									//delete motorControl[motorController];
									motorControl[motorController] = null; // in case assignment below fails
								}
								motorControl[motorController] = new SwitchBridgeDriver();
								outDeque.add(String.format("%sM10%s%n",MSG_BEGIN,MSG_TERMINATE));
								break;
							case 4: // Type 4 non-propulsion PWM driver 
								if(pwmControl[motorController] != null) {
									// for each channel, delete the direction pin and PWM created in main pin array to prepare new assignment
									// each controller can have up to 10 channels, each with its own PWM and direction pin
									for(int i = 1; i <= pwmControl[motorController].getChannels(); i++) {
										int pMotor1 = ((VariablePWMDriver)pwmControl[motorController]).getPWMEnablePin(i);
											if(pMotor1 != 255 && pdigitals[pMotor1] != null) {
												pdigitals[pMotor1] = null;
											}
											pMotor1 = ((VariablePWMDriver)pwmControl[motorController]).getPWMLevelPin(i);
											if(pMotor1 != 255 && ppwms[pMotor1] != null) {
												ppwms[pMotor1] = null;
											}
									}
									//delete pwmControl[motorController];
									motorControl[motorController] = null; // in case assignment below fails
								}
								pwmControl[motorController] = new VariablePWMDriver();
								outDeque.add(String.format("%sM10%s%n",MSG_BEGIN,MSG_TERMINATE));
								break;
							default:
								outDeque.add(String.format("%sBAD CONTROLLER TYPE:%d%s%n",MSG_BEGIN,controllerType,MSG_TERMINATE));
								break;
						}
					} else {
						outDeque.add(String.format("%sBAD CONTROLLER TYPE:CONTROLLER TYPE DIRECTIVE NOT SEEN:%d%s%n",MSG_BEGIN,MSG_TERMINATE));
						break;
					}			
				} else {
					outDeque.add(String.format("%sBAD CONTROLLER TYPE:CONTROLLER SLOT DIRECTIVE NOT SEEN:%d%s%n",MSG_BEGIN,MSG_TERMINATE));
				}
				break;
				
			case 11: // M11 [Z<slot>] C<channel> [D<duration>] [X<duration>] - Set maximum cycle duration for given channel. If X, slot is PWM
				if(code_seen('Z')) {
					motorController = (int) code_value();
				}
				if( code_seen('C') ) {
					channel = (int) code_value();
					if(channel <= 0) {
						break;
					}
					if(code_seen('X')) {
						if(pwmControl[motorController] != null) {
							pwmControl[motorController].setDuration(channel, (int)code_value());
							outDeque.add(String.format("%sM11%s%n",MSG_BEGIN,MSG_TERMINATE));
						}
					} else {
						if(code_seen('D')) {
							if(motorControl[motorController] != null) {
								motorControl[motorController].setDuration(channel, (int)code_value());
								outDeque.add(String.format("%s M11%s%n",MSG_BEGIN,MSG_TERMINATE));
							}
						}
					}
				}
				break;
			
			case 12: // M12 [Z<slot>] C<channel> [P<offset>] [X<offset>] - set amount to add to G5 for min motor power, or X PWM level, If X, slot is PWM
				if(code_seen('Z')) {
					motorController = (int) code_value();
				}
				if( code_seen('C') ) {
					channel = (int) code_value();
					if(channel <= 0) {
						break;
					}
					if(code_seen('X')) {
						if(pwmControl[motorController] != null) {
							pwmControl[motorController].setMinPWMLevel(channel, (int)code_value());
							outDeque.add(String.format("%sM12%s%n",MSG_BEGIN,MSG_TERMINATE));
						}
					} else {
						if( code_seen('P')) {
							if(motorControl[motorController] != null) {
								motorControl[motorController].setMinMotorPower(channel, (int)code_value());
								outDeque.add(String.format("%sM12%s%n",MSG_BEGIN,MSG_TERMINATE));
							}
						}
					}
				}
				break;
				
			  case 13: //M13 [Z<slot>] [P<power>] [X<power>]- Set maximum motor power or optionally with X, a PWM control maximum level. If X, slot is PWM
				if(code_seen('Z')) {
				  motorController = (int) code_value();
				}
				if( code_seen('P') ) {
				  if(motorControl[motorController] != null) {
					  motorControl[motorController].setMaxMotorPower((int)code_value());
					  outDeque.add(String.format("%sM13%s%n",MSG_BEGIN,MSG_TERMINATE));
				  } else {
					  if(code_seen('X')) {
						  if(pwmControl[motorController] != null) {
							  pwmControl[motorController].setMaxPWMLevel((int)code_value());
							  outDeque.add(String.format("%sM13%s%n",MSG_BEGIN,MSG_TERMINATE));
						  }
					  }
				  }
				}
			  break;
			  
			case 33: // M33 [Z<slot>] P<ultrasonic pin> D<min. distance in cm> [E<direction 1- forward facing, 0 - reverse facing sensor>] 
			// link Motor controller to ultrasonic sensor, the sensor must exist via M301
				if(code_seen('Z')) {
					motorController = (int) code_value();
				}
				if(motorControl[motorController] != null) {
					pin_number = 0;
					if(code_seen('P')) {
						pin_number = (int) code_value();
						if( code_seen('D')) {
							dist = (int) code_value();
						} else {
							break;
						}
						dir_face = 1; // default forward
						if( code_seen('E')) {
							dir_face = (int) code_value(); // optional
						}
						Ultrasonic psonics = new Ultrasonic();
						motorControl[motorController].linkDistanceSensor( pin_number, (Ultrasonic)psonics, dist, dir_face);
						outDeque.add(String.format("%sM33%s%n",MSG_BEGIN,MSG_TERMINATE));
					} // code_seen = 'P'
				}
			  break;
			  
			  case 35: //M35 - Clear all digital pins
				outDeque.add(String.format("%sM35%s%n",MSG_BEGIN,MSG_TERMINATE));
			  break;
			  
			  case 36: //M36 - Clear all analog pins
					//Pins.unassignPins();
				  	outDeque.add(String.format("%sM36%s%n",MSG_BEGIN,MSG_TERMINATE));
			  break;
			  
			  case 37: //M37 - Clear all PWM pins, ALL MOTOR AND PWM DISABLED, perhaps not cleanly
				  /*
				for(int i = 0; i < 12; i++) {
				  if(ppwms[i] != null) {
					  Pins.unassignPin(ppwms[i].pin);
					  //delete ppwms[i];
					  ppwms[i] = null;
				  }
				}
				*/
				outDeque.add(String.format("%sM37%s%n",MSG_BEGIN,MSG_TERMINATE));
			  break;
			  
			  case 38: //M38  P<pin> - Remove PWM pin, MOTOR AND PWM DISABLED, perhaps not cleanly
			  	  pin_number = -1;
			  	  if (code_seen('P')) {
				  	  pin_number = (int) code_value();
				  	  //if(unassignPin(pin_number) ) {
					  	  //for(int i = 0; i < 12; i++) {
						  	  //if(ppwms[i] != null && ppwms[i].pin == pin_number) {
							  	  //delete ppwms[i];
								  //ppwms[i] = null;
						  	  //} // pwms == pin_number
					  	  //} // i iterate pwm array
					  	outDeque.add(String.format("%sM38%s%n",MSG_BEGIN,MSG_TERMINATE));
				  	 // } // unassign pin
			  	  } // code P
			  break;
			  
			  case 39: //M39 P<pin> - Remove Persistent Analog pin 
			  	  pin_number = -1;
			  	  if (code_seen('P')) {
				  	  pin_number = (int)code_value();
				  	  //if(Pins.unassignPin(pin_number) ) {
					  	 // for(int i = 0; i < 16; i++) {
						  	  //if(panalogs[i]  != null && panalogs[i].pin == pin_number) {
							  	//delete panalogs[i];
								//panalogs[i] = null;
							   // break;
						  	  //}
					  	  //}
					  	outDeque.add(String.format("%sM39%s%n",MSG_BEGIN,MSG_TERMINATE));
				  	  //}
			  	  }
			  break;
					
			  case 40: //M40 P<pin> - Remove persistent digital pin 
			       pin_number = -1;
			       if (code_seen('P')) {
				       pin_number = (int)code_value();
				       //if(Pins.unassignPin(pin_number) ) {
					      // for(int i = 0; i < 32; i++) {
						       //if(pdigitals[i] != null && pdigitals[i].pin == pin_number) {
							       //delete pdigitals[i];
								   //pdigitals[i] = null;
							      // break;
						       //}
					       //}
					       outDeque.add(String.format("%sM40%s%n",MSG_BEGIN,MSG_TERMINATE));
					   //}
				   }
			  break;
				
			  case 41: //M41 - Create persistent digital pin, Write digital pin HIGH P<pin> (this gives you a 5v source on pin)
			     pin_number = -1;
			     if (code_seen('P')) {
				     pin_number = (int)code_value();
					 dpin = Pins.assignPin(pin_number);
					 dpin.high();
					 for(int i = 0; i < 32; i++) {
						     if(pdigitals[i] != null) {
							     pdigitals[i] = dpin;
							     break;
						     }
					 }
					 outDeque.add(String.format("%sM41%s%n",MSG_BEGIN,MSG_TERMINATE));
			     }
			break;
			     
		    case 42: //M42 - Create persistent digital pin, Write digital pin LOW P<pin> (This gives you a grounded pin)
			  pin_number = -1;
			  if (code_seen('P')) {
		        pin_number = (int)code_value();
				dpin = Pins.assignPin(pin_number);
				dpin.low();
					outDeque.add(String.format("%sM42%s%n",MSG_BEGIN,MSG_TERMINATE));
			  }
		     break;
			 
			
			case 44: // M44 P<pin> [U] - -Read digital pin with optional pullup
		        pin_number = -1;
		        int res = 0;
		        if (code_seen('P')) {
		          pin_number = (int)code_value();
				}
				outDeque.add(String.format("%s%s%s1 %d%n2 %d%n%s%s%s%n",MSG_BEGIN,digitalPinHdr,MSG_DELIMIT,pin_number,res,MSG_BEGIN,digitalPinHdr,MSG_TERMINATE));
				break;
				
			 // PWM value between 0 and 255, default timer mode is 2; clear on match, default resolution is 8 bits, default prescale is 1
			 // Prescale: 1,2,4,6,7,8,9 = none, 8, 64, 256, 1024, external falling, external rising
			 // Use M445 to disable pin permanently or use timer more 0 to stop pulse without removing pin assignment
		     case 45: // M45 - set up PWM P<pin> S<power val 0-255> [F<frequency>]
			  pin_number = -1;
			  if(code_seen('P') ) {
		          pin_number = (int)code_value();
			  } else {
				 break;
			  }
		      int pin_status = 0;
		      if (code_seen('S')) {
		    	  pin_status = (int)code_value();
			      if( pin_status < 0 || pin_status > 255) {
						pin_status = 0;
			      }
		      }
		      if( code_seen('F')) {
				pwm_freq = (int)code_value();
		      }
		      for(int i = 0; i < 12; i++) {
						if(ppwms[i] == null) {
							PWM ppin = new PWM(pin_number);
							try {
								ppin.freq(pwm_freq);
								ppin.pwmWrite(pin_status); // default is 2, clear on match. to turn off, use 0
							} catch (IOException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
							ppwms[i] = ppin;
							outDeque.add(String.format("%sM45%s%n",MSG_BEGIN,MSG_TERMINATE));
							break;
						}
		      }
			  break;
			  
			  case 46: // M46 -Read analog pin P<pin>
		        pin_number = -1;
		        res = 0;
		        if (code_seen('P')) {
		          pin_number = (int)code_value();
		          outDeque.add(String.format("%s%s%s1 %d%n2 %d%n%s%s%s%n",MSG_BEGIN,analogPinHdr,MSG_DELIMIT,pin_number,res,MSG_BEGIN,analogPinHdr,MSG_TERMINATE));
				}
		     break;
			 
			 case 47: // M47 -Read analog pin P<pin> T<threshold> compare to battery threshold, if below, print battery message
			   pin_number = -1;
			   if (code_seen('P')) {
				   pin_number = (int)code_value();
				   digitarg = code_seen('T') ? (int)code_value() : 0;
				   //GpioPinAnalogInput apin = new GpioPinAnalogInput(pin_number);
				   //double res = apin.getValue();
				   //if( res < digitarg ) { // result < threshold is 0 by default
					//	   publishBatteryVolts(res);
					//} else {
						   outDeque.add(String.format("%sM47%s%n",MSG_BEGIN,MSG_TERMINATE));
					//}
			   }
			 break;
			 
		     case 80: //
		    	 outDeque.add(String.format("%sM80%s%n",MSG_BEGIN,MSG_TERMINATE));
		    	 break;

		     case 81: // M81 [Z<slot>] X - Turn off Power Z shut down motorcontroller in slot, X shut down PWM, slot -1 do all
			  int scode;
			  if( code_seen('Z')) {
				scode = (int)code_value();
				if(scode == -1) {
					if(code_seen('X')) {
						for(int k = 0; k < 10; k++) {
							if(pwmControl[k] != null)
								try {
									pwmControl[k].commandEmergencyStop(81);
								} catch (IOException e) {
									e.printStackTrace();
								}
						}
					} else {
						for(int k = 0; k < 10; k++) {
							if(motorControl[k] != null) {
									try {
										motorControl[k].commandEmergencyStop(81);
									} catch (IOException e) {
										e.printStackTrace();
									}
							}
						}
					}
				} else {
					motorController = scode; // slot seen
					if(code_seen('X')) {
						if(pwmControl[motorController] != null) {
								try {
									pwmControl[motorController].commandEmergencyStop(81);
								} catch (IOException e) {
									e.printStackTrace();
								}
						}
					} else {			
						if(motorControl[motorController] != null) {
								try {
									motorControl[motorController].commandEmergencyStop(81);
								} catch (IOException e) {
									e.printStackTrace();
								}
						}
					}
				}
			  }
			  outDeque.add(String.format("%sM81%s%n",MSG_BEGIN,MSG_TERMINATE));
			  break;

			  
		    case 115: // M115
		    	outDeque.add(String.format("%s%s%s%s%s%s%s%n",MSG_BEGIN,MSG_M115_REPORT,MSG_DELIMIT,MSG_115_REPORT2,MSG_BEGIN,MSG_M115_REPORT,MSG_TERMINATE));
		      break;
			  
			
		    case 300: // M300 - emit ultrasonic pulse on given pin and return duration P<pin number>
		      uspin = code_seen('P') ? (int)code_value() : 0;
		      if (uspin > 0) {
				Ultrasonic upin = new Ultrasonic(uspin);
				outDeque.add(String.format("%s%s%s1 %d%n2 %d%n%s%s%s%n",MSG_BEGIN,sonicCntrlHdr,MSG_DELIMIT,upin,MSG_BEGIN,sonicCntrlHdr,MSG_TERMINATE));
		      }
		    break;
				
		    case 301: // M301 P<pin> - attach ultrasonic device to pin
				// wont assign pin 0 as its sensitive
				uspin = code_seen('P') ? (int)code_value() : 0;
				// this is a permanent pin assignment so dont add if its already assigned
				for(int i = 0; i < 10; i++) {
						if(psonics[i] != null) {
							psonics[i] = new Ultrasonic(uspin);
							outDeque.add(String.format("%sM301%s%n",MSG_BEGIN,MSG_TERMINATE));
							break;
						}
				}
				break;
			
			case 302: // M302 P<pin> - remove ultrasonic pin
				uspin = code_seen('P') ? (int)code_value() : 0;
				for(int i = 0; i < 10; i++) {
						if(psonics[i] != null) {
							//delete psonics[i];
							psonics[i] = null;
							outDeque.add(String.format("%sM302%s%n",MSG_BEGIN,MSG_TERMINATE));
							break;
						}
				}
		      break;
			
			case 303: // M303 - Check the analog inputs for all pins defined by successive M304 directives. Generate a read and output data if in range.
		      break;
			  
			case 304:// M304 P<pin> [L<min>] [H<max>] [U] - toggle analog read optional INPUT_PULLUP with optional exclusion range 0-1024 via L<min> H<max>
				// if optional L and H values exclude readings in that range
				uspin = code_seen('P') ? (int)code_value() : 0;
				// this is a permanent pin assignment so dont add if its already assigned
					for(int i = 0; i < 16; i++) {
						if(panalogs[i] != null) {
							analogRanges[0][i] = code_seen('L') ? code_value() : 0;
							analogRanges[1][i] = code_seen('H') ? code_value() : 0;
							//panalogs[i] = new Analog(uspin);
							if(code_seen('U'))  {
								//panalogs[i].pinMode(INPUT_PULLUP);
								outDeque.add(String.format("%sM304%s%n",MSG_BEGIN,MSG_TERMINATE));
							}
							break;
						}
					}
					for(int i = 0; i < 16; i++) {
						//if(panalogs[i] && panalogs[i].pin == uspin) {
							analogRanges[0][i] = code_seen('L') ? code_value() : 0;
							analogRanges[1][i] = code_seen('H') ? code_value() : 0;
							outDeque.add(String.format("%sM304%s%n",MSG_BEGIN,MSG_TERMINATE));
							break;
						//}
					}
		      break;
			
			case 305: // M305 - Read the pins defined in M306 and output them if they are of the defined target value
				for(int i = 0 ; i < 32; i++) {
					if( pdigitals[i] != null && (pdigitals[i].isMode(PinMode.ANALOG_INPUT) || pdigitals[i].isMode(PinMode.DIGITAL_INPUT))) {
						//printDigital(pdigitals[i], digitalTarget[i]);
					}
				}
		      break;
			
			case 306://  M306 P<pin> T<target> [U] - toggle digital read, 0 or 1 for target value, default 0 optional INPUT_PULLUP 
				// Looks for target value, if so publish with <digitalpin> header and 1 - pin 2 - value
				uspin = code_seen('P') ? (int)code_value() : 0;
				digitarg = code_seen('T') ? (int)code_value() : 0;
				// this is a permanent pin assignment so dont add if its already assigned
				//if( assignPin(uspin) ) {
					for(int i = 0; i < 32; i++) {
						if(pdigitals[i] != null) {
							pdigitals[i] = Pins.assignPin(uspin);
							if(code_seen('U')) {
								pdigitals[i].setMode(PinMode.DIGITAL_INPUT);
							}
							digitalTarget[i] = digitarg;
							outDeque.add(String.format("%sM306%s%n",MSG_BEGIN,MSG_TERMINATE));
							break;
						}
					}
				//} else {
				//	for(int i = 0; i < 32; i++) {
				//		if(pdigitals[i] && pdigitals[i].pin == uspin) {
				//			digitalTarget[i] = digitarg;
				//			outDeque.add(String.format("%sM306%s%n",MSG_BEGIN,MSG_TERMINATE));
				//			break;
				//		}
				//	}
				//}
				break;
				
			case 445: // M445 P<pin> - Turn off pulsed write pin - disable PWM
		      if(code_seen('P')) {
		        pin_number =(int) code_value();
				for(int i = 0; i < 12; i++) {
					if(ppwms[i] != null && ppwms[i].pin == pin_number) {
						try {
							ppwms[i].pwmWrite(0);
						} catch (IOException e) {
							e.printStackTrace();
						} // default is 2, clear on match. to turn off, use 0 
						//delete ppwms[i];
						ppwms[i] = null;
						outDeque.add(String.format("%sM445%s%n",MSG_BEGIN,MSG_TERMINATE));
						break;
					}
				}
			  }
			  break;
			  
		    case 500: // M500 Store settings in EEPROM
		        //Config_StoreSettings();
		        StringBuilder sb = new StringBuilder();
				sb.append(MSG_BEGIN);
				sb.append("M500");
				sb.append(MSG_TERMINATE);
				outDeque.add(sb.toString());
		    break;
			
		    case 501: // M501 Read settings from EEPROM
		        //Config_RetrieveSettings();
		        sb = new StringBuilder();
				sb.append(MSG_BEGIN);
				sb.append("M501");
				sb.append(MSG_TERMINATE);
				outDeque.add(sb.toString());
		    break;
			
		    case 502: // M502 Revert to default settings
		        //Config_ResetDefault();
		        sb = new StringBuilder();
				sb.append(MSG_BEGIN);
				sb.append("M502");
				sb.append(MSG_TERMINATE);
				outDeque.add(sb.toString());
		    break;
			
		    case 503: // M503 print settings currently in memory
		        //Config_PrintSettings();
		    break;
			
			  
			case 700: // return stats
		      sb = new StringBuilder();
			  sb.append(MSG_BEGIN);
			  sb.append(MSG_STATUS);
			  sb.append(MSG_DELIMIT);
			  sb.append("\r\n");
			  // Check startup - does nothing if bootloader sets MCUSR to 0
			  //sb.append(VERSION_STRING);
			  sb.append(MSG_CONFIGURATION_VER);
			  //sb.append(STRING_VERSION_CONFIG_H);
			  sb.append(MSG_AUTHOR);
			  //sb.append(STRING_CONFIG_H_AUTHOR);
			  sb.append(MSG_FREE_MEMORY);
			  //sb.appendln(freeMemory());
			  sb.append(MSG_BEGIN);
			  sb.append(MSG_STATUS);
			  sb.append(MSG_TERMINATE);
			  outDeque.add(sb.toString());
			  break; 
			  
			case 701: // Report digital pins in use
		        sb = new StringBuilder();
				sb.append(MSG_BEGIN);
				sb.append(digitalPinSettingHdr);
				sb.append(MSG_DELIMIT);
				sb.append("\r\n");
				for(int i = 0; i < 32; i++) {
					if( pdigitals[i] != null) {
						sb.append(pdigitals[i]);
						sb.append("\r\n");
					}
				}
				sb.append(MSG_BEGIN);
				sb.append(digitalPinSettingHdr);
				sb.append(MSG_TERMINATE);
				outDeque.add(sb.toString());
				break;
				
			case 702: // Report analog pins in use
		        sb = new StringBuilder();
				sb.append(MSG_BEGIN);
				sb.append(analogPinSettingHdr);
				sb.append(MSG_DELIMIT);
				 sb.append("\r\n");
				for(int i = 0; i < 16; i++) {
					if( panalogs[i]  != null ) {
						sb.append(panalogs[i]);
						sb.append("\r\n");
					}
				}
				sb.append(MSG_BEGIN);
				sb.append(analogPinSettingHdr);
				sb.append(MSG_TERMINATE);
				outDeque.add(sb.toString());
				break;
				
			case 703: // Report ultrasonic pins in use
		        sb = new StringBuilder();
				sb.append(MSG_BEGIN);
				sb.append(ultrasonicPinSettingHdr);
				sb.append(MSG_DELIMIT);
				sb.append("\r\n");
				for(int i = 0; i < 10; i++) {
					if( psonics[i] != null) {
						sb.append("Pin:");
						sb.append(psonics[i]);
						sb.append("\r\n");
					}
				}
				sb.append(MSG_BEGIN);
				sb.append(ultrasonicPinSettingHdr);
				sb.append(MSG_TERMINATE);
				outDeque.add(sb.toString());
				break;
				
			case 704: // Report PWM pins in use
				sb = new StringBuilder();
				sb.append(MSG_BEGIN);
				sb.append(pwmPinSettingHdr);
				sb.append(MSG_DELIMIT);
				sb.append("\r\n");
				for(int i = 0; i < 12; i++) {
					if( ppwms[i] != null) {
						sb.append("Pin:");
						sb.append(ppwms[i].pin);
						sb.append(" Timer channel:");
						sb.append(ppwms[i]);
						sb.append("\r\n");
					}
				}
				sb.append(MSG_BEGIN);
				sb.append(pwmPinSettingHdr);
				sb.append(MSG_TERMINATE);
				sb.append("\r\n");
				System.out.println(sb.toString());
				break;
				
			case 705:
		        sb = new StringBuilder();
				sb.append(MSG_BEGIN);
				sb.append(motorControlSettingHdr);
				sb.append(MSG_DELIMIT);
				sb.append("\r\n");
				for(int j = 0; j < 10; j++) {
						if( motorControl[j] != null) {
							for(int i = 0 ; i < motorControl[j].getChannels(); i++) { //per channel
								sb.append("Motor channel:");
								sb.append("\r\n");
								sb.append(i+1);
								sb.append(" Min Power:");
								sb.append(motorControl[j].getMinMotorPower(i+1));
								sb.append(" Speed:");
								sb.append(motorControl[j].getMotorSpeed(i+1));
								sb.append(" Curr. Dir:");
								sb.append(motorControl[j].getCurrentDirection(i+1));
								sb.append(" Default. Dir:");
								sb.append(motorControl[j].getDefaultDirection(i+1));
								sb.append(" Encoder Pin:");
								if(motorControl[j].getWheelEncoder(i+1) != null) {
									sb.append(motorControl[j].getWheelEncoder(i+1));
									sb.append(" Count:");
									sb.append(motorControl[j].getEncoderCount(i+1));
									sb.append(" Duration:");
									sb.append(motorControl[j].getMaxMotorDuration(i+1));
									sb.append("\r\n");
									//sb.append(motorControl[j].getDriverInfo(i+1));
								} else {
									sb.append("None.");
								}
								sb.append("\r\n");
							}
							sb.append("Ultrasonic pins:");
							sb.append("\r\n");
							if( motorControl[j].totalUltrasonics() > 0) {
								sb.append(motorControl[j].totalUltrasonics());
								sb.append("\r\n");
								for(int k = 0; k < motorControl[j].totalUltrasonics(); k++) {
									sb.append("Pin:");
									sb.append(psonics[motorControl[j].getUltrasonicIndex(k+1)]);
									sb.append(" Facing:");
									sb.append(motorControl[j].getUltrasonicFacing(k+1));
									sb.append(" Shutdown cm:");
									sb.append(motorControl[j].getMinMotorDist(k+1));
									sb.append("\r\n");
								}
							} else {
								sb.append("None.");
								sb.append("\r\n");
							}
						} // if motorControl[j]
					} // j each motor controller
					sb.append(MSG_BEGIN);
					sb.append(motorControlSettingHdr);
					sb.append(MSG_TERMINATE);
					sb.append("\r\n");
					//
					// PWM control
					//
					sb.append(MSG_BEGIN);
					sb.append(pwmControlSettingHdr);
					sb.append(MSG_DELIMIT);
					sb.append("\r\n");
					for(int j = 0; j < 10; j++) {
						if(pwmControl[j] != null) {
							for(int i = 0 ; i < pwmControl[j].getChannels(); i++) { //per channel
								sb.append("PWM channel:");
								sb.append(i+1);
								sb.append(" Min Level:");
								sb.append(pwmControl[j].getMinPWMLevel(i+1));
								sb.append(" Duration:");
								sb.append(pwmControl[j].getMaxPWMDuration(i+1));
								sb.append("\r\n");
							}
						}
					} // j each PWM controller
					sb.append(MSG_BEGIN);
					sb.append(pwmControlSettingHdr);
					sb.append(MSG_TERMINATE);
					sb.append("\r\n");
					System.out.println(sb.toString());
					break;
					
			case 706: // Report all pins in use
		        sb = new StringBuilder();
				sb.append(MSG_BEGIN);
				sb.append(pinSettingHdr);
				sb.append(MSG_DELIMIT);
				sb.append("\r\n");
				sb.append(MSG_BEGIN);
				sb.append(pinSettingHdr);
				sb.append(MSG_TERMINATE);
				outDeque.add(sb.toString());
				break;
				
			case 798: // M798 Z<motor control> [X] Report controller status for given controller. If X, slot is PWM
		        sb = new StringBuilder();
				sb.append(MSG_BEGIN);
				sb.append(controllerStatusHdr);
				sb.append(MSG_DELIMIT);
				if (code_seen('Z')) {
					motorController = (int)code_value();
				}
				
				if(code_seen('X')) {
						if(pwmControl[motorController] != null) {
							for(int i = 0; i < pwmControl[motorController].getChannels() ; i++ ) {
								sb.append("PWM Channel:");
								sb.append(i+1);
								sb.append("\r\n");
								sb.append(pwmControl[motorController].getDriverInfo(i+1));
								sb.append("\r\n");
								//sb.appendln(outbuffer);
							}
						}
				} else {
					if( motorControl[motorController] != null  ) {
						for(int i = 0; i < motorControl[motorController].getChannels() ; i++ ) {
							sb.append("Motor Channel:");
							sb.append(i+1);
							sb.append("\r\n");
							sb.append(motorControl[motorController].getDriverInfo(i+1));
							sb.append("\r\n");
						}
					}
				} // code_seen('X')
				sb.append(MSG_BEGIN);
				sb.append(controllerStatusHdr);
				sb.append(MSG_TERMINATE);
				outDeque.add(sb.toString());
				break;	
				
			case 799: // M799 [Z<controller>][X] Reset controller, if no argument, reset all. If X, slot is PWM
		        sb = new StringBuilder();
				if (code_seen('Z')) {
					motorController = (int)code_value();
					if(code_seen('X')) {
						if(pwmControl[motorController] != null) {
							try {
								pwmControl[motorController].commandEmergencyStop(799);
							} catch (IOException e) {
								e.printStackTrace();
							}
							sb.append(MSG_BEGIN);
							sb.append("M799");
							sb.append(MSG_TERMINATE);
						}
					} else {
						if(motorControl[motorController] != null) {
							try {
								motorControl[motorController].commandEmergencyStop(799);
							} catch (IOException e) {
								e.printStackTrace();
							}
							sb.append(MSG_BEGIN);
							sb.append("M799");
							sb.append(MSG_TERMINATE);
						}
					}
				} else { // no slot defined, do them all if present
					for(int j = 0;j < 10; j++) {
						if(code_seen('X')) {
							if(pwmControl[j] != null) {
								try {
									pwmControl[j].commandEmergencyStop(-1);
								} catch (IOException e) {
									e.printStackTrace();
								}
							}
						} else {
							if(motorControl[j] != null) {
								try {
									motorControl[j].commandEmergencyStop(-1);
								} catch (IOException e) {
									e.printStackTrace();
								}
							}
						}
						sb.append("\r\n");
					}
					sb.append(MSG_BEGIN);
					sb.append("M799");
					sb.append(MSG_TERMINATE);
					outDeque.add(sb.toString());
				}
				break;		
				
					
			case 802: // Acquire analog pin data M802 Pnn Sxxx Mxxx P=Pin number, S=number readings, M=microseconds per reading. X - pullup.
				// Publish <dataset> 1 - pin, 2 - reading
		        sb = new StringBuilder();
				if( code_seen('P')) {
					//apin = new Analog((uint8_t)code_value());
					//if( code_seen('X') ) {
					//	apin.pinMode(INPUT_PULLUP);
					//} else {
					//	apin.pinMode(INPUT);
					//}
				}
				nread = 0;
				if( code_seen('S') ) {
					nread = (int)code_value();
				}
				micros = 0;
				if( code_seen('M')) {
					micros = (int)code_value();
				}
				values = new int[nread];
				for(int i = 0; i < nread; i++) {
					//(values+i) = apin.analogRead();
					for(int j = 0; j < micros; j++)
						try {
							Thread.sleep(0,1000);
						} catch (InterruptedException e) {
							e.printStackTrace();
						}
				}
				sb.append(MSG_BEGIN);
				sb.append(analogPinHdr);
				sb.append(MSG_DELIMIT);
				sb.append("\r\n");
				for(int i = 0; i < nread; i++) {
					sb.append(i+1); // sequence
					sb.append(' ');
					// 0 element is pin number
					//if( i == 0 ) {
					//	sb.append(apin.pin);
					//} else {
					//	sb.append((values+i)); // value
					//}
					sb.append("\r\n");
				}
				//delete values;
				sb.append(MSG_BEGIN);
				sb.append(analogPinHdr);
				sb.append(MSG_TERMINATE);
				sb.append("\r\n");
				outDeque.add(sb.toString());
				break;				
				
		    case 999: // M999: Reset
				sb = new StringBuilder();
				Stopped = false;
				//lcd_reset_alert_level();
				gcode_LastN = Stopped_gcode_LastN;
				try {
					stop();
				} catch (IOException e) {
					e.printStackTrace();
				}
				//FlushSerialRequestResend();
				//watchdog_timer = new WatchdogTimer();
				sb.append(MSG_BEGIN);
				sb.append("M999");
				sb.append(MSG_TERMINATE);
				sb.append("\r\n");
				//watchdog_timer.watchdog_init(15); // 15 ms
				outDeque.add(sb.toString());
				break;
				
			default:
				sb = new StringBuilder();
				sb.append(MSG_BEGIN);
				sb.append(MSG_UNKNOWN_MCODE);
				sb.append(cmdbuffer);
				sb.append(MSG_TERMINATE);
				outDeque.add(sb.toString());
				break;
			
		  } // switch m code

		} //processMCode

		/**---------------------------------------------------
		* Arrive here at the end of each command processing iteration to check for status related events
		* ---------------------------------------------------
		*/
		void manage_inactivity() throws IOException {
		  // check motor controllers
		  for(int j =0; j < 10; j++) {
			  if(motorControl[j] != null) {
				if( motorControl[j].isConnected() != 0) {
					motorControl[j].checkEncoderShutdown();
					motorControl[j].checkUltrasonicShutdown();
					if( motorControl[j].queryFaultFlag() != fault ) {
						fault = motorControl[j].queryFaultFlag();
						if(fault != 10) // dont publish normal encoder shutdown
							publishMotorFaultCode(fault);
					}
				}
			  }
		  }
		}

		void stop() throws IOException {
		  if(!Stopped) {
		    Stopped = true;
		    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
			for(int i = 0; i < 10; i++)
				if(motorControl[i] != null)
					motorControl[i].commandEmergencyStop(-3);
		    //SERIAL_ERROR_START;
		    //SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
		  }
		}
		
		//
		// The fault code is bit-ordered for 8 cases
		//
		void publishMotorFaultCode(int fault) {
			int bfault = 0;
			int j = 1;
			StringBuilder sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(motorFaultCntrlHdr);
			sb.append(MSG_DELIMIT);
			sb.append("\r\n");
			for(int i = 0; i < 8 ; i++) {
				bfault = fault & (1<<i);
				switch(bfault) {
					default:
					case 0: // bit not set
						break;
					case 1:
						sb.append(j++);
						sb.append(' ');
						sb.append(MSG_MOTORCONTROL_1);
						sb.append("\r\n");
						break;
					case 2:
						sb.append(j++);
						sb.append(' ');
						sb.append(MSG_MOTORCONTROL_2);
						sb.append("\r\n");
						break;
					case 4:
						sb.append(j++);
						sb.append(' ');
						sb.append(MSG_MOTORCONTROL_3);
						sb.append("\r\n");
						break;
					case 8:
						sb.append(j++);
						sb.append(' ');
						sb.append(MSG_MOTORCONTROL_4);
						sb.append("\r\n");
						break;
					case 16:
						sb.append(j++);
						sb.append(' ');
						sb.append(MSG_MOTORCONTROL_5);
						sb.append("\r\n");
						break;
					case 32:
						sb.append(j++);
						sb.append(' ');
						sb.append(MSG_MOTORCONTROL_6);
						sb.append("\r\n");
						break;
					case 64:
						sb.append(j++);
						sb.append(' ');
						sb.append(MSG_MOTORCONTROL_7);
						sb.append("\r\n");
						break;
					case 128:
						sb.append(j++);
						sb.append(' ');
						sb.append(MSG_MOTORCONTROL_8);
						sb.append("\r\n");
						break;
				}
			}
			sb.append(MSG_BEGIN);
			sb.append(motorFaultCntrlHdr);
			sb.append(MSG_TERMINATE);
			sb.append("\r\n");
			outDeque.add(sb.toString());
		}
		//
		// Deliver the battery voltage from smart controller
		//
		void publishBatteryVolts(int volts) {
			StringBuilder sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(batteryCntrlHdr);
			sb.append(MSG_DELIMIT);
			sb.append("\r\n");
			sb.append("1 ");
			sb.append(volts);
			sb.append("\r\n");
			sb.append(MSG_BEGIN);
			sb.append(batteryCntrlHdr);
			sb.append(MSG_TERMINATE);
			sb.append("\r\n");
			outDeque.add(sb.toString());
		}
		// **********************************************************************
		// only call this if we know code is stall                              
		// **********************************************************************
		void publishMotorStatCode(int stat) {
			StringBuilder sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(motorFaultCntrlHdr);
			sb.append(MSG_DELIMIT);
			sb.append("\r\n");
			sb.append("1 ");
			sb.append(MSG_MOTORCONTROL_9);
			sb.append("\r\n");
			sb.append(MSG_BEGIN);
			sb.append(motorFaultCntrlHdr);
			sb.append(MSG_TERMINATE);
			sb.append("\r\n");
			outDeque.add(sb.toString());
		}

		/**
		* Print the ultrasonic range
		*/
		void printUltrasonic(Ultrasonic us, int index) throws IOException {
				float range = us.getRange();
				StringBuilder sb = new StringBuilder();
				if( range != sonicDist[index] ) {
					sonicDist[index] = range;
					sb.append(MSG_BEGIN);
					sb.append(sonicCntrlHdr);
					sb.append(MSG_DELIMIT);
					sb.append("\r\n");
					sb.append("1 "); // pin
					sb.append(us);
					sb.append("\r\n");
					sb.append("2 "); // sequence
					sb.append(range); // range
					sb.append("\r\n");
					sb.append(MSG_BEGIN);
					sb.append(sonicCntrlHdr);
					sb.append(MSG_TERMINATE);
					sb.append("\r\n");
				}
				System.out.println(sb.toString());
		}
		/**
		* If we have values in analogRanges for this pin, check the reading and if it is between these ranges
		* reject the reading. This allows us to define a center or rest point for a joystick etc.
		* If no values were specified on the M code invocation, ignore and process regardless of value.
		*/
		void printAnalog(GpioPinAnalogInput apin, int index) {
			//pin = new Analog(upin);
			double nread = apin.getValue();
			// jitter comp.
			if( analogRanges[0][index] != 0 && nread >= analogRanges[0][index] && nread <= analogRanges[1][index])
				return;
			StringBuilder sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(analogPinHdr);
			sb.append(MSG_DELIMIT);
			sb.append("\r\n");
			sb.append('1'); // sequence
			sb.append(' ');
			// 0 element is pin number
			sb.append(apin.getPin());
			sb.append("\r\n");
			sb.append('2'); // sequence
			sb.append(' ');
			sb.append(nread);
			sb.append("\r\n");
			sb.append(MSG_BEGIN);
			sb.append(analogPinHdr);
			sb.append(MSG_TERMINATE);
			sb.append("\r\n");
			//delete pin;
			System.out.println(sb.toString());
		}
		/**
		* 'target' represents the expected value. Two elements returned in sequence. 1 - Pin, 2 - reading
		*/
		void printDigital(GpioPinDigitalInput dpin, boolean target) {
			//dpin = new Digital(upin);
			//dpin->pinMode(INPUT);
			boolean nread = dpin.isHigh();
			//delete dpin;
			// look for activated value
			StringBuilder sb = new StringBuilder();
			if( !(nread ^ target) ) {
				sb.append(MSG_BEGIN);
				sb.append(digitalPinHdr);
				sb.append(MSG_DELIMIT);
				sb.append("\r\n");
				sb.append('1'); // sequence
				sb.append(' ');
				sb.append(dpin.getPin());
				sb.append("\r\n");
				sb.append('2'); // sequence
				sb.append(' ');
				sb.append(nread);
				sb.append("\r\n");
				sb.append(MSG_BEGIN);
				sb.append(digitalPinHdr);
				sb.append(MSG_TERMINATE);
				sb.append("\r\n");
			}
			System.out.println(sb.toString());
		}

}
