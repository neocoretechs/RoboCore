package com.neocoretechs.robocore.serialreader;

import java.io.IOException;
import java.util.ArrayList;

import com.neocoretechs.robocore.propulsion.PWM;
import com.neocoretechs.robocore.serialreader.marlinspikeport.Pins;
import com.neocoretechs.robocore.serialreader.marlinspikeport.control.AbstractMotorControl;
import com.neocoretechs.robocore.serialreader.marlinspikeport.control.RoboteqDevice;
import com.neocoretechs.robocore.serialreader.marlinspikeport.control.SplitBridgeDriver;
import com.neocoretechs.robocore.serialreader.marlinspikeport.control.SwitchBridgeDriver;
import com.neocoretechs.robocore.serialreader.marlinspikeport.control.SwitchHBridgeDriver;
import com.neocoretechs.robocore.serialreader.marlinspikeport.pwmcontrol.AbstractPWMControl;
import com.neocoretechs.robocore.serialreader.marlinspikeport.pwmcontrol.DelayedHBridgeDriver;
import com.neocoretechs.robocore.serialreader.marlinspikeport.pwmcontrol.HBridgeDriver;
import com.neocoretechs.robocore.serialreader.marlinspikeport.pwmcontrol.VariablePWMDriver;


/**
 * RoboCore robotic controller platform to SBC (Odroid, RPi) GPIO headers interface to object model.
 * Implementation of arduino type microcontroller firmware to Java asynchronous process.<p/>
 *
 * Like the original firmware, processes a variation of M and G code from CNC and 3D printing to control a range of motor controllers and drivers
 * and GPIO pins for smart machine and robotics hardware functionality.<p/>
 *
 * Previously geared toward the Mega2560 microcontroller chip, this code unifies the microcontroller platform and allows it to be easily accessed through
 * using 'standard' M and G codes from CNC. <p/>
 *
 * Unifies the object model with header pins for GPIO, analog inputs, and PWM hardware generation, and allows
 * various motor controllers like H-bridge, half bridge, and smart controllers to be controlled through UART, PWM, and GPIO functions.<p/>
 *
 * Instead of a main processing loop that receives M and G code commands a la USB attached Arduino, we call the command interface
 * through a method and receive a result response as a return value. The {@code AsynchDemuxer} then processes the return as it would
 * an attached microcontroller.<p/>
 * 
 * Ultrasonic sensors and wheel encoders can be attached to any motor driver through M code and used to
 * set up a minimum distance to detect an object before issuing a command to shut down the motor driver, or different M codes
 * can set up encoders to detect a number
 * of encoder pulses before generating an interrupt. Encoder signals can be defined as analog or digital inputs.<p/>
 *
 * Example encoders are hall effect sensors that can be attached to any motor driver through pins designated in the main
 * processing loop and used to detect wheel rotations, then set up pin change interrupts for those sensors through other
 * M code commands, then perform actions based on the wheel rotations and the interrupts generated and processed through the object model.<p/>
 *
 * Different global states can be established, like when faults encountered, 'stopped' is true, and
 * the Gcodes G0-G5 are ignored as a safety interlock.<p/>
 * 
 * The usual use case is to create a type of controller in the object model using M10 Z<slot> T,controller type> code variations, 
 * then reference that controller using the Z slot number parameter and C channel option in other M and G codes<p/>
 * Distinction is made between basic PWM controls and full motor controls. So many commands refer to 'PWM control' vs
 * 'Motor control' and they occupy different 'slots'. A basic PWM control has no encoders or interrupts and no forward/reverse direction. <p/>
 * 
 * Examples:<br/>
 * G5  - Command motor or PWM control C<Channel> [P<Motor Power -1000,1000>] [X<PWM level -1000,1000 scaled 0-2000>] <br/>
 * G5 Z0 C1 P500 <br/>
 * M10 - Central configuration directive for creating all types of motor and PWM controls and drivers M10 Z<slot> T<controller type> <br/>
 * M10 Z0 T1 <br/>
 * M700 - RoboCore - Retrieve startup params <br/>
 * M705 - Display Motor controller and channel attributes <br/>
 * M798 - Report status of attached controller <br/>
 * M999 - Restart after being stopped by error, clears 'stopped' flag <br/>
 * Responses are consumed by the calling process, typically in the form <MCode/> for success and standard error headers with the same </> delimiter for failure.
 * @author Jonathan Neville Groff Copyright (C) NeoCoreTechs 2020
*/
public class MarlinspikeDataPort implements DataPortCommandInterface {
	public static boolean DEBUG = true;
	private static final int MAX_CMD_SIZE = 1024;
	public static int DEFAULT_PWM_FREQUENCY = 50000;
	public static int DEFAULT_PWM_DUTY = 25000;
	public static int MAX_MOTOR_POWER = 1000;
	static int gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

	static boolean realtime_output = true; // Determines whether real time data from inactive period is streamed

	String cmdbuffer;
	String[] cmdtokens = new String[25];
	String outbuffer;
	char serial_char;

	int serial_read;
	int serial_count = 0;

	//Inactivity shutdown variables
	long previous_millis_cmd = 0;
	long max_inactive_time = 0;

	long starttime = 0;
	long stoptime = 0;

	boolean Stopped=false;
	boolean target_direction;
	
	//private GpioPinDigitalOutput dpin;
	//private GpioPinDigitalInput ipin;
	int nread = 0;
	long micros = 0;
	int[] values;
	String motorCntrlResp;
	int status;
	int fault = 0;
	// Dynamically defined ultrasonic rangers
	static Ultrasonic[] psonics = new Ultrasonic[10];
	// Last distance published per sensor
	static float[] sonicDist = new float[]{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
	// Dynamically defined analog pins
	static double[][] analogRanges = new double[2][16];
	static PWM[] panalogs = new PWM[10];//{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	// Dynamically defined digital pins
	static boolean[] digitalTarget = new boolean[12];
	// PWM control block
	static PWM[] ppwms = new PWM[12];//{0,0,0,0,0,0,0,0,0,0,0,0};
	static int pwm_freq = DEFAULT_PWM_FREQUENCY;
	static int pwm_duty = DEFAULT_PWM_DUTY;

	// &roboteqDevice, new HBridgeDriver, new SplitBridgeDriver...
	static AbstractMotorControl[] motorControl = new AbstractMotorControl[10];
	static AbstractPWMControl[] pwmControl= new AbstractPWMControl[10];
	int channel;
	  
	int digitarg;
	static int uspin = 0;
	long t;
	int pin_number, pin_numberB;
	int dir_pin, dir_default, enable_pin;
	int encode_pin = 0;
	int interrupt_pin = 0;
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
	final static String MALFORMED_GCODE= "Malformed G code ";
	final static String MALFORMED_MCODE= "Malformed M code ";
	final static String MSG_BAD_MOTOR ="Bad Motor command ";
	final static String MSG_BAD_PWM= "Bad PWM Driver command ";
	final static String MSG_NO_CONTROL = "Attempt to configure undefined device ";

	@Override
	/**
	 * Null method
	 */
	public void connect(boolean writeable) throws IOException {
	}
	
	@Override
	public boolean isConnected() {
		return true;
	}
	
	@Override
	public int read() throws IOException {
			return -1;
	}

	@Override
	public void write(int c) throws IOException {
	}

	@Override
	public void close() {
		try {
			stop();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	@Override
	public String readLine() {
		return null;
	}

	@Override
	public int bytesToRead() throws IOException {
		return -1;
	}

	@Override
	public void writeLine(String output) throws IOException {
	}

	@Override
	public String getPortName() {
		return this.toString();
	}

	@Override
	public String stringSettings() {
		return getPortName();
	}

	@Override
	public ArrayList<String> sendCommand(String command) throws IOException {
		cmdbuffer = command;
		get_command();
		return process_commands();
	}
	 
	void get_command() {
		serial_count = 0;
		serial_char = (char)cmdbuffer.charAt(0);
		if(serial_char == '\n' || serial_char == '\r' || serial_char == '#' ||  serial_char == ';' || serial_count >= (MAX_CMD_SIZE - 1) ) {
			return;
		}
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
			System.out.printf("%s get_command %s%n", this.getClass().getName(), cmdbuffer);
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
	ArrayList<String> process_commands() throws IOException {
		ArrayList<String> ret = new ArrayList<String>();
		if(code_seen('G')) {
			int cval = (int)code_value();
			return processGCode(cval);
		} else {
			if(code_seen('M') ) {
				int cval = (int)code_value();
				return processMCode(cval);
			}
		}
		// if neither G nor M code
		ret.add("Neither G nor M code encountered in command:"+cmdbuffer);
		return ret;
	}

	/**
	 * Processing of G-code command sequence
	 * @param cval
	 */
	ArrayList<String> processGCode(int cval) throws IOException {
		ArrayList<String> ret = new ArrayList<String>();
		if(DEBUG)
			System.out.printf("%s processGCode %s%n", this.getClass().getName(), String.valueOf(cval));
		switch(cval) {    
		case 4: // G4 dwell
			codenum = 0;
			if(code_seen('P')) codenum = (int) code_value(); // milliseconds to wait
			if(code_seen('S')) codenum = (int) (code_value() * 1000); // seconds to wait
			codenum += System.currentTimeMillis();  // keep track of when we started waiting
			previous_millis_cmd = System.currentTimeMillis();
			while(previous_millis_cmd  < codenum ) {
				manage_inactivity();
				try {
					Thread.sleep(1);
					previous_millis_cmd = System.currentTimeMillis();
				} catch (InterruptedException e) {
					break;
				}
			}
			ret.add(String.format("%sG4%s%n",MSG_BEGIN,MSG_TERMINATE));
			return ret;
			//
			// G5 [Z<controller>] C<Channel> [P<motor power -1000 to 1000>] [X<PWM power -1000 to 1000>(scaled 0-2000)]
			// Primary code to command power level to a given controller on a given channel.
			//
		case 5:
			if(!Stopped) {
				if(code_seen('Z')) {
					motorController = (int) code_value();
				}
				if(code_seen('C')) {
					motorChannel = (int) code_value(); // channel 1,2
					if(code_seen('P')) {
						motorPower = (int) code_value(); // motor power -1000,1000
						fault = 0; // clear fault flag
						if(DEBUG)
							System.out.printf("%s Command Motor Power control %s for slot %d%n", this.getClass().getName(), motorControl[motorController], motorController);
						if( (status=motorControl[motorController].commandMotorPower(motorChannel, motorPower)) != 0) {
							ret.add(String.format("%s%s%d %d %d%s%n",MSG_BEGIN,MSG_BAD_MOTOR,status,motorChannel,motorPower,MSG_TERMINATE));
						} else {
							ret.add(String.format("%sG5%s%n",MSG_BEGIN,MSG_TERMINATE));
						}
						if(DEBUG && status != 0)
							System.out.printf("%s Commanded Motor Power control %s for slot %d status was %d%n", this.getClass().getName(), motorControl[motorController], motorController, status);
						return ret;
					} else {// code P or X
						if(code_seen('X')) {
							PWMLevel = (int) code_value(); // PWM level -1000,1000, scaled to 0-2000 in PWM controller, as no reverse
							fault = 0; // clear fault flag
							if(DEBUG)
								System.out.printf("%s Command PWM Level control %s for slot %d%n", this.getClass().getName(), pwmControl[motorController], motorController);
							// use motor related index and value, as we have them
							if( (status=pwmControl[motorController].commandPWMLevel(motorChannel, PWMLevel)) != 0) {
								ret.add(String.format("%s%s%d %d %d%s%n",MSG_BEGIN,MSG_BAD_PWM,status,motorChannel,PWMLevel,MSG_TERMINATE));
							} else {
								ret.add(String.format("%sG5%s%n",MSG_BEGIN,MSG_TERMINATE));
							}
							if(DEBUG && status != 0)
								System.out.printf("%s Commanded PWM Level control %s for slot %d status was %d%n", this.getClass().getName(), pwmControl[motorController], motorController, status);
							return ret;
						} // code X
					}
				} // code C
			} // stopped
			break;
			//
			// G99 start watchdog timer. G99 T<time_in_millis> values are 15,30,60,120,250,500,1000,4000,8000 default 4000
			//
		case 99:
			if( code_seen('T') ) {
				//int time_val = (int) code_value();
				//watchdog_timer = new WatchdogTimer();
				ret.add(String.format("%sG99%s%n",MSG_BEGIN,MSG_TERMINATE));
				return ret;
				//watchdog_timer.watchdog_init(time_val);
			}
			break;
			//
			// G100 reset watchog timer before time interval is expired, otherwise a reset occurs
			//
		case 100:
			//if( watchdog_timer != null ) {
			//	watchdog_timer.watchdog_reset();
			//}
			ret.add(String.format("%sG100%s%n",MSG_BEGIN,MSG_TERMINATE));
			return ret;
		default:
			ret.add(String.format("%s%s%s%s%n",MSG_BEGIN,MSG_UNKNOWN_GCODE,cmdbuffer,MSG_TERMINATE));
			return ret;
		} // switch
		ret.add(String.format("%s%s:%s%s%n",MSG_BEGIN,MALFORMED_GCODE,cmdbuffer,MSG_TERMINATE));
		return ret;
	}

	/**
	 * Process M codes
	 * @param cval
	 */
	ArrayList<String> processMCode(int cval) {
		ArrayList<String> ret = new ArrayList<String>();
		if(DEBUG)
			System.out.printf("%s processMCode %s%n", this.getClass().getName(), String.valueOf(cval));
		int motorController = 0; 
		int PWMDriver = 0;  
		switch( cval ) {
		case 0: // M0 - Set real time output off
			realtime_output = false;
			ret.add(String.format("%sM0%s%n",MSG_BEGIN,MSG_TERMINATE));	
			return ret;
		case 1: // M1 - Set real time output on 
			realtime_output = true;
			ret.add(String.format("%sM1%s%n",MSG_BEGIN,MSG_TERMINATE));
			return ret;
			//
			// M2 [Z<slot>] [C<channel> W<encoder pin> E<default dir>] 
			// Set smart controller (default) with optional encoder pin per channel, can be issued multiple times
			// CHANNEL 1-10, NO CHANNEL ZERO!	
		case 2: 
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
					try {
						motorControl[motorController].createEncoder(channel, pin_number, 0);
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					} // presume interrupt dealt with internally
				}
				if(code_seen('E')) {
					motorControl[motorController].setDefaultDirection(channel, (int) code_value());
				}
				ret.add(String.format("%sM2%s%n",MSG_BEGIN,MSG_TERMINATE));
				return ret;
			}
			break;
			//
			// M3 [Z<slot>] P<pin> C<channel> D<direction pin> E<default dir> [F PWM frequency] [G PWM Duty cycle] [W<encoder pin>] 
			// Set HBridge PWM motor driver, map pin to channel.
			// For a motor control subsequent G5 commands are affected here.
			// and then D a direction pin that determines forward/backward , then E, the default value of the direction pin.
			// The D pin and E direction determine whether a HIGH or LOW determines FORWARD/BACK on the given controller channel. This is
			// to compensate for 'backward' motors mounted left/right, or differences in controller design. Using a combination of
			// these 2 parameters you can tune any controller/motor setup properly for forward/back.
			// Finally, W<encoder pin> to receive hall wheel sensor signals. 
			// Leave out W option and use M14 later to create a more robust encoder. If W is used, an analog input
			// that fires at the maximum input level is created.
			//
		case 3: 
			pin_number = -1;
			encode_pin = 0;
			interrupt_pin = 0;
			pwm_freq = DEFAULT_PWM_FREQUENCY;
			pwm_duty = DEFAULT_PWM_DUTY;
			if(code_seen('Z')) {
				motorController = (int) code_value();
			}
			// motorControl = (AbstractMotorControl*)&hBridgeDriver;
			if(motorControl[motorController] != null) {
				//((HBridgeDriver)motorControl[motorController]).setMotors(ppwms);
				//((HBridgeDriver)motorControl[motorController]).setDirectionPins(pdigitals);
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
					if(code_seen('F')) {
						pwm_freq = (int) code_value();
					}
					if(code_seen('H')) {
						pwm_duty = (int) code_value();
					}
					if( code_seen('W')) {
						encode_pin = (int) code_value();
					}
					if( code_seen('I')) {
						interrupt_pin = (int) code_value();
					}
					try {
						((HBridgeDriver)motorControl[motorController]).createPWM(channel, pin_number, dir_pin, dir_default, pwm_freq, pwm_duty);
						if(encode_pin != 0) {
							motorControl[motorController].createEncoder(channel, encode_pin, interrupt_pin);
						}
					} catch (IOException /*| GpioPinExistsException*/ gpioe) {
						ret.add(String.format("%s%s:%s%s%n",MSG_BEGIN,MALFORMED_MCODE,gpioe,MSG_TERMINATE));
						return ret;
					}
				}
				ret.add(String.format("%sM3%s%n",MSG_BEGIN,MSG_TERMINATE));
				return ret;
			} else { //motorcontrol[motorcontroller]
				ret.add(String.format("%s%s%s%n",MSG_BEGIN,MSG_NO_CONTROL,MSG_TERMINATE));
				return ret;
			}
			//
			// M4 [Z<slot>] P<pin> Q<pin> C<channel> D<enable pin> B<enable pin b> E<default dir> [W<encoder pin>] [F<frequency 1-1000000>] [G<duty < freq>]
			// Split bridge or 2 half bridge motor controller. Takes 2 inputs: one for forward,called P, one for backward,called Q, then motor channel, 
			// and then D, an enable pin. Finally, W<encoder pin>  to receive hall wheel sensor signals and 
			// optionally PWM timer setup [R<resolution 8,9,10 bits>] [X<prescale 0-7>].
			// Everything derived from HBridgeDriver can be done here.
			//
		case 4:
			pwm_freq = DEFAULT_PWM_FREQUENCY; // frequency
			pwm_duty = DEFAULT_PWM_DUTY;
			pin_number = -1;
			pin_numberB = -1;
			int dir_pinb = -1;
			encode_pin = 0;
			interrupt_pin = 0;
			if(code_seen('Z')) {
				motorController = (int)code_value();
			}
			if(motorControl[motorController]  != null) {
				//motorControl = (AbstractMotorControl*)&splitBridgeDriver;
				//((SplitBridgeDriver)motorControl[motorController]).setMotors(ppwms);
				//((SplitBridgeDriver)motorControl[motorController]).setDirectionPins(pdigitals);
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
					if( code_seen('I')) {
						interrupt_pin = (int) code_value();
					}
					if( code_seen('F')) {
						pwm_freq = (int) code_value();
					}
					try {
						((SplitBridgeDriver)motorControl[motorController]).createPWM(channel, pin_number, pin_numberB, dir_pin, dir_pinb, dir_default, pwm_freq, pwm_duty);
						if(encode_pin != 0) {
							motorControl[motorController].createEncoder(channel, encode_pin, interrupt_pin);
						}
					} catch (IOException /*| GpioPinExistsException*/ gpioe) {
						ret.add(String.format("%s%s:%s%s%n",MSG_BEGIN,MALFORMED_MCODE,gpioe,MSG_TERMINATE));
						return ret;
					}
					ret.add(String.format("%sM4%s%n",MSG_BEGIN,MSG_TERMINATE));
					return ret;
				} // code C
			} else { //motorcontrol[motorcontroller]
				ret.add(String.format("%s%s%s%n",MSG_BEGIN,MSG_NO_CONTROL,MSG_TERMINATE));
				return ret;
			}
			break;
			//
			// M5 Z<slot> P<pin> [Q<pin>] C<channel> D<enable pin> E<default dir> [W<encoder>] 
			// Create switch bridge Z slot, P forward pin, Q reverse pin, D enable, E default state of enable for dir
			// Switch bridge or 2 digital motor controller. Takes 2 inputs: one digital pin for forward,called P, 
			// one for backward,called Q, then motor channel,
			// and then D, an enable pin, and E default dir, with optional encoder.
			// If Q pin argument is missing, create the switch H bridge type, which is descended from switch bridge.
			// the switch H bridge takes a direction pin high/low and a signal pin high/low to determine direction and speed.
			//
		case 5: 
			pin_number = -1;
			pin_numberB = -1;
			encode_pin = 0;
			interrupt_pin = 0;
			boolean onePin = false;
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
					onePin = true;
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
					if( code_seen('I')) {
						interrupt_pin = (int) code_value();
					}
					//try {
					if(onePin)
						try {
							((SwitchHBridgeDriver)motorControl[motorController]).createDigital(channel, pin_number, dir_pin, dir_default);
						} catch (IOException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					else
						try {
							((SwitchBridgeDriver)motorControl[motorController]).createDigital(channel, pin_number, pin_numberB, dir_pin, dir_default);
						} catch (IOException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					if(DEBUG)
						System.out.printf("%s Created M5 control %s for slot %d%n", this.getClass().getName(), (SwitchBridgeDriver)motorControl[motorController], motorController);
					if(encode_pin != 0) {
						try {
							motorControl[motorController].createEncoder(channel, encode_pin, interrupt_pin);
						} catch (IOException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
					//} catch (GpioPinExistsException gpioe) {
					//	ret.add(String.format("%s%s:%s%s%n",MSG_BEGIN,MALFORMED_MCODE,gpioe,MSG_TERMINATE));
					//return ret;
					//}
					ret.add(String.format("%sM5%s%n",MSG_BEGIN,MSG_TERMINATE));
					return ret;
				} // code C
			} else { //motorcontrol[motorcontroller]
				ret.add(String.format("%s%s%s%n",MSG_BEGIN,MSG_NO_CONTROL,MSG_TERMINATE));
				return ret;
			}
			break;
			//
			// M6 [Z<slot>] [S<scale>] [X<scale>] 
			// Set motor or PWM scaling, divisor for final power to limit speed or level, set to 0 to cancel. If X, slot is PWM
			//
		case 6: 
			if(code_seen('Z')) {
				motorController = (int) code_value();
			}
			if( code_seen('S') ) {
				if(motorControl[motorController] != null) {
					motorControl[motorController].setMotorPowerScale((int) code_value());
					ret.add(String.format("%sM6%s%n",MSG_BEGIN,MSG_TERMINATE));
					return ret;
				}
			} else {
				if(code_seen('X')) {
					if(pwmControl[motorController] != null) {
						pwmControl[motorController].setPWMPowerScale((int) code_value());
						ret.add(String.format("%sM6%s%n",MSG_BEGIN,MSG_TERMINATE));
						return ret;
					}
				}
			}
			break;
			//
			// M7 [Z<slot>] [X]- Set motor override to stop motor operation, or optionally PWM operation, if X, slot is PWM
			//
		case 7:
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
					ret.add(String.format("%sM7%s%n",MSG_BEGIN,MSG_TERMINATE));
					return ret;
				}
			} else {
				if(motorControl[motorController] != null) {
					try {
						motorControl[motorController].setMotorShutdown();
					} catch (IOException e) {
						e.printStackTrace();
					}
					ret.add(String.format("%sM7%s%n",MSG_BEGIN,MSG_TERMINATE));
					return ret;
				}
			}
			break;
			//
			// M8 [Z<slot>][X] - Set motor override to start motor operation after stop override M7. If X, slot is PWM
			//
		case 8:
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
					ret.add(String.format("%sM8%s%n",MSG_BEGIN,MSG_TERMINATE));
					return ret;
				}
			} else {
				if(motorControl[motorController] != null ) {
					try {
						motorControl[motorController].setMotorRun();
					} catch (IOException e) {
						e.printStackTrace();
					}
					ret.add(String.format("%sM8%s%n",MSG_BEGIN,MSG_TERMINATE));
					return ret;
				}
			}
			break;
			//
			// M9 [Z<slot>] P<pin> C<channel> D<enable pin> E<dir default> [F<PWM frequency>] [G<PWM duty cycle>]
			// PWM control <br/>
			// Activate a previously created PWM controller of type AbstractPWMControl - a non propulsion PWM device such as LED or pump
			// Note there is no encoder or direction pin, and no possibility of reverse. What would be reverse in a motor control is 
			// the first half of the power scale instead.
			//
		case 9:
			pwm_freq = DEFAULT_PWM_FREQUENCY; // resolution in bits
			pwm_duty = DEFAULT_PWM_DUTY;
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
					if(code_seen('H')) {
						pwm_duty = (int) code_value();
					}
					try {
						((VariablePWMDriver)pwmControl[PWMDriver]).createPWM(channel, pin_number, enable_pin, dir_default, pwm_freq, pwm_duty);
					} catch (IOException /*| GpioPinExistsException*/ gpioe) {
						ret.add(String.format("%s%s:%s%s%n",MSG_BEGIN,MALFORMED_MCODE,gpioe,MSG_TERMINATE));
						return ret;
					}
					ret.add(String.format("%sM9%s%n",MSG_BEGIN,MSG_TERMINATE));
					return ret;
				}
			}
			break;
			//
			// M10 Z<controller slot> T<controller type>
			// Dynamically allocate a controller to a control slot. The slot parameter is used to refer
			// to the dynamically allocated controller in other M codes that relate to motor control functions.
			// The M10 code merely creates the instance of the proper controller and assigns the slot. Other M codes
			// refer to the slot and provide further configuration. when creating new type of controllers, this is the code
			// that can be expanded to instantiate those controllers. There can be up to 10 channels of controller types.<p>
			// T0 - Smart controller a la Roboteq <br>
			// T1 - H-Bridge PWM <br>
			// T2 - Split bridge, each channel has 2 PWM pins and an enable pin <br>
			// T3 - Switch bridge, each channel has 2 GPIO pins for full forward and back, no PWM, and an enable pin
			// T4 - Switch H-bridge
			// T5 - non-propulsion PWM driver
			// T8 - Delayed H-bridge. H-bridge with delay between direction change
		case 10:
			if( code_seen('Z') ) {
				motorController = (int) code_value();
				if( code_seen('T') ) {
					int controllerType = (int) code_value();
					//try {
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
						ret.add(String.format("%sM10%s%n",MSG_BEGIN,MSG_TERMINATE));
						return ret;
					case 1: // type 1 Hbridge
						// up to 10 channels, each channel has a direction pin (1), and a PWM pin (0)
						if(motorControl[motorController] != null) {
							// for each channel, delete the direction pin and PWM created in main pin array to prepare new assignment
							// each controller can have up to 10 channels, each with its own PWM and direction pin
							for(int i = 1; i <= motorControl[motorController].getChannels(); i++) {
								int pMotor1 = ((HBridgeDriver)motorControl[motorController]).getMotorPWMPin(i);
								int pMotor2 = ((HBridgeDriver)motorControl[motorController]).getMotorEnablePin(i);
								System.out.println("Existing control using pwm pin:"+pMotor1+", and enable pin:"+pMotor2);
								//if(pMotor2 != 255 && pdigitals[pMotor2] != null) {
								//	pdigitals[pMotor2] = null;
								//}
								//if(pMotor1 != 255 && ppwms[pMotor1] != null) {
								//	ppwms[pMotor1] = null;
								//}
							}
							motorControl[motorController] = null; // in case assignment below fails
						}
						motorControl[motorController] = new HBridgeDriver(MAX_MOTOR_POWER);
						ret.add(String.format("%sM10%s%n",MSG_BEGIN,MSG_TERMINATE));
						return ret;
					case 2: // type 2 Split bridge, each channel has 2 PWM pins and an enable pin, so up to 5 channels
						if(motorControl[motorController] != null) {
							// for each channel, delete the direction pin and PWM created in main pin array to prepare new assignment
							// each controller can have up to 10 channels, each with its own PWM and direction pin
							for(int i = 1; i <= motorControl[motorController].getChannels(); i++) {
								int pMotor1 = ((SplitBridgeDriver)motorControl[motorController]).getMotorPWMPin(i);
								int pMotor2 = ((SplitBridgeDriver)motorControl[motorController]).getMotorEnablePin(i);
								System.out.println("Existing control using PWM pin:"+pMotor1+", and enable pin:"+pMotor2);
								//if(pMotor2 != 255 && pdigitals[pMotor2] != null) {
								//	pdigitals[pMotor2] = null;
								//}
								//if(pMotor1 != 255 && ppwms[pMotor1] != null) {
								//	ppwms[pMotor1] = null;
								//}
								//pMotor1 = ((SplitBridgeDriver)motorControl[motorController]).getMotorPWMPinB(i);
								//if(pMotor1 != 255 && ppwms[pMotor1] != null) {
								//	ppwms[pMotor1] = null;
								//}
							}
							//delete motorControl[motorController];
							motorControl[motorController] = null; // in case assignment below fails
						}
						motorControl[motorController] = new SplitBridgeDriver(MAX_MOTOR_POWER);
						ret.add(String.format("%sM10%s%n",MSG_BEGIN,MSG_TERMINATE));
						return ret;
					case 3: // type 3 Switch bridge, each channel has 2 GPIO pins for full forward and back, no PWM, and an enable pin
						if(motorControl[motorController] != null) {
							// for each channel, delete the direction pin and PWM created in main pin array to prepare new assignment
							// each controller can have up to 10 channels, each with its own PWM and direction pin
							for(int i = 1; i <= motorControl[motorController].getChannels(); i++) {
								int pMotor1 = ((SwitchBridgeDriver)motorControl[motorController]).getMotorDigitalPin(i);
								int pMotor2 = ((SwitchBridgeDriver)motorControl[motorController]).getMotorEnablePin(i);
								System.out.println("Existing control using PWM pin:"+pMotor1+", and enable pin:"+pMotor2);
								//if(pMotor2 != 255 && pdigitals[pMotor2] != null) {
								//	pdigitals[pMotor2] = null;
								//}
								//if(pMotor1 != 255 && pdigitals[pMotor1] != null) {
								//	pdigitals[pMotor1] = null;
								//}
								//pMotor1 = ((SwitchBridgeDriver)motorControl[motorController]).getMotorDigitalPinB(i);
								//if(pMotor1 != 255 && pdigitals[pMotor1] != null) {
								//	pdigitals[pMotor1] = null;
								//}
							}
							//delete motorControl[motorController];
							motorControl[motorController] = null; // in case assignment below fails
						}
						motorControl[motorController] = new SwitchBridgeDriver();
						ret.add(String.format("%sM10%s%n",MSG_BEGIN,MSG_TERMINATE));
						return ret;
					case 4: // Type 4 switch H-bridge
						if(motorControl[motorController] != null) {
							motorControl[motorController] = null; // in case assignment below fails
						}
						motorControl[motorController] = new SwitchHBridgeDriver();
						ret.add(String.format("%sM10%s%n",MSG_BEGIN,MSG_TERMINATE));
						return ret;
					case 5: // Type 5 non-propulsion PWM driver 
						if(pwmControl[motorController] != null) {
							// for each channel, delete the direction pin and PWM created in main pin array to prepare new assignment
							// each controller can have up to 10 channels, each with its own PWM and direction pin
							for(int i = 1; i <= pwmControl[motorController].getChannels(); i++) {
								int pMotor1 = ((VariablePWMDriver)pwmControl[motorController]).getPWMEnablePin(i);
								System.out.println("Existing control using PWM pin:"+pMotor1);
								//if(pMotor1 != 255 && pdigitals[pMotor1] != null) {
								//	pdigitals[pMotor1] = null;
								//}
								//pMotor1 = ((VariablePWMDriver)pwmControl[motorController]).getPWMLevelPin(i);
								//if(pMotor1 != 255 && ppwms[pMotor1] != null) {
								//	ppwms[pMotor1] = null;
								//}
							}
							//delete pwmControl[motorController];
							pwmControl[motorController] = null; // in case assignment below fails
						}
						pwmControl[motorController] = new VariablePWMDriver();
						ret.add(String.format("%sM10%s%n",MSG_BEGIN,MSG_TERMINATE));
						return ret;
					case 8: // type 8 delayed H-bridge. Like a regular H-bridge but with a reverse direction delay
						// up to 10 channels, each channel has a direction pin (1), and a PWM pin (0)
						if(motorControl[motorController] != null) {
							// for each channel, delete the direction pin and PWM created in main pin array to prepare new assignment
							// each controller can have up to 10 channels, each with its own PWM and direction pin
							for(int i = 1; i <= motorControl[motorController].getChannels(); i++) {
								int pMotor1 = ((HBridgeDriver)motorControl[motorController]).getMotorPWMPin(i);
								int pMotor2 = ((HBridgeDriver)motorControl[motorController]).getMotorEnablePin(i);
								System.out.println("Existing control using PWM pin:"+pMotor1+", and enable pin:"+pMotor2);
							}
							motorControl[motorController] = null; // in case assignment below fails
						}
						motorControl[motorController] = new DelayedHBridgeDriver(MAX_MOTOR_POWER);
						ret.add(String.format("%sM10%s%n",MSG_BEGIN,MSG_TERMINATE));
						return ret;
					default:
						ret.add(String.format("%sBAD CONTROLLER TYPE:%d%s%n",MSG_BEGIN,controllerType,MSG_TERMINATE));
						return ret;
					}
				} else {
					ret.add(String.format("%sBAD CONTROLLER TYPE:CONTROLLER TYPE DIRECTIVE NOT SEEN:%d%s%n",MSG_BEGIN,MSG_TERMINATE));
					return ret;
				}			
			} else {
				ret.add(String.format("%sBAD CONTROLLER TYPE:CONTROLLER SLOT DIRECTIVE NOT SEEN:%d%s%n",MSG_BEGIN,MSG_TERMINATE));
				return ret;
			}
			//
			// M11 [Z<slot>] C<channel> [D<duration>] [X<duration>] 
			// Set maximum cycle duration for given channel. If X, slot is PWM
			//
		case 11:
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
						ret.add(String.format("%sM11%s%n",MSG_BEGIN,MSG_TERMINATE));
						return ret;
					}
				} else {
					if(code_seen('D')) {
						if(motorControl[motorController] != null) {
							motorControl[motorController].setDuration(channel, (int)code_value());
							ret.add(String.format("%s M11%s%n",MSG_BEGIN,MSG_TERMINATE));
							return ret;
						}
					}
				}
			}
			break;
			//
			// M12 [Z<slot>] C<channel> [P<offset>] [X<offset>] 
			// set amount to add to G5 for min motor power, or X PWM level, If X, slot is PWM
			//
		case 12:
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
						ret.add(String.format("%sM12%s%n",MSG_BEGIN,MSG_TERMINATE));
						return ret;
					}
				} else {
					if( code_seen('P')) {
						if(motorControl[motorController] != null) {
							motorControl[motorController].setMinMotorPower(channel, (int)code_value());
							ret.add(String.format("%sM12%s%n",MSG_BEGIN,MSG_TERMINATE));
							return ret;
						}
					}
				}
			}
			break;
			//
			// M13 [Z<slot>] [P<power>] [X<power>]
			// Set maximum motor power or optionally with X, a PWM control maximum level. If X, slot is PWM
			//
		case 13: 
			if(code_seen('Z')) {
				motorController = (int) code_value();
			}
			if( code_seen('P') ) {
				if(motorControl[motorController] != null) {
					motorControl[motorController].setMaxMotorPower((int)code_value());
					ret.add(String.format("%sM13%s%n",MSG_BEGIN,MSG_TERMINATE));
					return ret;
				} else {
					if(code_seen('X')) {
						if(pwmControl[motorController] != null) {
							pwmControl[motorController].setMaxPWMLevel((int)code_value());
							ret.add(String.format("%sM13%s%n",MSG_BEGIN,MSG_TERMINATE));
							return ret;
						}
					}
				}
			}
			break;
			//  
			// M14 [Z<slot>] C<channel> P<pin> L<low range active> H<high range active> N<number of counts before interrupt generated>
			// Create analog encoder for controller at slot and channel.
			// Activate interrupt between L low and H high range.
			// Detect range N times before interrupt
			//
		case 14:
			double analogRangeL, analogRangeH;
			int counts;
			interrupt_pin = 0;
			if(code_seen('Z')) {
				motorController = (int) code_value();
			}
			if( code_seen('C') ) {
				channel = (int) code_value();
				if(channel <= 0) {
					break;
				}
				if(code_seen('P')) {
					int pin = (int) code_value();
					analogRangeL = code_seen('L') ? code_value() : 0;
					analogRangeH = code_seen('H') ? code_value() : 0;
					counts = (int) (code_seen('N') ? code_value() : 1);
					if( code_seen('I')) {
						interrupt_pin = (int) code_value();
					}
					//try {
					try {
						motorControl[motorController].createEncoder(channel, pin, analogRangeL, analogRangeH, counts, interrupt_pin);
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					//} catch(GpioPinExistsException gpioe) {
					//	ret.add(String.format("%s%s:%s%s%n",MSG_BEGIN,MALFORMED_MCODE,gpioe,MSG_TERMINATE));
					//	return ret;
					//}
					ret.add(String.format("%sM14%s%n",MSG_BEGIN,MSG_TERMINATE));
					return ret;
				}
			}
			break;
			//  
			// M15 [Z<slot>] C<channel> P<pin> S<pin state 0 low, 1 high> N<number of counts before interrupt generated>
			// Create digital encoder for controller at slot and channel.
			// Activate interrupt at S pin state.
			// Detect range N times before interrupt
			//
		case 15:
			int digitalState;
			if(code_seen('Z')) {
				motorController = (int) code_value();
			}
			if( code_seen('C') ) {
				channel = (int) code_value();
				if(channel <= 0) {
					break;
				}
				if(code_seen('P')) {
					int pin = (int) code_value();
					digitalState = (int) (code_seen('S') ? code_value() : 1);
					counts = (int) (code_seen('N') ? code_value() : 1);
					if( code_seen('I')) {
						interrupt_pin = (int) code_value();
					}
					//try {
					try {
						motorControl[motorController].createDigitalEncoder(channel, pin, 1, counts, interrupt_pin);
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					//} catch(GpioPinExistsException gpioe) {
					//	ret.add(String.format("%s%s:%s%s%n",MSG_BEGIN,MALFORMED_MCODE,gpioe,MSG_TERMINATE));
					//	return ret;
					//}
					ret.add(String.format("%sM15%s%n",MSG_BEGIN,MSG_TERMINATE));
					return ret;
				}
			}
			break;
			//
			// M16 [Z<slot>] P<pin> C<channel> D<direction pin> E<default dir> [F PWM frequency] [G PWM Duty cycle] [W<encoder pin>] 
			// Set Delayed HBridge PWM motor driver, map pin to channel.
			// For a motor control subsequent G5 commands are affected here.
			// and then D a direction pin that determines forward/backward , then E, the default value of the direction pin.
			// The D pin and E direction determine whether a HIGH or LOW determines FORWARD/BACK on the given controller channel. This is
			// to compensate for 'backward' motors mounted left/right, or differences in controller design. Using a combination of
			// these 2 parameters you can tune any controller/motor setup properly for forward/back.
			// Finally, W<encoder pin> to receive hall wheel sensor signals. 
			// Leave out W option and use M14 later to create a more robust encoder. If W is used, an analog input
			// that fires at the maximum input level is created.
			//
		case 16: 
			pin_number = -1;
			encode_pin = 0;
			interrupt_pin = 0;
			pwm_freq = DEFAULT_PWM_FREQUENCY;
			pwm_duty = DEFAULT_PWM_DUTY;
			if(code_seen('Z')) {
				motorController = (int) code_value();
			}
			if(motorControl[motorController] != null) {
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
					if(code_seen('F')) {
						pwm_freq = (int) code_value();
					} else {
						break;
					}
					if(code_seen('H')) {
						pwm_duty = (int) code_value();
					} else {
						break;
					}
					if( code_seen('W')) {
						encode_pin = (int) code_value();
					}
					if( code_seen('I')) {
						interrupt_pin = (int) code_value();
					}
					try {
						((HBridgeDriver)motorControl[motorController]).createPWM(channel, pin_number, dir_pin, dir_default, pwm_freq, pwm_duty);
						if(encode_pin != 0) {
							motorControl[motorController].createEncoder(channel, encode_pin, interrupt_pin);
						}
					} catch (IOException gpioe) {
						ret.add(String.format("%s%s:%s%s%n",MSG_BEGIN,MALFORMED_MCODE,gpioe,MSG_TERMINATE));
						return ret;
					}
				}
				ret.add(String.format("%sM16%s%n",MSG_BEGIN,MSG_TERMINATE));
				return ret;
			}
			break;
			//
			// M33 [Z<slot>] P<ultrasonic pin> D<min. distance in cm> [E<direction 1- forward facing, 0 - reverse facing sensor>] 
			// link Motor controller to ultrasonic sensor, the sensor must exist via M301
			//
		case 33: 
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
					//try {
					motorControl[motorController].linkDistanceSensor( pin_number, (Ultrasonic)psonics, dist, dir_face);
					//} catch(GpioPinExistsException gpioe) {
					//	ret.add(String.format("%s%s:%s%s%n",MSG_BEGIN,MALFORMED_MCODE,gpioe,MSG_TERMINATE));
					//	return ret;
					//}
					ret.add(String.format("%sM33%s%n",MSG_BEGIN,MSG_TERMINATE));
					return ret;
				} // code_seen = 'P'
			}
			break;
			//
			// M35
			// Clear all digital pins assigned outside motor controller directives
			//
		case 35:
			ret.add(String.format("%sM35%s%n",MSG_BEGIN,MSG_TERMINATE));
			return ret;
			//
			// M36 
			// Clear all analog pins assigned outside motor controller directives
			//
		case 36: 
			//Pins.unassignPins();
			ret.add(String.format("%sM36%s%n",MSG_BEGIN,MSG_TERMINATE));
			return ret;
			//
			// M37 - Clear all PWM pins, ALL MOTOR AND PWM DISABLED, perhaps not cleanly
			//
		case 37:
			/*
				for(int i = 0; i < 12; i++) {
				  if(ppwms[i] != null) {
					  Pins.unassignPin(ppwms[i].pin);
					  //delete ppwms[i];
					  ppwms[i] = null;
				  }
				}
			 */
			ret.add(String.format("%sM37%s%n",MSG_BEGIN,MSG_TERMINATE));
			return ret;
			//
			// M38  P<pin> - Remove PWM pin, MOTOR AND PWM DISABLED, perhaps not cleanly
			//
		case 38:
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
				ret.add(String.format("%sM38%s%n",MSG_BEGIN,MSG_TERMINATE));
				return ret;
				// } // unassign pin
			} // code P
			break;
			//
			// M39 P<pin> - Remove Persistent Analog pin
			//
		case 39:
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
				ret.add(String.format("%sM39%s%n",MSG_BEGIN,MSG_TERMINATE));
				return ret;
				//}
			}
			break;
			//
			// M40 P<pin> - Remove persistent digital pin 
			//
		case 40: 
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
				ret.add(String.format("%sM40%s%n",MSG_BEGIN,MSG_TERMINATE));
				return ret;
				//}
			}
			break;
			//
			// M41 P<pin> 
			// Create persistent digital output pin
			//
		case 41:
			pin_number = -1;
			if (code_seen('P')) {
				pin_number = (int)code_value();
				// try {
				try {
					Pins.assignPin(pin_number);
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				ret.add(String.format("%sM41%s%n",MSG_BEGIN,MSG_TERMINATE));
				return ret;
				//} catch(GpioPinExistsException gpioe) {
				//		ret.add(String.format("%s%s:%s%s%n",MSG_BEGIN,MALFORMED_MCODE,gpioe,MSG_TERMINATE));
				//		return ret;
				//}
			}
			break;
			//
			// M42 P<pin> S<state> L[low value] H[high value] T
			// state value L - LOW state value, H HIGH state value T - toggle at high, ignore low
			// Write digital pin
			//
		case 42:
			pin_number = -1;
			if (code_seen('P')) 
				pin_number = (int)code_value();
			/*
					  dpin = Pins.getOutputPin(pin_number);
				      if(code_seen('S')) {
				    	 codenum = (int)code_value(); // current state
				    	 int low_val = 0;
				    	 int high_val = 0;
				    	 if( code_seen('L'))
				    		 low_val = (int)code_value();
				    	 if( code_seen('H'))
				    		 high_val = (int)code_value();
				    	 if(code_seen('T')) { // toggle if high, else ignore
				    		 if(codenum == high_val)
				    			 dpin.toggle();
				    	 } else {
				    		 if(codenum == low_val)
				    			 dpin.low();
				    		 else
				    			 if(codenum == high_val)
				    				 dpin.high();
				    	 }
				      }
					ret.add(String.format("%sM42%s%n",MSG_BEGIN,MSG_TERMINATE));
					return ret;
				  }
			 */
			break;
			//
			// M43 P<pin>
			// Create persistent digital input pin
			//
		case 43:
			pin_number = -1;
			if (code_seen('P')) {
				pin_number = (int)code_value();
				try {
					Pins.assignInputPin(pin_number);
					ret.add(String.format("%sM43%s%n",MSG_BEGIN,MSG_TERMINATE));
					return ret;
				} catch(IOException gpioe) {
					ret.add(String.format("%s%s:%s%s%n",MSG_BEGIN,MALFORMED_MCODE,gpioe,MSG_TERMINATE));
					return ret;
				}
			}
			break; 
			//
			// M44 P<pin> [U] - -Read digital pin with optional pullup
			//
		case 44:
			pin_number = -1;
			int res = 0;
			if (code_seen('P')) {
				pin_number = (int)code_value();
				/*
		          ipin = Pins.getInputPin(pin_number);
		          switch(ipin.getState()) {
		          	case LOW:
		          		res = 0;
		          		break;
		          	case HIGH:
		          		res = 1;
		          		break;
		          	default:
		          		res = 0;
		          		break;
		          }
				 */
				ret.add(String.format("%s%s%s%n",MSG_BEGIN,digitalPinHdr,MSG_DELIMIT));
				ret.add(String.format("0 %d%n",pin_number));
				ret.add(String.format("1 %d%n",res));
				ret.add(String.format("%s%s%s%n",MSG_BEGIN,digitalPinHdr,MSG_TERMINATE));
			}
			break;
			//	
			// M45 - set up PWM P<pin> S<power val 0-255> [F<frequency>]
			// PWM value between 0 and 255, default timer mode is 2; clear on match, default resolution is 8 bits, default prescale is 1
			// Prescale: 1,2,4,6,7,8,9 = none, 8, 64, 256, 1024, external falling, external rising
			// Use M445 to disable pin permanently or use timer more 0 to stop pulse without removing pin assignment
			//
		case 45:
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
			for(int i = 0; i < ppwms.length; i++) {
				if(ppwms[i] == null) {
					PWM ppin = new PWM(pin_number);
					try {
						ppin.freq(pwm_freq);
						ppin.duty(pin_status); // default is 2, clear on match. to turn off, use 0
					} catch (IOException e) {
						e.printStackTrace();
					}
					ppwms[i] = ppin;
				}
			}
			ret.add(String.format("%sM45%s%n",MSG_BEGIN,MSG_TERMINATE));
			return ret;
			//
			// M46 -Read analog pin P<pin>
			//
		case 46:
			pin_number = -1;
			res = 0;
			if (code_seen('P')) {
				pin_number = (int)code_value();
				/*GpioPinAnalogInput apin = null;
			      apin = Pins.getAnalogInputPin(pin_number);
			      if(apin == null) {
					try {
						Pins.assignAnalogInputPin(codenum);
					} catch(GpioPinExistsException gpioe) {
						ret.add(String.format("%s%s:%s%s%n",MSG_BEGIN,MALFORMED_MCODE,gpioe,MSG_TERMINATE));
						return ret;
					}
				 }
			     res = (int) apin.getValue();
				 */
				ret.add(String.format("%s%s%s%n",MSG_BEGIN,analogPinHdr,MSG_DELIMIT));
				ret.add(String.format("0 %d%n",pin_number));
				ret.add(String.format("1 %d%n",res));
				ret.add(String.format(MSG_BEGIN,analogPinHdr,MSG_TERMINATE));
				return ret;
			}
			break;
			//
			// M47 -Read analog pin P<pin> T<threshold> compare to battery threshold, if below, print battery message
			//
		case 47:
			pin_number = -1;
			if (code_seen('P')) {
				pin_number = (int)code_value();
				digitarg = code_seen('T') ? (int)code_value() : 0;
				pin_number = (int)code_value();
				/*
				      GpioPinAnalogInput apin = null;
				      apin = Pins.getAnalogInputPin(pin_number);
				      if(apin == null) {
						try {
							Pins.assignAnalogInputPin(codenum);
						} catch(GpioPinExistsException gpioe) {
							ret.add(String.format("%s%s:%s%s%n",MSG_BEGIN,MALFORMED_MCODE,gpioe,MSG_TERMINATE));
							return ret;
						}
					 }
				     res = (int) apin.getValue();
				     if( res < digitarg ) { // result < threshold is 0 by default
						   return publishBatteryVolts(res);
					 } else {
						   ret.add(String.format("%sM47%s%n",MSG_BEGIN,MSG_TERMINATE));
						   return ret;
					 }
				 */
			}
			break;

		case 80: //
			ret.add(String.format("%sM80%s%n",MSG_BEGIN,MSG_TERMINATE));
			return ret;
			//
			// M81 [Z<slot>] X 
			// Turn off Power Z shut down motorcontroller in slot, X shut down PWM, slot -1 do all
			//
		case 81: 
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
			ret.add(String.format("%sM81%s%n",MSG_BEGIN,MSG_TERMINATE));
			return ret;
			//
			// M115
			// Generate configuration report
			//
		case 115: 
			ret.add(String.format("%s%s%s%n",MSG_BEGIN,MSG_M115_REPORT,MSG_DELIMIT));
			ret.add(MSG_115_REPORT2);
			ret.add(String.format("%s%s%s%n",MSG_BEGIN,MSG_M115_REPORT,MSG_TERMINATE));
			return ret;
			//
			// M300 P<pin number>
			// emit ultrasonic pulse on given pin and return duration 
			//
		case 300:
			uspin = code_seen('P') ? (int)code_value() : 0;
			if (uspin > 0) {
				try {
					int i = 0;
					for(; i < psonics.length; i++) {
						if(psonics[i] == null) {
							psonics[i] = new Ultrasonic(uspin);
							break;
						}
						if(psonics[i].txrxPin == uspin || psonics[i].txPin == uspin)
							break;
					}
					ret.add(String.format("%s%s%s%n",MSG_BEGIN,sonicCntrlHdr,MSG_DELIMIT));
					ret.add(String.format("0 %d%n", uspin));
					ret.add(String.format("1 %d%n", psonics[i].getRange()));
					ret.add(String.format("%s%s%s%n",MSG_BEGIN,sonicCntrlHdr,MSG_TERMINATE));
					return ret;	
				} catch(/*GpioPinExistsException |*/ IOException gpee) {}
			}
			break;
			//
			// M301 P<pin>
			// attach ultrasonic device to pin
			//
		case 301: 
			// wont assign pin 0 as its sensitive
			uspin = code_seen('P') ? (int)code_value() : 0;
			// this is a permanent pin assignment so dont add if its already assigned
			for(int i = 0; i < psonics.length; i++) {
				if(psonics[i] != null) {
					psonics[i] = new Ultrasonic(uspin);
					ret.add(String.format("%sM301%s%n",MSG_BEGIN,MSG_TERMINATE));
					return ret;
				}
			}
			break;
			//
			// M302 P<pin>
			// remove ultrasonic pin
			//
		case 302:
			uspin = code_seen('P') ? (int)code_value() : 0;
			for(int i = 0; i < psonics.length; i++) {
				if(psonics[i] != null && psonics[i].txrxPin == uspin || psonics[i].txPin == uspin ) {
					psonics[i] = null;
					break;
				}
			}
			ret.add(String.format("%sM302%s%n",MSG_BEGIN,MSG_TERMINATE));
			return ret;

			//case 303: // M303 - Check the analog inputs for all pins defined by successive M304 directives. Generate a read and output data if in range.
			//break;
			//
			// M304 P<pin> [L<min>] [H<max>]  
			// analog read with optional exclusion range 0-1024 via L<min> H<max>
			//
		case 304:
			// if optional L and H values exclude readings in that range
			pin_number = -1;
			if (code_seen('P')) {
				pin_number = (int)code_value();
				int i = 0;
				analogRanges[0][i] = code_seen('L') ? code_value() : 0;
				analogRanges[1][i] = code_seen('H') ? code_value() : 0;
				/*
					GpioPinAnalogInput apin = null;
					apin = Pins.getAnalogInputPin(pin_number);
					if(apin == null) {
						try {
							Pins.assignAnalogInputPin(codenum);
						} catch(GpioPinExistsException gpioe) {
							ret.add(String.format("%s%s:%s%s%n",MSG_BEGIN,MALFORMED_MCODE,gpioe,MSG_TERMINATE));
							return ret;
						}
					}
					res = (int) apin.getValue();
					if( res >= analogRanges[0][i] && res <= analogRanges[1][i]) {
						ret.add(String.format("%s%s%s%n",MSG_BEGIN,analogPinHdr,MSG_DELIMIT));
						ret.add(String.format("0 %d%n",pin_number));
						ret.add(String.format("1 %d%n",res));
						ret.add(String.format(MSG_BEGIN,analogPinHdr,MSG_TERMINATE));
						return ret;
					}
				 */
			}
			ret.add(String.format("%sM304%s%n",MSG_BEGIN,MSG_TERMINATE));
			return ret;

			//
			// M445 P<pin> 
			// Turn off pulsed write pin - disable PWM
			//
		case 445:
			if(code_seen('P')) {
				pin_number =(int) code_value();
				for(int i = 0; i < ppwms.length; i++) {
					if(ppwms[i] != null && ppwms[i].pin == pin_number) {
						try {
							ppwms[i].duty(0);
						} catch (IOException e) {
							e.printStackTrace();
						} // default is 2, clear on match. to turn off, use 0 
						//delete ppwms[i];
						ppwms[i] = null;
						break;
					}
				}
			}
			ret.add(String.format("%sM445%s%n",MSG_BEGIN,MSG_TERMINATE));
			return ret;
			//
			// M500 
			// Make current settings persistent
			//
		case 500: 
			//Config_StoreSettings();
			StringBuilder sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append("M500");
			sb.append(MSG_TERMINATE);
			ret.add(sb.toString());
			return ret;
			//
			// M501 
			// Read settings from persistence
			//
		case 501:
			//Config_RetrieveSettings();
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append("M501");
			sb.append(MSG_TERMINATE);
			ret.add(sb.toString());
			return ret;
			//
			// M502 
			// Revert to default settings
			//
		case 502:
			//Config_ResetDefault();
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append("M502");
			sb.append(MSG_TERMINATE);
			ret.add(sb.toString());
			return ret;

			//case 503: // M503 print settings currently in memory
			//Config_PrintSettings();
			//break;
			//
			// M700
			// return stats regarding free memory and version
			//
		case 700:
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(MSG_STATUS);
			sb.append(MSG_DELIMIT);
			sb.append("\r\n");
			ret.add(sb.toString());
			// Check startup - does nothing if bootloader sets MCUSR to 0
			sb = new StringBuilder();
			//sb.append(VERSION_STRING);
			sb.append(MSG_CONFIGURATION_VER);
			//sb.append(STRING_VERSION_CONFIG_H);
			sb.append(MSG_AUTHOR);
			//sb.append(STRING_CONFIG_H_AUTHOR);
			sb.append(MSG_FREE_MEMORY);
			sb.append("\r\n");
			ret.add(sb.toString());
			sb = new StringBuilder();
			//sb.appendln(freeMemory());
			sb.append(MSG_BEGIN);
			sb.append(MSG_STATUS);
			sb.append(MSG_TERMINATE);
			sb.append("\r\n");
			ret.add(sb.toString());
			return ret;
			//
			// M701
			// Report digital pins in use
			//
		case 701:
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(digitalPinSettingHdr);
			sb.append(MSG_DELIMIT);
			sb.append("\r\n");
			ret.add(sb.toString());
			sb = new StringBuilder();
			sb.append(new Pins().toString());
			sb.append("\r\n");
			ret.add(sb.toString());
			sb.append(MSG_BEGIN);
			sb.append(digitalPinSettingHdr);
			sb.append(MSG_TERMINATE);
			sb.append("\r\n");
			ret.add(sb.toString());
			//
			// M702
			// Report analog pins in use
			//
		case 702:
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(analogPinSettingHdr);
			sb.append(MSG_DELIMIT);
			sb.append("\r\n");
			ret.add(sb.toString());
			for(int i = 0; i < 16; i++) {
				sb = new StringBuilder();
				if( panalogs[i]  != null ) {
					sb.append(panalogs[i]);
					sb.append("\r\n");
					ret.add(sb.toString());
				}
			}
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(analogPinSettingHdr);
			sb.append(MSG_TERMINATE);
			sb.append("\r\n");
			ret.add(sb.toString());
			//
			// M703
			// Report ultrasonic pins in use
			//
		case 703:
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(ultrasonicPinSettingHdr);
			sb.append(MSG_DELIMIT);
			sb.append("\r\n");
			ret.add(sb.toString());
			for(int i = 0; i < 10; i++) {
				if( psonics[i] != null) {
					sb = new StringBuilder();
					sb.append("Pin:");
					sb.append(psonics[i]);
					sb.append("\r\n");
					ret.add(sb.toString());
				}
			}
			sb.append(MSG_BEGIN);
			sb.append(ultrasonicPinSettingHdr);
			sb.append(MSG_TERMINATE);
			sb.append("\r\n");
			ret.add(sb.toString());
			return ret;
			//
			// M704
			// Report PWM pins in use
			//
		case 704:
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(pwmPinSettingHdr);
			sb.append(MSG_DELIMIT);
			sb.append("\r\n");
			ret.add(sb.toString());
			for(int i = 0; i < 12; i++) {
				if( ppwms[i] != null) {
					sb = new StringBuilder();
					sb.append("Pin:");
					sb.append(ppwms[i].pin);
					sb.append(" Timer channel:");
					sb.append(ppwms[i]);
					sb.append("\r\n");
					ret.add(sb.toString());
				}
			}
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(pwmPinSettingHdr);
			sb.append(MSG_TERMINATE);
			sb.append("\r\n");
			ret.add(sb.toString());
			if(DEBUG)
				System.out.println(sb.toString());
			return ret;
			//
			// M705
			// Definitive comprehensive report on all motor control and pwm controls configured
			//
		case 705:
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(motorControlSettingHdr);
			sb.append(MSG_DELIMIT);
			sb.append("\r\n");
			ret.add(sb.toString());
			for(int j = 0; j < motorControl.length; j++) {
				if( motorControl[j] != null) {
					sb = new StringBuilder();
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
							sb.append(motorControl[j].getDriverInfo(i+1));
							sb.append("\r\n");
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
					ret.add(sb.toString());
				} // if motorControl[j]
			} // j each motor controller
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(motorControlSettingHdr);
			sb.append(MSG_TERMINATE);
			sb.append("\r\n");
			ret.add(sb.toString());
			//
			// PWM control
			//
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(pwmControlSettingHdr);
			sb.append(MSG_DELIMIT);
			sb.append("\r\n");
			ret.add(sb.toString());
			for(int j = 0; j < pwmControl.length; j++) {
				if(pwmControl[j] != null) {
					sb = new StringBuilder();
					for(int i = 0 ; i < pwmControl[j].getChannels(); i++) { //per channel
						sb.append("PWM channel:");
						sb.append(i+1);
						sb.append(" Min Level:");
						sb.append(pwmControl[j].getMinPWMLevel(i+1));
						sb.append(" Duration:");
						sb.append(pwmControl[j].getMaxPWMDuration(i+1));
						sb.append("\r\n");
					}
					ret.add(sb.toString());
				}
			} // j each PWM controller
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(pwmControlSettingHdr);
			sb.append(MSG_TERMINATE);
			sb.append("\r\n");
			ret.add(sb.toString());
			return ret;
			//
			// M706
			// Report all pins in use
			//
		case 706:
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(pinSettingHdr);
			sb.append(MSG_DELIMIT);
			sb.append("\r\n");
			ret.add(sb.toString());
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(pinSettingHdr);
			sb.append(MSG_TERMINATE);
			sb.append("\r\n");
			ret.add(sb.toString());
			return ret;
			//
			// M798 Z<motor control> [X] 
			// Report controller status for given controller. If X, slot is PWM
			//
		case 798:
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(controllerStatusHdr);
			sb.append(MSG_DELIMIT);
			sb.append("\r\n");
			ret.add(sb.toString());
			if (code_seen('Z')) {
				motorController = (int)code_value();
			}		
			if(code_seen('X')) {
				if(pwmControl[motorController] != null) {
					for(int i = 0; i < pwmControl[motorController].getChannels() ; i++ ) {
						sb = new StringBuilder();
						sb.append("PWM Channel:");
						sb.append(i+1);
						sb.append("\r\n");
						sb.append(pwmControl[motorController].getDriverInfo(i+1));
						sb.append("\r\n");
						ret.add(sb.toString());
					}
				}
			} else {
				if( motorControl[motorController] != null  ) {
					for(int i = 0; i < motorControl[motorController].getChannels() ; i++ ) {
						sb = new StringBuilder();
						sb.append("Motor Channel:");
						sb.append(i+1);
						sb.append("\r\n");
						sb.append(motorControl[motorController].getDriverInfo(i+1));
						sb.append("\r\n");
						ret.add(sb.toString());
					}
				}
			} // code_seen('X')
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(controllerStatusHdr);
			sb.append(MSG_TERMINATE);
			sb.append("\r\n");
			ret.add(sb.toString());	
			return ret;
			//
			// M799 [Z<controller>][X] 
			// Reset controller, if no argument, reset all. If X, slot is PWM
			//
		case 799:
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
						sb.append("\r\n");
						ret.add(sb.toString());
						return ret;
					}
				} else {
					if(motorControl[motorController] != null) {
						try {
							motorControl[motorController].commandEmergencyStop(799);
						} catch (IOException e) {
							e.printStackTrace();
						}
						sb = new StringBuilder();
						sb.append(MSG_BEGIN);
						sb.append("M799");
						sb.append(MSG_TERMINATE);
						sb.append("\r\n");
						ret.add(sb.toString());	
						return ret;
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
				}
				sb.append(MSG_BEGIN);
				sb.append("M799");
				sb.append(MSG_TERMINATE);
				sb.append("\r\n");
				ret.add(sb.toString());	
				return ret;
			}
			//
			// M802 P<n> S<x> M<x>
			// Acquire analog pin data 
			// P=Pin number, S=number readings, M=microseconds per reading. 
			//
		case 802:
			// Publish <dataset> 0 - pin, 1-S - reading
			sb = new StringBuilder();
			/*
		        GpioPinAnalogInput apin = null;
				if( code_seen('P')) {
					codenum = (int) code_value();
					apin = Pins.getAnalogInputPin(codenum);
					if(apin == null) {
						try {
							Pins.assignAnalogInputPin(codenum);
						} catch(GpioPinExistsException gpioe) {
							ret.add(String.format("%s%s:%s%s%n",MSG_BEGIN,MALFORMED_MCODE,gpioe,MSG_TERMINATE));
							return ret;
						}
					}				
				}
			 */
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
				//values[i] = (int) apin.getValue();
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
			ret.add(sb.toString());
			for(int i = 0; i <= nread; i++) {
				sb = new StringBuilder();
				sb.append(i); // sequence
				sb.append(' ');
				if(i == 0)
					sb.append(codenum); // pin at reading 0
				else
					sb.append((values[i-1])); // value
				sb.append("\r\n");
				ret.add(sb.toString());
			}
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(analogPinHdr);
			sb.append(MSG_TERMINATE);
			sb.append("\r\n");
			ret.add(sb.toString());	
			return ret;
			//
			// M999
			// Reset
			//
		case 999:
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
			ret.add(sb.toString());
			return ret;

		default:
			sb = new StringBuilder();
			sb.append(MSG_BEGIN);
			sb.append(MSG_UNKNOWN_MCODE);
			sb.append(":");
			sb.append(cmdbuffer);
			sb.append(MSG_TERMINATE);
			sb.append("\r\n");
			ret.add(sb.toString());
			return ret;

		} // switch m code

		StringBuilder sb = new StringBuilder();
		sb.append(MSG_BEGIN);
		sb.append(MALFORMED_MCODE);
		sb.append(":");
		sb.append(cmdbuffer);
		sb.append(MSG_TERMINATE);
		sb.append("\r\n");
		ret.add(sb.toString());
		return ret;

	} //processMCode

	/**---------------------------------------------------
	 * Arrive here at the end of each command processing iteration to check for status related events
	 * ---------------------------------------------------
	 */
	void manage_inactivity() throws IOException {
		// check motor controllers
		for(int j =0; j < 10; j++) {
			if(motorControl[j] != null) {
				motorControl[j].checkEncoderShutdown();
				motorControl[j].checkUltrasonicShutdown();
				if( motorControl[j].queryFaultFlag() == fault ) {
					fault = motorControl[j].queryFaultFlag();
					if(fault != 10) // dont publish normal encoder shutdown
						publishMotorFaultCode(motorControl[j].getMotorFaultDescriptor(fault));
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
	String publishMotorFaultCode(String fault) {
		if(DEBUG) {
			System.out.printf("%s MOTOR FAULT DETECTED - %s%n", this.getClass().getName(), fault);
		}
		StringBuilder sb = new StringBuilder();
		sb.append(MSG_BEGIN);
		sb.append(motorFaultCntrlHdr);
		sb.append(MSG_DELIMIT);
		sb.append("\r\n");
		sb.append(fault);
		sb.append(MSG_BEGIN);
		sb.append(motorFaultCntrlHdr);
		sb.append(MSG_TERMINATE);
		sb.append("\r\n");
		return(sb.toString());
	}
	//
	ArrayList<String> publishBatteryVolts(int volts) {
		ArrayList<String> sb = new ArrayList<String>();
		sb.add(MSG_BEGIN+batteryCntrlHdr+MSG_DELIMIT+"\r\n");
		sb.add("1 "+volts+"\r\n");
		sb.add(MSG_BEGIN+batteryCntrlHdr+MSG_TERMINATE+"\r\n");
		return sb;
	}
	// **********************************************************************
	// only call this if we know code is stall                              
	// **********************************************************************
	ArrayList<String> publishMotorStatCode(String stat) {
		ArrayList<String> ret = new ArrayList<String>();
		StringBuilder sb = new StringBuilder();
		sb.append(MSG_BEGIN);
		sb.append(motorFaultCntrlHdr);
		sb.append(MSG_DELIMIT);
		sb.append("\r\n");
		ret.add(sb.toString());
		sb = new StringBuilder();
		sb.append("1 ");
		sb.append(stat);
		sb.append("\r\n");
		ret.add(sb.toString());
		sb = new StringBuilder();
		sb.append(MSG_BEGIN);
		sb.append(motorFaultCntrlHdr);
		sb.append(MSG_TERMINATE);
		sb.append("\r\n");
		ret.add(sb.toString());
		return ret;
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

		void printAnalog(GpioPinAnalogInput pin, int index) {
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
	 */
	/**
	 * 'target' represents the expected value. Two elements returned in sequence. 1 - Pin, 2 - reading

		void printDigital(GpioPinDigitalOutput dpin, boolean target) {
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
	 */

}
