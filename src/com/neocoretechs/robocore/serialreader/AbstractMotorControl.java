package com.neocoretechs.robocore.serialreader;
/**
* AbstractMotorControl
* Class to maintain the abstract collection of propulsion channels that comprise the traction power.
* Be they brushed motors driven by H-bridge, brushless DC, or a smart controller that uses a high level AT command protocol
* Controllers are channel-oriented. They manage collection of objects representing channels, or conceptually, wheels.
* IMPORTANT: Channels are referenced by the M and G code as a value from 1-10 but the indexes are from 0-9 in the arrays
* of channels, therefore, always subtract 1 when going from methods that take values from the processing loop, such as:
* commandMotorPower
* getDriverInfo
*
* Structure:
* 1) Since we can drive motors via PWM or switched on/off GPIO pins, with either split inputs with separate enable pins or
* one enable pin on a forward/reverse controller, we delegate those functions to the subclasses.
* 2) In each class and subclass, a sublist of pins. Multidimensional arrays which hold the index to the top level pins.
* these are indexed by channel. ultrasonicIndex, minMotorDist, etc. here. All PWM has a pin, a prescale, and a resolution. 
* We standardize the resolution to 8 bits typically.
* 3) Optionally, a duration and a minimum motor power. The duration represents a an abstraction of the maximum interval before the device
* is automatically shut down. In this case, the hall effect interrupts that are serviced by an interrupt handler that detect wheel rotation.
* The minimum motor power is an unsigned value that is added to the base power level typically to compensate for differences in
* motor integrity affecting values that represent the same speed on different channels.
* 4) The motorSpeed is indexed by channel and the value is the range that comes from the main controller, before any processing into a timer value.
* 5) the current direction and default direction have different meanings depending on subclass.
*
* Types of low level DC drivers supported:
* HBridge - A low level motor PWM driver that uses 1 enable pin with 2 states (logic high/low), to drive a mortor in the forward or backward direction.
* This is the most common type of low level PWM DC motor driver.
*
* SplitBridge - A low level motor PWM driver that uses 2 separate enable pins, each with one state (logic high),
* such that the driver can be configured to drive 2 motors in a forward direction,
* one motor in a forward or backward direction, or
* the two channels can be ganged together to deliver twice the current to one motor in a forward direction,
* all with a variable speed control via PWM.
*
* SwitchBridge - A low level motor driver that does not use PWM and that uses 2 separate enable pins, each with one state (logic high).
* Instead of PWM, full drive current is delivered via a switched mechanical or electronic device such as
* a mechanical or solid state relay using MOSFET transistor, BJT transistor, or IGBT transistor. By necessity it
* typically uses 2 separate enable pins that enable the operation of a DC motor in the same fashion as a
* SplitBridge but without variable speed control.
*
* Variable PWM Driver - A low level PWM driver that uses a single enable pin with one state(logic high) to switch the driver on or off.
* This type of driver would typically be used for LEDs with variable brightness, a single motor in the forward direction, etc.
*
* Originally Created: 10/2/2016 12:53:49 PM
* @author: Jonathan Groff Copyright (C) NeoCoreTechs 2022
*/
public abstract class AbstractMotorControl {
	private int channels = 0;
	// Ultrasonic arrays by channel:
	//private Ultrasonic usensor;
	private int[] minMotorDist = new int[]{0,0,0,0,0,0,0,0,0,0}; // Ranging device motor shutdown range, by channel
	// Ultrasonic array by channel:
	// 0-index in 'usensor' to ultrasonic distance sensor for minimum distance safety shutdown.
	// 1-forward or reverse facing ultrasonic (1 forward)
	private int[][] ultrasonicIndex = new int[][]{{255,1},{255,1},{255,1},{255,1},{255,1},{255,1},{255,1},{255,1},{255,1},{255,1}};
	private int[] maxMotorDuration = new int[]{4,4,4,4,4,4,4,4,4,4}; // number of pin change interrupts from wheel encoder before safety interlock
	// 10 channels of last motor speed
	protected int[] motorSpeed = new int[]{0,0,0,0,0,0,0,0,0,0};
	protected int[] currentDirection = new int[]{0,0,0,0,0,0,0,0,0,0};
	protected int[] defaultDirection = new int[]{0,0,0,0,0,0,0,0,0,0};
	protected int[] minMotorPower = new int[]{0,0,0,0,0,0,0,0,0,0}; // Offset to add to G5, use with care, meant to compensate for mechanical differences
	//protected CounterInterruptService[] wheelEncoderService = new CounterInterrupService[10]; // encoder service
	//protected PCInterrupts[] wheelEncoder = new PCInterrupts[10];
	protected int MOTORPOWERSCALE = 0; // Motor scale, divisor for motor power to reduce 0-1000 scale if non zero
	protected int MOTORSHUTDOWN = 0; // Override of motor controls, puts it up on blocks
	protected int MAXMOTORPOWER = 255; // Max motor power in PWM final timer units
	protected int fault_flag = 0;
	public abstract int commandMotorPower(int ch, int p);//make AbstractMotorControl not instantiable
	public abstract int commandEmergencyStop(int status);
	public abstract int isConnected();
	public abstract void getDriverInfo(int ch, char outStr);
	public abstract int queryFaultFlag();
	public abstract int queryStatusFlag();
	//public abstract void linkDistanceSensor(Ultrasonic us, int upin, int distance, int facing);
	public abstract boolean checkUltrasonicShutdown();
	public abstract boolean checkEncoderShutdown();
	public abstract void createEncoder(int channel, int encode_pin);
	public void setCurrentDirection(int ch, int val) { currentDirection[ch-1] = val; }
	// If the wheel is mirrored to speed commands or commutation, 0 - normal, 1 - mirror
	public void setDefaultDirection(int ch, int val) { defaultDirection[ch-1] = val; }
	public void setDuration(int ch, int durx) { maxMotorDuration[ch-1] = durx; }
	public void setMinMotorPower(int ch, int mpow) { 
		minMotorPower[ch-1] = mpow; 	
		if( mpow != 0 ) minMotorPower[ch-1] /= 4; 
	}
	public abstract int getEncoderCount(int ch);
	public int totalUltrasonics() {  
		int j = 0; 
		for(int i = 0; i < 10; i++) 
			if(ultrasonicIndex[i][0] != 255)
				++j; 
		return j; 
	}
	public int getUltrasonicFacing(int ch) { return ultrasonicIndex[ch-1][1]; }
	public int getMinMotorDist(int ch) { return minMotorDist[ch-1]; }
	public int getUltrasonicIndex(int ch) { return ultrasonicIndex[ch-1][0]; }
	public int getMaxMotorDuration(int ch) { return maxMotorDuration[ch-1]; }
	public int getMinMotorPower(int ch) { return minMotorPower[ch-1] ; }
	public void setMaxMotorPower(int p) { MAXMOTORPOWER = p; if( p != 0 ) MAXMOTORPOWER /= 4; }
	public int getMotorSpeed(int ch) { return motorSpeed[ch-1]; }
	public int getCurrentDirection(int ch) { return currentDirection[ch-1]; }
	public int getDefaultDirection(int ch) { return defaultDirection[ch-1]; }
	//public PCInterrupts getWheelEncoder(int ch) { return wheelEncoder[ch-1]; }
	//public CounterInterruptService getWheelEncoderService(int ch) { return wheelEncoderService[ch-1]; }
	public void setChannels(int ch) { channels = ch; }
	public int getChannels() { return channels; }
	public abstract void resetSpeeds();
	public abstract void resetEncoders();
	public void setMotorShutdown() { commandEmergencyStop(1); MOTORSHUTDOWN = 1;}
	public void setMotorRun() { commandEmergencyStop(0); MOTORSHUTDOWN = 0;}
	public int getMotorShutdown() { return MOTORSHUTDOWN; }
	public void setMotorPowerScale(int p) { MOTORPOWERSCALE = p; if( p != 0 ) MOTORPOWERSCALE /= 4;}
}
