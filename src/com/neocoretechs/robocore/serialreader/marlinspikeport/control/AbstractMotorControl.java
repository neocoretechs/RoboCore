package com.neocoretechs.robocore.serialreader.marlinspikeport.control;

import java.io.IOException;

import com.neocoretechs.robocore.serialreader.Ultrasonic;
import com.neocoretechs.robocore.serialreader.marlinspikeport.CounterInterruptService;
import com.neocoretechs.robocore.serialreader.marlinspikeport.PCInterrupts;
import com.pi4j.io.gpio.PinState;

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
	public static int channels = 10;
	// Ultrasonic arrays by channel:
	private Ultrasonic[] usensor = new Ultrasonic[channels];
	private int[] minMotorDist = new int[]{0,0,0,0,0,0,0,0,0,0,0}; // Ranging device motor shutdown range, by channel
	// Ultrasonic array by channel:
	// 0-index in 'usensor' to ultrasonic distance sensor for minimum distance safety shutdown.
	// 1-forward or reverse facing ultrasonic (1 forward)
	private int[][] ultrasonicIndex = new int[][]{{255,1},{255,1},{255,1},{255,1},{255,1},{255,1},{255,1},{255,1},{255,1},{255,1}};
	private int[] maxMotorDuration = new int[]{1,1,1,1,1,1,1,1,1,1,1}; // number of pin change interrupts from wheel encoder before safety interlock
	// 10 channels of last motor speed
	protected int[] motorSpeed = new int[]{0,0,0,0,0,0,0,0,0,0};
	protected int[] currentDirection = new int[]{0,0,0,0,0,0,0,0,0,0};
	protected int[] defaultDirection = new int[]{0,0,0,0,0,0,0,0,0,0};
	protected int[] minMotorPower = new int[]{0,0,0,0,0,0,0,0,0,0}; // Offset to add to G5, use with care, meant to compensate for mechanical differences
	protected CounterInterruptService[] wheelEncoderService = new CounterInterruptService[channels]; // encoder service
	protected PCInterrupts[] wheelEncoder = new PCInterrupts[channels];
	protected int MOTORPOWERSCALE = 0; // Motor scale, divisor for motor power to reduce 0-1000 scale if non zero
	protected boolean MOTORSHUTDOWN = true; // Override of motor controls, puts it up on blocks
	protected int MAXMOTORPOWER = 1000; // Max motor power in PWM final timer units
	protected int fault_flag = 0;
	public abstract int commandMotorPower(int ch, int p) throws IOException;//make AbstractMotorControl not instantiable
	public abstract int commandEmergencyStop(int status) throws IOException;
	public abstract int isConnected() throws IOException;
	public abstract String getDriverInfo(int ch);
	public abstract int queryFaultFlag();
	public abstract int queryStatusFlag();
	public void linkDistanceSensor(int channel, Ultrasonic us, int distance, int facing) {
		int i;
		for(i = 0; i < channels; i++) {
			if(usensor[i] == null)
				break;
		}
		usensor[i] = us;
		ultrasonicIndex[channel-1][0] = i;
		ultrasonicIndex[channel-1][1] = facing;
		minMotorDist[channel-1] = distance;
	}
	/**
	 * check all linked ultrasonic sensors, if something is in minimum range, and it is in the direction
	 * of current travel as defined by the currentDirection array and the direction the sensor is facing, shut down all channels.
	 * First check to see if any channels are active.
	 * The premise is that the distance from the sensor to 'front' of robot is set to prevent impact
	 * @return
	 * @throws IOException
	 */
	public boolean checkUltrasonicShutdown() throws IOException {
		boolean shutdown = false;
		for(int i = 0; i < channels; i++)
			if( motorSpeed[i] != 0 ) {
				break;
			}
		if( shutdown )
			return shutdown;
		// If we have a linked distance sensor. check range and possibly skip
		// ultrasonicIndex corresponds to ultrasonic object pointer array, element 0 points to Ultrasonic array element
		for(int i = 0; i < channels; i++) {
			if( ultrasonicIndex[i][0] != 255 ) {
				// does the direction of sensor match movement direction?
				// we stop if Moving backwards with backward facing sensor or forward with forward facing
				// If motor is mirrored such the speed commands are reversed, then default direction should initially be 1
				// So the decision to stop is based on distance from obstacle, the current direction of travel,
				// the desired direction of travel, and the way the sensor is facing.
				if( currentDirection[i] != 0 && ultrasonicIndex[i][1] != 0 ||
					 currentDirection[i] == 0 && ultrasonicIndex[i][1] == 0 ) {
					if( usensor[ultrasonicIndex[i][0]].getRange() < minMotorDist[i] ) {
						//commandEmergencyStop();
						shutdown = true;
						break;
					}
				}
			}
		}
		if( shutdown ) commandEmergencyStop(8);
		return shutdown;
	}
	/**
	* If we are using an encoder check the interval since last command.
	* Interrupt service counter counts number of timer compare match resets.
	* If number is exceeded issue shutdown and await next G5.
	* This shutdown is to prevent unchecked freewheeling.
	*/
	public boolean checkEncoderShutdown() throws IOException {
		boolean running = false;
		for(int i = 0; i < channels; i++)
			if( motorSpeed[i] != 0 ) {
				running = true;
				break;
			}
		if( !running )
			return running;
		for(int j = 0; j < channels; j++) { // by channel
			if( wheelEncoderService[j] != null ) {
				  int cntxmd = wheelEncoderService[j].get_counter();
				  if( cntxmd >= maxMotorDuration[j] ) {
						commandEmergencyStop(10);
						return true;
				  }
			}
		}
		return false;
	}
	public void createEncoder(int channel, int encode_pin) {
		wheelEncoderService[channel-1] = new CounterInterruptService(encode_pin, maxMotorDuration[channel-1]);
		wheelEncoder[channel-1] = PCInterrupts.getInstance();
		wheelEncoder[channel-1].attachInterrupt(encode_pin, wheelEncoderService[channel-1], PinState.HIGH);
	}
	public void setCurrentDirection(int ch, int val) { currentDirection[ch-1] = val; }
	// If the wheel is mirrored to speed commands or commutation, 0 - normal, 1 - mirror
	public void setDefaultDirection(int ch, int val) { defaultDirection[ch-1] = val; }
	public void setDuration(int ch, int durx) { maxMotorDuration[ch-1] = durx; }
	public void setMinMotorPower(int ch, int mpow) { 
		minMotorPower[ch-1] = mpow; 	
	}
	public int getEncoderCount(int ch) {
		if( wheelEncoderService[ch-1] != null )
			return wheelEncoderService[ch-1].get_counter();
		return -1;
	}
	public int totalUltrasonics() {  
		int j = 0; 
		for(int i = 0; i < channels; i++) 
			if(ultrasonicIndex[i][0] != 255)
				++j; 
		return j; 
	}
	public int getUltrasonicFacing(int ch) { return ultrasonicIndex[ch-1][1]; }
	public int getMinMotorDist(int ch) { return minMotorDist[ch-1]; }
	public int getUltrasonicIndex(int ch) { return ultrasonicIndex[ch-1][0]; }
	public int getMaxMotorDuration(int ch) { return maxMotorDuration[ch-1]; }
	public int getMinMotorPower(int ch) { return minMotorPower[ch-1] ; }
	public int getMaxMotorPower() { return MAXMOTORPOWER; }
	public void setMaxMotorPower(int p) { MAXMOTORPOWER = p; }
	public void setMotorSpeed(int ch, int speed) { motorSpeed[ch-1] = speed;};
	public int getMotorSpeed(int ch) { return motorSpeed[ch-1]; }
	public int getCurrentDirection(int ch) { return currentDirection[ch-1]; }
	public int getDefaultDirection(int ch) { return defaultDirection[ch-1]; }
	public PCInterrupts getWheelEncoder(int ch) { return wheelEncoder[ch-1]; }
	public CounterInterruptService getWheelEncoderService(int ch) { return wheelEncoderService[ch-1]; }
	public void setChannels(int ch) { channels = ch; }
	public int getChannels() { return channels; }
	public void resetSpeeds() {
		for(int i = 0; i < channels; i++) 
			motorSpeed[i] = 0; // all channels down
	}
	public void resetEncoders() {
		for(int i = 0; i < channels; i++) {
			if( wheelEncoderService[i] != null) {
				wheelEncoderService[i].set_counter(0);
			}
		}
	}
	public void setMotorShutdown() throws IOException { 
		commandEmergencyStop(1); 
		MOTORSHUTDOWN = true;
	}
	public void setMotorRun() throws IOException { 
		commandEmergencyStop(0); 
		MOTORSHUTDOWN = false;
	}
	public boolean getMotorShutdown() { return MOTORSHUTDOWN; }
	public void setMotorPowerScale(int p) { MOTORPOWERSCALE = p;}
	public int getMotorPowerScale() { return MOTORPOWERSCALE; }
}
