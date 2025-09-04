package com.neocoretechs.robocore.marlinspike;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.neocoretechs.robocore.marlinspike.gcodes.G5;
import com.neocoretechs.robocore.marlinspike.mcodes.M42;
import com.neocoretechs.robocore.marlinspike.mcodes.status.digitalpin;

/**
 * Class to facilitate the creation of logical controllers in the Marlinspike microcontroller subsystem.<p/>
 * M10 Controller types - Type 0=Smart controller, Type 1=HBridge, Type 2=Split Bridge, Type 3=Switch Bridge, Type 4= SwitchHBridge<p/>
 * Type 5=PWM driver which uses slots separate from controller types. Its different because it doesnt have directions and encoders etc.
 * and as such is derived from a different base class in the Marlinspike object hierarchy so the polymorphism is
 * Separately arranged..<p/>
 * Example organization of Robocore.Properties parameters:<br>
 * LUN[0].Name:LeftWheel<br>
 * LUN[0].NodeName:CONTROL1<br>
 * #LUN[0].Controller:/dev/ttyACM0 (this is commented out optional configuration 1)<br>
 * LUN[0].Controller:MarlinspikeDataPort<br>
 * LUN[0].Type:H-Bridge<br>
 * LUN[0].Slot:0<br>
 * #LUN[0].SignalPin0:8 (optional alternate config 1)<br>
 * LUN[0].SignalPin0:33<br>
 * LUN[0].Channel:1<br>
 * #LUN[0].EnablePin:38 (optional alternate config 1)<br>
 * LUN[0].EnablePin:2<br>
 * LUN[0].Direction:0<br>
 * #LUN[0].EncoderPin:68 (optional alternate config 1)<br>
 * LUN[0].EncoderPin:40<br>
 * LUN[0].EncoderType:Analog<br>
 * LUN[0].EncoderCount:1<br>
 * LUN[0].EncoderLoRange:1023<br>
 * LUN[0].EncoderHiRange:1023<br>
 * After M10 definition, these codes configure the defined driver:<br>
 * M2 - define SmartController. M2 [Z<slot>] [C<channel> W<encoder pin> E<default dir>] <br>
 * M3 - define H-bridge. M3 [Z<slot>] P<pin> C<channel> D<direction pin> E<default dir> W<encoder pin> [F<frequency>] [G <duty>]<br>
 * M4 - define SplitBridge. M4 [Z<slot>] P<pin> Q<pin> C<channel> D<enable pin> E<default dir> [W<encoder pin>] [F<frequency>] [G<duty>]<br>
 * M5 - define SwitchBridge or SwitchHBridge. M5 Z<slot> P<pin> Q<pin> C<channel> D<enable pin> E<default dir> [W<encoder>] if no Q pin, SwitchHBridge and enable pin is direction<br>
 * M9 - define PWM driver. M9 [Z<slot>] P<pin> C<channel> D<enable pin> [] <br>
 * M16 - define delayed H-bridge. M16 [Z<slot>] P<pin> C<channel> D<direction pin> E<default dir> W<encoder pin> [F<frequency>] [G <duty>]<br>
 * M41 - define basic GPIO output pin linked to control axis. M41 P<pin><br>
 * Additionally, after M10, an M14 or M15 encoder directive may be issued depending on default encoder status.<br>
 * M14 Z<slot> C<channel> P<pin> L<lo> H<hi> N<count>
 * Create digital encoder:<br>
 * M15 Z<slot> C<channel> P<pin> S<state 0-LOW 1-HIGH> N<count><p/>
 * <b>If the Type ends with 'Pin' such as OutputPin LEDDRiver config below, we assume the control is a standalone GPIO pin of some type, 
 * so these names are reserved directives:</b><p/>
 * LUN[3].Name:LEDDriver<br>
 * LUN[3].NodeName:AMI0<br>
 * LUN[3].Controller:MarlinspikeDataPort<br>
 * LUN[3].Type:OutputPin<br>
 * LUN[3].Pin:13<br>
 * LUN[3].Toggle:True
 * AXIS[3].AxisType:Trigger<br>
 * AXIS[3].Axis:4<p/>
 * Create analog encoder:<br>
 *
 * @author Jonathan Groff (C) NeoCoreTechs 2021,2022
 *
 */
public class TypeSlotChannelEnable implements Serializable {
	private static final long serialVersionUID = 1L;
	typeNames cntrltype;
	int pin;
	private int slot;
	int channel;
	int enable;
	int dirdefault = 0;
	int M10CtrlType = -1; // ordinal of typeNames for this instance
	int encoder = 0;
	boolean isAnalogEncoder = false;
	boolean isDigitalEncoder = false;
	int loAnalogEncoderRange = 0;
	int hiAnalogEncoderRange = 0;
	int encoderCount = 1;
	int encInterrupt;
	int digitalEncoderState = 0; // low
	int maxValue = 1000;
	int minValue = -1000;
	boolean pinToggle = false;
	/**
	 * M10 controller types
	 */
	public enum typeNames {
		SMARTCONTROLLER("SmartController"),
		HBRIDGE("H-Bridge"),
		SPLITBRIDGE("SplitBridge"),
		SWITCHBRIDGE("SwitchBridge"),
		SWITCHHBRIDGE("SwitchHBridge"),
		PWM("PWM"),
		INPUTPIN("InputPin"),
		OUTPUTPIN("OutputPin"),
		DELAYHBRIDGE("DelayH-Bridge");
		String name;
		typeNames(String name) { this.name = name;} 
		public String val() { return name; }
		public ActivationInterface activatorFactory(TypeSlotChannelEnable tsce) {
			switch(this) {
				case SMARTCONTROLLER:
				case HBRIDGE:
				case SPLITBRIDGE:
				case SWITCHBRIDGE:
				case SWITCHHBRIDGE:
				case PWM:
				case DELAYHBRIDGE:
					//return String.format("G5 Z%d C%d P%d%n",slot,channel,deviceLevel);
					return new G5(tsce);
				case INPUTPIN:
					//return String.format("M44 P%d%n", pin);
					return new digitalpin(tsce);
				case OUTPUTPIN:
					//if(deviceLevel != maxValue)
						//deviceLevel = 0;
					//return String.format("M42 P%d S%d%n", pin, deviceLevel);
					return new M42(tsce);
			}
			throw new RuntimeException("Bad TypeSlotChannel config "+this);
		}
	};
	
	// M code to generate for ordinal of typeNames
	String[] configCodes = {"2","3","4","5","5","9","43","41","16"};
	
	// This interface is responsible for generating the actual M or G code that is passed to the Marlinspike to activate the device
	private ActivationInterface activator = null;
	
	/**
	 * Create a template to initialize a logical controller within the Marlinspike where we have an enable pin and no direction pin.
	 * @param cntrltype One of SmartController, H-Bridge, SplitBridge, SwitchBridge, PWM etc.
	 * @param slot The numerical slot for the controller there are 10 for motor types and 10 for non motor types.
	 * @param channel The channel within the controller at the numerical slot; 10 channels per slotted controller
	 * @param enable The pin to enable the controller at the slot at the channel in the Marlinspike on the node.
	 */
	public TypeSlotChannelEnable(typeNames cntrltype, int slot, int channel, int enable) {
		this.cntrltype = cntrltype;
		this.slot = slot;
		this.channel = channel;
		this.enable = enable;
		this.activator = cntrltype.activatorFactory(this);
	}
	/**
	 * Create a template to initialize a logical controller within the Marlinspike where we have an enable pin and a direction pin.
	 * @param cntrltype One of SmartController, H-Bridge, SplitBridge, SwitchBridge, PWM etc.
	 * @param slot The numerical slot for the controller there are 10 for motor types and 10 for non motor types.
	 * @param channel The channel within the controller at the numerical slot; 10 channels per sloted controller
	 * @param enable The pin to enable the controller at the slot at the channel in the Marlinspike on the node.
	 * @param dirdefault The default direction for motor 0 is 'forward', 1 is 'reverse' in a case where a drive wheel is wired the same each side.
	 */
	public TypeSlotChannelEnable(typeNames cntrltype, int slot, int channel, int enable, int dirdefault) {
		this.cntrltype = cntrltype;
		this.slot = slot;
		this.channel = channel;
		this.enable = enable;
		this.dirdefault = dirdefault;
		this.activator = cntrltype.activatorFactory(this);
	}
	
	public TypeSlotChannelEnable(typeNames cntrltype, int pin) {
		this.cntrltype = cntrltype;
		this.pin = pin;
		this.activator = cntrltype.activatorFactory(this);
	}
	
	public void setEncoderPin(int ienc) {
		this.encoder = ienc;	
	}
	/**
	 * Minimum drive value, or if digital out pin, min value of control resulting in LOW state
	 * @param minValue
	 */
	public void setMinValue(int minValue) {
		this.minValue = minValue;
	}
	
	public int getMinValue() { return minValue; }
	/**
	 * Maximum drive value, or if digital out pin, max value of control that results in HIGH state
	 * @param maxValue
	 */
	public void setMaxValue(int maxValue) {
		this.maxValue = maxValue;
	}
	
	public int getMaxValue() { return maxValue; }
	/**
	 * CALL THIS FIRST to establish instance of controller for further operations and to refer to it numerically by type.<p/>
	 * M10 initializes a type of controller in a specific 'slot' that becomes an ordinal number we use to refer to the controller
	 * in subsequent M codes. Different controller types occupy different 'slots' depending on the object type hierarchy<p/>
	 * Types 0-3 occupy the 'motor control' slots while type 4 occupies the 'PWM' slot. Each has its own sequence.<p/>
	 * If we are just configuring pins for I/O, dont generate an M10, but perhaps a different M-code such as M41 for an output pin.<p/>
	 * M10 Z(slot) T(type) <br>
	 * M10 Z0 T0 - Smart controller type 0 , slot 0<br>
	 * M10 Z1 T1 - H-Bridge controller type 1, slot 1 <br>
	 * M10 Z2 T2 - Split bridge, type 2 slot 2; each channel has 2 PWM pins and an enable pin, so up to 5 channels <br>
	 * M10 Z3 T3 - Switch bridge, type 3, slot 3, each channel 2 GPIO pins for full forward and back, no PWM, and an enable pin <br>
	 * M10 Z0 T4 - PWM driver Type 4 control in slot 0, Type 4 is a PWM driver in separate slots from motor drivers<br>
	 * M10 Z0 T5 - Switch H bridge, type 5 slot 0, each channel has 1 GPIO pin, no PWN, and enable which functions as direction when<p/>
	 * M10 Z1 T8 - Delay H-Bridge Type 8 slot 1<br>
	 * @param ipin1 PWM primary drive pin0 from MarlinspikeManager.configureMarlinspike and properties file
	 * @param ipin0 PWM secondary drive pin1
	 * @return The M10 directive string, possibly multiple c/r delimited directives relating to configuring the type in the M10 preamble
	 */
	public List<String> genM10(int ipin0, int ipin1) {
		M10CtrlType = cntrltype.ordinal();
		StringBuilder sb = new StringBuilder();
		ArrayList<String> ab =  new ArrayList<String>();
		if(cntrltype.val().endsWith("Pin")) {
			ab.add(sb.append("M").append(configCodes[M10CtrlType]).append(" P").append(pin).append("\r\n").toString());
		} else {
			// Generate the M10 followed by the the M codes to create the type, the encoder, the interrupt linkage, etc.
			ab.add(sb.append("M10 ").append("Z").append(getSlot()).append(" T").append(M10CtrlType).append("\r\n").toString());
			sb = new StringBuilder();
			sb.append(genTypeAndSlot()).append(genDrivePins(ipin0, ipin1)).append(genChannelDirDefaultEncoder());
			ab.add(sb.toString());
			ab.add(genChannelEncoder());
		}
		return ab;
	}
	/**
	 * Call at time of activation of command immediately before sending to Marlinspike to generate proper
	 * M or G code sequence for configured resource in MarlinSpikeControl setDeviceLevel
	 * @param deviceLevel
	 * @return
	 */
	public String genActivate(int deviceLevel) {
		return activator.getActivation(deviceLevel);
	}
	/**
	 * Generate preamble to control<p/>
	 * sequence is genTypeAndSlot().genDrivePins(pri, sec).genChannelDirDefaultEncoder(encoder);
	 * M2 - define smart controller. M2 [Z<slot>] [C<channel> W<encoder pin> E<default dir>]
 	 * M3 - define H-bridge. M3 [Z<slot>] P<pin> C<channel> D<direction pin> E<default dir> W<encoder pin> [F frequency] [G duty]
	 * M4 - define Split Bridge. M4 [Z<slot>] P<pin> Q<pin> C<channel> D<enable pin> E<default dir> [W<encoder pin>] [F frequency] [G duty]
 	 * M5 - define Switch Bridge. M5 Z<slot> P<pin> Q<pin> C<channel> D<enable pin> E<default dir> [W<encoder>]
 	 * M5 - define Switch H Bridge. M5 Z<slot> P<pin> C<channel> D<enable pin> E<default dir> [W<encoder>]
	 * M9 - define PWM driver. M9 [Z<slot>] P<pin> C<channel> D<enable pin> [F frequency] [G Duty]
	 * M16 - define delay H-bridge. M16 [Z<slot>] P<pin> C<channel> D<direction pin> E<default dir> W<encoder pin> [F frequency] [G duty]
	 * @return The partially formed directive to send to Marlinspike for config
	 */
	private String genTypeAndSlot() {
		return new StringBuilder("M").append(configCodes[M10CtrlType]).append(" Z").append(getSlot()).toString();
	}
	
	public typeNames getType() { return cntrltype; }
	/**
	 * @return the slot
	 */
	public int getSlot() {
		return slot;
	}
	
	public int getChannel() {
		return channel;
	}

	public int getPin() {
		return pin;
	}
	// If digital out pin, toggle at high value?
	public boolean isPinToggle() { return pinToggle; }
	public void setPinToggle() { pinToggle = true; }
	/**
	 * Generate the drive pins for the controller, if its a smart controller (M10CtrlType == 0) return an empty string.
	 * If the type is 2 or 3, generate a Q<pin> for secondary drive pin.
	 * @param primary
	 * @param secondary
	 * @return
	 */
	private String genDrivePins(int primary, int secondary) {
		if(M10CtrlType == 0) // If its a smart controller, no drive pins
			return "";
		StringBuilder sb = new StringBuilder(" P");
		sb.append(primary);
		if(M10CtrlType == 2 || M10CtrlType == 3) {
			sb.append(" Q");
			sb.append(secondary);
		}
		return sb.toString();
	}
	/**
	 * Generate remaining portion of config for direction and encoder based on M10 type<p/>
	 * M10 Z0 T0 - Smart controller type 0 , slot 0<br>
	 * M10 Z1 T1 - H-Bridge controller type 1, slot 1 <br>
	 * M10 Z2 T2 - Split bridge, type 2 slot 2; each channel has 2 PWM pins and an enable pin, so up to 5 channels <br>
	 * M10 Z3 T3 - Switch bridge, type 3, slot 3, each channel has 2 GPIO pins for full forward and back, no PWM, and an enable pin <br>
	 * M10 Z0 T4 - Switch H bridge, type 4, slot 0, each channel has 1 signal gpio pin and 1 direction gpio pin<br>
	 * M10 Z0 T5 - PWM driver Type 4 control in slot 0, Type 4 is a PWM driver in separate slots from motor drivers<br>
	 * M10 Z1 T8 - Delay H-Bridge controller type 8, slot 1 <br>
	 * @param encoder The pin that receives hall effect or other encoder pulses per rotation of wheel. If 0, possibly add additional config
	 * @return
	 */
	private String genChannelDirDefaultEncoder() {
		StringBuilder sb = new StringBuilder(" C").append(channel);
		switch(M10CtrlType) { 
			case 0:// smart controller, may or may not have encoder defined with its firmware, not ours
				if(encoder != 0 && !isAnalogEncoder && !isDigitalEncoder)
					sb.append(" W").append(encoder);
				return sb.append(" E").append(dirdefault).append("\r\n").toString();
			case 5:// straight PWM driver, no motor, hence, no encoder
				return sb.append(" D").append(dirdefault).append("\r\n").toString();
			default:
				break;
		}
		// types 1, 2 and 3 have standard D-enable, E-default dir, W-optional encoder
		sb.append(" D").append(enable).append(" E").append(dirdefault).toString();
		if(encoder != 0 && !isAnalogEncoder && !isDigitalEncoder) {
			sb.append(" W").append(encoder);
			if(encInterrupt != 0)
				sb.append(" I").append(encInterrupt);
		}
		return sb.append("\r\n").toString();
	}
	
	private String genChannelEncoder() {
		StringBuilder sb = new StringBuilder();
		if(isAnalogEncoder) {
			sb.append("M14 Z").append(getSlot()).append(" C").append(channel).append(" P").append(encoder).append(" L").append(loAnalogEncoderRange).append(" H").append(hiAnalogEncoderRange).append(" N").append(encoderCount);
			if(encInterrupt != 0)
				sb.append(" I").append(encInterrupt);
			sb.append("\r\n");
		} else { 
			if(isDigitalEncoder) {
				sb.append("M15 Z").append(getSlot()).append(" C").append(channel).append(" P").append(encoder).append(" S").append(digitalEncoderState).append(" N").append(encoderCount);
				if(encInterrupt != 0)
					sb.append(" I").append(encInterrupt);
				sb.append("\r\n");
			}
		}
		return sb.toString();
	}
	
	public void setAnalogEncoder(int iencCount, int iencLoRange, int iencHiRange, int iencInterrupt) {
		isAnalogEncoder = true;
		encoderCount = iencCount;
		loAnalogEncoderRange = iencLoRange;
		hiAnalogEncoderRange = iencHiRange;
		encInterrupt = iencInterrupt;
	}
	
	public void setDigitalEncoder(int iencCount, int iencState, int iencInterrupt) {
		isDigitalEncoder = true;
		encoderCount = iencCount;
		digitalEncoderState = iencState;
		encInterrupt = iencInterrupt;
	}
	
	@Override
	public String toString() {
		String ret;
		if(cntrltype.val().endsWith("Pin")) {
			ret = String.format("Control type: %s pin:%d%n",cntrltype, pin);
		} else {
			ret = String.format("Control type: %s slot:%d channel:%d enable:%d default dir:%d M10 type:%d%n",  
				cntrltype, getSlot(), channel, enable, dirdefault, M10CtrlType);
			if(encoder != 0 && !isAnalogEncoder && !isDigitalEncoder) {
				ret += String.format(" Default encoder at pin:%d",encoder);
				if(encInterrupt != 0)
					ret += String.format(" Encoder Interrupt %d",encInterrupt);
				ret += "\r\n";
			} else {
				if(isAnalogEncoder) {
					ret += String.format(" Analog encoder at pin:%d range lo:%d, hi:%d count%d",encoder, loAnalogEncoderRange, hiAnalogEncoderRange, encoderCount);
					if(encInterrupt != 0)
						ret += String.format(" Encoder Interrupt %d",encInterrupt);
					ret += "\r\n";
				} else {
					if(isDigitalEncoder) {
						ret += String.format(" Digital encoder at pin:%d state:%d count%d",encoder, digitalEncoderState, encoderCount);
						if(encInterrupt != 0)
							ret += String.format(" Encoder Interrupt %d",encInterrupt);
						ret += "\r\n";
					}
				}
			}
		}
		return ret;			
	}
	
	/**
	 * Define diff driver for traction in slot 0, Type 1 is HBridge driver in motor driver slots
	 * M10 Z0 T1
	 * M3 Z0 P8 C1 D24 E0 W68
 	 * Config H-bridge diff driver slot 0 PWM pin 10 channel 2, Dir pin 22, encoder 69 default start dir 0
	 * M3 Z0 P10 C2 D22 E0 W69
 	 * Define H-bridge driver slot 1 for boom actuator, PWM pin 9 channel 1, Dir pin 26, no encoder
	 * M10 Z1 T1
 	 * config H-bridge actuator slot 1, PWM pin 9 channel 1, Dir pin 26, default dir 0
	 * M3 Z1 P9 C1 D26 E0
 	 * Define PWM LED driver Type 4 control in slot 0, Type 4 is a PWM driver in separate slots from motor drivers
	 * M10 Z0 T4
 	 * Now config LED driver slot 0 to PWM pin 13 on channel 1 enable pin 30
	 * M9 Z0 P13 C1 D30
 	 * Now Define SplitBridge lift actuator slot 2 to Type 2 split bridge driver in motor driver slot
	 * M10 Z2 T2
 	 * Config SplitBridge lift actuator slot 2, pwm pins 6 and 7 for forward/reverse channel 1, enable pin 32, default dir 0
	 * M4 Z2 P6 Q7 C1 D32 E0
	 * @param args
	 */
	public static void main(String[] args) {
		System.out.println("-----");		
		TypeSlotChannelEnable tsce = new TypeSlotChannelEnable(typeNames.SMARTCONTROLLER, 0, 1, 22);
		System.out.print(tsce.genM10(0,0));
		StringBuilder sb = new StringBuilder(tsce.genTypeAndSlot()).append(tsce.genDrivePins(0, 0)).append(tsce.genChannelDirDefaultEncoder());
		System.out.print(sb);
		System.out.println("-----");
		tsce = new TypeSlotChannelEnable(typeNames.HBRIDGE, 0, 1, 24, 1);
		System.out.print(tsce.genM10(0,0));
		tsce.setEncoderPin(68);
		sb = new StringBuilder(tsce.genTypeAndSlot()).append(tsce.genDrivePins(8, 0)).append(tsce.genChannelDirDefaultEncoder());
		System.out.print(sb);
		System.out.println("-----");		
		tsce = new TypeSlotChannelEnable(typeNames.PWM, 0, 1, 30);
		System.out.print(tsce.genM10(0,0));
		sb = new StringBuilder(tsce.genTypeAndSlot()).append(tsce.genDrivePins(13, 0)).append(tsce.genChannelDirDefaultEncoder());
		System.out.print(sb);
		System.out.println("-----");		
		tsce = new TypeSlotChannelEnable(typeNames.SPLITBRIDGE, 2, 1, 32);
		System.out.print(tsce.genM10(0,0));
		sb = new StringBuilder(tsce.genTypeAndSlot()).append(tsce.genDrivePins(6, 7)).append(tsce.genChannelDirDefaultEncoder());
		System.out.print(sb);
		System.out.println("-----");
		tsce = new TypeSlotChannelEnable(typeNames.SWITCHBRIDGE, 2, 1, 32);
		System.out.print(tsce.genM10(0,0));
		sb = new StringBuilder(tsce.genTypeAndSlot()).append(tsce.genDrivePins(6, 7)).append(tsce.genChannelDirDefaultEncoder());
		System.out.print(sb);
		System.out.println("-----");
	}


}
