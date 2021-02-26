package com.neocoretechs.robocore.marlinspike;

import java.util.Arrays;
import java.util.stream.Stream;

/**
 * M10 Controller types - Type 0=Smart controller, Type 1=HBridge, Type 2=Split Bridge, Type 3=Switch Bridge
 * Type 4=PWM driver which uses slots separate from controller types
 * After M10 definition, these codes configure the defined driver:
 * M2 - define smart controller. M2 [Z<slot>] [C<channel> W<encoder pin> E<default dir>]
 * M3 - define H-bridge. M3 [Z<slot>] P<pin> C<channel> D<direction pin> E<default dir> W<encoder pin> [R<resolution 8,9,10 bits>] [X<prescale 0-7>]
 * M4 - define Split Bridge. M4 [Z<slot>] P<pin> Q<pin> C<channel> D<enable pin> E<default dir> [W<encoder pin>] [R<resolution 8,9,10 bits>] [X<prescale 0-7>]
 * M5 - define Switch Bridge. M5 Z<slot> P<pin> Q<pin> C<channel> D<enable pin> E<default dir> [W<encoder>]
 * M9 - define PWM driver. M9 [Z<slot>] P<pin> C<channel> D<enable pin> [R<resolution 8,9,10 bits>] [X<prescale 0-7>
 * @author groff
 *
 */
public class TypeSlotChannelEnable {
	String cntrltype;
	int slot;
	int channel;
	int enable;
	int dirdefault = 0;
	int M10CtrlType = -1;
	/**
	 * M10 controller types
	 */
	public enum typeNames {
		SMARTCONTROLLER("SmartController"),
		HBRIDGE("H-Bridge"),
		SPLITBRIDGE("SplitBridge"),
		SWITCHBRIDGE("SwitchBridge"),
		PWM("PWM");
		String name;
		typeNames(String name) { this.name= name;} 
		public String val() { return name; }
	};

	String[] configCodes = {"2","3","4","5","9"};
	
	public TypeSlotChannelEnable(String cntrltype, int slot, int channel, int enable) {
		this.cntrltype = cntrltype;
		this.slot = slot;
		this.channel = channel;
		this.enable = enable;
	}
	
	public TypeSlotChannelEnable(String cntrltype, int slot, int channel, int enable, int dirdefault) {
		this.cntrltype = cntrltype;
		this.slot = slot;
		this.channel = channel;
		this.enable = enable;
		this.dirdefault = dirdefault;
	}
	/**
	 * CALL THIS FIRST to establish type of controller for further operations.
	 * Define diff driver for traction in slot 0, Type 1 is HBridge driver in motor driver slots
	 * M10 Z0 T1
	 * Define H-bridge driver slot 1 for boom actuator, PWM pin 9 channel 1, Dir pin 26, no encoder
	 * M10 Z1 T1
	 * Define PWM LED driver Type 4 control in slot 0, Type 4 is a PWM driver in separate slots from motor drivers
	 * M10 Z0 T4
	 * Now Define SplitBridge lift actuator slot 2 to Type 2 split bridge driver in motor driver slot
	 * M10 Z2 T2
	 * @return
	 */
	public String genM10() {
		typeNames[] t = typeNames.values();
		typeNames ctrl = Arrays.stream(t).filter(e -> e.name.equals(cntrltype)).findFirst().get();
		M10CtrlType = ctrl.ordinal();
		return new StringBuilder("M10 ").append("Z").append(slot).append(" T").append(M10CtrlType).append("\r\n").toString();
	}
	/**
	 * Generate preamble to control<p/>
	 * sequence is genTypeAndSlot().genDrivePins(pri, sec).genChannelDirDefaultEncoder(encoder);
	 * M2 - define smart controller. M2 [Z<slot>] [C<channel> W<encoder pin> E<default dir>]
 	 * M3 - define H-bridge. M3 [Z<slot>] P<pin> C<channel> D<direction pin> E<default dir> W<encoder pin> [R<resolution 8,9,10 bits>] [X<prescale 0-7>]
	 * M4 - define Split Bridge. M4 [Z<slot>] P<pin> Q<pin> C<channel> D<enable pin> E<default dir> [W<encoder pin>] [R<resolution 8,9,10 bits>] [X<prescale 0-7>]
 	 * M5 - define Switch Bridge. M5 Z<slot> P<pin> Q<pin> C<channel> D<enable pin> E<default dir> [W<encoder>]
	 * M9 - define PWM driver. M9 [Z<slot>] P<pin> C<channel> D<enable pin> [R<resolution 8,9,10 bits>] [X<prescale 0-7>
	 * @return The partially formed directive to send to Marlinspike for config
	 */
	public String genTypeAndSlot() {
		return new StringBuilder("M").append(configCodes[M10CtrlType]).append(" Z").append(slot).toString();
	}
	
	public String genDrivePins(int primary, int secondary) {
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
	 * Generate remaining portion of config
	 * @param encoder
	 * @return
	 */
	public String genChannelDirDefaultEncoder(int encoder) {
		StringBuilder sb = new StringBuilder(" C").append(channel);
		if(M10CtrlType == 0) { // smart controller, may or may not have encoder defined with its firmware, not ours
			if(encoder != 0)
				sb.append(" W").append(encoder);
			return sb.append(" E").append(dirdefault).toString();
		}
		if(M10CtrlType == 4) { // straight PWM driver, no motor, hence, no encoder
			return sb.append(" D").append(dirdefault).toString();
		}
		sb.append(" D").append(dirdefault).append(" E").append(dirdefault).toString();
		if(encoder != 0)
			sb.append(" W").append(encoder);
		return sb.append("\r\n").toString();
	}
	
	public static void main(String[] args) {
		TypeSlotChannelEnable tsce = new TypeSlotChannelEnable("SmartController", 0, 1, 22);
		System.out.println(tsce.genM10());
		StringBuilder sb = new StringBuilder(tsce.genTypeAndSlot()).append(tsce.genDrivePins(0, 0)).append(tsce.genChannelDirDefaultEncoder(0));
		System.out.println(sb);
	}
}
