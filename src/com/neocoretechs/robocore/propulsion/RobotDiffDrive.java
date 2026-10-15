package com.neocoretechs.robocore.propulsion;

import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.stream.Collectors;

import com.neocoretechs.robocore.config.Robot;
import com.neocoretechs.robocore.config.RobotInterface;
import com.neocoretechs.robocore.config.SlotEntry;
import com.neocoretechs.robocore.config.TypedWrapper;
import com.neocoretechs.robocore.marlinspike.MarlinspikeControlInterface;
import com.neocoretechs.robocore.marlinspike.SlotHandler;
import com.neocoretechs.robocore.marlinspike.TypeSlotChannelEnable;
import com.neocoretechs.robocore.marlinspike.TypeSlotChannelEnable.typeNames;

/**
* Configuration for generic robot differential drive.
* Based on the Marlinspike specification, there can be up to 10 AbstractMotorControl "slots".<p>
* Each "slot" can have up to 10 PWM pins (wheels) defined<p>
* So if you are powering an Ogre from the famous microgame "GEV", you can have 5 track banks on each side of the tank with each bank containing 10 tracks<p>
* This would translate to slots 0-9 each of which has channels 1-10 <p>
* However on the pico you are limited to 8 "slices" for PWM designations each with two A and B channels. so you
* would have to power 5 tracks on each side of the Ogre and switch them as they got knocked out by GEVs or
* artillery.<p>
* @author Jonathan Groff (C) NeoCoreTechs 2020,2021
*/
public class RobotDiffDrive extends SlotEntry implements RobotDiffDriveInterface, Serializable {
	public static boolean DEBUG = false;
	private static final long serialVersionUID = 1L;
	RobotInterface robot;
	TypedWrapper[] LUN;
	TypedWrapper[] WHEEL;
	TypedWrapper[] AXIS;
	TypedWrapper[] PID;
	int controllerAxisX = -1; 
	int controllerAxisY = -1;
	int channels = 0;

	/**
	 * Controller, devicename, slot goes into the initial slothandler entry, which is first wheel encountered.
	 * This sets up the sort order in the deviceToType mapping based on nodename, name
	 * @param deviceName
	 * @param NodeName
	 * @param LUN
	 * @param controller
	 * @param slot
	 */
	public RobotDiffDrive(RobotInterface robot2, String deviceName, String NodeName, int LUN, MarlinspikeControlInterface controller, int slot, TypedWrapper[] lUN, TypedWrapper[] wHEEL, TypedWrapper[] aXIS, TypedWrapper[] pID) {
		super(deviceName, NodeName, LUN, controller, slot);
		this.robot = robot2;
		this.LUN = lUN;
		this.WHEEL = wHEEL;
		this.AXIS = aXIS;
		this.PID = pID;
	}


	public static RobotWheel setDrive(TypedWrapper[] LUN, TypedWrapper[] WHEEL, TypedWrapper[] PID, String name, int wheelLun, int slot, int chan) {
		Optional<Float> wheelTrack = Optional.empty();// Props.toFloat("WheelTrackMM"); // millimeters
		Optional<Float> wheelDiameter = Optional.empty(); //Props.toFloat("WheelDiameterMM"); // millimeters, 16"
		Optional<Integer> ticksPerRevolution = Optional.empty();
		Optional<Float> motorKp = Optional.empty();
		Optional<Float> motorKd = Optional.empty();
		Optional<Float> motorKi = Optional.empty();
		Optional<Float> motorKo = Optional.empty();
		Optional<Integer> motorPIDRate = Optional.empty();
		Optional<Integer> minSpeed = Optional.empty();
		Optional<Integer> maxSpeed = Optional.empty();
		Optional<String> type = Optional.empty();
		Optional<Integer> enablePin = Optional.empty();
		
		type = Optional.ofNullable(LUN[wheelLun].get("Type")).filter(String.class::isInstance).map(e -> String.valueOf(e));
		enablePin = Optional.ofNullable(LUN[wheelLun].get("EnablePin")).filter(String.class::isInstance)
				.map(e -> Integer.parseInt((String) e));
		
		minSpeed = Optional.ofNullable(LUN[wheelLun].get("MinValue")).filter(String.class::isInstance)
				.map(e -> Integer.parseInt((String) e));
		maxSpeed = Optional.ofNullable(LUN[wheelLun].get("MaxValue")).filter(String.class::isInstance)
				.map(e -> Integer.parseInt((String) e));

		wheelTrack = Optional.ofNullable(WHEEL[wheelLun].get("WheelTrackMM")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		wheelDiameter = Optional.ofNullable(WHEEL[wheelLun].get("WheelDiameterMM")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		ticksPerRevolution = Optional.ofNullable(WHEEL[wheelLun].get("TicksPerRevolution")).filter(String.class::isInstance)
				.map(e -> Integer.parseInt((String) e));

		motorKp = Optional.ofNullable(PID[wheelLun].get("MotorKp")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		motorKd = Optional.ofNullable(PID[wheelLun].get("MotorKd")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		motorKi = Optional.ofNullable(PID[wheelLun].get("MotorKi")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		motorKo = Optional.ofNullable(PID[wheelLun].get("MotorKo")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		motorPIDRate = Optional.ofNullable(PID[wheelLun].get("MotorPIDRate")).filter(String.class::isInstance)
				.map(e -> Integer.parseInt((String) e));

		try {
			typeNames eType = null;
			//eType = typeNames.valueOf(type) will fail because we use mixed case names;
			for(typeNames et : typeNames.values()) {
				if(et.name.equals((String)type.get())) {
					eType = et;
					break;
				}
			}
			if(eType == null)
				throw new RuntimeException("Type of:"+(String)type.get()+" did not resolve to a legitimate type for "+name+" slot "+slot+" channel "+chan);
			
			return new RobotWheel(name, eType, slot, chan, enablePin.get(),
					wheelDiameter.get(), 
					wheelTrack.get(), 
					minSpeed.get(), 
					maxSpeed.get(), 
					ticksPerRevolution.get(), 
					motorKp.get(), motorKd.get(), 
					motorKi.get(), motorKo.get(),
					motorPIDRate.get());
		} catch(NoSuchElementException nse) {
			System.out.println("enable="+enablePin.get());
			System.out.println("wheelDiameter="+wheelDiameter.get());
			System.out.println("wheelTrack="+wheelTrack.get());
			System.out.println("minSpeed="+minSpeed.get());
			System.out.println("maxSpeed="+maxSpeed.get());
			System.out.println("ticksPerRevolution="+ticksPerRevolution.get());
			System.out.println("motorKp="+motorKp.get());
			System.out.println("motorKd="+motorKd.get());
			System.out.println("motorKi="+motorKi.get());
			System.out.println("motorKo="+motorKo.get());
			System.out.println("motorPIDRate="+motorPIDRate.get());
			throw new RuntimeException("Cannot configure robot wheel "+name+" LUN "+wheelLun+" slot:"+slot+" channel:"+chan);
		}

	}
	
	@Override
	/**
	 * Return the wheel channels for a given slot
	 * @param slot target slot
	 * @return ArrayList of channels of DriveWheelInterface for this slot or null if slot wasnt found
	 */
	public ArrayList<DrivenWheelInterface> getWheels(int slot) {
		ArrayList<TypeSlotChannelEnable> atsce = robot.getManager().getSlot(slot);
		if(atsce == null)
			return null;
		ArrayList<DrivenWheelInterface> rslot = null;
		if(atsce.getFirst() instanceof DrivenWheelInterface) {
			rslot = new ArrayList<DrivenWheelInterface>(atsce.size());
			for(TypeSlotChannelEnable tsce: atsce)
				rslot.add((RobotWheel)tsce);
		}
		return rslot;
	}

	/**
	 * Get the wheel for the given slot row and channel
	 * @param slot The array of channels for this slot from getWheels
	 * @param chan the target channel
	 * @return RobotWheel or null if slots is null or channel cant be found in row for slot.
	 */
	public static DrivenWheelInterface getWheel(ArrayList<DrivenWheelInterface> slot, int chan) {
		if(slot == null)
			return null;
		if(chan <= 0 || chan > 10)
			throw new RuntimeException("Channel must be from 1-10");
		for(int rsl2 = 0; rsl2 < slot.size(); rsl2++) {
			if(slot.get(rsl2).getChannel() == chan)
				return slot.get(rsl2);
		}
		return null;
	}
	
	@Override
	public ArrayList<DrivenWheelInterface> getWheels() {
		 ArrayList<TypeSlotChannelEnable> atsce = robot.getManager().getTypeSlotChannelEnable(this);
		 return atsce.stream().filter(RobotWheel.class::isInstance).map(RobotWheel.class::cast).collect(Collectors.toCollection(ArrayList::new));
	}	
	
	@Override
	public String toString() {
		StringBuffer sb = new StringBuffer();
		ArrayList<DrivenWheelInterface> wheels = getWheels();
		for(int sl = 0; sl < wheels.size(); sl++) {
			DrivenWheelInterface rslot = wheels.get(sl);
				sb.append(sl+1);
				sb.append(".) ");
				sb.append(rslot.toString());
				sb.append("ControllerAxisX=");
				sb.append(controllerAxisX);
				sb.append(" ControllerAxisY=");
				sb.append(controllerAxisY);
				sb.append("\r\n");
		}
		return sb.toString();
	}

	@Override
	public int getControllerAxisX() {
		return controllerAxisX;
	}

	@Override
	public int getControllerAxisY() {
		return controllerAxisY;
	}

	@Override
	public int getChannels() {
		return channels;
	}


}
