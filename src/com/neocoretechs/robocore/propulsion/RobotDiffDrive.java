package com.neocoretechs.robocore.propulsion;

import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.NoSuchElementException;
import java.util.Optional;

import com.neocoretechs.robocore.config.Robot;
import com.neocoretechs.robocore.config.TypedWrapper;
import com.neocoretechs.robocore.marlinspike.MarlinspikeControlInterface;
import com.neocoretechs.robocore.marlinspike.SlotHandler;

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
public class RobotDiffDrive implements RobotDiffDriveInterface, Serializable {
	public static boolean DEBUG = false;
	private static final long serialVersionUID = 1L;
	TypedWrapper[] LUN;
	TypedWrapper[] WHEEL;
	TypedWrapper[] AXIS;
	TypedWrapper[] PID;
	int controllerAxisX = -1; 
	int controllerAxisY = -1;
	int channels = 0;
	// Define the robot parameters, 
	// each of these will be by slot, each slot multiple channels, slot 0, channels 1 and 2 would be left, right front wheels
	// or front and rear 2 wheels depending on stick axis x,y setup for power blending
	private ArrayList<ArrayList<DrivenWheelInterface>> wheels = new ArrayList<ArrayList<DrivenWheelInterface>>(); 
	//public static int MAXOUTPUT = 50; // indoor
	// so ticks are IMU data in mm/s, I hope. In static form its locked to diameter but here, as IMU data, 
	// it is variable based on desired speed.
	private ArrayList<SlotHandler> slotHandlers = new ArrayList<SlotHandler>();
	
	public RobotDiffDrive(TypedWrapper[] lUN, TypedWrapper[] wHEEL, TypedWrapper[] aXIS, TypedWrapper[] pID) {
		this.LUN = lUN;
		this.WHEEL = wHEEL;
		this.AXIS = aXIS;
		this.PID = pID;
		// extract the wheel definitions from the logical units in config
		for(int i = 0; i < LUN.length; i++) {
			String name = ((String)LUN[i].get("Name"));
			if(name.endsWith("Wheel")) {
				Object s = LUN[i].get("Slot");
				if(s == null)
					throw new RuntimeException("Must declare Slot in configuration for differential drive wheel.");
				int slot = Integer.parseInt((String) s);
				Object ochan = LUN[i].get("Channel");
				if(ochan == null)
					throw new RuntimeException("Configuration is absent left or right wheel channel LUN declaration");
				int channel = Integer.parseInt((String) ochan);
				if(channel > channels) channels = channel;

				RobotWheel robotWheel = setDrive(name, i, slot, channel);
				int slotRow = -1;
				outerr: for(int sl = 0; sl < wheels.size(); sl++) {
					ArrayList<DrivenWheelInterface> rslot = wheels.get(sl);
					for(int rsl = 0; rsl < rslot.size(); rsl++) {
						if(rslot.get(rsl).getSlot() == slot) {
							slotRow = rsl;
							// make sure we dont have the channel already for this slot
							for(int rsl2 = 0; rsl2 < rslot.size(); rsl2++) {
								if(rslot.get(rsl2).getChannel() == channel)
									throw new RuntimeException("Duplicate channel "+channel+" for slot "+slot+" for name "+name);
							}
							// got slot, didnt find channel already, add it to row
							rslot.add(robotWheel);
							break outerr;
						}
					}
				}
				// did we find a row that had the matching slot?
				if(slotRow == -1) {
					ArrayList<DrivenWheelInterface> t = new ArrayList<DrivenWheelInterface>();
					t.add(robotWheel);
					wheels.add(t);
				}
			} 
			Optional<Integer> controllerAxisXO = Optional.ofNullable(AXIS[i].get("AxisX")).filter(String.class::isInstance)
					.map(e -> Integer.parseInt((String) e));
			Optional<Integer> controllerAxisYO = Optional.ofNullable(AXIS[i].get("AxisY")).filter(String.class::isInstance)
					.map(e -> Integer.parseInt((String) e));
			if(controllerAxisXO.isPresent()) {
				controllerAxisX = controllerAxisXO.get();
			} else {
				System.out.println("<<WARNING USING X AXIS DEFAULTS WheelLun: "+i+" AXIS[WheelLun]: ");
				controllerAxisX = 0;
			}
			if(controllerAxisYO.isPresent()) {
				controllerAxisY = controllerAxisYO.get();
			} else {
				System.out.println("<<WARNING USING Y AXIS DEFAULTS WheelLun: "+i+" AXIS[WheelLun]: ");
				controllerAxisY = 2;
			}
		}
	}

	private RobotWheel setDrive(String name, int wheelLun, int slot, int chan) {
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

			return new RobotWheel(name, slot, chan, 
					wheelDiameter.get(), 
					wheelTrack.get(), 
					minSpeed.get(), 
					maxSpeed.get(), 
					ticksPerRevolution.get(), 
					motorKp.get(), motorKd.get(), 
					motorKi.get(), motorKo.get(),
					motorPIDRate.get());
		} catch(NoSuchElementException nse) {
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
		ArrayList<DrivenWheelInterface> rslot = null;
		outerr: for(int sl = 0; sl < wheels.size(); sl++) {
			rslot = wheels.get(sl);
			for(int rsl = 0; rsl < rslot.size(); rsl++) {
				if(rslot.get(rsl).getSlot() == slot) {
					break outerr;
				}
			}
		}
		// did we find a row that had the matching slot?
		return rslot;
	}

	@Override
	/**
	 * Get the wheel for the given slot row and channel
	 * @param slot The array of channels for this slot from getWheels
	 * @param chan the target channel
	 * @return RobotWheel or null if slots is null or channel cant be found in row for slot.
	 */
	public DrivenWheelInterface getWheel(ArrayList<DrivenWheelInterface> slot, int chan) {
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
	public ArrayList<ArrayList<DrivenWheelInterface>> getWheels() {
		return wheels;
	}
	
	@Override
	public SlotHandler getSlotHandler(int slot) {
		return slotHandlers.get(slot);
	}
	
	public void createControllers(Robot robot) throws IOException {
		for(int i = 0; i < wheels.size(); i++) {
			ArrayList<DrivenWheelInterface> slotWheels = wheels.get(i);
			DrivenWheelInterface dwi = slotWheels.get(0);
			MarlinspikeControlInterface controller = robot.getManager().createController(Optional.empty(), (int)dwi.getSpeedsetPointInfo().getMaximum());
			slotHandlers.add(new SlotHandler(controller,dwi.getName(),i));
		}
	}
	
	@Override
	public String toString() {
		StringBuffer sb = new StringBuffer();
		for(int sl = 0; sl < wheels.size(); sl++) {
			ArrayList<DrivenWheelInterface> rslot = wheels.get(sl);
			for(int rsl = 0; rsl < rslot.size(); rsl++) {
				sb.append(rsl+1);
				sb.append(".) ");
				sb.append(rslot.get(rsl).toString());
				sb.append("ControllerAxisX=");
				sb.append(controllerAxisX);
				sb.append(" ControllerAxisY=");
				sb.append(controllerAxisY);
				sb.append("\r\n");
			}
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
