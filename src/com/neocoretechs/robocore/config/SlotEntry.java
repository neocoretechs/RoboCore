package com.neocoretechs.robocore.config;

import com.neocoretechs.robocore.marlinspike.MarlinspikeControlInterface;
import com.neocoretechs.robocore.marlinspike.SlotHandler;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

/**
 * Part of List of unique devices from RoboCore.properties loaded from parameter tree.<p>
 * A 'slot' can aggregate multiple devices. For instance, LeftWheel and RighWheel are collectively controlled by one slot
 * in the Marlinspike that takes multiple values to control propulsion. So SlotEntry extends DeviceEntry
 * and has a collection of SlotHandler instances.<p>
 * Assembled into collection in {@link com.neocoretechs.robocore.marlinspike.MarlinspikeManager}<p>
 * Contains the LUN, the Logical Unit Number an integer ordinal, and an integer value for Slot.<p>
 * These are a handled by derived class of specialized classes such as {@link com.neocoretechs.robocore.propulsion.RobotDiffDrive} subclasses.
 * Devices not part of slots used for PWM drivers are designated by {@link com.neocoretechs.robocore.config.DeviceEntry}.
 * The superclass contains the name entry in properties configuration file, such as "BoomActuator", the NodeName which is 
 * the node attached to the host computer name of the Ros node, such as "CONTROL1", the Controller which is
 * the physical device port the microcontroller for this entry is attached to, such as /dev/ttyACM0 using 
 * {@link com.neocoretechs.robocore.serialreader.ByteSerialDataPort}, or a class that
 * handles the same sort of input using a line reader such as {@link com.neocoretechs.robocore.serialreader.MarlinspikeDataPort}.
 * @see com.neocoretechs.robocore.serialreader.MarlinspikeDataPort
 * @see com.neocoretechs.robocore.serialreader.ByteSerialDataPort
 * @see com.neocoretechs.robocore.marlinspike.SlotHandler
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2022,2026
 *
 */
public class SlotEntry extends DeviceEntry implements Serializable {
	private static final long serialVersionUID = 1L;
	private ArrayList<SlotHandler> slotHandler = new ArrayList<SlotHandler>();

	public SlotEntry() {}
	/**
	 * @param NodeName the node attached to, typically the SSID name of the Ros node, such as "ROSCOE1"
	 * @param LUN integer LUN position, points to LUN array in Robot, such as 1
	 * @param controller alternate controller implementing MarlinspikeControlInterface
	 * @param slot the "Slot" property that corresponds to the Marlinspike slot in the M10 Z(slot) code
	 * @param Name DeviceName entry in properties configuration file, such as "BoomActuator"
	 */
	public SlotEntry(RobotInterface robot, String deviceName, String NodeName, int LUN, MarlinspikeControlInterface controller, int slot) {
		super(robot, deviceName, NodeName, LUN, controller);
		for(SlotHandler sh : slotHandler)
			if(sh.getSlot() != slot)
				throw new RuntimeException("Slots must be homogeneous in SlotEntry collection. Attempt to insert "+slot+" for collection based on "+sh.getSlot());
		slotHandler.add(new SlotHandler(controller, deviceName, slot));
		this.topicName = "Slot"+String.valueOf(slot);
	}

	public void setMarlinspikeControl(MarlinspikeControlInterface controller, String deviceName, int slot) {
		for(SlotHandler sh : slotHandler)
			if(sh.getSlot() != slot)
				throw new RuntimeException("Slots must be homogeneous in SlotEntry collection. Attempt to insert "+slot+" for collection based on "+sh.getSlot());
		slotHandler.add(new SlotHandler(controller, deviceName, slot));
		this.topicName = "Slot"+String.valueOf(slot);
	}
	
	public MarlinspikeControlInterface getMarlinspikeControl(int slot) {
		for(SlotHandler s: slotHandler)
			if(s.getSlot() == slot) 
				return s.getControl();
		return null;
	}
	
	public List<SlotHandler> getSlotHandlers() {
		return slotHandler;
	}
	
	@Override
	public void setOperating() {
		for(SlotHandler sh : slotHandler)
			robot.getOperating().replace(sh.getName(), true);
	}
	
	@Override
	public String toString() {
		return String.format("%s slots=%s%n", super.toString(), Arrays.toString(slotHandler.toArray()));
	}
}
