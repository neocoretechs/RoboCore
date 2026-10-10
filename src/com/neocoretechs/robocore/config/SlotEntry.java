package com.neocoretechs.robocore.config;

import com.neocoretechs.robocore.marlinspike.SlotHandler;
import java.io.Serializable;
import java.util.Objects;

/**
 * List of unique devices from RoboCore.properties loaded from parameter tree. 
 * Assembled into collection in {@link MarlinspikeManager}<p>
 * Contains the LUN, the Logical Unit Number an integer ordinal, and an integer value for Slot.<p>
 * The superclass contains the name entry in properties configuration file, such as "LeftWheel", the NodeName which is 
 * the node attached to the host computer name of the Ros node, such as "CONTROL1", the Controller which is
 * the physical device port the microcontroller for this entry is attached to, such as /dev/ttyACM0 using 
 * {@link com.neocoretechs.robocore.serialreader.ByteSerialDataPort}, or a class that
 * handles the same sort of input using a line reader such as {@link com.neocoretechs.robocore.serialreader.MarlinspikeDataPort}.
 * @see com.neocoretechs.robocore.serialreader.MarlinspikeDataPort
 * @see com.neocoretechs.robocore.serialreader.ByteSerialDataPort
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2022,2026
 *
 */
public class SlotEntry extends DeviceEntry implements Serializable {
	private static final long serialVersionUID = 1L;
	private int slot;
	private SlotHandler slotHandler;

	public SlotEntry() {}
	/**
	 * @param Name DeviceName entry in properties configuration file, such as "LeftWheel"
	 * @param NodeName the node attached to, typically the SSID name of the Ros node, such as "ROSCOE1"
	 * @param LUN integer LUN position, points to LUN array in Robot, such as 1
	 * @param controller alternate controller implementing MarlinspikeControlInterface
	 * @param slot the "Slot" property that corresponds to the Marlinspike slot in the M10 Z(slot) code
	 */
	public SlotEntry(RobotInterface robot, String deviceName, String NodeName, int LUN, String controller, int slot) {
		super(deviceName, NodeName, LUN, controller);
		this.slot = slot;
		this.slotHandler = new SlotHandler(robot, deviceName, slot);
	}

	public SlotHandler getSlotHandler() {
		return slotHandler;
	}
	
	@Override
	public boolean equals(Object obj) {
		boolean eq = super.equals(obj);
		if(!eq)
			return eq;
		int other = ((SlotEntry)obj).slot;
		return slot == other;
	}
	
	@Override
	public int hashCode() {
		return Objects.hash(getName(), getNodeName(), slot);
	}
	
	@Override
	public String toString() {
		return String.format("%s slot=%d%n", super.toString(), slot);
	}
}
