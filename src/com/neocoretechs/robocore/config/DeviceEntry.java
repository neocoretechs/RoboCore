package com.neocoretechs.robocore.config;

import java.io.Serializable;
import java.util.Objects;

/**
 * List of unique devices from RoboCore.properties loaded from parameter tree. 
 * Assembled into collection in {@link MarlinspikeManager}<p>
 * Contains the LUN, the Logical Unit Number an integer ordinal.
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
public class DeviceEntry extends SlotEntry implements Serializable{
	private static final long serialVersionUID = 1L;
	private int LUN; // integer LUN position, points to LUN array in Robot, such as 1

	public DeviceEntry() {}
	/**
	 * @param Name name entry in properties configuration file, such as "LeftWheel"
	 * @param NodeName the node attached to, typically the SSID name of the Ros node, such as "ROSCOE1"
	 * @param LUN integer LUN position, points to LUN array in Robot, such as 1
	 * @param controller alternate controller implementing MarlinspikeControlInterface
	 */
	public DeviceEntry(String Name, String NodeName, int LUN, String controller) {
		super(Name, NodeName, controller);
		this.LUN = LUN;
	}

	/**
	 * @return the lUN
	 */
	public int getLUN() {
		return LUN;
	}
	
	/**
	 * @param lUN the lUN to set
	 */
	public void setLUN(int lUN) {
		LUN = lUN;
	}
	
	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		SlotEntry other = (SlotEntry) obj;
		return Objects.equals(getName(), other.getName()) && Objects.equals(getNodeName(), other.getNodeName());
	}
	
	@Override
	public int hashCode() {
		return Objects.hash(getName(), getNodeName());
	}
	
	@Override
	public String toString() {
		return String.format("%s %s Node=%s LUN=%d Control=%s%n", this.getClass().getName(), getName(), getNodeName(), LUN, getMarlinspikeControl());
	}
}
