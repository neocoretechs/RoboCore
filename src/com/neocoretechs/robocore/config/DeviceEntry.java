package com.neocoretechs.robocore.config;

import java.io.Serializable;

import com.neocoretechs.robocore.marlinspike.MarlinspikeControlInterface;

/**
 * List of unique devices from RoboCore.properties loaded into parameter tree. Assembled into collection in {@link MarlinspikeManager}<p>
 * Contains the LUN, the name entry in properties configuration file, such as "LeftWheel", the NodeName which is 
 * the node attached to the host computer name of the Ros node, such as "CONTROL1", the Controller which is
 * the physical device port the microcontroller for this entry is attached to, such as /dev/ttyACM0 using 
 * {@link com.neocoretechs.robocore.serialreader.ByteSerialDataPort}, or a class that
 * handles the same sort of input using a line reader such as {@link com.neocoretechs.robocore.serialreader.MarlinspikeDataPort}.
 * @see com.neocoretechs.robocore.serialreader.MarlinspikeDataPort
 * @see com.neocoretechs.robocore.serialreader.ByteSerialDataPort
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2022,2026
 *
 */
public class DeviceEntry implements Serializable{
	private static final long serialVersionUID = 1L;
	private String Name; // name entry in properties configuration file, such as "LeftWheel"
	private String NodeName; // the node attached to, the host computer name of the Ros node, such as "CONTROL1"
	private int LUN; // integer LUN position, points to LUN array in Robot, such as 1
	private String Controller; // the physical device port microcontroller for this entry is attached to, such as /dev/ttyACM0
	private transient MarlinspikeControlInterface controlHost;
	/**
	 * 
	 * @param Name name entry in properties configuration file, such as "LeftWheel"
	 * @param NodeName the node attached to, typically the SSID name of the Ros node, such as "ROSCOE1"
	 * @param LUN integer LUN position, points to LUN array in Robot, such as 1
	 * @param Controller the physical device port microcontroller for this entry is attached to, such as /dev/ttyACM0
	 */
	public DeviceEntry(String Name, String NodeName, int LUN, String Controller) {
		this.Name = Name;
		this.NodeName = NodeName;
		this.LUN = LUN;
		this.Controller = Controller;
	}
	/**
	 * Form a template for locating in collection via name
	 * @param name
	 */
	public DeviceEntry(String name) {
		this.Name = name;
	}
	/**
	 * @return the name
	 */
	public String getName() {
		return Name;
	}
	/**
	 * @param name the name to set
	 */
	public void setName(String name) {
		Name = name;
	}
	/**
	 * @return the nodeName
	 */
	public String getNodeName() {
		return NodeName;
	}
	/**
	 * @param nodeName the nodeName to set
	 */
	public void setNodeName(String nodeName) {
		NodeName = nodeName;
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
	
	public void setMarlinspikeControl(MarlinspikeControlInterface controlHost) {
		this.controlHost = controlHost;
	}
	
	public MarlinspikeControlInterface getMarlinspikeControl() {
		return controlHost;
	}

	@Override
	public boolean equals(Object o) {
		return Name.equals(((DeviceEntry)o).getName());
	}
	@Override
	public int hashCode() {
		return Name.hashCode();
	}
	@Override
	public String toString() {
		return String.format("%s %s %s %d %s%n", this.getClass().getName(), Name, NodeName, LUN, Controller);
	}
}
