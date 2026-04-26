package com.neocoretechs.robocore.config;

import java.io.Serializable;
import java.util.Objects;

import com.neocoretechs.robocore.marlinspike.MarlinspikeControlInterface;

/**
 * List of unique Slots from RoboCore.properties loaded from parameter tree. 
 * Assembled into collection in {@link MarlinspikeManager}<p>
 * Contains the Slot name entry in properties configuration file, such as "0", the NodeName which is 
 * the node attached to the host computer name of the Ros node, such as "CONTROL1", the Controller which is
 * the physical device port the microcontroller for this entry is attached to, such as /dev/ttyACM0 using 
 * {@link com.neocoretechs.robocore.serialreader.ByteSerialDataPort}, or a class that
 * handles the same sort of input using a line reader such as {@link com.neocoretechs.robocore.serialreader.MarlinspikeDataPort}.
 * @see com.neocoretechs.robocore.serialreader.MarlinspikeDataPort
 * @see com.neocoretechs.robocore.serialreader.ByteSerialDataPort
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2022,2026
 *
 */
public class SlotEntry implements Serializable{
	private static final long serialVersionUID = 1L;
	private String Name; // name entry in properties configuration file, such as "LeftWheel"
	private String NodeName; // the node attached to, the host computer name of the Ros node, such as "CONTROL1"
	private transient MarlinspikeControlInterface controlHost;
	private String controlClass;
	
	public SlotEntry() {}
	/**
	 * 
	 * @param Name Slot name entry in properties configuration file, such as "0"
	 * @param NodeName the node attached to, typically the SSID name of the Ros node, such as "ROSCOE1"
	 * @param controller alternate controller implementing MarlinspikeControlInterface
	 */
	public SlotEntry(String Name, String NodeName, String controller) {
		this.Name = Name;
		this.NodeName = NodeName;
		this.controlClass = controller;
	}
	/**

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

	public void setMarlinspikeControl(MarlinspikeControlInterface controlHost) {
		this.controlHost = controlHost;
	}
	
	public MarlinspikeControlInterface getMarlinspikeControl() {
		return controlHost;
	}

	public String getControlClass() {
		return controlClass;
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
		return Objects.equals(Name, other.Name) && Objects.equals(NodeName, other.NodeName);
	}
	@Override
	public int hashCode() {
		return Objects.hash(Name, NodeName);
	}
	@Override
	public String toString() {
		return String.format("%s %s Node=%s Control=%s%n", this.getClass().getName(), Name, NodeName, getControlClass());
	}

}
