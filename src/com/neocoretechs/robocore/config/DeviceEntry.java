package com.neocoretechs.robocore.config;

import java.io.Serializable;
import java.util.Objects;

import com.neocoretechs.robocore.marlinspike.MarlinspikeControlInterface;

/**
 * List of unique Device Entry details from RoboCore.properties loaded from parameter tree. 
 * Assembled into collection in {@link MarlinspikeManager}<p>
 * Generic device entries are for effector that dont use slots, and are not part of the drive.
 * These are a handled by derived class of specialized classes such as {@link com.neocoretechs.robocore.propulsion.RobotDiffDrive} subclasses.
 * Slotted devices not part of drive are designated 
 file, such as "LEDPin", the NodeName which is 
 * the node attached to the host computer name of the Ros node, such as "CONTROL1", the Controller which is
 * the physical device port the microcontroller for this entry is attached to, such as /dev/ttyACM0 using 
 * {@link com.neocoretechs.robocore.serialreader.ByteSerialDataPort}, or a class that
 * handles the same sort of input using a line reader such as {@link com.neocoretechs.robocore.serialreader.MarlinspikeDataPort}.
 * @see com.neocoretechs.robocore.serialreader.MarlinspikeDataPort
 * @see com.neocoretechs.robocore.serialreader.ByteSerialDataPort
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2022,2026
 *
 */
public class DeviceEntry implements Serializable {
	private static final long serialVersionUID = 1L;
	protected RobotInterface robot;
	protected String Name; // name entry in properties configuration file, such as "BoomActuator"
	private String NodeName; // the node attached to, the host computer name of the Ros node, such as "CONTROL1"
	protected String topicName; // pub/sub bus topic name
	private int LUN; // integer LUN position, points to LUN array in Robot, such as 1
	private transient MarlinspikeControlInterface controlHost;
	private AxisInterface<?> manualControl;
	
	public DeviceEntry() {}
	/**
	 * topicName pub/sub bus name, for basic device same as deviceName, for slotted device, slot name Slot+slot number
	 * @param robot 
	 * @param Name Slot name entry in properties configuration file, such as "0"
	 * @param NodeName the node attached to, typically the SSID name of the Ros node, such as "ROSCOE1"
	 * @param controller alternate controller implementing MarlinspikeControlInterface
	 */
	public DeviceEntry(RobotInterface robot, String Name, String NodeName, int LUN, MarlinspikeControlInterface controller) {
		this.robot = robot;
		this.Name = Name;
		this.NodeName = NodeName;
		this.LUN = LUN;
		this.controlHost = controller;
		this.topicName = Name;
	}
	/**
	 * @return the lUN
	 */
	public int getLUN() {
		return LUN;
	}
	/**
	 * @return the name
	 */
	public String getName() {
		return Name;
	}
	
	public String getTopicName() {
		return topicName;
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
	
	public AxisInterface<?> getManualControl() {
		return manualControl;
	}
	public void setManualcontrol(AxisInterface<?> manualControl) {
		this.manualControl = manualControl;
	}

	public void setOperating() {
		robot.getOperating().replace(getName(), true);
	}
	
	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		DeviceEntry other = (DeviceEntry) obj;
		return Objects.equals(topicName, other.topicName) && Objects.equals(NodeName, other.NodeName);
	}
	@Override
	public int hashCode() {
		return Objects.hash(topicName, NodeName);
	}
	@Override
	public String toString() {
		return String.format("%s %s %s Node=%s Topic=%s LUN=%d Control=%s%n", this.getClass().getName(), robot, Name, NodeName, topicName, LUN, controlHost);
	}

}
