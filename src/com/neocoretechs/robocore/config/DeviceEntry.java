package com.neocoretechs.robocore.config;

import com.neocoretechs.robocore.marlinspike.NodeDeviceDemuxer;

/**
 * List of unique devices from RoboCore.properties. Assembled into collection in {@link MarlinspikeManager}
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2022
 *
 */
public class DeviceEntry {
	private String Name; // name entry in properties configuration file, such as "LeftWheel"
	private String NodeName; // the node attached to, the host computer name of the Ros node, such as "CONTROL1"
	private int LUN; // integer LUN position, points to LUN array in Robot, such as 1
	private String Controller; // the physical device port microcontroller for this entry is attached to, such as /dev/ttyACM0
	private NodeDeviceDemuxer ndd; // Demuxer for above properties, carries some redundant info, will appear many in this collection, added after constructor
	/**
	 * 
	 * @param Name name entry in properties configuration file, such as "LeftWheel"
	 * @param NodeName the node attached to, the host computer name of the Ros node, such as "CONTROL1"
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
	/**
	 * @return the controller
	 */
	public String getController() {
		return Controller;
	}
	/**
	 * @param controller the controller to set
	 */
	public void setController(String controller) {
		this.Controller = controller;
	}
	
	/**
	 * @return the ndd
	 */
	public NodeDeviceDemuxer getNodeDeviceDemuxer() {
		return ndd;
	}
	
	/**
	 * @param ndd the ndd to set
	 */
	public void setNodeDeviceDemuxer(NodeDeviceDemuxer ndd) {
		this.ndd = ndd;
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
