package com.neocoretechs.robocore.config;
/**
 * List of unique devices from RoboCore.properties. Assembled into collection in {@link MarlinspikeManager}
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2022
 *
 */
public class DeviceEntry {
	private String Name; // name entry in properties configuration file
	private String NodeName; // the node attached to
	private int LUN; // integer LUN position, points to LUN array in Robot
	private String Controller; // the physical device port microcontroller for this entry is attached to
	/**
	 * 
	 * @param Name
	 * @param NodeName
	 * @param LUN
	 * @param Controller
	 */
	public DeviceEntry(String Name, String NodeName, int LUN, String Controller) {
		this.Name = Name;
		this.NodeName = NodeName;
		this.LUN = LUN;
		this.Controller = Controller;
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
	
	@Override
	public boolean equals(Object o) {
		return Name.equals(((DeviceEntry)o).getName());
	}
	@Override
	public int hashCode() {
		return Name.hashCode();
	}
}