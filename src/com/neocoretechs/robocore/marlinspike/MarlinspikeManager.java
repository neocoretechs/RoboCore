package com.neocoretechs.robocore.marlinspike;

import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.neocoretechs.robocore.config.Robot;
import com.neocoretechs.robocore.config.RobotInterface;
import com.neocoretechs.robocore.config.TypedWrapper;

/**
 * Parse configs and allocate the necessary number of control elements for one or more Marlinspike boards
 * @author groff
 *
 */
public class MarlinspikeManager {
	int leftSlot = -1;
	int leftChannel = -1;
	int rightSlot = -1;
	int rightChannel = -1;
	TypedWrapper[] lun;
	TypedWrapper[] wheel;
	Object[] nodeNames; // one of these for each subscriber to serve AsynchDemuxer and DataPortInterface
	Object[] controllers; // one of these for each AsynchDemuxer and DataPort
	/**
	 * 
	 * @param lun
	 * @param wheel
	 */
	public MarlinspikeManager(TypedWrapper[] lun, TypedWrapper[] wheel) {
		this.lun = lun;
		this.wheel = wheel;
		nodeNames = aggregate(lun, "NodeName");
		controllers = aggregate(lun,"Controller");
	}
	/**
	 * NodeNames are the subscription topics for the ROS bus to send commands to the Marlinspikes
	 * attached to specific controllers. 
	 * @return The array of Strings representing unique NodeNames from configs file in no particular order.
	 */
	public Object[] getnodeNames() {
		return nodeNames;
	}
	/**
	 * Controllers are the device names with attached Marlinspikes. In particular
	 * the tty ports that will have DataPorts assigned to read/write the serial data from Marlinspike.
	 * There will be one for each AsynchDemuxer.
	 * @return The unique array of Strings of ports in no particular order.
	 */
	public Object getControllers() {
		return controllers;
	}
	
	public int getLeftWheelChannel() {
		return leftChannel;
	}

	public int getRightWheelChannel() {
		return rightChannel;
	}

	public int getControllerLeftSlot() {
		return leftSlot;
	}

	public int getControllerRightSlot() {
		return rightSlot;
	}
	
	private Object[] aggregate(TypedWrapper[] wrapper, String prop) {
		return Stream.of(wrapper).flatMap(map -> map.entrySet().stream()).filter(map->map.getKey().equals(prop)).distinct()
				.toArray();
				//.collect(Collectors.toMap(p -> p.getKey(), p -> p.getValue())).values().toArray();
	    		//.collect(Collectors.toList());
	}
	
	public String toSttring() {
		return String.format("Controller LeftSlot=%d, Left Channel=%d, RightSlot=%d Right Channel=%d\r\n",
				leftSlot, leftChannel, rightSlot, rightChannel);
	}
	public static void main(String[] args) {
		RobotInterface robot = new Robot();
		MarlinspikeManager mm = new MarlinspikeManager(robot.getLUN(), robot.getWHEEL());
		for(Object n : mm.aggregate(mm.lun, "NodeName"))
			System.out.println(n);
		System.out.println("----");
		for(Object n : mm.aggregate(mm.lun,"Controller"))
			System.out.println(n);
		System.out.println("----");
		for(int i = 0; i < mm.lun.length; i++) {
			System.out.println(i+".)"+mm.lun[i].get("Name")+","+mm.lun[i].get("NodeName")+","+mm.lun[i].get("Controller")+","+
		mm.lun[i].get("Type")+","+mm.lun[i].get("Slot")+","+mm.lun[i].get("PWMPin0")+","+mm.lun[i].get("Channel")+","+
					mm.lun[i].get("EnablePin")+","+mm.lun[i].get("Direction")+","+mm.lun[i].get("EncoderPin"));
		}
	}
}
