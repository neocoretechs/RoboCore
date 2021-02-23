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
	TypedWrapper[] lun;
	TypedWrapper[] wheel;
	/**
	 * 
	 * @param lun
	 * @param wheel
	 */
	public MarlinspikeManager(TypedWrapper[] lun, TypedWrapper[] wheel) {
		this.lun = lun;
		this.wheel = wheel;
	}
	
	private Object[] aggregate(TypedWrapper[] wrapper, String prop) {
		return Stream.of(wrapper).flatMap(map -> map.entrySet().stream()).filter(map->map.getKey().equals(prop)).distinct()
				.toArray();
				//.collect(Collectors.toMap(p -> p.getKey(), p -> p.getValue())).values().toArray();
	    		//.collect(Collectors.toList());
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
			System.out.println(i+".)"+mm.lun[i].get("Name")+","+mm.lun[i].get("NodeName")+","+mm.lun[i].get("Controller"));
		}
	}
}
