package com.neocoretechs.robocore.marlinspike;

import java.io.IOException;
import java.io.Serializable;
import java.util.Arrays;
import java.util.NoSuchElementException;

import com.neocoretechs.robocore.config.RobotInterface;

/**
 * This class defines the internal slot handler to control a drive via a single slot designation
 * Whereas we had subscribers for each named LUN device when another node processed movement commands
 * we collapsed the pipeline and made this internal class handle that traffic in one combined slot designation
 */
public final class SlotHandler implements Serializable{
	private static final long serialVersionUID = 1L;
	private static boolean DEBUG = true;
	MarlinspikeControlInterface control = null;
	String deviceName;
	int slot;
	private RobotInterface robot;
	
	public SlotHandler() {}
	public SlotHandler(RobotInterface robot, String deviceName, int slot) throws NoSuchElementException {
		this.robot = robot;
		this.deviceName = deviceName;
		this.slot = slot;
		this.control = robot.getManager().getMarlinspikeControl(deviceName);
		if(DEBUG)
			System.out.printf("%s got DeviceName:%s Slot:%d Control %s from MarlinspikeManager%n", this.getClass().getName(),deviceName, slot, control);
	}
	/**
	 * Set the speed for the 
	 * @param valch
	 * @throws IOException
	 */
	public void setSpeed(int[] valch) throws IOException{
		if(DEBUG)
			System.out.printf("%s DeviceName=%s args:%s Thread:%s%n", this.getClass().getName(), deviceName, Arrays.toString(valch), Thread.currentThread().getName());
		// keep Marlinspike from getting bombed with zeroes
		boolean affectorSpeed = false;
		for(int val: valch) {
			if(val != 0) {
				affectorSpeed = true;
				break;
			}
		}
		robot.getOperating().put(deviceName, affectorSpeed);
		if(DEBUG)
			System.out.printf("%s affector:%b DeviceName=%s speeds:%s operating:%b%n", this.getClass().getName(), affectorSpeed, 
					deviceName, Arrays.toString(valch), robot.getOperating().get(deviceName));
		switch(valch.length) {
		case 1:
			control.setDeviceLevels(deviceName, valch[0]);
			break;
		case 2:
			control.setDeviceLevels(deviceName, valch[0],valch[1]);
			break;
		case 3:
			control.setDeviceLevels(deviceName, valch[0],valch[1],valch[2]);
			break;
		case 4:
			control.setDeviceLevels(deviceName, valch[0],valch[1],valch[2],valch[3]);
			break;
		case 5:
			control.setDeviceLevels(deviceName, valch[0],valch[1],valch[2],valch[3],valch[4]);
			break;
		case 6:
			control.setDeviceLevels(deviceName, valch[0],valch[1],valch[2],valch[3],valch[4],valch[5]);
			break;
		case 7:
			control.setDeviceLevels(deviceName, valch[0],valch[1],valch[2],valch[3],valch[4],valch[5],valch[6]);
			break;
		case 8:
			control.setDeviceLevels(deviceName, valch[0],valch[1],valch[2],valch[3],valch[4],valch[5],valch[6],valch[7]);
			break;
		case 9:
			control.setDeviceLevels(deviceName, valch[0],valch[1],valch[2],valch[3],valch[4],valch[5],valch[6],valch[7],valch[8]);
			break;
		case 10:
			control.setDeviceLevels(deviceName, valch[0],valch[1],valch[2],valch[3],valch[4],valch[5],valch[6],valch[7],valch[8],valch[9]);
			break;
		default:
			System.out.println("Bad param length to control:"+valch.length);
			return;	
		}
		if(DEBUG)
			System.out.printf("NewMessage, thread %s received Affector directives DeviceName:%s%n",Thread.currentThread().getName(),deviceName);
	}
}
