package com.neocoretechs.robocore.marlinspike;

import java.io.IOException;
import java.io.Serializable;
import java.util.Arrays;
import java.util.NoSuchElementException;

import com.neocoretechs.robocore.config.Robot;
/**
 * This class defines the internal slot handler to control a drive via a single slot designation
 * Whereas we had subscribers for each named LUN device when another node processed movement commands
 * we collapsed the pipeline and made this internal class handle that traffic in one combined slot designation
 */
public final class SlotHandler implements Serializable {
	private static final long serialVersionUID = 1L;
	private static boolean DEBUG = false;
	MarlinspikeControlInterface control = null;
	String deviceName;
	int slot;
	
	public SlotHandler() {}
	public SlotHandler(MarlinspikeControlInterface control, String deviceName, int slot) throws NoSuchElementException {
		this.deviceName = deviceName;
		this.slot = slot;
		this.control = control;
		if(DEBUG)
			System.out.printf("%s got DeviceName:%s Slot:%d Control %s from MarlinspikeManager%n", this.getClass().getName(),deviceName, slot, control);
	}
	
	public int getSlot() {
		return slot;
	}
	
}
