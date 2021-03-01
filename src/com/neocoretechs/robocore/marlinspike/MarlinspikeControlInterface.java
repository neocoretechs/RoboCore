package com.neocoretechs.robocore.marlinspike;

import java.io.IOException;

import com.neocoretechs.robocore.config.TypedWrapper;

/**
 * This interface defines the object model to access the Marlinspike hardware controller.<p/>
 * It defines the contract between the callable methods and the M and G codes sent to the Marlispike controllers.<p/>
 * A logical unit consists of an AsynchDemuxer, a MarlinspikeControl, and a DataPortInterface<p/> 
 * Multiple motor controllers occupy virtual 'slots' that correspond to hardware such as bridges.
 * Each controller 'slot' can accommodate several motor control channels.
 * Each channel receives a value representing a power level.
 * We address the left and right motor in a presumed multichannel controller at the same time to reduce latency.
 * Propulsion motors are addressed via controller slot, controller slot channel, and value.
 * An abstraction of motor control interfaces.
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2017,2018,2020
 *
 */
public interface MarlinspikeControlInterface {
	public String reportAllControllerStatus() throws IOException;
	public String reportSystemId() throws IOException;
	public void commandStop() throws IOException;
	public String commandReset() throws IOException;
	public void setAbsoluteDiffDriveSpeed(int leftWheelSpeed, int rightWheelSpeed) throws IOException;
	public void setAbsolutePWMLevel(String tsl, int pwmLevel)throws IOException;
	public void setAffectorDriveSpeed(String tsl, int affectorSpeed) throws IOException;
	public void commandPWM(String string);
}
