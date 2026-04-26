package com.neocoretechs.robocore.marlinspike;

import java.io.IOException;

/**
 * This interface defines the object model to access the Marlinspike hardware controller.<p>
 * It defines the contract between the callable methods and the M and G codes sent to the Marlispike controllers.<p>
 * A logical unit consists of an AsynchDemuxer, a MarlinspikeControl, and a DataPortInterface<p>
 * These objects are aggregated in the {@link NodeDevice}, which lives in a collection in the {@link MarlinspikeManager}
 * which instantiates and control the lifecycle for these objects.<p>
 * Multiple controllers occupy virtual 'slots' that correspond to hardware such as bridges.
 * Each controller 'slot' can accommodate several motor control channels.
 * Each channel receives a value representing a power level. setDeviceLevels calls the {@link TypeSlotChannelEnable}
 * configuration for the given deviceName and applies the proper arguments to the G-code that controls that device.
 * It then writes the directive to the {@link AsynchDemuxer}.
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2017,2018,2020,2021
 *
 */
public interface MarlinspikeControlInterface {
	public String reportAllControllerStatus() throws IOException;
	public String reportSystemId() throws IOException;
	public void commandStop() throws IOException;
	public int commandReset() throws IOException;
	public void setDeviceLevels(String deviceName, int... deviceLevel) throws IOException;
}
