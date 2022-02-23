package com.neocoretechs.robocore.marlinspike;

import java.io.IOException;

/**
 * This interface defines the object model to access the Marlinspike hardware controller.<p/>
 * It defines the contract between the callable methods and the M and G codes sent to the Marlispike controllers.<p/>
 * A logical unit consists of an AsynchDemuxer, a MarlinspikeControl, and a DataPortInterface<p/>
 * These objects are aggregated in the {@link NodeDeviceDemuxer}, which lives in a collection in the {@link MarlinspikeManager}
 * which instantiates and control the lifecycle for these objects.<p/>
 * Multiple controllers occupy virtual 'slots' that correspond to hardware such as bridges.
 * Each controller 'slot' can accommodate several motor control channels.
 * Each channel receives a value representing a power level.
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2017,2018,2020,2021
 *
 */
public interface MarlinspikeControlInterface {
	public String reportAllControllerStatus() throws IOException;
	public String reportSystemId() throws IOException;
	public void commandStop() throws IOException;
	public String commandReset() throws IOException;
	public void setDeviceLevel(String deviceName, int deviceLevel) throws IOException;
	public void commandPWM(String string);
}
