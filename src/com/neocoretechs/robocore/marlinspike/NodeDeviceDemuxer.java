package com.neocoretechs.robocore.marlinspike;

import java.io.IOException;
import java.util.Map;

import com.neocoretechs.robocore.config.TypedWrapper;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;
/**
 * deviceName is a descriptor for the device, such as "LeftWheel" etc. The device itself is typically a tty
 * port that the Marlinspike board is attached to.
 * @author Jonathan Groff (C) NeoCoreTechs 2021
 *
 */
public class NodeDeviceDemuxer {
	private String deviceName;
	private String device;
	TypedWrapper lun;
	TypedWrapper wheel;
	TypedWrapper pid;
	AsynchDemuxer asynchDemuxer;
	private MarlinspikeControlInterface controlHost;
	
	/**
	 * @param deviceName descriptor for the device, such as "LeftWheel" etc.
	 * @param device the physical device the Marlinspike board is connected to, typically a tty via USB
	 */
	public NodeDeviceDemuxer(String deviceName, String device)  {
		this.deviceName = deviceName;
		this.device = device;
		asynchDemuxer = new AsynchDemuxer();
	}
	
	public void activateMarlinspikes(Map<String, TypeSlotChannelEnable> value) throws IOException {
		asynchDemuxer.connect(new ByteSerialDataPort(device));
		asynchDemuxer.init(value);
		controlHost = new MarlinspikeControl(asynchDemuxer);
	}
	
	public MarlinspikeControlInterface getMarlinspikeControl() {
		return controlHost;
	}
	
	public AsynchDemuxer getAsynchDemuxer() {
		return asynchDemuxer;
	}
	
	@Override
	public boolean equals(Object o) {
		return ((NodeDeviceDemuxer)o).device.equals(device);
	}
	/**
	 * @return the name of the device
	 */
	public String getDeviceName() {
		return deviceName;
	}
	/**
	 * @return The physical device descriptor
	 */
	public String getDevice() {
		return device;
	}
}
