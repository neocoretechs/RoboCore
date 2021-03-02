package com.neocoretechs.robocore.marlinspike;

import java.io.IOException;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import com.neocoretechs.robocore.config.TypedWrapper;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;
/**
 * deviceName is a descriptor for the device, such as "LeftWheel" etc. The device itself is typically a tty
 * port that the Marlinspike board is attached to.
 * @author Jonathan Groff (C) NeoCoreTechs 2021
 *
 */
public class NodeDeviceDemuxer {
	private static boolean DEBUG = true;
	private String deviceName;
	private String device;
	AsynchDemuxer asynchDemuxer = null;
	private MarlinspikeControlInterface controlHost;
	
	/**
	 * @param deviceName descriptor for the device, such as "LeftWheel" etc.
	 * @param device the physical device the Marlinspike board is connected to, typically a tty via USB
	 */
	public NodeDeviceDemuxer(String deviceName, String device)  {
		this.deviceName = deviceName;
		this.device = device;
	}
	/**
	 * Active the asynchDemuxer for the given Marlinspike if it has not been previously
	 * activated. We must ensure that 1 demuxer/device is activated for a particular physical port
	 * and that subsequent attempts at activation are met with an assignment 
	 * to an existing instance of asynchDemuxer.
	 * @param deviceToType The mapping of all NodeDeviceDemuxer to all the TypeSlotChannels
	 * @param value the particular TypeSlot Channel that we are trying to enable
	 * @throws IOException If we attempt to re-use a port, we box up the runtime exception with the IOException
	 */
	public void activateMarlinspikes(Map<NodeDeviceDemuxer, Map<String, TypeSlotChannelEnable>> deviceToType,
						Map<String, TypeSlotChannelEnable> value) throws IOException {
		Set<NodeDeviceDemuxer> sndd = deviceToType.keySet();
		for(NodeDeviceDemuxer ndd : sndd) {
			if(ndd != this && ndd.device.equals(device)) {
				if( ndd.asynchDemuxer == null ) {
					if(DEBUG)
						System.out.printf("%s.activateMarlinspikes preparing to initialize %s%n",this.getClass().getName(), value);
					asynchDemuxer = new AsynchDemuxer();
					asynchDemuxer.connect(new ByteSerialDataPort(device));
					asynchDemuxer.init(value);
					controlHost = new MarlinspikeControl(asynchDemuxer);
					return;
				}
				if(DEBUG)
					System.out.printf("%s.activateMarlinspikes preparing to copy %s %s%n", this.getClass().getName(), ndd, value);
				asynchDemuxer = ndd.asynchDemuxer;
				controlHost = ndd.controlHost;
				return;
			}
		}
		throw new IOException("Could not locate device:"+device+" in deviceToType map");
	
	}
	
	public MarlinspikeControlInterface getMarlinspikeControl() {
		return controlHost;
	}
	
	public AsynchDemuxer getAsynchDemuxer() {
		return asynchDemuxer;
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
	
	@Override
	public String toString() {
		return String.format("%s %s %s (key)%n",this.getClass().getName(), deviceName, device);
	}
}
