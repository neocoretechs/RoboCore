package com.neocoretechs.robocore.marlinspike;

import java.io.IOException;
import java.io.Serializable;
import java.util.Map;
import java.util.Set;

import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;
/**
 * deviceName is a descriptor for the device, such as "LeftWheel" etc. The device itself is typically a tty
 * port that the Marlinspike board is attached to.
 * @author Jonathan Groff (C) NeoCoreTechs 2021
 *
 */
public class NodeDeviceDemuxer implements Serializable {
	private static final long serialVersionUID = 1L;
	private static boolean DEBUG = false;
	private String nodeName;
	private String deviceName;
	private String device;
	transient AsynchDemuxer asynchDemuxer = null;
	private transient  MarlinspikeControlInterface controlHost;
	
	/**
	 * @param nodeName 
	 * @param deviceName descriptor for the device, such as "LeftWheel" etc.
	 * @param device the physical device the Marlinspike board is connected to, typically a tty via USB
	 */
	public NodeDeviceDemuxer(String nodeName, String deviceName, String device)  {
		this.nodeName = nodeName;
		this.deviceName = deviceName;
		this.device = device;
	}
	/**
	 * Active the asynchDemuxer for the given Marlinspike if it has not been previously
	 * activated. We must ensure that 1 demuxer/device is activated for a particular physical port
	 * and that subsequent attempts at activation are met with an assignment 
	 * to an existing instance of asynchDemuxer.
	 * @param marlinspikeManager 
	 * @param deviceToType The mapping of all NodeDeviceDemuxer to all the TypeSlotChannels
	 * @throws IOException If we attempt to re-use a port, we box up the runtime exception with the IOException
	 */
	public void activateMarlinspikes(MarlinspikeManager marlinspikeManager, Map<NodeDeviceDemuxer, Map<String, TypeSlotChannelEnable>> deviceToType) throws IOException {
		Set<NodeDeviceDemuxer> sndd = deviceToType.keySet();
		for(NodeDeviceDemuxer ndd : sndd) {
			if(ndd.device.equals(device)) {
				if( ndd.asynchDemuxer == null ) {
					if(DEBUG)
						System.out.printf("%s.activateMarlinspikes preparing to initialize %n",this.getClass().getName());
					asynchDemuxer = new AsynchDemuxer(marlinspikeManager);
					asynchDemuxer.connect(new ByteSerialDataPort(device));
					asynchDemuxer.init();
					controlHost = new MarlinspikeControl(asynchDemuxer);
					return;
				}
				if(ndd == this)
					return;
				if(DEBUG)
					System.out.printf("%s.activateMarlinspikes preparing to copy %s %n", this.getClass().getName(), ndd);
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
	
	public String getNodeName() {
		return nodeName;
	}
	/**
	 * @return Name descriptor for the device, such as "LeftWheel" etc.
	 */
	public String getDeviceName() {
		return deviceName;
	}
	/**
	 * @return The physical device descriptor, such as /dev/ttyACM0.
	 */
	public String getDevice() {
		return device;
	}
	
	@Override
	public String toString() {
		return String.format("%s %s %s %s (key)%n",this.getClass().getName(), nodeName, deviceName, device);
	}
}
