package com.neocoretechs.robocore.marlinspike;

import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;
/**
 * deviceName is a descriptor for the device, such as "LeftWheel" etc. The device itself is typically a tty
 * port that the Marlinspike board is attached to. These demuxers are unique to each device and as such the
 * hash and equals methods are keyed to device field.
 * @author Jonathan Groff (C) NeoCoreTechs 2021
 *
 */
public class NodeDeviceDemuxer implements Serializable {
	private static final long serialVersionUID = 1L;
	private static boolean DEBUG = false;
	private String nodeName;
	private String deviceName;
	private String device;
	private ArrayList<String> startup = new ArrayList<String>();
	transient AsynchDemuxer asynchDemuxer = null;
	private transient MarlinspikeControlInterface controlHost;
	
	/**
	 * @param nodeName The host name of the computer the device is connected to
	 * @param deviceName descriptor for the device, such as "LeftWheel" etc.
	 * @param device the physical device the Marlinspike board is connected to, typically a tty via USB, the Controller
	 */
	public NodeDeviceDemuxer(String nodeName, String deviceName, String device)  {
		this.nodeName = nodeName;
		this.deviceName = deviceName;
		this.device = device;
	}

	protected void setMarlinspikeControl(MarlinspikeControlInterface controlHost) {
		this.controlHost = controlHost;
	}
	
	public MarlinspikeControlInterface getMarlinspikeControl() {
		return controlHost;
	}
	/**
	 * Get the {@link AsynchDemuxer} that services this NodeDeviceDemuxer
	 */
	public AsynchDemuxer getAsynchDemuxer() {
		return asynchDemuxer;
	}
	/**
	 * Set the {@link AsynchDemuxer} that services this NodeDeviceDemuxer
	 * @param asynchDemuxer
	 */
	protected void setAsynchDemuxer(AsynchDemuxer asynchDemuxer) {
		this.asynchDemuxer = asynchDemuxer;
	}
	/**
	 * The node name of the computer we are operating upon.
	 * @return
	 */
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
	 * @return The physical device descriptor, such as /dev/ttyACM0 or MarlinspikeDataPort.
	 */
	public String getDevice() {
		return device;
	}
	/**
	 * Add the parameter to the startup collection, if the entry already exists, reject it such that
	 * all entries are unique. There is currently no known use case where a duplicate startup directive needs issued.
	 * @param m10Gen
	 */
	public void addInit(List<String> m10Gen) {
		for(String mElem : m10Gen) {
			if(!startup.contains(mElem))
				startup.add(mElem);
		}
	}
	
	public void init() throws IOException {
		if(startup.size() > 0)
			asynchDemuxer.config(startup);
	}
	
	@Override
	public boolean equals(Object o) {
		return device.equals(((NodeDeviceDemuxer)o).getDevice());
	}
	
	@Override
	public int hashCode() {
		return device.hashCode();
	}
	
	@Override
	public String toString() {
		return String.format("%s %s %s %s (key)%n",this.getClass().getName(), nodeName, deviceName, device);
	}
}
