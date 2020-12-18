package com.neocoretechs.robocore.machine.bridge;

import java.io.Serializable;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

/**
 * This class represents a machine reading taken from the Marlinspike code via the
 * attached single board computer. The format of this reading is a line delimited by a numeric.
 * This class supports generating a series of requests to query the Marlinspike realtime subsystem and retrieve status
 * reports via various M codes issued to the realtime processing loop.<p>
 * We issue the commands directly through the serial USB or other port and wait for status messages to be
 * returned from the Marlinspike.<p/>
 * These reports are delineated by XML type headers and have various levels of structure, so a handler
 * is used in a separate thread for each 'topic' that corresponds to a header from the status payload from the Marlinspike.<p/>
 * These processing threads get the asynchronously returned data by demultiplexing them from a circular blocking queue
 * populated by the main run method of (@code AsynchDemuxer} which reads from the Marlinspike using the third party
 * open source RxTx library and its own read and write threads.<p/>
 * Once each handler has demuxxed the data from the queue, it creates a series of {@code MachineReading} instances which
 * serve to order and give further structure to the retrieved data.<p/>
 * Each topic handler has it own queue of MachineReading instances and so in this way the realtime data which may come back
 * in all sorts of order and at various times is categorized and ordered.<p/>
 * The other purpose of the MachineReading and the {@code MachineBridge} arbiter that mitigates the intermediate MachineReading
 * processing is XML formatting using JAXB and suppling a JQuery ready set of XML compliant structures for web facing
 * applications.
 * @author Jonathan Neville Groff (C) NeoCoreTechs 2020
 *
 */
@XmlRootElement(name="MachineReading")
@XmlAccessorType(XmlAccessType.PROPERTY)
public final class MachineReading implements Serializable{

	private static final long serialVersionUID = -3231437136596373851L;
	private int rawGroup;
	private int rawSeq;
	private int rawReadingNum;
	private double readingVal;
	private int readingValInt;
	private String readingValString;
	@XmlElement
	public int getRawGroup() {
		return rawGroup;
	}
	public void setRawGroup(int rawGroup) {
		this.rawGroup = rawGroup;
	}
	@XmlElement
	public int getRawSeq() {
		return rawSeq;
	}
	public void setRawSeq(int rawSeq) {
		this.rawSeq = rawSeq;
	}
	@XmlElement
	public int getRawReadingNum() {
		return rawReadingNum;
	}
	public void setRawReadingNum(int rawReadingNum) {
		this.rawReadingNum = rawReadingNum;
	}
	@XmlElement
	public double getReadingValDouble() {
		return readingVal;
	}
	@XmlElement
	public int getReadingValInt() {
		return readingValInt;
	}
	@XmlElement
	public String getReadingValString() {
		return readingValString;
	}
	
	public static MachineReading EMPTYREADING = new MachineReading(-1,-1,Integer.MAX_VALUE,-1);
	
	public void setReadingValDouble(double readingVal) {
		this.readingVal = readingVal;
	}
	public void setReadingValInt(int readingVal) {
		this.readingValInt = readingVal;
	}
	public void setReadingValString(String readingVal) {
		this.readingValString = readingVal;
	}
	
	public MachineReading(int rawGroup, int rawSeq, int rawReadingNum, double readingVal) {
		this.rawGroup = rawGroup;
		this.rawSeq = rawSeq;
		this.rawReadingNum = rawReadingNum;
		this.readingVal = readingVal;
	}
	public MachineReading(int rawGroup, int rawSeq, int rawReadingNum, int readingVal) {
		this.rawGroup = rawGroup;
		this.rawSeq = rawSeq;
		this.rawReadingNum = rawReadingNum;
		this.readingValInt = readingVal;
	}
	public MachineReading(int rawGroup, int rawSeq, int rawReadingNum, String readingVal) {
		this.rawGroup = rawGroup;
		this.rawSeq = rawSeq;
		this.rawReadingNum = rawReadingNum;
		this.readingValString = readingVal;
	}
	public MachineReading(String readingVal) {
		this.rawGroup = -1;
		this.rawSeq = -1;
		this.rawReadingNum = -1;
		this.readingValString = readingVal;
	}
	
	public MachineReading() {}
	
	@Override
	public String toString() {
		if(rawGroup == -1 && rawSeq == -1 && rawReadingNum == -1 )
			return readingValString;
		else
			return "Group "+rawGroup+" Sequence "+rawSeq+" Reading # "+rawReadingNum+" = "+readingVal+" "+readingValInt+" "+readingValString;
	}
	
	@Override
	public boolean equals(Object o) {
		if(o.getClass() != MachineReading.class)
			return false;
		if(((MachineReading)o).rawGroup != this.rawGroup || 
		   ((MachineReading)o).rawSeq != this.rawSeq || 
		   ((MachineReading)o).rawReadingNum == this.rawReadingNum)
			return false;
		if((((MachineReading)o).readingValString == null && readingValString != null))
			return false;
		if((((MachineReading)o).readingValString != null && readingValString == null))
			return false;
		if((((MachineReading)o).readingValString == null && readingValString == null))
			return true;
		return (((MachineReading)o).readingValString.equals(readingValString));
	}
}
