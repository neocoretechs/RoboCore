package com.neocoretechs.robocore.machine.bridge;

import java.io.Serializable;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

/**
 * @author jg
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
	
	public MachineReading() {}
	
	@Override
	public String toString() {
		return "Group "+rawGroup+" Sequence "+rawSeq+" Reading # "+rawReadingNum+" = "+readingVal+" "+readingValInt+" "+readingValString;
	}
}
