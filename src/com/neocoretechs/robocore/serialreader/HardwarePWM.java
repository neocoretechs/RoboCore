package com.neocoretechs.robocore.serialreader;

public abstract class HardwarePWM {
	public int pin;
	public enum mode { INPUT, OUTPUT };
	int channel = 0;
	InterruptService interruptService=null;
	public HardwarePWM(int pin) {
		this.pin = pin;
	}
	public abstract void init(int spin);
	public abstract void pwmWrite(int val, int outputMode);
	public void pwmOff() { pwmWrite(0, 0); };
	public abstract void setPWMResolution(int bitResolution);
	public abstract void setPWMPrescale(int prescalar);
	public abstract void setCounter(int cntx);
	public abstract int getCounter();
	public abstract void digitalWrite(int val);
	public abstract void pinModeOut();
	public abstract void attachInterrupt(InterruptService cins, boolean overflow);
	public abstract void detachInterrupt(boolean overflow);
}
