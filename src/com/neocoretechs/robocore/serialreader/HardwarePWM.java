package com.neocoretechs.robocore.serialreader;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.RandomAccessFile;
/**
* Handle the hardware PWM pins on microcontroller of Odroid, RPi, etc
* since support for the hardware PWM under Odroid is inexplicably absent from the Hardkernel WiringPi
* implementation.<p/>
* Presupposes a modprobe of these devices has been performed:<br/>
* sudo modprobe pwm-meson npwm=2 #USING 1 PWM PIN (0 = pin 33, 1 = pin 19)  <br/>
* sudo modprobe pwm-ctrl									<br/>
* or that the modprobe directives have been added to /etc/modules <br/>
* @author Jonathan Groff Copyright (C) NeoCoreTechs 2022
*/
public abstract class HardwarePWM {
	private static final String duty0 = "/sys/devices/platform/pwm-ctrl/duty0"; // 0-1023
	private static final String freq0 = "/sys/devices/platform/pwm-ctrl/freq0"; // in Hz 0 - 1000000
	private static final String enable0 = "/sys/devices/platform/pwm-ctrl/enable0"; // 0 or 1
	private static final String duty1 = "/sys/devices/platform/pwm-ctrl/duty1";
	private static final String freq1 = "/sys/devices/platform/pwm-ctrl/freq1"; // in Hz 0 - 1000000
	private static final String enable1 = "/sys/devices/platform/pwm-ctrl/enable1"; // 0 or 1
	static RandomAccessFile pwmDuty0 = null, pwmFreq0 = null, pwmEnable0 = null, pwmDuty1 = null, pwmFreq1 = null, pwmEnable1 = null;
	public int pin;
	public enum mode { INPUT, OUTPUT };
	int channel = 0;
	InterruptService interruptService=null;
	public HardwarePWM(int pin) {
		this.pin = pin;
	}
	
	public void init(int pin) {
		try {
			switch(pin) {
				case 19:
					if(pwmDuty1 == null) {
						pwmDuty1 = new RandomAccessFile(duty1,"rw");
						pwmFreq1 = new RandomAccessFile(freq1,"rw");
						pwmEnable1 = new RandomAccessFile(enable1,"rw");
					}
				break;
				case 33:
					if(pwmDuty0 == null) {
						pwmDuty0 = new RandomAccessFile(duty0,"rw");
						pwmFreq0 = new RandomAccessFile(freq0,"rw");
						pwmEnable0 = new RandomAccessFile(enable0,"rw");
					}
				break;
				default:
					throw new RuntimeException("PWM pins limited to PWM0 - Pin 33, PWM1 = Pin 19");
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}
	public abstract void pwmWrite(int val, int outputMode) throws IOException;
	public void pwmOff() throws IOException { pwmWrite(0, 0); };
	public abstract void setCounter(int cntx);
	public abstract int getCounter();
	public abstract void digitalWrite(int val) throws IOException;
	public abstract void pinModeOut();
	public abstract void attachInterrupt(InterruptService cins, boolean overflow);
	public abstract void detachInterrupt(boolean overflow);
}
