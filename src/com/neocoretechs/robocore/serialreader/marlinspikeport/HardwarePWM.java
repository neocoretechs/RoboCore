package com.neocoretechs.robocore.serialreader.marlinspikeport;

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
	InterruptServiceInterface interruptService=null;
	// explicitly disable PWM for safety
	static {
		try {
			pwmEnable0 = new RandomAccessFile(enable0,"rw");
			pwmEnable0.writeBytes(String.valueOf(0));
			pwmEnable0.seek(0);
		} catch (IOException e) {}
		try {
			pwmEnable1 = new RandomAccessFile(enable1,"rw");
			pwmEnable1.writeBytes(String.valueOf(0));
			pwmEnable1.seek(0);
		} catch (IOException e) {}
	}
	public HardwarePWM(int pin) {
		this.pin = pin;
	}
	
	public synchronized void init(int pin, int timer_freq) throws IOException {
		try {
			switch(pin) {
				case 19:
					if(pwmDuty1 == null) {
						pwmDuty1 = new RandomAccessFile(duty1,"rw");
						pwmFreq1 = new RandomAccessFile(freq1,"rw");
						//pwmEnable1 = new RandomAccessFile(enable1,"rw");
						pwmFreq1.writeBytes(String.valueOf(timer_freq));
						pwmFreq1.seek(0);
					}
				break;
				case 33:
					if(pwmDuty0 == null) {
						pwmDuty0 = new RandomAccessFile(duty0,"rw");
						pwmFreq0 = new RandomAccessFile(freq0,"rw");
						//pwmEnable0 = new RandomAccessFile(enable0,"rw");
						pwmFreq0.writeBytes(String.valueOf(timer_freq));
						pwmFreq0.seek(0);
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
	public abstract void enable(boolean enable) throws IOException;
	public abstract void freq(int hZ) throws IOException;
	public abstract void pwmWrite(int val) throws IOException;
	public void pwmOff() throws IOException {
		enable(false);
		pwmWrite(0); 
	};
	public abstract void setCounter(int cntx);
	public abstract int getCounter();
	public abstract void digitalWrite(int val) throws IOException;
	public abstract void pinModeOut();
	public abstract void attachInterrupt(InterruptServiceInterface cins, boolean overflow);
	public abstract void detachInterrupt(boolean overflow);
}
