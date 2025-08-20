package com.neocoretechs.robocore.propulsion;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.RandomAccessFile;

import com.neocoretechs.robocore.serialreader.marlinspikeport.InterruptServiceInterface;
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
	static RandomAccessFile pwmDuty0 = null, pwmFreq0 = null, pwmEnable0 = null, pwmDuty1 = null, pwmFreq1 = null, pwmEnable1 = null;
	public int pin;
	public enum mode { INPUT, OUTPUT };
	int channel = 0;
	InterruptServiceInterface interruptService=null;
	// explicitly disable PWM for safety
	static {
		try {
			pwmEnable0 = new RandomAccessFile(PWMDevice.getEnable0(),"rw");
			pwmEnable0.writeBytes(String.valueOf(0));
			pwmEnable0.seek(0);
		} catch (IOException e) {}
		try {
			pwmEnable1 = new RandomAccessFile(PWMDevice.getEnable1(),"rw");
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
						pwmDuty1 = new RandomAccessFile(PWMDevice.getDuty1(),"rw");
						pwmFreq1 = new RandomAccessFile(PWMDevice.getFreq1(),"rw");
						//pwmEnable1 = new RandomAccessFile(enable1,"rw");
						pwmFreq1.writeBytes(String.valueOf(timer_freq));
						pwmFreq1.seek(0);
					}
				break;
				case 33:
					if(pwmDuty0 == null) {
						pwmDuty0 = new RandomAccessFile(PWMDevice.getDuty0(),"rw");
						pwmFreq0 = new RandomAccessFile(PWMDevice.getFreq0(),"rw");
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
