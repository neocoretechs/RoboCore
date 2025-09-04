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
* sudo modprobe pwm-meson npwm=2 #USING 1 PWM PIN (0 = pin 7, 1 = pin 12)  <br/>
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
			init0();
		} catch (IOException e) {}
		try {
			init1();
		} catch (IOException e) {}
	}
	public HardwarePWM(int pin) throws IOException {
		switch(pin) {
			case 7 -> init0();
			case 12 -> init1();
			default -> throw new IOException("Only pins 7 and 12 are valid for PWM");
		}
		this.pin = pin;
	}
	
	private static void init0() throws IOException {
		if(pwmDuty0 == null) {
			pwmDuty0 = new RandomAccessFile(PWMDevice.getDuty0(),"rw");
			pwmFreq0 = new RandomAccessFile(PWMDevice.getFreq0(),"rw");
			pwmFreq0.writeBytes(String.valueOf(0));
			pwmFreq0.seek(0);
			pwmDuty0.writeBytes(String.valueOf(0));
			pwmDuty0.seek(0);
			pwmEnable0 = new RandomAccessFile(PWMDevice.getEnable0(),"rw");		
			pwmEnable0.writeBytes(String.valueOf(0));
			pwmEnable0.seek(0);
		}
	}
	
	private static void init1() throws IOException {
		if(pwmDuty1 == null) {
			pwmDuty1 = new RandomAccessFile(PWMDevice.getDuty1(),"rw");
			pwmFreq1 = new RandomAccessFile(PWMDevice.getFreq1(),"rw");
			pwmFreq1.writeBytes(String.valueOf(0));
			pwmFreq1.seek(0);
			pwmDuty1.writeBytes(String.valueOf(0));
			pwmDuty1.seek(0);
			pwmEnable1 = new RandomAccessFile(PWMDevice.getEnable1(),"rw");
			pwmEnable1.writeBytes(String.valueOf(0));
			pwmEnable1.seek(0);
		}
	}
	
	public abstract void enable(boolean enable) throws IOException;
	public abstract void freq(int hZ) throws IOException;
	public abstract void duty(int val) throws IOException;
	public void pwmOff() throws IOException {
		enable(false);
		//duty(0); 
	};
	public abstract void setCounter(int cntx);
	public abstract int getCounter();

	public abstract void attachInterrupt(InterruptServiceInterface cins, boolean overflow);
	public abstract void detachInterrupt(boolean overflow);
	
	public String toString() {
		if(this.pin == 0)
			return "PWM pin/devices uninitialized";
		switch(pin) {
		case 12:
			if(pwmDuty1 == null) {
				return "PWM pin set to:"+String.valueOf(pin)+" but PWM device files uninitialized";
			} else {
				return("PWM pin:"+String.valueOf(pin)+" Duty:"+pwmDuty1.toString()+" Freq:"+pwmFreq1.toString()+" Enable:"+pwmEnable1.toString());
			}
		case 7:
			if(pwmDuty0 == null) {
				return "PWM pin set to:"+String.valueOf(pin)+" but PWM device files uninitialized";
			} else {
				return("PWM pin:"+String.valueOf(pin)+" Duty:"+pwmDuty0.toString()+" Freq:"+pwmFreq0.toString()+" Enable:"+pwmEnable0.toString());
			}
		default:
			return ("PWM pin "+String.valueOf(pin)+" invalid, only 7 and 12 apply");
		}
	}
}
