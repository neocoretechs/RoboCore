package com.neocoretechs.robocore.propulsion;

import java.io.FileNotFoundException;
import java.io.FileWriter;
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
	public int pin;
	public enum mode { INPUT, OUTPUT };
	int channel = 0;
	InterruptServiceInterface interruptService=null;
	// explicitly disable PWM for safety
	/*
	static {
		try {
			init0();
		} catch (IOException e) {}
		try {
			init1();
		} catch (IOException e) {}
	}
	*/
	public HardwarePWM(int pin) throws IOException {
		switch(pin) {
			case 7 -> init0();
			case 12 -> init1();
			default -> throw new IOException("Only pins 7 and 12 are valid for PWM");
		}
		this.pin = pin;
	}
	/**
	 * Order freq - duty - enable
	 * @throws IOException
	 */
	private static void init0() throws IOException {
		pwm0("100000","0");
		disable0();
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {}
	}
	/**
	 * Order freq - duty - enable
	 * @throws IOException
	 */
	private static void init1() throws IOException {
		pwm1("100000","0");
		disable1();
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {}
	}
	
	public static void pwm0(String freq, String duty) throws IOException {
		writeSysfs(PWMDevice.getFreq0(), freq);
		writeSysfs(PWMDevice.getDuty0(), duty);
		enable0();
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {}
	}
	public static void pwm1(String freq, String duty) throws IOException {
		writeSysfs(PWMDevice.getFreq1(), freq);
		writeSysfs(PWMDevice.getDuty1(), duty);
		enable1();
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {}
	}
	public static void enable0() throws IOException {
		writeSysfs(PWMDevice.getEnable0(), "1");
	}
	public static void disable0() throws IOException {
		writeSysfs(PWMDevice.getEnable0(), "0");
	}
	public static void enable1() throws IOException {
		writeSysfs(PWMDevice.getEnable1(), "1");
	}
	public static void disable1() throws IOException {
		writeSysfs(PWMDevice.getEnable1(), "0");
	}
	
	public static void writeSysfs(String path, String value) throws IOException {
	    try (FileWriter fw = new FileWriter(path)) {
	        fw.write(value);
	        fw.flush();
	    }
	}
	public abstract void enable(boolean enable) throws IOException;
	public abstract void freqDuty(int hZ, int val) throws IOException;
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
			StringBuilder sb = new StringBuilder("PWM pin:"+String.valueOf(pin));
			sb.append(" PWMFreq1:");
			sb.append(PWMDevice.getFreq1());
			sb.append(" PWMDuty1:");
			sb.append(PWMDevice.getDuty1());
			sb.append(" PWMEnable1:");
			sb.append(PWMDevice.getEnable1());
			return sb.toString();
		case 7:
			sb = new StringBuilder("PWM pin:"+String.valueOf(pin));
			sb.append(" PWMFreq0:");
			sb.append(PWMDevice.getFreq0());
			sb.append(" PWMDuty0:");
			sb.append(PWMDevice.getDuty0());
			sb.append(" PWMEnable0:");
			sb.append(PWMDevice.getEnable0());
			return sb.toString();
		default:
			return ("PWM pin "+String.valueOf(pin)+" invalid, only 7 and 12 apply");
		}
	}
}
