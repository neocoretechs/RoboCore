package com.neocoretechs.robocore.serialreader;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.RandomAccessFile;
/**
 * Handle the hardware PWM pins on microcontroller of Odroid, RPi, etc
 * since support for the hardware PWM under Odroid is inexplicably absent from the Hardkernel WiringPi
 * implementation.<p/>
 * Presupposes a modprobe of these devices has been performed:<br/>
 * sudo modprobe pwm-meson npwm=2 #USING 1 PWM PIN (33, 19)  <br/>
 * sudo modprobe pwm-ctrl									<br/>
 * or that the modprobe directives have been added to /etc/modules <br/>
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2022
 *
 */
public class PWM {
	private static final String duty0 = "/sys/devices/platform/pwm-ctrl/duty0"; // 0-1023
	private static final String freq0 = "/sys/devices/platform/pwm-ctrl/freq0"; // in Hz 0 - 1000000
	private static final String enable0 = "/sys/devices/platform/pwm-ctrl/enable0"; // 0 or 1
	private static final String duty1 = "/sys/devices/platform/pwm-ctrl/duty1";
	private static final String freq1 = "/sys/devices/platform/pwm-ctrl/freq1"; // in Hz 0 - 1000000
	private static final String enable1 = "/sys/devices/platform/pwm-ctrl/enable1"; // 0 or 1
	static {
		try {
			pwmDuty0 = new RandomAccessFile(duty0,"rw");
			pwmFreq0 = new RandomAccessFile(freq0,"rw");
			pwmEnable0 = new RandomAccessFile(enable0,"rw");
			pwmDuty1 = new RandomAccessFile(duty1,"rw");
			pwmFreq1 = new RandomAccessFile(freq1,"rw");
			pwmEnable1 = new RandomAccessFile(enable1,"rw");
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}
	static RandomAccessFile pwmDuty0, pwmFreq0, pwmEnable0, pwmDuty1, pwmFreq1, pwmEnable1;
	public void write(int ch, int level) throws IOException {
		switch(ch) {
			case 1:
				pwmDuty0.writeBytes(String.valueOf(level));
				pwmDuty0.seek(0);
				break;
			case 2:
				pwmDuty1.writeBytes(String.valueOf(level));
				pwmDuty1.seek(0);
			default:
				System.out.println("Must specify channel 1 or 2 for hardware PWM write!");
				return;
		}

	}
	
	public String read(int ch) throws IOException {
		String level = null;
		switch(ch) {
			case 1:
				level = pwmDuty0.readLine();
				pwmDuty0.seek(0);
				return level;
			case 2:
				level = pwmDuty1.readLine();
				pwmDuty1.seek(0);
				return level;
			default:
				System.out.println("Must specify channel 1 or 2 for hardware PWM read!");
				return null;
		}
	}
	
	public void enable(int ch, boolean enable) throws IOException {
		switch(ch) {
			case 1:
				pwmEnable0.writeBytes(enable ? "1" : "0");
				pwmEnable0.seek(0);
				break;
			case 2:
				pwmEnable1.writeBytes(enable ? "1" : "0");
				pwmEnable1.seek(0);
			default:
				System.out.println("Must specify channel 1 or 2 for hardware PWM enable!");
				return;
		}

	}
	
	public void freq(int ch, int level) throws IOException {
		switch(ch) {
			case 1:
				pwmFreq0.writeBytes(String.valueOf(level));
				pwmFreq0.seek(0);
				break;
			case 2:
				pwmFreq1.writeBytes(String.valueOf(level));
				pwmFreq1.seek(0);
			default:
				System.out.println("Must specify channel 1 or 2 for hardware PWM frequency!");
				return;
		}

	}
}
