package com.neocoretechs.robocore.propulsion;

import java.io.IOException;
import java.io.RandomAccessFile;
/**
 * Relies on -DPWMDevices=legacy or be absent to support older distros
 * must be exported first: echo 0 > /sys/class/pwm/pwmchip0/export
 * echo 500000 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle 
 * the duty cycle (in nanoseconds). This value should be less than or equal to the period.
 * If you want a PWM signal with a frequency of 1 kHz and a 50% duty cycle, you would set the period to 
 * 1,000,000 ns (1 ms) and the duty cycle to 500,000 ns (0.5 ms).
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2025
 */
public class PWMDevice {
	private static String duty0 = "/sys/devices/platform/pwm-ctrl/duty0"; // 0-1023
	private static String freq0 = "/sys/devices/platform/pwm-ctrl/freq0"; // in Hz 0 - 1000000
	private static String enable0 = "/sys/devices/platform/pwm-ctrl/enable0"; // 0 or 1
	private static String duty1 = "/sys/devices/platform/pwm-ctrl/duty1";
	private static String freq1 = "/sys/devices/platform/pwm-ctrl/freq1"; // in Hz 0 - 1000000
	private static String enable1 = "/sys/devices/platform/pwm-ctrl/enable1"; // 0 or 1
	static {
		if(System.getProperty("PWMDevices") != null && !System.getProperty("PWMDevices").equals("legacy")) {
			duty0 = "/sys/class/pwm/pwmchip0/pwm0/duty_cycle"; // 0-1023
			freq0 = "/sys/class/pwm/pwmchip0/pwm0/period"; // in Hz 0 - 1000000
			enable0 = "/sys/class/pwm/pwmchip0/pwm0/enable"; // 0 or 1
			duty1 = "/sys/class/pwm/pwmchip0/pwm1/duty_cycle";
			freq1 = "/sys/class/pwm/pwmchip0/pwm1/period"; // in Hz 0 - 1000000
			enable1 = "/sys/class/pwm/pwmchip0/pwm1/enable"; // 0 or 1
		}
	}
	/**
	 * @return the duty0
	 */
	public static String getDuty0() {
		return duty0;
	}
	/**
	 * @return the freq0
	 */
	public static String getFreq0() {
		return freq0;
	}
	/**
	 * @return the enable0
	 */
	public static String getEnable0() {
		return enable0;
	}
	/**
	 * @return the duty1
	 */
	public static String getDuty1() {
		return duty1;
	}
	/**
	 * @return the freq1
	 */
	public static String getFreq1() {
		return freq1;
	}
	/**
	 * @return the enable1
	 */
	public static String getEnable1() {
		return enable1;
	}
	public static void main(String[] args) throws IOException {
		try (RandomAccessFile pwmE = new RandomAccessFile(enable0,"rw")) {
		try (RandomAccessFile pwmD = new RandomAccessFile(duty0,"rw")) {
			try (RandomAccessFile pwmP = new RandomAccessFile(freq0,"rw")) {
					pwmE.writeBytes("1");
					pwmD.writeBytes("1000000");
					pwmP.writeBytes("1000000");
					try {
						Thread.sleep(30000);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
					pwmE.writeBytes("0"); // disable
			}
		}
		}

	}
}
