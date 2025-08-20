package com.neocoretechs.robocore.propulsion;

import java.io.IOException;
import java.io.RandomAccessFile;
/**
 * Relies on -DPWMDevices=legacy or be absent to support older distros
 * @author Jonathan Groff Copyright (C) NeoCoreTEchs 2025
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
			enable0 = ":/sys/class/pwm/pwmchip0/pwm0/enable"; // 0 or 1
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
		try (RandomAccessFile pwm = new RandomAccessFile("/sys/devices/platform/pwm-ctrl/duty0","rw")) {
			for(int i = 0; i < args.length; i++) {
				pwm.writeBytes(args[i]);
				pwm.seek(0);
				String pwmr = pwm.readLine();
				System.out.println(pwmr);
			}
			//FileDescriptor fd = pwm.getFD();
		}

	}
}
