package com.neocoretechs.robocore.propulsion;

import java.io.IOException;
import java.io.RandomAccessFile;
/**
 * Relies on -DPWMDevices=legacy or be absent to support older distros
 * must be exported first: echo 0 > /sys/class/pwm/pwmchip0/export - this will create the pwm device tree under the export directory.
 * The order is important: must write 1 - period, 2 - duty_cycle, 3 - enable. <p>
 * echo 50000 > /sys/class/pwm/pwmchip0/pwm0/period <br>
 * echo 25000 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle <br>
 * the duty cycle (in nanoseconds). This value should be less than or equal to the period.
 * If you want a PWM signal with a frequency of 1 kHz and a 50% duty cycle, you would set the period to 
 * 1,000,000 ns (1 ms) and the duty cycle to 500,000 ns (0.5 ms). 10kHz set the period to 50000 and duty_cycle to 25000.
 * On Odroid C4 PWM_C pin 7 maps to pwmchip0/pwm0 and PWM_E pin 12 maps to pwmchip4/pwm0. Under Ubuntu 20.04 kernel 5.x <br>
 * On the ROSLET robot left wheel maps to PWM_C and dir is PIN_16 (offset 66). Right wheel is PWM_E and PIN_18 (offset 67) on gpiochip1.<br>
 * The gpioset gpiochip1 66=1 sets PIN_16, which is offset 66, to 1. At 1kHz 100% duty cycle the 35k gear motor draws 28W <br>
 * At 10kHz and 50% duty cycle only 13W but with almost no noticeable difference in speed.<br>
 * IMPORTANT: Cannot immediately reverse direction with 35k gear motors. Board will crash. 
 * Must disable PWM, wait 200-500ms, set opposite direction, then enable PWM.
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2025
 */
public class PWMDevice {
	private static String duty0 = ((System.getProperty("PWMDevices") != null && System.getProperty("PWMDevices").equals("legacy")) ?
			"/sys/devices/platform/pwm-ctrl/duty0" : "/sys/class/pwm/pwmchip0/pwm0/duty_cycle"); 
	private static String freq0 = ((System.getProperty("PWMDevices") != null && System.getProperty("PWMDevices").equals("legacy")) ?
			"/sys/devices/platform/pwm-ctrl/freq0" : "/sys/class/pwm/pwmchip0/pwm0/period"); 
	private static String enable0 = ((System.getProperty("PWMDevices") != null && System.getProperty("PWMDevices").equals("legacy")) ?
			"/sys/devices/platform/pwm-ctrl/enable0" : "/sys/class/pwm/pwmchip0/pwm0/enable"); // 0 or 1
	private static String duty1 = ((System.getProperty("PWMDevices") != null && System.getProperty("PWMDevices").equals("legacy")) ?
			"/sys/devices/platform/pwm-ctrl/duty1" : "/sys/class/pwm/pwmchip4/pwm0/duty_cycle");
	private static String freq1 =((System.getProperty("PWMDevices") != null && System.getProperty("PWMDevices").equals("legacy")) ?
			"/sys/devices/platform/pwm-ctrl/freq1" : "/sys/class/pwm/pwmchip4/pwm0/period");
	private static String enable1 = ((System.getProperty("PWMDevices") != null && System.getProperty("PWMDevices").equals("legacy")) ?
			"/sys/devices/platform/pwm-ctrl/enable1" : "/sys/class/pwm/pwmchip4/pwm0/enable"); // 0 or 1
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
	
}
