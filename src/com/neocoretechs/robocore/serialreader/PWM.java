package com.neocoretechs.robocore.serialreader;

import java.io.IOException;

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
public class PWM extends HardwarePWM {
	
	public PWM(int pin) {
		super(pin);
	}
	
	public String read() throws IOException {
		String level = null;
		switch(pin) {
			case 19:
				level = pwmDuty1.readLine();
				pwmDuty1.seek(0);
				return level;
			case 33:
				level = pwmDuty0.readLine();
				pwmDuty0.seek(0);
				return level;
			default:
				System.out.println("Must specify  1 or 2 for hardware PWM read!");
				return null;
		}
	}
	
	public void enable(boolean enable) throws IOException {
		switch(pin) {
			case 19:
				pwmEnable1.writeBytes(enable ? "1" : "0");
				pwmEnable1.seek(0);
				break;
			case 33:
				pwmEnable0.writeBytes(enable ? "1" : "0");
				pwmEnable0.seek(0);
				break;
			default:
				System.out.println("Pin must specify 19 or 33 for hardware PWM enable!");
				return;
		}

	}
	
	public void freq(int hZ) throws IOException {
		switch(pin) {
			case 19:
				pwmFreq1.writeBytes(String.valueOf(hZ));
				pwmFreq1.seek(0);
				break;
			case 33:
				pwmFreq0.writeBytes(String.valueOf(hZ));
				pwmFreq0.seek(0);
				break;
			default:
				System.out.println("Pin Must specify 19 or 33 for hardware PWM frequency!");
				return;
		}
	}

	@Override
	public void pwmWrite(int val) throws IOException {
		switch(pin) {
			case 19:
				pwmDuty1.writeBytes(String.valueOf(val));
				pwmDuty1.seek(0);
				break;
			case 33:
				pwmDuty0.writeBytes(String.valueOf(val));
				pwmDuty0.seek(0);
				break;
			default:
				System.out.println("Must specify 19 or 33 for hardware PWM write!");
				return;
		}
	}


	@Override
	public void setCounter(int cntx) {
		((CounterInterruptService)interruptService).set_counter(cntx);
	}


	@Override
	public int getCounter() {
		return ((CounterInterruptService)interruptService).get_counter();
	}


	@Override
	public void digitalWrite(int val) throws IOException {
		if(val == 0)
			pwmWrite(0);
		else
			pwmWrite(1000);
	}


	@Override
	public void pinModeOut() {
		
	}


	@Override
	public void attachInterrupt(InterruptService cins, boolean overflow) {
		this.interruptService = cins;	
	}


	@Override
	public void detachInterrupt(boolean overflow) {
		this.interruptService = null;
		
	}
}
