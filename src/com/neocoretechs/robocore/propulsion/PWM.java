package com.neocoretechs.robocore.propulsion;

import java.io.IOException;

import com.neocoretechs.robocore.serialreader.marlinspikeport.CounterInterruptService;
import com.neocoretechs.robocore.serialreader.marlinspikeport.InterruptServiceHandlerInterface;
import com.neocoretechs.robocore.serialreader.marlinspikeport.InterruptServiceInterface;

/**
 * Handle the hardware PWM pins on microcontroller of Odroid, RPi, etc
 * since support for the hardware PWM under Odroid is inexplicably absent from the Hardkernel WiringPi
 * implementation.<p/>
 * old:
 * Presupposes a modprobe of these devices has been performed:<br/>
 * sudo modprobe pwm-meson npwm=2 #USING 1 PWM PIN (33, 19)  <br/>
 * sudo modprobe pwm-ctrl									<br/>
 * or that the modprobe directives have been added to /etc/modules <br/>
 * New way to enable pwm, create /usr/local/bin/pwm-init.sh
 *#!/bin/bash
 *# Export PWM channels
 *echo 0 > /sys/class/pwm/pwmchip0/export
 *echo 0 > /sys/class/pwm/pwmchip1/export
 *# Configure pwmchip0
 *echo 0 > /sys/class/pwm/pwmchip0/pwm0/period
 *echo 0 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle
 *echo 0 > /sys/class/pwm/pwmchip0/pwm0/enable
 *# Configure pwmchip4
 *echo 0 > /sys/class/pwm/pwmchip4/pwm0/period
 *echo 0 > /sys/class/pwm/pwmchip4/pwm0/duty_cycle
 *echo 0 > /sys/class/pwm/pwmchip4/pwm0/enable
 *# Log the milestone
 *echo "PWM channels initialized." >> /var/log/pwm-init.log
 *Create service /etc/systemd/system/pwm-init.service: <br>
 *[Unit]
 *Description=Initialize PWM channels for narratable motor control
 *After=multi-user.target
 *[Service]
 *Type=oneshot
 *ExecStart=/usr/local/bin/pwm-init.sh
 *RemainAfterExit=true
 *[Install]
 *WantedBy=multi-user.target
 *Then, commands to enable:
 *sudo systemctl daemon-reexec
 *sudo systemctl enable pwm-init.service
 *sudo systemctl start pwm-init.service
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2022
 *
 */
public class PWM extends HardwarePWM implements InterruptServiceHandlerInterface {
	public static boolean DEBUG = false;
	public static boolean DEBUGENABLE = false;
	private boolean enabled = false;
	
	public PWM(int pin) throws IOException {
		super(pin);
	}
	
	public synchronized void enable(boolean enable) throws IOException {
		enabled = enable;
		switch(pin) {
			case 7:
				enable0();
				break;
			case 12:
				enable1();
				break;
			default:
				System.out.println("Pin must specify 7 or 12 for hardware PWM enable!");
				return;
		}
		if(DEBUG || DEBUGENABLE)
			System.out.printf("%s enable pin %d value %b%n", this.getClass().getName(), pin, enabled);

	}
	@Override
	public synchronized void freqDuty(int hZ, int val) throws IOException {
		switch(pin) {
			case 7:
				pwm0(String.valueOf(hZ),String.valueOf(val));
				break;
			case 12:
				pwm1(String.valueOf(hZ),String.valueOf(val));
				break;
			default:
				System.out.println("Pin Must specify 7 or 12 for hardware PWM frequency!");
		}
		if(DEBUG)
			System.out.printf("%s freqDuty pin %d freq:%d duty:%d%n", this.getClass().getName(), pin, hZ, val);
	}

	@Override
	public synchronized void setCounter(int cntx) {
		((CounterInterruptService)interruptService).set_counter(cntx);
	}

	@Override
	public synchronized int getCounter() {
		return ((CounterInterruptService)interruptService).get_counter();
	}

	@Override
	public synchronized void attachInterrupt(InterruptServiceInterface cins, boolean overflow) {
		this.interruptService = cins;	
		cins.setInterruptServiceHandler(this);
	}

	@Override
	public synchronized void detachInterrupt(boolean overflow) {
		this.interruptService = null;	
	}

	@Override
	/**
	 * If we have attached a handler via setInterruptServiceHandler of {@link InterruptServiceInterface}
	 * then this method is called. We should have done this here in attachInterrupt. this method is part of
	 * {@link InterruptServiceHandlerInterface}
	 */
	public synchronized void handleInterrupt() throws IOException {
		enable(false);	
	}
}
