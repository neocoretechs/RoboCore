package com.neocoretechs.robocore.serialreader.marlinspikeport;

import com.pi4j.io.gpio.PinState;
import com.pi4j.io.gpio.event.GpioPinDigitalStateChangeEvent;
import com.pi4j.io.gpio.event.GpioPinListenerDigital;

/**
 * Interrupt service that increments a counter. It can be attached to any timer or pin change to provide a monotomically
 * increasing counter of the number of overflows/compares performed by the timer.
 * In PWM, this is used to determine the number of PWM 'cycles' performed to provide a dead man switch.
 * Created: 9/9/2016 3:03:02 PM
 * @uthor Jonathan Groff Copyright (C) NeoCoreTechs 2022
 */ 
public class CounterInterruptService implements InterruptService {
	private static boolean DEBUG = true;
	private volatile int counter;
	private int pin;
	private int maxcount;
	public CounterInterruptService(int pin, int tmax) {
		this.maxcount = tmax;
		counter = 0;
	}
	
	public int getPin() {
		return pin;
	}
	
	public int get_counter() {
		return counter;
	}
		
	public void set_counter(int cntx) {
		counter = cntx;
	}

	@Override
	public void service() {
		if( counter < maxcount ) {
			++counter;
		} 
	}

}
