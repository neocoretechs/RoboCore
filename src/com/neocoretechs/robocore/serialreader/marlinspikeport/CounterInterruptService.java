package com.neocoretechs.robocore.serialreader.marlinspikeport;

import java.io.IOException;

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
public class CounterInterruptService implements InterruptServiceInterface {
	private static boolean DEBUG = false;
	private volatile int counter;
	private int pin;
	private int maxcount;
	private InterruptServiceHandlerInterface ishi;
	public CounterInterruptService(int pin, int tmax) {
		this.pin = pin;
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
	public void setInterruptServiceHandler(InterruptServiceHandlerInterface ishi) {
		this.ishi = ishi;
		if(DEBUG)
			System.out.printf("%s pin %d handler %s%n", this.getClass().getName(), pin, ishi);
	}
	
	@Override
	public void service() throws IOException {
		if( counter < maxcount ) {
			++counter;
		} else {
			if(ishi != null)
				ishi.handleInterrupt();
			counter = 0;
		}
	}
	
	@Override
	public String toString() {
		return String.format("CounterInterruptService pin:%d counter:%d max count:%d", pin, counter, maxcount);
	}

}
