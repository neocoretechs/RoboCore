package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.ArrayList;
import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
* Split bridge or 2 half bridge motor controller. Takes 2 inputs: one for forward,called P, one for backward,called Q, then motor channel, 
* and then D, an enable pin. Finally, W<encoder pin>  to receive hall wheel sensor signals and 
* optionally PWM timer setup [R<resolution 8,9,10 bits>] [X<prescale 0-7>].
* Everything derived from HBridgeDriver can be done here.
* M4 [Z<slot>] P<pin> Q<pin> C<channel> D<enable pin> E<default dir> W<encoder pin> [R<resolution 8,9,10 bits>] [X<prescale 0-7>]
* @author Jonathan Groff (C) NeoCoreTechs 2020,2021
*
*/
public class M4 implements Runnable {
	private boolean DEBUG;
	private boolean shouldRun = true;
	private TopicList topicList;
	AsynchDemuxer asynchDemuxer;
	private Object mutex = new Object();
	public M4(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		this.asynchDemuxer = asynchDemuxer;
		//
		// M4 - setup bridge controller
		//
		this.topicList = new TopicList(asynchDemuxer, topicNames.M4.val(), 2) {
			@Override
			public void retrieveData(ArrayList<String> readLine) throws InterruptedException {
				synchronized(mutex) {
					mutex.notify();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		};
		topics.put(topicNames.M4.val(), topicList);
	}
	@Override
	public void run() {
		while(shouldRun) {
			synchronized(mutex) {
				try {
					mutex.wait();
					topicList.getMachineBridge().add(MachineReading.EMPTYREADING);
					synchronized(asynchDemuxer.mutexWrite) {
						asynchDemuxer.mutexWrite.notifyAll();
					}
				} catch (InterruptedException e) {
					shouldRun = false;
				}
			}
		}
		
	}

}
