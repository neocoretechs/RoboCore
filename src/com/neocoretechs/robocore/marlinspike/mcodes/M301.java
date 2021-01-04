package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M301 P<pin> - attach ultrasonic device to pin
 * wont assign pin 0 as its sensitive
 * @author Jonathan Groff Copright (C) NeoCoreTechs 2020,2021
 *
 */
public class M301 implements Runnable {
	private boolean DEBUG;
	private boolean shouldRun = true;
	private TopicList topicList;
	AsynchDemuxer asynchDemuxer;
	private Object mutex = new Object();
	String data;
	public M301(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		this.asynchDemuxer = asynchDemuxer;
		//
		// M301
		//
		this.topicList = new TopicList(asynchDemuxer, topicNames.M301.val(), 2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				data = asynchDemuxer.getMarlinLines().takeFirst();
				synchronized(mutex) {
					mutex.notify();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		};
		topics.put(topicNames.M301.val(), topicList);
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
