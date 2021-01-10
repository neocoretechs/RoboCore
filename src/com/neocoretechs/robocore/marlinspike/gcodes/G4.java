package com.neocoretechs.robocore.marlinspike.gcodes;

import java.util.ArrayList;
import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * The responsibility of each of these consumers it to consume the first element
 * on the queue of lines read from the Marlinspike, be it the same as the passed
 * parameter 'peeked' into the retrieveData method, and continue to retrieve queue
 * elements until all elements relevant to this topic are thus consumed.<p/>
 * Many of these consumers follow the pattern of merely consuming the 'ack' from
 * a processed code.
 * G4 dwell
 * @author Jonathan N. Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class G4 implements Runnable {
	private boolean DEBUG = false;
	private boolean shouldRun = true;
	private TopicList topicList;
	AsynchDemuxer asynchDemuxer;
	private Object mutex = new Object();
	String data;
	public G4(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		this.asynchDemuxer = asynchDemuxer;
		//
		// G4
		//
		this.topicList = new TopicList(asynchDemuxer, topicNames.G4.val(), 2) {
			@Override
			public void retrieveData(ArrayList<String> readLine) throws InterruptedException {
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
		topics.put(topicNames.G4.val(), topicList);
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
