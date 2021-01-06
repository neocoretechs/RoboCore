package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;

public class badmotor implements Runnable {
	private boolean DEBUG = true;
	private boolean shouldRun = true;
	private TopicList topicList;
	AsynchDemuxer asynchDemuxer;
	private Object mutex = new Object();
	String data;
	public badmotor(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		this.asynchDemuxer = asynchDemuxer;
		//
		// BADMOTOR
		//
		this.topicList = new TopicList(asynchDemuxer, topicNames.BADMOTOR.val(), 8) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				data = readLine;
				synchronized(mutex) {
					mutex.notify();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr;
			}
		};
		topics.put(topicNames.BADMOTOR.val(), topicList);
	}
	@Override
	public void run() {
		while(shouldRun) {
			synchronized(mutex) {
				try {
					mutex.wait();		
					MachineReading mr = null;
					int iseq = 1;
					if(asynchDemuxer.isLineTerminal(data)) {
						String sload = asynchDemuxer.extractPayload(data, topicNames.BADMOTOR.val());
						if(sload != null) {
							int reading = asynchDemuxer.getReadingNumber(sload);
							String datax =  asynchDemuxer.getReadingValueString(sload);
							mr = new MachineReading(1, reading, reading+1, datax);
						} else {
							mr = new MachineReading(data);
						}
						topicList.getMachineBridge().add(mr);
					} else {
						while(data != null && !asynchDemuxer.isLineTerminal(data)) {
							data = asynchDemuxer.getMarlinLines().takeFirst();
							if(DEBUG)
								System.out.println(this.getClass().getName()+":"+data);
							if( data == null || data.length() == 0 ) {
								//if(DEBUG)System.out.println("Empty line returned from readLine");
								//continue;
								break;
							}
							int reading = asynchDemuxer.getReadingNumber(data);
							String datax =  asynchDemuxer.getReadingValueString(data);
							mr = new MachineReading(1, iseq++, reading, datax);
							topicList.getMachineBridge().add(mr);
						}
					}
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
