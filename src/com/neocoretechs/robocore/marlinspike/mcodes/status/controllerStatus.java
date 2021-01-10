package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.ArrayList;
import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineBridge;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;

public class controllerStatus implements Runnable {
	private boolean DEBUG = true;
	private boolean shouldRun = true;
	private TopicList topicList;
	AsynchDemuxer asynchDemuxer;
	private Object mutex = new Object();
	ArrayList<String> datax;
	public controllerStatus(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		this.asynchDemuxer = asynchDemuxer;
		//
		// CONTROLLERSTATUS
		//
		this.topicList = new TopicList(asynchDemuxer, topicNames.CONTROLLERSTATUS.val(), 16) {
			@Override
			public void retrieveData(ArrayList<String> readLine) throws InterruptedException {
				datax = readLine;
				synchronized(mutex) {
					mutex.notify();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		};
		topics.put(topicNames.CONTROLLERSTATUS.val(), topicList);
	}
	@Override
	public void run() {
		while(shouldRun) {
			synchronized(mutex) {
				try {
					mutex.wait();
					MachineBridge mb = topicList.getMachineBridge();
					MachineReading mr = null;
					synchronized(mb) {
						for(String data: datax) {
							String sload = asynchDemuxer.parseDirective(data);
							if(sload != null && !(asynchDemuxer.isLineTerminal(data) && sload.equals(topicNames.CONTROLLERSTATUS.val()))) {
								if(DEBUG)
									System.out.println(this.getClass().getName()+":"+data);
								if( data == null || data.length() == 0 ) {
									//if(DEBUG)System.out.println("Empty line returned from readLine");
									continue;
								}
								sload = asynchDemuxer.parseDirective(data);
								mr = new MachineReading(data);
								mb.add(mr);
							}
						}
						mb.add(MachineReading.EMPTYREADING);
					}
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
