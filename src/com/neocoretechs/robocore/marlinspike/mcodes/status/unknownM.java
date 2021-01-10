package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.ArrayList;
import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.machine.bridge.MachineBridge;

public class unknownM implements Runnable {
	private boolean DEBUG = false;
	private boolean shouldRun = true;
	private TopicList topicList;
	AsynchDemuxer asynchDemuxer;
	private Object mutex = new Object();
	String data;
	public unknownM(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		this.asynchDemuxer = asynchDemuxer;
		//
		// UNNOWNM error
		//
		this.topicList = new TopicList(asynchDemuxer, topicNames.UNKNOWNM.val(), 2) {
			@Override
			public void retrieveData(ArrayList<String> readLine) throws InterruptedException {
				//data = readLine;
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
		topics.put(topicNames.UNKNOWNM.val(), topicList);
	}
	@Override
	public void run() {
		while(shouldRun) {
			synchronized(mutex) {
				try {
					mutex.wait();
					MachineBridge mb = topicList.getMachineBridge();
					synchronized(mb) {
						MachineReading mr = null;
						String directive = asynchDemuxer.parseDirective(data);
						while(directive != null && 
						 !(directive.equals(topicNames.UNKNOWNM.val()) && asynchDemuxer.isLineTerminal(data)) ) {
							data = asynchDemuxer.getMarlinLines().takeFirst();
							if(DEBUG)
								System.out.println(this.getClass().getName()+":"+data);
							if( data == null || data.length() == 0 ) {
								//if(DEBUG)System.out.println("Empty line returned from readLine");
								//continue;
								break;
							}
							directive = asynchDemuxer.parseDirective(data);
							mr = new MachineReading(data);
							mb.add(mr);
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
