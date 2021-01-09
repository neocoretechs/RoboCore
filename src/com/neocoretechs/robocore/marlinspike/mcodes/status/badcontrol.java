package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;

public class badcontrol implements Runnable {
	private boolean DEBUG = false;
	private boolean shouldRun = true;
	private TopicList topicList;
	AsynchDemuxer asynchDemuxer;
	private Object mutex = new Object();
	String data;
	public badcontrol(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		this.asynchDemuxer = asynchDemuxer;
		//
		// BADCONTROL error
		//
		this.topicList = new TopicList(asynchDemuxer, topicNames.BADCONTROL.val(), 2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				//data = readLine;
				data = asynchDemuxer.getMarlinLines().takeFirst();
				synchronized(mutex) {
					mutex.notify();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr;
			}
		};
		topics.put(topicNames.BADCONTROL.val(), topicList);
	}
	@Override
	public void run() {
		while(shouldRun) {
			synchronized(mutex) {
				try {
					mutex.wait();		
					MachineReading mr = null;
					String directive = asynchDemuxer.parseDirective(data);
					while(directive != null && 
						 !(directive.equals(topicNames.BADCONTROL.val()) && asynchDemuxer.isLineTerminal(data)) ) {
							data = asynchDemuxer.getMarlinLines().takeFirst();
							if(DEBUG)
								System.out.println(this.getClass().getName()+":"+data);
							if( data == null || data.length() == 0 ) {
								//if(DEBUG)System.out.println("Empty line returned from readLine");
								//continue;
								break;
							}
							directive = asynchDemuxer.parseDirective(data);
							//String sload = asynchDemuxer.extractPayload(data, topicNames.M115.val());
							// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
							//if(sload != null) {
								//int reading = asynchDemuxer.getReadingNumber(sload);
								//String data =  asynchDemuxer.getReadingValueString(sload);
								//mr = new MachineReading(1, reading, reading+1, data);
							//} else {
								//mr = new MachineReading(data);
								mr = new MachineReading(data);
							//}
							topicList.getMachineBridge().add(mr);
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
