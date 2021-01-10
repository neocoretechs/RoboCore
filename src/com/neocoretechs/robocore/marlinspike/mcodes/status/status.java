package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.ArrayList;
import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineBridge;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M700 - return controller status, this is the response
 * @author groff
 *
 */
public class status implements Runnable {
	private boolean DEBUG = true;
	private boolean shouldRun = true;
	private TopicList topicList;
	AsynchDemuxer asynchDemuxer;
	private Object mutex = new Object();
	String data;
	public status(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		this.asynchDemuxer = asynchDemuxer;
		//
		// M115
		//
		this.topicList = new TopicList(asynchDemuxer, topicNames.STATUS.val(), 128) {
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
		topics.put(topicNames.STATUS.val(), topicList);
	}
	@Override
	public void run() {
		while(shouldRun) {
			synchronized(mutex) {
				try {
					mutex.wait();		
					MachineReading mr = null;
					MachineBridge mb = topicList.getMachineBridge();
					synchronized(mb) {
					String sload = asynchDemuxer.parseDirective(data);
					while(!(asynchDemuxer.isLineTerminal(data) && (sload != null && sload.equals(topicNames.STATUS.val()))) ) {
							data = asynchDemuxer.getMarlinLines().takeFirst();
							if(DEBUG)
								System.out.println(this.getClass().getName()+":"+data);
							if( data == null || data.length() == 0 ) {
								//if(DEBUG)System.out.println("Empty line returned from readLine");
								//continue;
								break;
							}
							sload = asynchDemuxer.parseDirective(data);
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
