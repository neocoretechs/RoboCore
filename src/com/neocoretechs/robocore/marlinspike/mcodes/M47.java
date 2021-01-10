package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.ArrayList;
import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M47 -Read analog pin P<pin> T<threshold> compare to battery threshold, if below, print battery message.<p/>
 * Unlike other operations, this one will ack with either the return M47/, or a battery message:<br/>
 * 	SERIAL_PGM(MSG_BEGIN);<br/>
 *	SERIAL_PGM(batteryCntrlHdr);<br/>
 *	SERIAL_PGMLN(MSG_DELIMIT);<br/>
 *	SERIAL_PGM("1 ");<br/>
 *	SERIAL_PORT.println(volts);<br/>
 *	SERIAL_PGM(MSG_BEGIN);<br/>
 *	SERIAL_PGM(batteryCntrlHdr);<br/>
 *	SERIAL_PGMLN(MSG_TERMINATE);<br/>
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M47 implements Runnable {
	private boolean DEBUG;
	private boolean shouldRun = true;
	private TopicList topicList;
	AsynchDemuxer asynchDemuxer;
	private Object mutex = new Object();
	public M47(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		this.asynchDemuxer = asynchDemuxer;
		//
		// M47
		//
		this.topicList = new TopicList(asynchDemuxer, topicNames.M47.val(), 2) {
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
		topics.put(topicNames.M47.val(), topicList);
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
