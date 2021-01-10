package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.ArrayList;
import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M300
 * 		SERIAL_PGM(MSG_BEGIN);
 *		SERIAL_PGM(sonicCntrlHdr);
 *		SERIAL_PGMLN(MSG_DELIMIT);
 *		SERIAL_PGM("1 "); // pin
 *		SERIAL_PORT.println(pin_number);
 *		SERIAL_PGM("2 "); // sequence
 *		SERIAL_PORT.println(upin->getRange()); // range
 *		SERIAL_PGM(MSG_BEGIN);
 *		SERIAL_PGM(sonicCntrlHdr);
 *		SERIAL_PGMLN(MSG_TERMINATE);
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class ultrasonic implements Runnable {
	private boolean DEBUG;
	private boolean shouldRun = true;
	private TopicList topicList;
	AsynchDemuxer asynchDemuxer;
	private Object mutex = new Object();
	String data;
	public ultrasonic(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		this.asynchDemuxer = asynchDemuxer;
		//
		// M46
		//
		this.topicList = new TopicList(asynchDemuxer, topicNames.ULTRASONIC.val(), 8) {
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
		topics.put(topicNames.ULTRASONIC.val(), topicList);
	}
	@Override
	public void run() {
		while(shouldRun) {
			synchronized(mutex) {
				try {
					mutex.wait();
					MachineReading mr = null;
					if(asynchDemuxer.isLineTerminal(data)) {
							String sload = asynchDemuxer.extractPayload(data, topicNames.ULTRASONIC.val());
							if(sload != null) {
								int reading = asynchDemuxer.getReadingNumber(sload);
								int data =  asynchDemuxer.getReadingValueInt(sload);
								mr = new MachineReading(1, reading, reading+1, data);
							} else {
								mr = new MachineReading(data);
							}
							topicList.getMachineBridge().add(mr);
					} else {
							while( !asynchDemuxer.isLineTerminal(data) ) {
								data = asynchDemuxer.getMarlinLines().takeFirst();
								if( data == null || data.length() == 0 ) {
									//if(DEBUG)System.out.println("Empty line returned from readLine");
									//continue;
									break;
								}
								String sload = asynchDemuxer.extractPayload(data, topicNames.ULTRASONIC.val());
								// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
								if(sload != null) {
									int reading = asynchDemuxer.getReadingNumber(sload);
									int data =  asynchDemuxer.getReadingValueInt(sload);
									mr = new MachineReading(1, reading, reading+1, data);
								} else {
									mr = new MachineReading(sload);
								}
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
