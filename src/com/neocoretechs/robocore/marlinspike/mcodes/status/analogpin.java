package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.ArrayList;
import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.machine.bridge.MachineBridge;
/**
 * M46 P<pin> - -Read analog pin 
 *	SERIAL_PGM(MSG_BEGIN);
 *	SERIAL_PGM(analogPinHdr);
 *	SERIAL_PGMLN(MSG_DELIMIT);
 *	SERIAL_PGM("1 ");
 *	SERIAL_PORT.println(pin_number);
 *	SERIAL_PGM("2 ");
 *	SERIAL_PORT.println(res);
 *	SERIAL_PGM(MSG_BEGIN);
 *	SERIAL_PGM(analogPinHdr);
 *	SERIAL_PGMLN(MSG_TERMINATE);
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class analogpin implements Runnable {
	private boolean DEBUG;
	private boolean shouldRun = true;
	private TopicList topicList;
	AsynchDemuxer asynchDemuxer;
	private Object mutex = new Object();
	ArrayList<String> datax;
	public analogpin(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		this.asynchDemuxer = asynchDemuxer;
		//
		// M46
		//
		this.topicList = new TopicList(asynchDemuxer, topicNames.ANALOGPIN.val(), 5) {
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
		topics.put(topicNames.ANALOGPIN.val(), topicList);
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
						for(String data : datax) {
							if(asynchDemuxer.isLineTerminal(data)) {
								String sload = asynchDemuxer.extractPayload(data, topicNames.ANALOGPIN.val());
							 	if(sload != null) {
								 	int reading = asynchDemuxer.getReadingNumber(sload);
									int datai =  asynchDemuxer.getReadingValueInt(sload);
									mr = new MachineReading(1, reading, reading+1, datai);
							 	} else {
							 		mr = new MachineReading(data);
							 	}
							 	mb.add(mr);
							} else {
								if( data == null || data.length() == 0 ) {
									//if(DEBUG)System.out.println("Empty line returned from readLine");
									continue;
								}
								String sload = asynchDemuxer.extractPayload(data, topicNames.ANALOGPIN.val());
								// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
								if(sload != null) {
									int reading = asynchDemuxer.getReadingNumber(sload);
									int datai =  asynchDemuxer.getReadingValueInt(sload);
									mr = new MachineReading(1, reading, reading+1, datai);
								} else {
									mr = new MachineReading(sload);
								}
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
