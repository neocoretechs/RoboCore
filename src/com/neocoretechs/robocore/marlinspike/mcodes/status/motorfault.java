package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
* 	SERIAL_PGM(MSG_BEGIN);
*	SERIAL_PGM(motorFaultCntrlHdr);
*	SERIAL_PGMLN(MSG_DELIMIT);
*	for(int i = 0; i < 8 ; i++) {
*		bfault = fault & (1<<i);
*		switch(bfault) {
*			default:
*			case 0: // bit not set
*				break;
*			case 1:
*				SERIAL_PORT.print(j++);
*				SERIAL_PORT.print(' ');
*				SERIAL_PGMLN(MSG_MOTORCONTROL_1);
*				break;
*			case 2:
*				SERIAL_PORT.print(j++);
*				SERIAL_PORT.print(' ');
*				SERIAL_PGMLN(MSG_MOTORCONTROL_2);
*				break;
*			case 4:
*				SERIAL_PORT.print(j++);
*				SERIAL_PORT.print(' ');
*				SERIAL_PGMLN(MSG_MOTORCONTROL_3);
*				break;
*			case 8:
*				SERIAL_PORT.print(j++);
*				SERIAL_PORT.print(' ');
*				SERIAL_PGMLN(MSG_MOTORCONTROL_4);
*				break;
*			case 16:
*				SERIAL_PORT.print(j++);
*				SERIAL_PORT.print(' ');
*				SERIAL_PGMLN(MSG_MOTORCONTROL_5);
*				break;
*			case 32:
*				SERIAL_PORT.print(j++);
*				SERIAL_PORT.print(' ');
*				SERIAL_PGMLN(MSG_MOTORCONTROL_6);
*				break;
*			case 64:
*				SERIAL_PORT.print(j++);
*				SERIAL_PORT.print(' ');
*				SERIAL_PGMLN(MSG_MOTORCONTROL_7);
*				break;
*			case 128:
*				SERIAL_PORT.print(j++);
*				SERIAL_PORT.print(' ');
*				SERIAL_PGMLN(MSG_MOTORCONTROL_8);
*				break;
*		}
*	}
*	SERIAL_PGM(MSG_BEGIN);
*	SERIAL_PGM(motorFaultCntrlHdr);
*	SERIAL_PGMLN(MSG_TERMINATE);
* @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
*
*/
public class motorfault implements Runnable {
	private boolean DEBUG;
	private boolean shouldRun = true;
	private TopicList topicList;
	AsynchDemuxer asynchDemuxer;
	private Object mutex = new Object();
	String data;
	public motorfault(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		this.asynchDemuxer = asynchDemuxer;
		//
		// M46
		//
		this.topicList = new TopicList(asynchDemuxer, topicNames.MOTORFAULT.val(), 16) {
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
		topics.put(topicNames.MOTORFAULT.val(), topicList);
	}
	@Override
	public void run() {
		while(shouldRun) {
			synchronized(mutex) {
				try {
					mutex.wait();
					MachineReading mr = null;
					if(asynchDemuxer.isLineTerminal(data)) {
							String sload = asynchDemuxer.extractPayload(data, topicNames.MOTORFAULT.val());
							if(sload != null) {
								int reading = asynchDemuxer.getReadingNumber(sload);
								String datax =  asynchDemuxer.getReadingValueString(sload);
								mr = new MachineReading(1, reading, reading+1, datax);
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
								String sload = asynchDemuxer.extractPayload(data, topicNames.MOTORFAULT.val());
								// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
								if(sload != null) {
									int reading = asynchDemuxer.getReadingNumber(sload);
									String datax =  asynchDemuxer.getReadingValueString(sload);
									mr = new MachineReading(1, reading, reading+1, datax);
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