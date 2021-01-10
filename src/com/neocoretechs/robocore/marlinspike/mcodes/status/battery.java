package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.ArrayList;
import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.machine.bridge.MachineBridge;
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
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class battery implements Runnable {
	private boolean DEBUG;
	private boolean shouldRun = true;
	private TopicList topicList;
	AsynchDemuxer asynchDemuxer;
	private Object mutex = new Object();
	ArrayList<String> datax;
	public battery(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		this.asynchDemuxer = asynchDemuxer;
		//
		// battery
		//
		this.topicList = new TopicList(asynchDemuxer, topicNames.BATTERY.val(), 5) {
			@Override
			public void retrieveData(ArrayList<String> readLine) throws InterruptedException {
				datax = readLine;
				synchronized(mutex) {
					mutex.notify();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValInt();
			}
		};
		topics.put(topicNames.BATTERY.val(), topicList);
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
							if(asynchDemuxer.isLineTerminal(data)) {
								String sload = asynchDemuxer.extractPayload(data, topicNames.BATTERY.val());
								if(sload != null) {
									int reading = asynchDemuxer.getReadingNumber(sload);
									int datai =  asynchDemuxer.getReadingValueInt(sload);
									mr = new MachineReading(1, reading, reading+1, datai);		
									mb.add(mr);
								}
							} else {
								if( !asynchDemuxer.isLineTerminal(data) ) {
								if( data == null || data.length() == 0 ) {
									//if(DEBUG)System.out.println("Empty line returned from readLine");
									continue;
								}
								String sload = asynchDemuxer.extractPayload(data, topicNames.BATTERY.val());
								// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
								if(sload != null) {
									int reading = asynchDemuxer.getReadingNumber(sload);
									int datai =  asynchDemuxer.getReadingValueInt(sload);
									mr = new MachineReading(1, reading, reading, datai);
								} else {
									mr = new MachineReading(sload);
								}
								mb.add(mr);
							}
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
