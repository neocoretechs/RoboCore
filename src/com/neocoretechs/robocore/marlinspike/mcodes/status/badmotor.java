package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.Map;
import java.util.HashMap;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * 2 = HBridge driver enable pin not found
 * 4 = SplitBridge driver enable pin not found 
 * 6 = SwitchBridge driver enable pin not found
 * 7 = Variable PWM driver enable pin not found
 * 1 = Set Motor Shutdown / Set PWM Shutdown
 * 0 = Set Motor run / Set PWM run
 * 8 = Ultrasonic shutdown
 * 10 = Encoder shutdown
 * 81 = M81
 * 799 = M799 Shutdown motor/PWM
 * -1 = M799 shutdown ALL motor/PWM
 * -2 = Kill method called
 * -3 = Stop method called
 * <Bad Motor command s c p/> status, channel, power
 * @author groff
 *
 */
public class badmotor implements Runnable {
	private boolean DEBUG = false;
	private boolean shouldRun = true;
	private TopicList topicList;
	AsynchDemuxer asynchDemuxer;
	private Object mutex = new Object();
	String data;
	private HashMap<String, String> faultCodes = new HashMap<String, String>();
	public badmotor(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		this.asynchDemuxer = asynchDemuxer;
		faultCodes.put("2","HBridge driver enable pin not found");
		faultCodes.put("4", "SplitBridge driver enable pin not found");
		faultCodes.put("6", "SwitchBridge driver enable pin not found");
		faultCodes.put("7", "Variable PWM driver enable pin not found");
		faultCodes.put("1", "Set Motor Shutdown / Set PWM Shutdown");
		faultCodes.put("0", "Set Motor run / Set PWM run");
		faultCodes.put("8", "Ultrasonic shutdown");
		faultCodes.put("10", "Encoder shutdown");
		faultCodes.put("81", "M81");
		faultCodes.put("799", "M799 Shutdown motor/PWM");
		faultCodes.put("-1", "M799 shutdown ALL motor/PWM");
		faultCodes.put("-2", "Kill method called");
		faultCodes.put("-3", "Stop method called");
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
							String[] sarray = sload.trim().split(" ");
							StringBuilder sout = new StringBuilder();
							if(sarray.length > 0) 
								sout.append(faultCodes.get(sarray[0])); 
							else
								sout.append("FAULT");
							sout.append(" channel ");
							if(sarray.length > 1)
								sout.append(sarray[1]);
							else
								sout.append("UNKNOWN");
							sout.append(" power ");
							if(sarray.length > 2)
								sout.append(sarray[2]);
							else
								sout.append("UNKNOWN");
							mr = new MachineReading(sout.toString());
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
							mr = new MachineReading(data);
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
