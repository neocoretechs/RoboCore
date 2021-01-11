package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.ArrayList;
import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineBridge;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
/**
 * Abstraction of Marlinspike topic result handler that loads the data without any
 * special manipulation or consideration of special formats, merely line by line copy.
 * @author groff
 *
 */
public abstract class AbstractBasicDataLoader implements Runnable {
	private boolean DEBUG = true;
	private boolean shouldRun = true;
	private TopicList topicList;
	AsynchDemuxer asynchDemuxer;
	private Object mutex = new Object();
	ArrayList<String> datax;
	String topicName;
	int queueSize;
	public AbstractBasicDataLoader(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics, String topicName, int queueSize) {
		this.asynchDemuxer = asynchDemuxer;
		this.topicName = topicName;
		this.queueSize = queueSize;
		//
		// CONTROLLERSTATUS
		//
		this.topicList = new TopicList(asynchDemuxer, topicName, queueSize) {
			@Override
			public void retrieveData(ArrayList<String> readLine) throws InterruptedException {
				datax = readLine;
				synchronized(mutex) {
					mutex.notify();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return getMachineReadingResult(mr);
			}
		};
		topics.put(topicName, topicList);
	}
	/**
	 * Typically just returning the passed MachineReading as the String payload therein,
	 * but may return the entire MachineReading if manipulation of the various sequence elements is necessary.
	 * This is used at the endpoint to deliver the final formatted MachineReadings from the MachineBridge
	 * to ultimate consumer.
	 * @param mr
	 * @return
	 */
	public abstract Object getMachineReadingResult(MachineReading mr); //return mr.getReadingValString();
	/**
	 * Format each element of the MachineReading to be added to MachineBridge from retrieved Marlinspike response lines.
	 * @param sdata The line by line Marlinspike responses
	 * @return the MachineReading formatted to spec
	 */
	public abstract MachineReading formatMachineReading(String sdata);
	
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
							if(data != null && data.length() > 0) {
								if(DEBUG)
									System.out.println(this.getClass().getName()+" "+topicName+" machineBridge:"+data);
								String sload = asynchDemuxer.parseDirective(data);
								boolean isTopic = (sload != null && sload.equals(topicName));
								if(sload != null && sload.length() > 0 && (asynchDemuxer.isLineTerminal(data) && isTopic)) 
									break;
								if( !isTopic ) {
									mr = formatMachineReading(data);
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
