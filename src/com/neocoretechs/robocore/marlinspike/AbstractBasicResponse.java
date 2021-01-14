package com.neocoretechs.robocore.marlinspike;

import java.util.ArrayList;
import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.MachineBridge;
/**
 * Abstract response where queue size is always 2, the ack is merely the code, 
 * returned with the terminal line format. i.e. "<M0/>" <p/>
 * The responsibility of each of these consumers it to consume the first element
 * on the queue of lines read from the Marlinspike, be it the same as the passed
 * parameter 'peeked' into the retrieveData method, and continue to retrieve queue
 * elements until all elements relevant to this topic are thus consumed.<p/>
 * Many of these consumers follow the pattern of merely consuming the 'ack' from
 * a processed code.
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public abstract class AbstractBasicResponse {
	private boolean DEBUG;
	private TopicList topicList;
	AsynchDemuxer asynchDemuxer;
	protected String topicName;
	public AbstractBasicResponse(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics, String topicName) {
		this.asynchDemuxer = asynchDemuxer;
		this.topicName = topicName;
		this.topicList = new TopicList(asynchDemuxer, topicName, 2) {
			@Override
			public void retrieveData(ArrayList<String> readLine) throws InterruptedException {
				run();
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr;
			}
		};
		//topics.put(topicName, topicList);
	}
	public AbstractBasicResponse(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics, String topicName, int queueSize) {
		this.asynchDemuxer = asynchDemuxer;
		this.topicList = new TopicList(asynchDemuxer, topicName, queueSize) {
			@Override
			public void retrieveData(ArrayList<String> readLine) throws InterruptedException {
				run();
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr;
			}
		};
		//topics.put(topicName, topicList);
	}
	public TopicList getTopicList() { return topicList; }
	
	public void run() {
		MachineBridge mb = topicList.getMachineBridge();
		synchronized(mb) {
			mb.add(MachineReading.EMPTYREADING);
			mb.notifyAll();
		}
		synchronized(asynchDemuxer.mutexWrite) {
			try {
				asynchDemuxer.mutexWrite.unlock();
			} catch(IllegalMonitorStateException ims) {
				System.out.println(ims+" "+topicName);
				ims.printStackTrace();
			}
		}
	}

}
