package com.neocoretechs.robocore.marlinspike;

import java.util.ArrayList;
import java.util.concurrent.BrokenBarrierException;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.TopicListInterface;
import com.neocoretechs.robocore.machine.bridge.MachineBridge;
/**
 * Abstract response from Marlinspike where queue size is always 2, the ack is merely the code, 
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
public abstract class AbstractBasicResponse implements ResponseInterface {
	private boolean DEBUG = false;
	protected TopicListInterface topicList;
	protected AsynchDemuxer asynchDemuxer;
	protected String topicName;
	public AbstractBasicResponse(AsynchDemuxer asynchDemuxer, String topicName) {
		this.asynchDemuxer = asynchDemuxer;
		this.topicName = topicName;
		this.topicList = new TopicList(topicName, 2) {
			@Override
			public void retrieveData(ArrayList<String> readLine) throws InterruptedException {
				run(readLine);
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr;
			}
		};
		//topics.put(topicName, topicList);
	}
	
	public AbstractBasicResponse(AsynchDemuxer asynchDemuxer, String topicName, int queueSize) {
		this.asynchDemuxer = asynchDemuxer;
		this.topicName = topicName;
		this.topicList = new TopicList(topicName, queueSize) {
			@Override
			public void retrieveData(ArrayList<String> readLine) throws InterruptedException {
				run(readLine);
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr;
			}
		};
		//topics.put(topicName, topicList);
	}
	
	public TopicListInterface getTopicList() { return topicList; }
	
	public void run(ArrayList<String> readLine) {
		MachineBridge mb = topicList.getMachineBridge();
		synchronized(mb) {
			if(DEBUG)
				System.out.println(this.getClass().getName()+" "+topicName+" machineBridge:"+mb);
			//mb.add(MachineReading.EMPTYREADING);
			mb.notifyAll();
		}
		try {
			asynchDemuxer.mutexWrite.await();
		} catch(IllegalMonitorStateException | InterruptedException | BrokenBarrierException ims) {
			System.out.println(this.getClass().getName()+" "+Thread.currentThread().getName()+" "+
					ims+" "+topicName);
			ims.printStackTrace();
		}
	}

}
