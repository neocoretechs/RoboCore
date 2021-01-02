package com.neocoretechs.robocore.marlinspike.gcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.TopicListInterface;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;

public class G4 {
	private boolean DEBUG;
	private Map<String, TopicList> topics;
	AsynchDemuxer asynchDemuxer;
	public G4(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		this.asynchDemuxer = asynchDemuxer;
		this.topics = topics;
		//
		// G4
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.G4.val());
		topics.put(topicNames.G4.val(), new TopicList(asynchDemuxer, topicNames.G4.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				getMachineBridge().add(MachineReading.EMPTYREADING);
				synchronized(asynchDemuxer.mutexWrite) {
					asynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
	}

}
