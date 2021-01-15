package com.neocoretechs.robocore.marlinspike;

import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicListInterface;

public interface PublishResponseInterface<T> extends Runnable {
	public void takeBridgeAndQueueMessage(String topicName, byte dstatus) throws IllegalStateException;
	public TopicListInterface getTopicList();
	public void setUp();
	public void addTo(MachineReading mr);
	public void publish();
}
