package com.neocoretechs.robocore.marlinspike;

import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicListInterface;

public interface PublishResponseInterface<T> /*extends Runnable*/ {
	//public void takeBridgeAndQueueMessage(String topicName, byte dstatus) throws IllegalStateException;
	public TopicListInterface getTopicList();
	public void setUp();
	public void addTo(MachineReading mr);
	public void publish();
	/**
	 * Initialize the processing in preparation for threading.
	 * @param topicName The topic which this class will service;
	 * @param dstatus the DiagnosticStatus flag ERRROR, WARN, OK which will be propagated on the bus
	 * @throws IllegalStateException If the topic cant be retrieved from the collection of established topics
	 */
	void takeBridgeAndQueueMessage(String topicName, TopicListInterface tli, byte dstatus) throws IllegalStateException;
}
