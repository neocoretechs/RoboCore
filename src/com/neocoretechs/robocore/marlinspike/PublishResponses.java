package com.neocoretechs.robocore.marlinspike;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.robocore.machine.bridge.MachineBridge;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicListInterface;

/**
 * Once the response lines have been received from the Marlinspike realtime subsystem in response to an M or G code,
 * the (@see AsynchDemuxer} will demux them onto a thread bearing a waiting handler that consumes the messages
 * associated with the response (in effect, the 'ack' or 'nak' to the code).<p/>
 * The handler will consume all the lines from the response, thus clearing the inbound AsynchDemuxer message queue of
 * those response lines, and place them on a (@see MachineBridge} queue as a series of {@see MachineReading}.<p/>
 * The purpose of the MachineBridge is to categorize and encapsulate and present each topic as MachineReadings 
 * in a common format that can be manipulated for display or analysis, and as such the MachineBridge and MachineReadings 
 * are XML compliant and can be used by JAXB and JQuery or other XML processing.<p/>
 * The class here disseminates those bridges and their waiting MachineReading response payloads to the diagnostic bus
 * for consumption by logging or other downstream processing.<p/>
 * @author Jonathan Groff (C) NeoCoreTechs 2021
 *
 */
public abstract class PublishResponses<T> implements PublishResponseInterface<T> {
	private static boolean DEBUG = false;
	protected ConnectedNode node;
	protected Publisher<T> publisher;
	protected T msg = null;
	protected int messageSize = 0;
	private CircularBlockingDeque<T> outgoingQueue;
	private boolean shouldRun = true;
	private TopicListInterface tli = null;
	protected String topicName = null;
	protected byte dstatus;
	/**
	 * Constructor to build the per thread queuing for Marlinspike MachineBridge response payloads onto the
	 * outgoing diagnostic status messaging bus.
	 * @param node The ConnectedNode that will do the publishing of the Diagnostic Message topics
	 * @param statpub The publisher of DiagnosticStatus messages topics connected to the ConnectedNode
	 * @param outgoingDiagnostics The queue that will finally receive and manage the responses built from the demuxer bridge payloads here
	 */
	public PublishResponses(ConnectedNode node, Publisher<T> pub, CircularBlockingDeque<T> outgoingQueue) {
		this.node = node;
		this.publisher = pub;	
		this.outgoingQueue = outgoingQueue;
	}
	/**
	 * Initialize the processing in preparation for threading.
	 * @param topicName The topic which this class will service;
	 * @param dstatus the DiagnosticStatus flag ERRROR, WARN, OK which will be propagated on the bus
	 * @throws IllegalStateException If the topic cant be retrieved from the collection of established topics
	 */
	@Override
	public void takeBridgeAndQueueMessage(String topicName, TopicListInterface tli, byte dstatus) throws IllegalStateException {
		this.topicName = topicName;
		this.dstatus = dstatus;
		this.tli = tli;//asynchDemuxer.getTopic(topicName);
		if( tli == null ) {
			System.out.println("Can't find Topic "+topicName);
			throw new IllegalStateException("Can't find Topic "+topicName+" in MarlinSpike AsynchDemuxer, possible programmatic initialization problem");
		}
	}
	
	@Override
	public TopicListInterface getTopicList() {
		return tli;
	}
	
	@Override
	public abstract void setUp();
	
	@Override
	public abstract void addTo(MachineReading mr);
	
	public abstract String displayMessage();
	
	@Override
	public void publish() {
		//while(shouldRun ) {
			//try {
				MachineBridge mb = getTopicList().getMachineBridge();
				synchronized(mb) {
					if( mb.get().isEmpty() ) {
						return;
					}
					setUp();
					messageSize = 0;
					int queueLen = mb.get().length();
					while(!mb.get().isEmpty()) {
						// failsafe to limit consumption of message elements to max size of MachineBridge queue
						// this theoretically gives us one message at a time to queue on the outbound message bus
						// and keeps system from stalling on endless consumption of one incoming message stream
						if(messageSize++ >= queueLen) {
							System.out.println(this.getClass().getName()+" "+Thread.currentThread().getName()+" message size "+messageSize+" exceeds queue length "+queueLen);
							publish();
							messageSize = 0;
							break;
						}
						MachineReading mr2 = mb.waitForNewReading();
						if( getTopicList().getResult(mr2) == null)
							continue;
						// EMPTYREADING delineates messages within queue	
						//if(mr2.equals(MachineReading.EMPTYREADING)) {
							//publish();
							//if( DEBUG ) 
								//System.out.println(this.getClass().getName()+" "+Thread.currentThread().getName()+" Queued "+topicName+": "+displayMessage());
							//continue;
						//}
						addTo(mr2);
					}
					if(messageSize > 0)
						outgoingQueue.addLast(msg);
				} // mutex MachineBridge
			//} catch (InterruptedException e) {
				//shouldRun = false;
				//if(DEBUG)
				//	System.out.println(this.getClass().getName()+" "+Thread.currentThread().getName()+" INTERRUPTED");
			//} 
			// wait for possible payload
		}
		
	//}
}
