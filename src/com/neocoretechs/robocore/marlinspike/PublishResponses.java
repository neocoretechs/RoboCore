package com.neocoretechs.robocore.marlinspike;

import java.text.DateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.robocore.machine.bridge.MachineBridge;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicListInterface;

import diagnostic_msgs.DiagnosticStatus;
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
public class PublishResponses implements Runnable {
	private static boolean DEBUG = true;
	private AsynchDemuxer asynchDemuxer;
	private ConnectedNode node;
	private Publisher<DiagnosticStatus> statpub;
	private diagnostic_msgs.DiagnosticStatus statmsg = null;
	private CircularBlockingDeque<DiagnosticStatus> outgoingDiagnostics;
	private boolean shouldRun = true;
	private TopicListInterface tli = null;
	private String topicName = null;
	private byte dstatus;
	/**
	 * Constructor to build the per thread queuing for Marlinspike MachineBridge response payloads onto the
	 * outgoing diagnostic status messaging bus.
	 * @param asynchDemuxer The demuxer that initially builds the bridges of Marlinspike responses
	 * @param node The ConnectedNode that will do the publishing of the Diagnostic Message topics
	 * @param statpub The publisher of DiagnosticStatus messages topics connected to the ConnectedNode
	 * @param outgoingDiagnostics The queue that will finally receive and manage the responses built from the demuxer bridge payloads here
	 */
	public PublishResponses(AsynchDemuxer asynchDemuxer, ConnectedNode node, Publisher<DiagnosticStatus> statpub, CircularBlockingDeque<DiagnosticStatus> outgoingDiagnostics) {
		this.asynchDemuxer = asynchDemuxer;
		this.node = node;
		this.statpub = statpub;	
		this.outgoingDiagnostics = outgoingDiagnostics;
	}
	/**
	 * Initialize the processing in preparation for threading.
	 * @param topicName The topic which this class will service;
	 * @param dstatus the DiagnosticStatus flag ERRROR, WARN, OK which will be propagated on the bus
	 * @throws IllegalStateException If the topic cant be retrieved from the collection of established topics
	 */
	public void takeBridgeAndQueueMessage(String topicName, byte dstatus) throws IllegalStateException {
		this.topicName = topicName;
		this.dstatus = dstatus;
		this.tli = asynchDemuxer.getTopic(topicName);
		if( tli == null ) {
			System.out.println("Can't find Topic "+topicName);
			throw new IllegalStateException("Can't find Topic "+topicName+" in MarlinSpike AsynchDemuxer "+asynchDemuxer+", possible programmatic initialization problem");
		}
	}

	@Override
	public void run() {
		while(shouldRun ) {
			MachineBridge mb = tli.getMachineBridge();
			synchronized(mb) {
			if( !mb.get().isEmpty() ) {
					String mfd = (String) tli.getResult(mb.waitForNewReading());
					statmsg = statpub.newMessage();
					statmsg.setName(topicName);
					statmsg.setLevel(dstatus);
					statmsg.setHardwareId(node.getUri().toString());
					statmsg.setMessage(topicName+":"+mfd);
					diagnostic_msgs.KeyValue kv = node.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					DateFormat d = DateFormat.getDateTimeInstance();
					kv.setValue(d.format(new Date()));
					li.add(kv);
					int messageSize = 0;
					while(!mb.get().isEmpty()) {
						MachineReading mr2 = mb.waitForNewReading();
						// failsafe to limit consumption of message elements to max size of MachineBridge queue
						// this theoretically gives us one message at a time to queue on the outbound message bus
						// and keeps system from stalling on endless consumption of one incoming message stream
						if(messageSize++ > mb.get().length())
							break;
						if(mr2.equals(MachineReading.EMPTYREADING))
							continue;
						diagnostic_msgs.KeyValue kv2 = node.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
						kv2.setKey(String.valueOf(messageSize)+")");
						kv2.setValue(String.valueOf(tli.getResult(mr2)));
						li.add(kv2);
					}
					statmsg.setValues(li);
					outgoingDiagnostics.addLast(statmsg);
					if( DEBUG ) 
						System.out.println("Queued "+topicName+": "+statmsg.getMessage().toString());
			} else {
				try {
					Thread.sleep(1);
				} catch (InterruptedException e) {
					shouldRun = false;
					if(DEBUG)
						System.out.println(this.getClass().getName()+" "+Thread.currentThread().getName()+" INTERRUPTED");
				} // wait for possible payload
			}
			}
		}
		
	}
}
