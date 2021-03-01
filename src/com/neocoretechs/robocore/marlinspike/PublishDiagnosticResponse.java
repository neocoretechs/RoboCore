package com.neocoretechs.robocore.marlinspike;

import java.text.DateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.robocore.machine.bridge.MachineReading;

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
 * Class was then extended to allow generic diagnostic responses to be constructed for publishing to status alert
 * consumer outside of the Marlinspike subsystem with the addition of monolithic constructor.<p/>
 * @author Jonathan Groff (C) NeoCoreTechs 2021
 *
 */
public class PublishDiagnosticResponse extends PublishResponses<DiagnosticStatus> {
	private static boolean DEBUG = false;
	private static int MAXMESSAGESIZE = 1024; // upper bound to keep endless loops from hanging status
	List<diagnostic_msgs.KeyValue> li = null;	
	/**
	 * Constructor to build the per thread queuing for Marlinspike MachineBridge response payloads onto the
	 * outgoing diagnostic status messaging bus.
	 * @param asynchDemuxer The demuxer that initially builds the bridges of Marlinspike responses
	 * @param node The ConnectedNode that will do the publishing of the Diagnostic Message topics
	 * @param statpub The publisher of DiagnosticStatus messages topics connected to the ConnectedNode
	 * @param outgoingDiagnostics The queue that will finally receive and manage the responses built from the demuxer bridge payloads here
	 */
	public PublishDiagnosticResponse(ConnectedNode node, Publisher<DiagnosticStatus> statpub, CircularBlockingDeque<DiagnosticStatus> outgoingDiagnostics) {
		super(node, statpub, outgoingDiagnostics);
	}
	/**
	 * Generic method to format status message and publish to stated publisher outside of AsynchDemuxer.
	 * @param node The node originating the status message
	 * @param statpub The publisher to the status message bus
	 * @param outgoingDiagnostics queue to stash response for future publishing
	 * @param messages A list of string values to be formatted and published
	 */
	public PublishDiagnosticResponse(ConnectedNode node, Publisher<DiagnosticStatus> statpub, 
										CircularBlockingDeque<DiagnosticStatus> outgoingDiagnostics, 
										String topicName, byte dstatus, List<String> messages) {
		super(node, statpub, outgoingDiagnostics);
		if(messages.size() > MAXMESSAGESIZE || outgoingDiagnostics.length() > MAXMESSAGESIZE)
			throw new RuntimeException("GLOBAL MAXIMUM MESSAGE SIZE OF "+MAXMESSAGESIZE+
					" EXCEEDED BY EITHER NEW MESSAGE BUFFER AT:"+messages.size()+" OR "+
					" OUTGOING MESSAGE QUEUE AT:"+outgoingDiagnostics.length());
		this.topicName = topicName;
		this.dstatus = dstatus;
		setUp();
		for(String s : messages) {
			diagnostic_msgs.KeyValue kv2 = node.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
			kv2.setKey(String.valueOf(messageSize++)+".)");
			kv2.setValue(s);
			li.add(kv2);
		}
		outgoingDiagnostics.addLast(msg);
	}
	
	@Override
	public void setUp() {
		messageSize = 0;
		msg = publisher.newMessage();
		msg.setName(topicName);
		msg.setLevel(dstatus);
		msg.setHardwareId(node.getUri().toString());
		DateFormat d = DateFormat.getDateTimeInstance();
		msg.setMessage(d.format(new Date()));
		li = new ArrayList<diagnostic_msgs.KeyValue>();
		msg.setValues(li);
	}
	
	@Override
	public void addTo(MachineReading mr) {
		diagnostic_msgs.KeyValue kv2 = node.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
		kv2.setKey(String.valueOf(messageSize)+".)");
		kv2.setValue(String.valueOf(getTopicList().getResult(mr)));
		li.add(kv2);
	}
	
	@Override
	public String displayMessage() {
		return msg.getHardwareId()+", "+msg.getName()+", "+msg.getMessage();
	}
}
