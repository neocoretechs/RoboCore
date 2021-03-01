package com.neocoretechs.robocore.marlinspike;

import org.ros.message.Time;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.robocore.machine.bridge.MachineReading;

import sensor_msgs.Range;

public class PublishUltrasonicResponse extends PublishResponses<Range> {
	private static int sequenceNumber = 0;

	public PublishUltrasonicResponse(ConnectedNode node, Publisher<Range> rangepub, CircularBlockingDeque<Range> outgoingRanges) {
		super(node, rangepub, outgoingRanges);
	}
	
	@Override
	public void setUp() {
		std_msgs.Header ihead = node.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
		ihead.setSeq(sequenceNumber);
		Time tst = node.getCurrentTime();
		ihead.setStamp(tst);
		ihead.setFrameId("0");
		msg = publisher.newMessage();
		msg.setHeader(ihead);
		msg.setFieldOfView(30);
		msg.setMaxRange(600);
		msg.setMinRange(6);
		msg.setRadiationType(sensor_msgs.Range.ULTRASOUND);
	}
	
	@Override
	public void addTo(MachineReading mr) {
		Double range = mr.getReadingValDouble();
		msg.setRange(range.floatValue());
	}

	@Override
	public String displayMessage() {
		return msg != null ? msg.toString() : "NULL";
	}

	
}
