package com.neocoretechs.robocore.test;

import org.apache.commons.logging.Log;
import org.ros.Topics;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import std_msgs.Time;


/**
 * This class generates log messages to check connectivity between nodes in a basic manner
 * @author jg
 *
 */
public class LogPubTester extends AbstractNodeMain  {
	private boolean DEBUG = true;

	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("logpub");
	}

	/**
	 *
	 */
	@Override
	public void onStart(final ConnectedNode connectedNode) {

		//final Log log = connectedNode.getLog();
		final Publisher<rosgraph_msgs.Log> logpub =
				connectedNode.newPublisher(Topics.ROSOUT, rosgraph_msgs.Log._TYPE);
		
	/**
	 * Main publishing loop. Essentially we are publishing the data in whatever state its in, using the
	 * mutex appropriate to establish critical sections. A sleep follows each publication to keep the bus arbitrated
	 * This CancellableLoop will be canceled automatically when the node shuts down
	 */
	connectedNode.executeCancellableLoop(new CancellableLoop() {
		private int sequenceNumber;
	
		@Override
		protected void setup() {
			sequenceNumber = 0;
		}

		@Override
		protected void loop() throws InterruptedException {
			//log.info(connectedNode.getName()+" "+sequenceNumber);
			
			std_msgs.Header imghead = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
			imghead.setSeq(sequenceNumber);
			org.ros.message.Time tst = connectedNode.getCurrentTime();
			imghead.setStamp(tst);
			imghead.setFrameId(tst.toString());
			rosgraph_msgs.Log logmess = logpub.newMessage();
			logmess.setHeader(imghead);
			logpub.publish(logmess);
			sequenceNumber++;  	
			Thread.sleep(100);		
		}
	});
	}
	
}
