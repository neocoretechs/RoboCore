package com.neocoretechs.robocore;

import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import sensor_msgs.Range;

import com.neocoretechs.talker.VoxHumana;

import diagnostic_msgs.DiagnosticStatus;
import diagnostic_msgs.KeyValue;

/**
 * StatusAlerts comprise robocore/status - Which has 'List' of Key/Value messages in diagnostic_msgs.DiagnosticStatus
 * robocore/range has integer range val in sensor_msgs.Range. The option to activate the speech option is also provided.
 * @author jg
 *
 */
public class StatusAlertSubs extends AbstractNodeMain {
	private static boolean DEBUG = false;
	private static boolean speak = true;
	ConcurrentLinkedQueue<String> cbq = new ConcurrentLinkedQueue<String>();
	//public static VoxHumana speaker = null;
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("pubs_status");
	}
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		
		//if( speak ) {
		//	speaker = VoxHumana.getInstance();
		//}
		//final Log log = connectedNode.getLog();
		//Subscriber<diagnostic_msgs.DiagnosticStatus> subsbat = connectedNode.newSubscriber("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);
		final Publisher<diagnostic_msgs.DiagnosticStatus> statpub =
				connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);
		Subscriber<sensor_msgs.Range> subsrange = connectedNode.newSubscriber("robocore/range", sensor_msgs.Range._TYPE);
		Subscriber<sensor_msgs.Range> subsrange2 = connectedNode.newSubscriber("ardrone/range", sensor_msgs.Range._TYPE);
	
		//subsbat.addMessageListener(new MessageListener<diagnostic_msgs.DiagnosticStatus>() {
		//@Override
		//public void onNewMessage(DiagnosticStatus message) {
		//	try
		//	{
		//		System.out.println("Status "+message.getMessage());
		//		if( speak ) {
		//			StringBuilder sb = new StringBuilder();
		//			sb.append(message.getMessage()+"\r\n");
		//			List<KeyValue> diagMsgs = message.getValues();
		//			if( diagMsgs != null ) {
		//				for( KeyValue msg : diagMsgs) {
		//					sb.append(msg.getValue()+"\r\n");
		//					//speaker.doSpeak(msg.getValue());
		//				}
		//			}
		//			speaker.doSpeak(sb.toString());
		//		}
		//	}
		//	catch (Throwable e)
		//	{
		//		e.printStackTrace();
		//	}
		//}

		//});
		
		subsrange.addMessageListener(new MessageListener<sensor_msgs.Range>() {
			@Override
			public void onNewMessage(Range message) {
				float range = message.getRange();
				if( DEBUG )
					System.out.println("Floor Range "+range);
				try {
					if(speak && (message.getRange() < 300.0) ) {
						//speaker.doSpeak
						cbq.add("Excuse me but you are "+(int)range+" centimeters too close to my feet");
					}
				} catch (Throwable e) {
					e.printStackTrace();
				}	
			}
		});
		
		subsrange2.addMessageListener(new MessageListener<sensor_msgs.Range>() {
			@Override
			public void onNewMessage(Range message) {
				float range = message.getRange();
				if( DEBUG )
					System.out.println("Head Range "+range);
				try {
					if( speak && message.getRange() < 350.0 ) {
						//speaker.doSpeak
						cbq.add("Excuse me but you are "+(int)range+" centimeters too close to my head");
					}
				} catch (Throwable e) {
					e.printStackTrace();
				}	
			}
		});
		
		// This CancellableLoop will be canceled automatically when the node shuts
		// down.
		connectedNode.executeCancellableLoop(new CancellableLoop() {
			private int sequenceNumber;
			diagnostic_msgs.DiagnosticStatus statmsg = statpub.newMessage();
			@Override
			protected void setup() {
				sequenceNumber = 0;
			}

			@Override
			protected void loop() throws InterruptedException {
				// the message in string then the values under it are extracted
				String s = cbq.poll();
				if( s != null ) {
					++sequenceNumber;
					statmsg.setMessage(s);
					statpub.publish(statmsg);
				}
				Thread.sleep(1);
			}
		});
		
	}

}
