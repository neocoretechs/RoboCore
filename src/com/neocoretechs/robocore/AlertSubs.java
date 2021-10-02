package com.neocoretechs.robocore;


import javax.sound.sampled.LineUnavailableException;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;



/**
 * Listen for range finder messages and generate alarm upon receipt
 * @author jg
 *
 */
public class AlertSubs extends AbstractNodeMain {
	private static boolean DEBUG = false;
	private static boolean speak = true;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_speech");
	}
	@Override
	public void onStart(final ConnectedNode connectedNode) {
	
		//final Log log = connectedNode.getLog();
		Subscriber<std_msgs.String> subsbat = connectedNode.newSubscriber("robocore/alerts", std_msgs.String._TYPE);
	
		subsbat.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
			try
			{
				System.out.println("Status "+message.getData());
				for(int i = 0; i < 15; i++) {
					try {
						GenerateTone.generateTone(1500, 50, 100, true);
					} catch (LineUnavailableException e1) {
						e1.printStackTrace();
					}
					try {
						Thread.sleep(10);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			} catch (Throwable e) {
				e.printStackTrace();
			}
			}
		});
		
	}

}
