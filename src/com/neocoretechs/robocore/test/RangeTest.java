package com.neocoretechs.robocore.test;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

/**
 * This class tests ROS node function
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2025
 *
 */
public class RangeTest extends AbstractNodeMain  {
	private boolean DEBUG = true;

    Object mutex = new Object();
    
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("rangetest");
	}

	/**
	 *
	 */
	@Override
	public void onStart(final ConnectedNode connectedNode) {

		//final Log log = connectedNode.getLog();
		
		final Subscriber<std_msgs.String> subsrange = 
				connectedNode.newSubscriber("/sensor_msgs/range",std_msgs.String._TYPE);
		
		subsrange.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				System.out.println(message.getData());
			}
		});
	}
}
