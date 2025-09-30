package com.neocoretechs.robocore.test;


import java.util.Arrays;

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
public class IMUTest extends AbstractNodeMain  {
	private boolean DEBUG = true;

    Object mutex = new Object();
    
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("imutest");
	}

	/**
	 *
	 */
	@Override
	public void onStart(final ConnectedNode connectedNode) {

		//final Log log = connectedNode.getLog();
		
		Subscriber<sensor_msgs.Imu> imusub = connectedNode.newSubscriber("/sensor_msgs/Imu", sensor_msgs.Imu._TYPE);

		imusub.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
			@Override
			public void onNewMessage(sensor_msgs.Imu message) {
				System.out.println(Arrays.toString(new float[] {message.getCompassHeadingDegrees(), message.getRoll(), message.getPitch()}));
			}
		});
	}
}
