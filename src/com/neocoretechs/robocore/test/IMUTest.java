package com.neocoretechs.robocore.test;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.InputEvent;
import java.awt.event.KeyEvent;
import java.util.Arrays;
import java.util.Scanner;
import java.util.concurrent.CountDownLatch;

import javax.swing.AbstractAction;
import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;
import javax.swing.JTextArea;
import javax.swing.KeyStroke;
import javax.swing.SwingUtilities;
import javax.swing.text.DefaultEditorKit;
import javax.swing.text.DefaultEditorKit.PasteAction;
import javax.swing.Action;

import org.apache.commons.logging.Log;
import org.ros.Topics;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import std_msgs.Time;

/**
 * This class tests LLM ROS node function via user integration
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
				System.out.println(Arrays.toString(message.getOrientationCovariance()));
			}
		});
	}
}
