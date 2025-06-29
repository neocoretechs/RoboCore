package com.neocoretechs.robocore.test;

import java.util.Scanner;

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
public class ChatTester extends AbstractNodeMain  {
	private boolean DEBUG = true;
	public static final String SYSTEM_PROMPT = "/system_prompt";
	public static final String USER_PROMPT = "/user_prompt";
	public static final String ASSIST_PROMPT = "/assist_prompt";
	public static final String LLM = "/model";
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("chattest");
	}

	/**
	 *
	 */
	@Override
	public void onStart(final ConnectedNode connectedNode) {

		//final Log log = connectedNode.getLog();
		final Publisher<std_msgs.String> chatpub =
				connectedNode.newPublisher(USER_PROMPT, std_msgs.String._TYPE);
		Subscriber<std_msgs.String> subschat = connectedNode.newSubscriber(LLM, std_msgs.String._TYPE);
		subschat.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				System.out.println("<"+message.getData()+">");
			}
		});
		
	/**
	 * Main publishing loop. Essentially we are publishing the data in whatever state its in, using the
	 * mutex appropriate to establish critical sections. A sleep follows each publication to keep the bus arbitrated
	 * This CancellableLoop will be canceled automatically when the node shuts down
	 */
	connectedNode.executeCancellableLoop(new CancellableLoop() {
		private int sequenceNumber;
		Scanner in = null;
		
		@Override
		protected void setup() {
			sequenceNumber = 0;
			in = new Scanner(System.in);
		}

		@Override
		protected void loop() throws InterruptedException {
			++sequenceNumber;
			System.out.print("> ");
			String s = in.nextLine();
			std_msgs.String sm = chatpub.newMessage();
			sm.setData(s);
			chatpub.publish(sm);
		}
	});
	}
	
}
