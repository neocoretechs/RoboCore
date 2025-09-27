package com.neocoretechs.robocore.test;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.InputEvent;
import java.awt.event.KeyEvent;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Scanner;
import java.util.concurrent.CountDownLatch;

import javax.swing.AbstractAction;
import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
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
import org.ros.internal.node.server.SynchronizedThreadManager;
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
public class ChatTexter extends AbstractNodeMain  {
	private boolean DEBUG = true;
	public static final String SYSTEM_PROMPT = "/system_prompt";
	public static final String USER_PROMPT = "/user_prompt";
	public static final String ASSIST_PROMPT = "/assist_prompt";
	public static final String LLM = "/model";
	CountDownLatch latch = new CountDownLatch(1);
    String inputArea = null;
 
    Object mutex = new Object();
    boolean pasting = false;
	Scanner in = null;
    
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
		in = new Scanner(System.in);
		SynchronizedThreadManager.getInstance().spin(new Runnable() {
		    public void run() {
		    	while(true) {
		    	System.out.println();
				System.out.print(">");
				inputArea = in.nextLine();  // Read user input
		        if (!inputArea.isEmpty()) {
		                latch.countDown();
		        }
		    	}
		    }
		});

		subschat.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				System.out.println( message.getData());
			}
		});
		
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
			++sequenceNumber;
			latch.await();
			std_msgs.String sm = chatpub.newMessage();
			sm.setData(inputArea.trim());
			chatpub.publish(sm);
			latch = new CountDownLatch(1);
		}
	});
	}
	
}
