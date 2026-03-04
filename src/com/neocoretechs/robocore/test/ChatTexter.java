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
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import std_msgs.Time;

/**
 * This class tests LLM ROS node function via user integration
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2025
 *
 */
public class ChatTexter extends AbstractNodeMain {

    private static final String USER_PROMPT = "/user_prompt";
    private static final String LLM = "/model";

    private volatile boolean running = true;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("chattest");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {

        final Publisher<std_msgs.String> chatpub =
                connectedNode.newPublisher(USER_PROMPT, std_msgs.String._TYPE);

        connectedNode.newSubscriber(LLM, std_msgs.String._TYPE)
                .addMessageListener(msg -> {
                    System.out.println();
                    System.out.println(((std_msgs.String) msg).getData());
                });

        Thread inputThread = new Thread(() -> {
            try (Scanner in = new Scanner(System.in)) {
				while (running && in.hasNextLine()) {
				    System.out.print(">");
				    String line = in.nextLine().trim();
				    if (!line.isEmpty()) {
				        std_msgs.String sm = chatpub.newMessage();
				        sm.setData(line);
				        chatpub.publish(sm);
				    }
				}
			}
        });

        inputThread.setDaemon(true);
        inputThread.start();

        connectedNode.addListener(new NodeListener() {
			@Override
			public void onError(Node arg0, Throwable arg1) {	
				running = false;
			}
			@Override
			public void onShutdown(Node arg0) {
				running = false;	
			}
			@Override
			public void onShutdownComplete(Node arg0) {		
			}
			@Override
			public void onStart(ConnectedNode arg0) {
			}
        });
    }
}