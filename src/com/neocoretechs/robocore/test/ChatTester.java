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
    JTextArea inputArea = null;
    JTextArea outputArea = null;
    Object mutex = new Object();
    boolean pasting = false;
    
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
		CountDownLatch latch = new CountDownLatch(1);
		SwingUtilities.invokeLater(new Runnable() {
		    public void run() {  
		        JFrame frame = new JFrame("Relatrix Chat Interface");
		        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		        frame.setSize(800, 600);
		        inputArea = new JTextArea();
		        outputArea = new JTextArea();
		        inputArea.setLineWrap(true);
		        outputArea.setLineWrap(true);
		        outputArea.setEditable(false);
		        inputArea.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_C, InputEvent.CTRL_DOWN_MASK), "copy");
		        inputArea.getActionMap().put("copy", new DefaultEditorKit.CopyAction());
		        inputArea.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_V, InputEvent.CTRL_DOWN_MASK), "paste");
		        inputArea.getActionMap().put("paste", new DefaultEditorKit.PasteAction());
		        inputArea.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_X, InputEvent.CTRL_DOWN_MASK), "cut");
		        inputArea.getActionMap().put("cut", new DefaultEditorKit.CutAction());
		        JScrollPane inputScroll = new JScrollPane(inputArea);
		        JScrollPane outputScroll = new JScrollPane(outputArea);
		        inputScroll.setBorder(BorderFactory.createTitledBorder("You"));
		        outputScroll.setBorder(BorderFactory.createTitledBorder("Model"));
		        JSplitPane splitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, inputScroll, outputScroll);
		        splitPane.setResizeWeight(0.5);
		        JButton submitButton = new JButton("Submit (Ctrl+Enter)");
		        JButton fileButton = new JButton("Read File");
		        JButton exitButton = new JButton("Exit");

		        submitButton.addActionListener(e -> {
		            String userInput = inputArea.getText().trim();
		            if (!userInput.isEmpty()) {
		                //outputArea.setText("🧠 Responding to:\n" + userInput + "\n\n🔧");
		            	inputArea.append("\n\n🧠");
		            	outputArea.append("\n\n🔧");
		                //inputArea.setText(""); // clear input
		                synchronized(mutex) {
		                	mutex.notify();
		                }
		            }
		        });
		        
		        fileButton.addActionListener(e -> {
		        	JFileChooser fileChooser = new JFileChooser();
		        	int result = fileChooser.showOpenDialog(null);
		        	if (result == JFileChooser.APPROVE_OPTION) {
		        		File selectedFile = fileChooser.getSelectedFile();
		        		try {
		        			String content = Files.readString(selectedFile.toPath(), StandardCharsets.UTF_8);
		        			inputArea.append(content);
		        		} catch (IOException ex) {
		        			ex.printStackTrace();
		        			JOptionPane.showMessageDialog(null, "Failed to read file: " + ex.getMessage());
		        		}
		        	}
		        });


		        exitButton.addActionListener(e -> System.exit(0));

		        // Add Ctrl+Enter key binding for Submit
		        inputArea.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ENTER, InputEvent.CTRL_DOWN_MASK), "submitText");
		        inputArea.getActionMap().put("submitText", new AbstractAction() {
		            public void actionPerformed(ActionEvent e) {
		                submitButton.doClick();
		            }
		        });
		        JPanel buttonPanel = new JPanel();
		        buttonPanel.add(submitButton);
		        buttonPanel.add(fileButton);
		        buttonPanel.add(exitButton);
		        frame.getContentPane().add(splitPane, BorderLayout.CENTER);
		        frame.getContentPane().add(buttonPanel, BorderLayout.SOUTH);
		        frame.setVisible(true);
		        latch.countDown();
		    }
		});
		
		subschat.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				outputArea.append(message.getData());
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
			synchronized(mutex) {
				mutex.wait();
				std_msgs.String sm = chatpub.newMessage();
				sm.setData(inputArea.getText().trim());
				chatpub.publish(sm);
				inputArea.setText("");
			}
		}
	});
	}
	
}
