package com.neocoretechs.robocore.test;


import java.io.BufferedReader;
import java.io.Console;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Scanner;
import java.util.concurrent.atomic.AtomicInteger;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.NodeListener;

public class ChatTexter extends AbstractNodeMain {

    private static final String USER_PROMPT = "/user_prompt";
    private static final String LLM = "/model";

    //private volatile boolean running = true;
    //private Thread inputThread;
    private final AtomicInteger sequenceNumber = new AtomicInteger(0);
    Console console;
    
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("chattest");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {

        final Publisher<std_msgs.String> chatpub =
                connectedNode.newPublisher(USER_PROMPT, std_msgs.String._TYPE);
        final Subscriber<std_msgs.String> subschat = connectedNode.newSubscriber(LLM, std_msgs.String._TYPE);
        subschat.addMessageListener(
        		new MessageListener<std_msgs.String>() {
        			@Override
        			public void onNewMessage(std_msgs.String msg) {
        				System.out.println(sequenceNumber.get() + ".) " + msg.getData());
        			}
                }
        );

        connectedNode.addListener(new NodeListener() {
            @Override
            public void onError(Node node, Throwable throwable) {
            	System.out.println("Error "+throwable+" on node:"+node);
            	node.shutdown();
                //running = false;
                //if (inputThread != null) inputThread.interrupt();
            }
            @Override
            public void onShutdown(Node node) {
             	System.out.println("Shutdown on node:"+node);
                //running = false;
                //if (inputThread != null) inputThread.interrupt();
            }
            @Override public void onShutdownComplete(Node node) {
               	System.exit(1);
            }
            @Override public void onStart(ConnectedNode node) {
            	System.out.println("Startup on node:"+node);
            }
        });
        
    	connectedNode.executeCancellableLoop(new CancellableLoop() {
    		
    		@Override
    		protected void setup() {
    			console = System.console();
    	        if (console == null) {
    	            System.out.println("Console not available");
    	            return;
    	        }
    		}

    		@Override
    		protected void loop() throws InterruptedException {
    				int seq = sequenceNumber.incrementAndGet();
    				System.out.print(seq + ">");
    				String line = console.readLine();
    				if(line != null && !line.isBlank() && !line.isEmpty()) {
    					line = line.trim();
    					std_msgs.String sm = chatpub.newMessage();
    					sm.setData(line);
    					System.out.println("publishing to "+chatpub);
    					chatpub.publish(sm);
    				}
    		}
    	});
    }
    	
}
