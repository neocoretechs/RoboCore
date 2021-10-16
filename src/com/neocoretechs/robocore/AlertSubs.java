package com.neocoretechs.robocore;

import java.util.Map;

import javax.sound.sampled.LineUnavailableException;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

/**
 * Listen for range finder messages and generate alarm upon receipt.
 * __MINRANGE:=30 __MAXRANGE:=300 for URM37
 * __ALERT:=true to enable audio out
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2021
 *
 */
public class AlertSubs extends AbstractNodeMain {
	private static boolean DEBUG = false;
	private static boolean ALERT = false;
	private static double MAXRANGE = -1;
	private static double MINRANGE = -1;
	private static int SEQUENCE = -1;
	private static int numberDetected = 0;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_alert");
	}
	@Override
	public void onStart(final ConnectedNode connectedNode) {
	
		//final Log log = connectedNode.getLog();
		Subscriber<std_msgs.String> subsbat = connectedNode.newSubscriber("robocore/alerts", std_msgs.String._TYPE);
		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		if( remaps.containsKey("__MAXRANGE") )
			MAXRANGE = Double.parseDouble(remaps.get("__MAXRANGE"));
		if( remaps.containsKey("__MINRANGE") )
			MINRANGE = Double.parseDouble(remaps.get("__MINRANGE"));
		if( remaps.containsKey("__SEQUENCE") )
			SEQUENCE = Integer.parseInt(remaps.get("__SEQUENCE"));
		if( remaps.containsKey("__ALERT") )
			ALERT = true;
		subsbat.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
			try
			{
				System.out.println("Status "+message.getData());
				if( (MAXRANGE != -1 && Double.parseDouble(message.getData()) < MAXRANGE) && 
					(MINRANGE == -1 || Double.parseDouble(message.getData()) > MINRANGE) ) {
					if(SEQUENCE == -1 || numberDetected >= SEQUENCE) {
						numberDetected = 0;
						System.out.println("ALERT!");
						// audio loop
						for(int i = 0; i < 15; i++) {
							try {
								if(ALERT)
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
					} else {
						++numberDetected;
					}
				} else {
					numberDetected = 0; // sequence is broken
				}
			} catch (Throwable e) {
				e.printStackTrace();
			}
			}
		});
		
	}

}
