package com.neocoretechs.robocore;

import java.time.LocalDateTime;
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
 * __TIME:=minimum milliseconds between detections to activate
 * __NUMBER:=number of readings within __TIME to activate
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2021
 *
 */
public class AlertSubs extends AbstractNodeMain {
	private static boolean DEBUG = false;
	private static boolean ALERT = false;
	//private static double MAXRANGE = -1;
	//private static double MINRANGE = -1;
	private static int TIME = -1;
	private static int NUMBER = 2;
	private static int numberDetected = 0;
	private static long time = -1;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_alert");
	}
	@Override
	public void onStart(final ConnectedNode connectedNode) {
	
		//final Log log = connectedNode.getLog();
		Subscriber<std_msgs.String> subsbat = connectedNode.newSubscriber("robocore/alerts", std_msgs.String._TYPE);
		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		//if( remaps.containsKey("__MAXRANGE") )
		//	MAXRANGE = Double.parseDouble(remaps.get("__MAXRANGE"));
		//if( remaps.containsKey("__MINRANGE") )
		//	MINRANGE = Double.parseDouble(remaps.get("__MINRANGE"));
		if( remaps.containsKey("__TIME") ) {
			TIME = Integer.parseInt(remaps.get("__TIME"));
		}
		if( remaps.containsKey("__NUMBER") ) {
			NUMBER = Integer.parseInt(remaps.get("__NUMBER"));
		}
		if( remaps.containsKey("__ALERT") ) {
			ALERT = true;
		}
		time = System.currentTimeMillis();
		subsbat.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				try {
					System.out.println("Status: "+message.getData()+" elapsed:"+(System.currentTimeMillis() - time)+" Count:"+numberDetected+" minimum:"+TIME+" Date:"+LocalDateTime.now());
					//( (MINRANGE != -1 && MAXRANGE != -1) && 
					//	(Double.parseDouble(message.getData()) < MAXRANGE) && (Double.parseDouble(message.getData()) > MINRANGE) ) {
					if(TIME == -1 || (System.currentTimeMillis() - time) <= TIME) {
						if(numberDetected >= NUMBER) {
							System.out.println("ALERT! @ "+LocalDateTime.now());
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
							// alerted, restart number detections
							numberDetected = 0;
							time = System.currentTimeMillis();
						} else {
							++numberDetected; // in time range, increment detections
						}
					} else {
						numberDetected = 0; // out of time range, start count of detections over
						time = System.currentTimeMillis();
					}
				} catch (Throwable e) {
					e.printStackTrace();
				}
			}
		});

	}

}
