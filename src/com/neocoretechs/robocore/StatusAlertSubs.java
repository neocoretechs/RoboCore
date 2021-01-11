package com.neocoretechs.robocore;

import java.util.List;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.talker.VoxHumana;

import diagnostic_msgs.DiagnosticStatus;
import diagnostic_msgs.KeyValue;

/**
 * Listen for status messages on the 'status' channel and translate to speech or display on console.
 * @author Jonathan Groff (C) NeocoreTechs 2020
 *
 */
public class StatusAlertSubs extends AbstractNodeMain {
	private static boolean DEBUG = false;
	private static boolean speak = false;
	public static VoxHumana speaker = null;
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_statusalerts");
	}
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		if( speak ) {
			speaker = VoxHumana.getInstance();
		}
		//final Log log = connectedNode.getLog();
		Subscriber<diagnostic_msgs.DiagnosticStatus> subsbat = connectedNode.newSubscriber("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);
	
		subsbat.addMessageListener(new MessageListener<diagnostic_msgs.DiagnosticStatus>() {
			@Override
			public void onNewMessage(DiagnosticStatus message) {
				//System.out.println(message.getHardwareId()+" Status "+message.getMessage());
				StringBuilder sb = new StringBuilder();
				sb.append(message.getMessage()+"\r\n");
				sb.append(message.getHardwareId()+" ");
				List<KeyValue> diagMsgs = message.getValues();
				if( diagMsgs != null ) {
						for( KeyValue msg : diagMsgs) {
							sb.append(msg.getKey()+" ");
							if( msg.getValue() != null ) {
								sb.append(msg.getValue()+"\r\n");
							}
						}
						System.out.println(sb.toString());
				}
				try {
					if( speak ) {
						speaker.doSpeak(sb.toString());
					}
				} catch(Throwable e) {			
					e.printStackTrace();
				}

			}
		});
		
	}

}
