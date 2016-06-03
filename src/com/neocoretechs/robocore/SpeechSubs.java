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
 * Listen for status messages on the 'status' channel and translate to speech
 * @author jg
 *
 */
public class SpeechSubs extends AbstractNodeMain {
	private static boolean DEBUG = false;
	private static boolean speak = true;
	public static VoxHumana speaker = null;
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_status");
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
			try
			{
				System.out.println("Status "+message.getMessage());
				if( speak ) {
					StringBuilder sb = new StringBuilder();
					sb.append(message.getMessage()+"\r\n");
					List<KeyValue> diagMsgs = message.getValues();
					if( diagMsgs != null ) {
						for( KeyValue msg : diagMsgs) {
							sb.append(msg.getKey()+"\r\n");
							if( msg.getValue() != null ) {
								sb.append(msg.getValue()+"\r\n");
							}
						}
					}
					speaker.doSpeak(sb.toString());
				}
			}
			catch (Throwable e)
			{
				e.printStackTrace();
			}
		}

		});
		
	}

}
