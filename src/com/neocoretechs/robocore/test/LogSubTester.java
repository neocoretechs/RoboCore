package com.neocoretechs.robocore.test;

import java.io.IOException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

import org.ros.Topics;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.relatrix.Relation;
import com.neocoretechs.relatrix.client.asynch.AsynchRelatrixClientTransaction;
import com.neocoretechs.rocksack.TransactionId;


/**
 * This class receives log messages to check connectivity between nodes in a basic manner
 * @author jg
 *
 */
public class LogSubTester extends AbstractNodeMain  {
	private boolean DEBUG = true;
	private AsynchRelatrixClientTransaction rct;
	private TransactionId xid;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("logsub");
	}

	/**
	 *
	 */
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		Subscriber<rosgraph_msgs.Log> subslog = connectedNode.newSubscriber(Topics.ROSOUT, rosgraph_msgs.Log._TYPE);
		try {
			rct = connectedNode.getRelatrixClient();
			xid = rct.getTransactionId();
		} catch (IOException e) {
			e.printStackTrace();
		}
		subslog.addMessageListener(new MessageListener<rosgraph_msgs.Log>() {
			@Override
			public void onNewMessage(rosgraph_msgs.Log message) {
				System.out.println(message);
				CompletableFuture<Relation> r = rct.store(xid,System.currentTimeMillis(), message.getLine(),message.getMsg());
				try {
					r.get();
				} catch (InterruptedException | ExecutionException e) {
					e.printStackTrace();
				}
			}

		});

		connectedNode.executeCancellableLoop(new CancellableLoop() {
			@Override
			protected void setup() {
			}

			@Override
			protected void loop() throws InterruptedException {	
				//Thread.sleep(100);		
			}
		});
		
	}
	
}
