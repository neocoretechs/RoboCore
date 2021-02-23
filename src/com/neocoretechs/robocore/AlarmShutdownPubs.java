package com.neocoretechs.robocore;

import java.io.File;
import java.net.InetSocketAddress;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.CountDownLatch;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;
import org.ros.internal.loader.CommandLineLoader;


/**
 * Publishes global alarm shutdown messages, for those nodes which may currently be alarmed.
 * Sends a time marker to the alarm/shutdown topic.
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 */
public class AlarmShutdownPubs extends AbstractNodeMain  {
	private static final boolean DEBUG = false;
	private String host;
	private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);

	
	public AlarmShutdownPubs(String host, InetSocketAddress master) {
		this.host = host;
		this.master = master;
		NodeConfiguration nc = build();
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, nc);
	    
	}
	
	public AlarmShutdownPubs(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	}
	
	public AlarmShutdownPubs() {}
	
	public GraphName getDefaultNodeName() {
		return GraphName.of("pubs_alarmshutdown");
	}

	/**
	 * Create NodeConfiguration 
	 * @throws URISyntaxException 
	 */
	public NodeConfiguration build()  {
		//NodeConfiguration nodeConfiguration = NodeConfiguration.copyOf(Core.getInstance().nodeConfiguration);
		NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(host, master);
		nodeConfiguration.setParentResolver(NameResolver.newFromNamespace("/"));
		nodeConfiguration.setRosRoot(null); // currently unused
		nodeConfiguration.setRosPackagePath(new ArrayList<File>());
		nodeConfiguration.setMasterUri(master);
		nodeConfiguration.setNodeName(getDefaultNodeName());
		return nodeConfiguration;
	}


	@Override
	public void onStart(final ConnectedNode connectedNode) {
	
	//final RosoutLogger log = (Log) connectedNode.getLog();
	
		final Publisher<std_msgs.Empty> alarmpub =
				connectedNode.newPublisher("alarm/shutdown", std_msgs.Time._TYPE);
		
		// tell the waiting constructors that we have registered publishers
		awaitStart.countDown();
		
		connectedNode.executeCancellableLoop(new CancellableLoop() {
	
			@Override
			protected void setup() {
			}

			@Override
			protected void loop() throws InterruptedException {
			    try {
					awaitStart.await();
				} catch (InterruptedException e) {}
				std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Time._TYPE);
				alarmpub.publish(val);
				cancel();
			}
		}); 
	}

}
