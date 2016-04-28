/**
 * Publish the data acquired from the Mega board through the serial interface. Motor controller, ultrasonic sensor
 * voltage, etc and all that is acquired from the attached USB of an aux board such as Mega2560 via RS-232
 * @author jg
 */
package com.neocoretechs.robocore;

import java.io.File;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
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



public class TwistPubs extends AbstractNodeMain  {
	private static final boolean DEBUG = true;
	float volts;
	Object statMutex = new Object(); 
	Object navMutex = new Object();
	private String host;
	private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);
	private static int[] data = new int[]{-10,10,100};
	
	public TwistPubs(String host, InetSocketAddress master) {
		this.host = host;
		this.master = master;
		NodeConfiguration nc = build();
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, nc);
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	    
	}
	
	public TwistPubs(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	}
	
	public TwistPubs() {}
	
	public GraphName getDefaultNodeName() {
		return GraphName.of("pubs_twist");
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

	final Publisher<geometry_msgs.Twist> twistpub =
		connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);

	// This CancellableLoop will be canceled automatically when the node shuts
	// down.
	connectedNode.executeCancellableLoop(new CancellableLoop() {
		private int sequenceNumber;
		@Override
		protected void setup() {
			sequenceNumber = 0;
		}

		@Override
		protected void loop() throws InterruptedException {
			synchronized(data) {
				geometry_msgs.Twist twistmsg = twistpub.newMessage();
				geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
				val.setX(45);
				val.setY(100);
				twistmsg.setLinear(val);
				//int targetPitch = (int) val.getX();
				//int targetDist = (int) val.getY();
				geometry_msgs.Vector3 vala = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
				vala.setZ(.5f);
				//float targetYaw = (float) val.getZ();
				twistmsg.setAngular(vala);
				twistpub.publish(twistmsg);
				Thread.sleep(10);
				val.setX(225);
				twistmsg.setLinear(val);
				twistpub.publish(twistmsg);
				//mc.moveRobotRelative(.5f, 45, 100);
				//mc.moveRobotRelative(-.25f, 45, 100);
				//mc.moveRobotRelative(.25f, 45, 100);
				//mc.moveRobotRelative(-.25f, 225, 100);
				//mc.moveRobotRelative(.0f, 0, 100);
				//val = message.getLinear();
				//int targetPitch = (int) val.getX();
				//int targetDist = (int) val.getY();
				//val = message.getAngular();
				//float targetYaw = (float) val.getZ();
				//motorControlHost.moveRobotRelative(targetYaw, targetPitch, targetDist);
				
			}
	
			Thread.sleep(10);
		}
	});  


}

}
