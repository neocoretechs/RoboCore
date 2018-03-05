package com.neocoretechs.robocore.test;

//import org.apache.commons.logging.Log;
import java.io.File;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CountDownLatch;

import org.ros.concurrent.CancellableLoop;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Subscriber;


/**
 * Test reception of motor control data
 * @author jg Copyright (C) NeoCoreTechs 2017,2018
 *
 */
public class MotionController extends AbstractNodeMain {
	private static boolean DEBUG = false;
	private String host;
	private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);
	private int sequenceNumber;



	/**
	 * We really only use these methods if we want to pull remapped params out of command line or do
	 * some special binding, otherwise the default uses the ROS_HOSTNAME environment or the remapped __ip:= and __master:=
	 * @param host
	 * @param master
	 */
	public MotionController(String host, InetSocketAddress master) {
		this.host = host;
		this.master = master;
		NodeConfiguration nc = build();
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, nc);
	    if( DEBUG ) {
	    	System.out.println("Bringing up MotionControl with host and master:"+host+" "+master);
	    }
	    try {
			awaitStart.await();
		    if( DEBUG )
		    	System.out.println("Pub/subs registered..");
		} catch (InterruptedException e) {}
	    
	}
	
	public MotionController(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	    if( DEBUG ) {
	    	System.out.println("Bringing up MotionControl with args:"+args[0]+" "+args[1]+" "+args[2]);
	    }
	    try {
			awaitStart.await();
		    if( DEBUG )
		    	System.out.println("Pub/subs registered..");
		} catch (InterruptedException e) {}
	}
	
	public MotionController() { }
	
	/**
	 * Create NodeConfiguration 
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
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_joytest");
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		//final Log log = connectedNode.getLog();
		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();

		final Subscriber<std_msgs.Int32MultiArray> velsub =
				connectedNode.newSubscriber("absolute/cmd_vel", std_msgs.Int32MultiArray._TYPE);
		
		
		velsub.addMessageListener(new MessageListener<std_msgs.Int32MultiArray>() {
			@Override
			public void onNewMessage(std_msgs.Int32MultiArray message) {
				int[] dat = message.getData();
				System.out.println(++sequenceNumber+": L="+dat[0]+" R="+dat[1]);
			}
		});
	
		
		// tell the waiting constructors that we have registered publishers
		awaitStart.countDown();
		//
		// Begin publishing loop
		//
		connectedNode.executeCancellableLoop(new CancellableLoop() {

			@Override
			protected void setup() {
				sequenceNumber = 0;
			}

			@Override
			protected void loop() throws InterruptedException {
				Thread.sleep(100);
		}
	}); // cancellable loop

	}
	/**
	 * Move the buffered values into the publishing message to send absolute vals to motor control
	 * @param connectedNode
	 * @return
	 */
	private std_msgs.Int32MultiArray setupPub(ConnectedNode connectedNode, ArrayList<Integer> valBuf) {
		std_msgs.Int32MultiArray val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Int32MultiArray._TYPE);
		std_msgs.MultiArrayLayout vlayout = new std_msgs.MultiArrayLayout();
		List<std_msgs.MultiArrayDimension> vdim = new ArrayList<std_msgs.MultiArrayDimension>();
		std_msgs.MultiArrayDimension vdim1 = new std_msgs.MultiArrayDimension();
		std_msgs.MultiArrayDimension vdim2 = new std_msgs.MultiArrayDimension();
		vdim1.setLabel("Motor Channel");
		vdim2.setLabel("Motor Channel value");
		vdim1.setSize(2);
		vdim2.setSize(valBuf.size()/2);
		vdim1.setStride(valBuf.size());
		vdim2.setStride(2);
		vdim.add(vdim1);
		vdim.add(vdim2);
		vlayout.setDim(vdim);
		val.setLayout(vlayout);
		int[] vali = new int[valBuf.size()];
		int i = 0;
		for( Integer inv : valBuf)
			vali[i++] = inv;
		val.setData(vali);
		return val;
	}
	

}
