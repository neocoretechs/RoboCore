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
import org.ros.message.Time;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.BatteryListener;
import com.neocoretechs.robocore.machine.bridge.MotorFaultListener;
import com.neocoretechs.robocore.machine.bridge.UltrasonicListener;


public class MegaPubs extends AbstractNodeMain  {
	private static final boolean DEBUG = true;
	float volts;
	Object statMutex = new Object(); 
	Object navMutex = new Object();
	private String host;
	private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);
	
	public MegaPubs(String host, InetSocketAddress master) {
		this.host = host;
		this.master = master;
		NodeConfiguration nc = build();
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, nc);
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	    
	}
	
	public MegaPubs(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	}
	
	public MegaPubs() {}
	
	public GraphName getDefaultNodeName() {
		return GraphName.of("robocore");
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

	final Publisher<diagnostic_msgs.DiagnosticStatus> statpub =
		connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);

	final Publisher<sensor_msgs.Range> rangepub = 
		connectedNode.newPublisher("robocore/range", sensor_msgs.Range._TYPE);
	// Start reading from serial port
	try {
		AsynchDemuxer.getInstance().config();
	} catch (IOException e) {
		System.out.println("Could not start process to read attached serial port.."+e);
		e.printStackTrace();
		return;
	}
	
	// tell the waiting constructors that we have registered publishers
	awaitStart.countDown();

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
			boolean upSeq = false;
			std_msgs.Header ihead = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);

			diagnostic_msgs.DiagnosticStatus statmsg = statpub.newMessage();
			sensor_msgs.Range rangemsg = rangepub.newMessage();

				if( !BatteryListener.data.isEmpty() ) {
					Float batt = BatteryListener.data.take();
					volts = batt.floatValue();
					statmsg.setName("battery");
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
					statmsg.setMessage("Battery voltage warning "+((int)volts)+" volts");
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					statpub.publish(statmsg);
					Thread.sleep(1);
					if( DEBUG) System.out.println("Published seq#"+sequenceNumber+" battery: "+statmsg.getMessage().toString());
				}		
	
				if( !UltrasonicListener.data.isEmpty() ) {
					Integer range = UltrasonicListener.data.take();
					ihead.setSeq(sequenceNumber);
					Time tst = connectedNode.getCurrentTime();
					ihead.setStamp(tst);
					ihead.setFrameId("0");
					rangemsg.setHeader(ihead);
					rangemsg.setFieldOfView(30);
					rangemsg.setMaxRange(600);
					rangemsg.setMinRange(6);
					rangemsg.setRadiationType(sensor_msgs.Range.ULTRASOUND);
					rangemsg.setRange(range.floatValue());
					rangepub.publish(rangemsg);
					Thread.sleep(1);
					if( DEBUG ) System.out.println("Published seq#"+sequenceNumber+" range: "+rangemsg.getRange());
					upSeq = true;
				}				
			
	
				if( !MotorFaultListener.data.isEmpty() ) {
					String mfd = MotorFaultListener.data.take();
					statmsg.setName("motor");
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.ERROR);
					statmsg.setMessage("Motor fault warning "+mfd);
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					statpub.publish(statmsg);
					if( DEBUG) System.out.println("Published seq#"+sequenceNumber+" motor fault: "+statmsg.getMessage().toString());
				}			
		
			if( upSeq )
				++sequenceNumber;
			Thread.sleep(1);
		}
	});  


}

}
