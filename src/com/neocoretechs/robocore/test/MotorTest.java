package com.neocoretechs.robocore.test;

import java.io.IOException;
import java.util.Arrays;
import java.util.Map;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeMainExecutor;
import org.ros.node.parameter.ParameterTree;

import com.neocoretechs.robocore.config.Robot;
import com.neocoretechs.robocore.config.RobotInterface;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.MarlinspikeManager;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;

public class MotorTest extends AbstractNodeMain {
	private static boolean DEBUG = true;
	private RobotInterface robot;
	private String robotName;

	public MotorTest(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		//String mode="";
		//if( remaps.containsKey("__mode") )
		//	mode = remaps.get("__mode");
		//if( mode.equals("startup")) {

		if(remaps.containsKey("__robot") ) {
			robotName = remaps.get("__robot");
		} else {
			throw new RuntimeException("Must specify __robot:=<name> to configure parameters.");
		}
		if(DEBUG)
			System.out.printf("Robot reports host name as %s%n",robotName);
		ParameterTree pTree = connectedNode.getParameterTree();
		robot = (RobotInterface) pTree.get(robotName, null);
		if(robot == null)
			throw new RuntimeException("Could not fetch parameters for robot name:"+robotName+". Must start MotionController first.");
		MarlinspikeManager mm = new MarlinspikeManager(robot);
		AsynchDemuxer ad = new AsynchDemuxer(mm);
		try {
			ad.connect(new ByteSerialDataPort());
		} catch (IOException e) {
			e.printStackTrace();
		}
		String motorCommand = "M0"; // turn off realtime output
		AsynchDemuxer.addWrite(ad,motorCommand);
		// config smart controller channel 1 default direction
		motorCommand = "M2 C1 E0";
		AsynchDemuxer.addWrite(ad,motorCommand);
		// config smart controller channel 2, invert direction
		motorCommand = "M2 C2 E1"; 
		AsynchDemuxer.addWrite(ad,motorCommand);
		motorCommand = "M6 S10"; //scale by 10 to slow default speed
		AsynchDemuxer.addWrite(ad,motorCommand);
		motorCommand = "M33 P22 D60 E0"; //link the ultrasonic sensor pointing forward(E0) on pin 22 to stop motor at distance 60cm
		AsynchDemuxer.addWrite(ad,motorCommand);
		for(int i = 0 ; i < 5000; i++) {
			motorCommand = "G5 C1 P500"; // half power channel 1 / 10
			AsynchDemuxer.addWrite(ad,motorCommand);
			motorCommand = "G5 C2 P500"; // half power channel 1 / 10
			AsynchDemuxer.addWrite(ad,motorCommand);
			System.out.println("Forward "+i);
			motorCommand = "G5 C1 P0"; // stop
			AsynchDemuxer.addWrite(ad,motorCommand);
			motorCommand = "G5 C2 P0"; // stop
			AsynchDemuxer.addWrite(ad,motorCommand);
			System.out.println("stop1 "+i);
			//Thread.sleep(100);
			motorCommand = "G5 C1 P-500"; // half power channel 1 / 10 reverse
			AsynchDemuxer.addWrite(ad,motorCommand);
			//Thread.sleep(100);
			motorCommand = "G5 C2 P-500"; // half power channel 1 / 10 reverse
			AsynchDemuxer.addWrite(ad,motorCommand);
			System.out.println("Reverse "+i);
			motorCommand = "G5 C1 P0"; // stop
			AsynchDemuxer.addWrite(ad,motorCommand);
			motorCommand = "G5 C2 P0"; // stop
			AsynchDemuxer.addWrite(ad,motorCommand);
			System.out.println("stop2 "+i);
		}
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("motortest");
	}
}
