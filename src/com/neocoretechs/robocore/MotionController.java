package com.neocoretechs.robocore;

import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

/**
 * This class is used as a ROS publisher to the ardrone through messages on cmd_vel topic.
 * Twist messages are constructed and sent to cmd_vel topic where they ultimately activate motor control
 * through the process that manages serial data to the motor controller.
 * All sensor data should be available here including accel and US ranges should we choose to subscribe to them.
 * We subscribe to the ardrone/vision topic to get messages about detecting tags so we can stop if we see one
 * @author jg
 *
 */
public class MotionController extends AbstractNodeMain {
	//private static MotionController instance = null;
	//private MotionController(){}
	//public static MotionController getInstance() {
	//	if( instance == null ) {
	//		instance = new MotionController();
	//		mc = new MotorControl();
	//	}
	//	return instance;
	//}
	public MotionController() { }
	boolean hasData = false; // have we received any feedback from callback?
	boolean init = true;

	Object mutex = new Object();
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("motion_control");
	}
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		//final Log log = connectedNode.getLog();
		final Publisher<geometry_msgs.Twist> mopub = connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);
		
		final geometry_msgs.Twist tmesg = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Twist._TYPE);
		final geometry_msgs.Vector3 linear = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
		final geometry_msgs.Vector3 angular = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
		
		connectedNode.executeCancellableLoop(new CancellableLoop() {
			private int sequenceNumber;

			@Override
			protected void setup() {
				sequenceNumber = 0;
			}

			@Override
			protected void loop() throws InterruptedException {
					// creep forward
					linear.setX(50.0d);
					linear.setY(0.0d);
					linear.setZ(0.0d);
					tmesg.setLinear(linear);
					angular.setX(0.0d);
					angular.setY(0.0d);
					angular.setZ(0.0d);
					tmesg.setAngular(angular);
					System.out.println("Motion control pub move forward");
					mopub.publish(tmesg);
				//float targetRoll = (float) val.getY();
				//float targetPitch = (float) val.getX();
				//float targetVertvel = (float) val.getZ();
				//val = message.getAngular();
				//float targetYaw = (float) val.getZ();
				
				// no data yet ,send a command stop down the line to set up data callback stream
				++sequenceNumber;
				Thread.sleep(500);
		}
	}); // cancellable loop

	}
}
