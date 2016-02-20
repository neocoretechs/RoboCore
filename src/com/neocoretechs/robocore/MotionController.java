package com.neocoretechs.robocore;


//import org.apache.commons.logging.Log;
import java.util.List;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;


/**
 * This class is used as a ROS subscriber to the ardrone through messages on ardrone/range and ardrone/navdata
 * Sensor data is fused and decisions made about motion are publish as TWIST directives on the cmd_vel topic.
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
		
		//final sensor_msgs.Imu imumesg = connectedNode.getTopicMessageFactory().newFromType(sensor_msgs.Imu._TYPE);
		
		final geometry_msgs.Twist twistmsg = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Twist._TYPE);
		final geometry_msgs.Vector3 angular = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
		final geometry_msgs.Vector3 linear = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
		final geometry_msgs.Quaternion orientation =  connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE); 
		
		Subscriber<sensor_msgs.Imu> subsimu = connectedNode.newSubscriber("ardrone/navdata", sensor_msgs.Imu._TYPE);
		Subscriber<sensor_msgs.PointCloud> subsrange = connectedNode.newSubscriber("ardrone/range", sensor_msgs.PointCloud._TYPE);
		
		subsimu.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
		@Override
		public void onNewMessage(sensor_msgs.Imu message) {
			try
			{
				angular.setX(message.getAngularVelocity().getX());
				angular.setY(message.getAngularVelocity().getY());
				angular.setZ(message.getAngularVelocity().getZ());
				linear.setX(message.getLinearAcceleration().getX());
				linear.setY(message.getLinearAcceleration().getY());
				linear.setZ(message.getLinearAcceleration().getZ());
				orientation.setX(message.getOrientation().getX());
				orientation.setY(message.getOrientation().getY());
				orientation.setZ(message.getOrientation().getZ());
				orientation.setW(message.getOrientation().getW());
			}
			catch (Throwable e)
			{
				e.printStackTrace();
			}
		}

		});
		
		subsrange.addMessageListener(new MessageListener<sensor_msgs.PointCloud>() {
		@Override
		public void onNewMessage(sensor_msgs.PointCloud message) {
			try
			{
				List<geometry_msgs.Point32> ranges = message.getPoints();
			}
			catch (Throwable e)
			{
				e.printStackTrace();
			}
		}

		});
		
		connectedNode.executeCancellableLoop(new CancellableLoop() {
			private int sequenceNumber;

			@Override
			protected void setup() {
				sequenceNumber = 0;
			}

			@Override
			protected void loop() throws InterruptedException {
				/*
				NavPacket np = data.pop();
				int linearMove = 0;
				int courseOffset = 0;
				// we have x,y theta where x is frame x and y is distance to object
				if( np.isVision() ) {
					int range = (int)np.getVisionDistance();
					System.out.println("Range: "+range);
					if( range < 200 ) {
						courseOffset = 0; // stop
						System.out.println("Course offset zero");
					} else {
						int x = (int) np.getVisionX();
						if( x > 500 ) {
							x -=500;
						} else {
							x = 500 - x;
							x = -x; // turn left
						}
						// div by 10 to scale 500 to 0-5 degrees off center turn
						x /= 100;
						courseOffset = x;
						linearMove = 50;
						if( courseOffset != 0 ) linearMove = 0; // turn in place
						System.out.println("Robot offset course with "+ courseOffset +" "+linearMove);
					}
				} else {
					// no vision markers to influence the move
					courseOffset = np.getTargetYaw();
					linearMove = np.getTargetPitch();
					System.out.println("Robot normal course with "+ courseOffset +" "+linearMove);
				}
				hasMoved = motorControlListener.move2DRelative(np.getGyros()[0] , courseOffset, linearMove
						, np.getTimeVal(), np.getAccs(), np.getRanges());
					//System.out.println("Robot should have Moved to "+(robotTheta *= 57.2957795)+" degrees"); // to degrees		
			}
			*/
				// creep forward
				// value linear and angular may have current IMU vals
				linear.setX(50.0);linear.setY(50.0);linear.setZ(50.0);
				twistmsg.setLinear(linear);
				angular.setX(0.0); angular.setY(0.0); angular.setZ(0.0);
				twistmsg.setAngular(angular);
				System.out.println("Motion control pub move forward");
				mopub.publish(twistmsg);
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
