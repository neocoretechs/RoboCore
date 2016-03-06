package com.neocoretechs.robocore;

import java.util.List;
import java.util.Map;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;


/**
 * TwistSubs listen on cmd_vel topic for TWIST messages and relay that to motor control 
 * @author jg
 *
 */
public class TwistSubs extends AbstractNodeMain {
	private static boolean DEBUG = true;
	private String motorControlHost = null;
	NavListenerMotorControlInterface navListener = null;
	private boolean isMoving;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_twist");
	}
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		/*
		environment = System.getenv();
		if( environment.containsKey("MOTORCONTROL") ) {
			motorControlHost = environment.get("MOTORCONTROL");
			System.out.println("Establishing motor control host as "+motorControlHost);
		} else {
			motorControlHost = "127.0.0.1";
			System.out.println("Motor control host DEFAULTS TO LOOPBACK");
		}
		*/
		// Look for the __motorcontrol:=host on the special remappings of the command line
		Map<String,String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		if( remaps.get("__motorcontrol") != null )
			motorControlHost = remaps.get("__motorcontrol");
		else
			motorControlHost = "127.0.0.1";
		
		System.out.println("Motor control host set to "+motorControlHost);	
		
		//final Log log = connectedNode.getLog();
		Subscriber<geometry_msgs.Twist> substwist = connectedNode.newSubscriber("cmd_vel", geometry_msgs.Twist._TYPE);
		Subscriber<sensor_msgs.PointCloud> subsrange = connectedNode.newSubscriber("ardrone/range", sensor_msgs.PointCloud._TYPE);

		subsrange.addMessageListener(new MessageListener<sensor_msgs.PointCloud>() {
			@Override
			public void onNewMessage(sensor_msgs.PointCloud message) {
				List<geometry_msgs.Point32> range = message.getPoints();
				if( DEBUG )
					System.out.println("Floor Range "+range);
				try {
			
				} catch (Throwable e) {
					e.printStackTrace();
				}	
			}
		});
		
		/**
		 * Extract the linear and angular components from cmd_vel topic Twist quaternion, take the linear X (pitch) and
		 * angular Z (yaw) and send them to motor control. This results in motion planning computing a turn which
		 * involves rotation about a point in space located at a distance related to speed and angular velocity. The distance
		 * is used to compute the radius of the arc segment traversed by the wheel track to make the turn. If not moving we can
		 * make the distance 0 and rotate about a point in space, otherwise we must inscribe an appropriate arc. The distance
		 * in that case is the linear travel, otherwise the distance is the diameter of the arc segment.
		 * If we get commands on the cmd_vel topic we assume we are moving, if we do not get the corresponding IMU readings, we have a problem
		 * If we get a 0,0 on the X,yaw move we stop. If we dont see stable IMU again we have a problem, Houston.
		 */
		substwist.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
		@Override
		public void onNewMessage(geometry_msgs.Twist message) {
			geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
			val = message.getLinear();
			float targetPitch = (float) val.getX();
			val = message.getAngular();
			float targetYaw = (float) val.getZ();

			if( targetPitch == 0.0 && targetYaw == 0.0 )
					isMoving = false;
			else
					isMoving = true;
	
			System.out.println("Robot commanded to move:" + targetPitch + "mm linear in orientation " + targetYaw+" euler:"+targetPitch);
			//log.debug("Robot commanded to move:" + targetPitch + "mm linear in orientation " + targetYaw);
		}
		});
	}

}
