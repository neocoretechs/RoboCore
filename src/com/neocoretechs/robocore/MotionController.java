package com.neocoretechs.robocore;

//import org.apache.commons.logging.Log;
import java.util.ArrayList;
import java.util.List;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;


/**
 * This class is used as a ROS subscriber to the ardrone through messages on range/ultrasonic/ardrone and ardrone/navdata
 * Sensor data is fused and decisions made about motion are publish as TWIST directives on the cmd_vel topic.
 * Twist messages are constructed and sent to cmd_vel topic where they ultimately activate motor control
 * through the process that manages serial data to the motor controller.
 * All sensor data should be available here including accel and US ranges should we choose to subscribe to them.
 * We subscribe to the ardrone/vision topic to get messages about detecting tags so we can stop if we see one
 * @author jg
 *
 */
public class MotionController extends AbstractNodeMain {
	private static boolean DEBUG = true;
	//private static MotionController instance = null;
	//private MotionController(){}
	//public static MotionController getInstance() {
	//	if( instance == null ) {
	//		instance = new MotionController();
	//		mc = new MotorControl();
	//	}
	//	return instance;
	//}
	float rangetop, rangebot;
	double pressure, temperature;
	double[] mag = new double[3];
	
	public static boolean isShock = false;
	// deltas. 971, 136, 36 relatively normal values. seismic: last value swings from rangeTop -40 to 140
	public static float[] SHOCK_BASELINE = { 971.0f, 136.0f, 36.0f};

	public static float[] SHOCK_THRESHOLD = {1000.0f,1000.0f,1000.0f}; 
	public static boolean isMag = false;
	public static short[] MAG_THRESHOLD = {-1,-1,-1};
	//public static boolean isMag = false;
	public static int PRESSURE_THRESHOLD = 100000; // pressure_meas is millibars*100 30in is 1014 milli
	public static boolean isPressure = false;
	public static long lastPressureNotification = 0; // time so we dont just keep yapping about the weather
	public static int TEMPERATURE_THRESHOLD = 50000; // C*1000 122F
	public static boolean isTemperature = false;
	public static boolean isMoving = false;
	public static boolean isVision = false; // machine vision recognition event
	
	boolean hasData = false; // have we received any feedback from callback?
	boolean init = true;

	Object rngMutex1 = new Object();
	Object rngMutex2 = new Object();
	Object navMutex = new Object();
	Object magMutex = new Object();
	
	public MotionController() { }
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("motion_control");
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		//final Log log = connectedNode.getLog();
		final Publisher<geometry_msgs.Twist> mopub = connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);
		// statpub has status alerts that may come from ARDrone extreme attitude, temp etc.
		final Publisher<diagnostic_msgs.DiagnosticStatus> statpub =
				connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);
		//final sensor_msgs.Imu imumesg = connectedNode.getTopicMessageFactory().newFromType(sensor_msgs.Imu._TYPE);
		
		final geometry_msgs.Twist twistmsg = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Twist._TYPE);
		final geometry_msgs.Vector3 angular = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
		final geometry_msgs.Vector3 linear = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
		final geometry_msgs.Quaternion orientation =  connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE); 
		
		Subscriber<sensor_msgs.Imu> subsimu = connectedNode.newSubscriber("ardrone/navdata", sensor_msgs.Imu._TYPE);
		Subscriber<sensor_msgs.Range> subsrangetop = connectedNode.newSubscriber("range/ultrasonic/ardrone", sensor_msgs.Range._TYPE);
		Subscriber<sensor_msgs.Range> subsrangebot = connectedNode.newSubscriber("range/ultrasonic/robocore", sensor_msgs.Range._TYPE);
		Subscriber<sensor_msgs.MagneticField> subsmag = connectedNode.newSubscriber("ardrone/magnetic_field", sensor_msgs.MagneticField._TYPE);
		Subscriber<sensor_msgs.Temperature> substemp = connectedNode.newSubscriber("ardrone/temperature", sensor_msgs.Temperature._TYPE);
		Subscriber<sensor_msgs.FluidPressure> subspress = connectedNode.newSubscriber("ardrone/pressure", sensor_msgs.FluidPressure._TYPE);
		Subscriber<geometry_msgs.Quaternion> substag = connectedNode.newSubscriber("ardrone/image_tag", geometry_msgs.Quaternion._TYPE);
		Subscriber<sensor_msgs.PointCloud> subsrange = connectedNode.newSubscriber("robocore/kinect", sensor_msgs.PointCloud._TYPE);

		subsrange.addMessageListener(new MessageListener<sensor_msgs.PointCloud>() {
			@Override
			public void onNewMessage(sensor_msgs.PointCloud message) {
				List<geometry_msgs.Point32> cloud = message.getPoints();
				if( DEBUG )
					System.out.println("Kinect.."+cloud.size()+" points..");
				try {
			
				} catch (Throwable e) {
					e.printStackTrace();
				}	
			}
		});
		
		subsimu.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
		@Override
		public void onNewMessage(sensor_msgs.Imu message) {
			synchronized(navMutex) {
				try {
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
				} catch (Throwable e) {
					System.out.println("Nav subs exception:"+e.getMessage());
					e.printStackTrace();
				}
			}
		}

		});
		
		subsrangetop.addMessageListener(new MessageListener<sensor_msgs.Range>() {
		@Override
		public void onNewMessage(sensor_msgs.Range message) {
			synchronized(rngMutex1) {
				try {
					rangetop = message.getRange();
				} catch (Throwable e) {
					System.out.println("Range top subs exception:"+e.getMessage());
					e.printStackTrace();
				}
			}
		}
		});

		subsrangebot.addMessageListener(new MessageListener<sensor_msgs.Range>() {
		@Override
		public void onNewMessage(sensor_msgs.Range message) {
			synchronized(rngMutex2) {
				try {
					rangebot = message.getRange();
				} catch (Throwable e) {
					System.out.println("Range bottom subs exception:"+e.getMessage());
					e.printStackTrace();
				}
			}
		}
		});
		
		subsmag.addMessageListener(new MessageListener<sensor_msgs.MagneticField>() {
		@Override
		public void onNewMessage(sensor_msgs.MagneticField message) {
			geometry_msgs.Vector3 mag3 = message.getMagneticField();
			synchronized(magMutex) {
				mag[0] = mag3.getX();
				mag[1] = mag3.getY();
				mag[2] = mag3.getZ();
				if( DEBUG ) {
					System.out.println("Mag:"+mag[0]+","+mag[1]+","+mag[2]);
				}
				if( MAG_THRESHOLD[0] != -1 ) {
					if( mag[0] > MAG_THRESHOLD[0] && mag[1] > MAG_THRESHOLD[1] && mag[2] > MAG_THRESHOLD[2] ) {
						isMag = true;
					}
				}
			}
		}
		});
		
		subspress.addMessageListener(new MessageListener<sensor_msgs.FluidPressure>() {
			@Override
			public void onNewMessage(sensor_msgs.FluidPressure message) {
				pressure = message.getFluidPressure();
				isPressure = true;
				if( DEBUG ) 
					System.out.println("Pressure: "+pressure);
			}
		});
		
		substemp.addMessageListener(new MessageListener<sensor_msgs.Temperature>() {
			@Override
			public void onNewMessage(sensor_msgs.Temperature message) {
				temperature = message.getTemperature();
				isTemperature = true;
				if( DEBUG )
					System.out.println("Temp:"+temperature);
			}
		});
		
		substag.addMessageListener(new MessageListener<geometry_msgs.Quaternion>() {
			@Override
			public void onNewMessage(geometry_msgs.Quaternion message) {
				System.out.println("IMAGE TAG AT:"+ message.getX()+","+message.getY()+","+message.getZ());
			}
		});
		
		//
		// Begin publishing loop
		//
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
				//linear.setX(50.0);linear.setY(50.0);linear.setZ(50.0);
				//twistmsg.setLinear(linear);
				//angular.setX(0.0); angular.setY(0.0); angular.setZ(0.0);
				//twistmsg.setAngular(angular);
				//System.out.println("Motion control pub move forward");
				//mopub.publish(twistmsg);
				//float targetRoll = (float) val.getY();
				//float targetPitch = (float) val.getX();
				//float targetVertvel = (float) val.getZ();
				//val = message.getAngular();
				//float targetYaw = (float) val.getZ();
				System.out.println("Seq: "+sequenceNumber);
				
				synchronized(rngMutex1) {
					System.out.println("Range Top:"+rangetop);
				}
				synchronized(rngMutex2) {
					System.out.println("Range Bottom:"+rangebot);
				}
	
				synchronized(navMutex) {
					System.out.println();
					System.out.println("Nav:");
					System.out.println("Angular X:"+angular.getX());
					System.out.println("Angular Y:"+angular.getY());
					System.out.println("Angular Z:"+angular.getZ());
					System.out.println("Linear X:"+linear.getX());
					System.out.println("Linear Y:"+linear.getY());
					System.out.println("Linear Z:"+linear.getZ());
					System.out.println("Orientation X:"+orientation.getX());
					System.out.println("Orientation Y:"+orientation.getY());
					System.out.println("Orientation Z:"+orientation.getZ());
					System.out.println("Orientation W:"+orientation.getW());
					//System.out.println("Shock:"+accs[0]+" "+accs[1]+" "+accs[2]);
					if( SHOCK_THRESHOLD[0] != -1 ) {
						if( Math.abs(linear.getX()-SHOCK_BASELINE[0]) > SHOCK_THRESHOLD[0] ) {
							isShock = true;
							
						}
						if( Math.abs(linear.getY()-SHOCK_BASELINE[1]) > SHOCK_THRESHOLD[1] ) {
							isShock = true;
							
						}
						if( Math.abs(linear.getZ()-SHOCK_BASELINE[2]) > SHOCK_THRESHOLD[2] ) {
							isShock = true;
							
						}
					}
				}
				
				// publish messages to status listener if applicable
				
				if( isMag && !isMoving) {
					isMag = false;
					diagnostic_msgs.DiagnosticStatus statmsg = statpub.newMessage();
					statmsg.setName("magnetic");
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
					statmsg.setMessage("Magnetic anomaly detected");
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					statpub.publish(statmsg);
					Thread.sleep(1);
				}
				
				if( isShock && !isMoving) {
					diagnostic_msgs.DiagnosticStatus statmsg = statpub.newMessage();
					isShock = false;
					statmsg.setName("shock");
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
					statmsg.setMessage("Accelerometer shock warning");
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					statpub.publish(statmsg);
					Thread.sleep(1);
				}
				
					
				if( isPressure ) { // check for dropping
						diagnostic_msgs.DiagnosticStatus statmsg = statpub.newMessage();
							//long meas = System.currentTimeMillis()-lastPressureNotification;
							//if( meas > 1000000) {
							//	isPress = true;
							//}
							//return;
							//}
						isPressure = false;
						statmsg.setName("pressure");
						statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
						statmsg.setMessage("Atmospheric pressure warning");
						diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
						List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
						li.add(kv);
						statmsg.setValues(li);
						statpub.publish(statmsg);
						Thread.sleep(1);
				}
				
				if(isTemperature) {
						diagnostic_msgs.DiagnosticStatus statmsg = statpub.newMessage();
						isTemperature = false;
						statmsg.setName("temprature");
						statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
						statmsg.setMessage("Temperature warning");
						diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
						List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
						li.add(kv);
						statmsg.setValues(li);
						statpub.publish(statmsg);
						Thread.sleep(1);
				}
		
				++sequenceNumber;
				Thread.sleep(250);
		}
	}); // cancellable loop

	}
}
