package com.neocoretechs.robocore.propulsion;

import java.io.IOException;

import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.SubmissionPublisher;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.ros.concurrent.CancellableLoop;

import java.util.stream.Collectors;

import org.ros.internal.loader.CommandLineLoader;
import org.ros.internal.node.topic.PublisherIdentifier;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;

import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeMainExecutor;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.CountDownServiceServerListener;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.SubscriberListener;

import com.neocoretechs.robocore.AuxGPIOControl;
import com.neocoretechs.robocore.RosArrayUtilities;
import com.neocoretechs.robocore.SynchronizedThreadManager;
import com.neocoretechs.robocore.PID.MotionPIDController;

import com.neocoretechs.robocore.config.Robot;
import com.neocoretechs.robocore.config.RobotInterface;
import com.neocoretechs.robocore.config.AxisInterface;
import com.neocoretechs.robocore.config.DeviceEntry;
import com.neocoretechs.robocore.config.POVAxis;
import com.neocoretechs.robocore.config.POVAxis.POVType;

import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;

import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.DeviceSubscriber;
import com.neocoretechs.robocore.marlinspike.MarlinspikeControl;
import com.neocoretechs.robocore.marlinspike.MarlinspikeControlInterface;
import com.neocoretechs.robocore.marlinspike.MarlinspikeManager;
import com.neocoretechs.robocore.marlinspike.PublishDiagnosticResponse;
import com.neocoretechs.robocore.marlinspike.PublishResponseInterface;

import com.neocoretechs.robocore.navigation.NavListenerMotorControlInterface;

import com.neocoretechs.robocore.services.ControllerStatusMessage;
import com.neocoretechs.robocore.services.ControllerStatusMessageRequest;
import com.neocoretechs.robocore.services.ControllerStatusMessageResponse;
import com.neocoretechs.robocore.services.GPIOControlMessage;
import com.neocoretechs.robocore.services.GPIOControlMessageRequest;
import com.neocoretechs.robocore.services.GPIOControlMessageResponse;
import com.neocoretechs.robocore.services.PWMControlMessage;
import com.neocoretechs.robocore.services.PWMControlMessageRequest;
import com.neocoretechs.robocore.services.PWMControlMessageResponse;

import diagnostic_msgs.DiagnosticStatus;

import org.json.JSONObject;

import sensor_msgs.Joy;

import trajectory_msgs.ComeToHeadingStamped;

/**
 * MotionController is the primary process we need to configure the other nodes via parameter tree.<p>
 * We are fusing data from the IMU, any joystick input, and other autonomous controls to send down to the Marlinspike controllers.
 * Subscribing to:
 * <pre>
 * The joystick message from the ps3 using jinput delivers the message with the following structure:<br>
 * float[]-axis
 * [0] - left horiz (X)						<br>
 * [1] - left vert (Y)						<br>
 * [2] - right horiz (X)						<br>
 * [3] - right vert (Y) 						<br>
 * [4] - left trigger						<br>
 * [5] - right trigger						<br>
 * [6] - POV									<br>
 * --int[] buttons--							<br>
 * [0] - select								<br>
 * [1] - start								<br>
 * [2] - left stick press					<br>
 * [3] - right stick press					<br>
 * [4] - triangle -- HOLD CURRENT BEARING	<br>
 * [5] - circle -- RIGHT PIVOT				<br>
 * [6] - X button -- EMERGENCY STOP			<br>
 * [7] - square -- LEFT PIVOT				<br>
 * [8] - left bumper							<br>
 * [9] - right bumper						<br>
 * -- joystick only --						<br>
 * [10] - back								<br>
 * [11] - extra								<br>
 * [12] - mode								<br>
 * [13] - side								<br><br>
 * </pre>
 * <pre>
 * sensor_msgs/Joy (joystick information boxed as an array of buttons and axis values common to standard controllers):
 * NYCO CORE CONTROLLER: <br>
 * Identifies as : USB Joystick<br>
 *Axis x		Left stick (left/right)<br>
 *Axis y		Left stick (up/down)<br>
 *Axis rx		N/A	 <br>
 *Axis ry		N/A	 <br>
 *Axis z	 	Right stick (left/right)<br>
 *Axis rz		Right stick (<br>
 *Button 1	circle - Thumb<br>
 *Button 2	square - Top<br>
 *Button 3	triangle - Trigger<br>
 *Button 4	Left Bumper	- Top2<br>
 *Button 5	Right Bumper - Pinkie<br>
 *Button 6	Select - Base3<br>
 *Button 7	Start - Base4<br>
 *Button 8	Left Stick Press - Base5<br>
 *Button 9	Right Stick Press - Base6<br>
 *<p>
 * AFTERGLOW CONTROLLER:<br>
 * Identifies as : Generic X-Box Pad (right, even though its a PS3!)<br>
 *Axis x		Left stick (left/right)<br>
 *Axis y		Left stick (up/down)<br>
 *Axis z	 	Left trigger<br>
 *Axis rx(rx)	Right stick (left/right)	<br> 
 *Axis ry(ry)	Right stick (up/down) <br>
 *Axis rz		Right trigger<br>
 *Button B(B) - circle<br>
 *Button X(X) - square<br>
 *Button A(A) - X<br>
 *Button Left Thumb - left bumper<br>
 *Button Right Thumb - right bumper<br>
 *Button Left Thumb 3 - left stick press<br>
 *Button Right Thumb 3 - right stick press<br>
 *Button Select - Select<br>
 *Button Mode - Mode<br>
 *Button Unknown - Home<br>
 *<p>
 * LOGITECH, INC F310 GAMEPAD<br>
 * Identifies as : Generic X-box pad (Controls are the same as AfterGlow) or Logitech Gamepad F310<br>
 *Axis x		Left stick (left/right)<br>
 *Axis y		Left stick (up/down)<br>
 *Axis z	 	Left trigger<br>
 *Axis rx(rx)	Right stick (left/right)	<br> 
 *Axis ry(ry)	Right stick (up/down) <br>
 *Axis rz		Right trigger<br>
 *Button B(B) - circle<br>
 *Button X(X) - square<br>
 *Button A(A) - X<br>
 *Button Y(Y) - triangle<br>
 *Button Left Thumb - left bumper<br>
 *Button Right Thumb - right bumper<br>
 *Button Left Thumb 3 - left stick press<br>
 *Button Right Thumb 3 - right stick press<br>
 *Button Select - Select<br>
 *Button Mode - Mode<br><br>
 *</pre>
 *
 * Process joystick messages. <p>
 * The buttons are basically hardwired to predefined functions, and LUN[0] and LUN[1] are typically assigned device names LeftWheel
 * and RightWheel and the 2 axes of stick specified in the config file are blended into a steering speed and angle based on the 2 axes,
 * which is published to each wheel channel as a speed in the range -1000 to +1000.<p>
 * 
 * The other device name LUNs can be configured as desired with the values interpolated into a -1000 to +1000 range based on the type
 * of axis defined and the values(s) published to the DeviceName LUN channel as a Ros int32 multi array.<p>
 *
 * Joystick data will have array of axis and buttons, typically, axis[0] and axis[2] are left stick x,y axis[1] and axis[3] are right.
 * The code calculates the theoretical speed for each wheel in the 0-1000 scale or SPEEDSCALE based on target point vs IMU yaw angle.
 * If the joystick chimes in the target point becomes the current course minus relative stick position,
 * and the speed is modified by the Y value of the stick.
 * Button presses cause rotation in place or emergency stop or cause the robot to hold to current course, using the IMU to 
 * correct for deviation an wheel slippage.<p>
 * 
 * To turn, we are going to calculate the arc lengths of the inner wheel and outer wheel based on speed we are presenting by stick y
 * (speed) forming the radius of the arc and the offset of stick angle x,y degrees from 0, added to current heading, forming theta.
 * Theta may also be formed by button press, and the difference in current heading and ongoing IMU headings for bearing on a forward course.
 * We are then going to assume that the distance each wheel has to travel represents a ratio that is also the ratio
 * of speeds of each wheel, time and speed being distance, and the fact that both wheels, being attached to the robot,
 * have to arrive at essentially the same time after covering the desired distance based on desired speed.<p>
 * 
 * The net effect that as speed increases the turning radius also increases, as the radius is formed by speed (Y of stick) scaled by 1000
 * in the case of the inner wheel, and inner wheel plus 'effective robot width' or WHEELBASE as the outer wheel arc radius to traverse.
 * So we have the theta formed by stick and IMU, and 2 radii formed by stick y and WHEELBASE, and we generate 2 arc lengths that are taken
 * as a ratio that is multiplied by the proper wheel depending on direction, to reduce power in one wheel, to affect turn.
 * The ratio of arc lengths depending on speed and turn angle becomes the ratio of speeds at each wheel.<p>
 * 
 * The above method is used for interactive control via joystick and for large granularity correction in autonomous mode.
 * The same technique is used in autonomous mode for finer correction by substituting the base of a right triangle as the speed 
 * for the inner arc, and the hypotenuse computed by the base and chord formed from course deviation, and half wheelbase, for the outer arc.
 * The triangle solution uses radius in the forward direction, rather than at right angles with WHEELBASE as the arcs do, to bring
 * us into refined tangents to the crosstrack. At final correction, a PID algorithm is used to maintain fine control.
 * axis[6] is POV, it has quantized values to represent its states.<p>
 * * Functions:<br>
 * Process data acquired from the Real-time telemetry from the Marlinspike realtime subsystem.
 * Respond to messages from bus as publisher and subscriber to device name in configuration and process via realtime subsystem.
 * <p>
 * Messages can relate to Motor control, controller status, ultrasonic sensor, voltage reading, etc, acquired from the attached UART/USB of an aux 
 * board such as Mega2560 via RS-232, Pico via USB, or from the SBC running the Ros node using the GPIO header and libgpiod, or from the USB
 * port of the SBC talking to a smart controller. The Marlinspike microcontroller code specifically uses the RPi RP2040 Pico via the USB port
 * to process commands from the {@link AsynchDemuxer} based on configuration generated from the RoboCore.properties properties file to configure and drive
 * various GPIO and H-Bridge PWM controller functions via M anf G codes such as those used in CNC and 3D printing.<p>
 * <p> 
 * Serial communications are centralized in the {@link com.neocoretechs.robocore.serialreader.DataPortInterface} and {@link com.neocoretechs.robocore.serialreader.DataPortCommandInterface} interface class implementors.<p>
 * 
 * As input from realtime subsystem is received, the {@link AsynchDemuxer} decodes the header and prepares the data for publication
 * to the Ros bus. It may also publish various warnings over the 'robocore/status' topic. It will also send motor controller and other configuration information
 * over the robocore/status topic.<p>
 * The MarlinSpikeManager is constructed with the static instance of (@link Robot}, then the configureMarlinSpike method is called.
 * We can then iterate the {@link DeviceEntry} from MarlinSpikeManager.getDevices().
 * for each DeviceEntry, create a reactive internal subscriber on the topic of type Int32MultiArray.
 * The topic name is formed from the name entry in the properties file, or designated by slot via Slot+slot, which should be unique across the bus.
 * Based on the derived topic name, get the MarlinspikeControlInterface from the MarlinspikeManager.
 * We can then set the power level etc on the device from the data in the message. When a new message comes in it gets
 * the topic name based on the subscriber hash map. It then gets the {@link MarlinspikeControlInterface} based on topic name. The
 * message has an array if integers as payload. The setDeviceLevel method is called on the control with topic name and value for each array value.
 * <p>
 * <h3>AsynchDemuxer:</h3>
 * The demuxer processes formatted responses from the realtime subsystem in response to various directives.<p>
 * The messages are organized by topics identified by a header delimited by chevrons and containing the message topic identifier.<p>
 * Each line after the header has a parameter number and value. For instance, for an analog pin input or an ultrasonic reading
 * we have numeric values for the distance or pin reading value bracketed by the proper headers.
 * The end of the message is delimited with '&lt;/topic&gt;' with topic being the matching header element.<p>
 * 
 * The realtime subsystem is activated by being issued a series of M and G codes similar to 3D printer and CNC standard codes.<br/>
 * Examples:<br>
 * <code> G5 Z[slot] P[power] Q[power] R[power] S...Y[power]</code>- Issue move command to controller in slot Z&lt;slot&gt; for channels P-Y (0 and 1 element left/right wheel) with P-Y&lt;-1000 to 1000&gt;
 * a negative value on power means reverse.
 * M2 - start reception of controller messages - fault/ battery/status demultiplexed in AsynchDemuxer
 * <p>
 * H Bridge Drivers:<br>
 * Organized into "slots" that control multiple channels, up to 10 slots each with 10 channels each with one or more PWM pins and an enable/direction pin.
 * M10 codes define a slot with a specific type of H-bridge driver depending on one or 2 PWM pins for direction and an enable pin, or collection of GPIO pins.
 * Depending on the properties file configuration the proper M10 codes are generated in a preamble. Special consideration is given to names ending in "Wheel" to
 * configure the differential drive.<p>
 * Each channel generates more M-codes in the preamble to define each channel in each slot. 
 * Basic GPIO functions can be defined based on type of "Pin" in the properties:
 * <code><br/>
 * M45 P[pin] S[0-255] - start PWM on pin P with value S, many optional arguments for timer setup<br>
 * M41 P[pin] S1- Set digital pin P high forward <br>
 * M41 P[pin] S0- Set digital pin P low reverse <br>
 * </code>
 * Certain command line remappings are used to affect behavior as follows:<br>
 * determine debugging directives __debug:=publisher,demuxer,marlinspike any of these 3 arguments are optional to turn on debugging<br>
 * __pwm:=controller<br>
 * Indicates that PWM directives sent as a service are to be processed through a PWM based controller
 * initialized with a previous M-code<p>
 * __pwm:=direct<br>
 * Indicates that PWM directives sent as a service are to be directly applied as a set of values working
 * on a PWM pin<p>
 * GPIO service invocation always works directly on a pin.<p>  
 *<pre>
 * Publishing to:<br>
 * Devices function as publishers to data topics as well as subscribers to topics such as the sensor_msgs/Joy joystick channel<be>
 * All topics enumerated in typeNames which correspond to types that may appear in configuration by LUN.k<br>
 * robocore/status (status update channel to receive information on alerts, etc, derived from IMU and other sensors)<br>
 * </pre>
 * Published values from 0-1000 represent actual power to each differential drive wheel.<p>
 * Subscribers include the process that manages serial data to the motor controller, possibly on a separate computer.<p>
 * Publishers on robocore/status provide status updates relating to IMU extremes that are picked up by the VoxHumana voice synth or others,
 * again most probably on a completely separate single board computer within the robot.
 *<pre>
 *<pre>
 * sensor_msgs/MagneticField (3 axis magnetic flux information most likely from IMU)<br>
 * sensor_msgs/Temperature (most likely from IMU)<br>
 * sensor_msgs/Imu (absolutely from IMU, contains heading information roll/pitch/yaw in Euler and Quaternion, pre-fused)<br>
 *</pre>
 * To reduce incoming data, Two sets of flags and a threshold value are used to determine if:<br>
 * a) The values have changed from last message<br>
 * b) The value exceed a threshold deemed noteworthy<br><p>
 * </pre>
 * We take steps to align the joystick reference frame to the following:<br>
 * <code>
 *      &nbsp;/ 0 \<br>
 *   90   |  -90<br>
 *    &nbsp;\   |   /<br>
 *    &nbsp;180,-180<br>
 * </code>
 * The ratio of arc lengths of radius speed and speed+WHEELBASE, and turn angle theta from course difference 
 * becomes the ratio of speeds at each wheel.<br>
 * When in autonomous mode we use the same steering method for large granularity turns. At mid granularity we compute a solution
 * using a right triangle whose base is speed and whose hypotenuse is computed from scaled speed and course deviation. <p>For the short leg,
 * course deviation and half wheeltrack form a chord with radius of WHEELBASE/2 and angle of course deviation: 
 * <code><p>
 * 2*(wheelbase/2)*sin(deviation/2). 
 * </code><p>
 * The ratio of hypotenuse to base is our distance and hence speed for each wheel as we do with the arc solution.<br>
 * Finally, at the smallest granularity, a PID controller takes over to maintain critical crosstrack damping.<br>
 * This methodology is analogous to a human driver, who upon leaving the road and entering median, must compute a wide arc to re-enter
 * the roadway, then perform adjustments that bring them to a tangent of the center of the lane, and from there perform a series
 * of small corrections to stay on course. Regardless of the amount the machine is knocked off course it can find its way back to the
 * proper heading through a circuitous route gradually refined to require only small correction.<p>
 * 
 * MarlinspikeManager will load but not activate the nodes listed in configuration when {@link MarlinspikeManager#createControllers(boolean)}
 * is called with first parameter 'override' set to true, and second parameter 'activate' set to false to indicate we
 * want to get configuration for all available control nodes, but not activate them from this module since this module
 * issues directives to the attached controllers rather than control them directly.<p>
 * We then create collection of {@link DeviceEntry} by calling {@link MarlinspikeManager#getNodeDeviceDemuxerByType}.<p>
 * Finally we create the isActive boolean array to hold/control status of devices.
 *
 * @see MarlinspikeManager
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2017,2018,2019,2020
 *
 */
public class MotionController extends AbstractNodeMain {
	private static boolean DEBUG = true;
	private static boolean IMUDEBUG = false;
	private static boolean DEBUGBEARING = true;
	private static boolean DEBUGSTICK = true;
	//
	// Initialize various types of responses that will be published to the various outgoing message busses.
	// Generate base {@link Robot} that bootstraps configs from RoboCore.properties via VM command line directive
	//
	static RobotInterface robot;
	static String robotName;
	public String RPT_SERVICE = "robo_status";
	private final String PWM_SERVICE = "cmd_pwm";
	private final String GPIO_SERVICE = "cmd_gpio";
	private String PWM_MODE = "direct"; // in direct mode, our 2 PWM values are pin, value, otherwise values of channel 1 and 2 of slot 0 controller
	
	private CircularBlockingDeque<diagnostic_msgs.DiagnosticStatus> statusQueue = new CircularBlockingDeque<diagnostic_msgs.DiagnosticStatus>(1024);
	
	private CircularBlockingDeque<InternalPayload> stickQueue = new CircularBlockingDeque<InternalPayload>(3);

	//private String serveNode = null;
	float rangetop, rangebot;
	double pressure, temperature;
	static class EulerTime {
		float compassHeading;
		float roll;
		float pitch;
		float temperature;
		long eulerTime = 0L;
		public String toString() {
			return String.format("compass heading:%d roll:%d pitch:%d temperature:%d", compassHeading, roll, pitch, temperature);
		}
	}
	EulerTime euler = new EulerTime();

	static class RangeTime {
		float range;
		long rangeTime = 0L;
	}
	RangeTime ranges = new RangeTime();

	static class MagTime {
		geometry_msgs.Vector3 mag3;
		long magTime = 0L;
		public String toString() {
			return String.format("Mag field: %d %d %d",mag3.getX(),mag3.getY(),mag3.getZ());
		}
	}
	MagTime mags = new MagTime();

	static class AngularTime {
		geometry_msgs.Vector3 angular; 
		geometry_msgs.Vector3 linear; 
		geometry_msgs.Quaternion orientation;
		long angTime = 0L;
		public String toString() {
			return String.format("Nav:Orientation X:%d Y:%d Z:%d W:%d, Linear: %d %d %d, Angular: %d %d %d",orientation.getX(),orientation.getY(),orientation.getZ(),orientation.getW(),
					linear.getX(), linear.getY(), linear.getZ(), angular.getX(), angular.getY(), angular.getZ());
		}
	}
	AngularTime angs = new AngularTime();

	//TwistInfo twistInfo;
	float last_theta;

	static final float TWOPI = (float) (Math.PI * 2.0f);
	public static boolean isShock = false;
	public static boolean isOverShock = false;

	// deltas. 971, 136, 36 relatively normal values. seismic: last value swings from rangeTop -40 to 140
	public static float[] SHOCK_BASELINE = { 971.0f, 136.0f, 36.0f};

	public static float[] SHOCK_THRESHOLD = {1000.0f,1000.0f,1000.0f}; 
	public static boolean isMag = false;
	public static boolean isOverMag = false;
	public static short[] MAG_THRESHOLD = {-1,-1,-1};
	private boolean SOFTSTOP = false;
	//public static boolean isMag = false;
	public static int PRESSURE_THRESHOLD = 100000; // pressure_meas is millibars*100 30in is 1014 milli
	public static boolean isOverPressure = false;
	public static boolean isPressure = false;
	public static long lastPressureNotification = 0; // time so we dont just keep yapping about the weather
	public static boolean isTemperature = false;
	public static boolean isOverTemp = false;
	public static boolean isMoving = false;
	public static boolean isNav = false;
	public static boolean isRangeUpperFront = false;
	public static boolean isRangeLowerFront = false;
	public static boolean isVision = false; // machine vision recognition event
	// If we call the wheelbase 1000, it implies that at maximum speed of 1000 it take 2 robot radii to turn 90 degrees
	// This is because the radius of the inner wheel is from speed 0 to speed max, and the outer is from speed 0 to speed max plus wheelbase.
	// Correspondingly at speed 1 the arc describing the inner wheel is 1 and the inner wheel turns at 1, or in place 
	// and the arc describing the outer wheel has a radius equivalent to the wheelbase and the 
	// outer wheel turns at the robot wheelbase through 90 degrees.

	boolean hasData = false; // have we received any feedback from callback?
	boolean init = true;
	
	static float speedL, speedR;
	static float SPEEDSCALE = 1000.0f;
	static int lastSpeedL, lastSpeedR;
	static long lastTime = System.currentTimeMillis();
	static int SampleTime = 100; //.1 sec
	static float outMin, outMax;
	static boolean inAuto = false;

	static final long SHUTDOWN_NS = 250000;
	static long lastCmdTime = System.nanoTime();
	

	Object statMutex = new Object(); 
	Object navMutex = new Object();
	//private String host;
	//private InetSocketAddress master;
	//private MarlinspikeControlInterface motorControlHost;
	NavListenerMotorControlInterface navListener = null;
	private AuxGPIOControl auxGPIO = null;

	// if angularMode is true, interpret the last TWIST subscription as intended target to move to and
	// compute incoming channel velocities as scaling factors to that movement
	protected boolean angularMode = false;

	// Queue for outgoing ranging messages, if any such sensors are present
	CircularBlockingDeque<sensor_msgs.Range> outgoingRanges = new CircularBlockingDeque<sensor_msgs.Range>(256);
	// Preload the collection of response handlers that match responses from attached Marlinspike microcontrollers
	String[] stopics = new String[] {
	AsynchDemuxer.topicNames.BATTERY.val(),
	AsynchDemuxer.topicNames.MOTORFAULT.val(), 
	AsynchDemuxer.topicNames.TIME.val(),
	AsynchDemuxer.topicNames.CONTROLLERSTATUS.val(),
	AsynchDemuxer.topicNames.STATUS.val(),
	AsynchDemuxer.topicNames.CONTROLLERSTOPPED.val(),
	AsynchDemuxer.topicNames.NOMORGCODE.val(),
	AsynchDemuxer.topicNames.BADMOTOR.val(),
	AsynchDemuxer.topicNames.BADPWM.val(),
	AsynchDemuxer.topicNames.UNKNOWNG.val(),
	AsynchDemuxer.topicNames.UNKNOWNM.val(),
	AsynchDemuxer.topicNames.BADCONTROL.val(),
	AsynchDemuxer.topicNames.NOCHECKSUM.val(),
	AsynchDemuxer.topicNames.NOLINECHECK.val(),
	AsynchDemuxer.topicNames.CHECKMISMATCH.val(),
	AsynchDemuxer.topicNames.LINESEQ.val(),
	AsynchDemuxer.topicNames.M115.val(),
	AsynchDemuxer.topicNames.ASSIGNEDPINS.val(),
	AsynchDemuxer.topicNames.MOTORCONTROLSETTING.val(),
	AsynchDemuxer.topicNames.PWMCONTROLSETTING.val(),
	AsynchDemuxer.topicNames.DIGITALPINSETTING.val(),
	AsynchDemuxer.topicNames.ANALOGPINSETTING.val(),
	AsynchDemuxer.topicNames.ULTRASONICPINSETTING.val(),
	AsynchDemuxer.topicNames.PWMPINSETTING.val()
	};

	Byte[] publishStatus = new Byte[] {	 diagnostic_msgs.DiagnosticStatus.WARN,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.WARN,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK};
		
	public static class InternalPayload {
		private float[] axis;
		private int[]  buttons;
		public InternalPayload(Joy stick) {
			axis = new float[stick.getAxes().length];
			buttons = new int[stick.getButtons().length];
			System.arraycopy(stick.getAxes(), 0, axis, 0, stick.getAxes().length);
			System.arraycopy(stick.getButtons(), 0, buttons, 0, stick.getButtons().length);
		}
		public float[] getAxis() {
			return axis;
		}
		public int[] getButtons() {
			return buttons;
		}
	}
	
	PublishResponseInterface<diagnostic_msgs.DiagnosticStatus>[] responses;
	PublishResponseInterface<sensor_msgs.Range>[] ultrasonic;
	
	private Collection<String> statPub = Collections.synchronizedCollection(new ArrayList<String>());
	protected CountDownLatch awaitStart = new CountDownLatch(1);
	
	SubmissionPublisher<InternalPayload> publisherAxis = new SubmissionPublisher<>();

	public MotionController(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
		NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
		nodeMainExecutor.execute(this, cl.build());
		if( DEBUG ) {
			System.out.println("Bringing up MotionControl with args:"+args[0]+" "+args[1]+" "+args[2]);
		}
	}

	public MotionController() {}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("pubsubs_motion");
	}

	/**
	 * Extract the linear and angular components from cmd_vel topic Twist quaternion, take the linear X (pitch) and
	 * angular Z (yaw) and send them to motor control. This results in motion planning computing a turn which
	 * involves rotation about a point in space located at a distance related to speed and angular velocity. The distance
	 * is used to compute the radius of the arc segment traversed by the wheel track to make the turn. If not moving we can
	 * make the distance 0 and rotate about a point in space, otherwise we must inscribe an appropriate arc. The distance
	 * in that case is the linear travel, otherwise the distance is the diameter of the arc segment.
	 * If we get commands on the cmd_vel topic we assume we are moving, if we do not get the corresponding IMU readings, we have a problem
	 * If we get a 0,0 on the X,yaw move we stop. If we dont see stable IMU again we have a problem, Houston.
	 * 'targetDist' is x. if x = 0 we are going to turn in place.
	 *  If we are turning in place and th &lt; 0, turn left. if th &gt;= 0 turn right
	 *  If x != 0 and th = 0 its a forward or backward motion
	 *  If x != 0 and th != 0 its a rotation around a point in space with forward motion, describing an arc.
	 *  theta is th and gets calculated as difference of last imu and wheel theta.
	 *  Rotation about a point in space
	 *  <code>
	 *  <pre>
	 *		if( th &lt; 0 ) { // left
	 *			spd_left = (float) (x - th * wheelTrack / 2.0);
	 *			spd_right = (float) (x + th * wheelTrack / 2.0);
	 *		} else {
	 *			spd_right = (float) (x - th * wheelTrack / 2.0);
	 *			spd_left = (float) (x + th * wheelTrack / 2.0);
	 *		}
	 *</pre>
	 *</code>
	 */
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		if(remaps.containsKey("__robot") ) {
			robotName = remaps.get("__robot");
		} else {
			throw new RuntimeException("Must specify __robot:=<name> to configure parameters.");
		}

		//final Log log = connectedNode.getLog();

		if( remaps.containsKey("__debug") ) {
			String debug = remaps.get("__debug");
			DEBUG = true;
			String[] debugs = debug.split(",");
			for(String debugx : debugs) {
				if(debugx.equals("demuxer"))
					AsynchDemuxer.DEBUG = true;
				else
					if(debugx.equals("marlinspike"))
						MarlinspikeControl.DEBUG = true;
					else
						if(debugx.equals("config"))
							Robot.DEBUG = true;
			}
		}

		final Publisher<diagnostic_msgs.DiagnosticStatus> diagnosticStatus = connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);

		try {
			ParameterTree pTree = connectedNode.getParameterTree();
			robot = new Robot(robotName);
			pTree.set(robotName, robot);
			if(robot.getManager().getDemuxer() == null) {
				if(DEBUG)
					System.out.printf("%s Creating Demuxer came back null...%n", this.getClass().getName());
			}
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
		//
		// Add a message listener for each subscriber device key. When a new message comes in
		// extract the device name from the map based on the subscriber instance associated with the incoming message.
		// Based on the device name, get the MarlinspikeControlInterface from the MarlinspikeManager.
		// We can then set the power level etc on the device from the data in the message.
		//
		// process the DeviceEntries into publishing channels
		configureSubscribers(connectedNode, diagnosticStatus, statusQueue, statPub);
		// in direct mode, our 2 PWM values are pin, value, otherwise values of channel 1 and 2 of slot 0 controller
		if( remaps.containsKey("__pwm") )
			PWM_MODE = remaps.get("__pwm");

		//-----------------------------
		// process remappings to override configs
		if(remaps.containsKey("__speedlimit") ) {
			ArrayList<DrivenWheelInterface> wheels = robot.getDiffDrive().getWheels();
			for(DrivenWheelInterface dw: wheels) {
				dw.getSpeedsetPointInfo().setMaximum(Float.parseFloat(remaps.get("__speedlimit")));
			}
		}
		if( remaps.containsKey("__kp") )
			robot.getMotionPIDController().setKp(Float.parseFloat(remaps.get("__kp")));
		if( remaps.containsKey("__kd") )
			robot.getMotionPIDController().setKd(Float.parseFloat(remaps.get("__kd")));
		//final Publisher<geometry_msgs.Twist> mopub = connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);
		// statpub has status alerts that may come from sensors.
		// Check startup for indoor speed scale setting ((stick -1 to 1) * SPEEDSCALE), then check cmdl for override of speed scale from startup and defaults
		// MAX is always 1000
		if( remaps.containsKey("__speedscale") )
			SPEEDSCALE = Float.parseFloat(remaps.get("__speedscale"));
		// see if we are implementing software dead-man protocol True or False
		if( remaps.containsKey("__softstop") )
			SOFTSTOP  = Boolean.parseBoolean(remaps.get("__softstop"));
		//-----------------------------
		responses = new PublishDiagnosticResponse[stopics.length];

		if(DEBUG)
			System.out.println("Robot:"+robot);
		//final RosoutLogger log = (Log) connectedNode.getLog();
		//                                                                                                                             
		//-------------------------------------
		// Iterate the list of slots we put together in configureMarlinspikeManager.
		// for each slot, create a subscriber on the channel <slot name> of type Int32MultiArray
		// Initialize the collection of DiagnosticStatus response handlers
		for(int i = 0; i < stopics.length; i++) {
			responses[i] = new PublishDiagnosticResponse(connectedNode, diagnosticStatus, statusQueue);
			responses[i].takeBridgeAndQueueMessage(stopics[i], robot.getManager().getDemuxer().getTopic(stopics[i]), publishStatus[i]);
		}
		//final Subscriber<std_msgs.Int32MultiArray> subsvelocity = 
		//		connectedNode.newSubscriber("absolute/cmd_vel", std_msgs.Int32MultiArray._TYPE);
		//final Subscriber<std_msgs.Int32MultiArray> substrigger = 
		//connectedNode.newSubscriber("absolute/cmd_periph1", std_msgs.Int32MultiArray._TYPE);

		final CountDownServiceServerListener<ControllerStatusMessageRequest, ControllerStatusMessageResponse> serviceServerListener = CountDownServiceServerListener.newDefault();
		configureControllerStatusServer(serviceServerListener, connectedNode, diagnosticStatus);	

		final CountDownServiceServerListener<PWMControlMessageRequest, PWMControlMessageResponse> servicePWMServerListener = CountDownServiceServerListener.newDefault();
		configurePWMServer(servicePWMServerListener, connectedNode, diagnosticStatus);

		final CountDownServiceServerListener<GPIOControlMessageRequest, GPIOControlMessageResponse> serviceGPIOServerListener = CountDownServiceServerListener.newDefault();
		configureGPIOServer(serviceGPIOServerListener, connectedNode, diagnosticStatus);	

		//Subscriber<sensor_msgs.Range> subsrange = connectedNode.newSubscriber("LowerFront/sensor_msgs/Range", sensor_msgs.Range._TYPE);
		// publisher to twist cmd_vel
		//final Publisher<geometry_msgs.Twist> twistpub = connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);
		//final geometry_msgs.Twist twistmsg = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Twist._TYPE);
		// We use the twist subscriber cmd_vel topic to get the generic universal stop command for all attached devices when pose = -1,-1,-1
		//final Subscriber<geometry_msgs.Twist> substwist = connectedNode.newSubscriber("cmd_vel", geometry_msgs.Twist._TYPE);
		final Subscriber<sensor_msgs.Joy> substick = connectedNode.newSubscriber("/sensor_msgs/Joy", sensor_msgs.Joy._TYPE);
		final Subscriber<sensor_msgs.Imu> subsimu = connectedNode.newSubscriber("/sensor_msgs/Imu", sensor_msgs.Imu._TYPE);
		final Subscriber<std_msgs.String> subsrange = connectedNode.newSubscriber("/sensor_msgs/range",std_msgs.String._TYPE);
		//Subscriber<sensor_msgs.Range> subsrangebot = connectedNode.newSubscriber("LowerFront/sensor_msgs/Range", sensor_msgs.Range._TYPE);
		final Subscriber<sensor_msgs.MagneticField> subsmag = connectedNode.newSubscriber("/sensor_msgs/MagneticField", sensor_msgs.MagneticField._TYPE);
		final Subscriber<sensor_msgs.Temperature> substemp = connectedNode.newSubscriber("/sensor_msgs/Temperature", sensor_msgs.Temperature._TYPE);
		final Subscriber<trajectory_msgs.ComeToHeadingStamped> subsmodelmove = connectedNode.newSubscriber("/model_commands/move", ComeToHeadingStamped._TYPE);
		/*
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
		 */
		MessageProcessor messageProcessor = new MessageProcessor(this, publisherAxis);
		SynchronizedThreadManager.getInstance().init(new String[]{"MOTIONCONTROL"});
		SynchronizedThreadManager.getInstance().spin(messageProcessor, "MOTIONCONTROL");	
		
		substick.addSubscriberListener(new SubscriberListener<sensor_msgs.Joy>() {
			@Override
			public void onMasterRegistrationFailure(Subscriber<Joy> subs) {
				if(DEBUG)
					System.out.printf("%s Subscsriber %s failed to register with master!%n", this.getClass().getName(), subs);
				ArrayList<String> st = new ArrayList<String>();
				st.addAll(Arrays.stream(new Throwable().getStackTrace()).map(m->m.toString()).collect(Collectors.toList()));
				new PublishDiagnosticResponse(connectedNode, diagnosticStatus, statusQueue, this.getClass().getName(),diagnostic_msgs.DiagnosticStatus.ERROR, st);
			}
			@Override
			public void onMasterUnregistrationSuccess(Subscriber<Joy> subs) {
				if(DEBUG)
					System.out.printf("%s Subscsriber %s unregistered with master!%n", this.getClass().getName(), subs);			
			}
			@Override
			public void onMasterRegistrationSuccess(Subscriber<Joy> subs) {

			}
			@Override
			public void onMasterUnregistrationFailure(Subscriber<Joy> subs) {
				if(DEBUG)
					System.out.printf("%s Subscsriber %s failed to unregister with master!%n", this.getClass().getName(), subs);					
			}
			@Override
			public void onNewPublisher(Subscriber<Joy> subs, PublisherIdentifier pubs) {			
				if(DEBUG)
					System.out.printf("%s Subscsriber %s registered with publisher %s!%n", this.getClass().getName(), subs, pubs);
				substick.addMessageListener(new MessageListener<sensor_msgs.Joy>() {
					@Override
					public void onNewMessage(sensor_msgs.Joy message) {
						try {
							queueJoystickMessages(message);
						} catch(IOException ioe) {
							throw new RuntimeException(ioe);
						}
					} // onMessage from Joystick controller, with all the axes[] and buttons[]	
				});
			}
			@Override
			public void onShutdown(Subscriber<Joy> subs) {
				if(DEBUG)
					System.out.printf("%s Subscsriber %s shutdown!%n", this.getClass().getName(), subs);
			}
		});

		subsmodelmove.addSubscriberListener(new SubscriberListener<trajectory_msgs.ComeToHeadingStamped>() {
			@Override
			public void onMasterRegistrationFailure(Subscriber<ComeToHeadingStamped> subs) {
				if(DEBUG)
					System.out.printf("%s Subscsriber %s failed to register with master!%n", this.getClass().getName(), subs);
				ArrayList<String> st = new ArrayList<String>();
				st.addAll(Arrays.stream(new Throwable().getStackTrace()).map(m->m.toString()).collect(Collectors.toList()));
				new PublishDiagnosticResponse(connectedNode, diagnosticStatus, statusQueue, this.getClass().getName(),
						diagnostic_msgs.DiagnosticStatus.ERROR, st);
			}
			@Override
			public void onMasterUnregistrationSuccess(Subscriber<ComeToHeadingStamped> subs) {
				if(DEBUG)
					System.out.printf("%s Subscsriber %s unregistered with master!%n", this.getClass().getName(), subs);			
			}
			@Override
			public void onMasterRegistrationSuccess(Subscriber<ComeToHeadingStamped> subs) {
			}
			@Override
			public void onMasterUnregistrationFailure(Subscriber<ComeToHeadingStamped> subs) {
				if(DEBUG)
					System.out.printf("%s Subscsriber %s failed to unregister with master!%n", this.getClass().getName(), subs);					
			}
			@Override
			public void onNewPublisher(Subscriber<ComeToHeadingStamped> subs, PublisherIdentifier pubs) {
				if(DEBUG)
					System.out.printf("%s Subscsriber %s registered with publisher %s!%n", this.getClass().getName(), subs, pubs);
				subsmodelmove.addMessageListener(new MessageListener<trajectory_msgs.ComeToHeadingStamped>() {
					@Override
					public void onNewMessage(trajectory_msgs.ComeToHeadingStamped message) {
						try {
							moveRobotRelative(message);
						} catch (IOException e) {
							ArrayList<String> st = new ArrayList<String>();
							st.addAll(Arrays.stream(e.getStackTrace()).map(m->m.toString()).collect(Collectors.toList()));
							new PublishDiagnosticResponse(connectedNode, diagnosticStatus, statusQueue, "/model_commands/move",
									diagnostic_msgs.DiagnosticStatus.ERROR, st);
						}
					} // onMessage move from model
				});
			}
			@Override
			public void onShutdown(Subscriber<ComeToHeadingStamped> subs) {
				if(DEBUG)
					System.out.printf("%s Subscsriber %s shutdown!%n", this.getClass().getName(), subs);
			}
		});

		subsimu.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
			@Override
			public void onNewMessage(sensor_msgs.Imu message) {
				try {
					synchronized(angs) {
						if(angs.angular == null || message.getAngularVelocity().getX() != angs.angular.getX() ||
								message.getAngularVelocity().getY() != angs.angular.getY() ||
								message.getAngularVelocity().getZ() != angs.angular.getZ() ||
								message.getLinearAcceleration().getX() != angs.linear.getX() ||
								message.getLinearAcceleration().getY() != angs.linear.getY() ||
								message.getLinearAcceleration().getZ() != angs.linear.getZ()) {
							angs.angular = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
							angs.linear = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
							angs.orientation =  connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
							angs.orientation.setX(message.getOrientation().getX());
							angs.orientation.setY(message.getOrientation().getY());
							angs.orientation.setZ(message.getOrientation().getZ());
							angs.orientation.setW(message.getOrientation().getW());

							euler.compassHeading = message.getCompassHeadingDegrees();
							euler.roll = message.getRoll();
							euler.pitch = message.getPitch();
							euler.temperature = message.getTemperature();
							euler.eulerTime = System.currentTimeMillis();
							angs.angTime = System.currentTimeMillis();
							if(robot.getDiffDrive() != null)
								robot.getDiffDrive().setCompassHeading(euler.compassHeading);
							
							if(IMUDEBUG) {
								System.out.println("Nav:"+euler.toString()+" "+angs.toString());
							}
							isNav = true;
							isOverShock = false;
							if( SHOCK_THRESHOLD[0] != -1 ) {
								for(int i = 0; i < SHOCK_BASELINE.length; i++) {
									if( Math.abs(angs.linear.getX()-SHOCK_BASELINE[i]) > SHOCK_THRESHOLD[i] ) {
										isOverShock = true;
									}
								}
								if( isOverShock && !isMoving) {
									if(DEBUG)
										System.out.println("OVERSHOCK!");
									ArrayList<String> shockVals = new ArrayList<String>(2);
									synchronized(angs) {
										shockVals.add(angs.toString());
									}
									new PublishDiagnosticResponse(connectedNode, diagnosticStatus, statusQueue, "Accelerometer shock warning", 
											diagnostic_msgs.DiagnosticStatus.WARN, shockVals);
								}
							}
						} else
							isNav = false;
					}
				} catch (Throwable e) {
					isNav = false;
					//System.out.println("Nav subs exception:"+e.getMessage());
					ArrayList<String> st = new ArrayList<String>();
					st.addAll(Arrays.stream(e.getStackTrace()).map(m->m.toString()).collect(Collectors.toList()));
					new PublishDiagnosticResponse(connectedNode, diagnosticStatus, statusQueue, "IMU",
							diagnostic_msgs.DiagnosticStatus.ERROR, st);
				}
			}
		},5); // buffer 5 messages

		subsrange.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				if( DEBUG )
					System.out.println("New range msg:"+message);
				try {
					JSONObject jDist = new JSONObject(message.getData());
					float fDist = jDist.getFloat("nowdistance");
					if(fDist != rangetop) {
						rangetop = fDist;
						isRangeUpperFront = true;
						System.out.println(" Range Top:"+rangetop);
					} else {
						isRangeUpperFront = false;
					}
				} catch (Throwable e) {
					isRangeUpperFront = false;
					System.out.println("Range top subs exception:"+e.getMessage());
					e.printStackTrace();
					ArrayList<String> st = new ArrayList<String>();
					st.addAll(Arrays.stream(e.getStackTrace()).map(m->m.toString()).collect(Collectors.toList()));
					new PublishDiagnosticResponse(connectedNode, diagnosticStatus, statusQueue, "RANGE",
							diagnostic_msgs.DiagnosticStatus.ERROR, st);
				}
			}
		});
		/*
		subsrangebot.addMessageListener(new MessageListener<sensor_msgs.Range>() {
		@Override
		public void onNewMessage(sensor_msgs.Range message) {
			if( DEBUG )
				System.out.println("New range msg:"+message);
			synchronized(rngMutex2) {
				try {
					if( message.getRange() != rangebot) {
						rangebot = message.getRange();
						isRangeLowerFront = true;
						System.out.println(" Range Bottom:"+rangebot);
					} else
						isRangeLowerFront = false;
				} catch (Throwable e) {
					isRangeLowerFront = false;
					System.out.println("Range bottom subs exception:"+e.getMessage());
					e.printStackTrace();
				}
			}
		}
		});
		 */
		subsmag.addMessageListener(new MessageListener<sensor_msgs.MagneticField>() {
			@Override
			public void onNewMessage(sensor_msgs.MagneticField message) {
				if( IMUDEBUG )
					System.out.println("New mag msg:"+message);
				synchronized(mags) {
					if(mags.mag3 == null)
						mags.mag3 = message.getMagneticField();
					if( mags.mag3.getX() != message.getMagneticField().getX() || 
							mags.mag3.getY() != message.getMagneticField().getY() || 
							mags.mag3.getZ() != message.getMagneticField().getZ()) {
						mags.mag3.setX(message.getMagneticField().getX()); 
						mags.mag3.setY(message.getMagneticField().getY());
						mags.mag3.setZ(message.getMagneticField().getZ());
						mags.magTime = System.currentTimeMillis();
						isMag = true;
						if(IMUDEBUG)
							System.out.println(mags);
						if( MAG_THRESHOLD[0] != -1 ) {
							if( mags.mag3.getX() > MAG_THRESHOLD[0] && 
									mags.mag3.getY() > MAG_THRESHOLD[1] && 
									mags.mag3.getZ() > MAG_THRESHOLD[2] ) {
								if(DEBUG)
									System.out.println("Mag EXCEEDS threshold..");
								ArrayList<String> magVals = new ArrayList<String>(3);
								magVals.add(String.valueOf(mags.mag3.getX()));
								magVals.add(String.valueOf(mags.mag3.getY()));
								magVals.add(String.valueOf(mags.mag3.getZ()));		
								new PublishDiagnosticResponse(connectedNode, diagnosticStatus, statusQueue, "Magnetic anomaly detected", 
										diagnostic_msgs.DiagnosticStatus.WARN, magVals);
							}
						}
					} else
						isMag = false;
				}
			}
		});

		substemp.addMessageListener(new MessageListener<sensor_msgs.Temperature>() {
			@Override
			public void onNewMessage(sensor_msgs.Temperature message) {
				if( IMUDEBUG )
					System.out.println("New temp msg:"+message);
				if(temperature != message.getTemperature()) {
					temperature = message.getTemperature();
					if(DEBUG || IMUDEBUG)
						System.out.println(" Temp:"+temperature);
					isTemperature = true;
					if( temperature > robot.getTemperatureThreshold() ) {
						isOverTemp = true;
						if( DEBUG )
							System.out.println("DANGER Temp:"+temperature);
						isOverTemp = false;
						ArrayList<String> tempVals = new ArrayList<String>(1);
						tempVals.add(String.valueOf(temperature));
						new PublishDiagnosticResponse(connectedNode, diagnosticStatus, statusQueue, "Temperature warning", 
								diagnostic_msgs.DiagnosticStatus.WARN, tempVals);
					}
				} else
					isTemperature = false;
				if( isOverPressure ) { // check for dropping
					isOverPressure = false;
					ArrayList<String> pressVals = new ArrayList<String>(1);
					pressVals.add(String.valueOf(pressure));
					new PublishDiagnosticResponse(connectedNode, diagnosticStatus, statusQueue, "Atmospheric pressure warning", 
							diagnostic_msgs.DiagnosticStatus.WARN, pressVals);
				}
			}
		});
		//-------------------------------------------
		//
		/*substwist.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
			@Override
			public void onNewMessage(geometry_msgs.Twist message) {
				geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
				val = message.getLinear();
				targetPitch = (int) val.getX();
				targetDist = (int) val.getY();
				val = message.getAngular();
				targetYaw = (float) val.getZ();
				if( targetPitch == -1 && targetDist == -1 && targetYaw == -1) {
					shouldMove = false;
					try {
						Iterator<?> it = robot.getManager().getDevices().keySet().iterator();
						while(it.hasNext()) {
							DeviceEntry ndd = (DeviceEntry) it.next();
							ndd.getMarlinspikeControl().commandStop();
						}
					} catch (IOException e) {
						e.printStackTrace();
					}
					robot.getOperating().replaceAll((key, value) -> false);
				}		
			}
		});*/

		awaitStart.countDown();
		//----------------------------------------
		// Begin publishing loop
		//
		// A CancellableLoop will be canceled automatically when the node shuts down.
		// Check the various topic message queues for entries and if any are present, multiplex them onto the outbound bus
		// under the DiagnosticStatus topics for any waiting subscribers to process and deal with.
		//
		//----------------------------------------
		connectedNode.executeCancellableLoop(new CancellableLoop() {
			private int sequenceNumber;

			@Override
			protected void setup() {
			    try {
					awaitStart .await();
				} catch (InterruptedException e) {}
				sequenceNumber = 0;
				robot.getIMUSetpointInfo().setPrevErr(0.0f); // 0 degrees yaw
				robot.getIMUSetpointInfo().setMinimum(-robot.getIMUSetpointInfo().getMaximum()); //degree minimum integral windup
				robot.getMotionPIDController().clearPID();
				// default kp,ki,kd;
				//SetTunings(5.55f, 1.0f, 0.5f); // 5.55 scales a max +-180 degree difference to the 0 1000,0 -1000 scale
				//SetOutputLimits(0.0f, SPEEDLIMIT); when pid controller created, max is specified
				// Invoke the collection of response handlers, this is done for each asynchDemuxer attached to this node, i.e. each Marlinspike	
				if(DEBUG)
					System.out.printf("%s.setup responses to publish: %d%n",this.getClass().getName(),responses.length);
				for(int i = 0; i < stopics.length; i++) {
					if(DEBUG)
						System.out.printf("%s.setup response publish: %s%n",this.getClass().getName(),responses[i]);
					responses[i].publish();
				}
			}

			@Override
			protected void loop() throws InterruptedException {	
				diagnostic_msgs.DiagnosticStatus statmsg = null;
				//
				// Poll the outgoing message array for diagnostics enqueued by the above processing
				//
				statmsg = statusQueue.poll(1, TimeUnit.MILLISECONDS);
				if(statmsg != null) {
					if(DEBUG)
						System.out.printf("%s.loop outgoing diagnostics: %s%n",this.getClass().getName(),statusQueue);
					diagnosticStatus.publish(statmsg);
					if(DEBUG)
						System.out.println("Published diagnostic message:"+statmsg.getMessage());
					Thread.sleep(1);
				}
			}
		}); // cancellable loop

	} // onStart
	

	/**
	* Configure a subscriber to talk to a MarlinSpikeControlInterface which receives 
	* a message of type: LUN#, value of control for that LUN#. <br>
	* Create publishing channels based on configured {@link DeviceEntry}<br>
	* The slotted entries represent a slot in the configuration which can be an aggregate of property file LUN's;
	* a collection of node, device, channel of a controller attached to a ROS node, which has
	* a number of tty ports (devices) which have Marlinspike boards attached (Mega2560 or Pico via USB running the firmware)
	* which have a number of logical software controllers configured by slot and channels.<p>
	* Slots are handled internally via the the SlotToHandler class, 
	* Set up a trigger value for the remote destination, most likely originating from a controller such as PS/3 or the AI model.<p>
	* Take the 2 trigger values as int32 and send them on to the microcontroller 
	* related subsystem at the int valued LUN slots. The Marlinspike subsystem is composed of a software 
	* controller or SBC such as Pico talking to a hardware driver such as an H-bridge or half bridge or even a simple switch.<p>
	* Non slotted devices are treated as channels to be published to.
	* Unslotted entries are configured by name in the properties file.
	* @param connectedNode the Ros node we are using
	* @param statpub the diagnostic channel if error, called through creation of new PublishDiagnosticResponse
	* @param diagQueue diagnostic queue
	* @param statusQueue status queue
	*/
	private void configureSubscribers(ConnectedNode connectedNode, Publisher<diagnostic_msgs.DiagnosticStatus> statpub, CircularBlockingDeque<DiagnosticStatus> diagQueue, Collection<String> statusQueue) {
		robot.getManager().getDevices().keySet().forEach(ndd -> {
			DeviceSubscriber<InternalPayload> dsa = new DeviceSubscriber<InternalPayload>(connectedNode, statpub, diagQueue, statusQueue, ndd);
			publisherAxis.subscribe(dsa);
			if(DEBUG)
				System.out.printf("%s Created axis Handler for device %s subscriber:%s%n", this.getClass().getName(), ndd.getTopicName(), dsa);	
		});
	}

	static class MessageProcessor implements Runnable {
		private MotionController motionControl;
		private SubmissionPublisher<InternalPayload> publisherAxis;
		private volatile boolean shouldRun = true;
		public MessageProcessor(MotionController motionControl, SubmissionPublisher<InternalPayload> publisherAxis) {
			this.motionControl = motionControl;
			this.publisherAxis = publisherAxis;
		}
		@Override
		public void run() {
			while(shouldRun) {
				InternalPayload message;
				try {
					message = motionControl.stickQueue.take();
				} catch (InterruptedException e) {
					shouldRun = false;
					continue;
				}
				publisherAxis.submit(message);
			}
		}
	}
	
	private void queueJoystickMessages(Joy message) throws IOException {
		stickQueue.addLast(new InternalPayload(message));
	}
	
	/**
	 * Send the command to move the robot relative to designated heading
	 * @param connectedNode
	 * @param heading
	 * @throws IOException
	 */
	public void moveRobotRelative(ComeToHeadingStamped heading) throws IOException {
		int[] speeds = null;
		if(robot.getDiffDrive() == null)
			return;
		synchronized(euler) {
			if((System.currentTimeMillis()-euler.eulerTime) > (System.currentTimeMillis()-heading.getTime().getData())) {
				speeds = robot.getDiffDrive().moveRobotRelative(heading.getHeading().getData(), heading.getHeading().getData(), heading.getDist().getData());
			} else {
				speeds = robot.getDiffDrive().moveRobotRelative((float) euler.compassHeading, heading.getHeading().getData(), heading.getDist().getData());
			}
		}
		speedL = speeds[0];
		speedR = speeds[1];
		robot.getDiffDrive().commandMotorPower((int)speedL, (int)speedR);
	}
	/**
	 * Configure the GPIO direct service.
	 * @param serviceGPIOServerListener
	 * @param connectedNode
	 * @param tstatpub
	 */
	private void configureGPIOServer(CountDownServiceServerListener<GPIOControlMessageRequest, GPIOControlMessageResponse> serviceGPIOServerListener,
		ConnectedNode connectedNode, Publisher<DiagnosticStatus> tstatpub) {
		final ServiceServer<GPIOControlMessageRequest, GPIOControlMessageResponse> serviceGPIOServer = connectedNode.newServiceServer(GPIO_SERVICE, GPIOControlMessage._TYPE,
			    new ServiceResponseBuilder<GPIOControlMessageRequest, GPIOControlMessageResponse>() {
					@Override
					public void build(GPIOControlMessageRequest request, GPIOControlMessageResponse response) {	
						try {
							System.out.println("GPIO direct");
							if( auxGPIO == null )
								auxGPIO = new AuxGPIOControl();
							auxGPIO.activateAux(robot.getManager().getMarlinspikeControl("GPIO"), request.getData().getData());
							response.setData("success");
						} catch (IOException e) {
							System.out.println("EXCEPTION ACTIVATING MARLINSPIKE VIA GPIO SERVICE");
							e.printStackTrace();
							response.setData("fail");
							synchronized(statPub) {
								statPub.add("EXCEPTION ACTIVATING MARLINSPIKE VIA GPIO SERVICE:"+e);
								new PublishDiagnosticResponse(connectedNode, tstatpub, statusQueue, "MARLINSPIKE GPIO ACTIVATION", diagnostic_msgs.DiagnosticStatus.ERROR, statPub );
								while(!statusQueue.isEmpty()) {
									try {
										tstatpub.publish(statusQueue.takeFirst());
										Thread.sleep(1);
									} catch (InterruptedException e1) {}
								}
							}
						}
					}
				});	
			serviceGPIOServer.addListener(serviceGPIOServerListener);	      
			try {
				serviceGPIOServerListener.awaitMasterRegistrationSuccess(1, TimeUnit.SECONDS);
			} catch (InterruptedException e1) {
				System.out.println("GPIO SERVICE REGISTRATION WAS INTERRUPTED");
				e1.printStackTrace();
			}
	
	}
	/**
	 * Configure the PWM direct service
	 * @param servicePWMServerListener
	 * @param connectedNode
	 * @param tstatpub
	 */
	private void configurePWMServer(CountDownServiceServerListener<PWMControlMessageRequest, PWMControlMessageResponse> servicePWMServerListener,
		ConnectedNode connectedNode, Publisher<DiagnosticStatus> tstatpub) {
		final ServiceServer<PWMControlMessageRequest, PWMControlMessageResponse> servicePWMServer = connectedNode.newServiceServer(PWM_SERVICE, PWMControlMessage._TYPE,
			    new ServiceResponseBuilder<PWMControlMessageRequest, PWMControlMessageResponse>() {
					@Override
					public void build(PWMControlMessageRequest request, PWMControlMessageResponse response) {	
						switch(PWM_MODE) {
							case "controller":
								break;
								// Directly Activates PWM M45 Pin Level on Marlinspike with 2 values of request.
								// Activates a direct PWM timer without going through one of the various controller
								// objects. Issues an M45 to the Marlinspike on the Mega2560
							case "direct":
							default:
								System.out.println("PWM direct");
								break;
						}
						response.setData("success");
					}
				});	
			servicePWMServer.addListener(servicePWMServerListener);	      
			try {
				servicePWMServerListener.awaitMasterRegistrationSuccess(1, TimeUnit.SECONDS);
			} catch (InterruptedException e1) {
				System.out.println("PWM SERVICE REGISTRATION WAS INTERRUPTED");
				e1.printStackTrace();
			}
	}
	/**
	 * Configure the controller status service
	 * @param serviceServerListener
	 * @param connectedNode
	 * @param tstatpub
	 */
	private void configureControllerStatusServer(CountDownServiceServerListener<ControllerStatusMessageRequest, ControllerStatusMessageResponse> serviceServerListener, 
			ConnectedNode connectedNode, Publisher<DiagnosticStatus> tstatpub) {
		final ServiceServer<ControllerStatusMessageRequest, ControllerStatusMessageResponse> serviceServer = connectedNode.newServiceServer(RPT_SERVICE, ControllerStatusMessage._TYPE,
		    new ServiceResponseBuilder<ControllerStatusMessageRequest, ControllerStatusMessageResponse>() {
				@Override
				public void build(ControllerStatusMessageRequest request,ControllerStatusMessageResponse response) {
					StringBuilder sb = new StringBuilder();
					try {
						Iterator<?> it = robot.getManager().getDevices().keys().asIterator();
						switch(request.getData()) {
							case "id":
								while(it.hasNext()) {
									DeviceEntry ndd = (DeviceEntry) it.next();
									sb.append(ndd.getMarlinspikeControl().reportSystemId());
								}
								response.setData(sb.toString());
								break;
							case "reset":
								while(it.hasNext()) {
									DeviceEntry ndd = (DeviceEntry) it.next();
									sb.append(ndd.getMarlinspikeControl().commandReset());
								}
								response.setData(sb.toString());
								break;
							case "status":
							default:
								while(it.hasNext()) {
									DeviceEntry ndd = (DeviceEntry) it.next();
									sb.append(ndd.getMarlinspikeControl().reportAllControllerStatus());
								}
								response.setData(sb.toString());
								break;		
						}
					} catch (IOException e) {
						System.out.println("EXCEPTION FROM SERVICE REQUESTING ALL CONTROLLER STATUS REPORT FROM MARLINSPIKE:"+e);
						e.printStackTrace();
						synchronized(statPub) {
							statPub.add("EXCEPTION FROM SERVICE REQUESTING ALL CONTROLLER STATUS REPORT FROM MARLINSPIKE:"+e);
							new PublishDiagnosticResponse(connectedNode, tstatpub, statusQueue, "MARLINSPIKE STATUS", 
									diagnostic_msgs.DiagnosticStatus.ERROR, statPub );
							while(!statusQueue.isEmpty()) {
								try {
									tstatpub.publish(statusQueue.takeFirst());
									Thread.sleep(1);
								} catch (InterruptedException e1) {}
							}
						}
					}
				}
			});	
		serviceServer.addListener(serviceServerListener);	      
		try {
			serviceServerListener.awaitMasterRegistrationSuccess(1, TimeUnit.SECONDS);
		} catch (InterruptedException e1) {
			System.out.println("REPORT SERVICE REGISTRATION WAS INTERRUPTED");
			e1.printStackTrace();
		}
	}


	/*
	 // Create Roll Pitch Yaw Angles from Quaternions 
	double yy = quat.y() * quat.y(); // 2 Uses below
	double roll = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()), 1 - 2*(quat.x() * quat.x() + yy));
	double pitch = asin(2 * quat.w() * quat.y() - quat.x() * quat.z());
	double yaw = atan2(2 * (quat.w() * quat.z() + quat.x() * quat.y()), 1 - 2*(yy+quat.z() * quat.z()));

	//Convert Radians to Degrees 
	float rollDeg  = 57.2958 * roll;
	float pitchDeg = 57.2958 * pitch;
	float yawDeg   = 57.2958 * yaw;
	 */
	public static void main(String[] args) throws IOException {
		Robot robot = new Robot();
		MotionController.robot = robot;
		ArrayList<String> pubs = new ArrayList<String>();
		Iterator<?> it = MotionController.robot.getManager().getDevices().keys().asIterator();
		while(it.hasNext()) {
			DeviceEntry ndd = (DeviceEntry) it.next();
			System.out.println(ndd.getName()+" topic:"+ndd.getTopicName());
			if(DEBUG)
				System.out.println("Bringing up publisher "+ndd.getTopicName());
			pubs.add("/"+ndd.getTopicName());
		}
		System.out.printf("Diffydrive; %s%n", robot.getDiffDrive());
		System.out.println("----");
		System.out.println(pubs.stream().filter(e -> e.toString().contains("LEDDriver"))
				.findFirst().get());
		System.out.println(pubs.stream().filter(e -> e.toString().contains("BoomActuator"))
				.findFirst().get());
		System.out.println(pubs.stream().filter(e -> e.toString().contains("DiffDrive"))
				.findFirst().get());
		System.out.println(pubs.stream().filter(e -> e.toString().contains("LiftActuator"))
				.findFirst().get());
	}
}
