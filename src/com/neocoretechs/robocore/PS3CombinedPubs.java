package com.neocoretechs.robocore;

import net.java.games.input.Component;
import net.java.games.input.Component.Identifier;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Version;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.net.InetSocketAddress;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CountDownLatch;

import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.internal.node.server.ThreadPoolManager;


/**
 * Publishes user constructed Joystick or Gamepad messages on the /sensor_msgs/Joy topic. 
 * Associate Joystick or PS3 controller axis made up of components like digital hats, analog axis, digital axis.
 * The __refresh parameter dictates the minimum time between data publishing. If the data changes from the previous state
 * publishing takes place. If the data does not change, but the refresh interval is exceeded, check the current data
 * against the default values established at startup, and if there is deviation, publish the data.
 * We are using the standard ROS Joystick message, which boxes up the axis and buttons as arrays of floats for the axes
 * and another array of ints for buttons, so the full payload is sent each pub. Upon reception of payload, one can assume
 * the data has changed or the interval exceeded and a pulse of the previous data is incoming. The effort is to intuitively
 * provide the type of response expected, wherein if the stick is pressed, you keep getting data at some predefined rate.
 * The rate of refresh has a default value which can be modified by the command line parameter __refresh:=1000 to set it
 * to 1 second for example.
 * Identifiers:
 * NYCO CORE CONTROLLER:
 * Identifies as : USB Joystick
 *Axis x		Left stick (left/right)
 *Axis y		Left stick (up/down)
 *Axis rx		N/A	 
 *Axis ry		N/A	 
 *Axis z	 	Right stick (left/right)
 *Axis rz		Right stick (up/down)
 *Button 0	x  - Thumb 2
 *Button 1	circle - Thumb
 *Button 2	square - Top
 *Button 3	triangle - Trigger
 *Button 4	Left Bumper	- Top2
 *Button 5	Right Bumper - Pinkie
 *Button 6	Select - Base3
 *Button 7	Start - Base4
 *Button 8	Left Stick Press - Base5
 *Button 9	Right Stick Press - Base6
 *
 * AFTERGLOW CONTROLLER:
 * Identifies as : Generic X-Box Pad (right, even though its a PS3!)
 *Axis x		Left stick (left/right)
 *Axis y		Left stick (up/down)
 *Axis z	 	Left trigger
 *Axis rx(rx)	Right stick (left/right)	 
 *Axis ry(ry)	Right stick (up/down) 
 *Axis rz		Right trigger
 *Button B(B) - circle
 *Button X(X) - square
 *Button A(A) - X
 *Button Y(Y) - triangle
 *Button Left Thumb - left bumper
 *Button Right Thumb - right bumper
 *Button Left Thumb 3 - left stick press
 *Button Right Thumb 3 - right stick press
 *Button Select - Select
 *Button Mode - Mode
 *Button Unknown - Home
 *
 * LOGITECH, INC F310 GAMEPAD
 * Identifies as : Generic X-box pad (Controls are the same as AfterGlow) or Logitech Gamepad F310
 *Axis x		Left stick (left/right)
 *Axis y		Left stick (up/down)
 *Axis z	 	Left trigger
 *Axis rx(rx)	Right stick (left/right)	 
 *Axis ry(ry)	Right stick (up/down) 
 *Axis rz		Right trigger
 *Button B(B) - circle
 *Button X(X) - square
 *Button A(A) - X
 *Button Y(Y) - triangle
 *Button Left Thumb - left bumper
 *Button Right Thumb - right bumper
 *Button Left Thumb 3 - left stick press
 *Button Right Thumb 3 - right stick press
 *Button Select - Select
 *Button Mode - Mode
 *Button Unknown - Home
 *
 * If you dont know the controller name run the code and it will enumerate the found controllers.
 * On the command line, specify the controller name to use, and the controller type of joystick or gamepad.
 * Some cheap gamepads, such as the nyco, identify as joystick, presumably for greater fit on old machines
 * @author jg Copyright (C) NeoCoreTechs 2017,2018
 */
public class PS3CombinedPubs extends AbstractNodeMain  {
	private static boolean DEBUG = true;
	private static volatile boolean shouldRun = false; // main controller run method loop control
	private String host;
	private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);
	sensor_msgs.Joy joymsg = null;
	public ConcurrentHashMap<Identifier, Float> pubdata = new ConcurrentHashMap<Identifier,Float>();
	public ConcurrentHashMap<Identifier, Float> prevdata = new ConcurrentHashMap<Identifier,Float>();
	public ConcurrentHashMap<Identifier, Float> basedata = new ConcurrentHashMap<Identifier,Float>();
	float[] axisPub = new float[7];
	int[] buttonPub = new int[14];

	List<ControllerManager> controllers = new ArrayList<ControllerManager>();
	final static String DEFAULT_CONTROLLER = "Generic X-box pad";
	final static String DEFAULT_TYPE = "gamepad";
	final static int DEFAULT_REFRESH = 100; // ms max between messages regardless of data
	String controllerName = DEFAULT_CONTROLLER;
	String controllerType = DEFAULT_TYPE;
	int refresh = DEFAULT_REFRESH;
	final static double DEADZONE = 0.00007;
	private static long lastPubTime = System.currentTimeMillis();
	private static long lastRefreshTime = System.currentTimeMillis();
	private boolean shouldPublish = false;
	private boolean onceThrough = true; // sets initial defaults from stick readings, done at first and then when refresh interval exceeded
	
	
	public PS3CombinedPubs(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	}
	
	public PS3CombinedPubs() {}
	
	public GraphName getDefaultNodeName() {
		return GraphName.of("pubs_ps3unit1");
	}



@Override
public void onStart(final ConnectedNode connectedNode) {
	//fileReader reader = new fileReader();
	//ThreadPoolManager.getInstance().spin(reader, "SYSTEM");
	//final RosoutLogger log = (Log) connectedNode.getLog();
	
	final Publisher<sensor_msgs.Joy> velpub =
		connectedNode.newPublisher("/sensor_msgs/Joy", sensor_msgs.Joy._TYPE);
	/*
	final Publisher<std_msgs.Empty> tofpub = connectedNode.newPublisher("ardrone/takeoff", std_msgs.Empty._TYPE);
	final Publisher<std_msgs.Empty> lanpub = connectedNode.newPublisher("ardrone/land", std_msgs.Empty._TYPE);
	final Publisher<std_msgs.Empty> forpub = connectedNode.newPublisher("ardrone/forward", std_msgs.Empty._TYPE);
	final Publisher<std_msgs.Empty> bakpub = connectedNode.newPublisher("ardrone/backward", std_msgs.Empty._TYPE);
	final Publisher<std_msgs.Empty> uppub = connectedNode.newPublisher("ardrone/up", std_msgs.Empty._TYPE);
	final Publisher<std_msgs.Empty> downpub = connectedNode.newPublisher("ardrone/down", std_msgs.Empty._TYPE);
	final Publisher<std_msgs.Empty> golpub = connectedNode.newPublisher("ardrone/goleft", std_msgs.Empty._TYPE);
	final Publisher<std_msgs.Empty> gorpub = connectedNode.newPublisher("ardrone/goright", std_msgs.Empty._TYPE);
	final Publisher<std_msgs.Empty> spinlpub = connectedNode.newPublisher("ardrone/spinleft", std_msgs.Empty._TYPE);
	final Publisher<std_msgs.Empty> spinrpub = connectedNode.newPublisher("ardrone/spinright", std_msgs.Empty._TYPE);
	final Publisher<std_msgs.Int16> speedpub = connectedNode.newPublisher("ardrone/setspeed", std_msgs.Int16._TYPE);
	final Publisher<std_msgs.Int16> altpub = connectedNode.newPublisher("ardrone/setalt", std_msgs.Int16._TYPE);
	final Publisher<std_msgs.Empty> freezepub = connectedNode.newPublisher("ardrone/freeze", std_msgs.Empty._TYPE);
	final Publisher<std_msgs.Empty> hovpub = connectedNode.newPublisher("ardrone/hover", std_msgs.Empty._TYPE);
	final Publisher<std_msgs.Empty> respub = connectedNode.newPublisher("ardrone/reset", std_msgs.Empty._TYPE);
	*/
	
	// check command line remappings for __controller:=CONTROLLER or whatever your controller identifies as
	Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();

	if( remaps.containsKey("__controller") )
		controllerName = remaps.get("__controller");
	if( remaps.containsKey("__type") )
		controllerType = remaps.get("__type");
	System.out.println("Target controller is "+controllerName+" of type "+controllerType);
	if( remaps.containsKey("__refresh") )
		refresh = Integer.parseInt(remaps.get("__refresh"));
	System.out.println("Refresh rate is "+refresh+" ms.");
	if( remaps.containsKey("__debug") )
		DEBUG = true;
	// Main controller creation and thread invocation
	ControllerReader(pubdata, basedata, controllerName);
	
	awaitStart.countDown();
	/**
	 * Main loop. Populate array as follows in Joy to be published as sensor_msgs.Joy:
	 * --float[] axis:--
	 * [0] - left horiz (X)
	 * [1] - left vert (Y)
	 * [2] - right horiz (X)
	 * [3] - right vert (Y) 
	 * [4] - left trigger
	 * [5] - right trigger
	 * [6] - POV
	 * --int[] buttons--
	 * [0] - select
	 * [1] - start
	 * [2] - left stick press
	 * [3] - right stick press
	 * [4] - triangle
	 * [5] - circle
	 * [6] - X button
	 * [7] - square
	 * [8] - left bumper
	 * [9] - right bumper
	 * -- joystick only --
	 * [10] - back
	 * [11] - extra
	 * [12] - mode
	 * [13] - side
	 */
	connectedNode.executeCancellableLoop(new CancellableLoop() {
		@Override
		protected void setup() {
			//sequenceNumber = 0;
			//val = connectedNode.getTopicMessageFactory().newFromType(sensor_msgs.Joy._TYPE);
			
		}

		@Override
		protected void loop() throws InterruptedException {
			if( !shouldRun) {
				System.out.println("Cancelling publishing loop");
				this.cancel();
				return;
			}
		    try {
				awaitStart.await();
			} catch (InterruptedException e) {}
			// Basically, if its not a default controller Generic Xbox pad, we assume its USB Joystick type
			Float data = null;
			Float predata = null;
			shouldPublish = false;
			if( !pubdata.isEmpty() ) { // no data yet?
				if( !controllerType.equals(DEFAULT_TYPE) ) { // not a gamepad, but a joystick?
					
				    if(pubdata.containsKey(Component.Identifier.Axis.X)) { // left stick horiz.
							data = pubdata.get(Component.Identifier.Axis.X);
							predata = prevdata.get(Component.Identifier.Axis.X);
							if( predata == null || Math.abs(predata.floatValue() - data) > DEADZONE) {
								prevdata.put(Component.Identifier.Axis.X, data);
								if( !onceThrough ) {
									if( DEBUG )System.out.println("Left stick horiz.(X):"+data);
									shouldPublish = true;
								}
							}
							axisPub[0] = data;
					} 
					if(pubdata.containsKey(Component.Identifier.Axis.Z)) { // right stick horiz.
							data = pubdata.get(Component.Identifier.Axis.Z);
							predata = prevdata.get(Component.Identifier.Axis.Z);	
							if( predata == null || Math.abs(predata.floatValue() - data) > DEADZONE) {
								prevdata.put(Component.Identifier.Axis.Z, data);
								if( !onceThrough ) {
									if( DEBUG )System.out.println("Right stick horiz.(X):"+data);
									shouldPublish = true;
								}
							}
							axisPub[1] = data;
					}
					  
					if(pubdata.containsKey(Component.Identifier.Axis.Y)) { // left stick vertical (Y) for USB Joystick
						data = pubdata.get(Component.Identifier.Axis.Y);
						predata = prevdata.get(Component.Identifier.Axis.Y);
						if( predata == null || Math.abs(predata.floatValue() - data) > DEADZONE ) {
								prevdata.put(Component.Identifier.Axis.Y, data);
								if( !onceThrough ) {
									if( DEBUG )System.out.println("Left stick vertical (Y):"+data);
									shouldPublish = true;
								}
								axisPub[2] = data;
						}
					}
					
					if( pubdata.containsKey(Component.Identifier.Axis.RZ))  { // Right stick vertical
						data = pubdata.get(Component.Identifier.Axis.RZ);
						predata = prevdata.get(Component.Identifier.Axis.RZ);
						if( predata == null || Math.abs(predata.floatValue() - data) > DEADZONE ) {
								prevdata.put(Component.Identifier.Axis.RZ, data);
								if( !onceThrough ) {
									if( DEBUG )System.out.println("Right Stick vertical (Y):"+data);
									shouldPublish = true;
								}
								axisPub[3] = data;
							}
					}
					if(pubdata.containsKey(Component.Identifier.Button.BASE)) { // left trigger
						data = pubdata.get(Component.Identifier.Button.BASE);
						predata = prevdata.get(Component.Identifier.Button.BASE);
						if( predata == null || predata.floatValue() != data ) {
								prevdata.put(Component.Identifier.Button.BASE, data);
								if( !onceThrough ) {
									if( DEBUG )System.out.println("Trigger left:"+data);
									shouldPublish = true;
								}
								axisPub[4] = data;
							}
					}
					if(pubdata.containsKey(Component.Identifier.Button.BASE2)) { // trigger right
						data = pubdata.get(Component.Identifier.Button.BASE2);
						predata = prevdata.get(Component.Identifier.Button.BASE2);
						if( predata == null || predata.floatValue() != data ) {
							prevdata.put(Component.Identifier.Button.BASE2, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Trigger right:"+data);
								shouldPublish = true;
							}
							axisPub[5] = data;
						}
					}
					if(pubdata.containsKey(Component.Identifier.Button.BASE3)) { // select
						data = pubdata.get(Component.Identifier.Button.BASE3);
						predata = prevdata.get(Component.Identifier.Button.BASE3);
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.BASE3, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Select:"+data);
								shouldPublish = true;
							}
							buttonPub[0] = data.intValue();
						}
					}
					if(pubdata.containsKey(Component.Identifier.Button.BASE4)) { // start
						data = pubdata.get(Component.Identifier.Button.BASE4);
						predata = prevdata.get(Component.Identifier.Button.BASE4);
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.BASE4, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Start:"+data);
								shouldPublish = true;
							}
							buttonPub[1] = data.intValue();
						}
	
					}
					if(pubdata.containsKey(Component.Identifier.Button.BASE5)) { // left stick press
						data = pubdata.get(Component.Identifier.Button.BASE5);
						predata = prevdata.get(Component.Identifier.Button.BASE5);
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.BASE5, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Left Stick Press:"+data);
								shouldPublish = true;
							}
							buttonPub[2] = data.intValue();
						}
					}
					if(pubdata.containsKey(Component.Identifier.Button.BASE6)) { // right stick press
						data = pubdata.get(Component.Identifier.Button.BASE6);
						predata = prevdata.get(Component.Identifier.Button.BASE6);	
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.BASE6, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Right Stick Press:"+data);
								shouldPublish = true;
							}
							buttonPub[3] = data.intValue();
						}
					}
					if(pubdata.containsKey(Component.Identifier.Button.TRIGGER)) { // triangle
					  data = pubdata.get(Component.Identifier.Button.TRIGGER);
					  predata = prevdata.get(Component.Identifier.Button.TRIGGER);
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.TRIGGER, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Trigger (Triangle) Press:"+data);
								shouldPublish = true;
							}
							buttonPub[4] = data.intValue();
						}
					}
					if(pubdata.containsKey(Component.Identifier.Button.THUMB)) { // circle
					  data = pubdata.get(Component.Identifier.Button.THUMB);
					  predata = prevdata.get(Component.Identifier.Button.THUMB);
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.THUMB, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Thumb (circle) Press:"+data);
								shouldPublish = true;
							}
							buttonPub[5] = data.intValue();
						}
					}
					if(pubdata.containsKey(Component.Identifier.Button.THUMB2)) { // X button
					  data = pubdata.get(Component.Identifier.Button.THUMB2);
					  predata = prevdata.get(Component.Identifier.Button.THUMB2);
					  if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.THUMB2, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Thumb (X Button) Press:"+data);
								shouldPublish = true;
							}
							buttonPub[6] = data.intValue();
						}
					}
					if(pubdata.containsKey(Component.Identifier.Button.TOP)) { // square
						data = pubdata.get(Component.Identifier.Button.TOP);
						predata = prevdata.get(Component.Identifier.Button.TOP);
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.TOP, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Top (square) Press:"+data);
								shouldPublish = true;
							}
							buttonPub[7] = data.intValue();
						}
					}
					if(pubdata.containsKey(Component.Identifier.Button.TOP2)) {  //left bumper
						data = pubdata.get(Component.Identifier.Button.TOP2);
						predata = prevdata.get(Component.Identifier.Button.TOP2);
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.TOP2, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Top2 (left bumper) Press:"+data);
								shouldPublish = true;
							}
							buttonPub[8] = data.intValue();
						}
					}
					if(pubdata.containsKey(Component.Identifier.Button.PINKIE)) { // right bumper
						data = pubdata.get(Component.Identifier.Button.PINKIE);
						predata = prevdata.get(Component.Identifier.Button.PINKIE);
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.PINKIE, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Pinkie (right bumper) Press:"+data);
								shouldPublish = true;
							}
							buttonPub[9] = data.intValue();
						}
					}
					if(pubdata.containsKey(Component.Identifier.Button.BACK)) { 
						data = pubdata.get(Component.Identifier.Button.BACK);
						predata = prevdata.get(Component.Identifier.Button.BACK);
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.BACK, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Back:"+data);
								shouldPublish = true;
							}
							buttonPub[10] = data.intValue();	 
						}
					}
					if(pubdata.containsKey(Component.Identifier.Button.EXTRA)) { 
						data = pubdata.get(Component.Identifier.Button.EXTRA);
						predata = prevdata.get(Component.Identifier.Button.EXTRA);
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.EXTRA, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Extra:"+data);
								shouldPublish = true;
							}
							buttonPub[11] = data.intValue();	 
						}
					}
					if(pubdata.containsKey(Component.Identifier.Button.MODE)) { 
						data = pubdata.get(Component.Identifier.Button.MODE);
						predata = prevdata.get(Component.Identifier.Button.MODE);
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.MODE, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Mode:"+data);
								shouldPublish = true;
							}
							buttonPub[12] = data.intValue();	 
						}
					}
					if(pubdata.containsKey(Component.Identifier.Button.SIDE)) { 
						data = pubdata.get(Component.Identifier.Button.SIDE);
						predata = prevdata.get(Component.Identifier.Button.SIDE);
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.SIDE, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Side:"+data);
								shouldPublish = true;
							}
							buttonPub[13] = data.intValue();	 
						}
					}
					
				} else { // Default pad
					
				     if(pubdata.containsKey(Component.Identifier.Axis.X)) { // left stick horiz.
							data = pubdata.get(Component.Identifier.Axis.X);
							predata = prevdata.get(Component.Identifier.Axis.X);
							if( predata == null || Math.abs(predata.floatValue() - data) > DEADZONE) {
								prevdata.put(Component.Identifier.Axis.X, data);
								if( !onceThrough ) {
									if( DEBUG )System.out.println("Left stick horiz.(X):"+data);
									shouldPublish = true;
								}
							}
							axisPub[0] = data;
					 }
				      
					 if(pubdata.containsKey(Component.Identifier.Axis.RX)) { // right stick horiz.
							data = pubdata.get(Component.Identifier.Axis.RX);
							predata = prevdata.get(Component.Identifier.Axis.RX);	
							if( predata == null || Math.abs(predata.floatValue() - data) > DEADZONE) {
								prevdata.put(Component.Identifier.Axis.RX, data);
								if( !onceThrough ) {
									if( DEBUG )System.out.println("Right stick horiz.(X):"+data);
									shouldPublish = true;
								}
							}
							axisPub[1] = data;
					}
					  
					if(pubdata.containsKey(Component.Identifier.Axis.Y)) { // left vertical
						data = pubdata.get(Component.Identifier.Axis.Y);
						predata = prevdata.get(Component.Identifier.Axis.Y);
						if( predata == null || Math.abs(predata.floatValue() - data) > DEADZONE ) {
							prevdata.put(Component.Identifier.Axis.Y, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Left stick vertical (Y):"+data);
								shouldPublish = true;
							}
							axisPub[2] = data;
						}
					}
					
					if( pubdata.containsKey(Component.Identifier.Axis.RY))  { // right vertical
						data = pubdata.get(Component.Identifier.Axis.RY);
						predata = prevdata.get(Component.Identifier.Axis.RY);
						if( predata == null || Math.abs(predata.floatValue() - data) > DEADZONE ) {
							prevdata.put(Component.Identifier.Axis.RY, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Right stick vertical (Y):"+data);
								shouldPublish = true;
							}
							axisPub[3] = data;
						}
					}
	
					if(pubdata.containsKey(Component.Identifier.Axis.Z)) { // left trigger
						data = pubdata.get(Component.Identifier.Axis.Z);
						predata = prevdata.get(Component.Identifier.Axis.Z);
						if( predata == null || Math.abs(predata.floatValue() - data) > DEADZONE) {
							prevdata.put(Component.Identifier.Axis.Z, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Left Trigger (Z):"+data);
								shouldPublish = true;
							}
							axisPub[4] = data;
						}
					} 
	
					if(pubdata.containsKey(Component.Identifier.Axis.RZ)) { // trigger right
						data = pubdata.get(Component.Identifier.Axis.RZ);
						predata = prevdata.get(Component.Identifier.Axis.RZ);	
						if( predata == null || Math.abs(predata.floatValue() - data) > DEADZONE) {
							prevdata.put(Component.Identifier.Axis.RZ, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Right Trigger (RZ):"+data);
								shouldPublish = true;
							}
							axisPub[5] = data;
						}
					}
	
					if(pubdata.containsKey(Component.Identifier.Button.SELECT)) { // select
						data = pubdata.get(Component.Identifier.Button.SELECT);
						predata = prevdata.get(Component.Identifier.Button.SELECT);	
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.SELECT, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Select:"+data);
								shouldPublish = true;
							}
							buttonPub[0] = data.intValue();
						}
					} 
	
					if(pubdata.containsKey(Component.Identifier.Button.MODE)) { // start
						data = pubdata.get(Component.Identifier.Button.MODE);
						predata = prevdata.get(Component.Identifier.Button.MODE);
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.MODE, data);
							if( !onceThrough ) {
								if( DEBUG ) System.out.println("Start (MODE):"+data);
								shouldPublish = true;
							}
							buttonPub[1] = data.intValue();
						}
					}

					if(pubdata.containsKey(Component.Identifier.Button.LEFT_THUMB3)) { // left stick press
						data = pubdata.get(Component.Identifier.Button.LEFT_THUMB3);
						predata = prevdata.get(Component.Identifier.Button.LEFT_THUMB3);
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.LEFT_THUMB3, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Left Stick Press (Left Thumb3):"+data);
								shouldPublish = true;
							}
							buttonPub[2] = data.intValue();
						}
					}
			  
					if(pubdata.containsKey(Component.Identifier.Button.RIGHT_THUMB3)) { // right stick press
						data = pubdata.get(Component.Identifier.Button.RIGHT_THUMB3);
						predata = prevdata.get(Component.Identifier.Button.RIGHT_THUMB3);	
						if( predata == null || predata.floatValue() != data) {
							//prevdata.put(Component.Identifier.Button.BASE2, data);
							prevdata.put(Component.Identifier.Button.RIGHT_THUMB3, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Right Stick Press (Right Thumb3):"+data);
								shouldPublish = true;
							}
							buttonPub[3] = data.intValue();
						}
					}
	
					if(pubdata.containsKey(Component.Identifier.Button.Y)) { // triangle
						data = pubdata.get(Component.Identifier.Button.Y);
						predata = prevdata.get(Component.Identifier.Button.Y);  
						if( predata == null || predata.floatValue() != data) {
						//prevdata.put(Component.Identifier.Button.TRIGGER, data);
							prevdata.put(Component.Identifier.Button.Y, data);
							if( !onceThrough ) {
								if(DEBUG)System.out.println("Triangle (Y) :"+data);
								shouldPublish = true;
							}
							buttonPub[4] = data.intValue();
						}
					}
	
					if(pubdata.containsKey(Component.Identifier.Button.B)) { // circle
						data = pubdata.get(Component.Identifier.Button.B);
						predata = prevdata.get(Component.Identifier.Button.B);
						if( predata == null || predata.floatValue() != data) {
						//prevdata.put(Component.Identifier.Button.THUMB, data);
							prevdata.put(Component.Identifier.Button.B, data);
							if( !onceThrough ) {
								if(DEBUG)System.out.println("Circle (B) :"+data);
								shouldPublish = true;
							}
							buttonPub[5] = data.intValue();
						}
					}
	
					if(pubdata.containsKey(Component.Identifier.Button.A)) { // X button
						data = pubdata.get(Component.Identifier.Button.A);
						predata = prevdata.get(Component.Identifier.Button.A);
						if( predata == null || predata.floatValue() != data) {
							//prevdata.put(Component.Identifier.Button.THUMB2, data);
							prevdata.put(Component.Identifier.Button.A, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("X (A):"+data);
								shouldPublish = true;
							}
							buttonPub[6] = data.intValue();
						}
					}
	
					if(pubdata.containsKey(Component.Identifier.Button.X)) { // square
						data = pubdata.get(Component.Identifier.Button.X);
						predata = prevdata.get(Component.Identifier.Button.X);  
						if( predata == null || predata.floatValue() != data) {
						//prevdata.put(Component.Identifier.Button.TOP, data);
							prevdata.put(Component.Identifier.Button.X, data);
							if( !onceThrough ) {
								if(DEBUG)System.out.println("Square (X):"+data);
								shouldPublish = true;
							}
							buttonPub[7] = data.intValue();
						}
					}
	
					if(pubdata.containsKey(Component.Identifier.Button.LEFT_THUMB)) {  //left bumper
						data = pubdata.get(Component.Identifier.Button.LEFT_THUMB);
						predata = prevdata.get(Component.Identifier.Button.LEFT_THUMB);
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.LEFT_THUMB, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Left bumper (Left Thumb) :"+data);
								shouldPublish = true;
							}
							buttonPub[8] = data.intValue();
						}
					}
	
					if(pubdata.containsKey(Component.Identifier.Button.RIGHT_THUMB)) { // right bumper
						data = pubdata.get(Component.Identifier.Button.RIGHT_THUMB);
						predata = prevdata.get(Component.Identifier.Button.RIGHT_THUMB);	  
						if( predata == null || predata.floatValue() != data) {
							prevdata.put(Component.Identifier.Button.RIGHT_THUMB, data);
							if( !onceThrough ) {
								if( DEBUG )System.out.println("Right Bumper (Right thumb):"+data);
								shouldPublish = true;
							}
							buttonPub[9] = data.intValue();
						}
					}
	
			} // default controller
			//
			// COMMON controls
			// POV if present is assumed common to all platforms
		    if(pubdata.containsKey(Component.Identifier.Axis.POV)) {
				  data = pubdata.get(Component.Identifier.Axis.POV);
				  predata = prevdata.get(Component.Identifier.Axis.POV);
				  if( predata == null || predata.floatValue() != data) {
					prevdata.put(Component.Identifier.Axis.POV, data);
					if( !onceThrough ) {
					  shouldPublish = true;
					  if (data == Component.POV.OFF){
						if(DEBUG)System.out.println("POV OFF");
					  } else if ( data == Component.POV.UP) {
						if(DEBUG)System.out.println("POV Up");
						//std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						//forpub.publish(val);
					  } else if ( data == Component.POV.UP_RIGHT) {
						if(DEBUG)System.out.println("POV Up_Right");
						//std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						//uppub.publish(val);
					  } else if ( data == Component.POV.RIGHT) {
						if(DEBUG)System.out.println("POV Right");
						//std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						//gorpub.publish(val);
					  } else if ( data == Component.POV.DOWN_RIGHT) {
						if(DEBUG)System.out.println("POV Down Right");
						//std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						//downpub.publish(val);
					  } else if ( data == Component.POV.DOWN) {
						if(DEBUG)System.out.println("POV Down");
						//std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						//bakpub.publish(val);
					  } else if ( data == Component.POV.DOWN_LEFT) {
						if(DEBUG)System.out.println("POV Down left");
						//std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						//downpub.publish(val);
					  } else if ( data == Component.POV.LEFT) {
						if(DEBUG)System.out.println("POV Left");
						//std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						//golpub.publish(val);
					  } else if ( data == Component.POV.UP_LEFT) {
						if(DEBUG)System.out.println("POV Up Left");
						//std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						//uppub.publish(val);
					  } else { // should never happen
						if(DEBUG)System.out.println("POV IMPOSSIBLE!!");
					  }
					}
					axisPub[6] = data;
				  } // POV data reduction
			  } // POV
		    onceThrough = false;
		    // check if refresh interval exceeded, if so check against default values
		    if( !shouldPublish && (System.currentTimeMillis() - lastRefreshTime) > refresh) {
		    	synchronized(prevdata) {
		    		Set<Entry<Identifier,Float>> bb = prevdata.entrySet();
		    		for(Entry<Identifier,Float> pv : bb) {
		    			if(Math.abs(basedata.get(pv.getKey()) - pv.getValue()) > DEADZONE) {
		    				shouldPublish = true;
		    				break;
		    			}
		    		}
		    	}
		    	lastRefreshTime = System.currentTimeMillis();
		    	//if( DEBUG ) {
		    	//	for(Float f : prevdata.values()) System.out.print(f+" ");
		    	//	System.out.println();
		    	//}
		    }
		  } // pubdata not empty
		
		if(shouldPublish) {
			shouldPublish = false;
			sensor_msgs.Joy val = setupPub(connectedNode, axisPub, buttonPub);
			velpub.publish(val);
			if( DEBUG ) 
				System.out.println("Published with "+val+" rate:"+(System.currentTimeMillis()-lastPubTime));
			lastPubTime = System.currentTimeMillis();
			lastRefreshTime = lastPubTime; // a pub counts as refresh since something changed
		  }
		} // loop	
	}); // cancellable loop
}

private abstract static class Axis {
	Component axis;
	float data;

	public Axis(Component ax){
		axis = ax;
		if( DEBUG )
			System.out.println("Axis constructed from component:"+ax.getName()+"("+ax.getIdentifier()+")");
	}

	public void poll(){
		data = axis.getPollData();
		renderData();
	}

	public Component getAxis() {
		return axis;
	}

	protected abstract void renderData();
}

private static class DigitalAxis extends Axis {
	ConcurrentHashMap<Identifier, Float> pubdata2;
	ConcurrentHashMap<Identifier, Float> basedata2;
	public DigitalAxis(ConcurrentHashMap<Identifier, Float> pubdata2,ConcurrentHashMap<Identifier, Float> basedata2, Component ax) {
		super(ax);
		this.pubdata2 = pubdata2;
		this.basedata2 = basedata2;
	}

	protected void renderData(){
		if( pubdata2.containsKey(getAxis().getIdentifier())) {
			if( DEBUG )
				System.out.println("DigitalAxis:"+getAxis().getIdentifier()+" replacing data:"+data);
			pubdata2.replace(getAxis().getIdentifier(), data);
		} else {
			if( DEBUG )
				System.out.println("DigitalAxis:"+getAxis().getIdentifier()+" putting data:"+data);
			pubdata2.put(getAxis().getIdentifier(), data);
			basedata2.put(getAxis().getIdentifier(), data);
		}
	}
}

private static class DigitalHat extends Axis {
	ConcurrentHashMap<Identifier,Float> pubdata2;
	ConcurrentHashMap<Identifier,Float> basedata2;
	public DigitalHat(ConcurrentHashMap<Identifier,Float> pubdata2, ConcurrentHashMap<Identifier,Float> basedata2, Component ax) {
		super(ax);
		this.pubdata2 = pubdata2;
		this.basedata2 = basedata2;
	}

	protected void renderData(){
		/*
		if (data == Component.POV.OFF){
			
		} else if ( data == Component.POV.UP) {
			
		} else if ( data == Component.POV.UP_RIGHT) {
			
		} else if ( data == Component.POV.RIGHT) {
			
		} else if ( data == Component.POV.DOWN_RIGHT) {
			
		} else if ( data == Component.POV.DOWN) {
			
		} else if ( data == Component.POV.DOWN_LEFT) {
			
		} else if ( data == Component.POV.LEFT) {
			
		} else if ( data == Component.POV.UP_LEFT) {
		
		} else { // should never happen
	
		}
		*/

		if( pubdata2.containsKey(getAxis().getIdentifier())) {
			if( DEBUG )
				System.out.println("DigitalHat:"+getAxis().getIdentifier()+" replacing data:"+data);
			pubdata2.replace(getAxis().getIdentifier(), data);
		} else {
			if( DEBUG )
				System.out.println("DigitalHat:"+getAxis().getIdentifier()+" putting data:"+data);
			pubdata2.put(getAxis().getIdentifier(), data);
			basedata2.put(getAxis().getIdentifier(), data);
		}
		
	}
}

private static class AnalogAxis extends Axis {
	ConcurrentHashMap<Identifier, Float> pubdata2;
	ConcurrentHashMap<Identifier,Float> basedata2;
	public AnalogAxis(ConcurrentHashMap<Identifier, Float> pubdata2, ConcurrentHashMap<Identifier,Float> basedata2, Component ax) {
		super(ax);
		this.pubdata2 = pubdata2;
		this.basedata2 = basedata2;
	}

	protected void renderData() {
		if (getAxis().getDeadZone() >= Math.abs(data)) {
			//" (DEADZONE)";
			data = 0;
		}
		if( pubdata2.containsKey(getAxis().getIdentifier())) {
			if( DEBUG )
				System.out.println("AnalogAxis:"+getAxis().getIdentifier()+" replacing data:"+data);
			pubdata2.replace(getAxis().getIdentifier(), data);
		} else {
			if( DEBUG )
				System.out.println("AnalogAxis:"+getAxis().getIdentifier()+" putting data:"+data);
			pubdata2.put(getAxis().getIdentifier(), data);
			basedata2.put(getAxis().getIdentifier(), data);
		}
	}
}
/**
 * ControllerManager for a particular controller. A controller has a list of Axis.
 * A list of Axis starts with a list of components by controller. 
 * @author jg
 *
 * @param <T>
 */
public static class ControllerManager {
	Controller ca;
	List<Axis> axisList = new ArrayList<Axis>();
	boolean disabled = false;
	ConcurrentHashMap<Identifier, Float> pubdata2;
	ConcurrentHashMap<Identifier,Float> basedata2;

	public ControllerManager(ConcurrentHashMap<Identifier, Float> pubdata2, ConcurrentHashMap<Identifier,Float> basedata2, Controller ca){
		this.pubdata2 = pubdata2;
		this.basedata2 = basedata2;
		this.ca = ca;
		Component[] components = ca.getComponents();
		if( DEBUG )
			System.out.println("Controller:"+ca.getName()+" Component count = "+components.length);
		if (components.length>0) {
			for(int j=0;j<components.length;j++){
				addAxis(pubdata2, basedata2, components[j]);
			} 
		}
	}

	public boolean disabled() {
		return disabled;
	}

	private void setDisabled(boolean b){
		disabled = b;
		if (!disabled){
			if( DEBUG )
				System.out.println(ca.getName()+" enabled");
		} else {
			if( DEBUG )
				System.out.println(ca.getName()+" disabled");
		}
	}

	private void addAxis(ConcurrentHashMap<Identifier,Float> pubdata2, ConcurrentHashMap<Identifier,Float> basedata2, Component ax){
		Axis p2;
		if (ax.isAnalog()) {
			p2 = new AnalogAxis(pubdata2, basedata2, ax);
		} else {
			if (ax.getIdentifier() == Component.Identifier.Axis.POV) {
				p2 = new DigitalHat(pubdata2, basedata2, ax);
			} else {     
				p2 = new DigitalAxis(pubdata2, basedata2, ax);
			}
		}
		axisList.add(p2);
		//ax.setPolling(true);
	}

	public void poll(){
		if (!ca.poll()) {
			if (!disabled()){
				setDisabled(true);
			}
			return;
		} 
		if (disabled()){
			setDisabled(false);
		}
		//System.out.println("Polled "+ca.getName());
		for(Iterator<Axis> i =axisList.iterator();i.hasNext();){
			try {
				((Axis)i.next()).poll();
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}
}

/**
 * Top level method to publish to a particular topic via placing retrieved data value
 * in ConcurrentHashMap of componentId, data value. Publishing loop will retrieve desired component
 * by name from map. 
 * We look specifically for controllers with cotype like "USB Joystick" 
 * or we get keyboard, mouse, etc as well.
 * @param pubdata2 The map that holds the values returned from each controller index
 * @param cotype The string that the desired controller name will contain to identify it
 */
public void ControllerReader(ConcurrentHashMap<Identifier, Float> pubdata2, ConcurrentHashMap<Identifier,Float> basedata2, String cotype) {

	if( DEBUG )
		System.out.println("Controller version: " + Version.getVersion());
	ControllerEnvironment ce = ControllerEnvironment.getDefaultEnvironment();
	Controller[] ca = ce.getControllers();
	for(int i =0;i<ca.length;i++){
		if( DEBUG )
			System.out.println("Found Controller:"+ca[i].getName()+" while looking for name that contains:"+cotype);
		if( ca[i].getName().contains(cotype)) { //i.e. "USB Joystick"
			makeController(pubdata2, basedata2, ca[i]);
			shouldRun = true;
		}
	}

	if( !shouldRun )
		System.out.println("NO CONTROLLER MATCHING "+cotype+" FOUND, CONTROL THREAD WILL EXIT.");
	ThreadPoolManager.getInstance().init(new String[] {"SYSTEM"}, true);
	org.ros.internal.node.server.ThreadPoolManager.getInstance().spin(new Runnable() {
		public void run(){
			try {
				while(shouldRun){
					for(Iterator<ControllerManager> i=controllers.iterator();i.hasNext();){
						try {
								ControllerManager cw = (ControllerManager)i.next();
								cw.poll();
						} catch (Exception e) {
								e.printStackTrace();
						}
						Thread.sleep(DEFAULT_REFRESH);
					}
				}
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}, "SYSTEM");
}

private void makeController(ConcurrentHashMap<Identifier, Float> pubdata2, 	ConcurrentHashMap<Identifier,Float> basedata2, Controller c) {
	Controller[] subControllers = c.getControllers();
	if (subControllers.length == 0 ) {
		createController(pubdata2, basedata2, c);
	} else {
		for(int i=0;i<subControllers.length;i++){
			makeController(pubdata2, basedata2, subControllers[i]);
		}
	}
}

private void createController(ConcurrentHashMap<Identifier, Float> pubdata2, ConcurrentHashMap<Identifier,Float> basedata2, Controller c){
	if(DEBUG)
		System.out.println(this.getClass().getName()+".createController adding controller:"+c.getName()+" port:"+c.getPortNumber()+" to controller collection");
	controllers.add(new ControllerManager(pubdata2, basedata2, c));
}   

/**
 * Move the buffered values into the publishing message
 * @param connectedNode
 * @return
 */
private sensor_msgs.Joy setupPub(ConnectedNode connectedNode, float[] taxes, int[] tbuttons) {
	sensor_msgs.Joy val = connectedNode.getTopicMessageFactory().newFromType(sensor_msgs.Joy._TYPE);
	val.setAxes(taxes);
	val.setButtons(tbuttons);
	/*
	std_msgs.MultiArrayLayout vlayout = new std_msgs.MultiArrayLayout();
	List<std_msgs.MultiArrayDimension> vdim = new ArrayList<std_msgs.MultiArrayDimension>();
	std_msgs.MultiArrayDimension vdim1 = new std_msgs.MultiArrayDimension();
	std_msgs.MultiArrayDimension vdim2 = new std_msgs.MultiArrayDimension();
	// multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
	// thin data via curve fitting
	// Collect data.
	if( DEBUG )
		System.out.println("thinning "+valBuf.size());
	final WeightedObservedPoints obs = new WeightedObservedPoints();
	Integer xold = new Integer(Integer.MAX_VALUE);
	Integer yold = new Integer(Integer.MAX_VALUE);
	for(int i = 0; i < valBuf.size(); i+=2) {
		if( valBuf.get(i) != xold || valBuf.get(i+1) != yold) {
			obs.add(valBuf.get(i), valBuf.get(i+1));
			xold = valBuf.get(i);
			yold = valBuf.get(i+1);
		}
	}
	if( DEBUG) 
		System.out.println("Observer points: "+obs.toList().size());
	// Instantiate a third-degree polynomial fitter.
	final PolynomialCurveFitter fitter = PolynomialCurveFitter.create(3);
	// Retrieve fit parameters (coefficients of the polynomial function).
	final double[] coeff = fitter.fit(obs.toList());
	int[] vali = new int[coeff.length];
	for( double inv : coeff) vali[i++] = inf;
	if( DEBUG )
		System.out.println("Fit points:"+coeff.length);
	vdim2.setSize(coeff.length/2);
	vdim1.setStride(coeff.length);
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
	*/
	std_msgs.Header h = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
	h.setStamp(Time.fromMillis(System.currentTimeMillis()));
	val.setHeader(h);
	return val;
}

class fileReader implements Runnable {
	public volatile boolean shouldRun = true;
	
	@Override
	public void run() {
		while(shouldRun) {
		try {
			FileReader fis = new FileReader("/home/jg/coords");
			BufferedReader br = new BufferedReader(fis);
			String s = br.readLine();
			br.close();
			fis.close();
			System.out.println(s);
			
			String left = s.substring(0,s.indexOf(","));
			String right = s.substring(s.indexOf(",")+1);
			System.out.println(left+","+right);
			int l = Integer.parseInt(left,10);
			int r = Integer.parseInt(right,10);
			//pubdata.addLast(new int[]{l,r});
			Thread.sleep(5);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		}
	}
	
}
}
