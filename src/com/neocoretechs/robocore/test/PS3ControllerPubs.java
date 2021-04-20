package com.neocoretechs.robocore.test;

import net.java.games.input.Component;
import net.java.games.input.Component.Identifier;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Version;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
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

import org.ros.internal.node.server.ThreadPoolManager;


/**
 * Publishes user constructed Twist messages on the cmd_vel topic or direct motor speed differential
 * drive params on absolute/cmd_vel. Associate PS3 controller axis made up of components like digital hats and
 * analog axis, digital axis. 
 * Identifiers:
 * 
 * POV	Directional Pad	
 * x	Left stick (left/right)	
 * y	Left stick (up/down)	
 * rx	Right stick (left/right)
 * ry	Right stick (up/down) 
 * z	-1 to 0 Right trigger	1-0Left trigger 
 * rz	N/A?
 * Button 0	x
 * Button 1	circle
 * Button 2	square
 * Button 3	triangle
 * Button 4	Bumper	L1
 * Button 5	Bumper	R1
 * Button 6	Select
 * Button 7	Start
 * Button 8	Stick Press
 * Button 9	Stick Press
 * 
 * @author jg
 */
public class PS3ControllerPubs extends AbstractNodeMain  {
	private static final boolean DEBUG = true;
	private String host;
	private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);
	geometry_msgs.Twist twistmsg = null;
	public ConcurrentHashMap<Identifier, Float> pubdata = new ConcurrentHashMap<Identifier,Float>();
	static final long HEARTBEATMS =100; // 10th of a second
	List<ControllerManager> controllers = new ArrayList<ControllerManager>();

	final static int motorSpeedMax = 1000;
	// scale it to motor speed
	
	public PS3ControllerPubs(String host, InetSocketAddress master) {
		this.host = host;
		this.master = master;
		NodeConfiguration nc = build();
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, nc);
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	    
	}
	
	public PS3ControllerPubs(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	}
	
	public PS3ControllerPubs() {}
	
	public GraphName getDefaultNodeName() {
		return GraphName.of("pubs_ps3unit1");
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
	//fileReader reader = new fileReader();
	//ThreadPoolManager.getInstance().spin(reader, "SYSTEM");
	//final RosoutLogger log = (Log) connectedNode.getLog();
	
	final Publisher<std_msgs.Int32MultiArray> velpub =
		connectedNode.newPublisher("absolute/cmd_vel", std_msgs.Int32MultiArray._TYPE);
	
	// Start reading from serial port
	// check command line remappings for __mode:=startup to issue the startup code to the attached processor
	// ONLY DO IT ONCE ON INIT!
	//Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
	//String mode="";
	//if( remaps.containsKey("__mode") )
	//	mode = remaps.get("__mode");
	//if( mode.equals("startup")) {
	//	try {
			ControllerReader(pubdata);
	//	} catch (IOException e) {
	//	System.out.println("Could not start process to read attached controller.."+e);
	//		e.printStackTrace();
	//		return;
	//	}
	//} else {
	//	AsynchDemuxer.getInstance();	
	//}
	// This CancellableLoop will be canceled automatically when the node shuts
	// down.
	connectedNode.executeCancellableLoop(new CancellableLoop() {
		private int sequenceNumber;
		private boolean deadStick = false;
		int[] vali;
		@Override
		protected void setup() {
			sequenceNumber = 0;
		}

		@Override
		protected void loop() throws InterruptedException {
			if( !pubdata.isEmpty() && (pubdata.containsKey(Component.Identifier.Axis.Y) &&
									  pubdata.containsKey(Component.Identifier.Axis.RZ)) ) {
				std_msgs.Int32MultiArray val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Int32MultiArray._TYPE);
				Float flj = pubdata.get(Component.Identifier.Axis.Y);
				Float frj = pubdata.get(Component.Identifier.Axis.RZ);
				if( flj == 0 && frj == 0 ) {
					if( !deadStick ) {
						vali = (new int[]{0,0});
						val.setData(vali);
						velpub.publish(val);
						if( DEBUG ) 
							System.out.println("Published "+flj+","+frj);
					}	
					deadStick = true;
				} else {
					deadStick = false;
					vali = (new int[]{(int)-(flj*1000), (int)-(frj*1000)});
					val.setData(vali);
					velpub.publish(val);
					if( DEBUG ) 
						System.out.println("Published "+flj+","+frj);
				}
			}
			Thread.sleep(5);
		}
			
	});
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
	public DigitalAxis(ConcurrentHashMap<Identifier, Float> pubdata2, Component ax) {
		super(ax);
		this.pubdata2 = pubdata2;
	}

	protected void renderData(){
		if( pubdata2.containsKey(getAxis().getIdentifier())) {
			//if( DEBUG )
			//	System.out.println("DigitalAxis replacing "+getAxis().getIdentifier());
			pubdata2.replace(getAxis().getIdentifier(), data);
		} else {
			if( DEBUG )
				System.out.println("DigitalAxis putting "+getAxis().getIdentifier());
			pubdata2.put(getAxis().getIdentifier(), data);
		}
	}
}

private static class DigitalHat extends Axis {
	ConcurrentHashMap<Identifier,Float> pubdata2;
	public DigitalHat(ConcurrentHashMap<Identifier,Float> pubdata2, Component ax) {
		super(ax);
		this.pubdata2 = pubdata2;
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
			//if( DEBUG )
			//	System.out.println("DigitalHat replacing "+getAxis().getIdentifier());
			pubdata2.replace(getAxis().getIdentifier(), data);
		} else {
			if( DEBUG )
				System.out.println("DigitalHat putting "+getAxis().getIdentifier());
			pubdata2.put(getAxis().getIdentifier(), data);
		}
		
	}
}

private static class AnalogAxis extends Axis {
	ConcurrentHashMap<Identifier, Float> pubdata2;
	public AnalogAxis(ConcurrentHashMap<Identifier, Float> pubdata2, Component ax) {
		super(ax);
		this.pubdata2 = pubdata2;
	}

	protected void renderData() {
		if (getAxis().getDeadZone() >= Math.abs(data)) {
			//" (DEADZONE)";
			// add at least one power 0 shutdown at dead zone if not there and we are here
			data = 0;
		}
		if( pubdata2.containsKey(getAxis().getIdentifier())) {
			//if( DEBUG )
			//	System.out.println("AnalogAxis replacing "+getAxis().getIdentifier());
			pubdata2.replace(getAxis().getIdentifier(), data);
		} else {
			if( DEBUG )
				System.out.println("AnalogAxis putting "+getAxis().getIdentifier());
			pubdata2.put(getAxis().getIdentifier(), data);
		}
		// add the val
		//pubdata2.addLast(new int[]{(int) -(data*1000)}); // invert and scale
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

	public ControllerManager(ConcurrentHashMap<Identifier, Float> pubdata2, Controller ca){
		this.pubdata2 = pubdata2;
		this.ca = ca;
		Component[] components = ca.getComponents();
		if( DEBUG )
			System.out.println("Controller:"+ca.getName()+" Component count = "+components.length);
		if (components.length>0) {
			for(int j=0;j<components.length;j++){
				addAxis(pubdata2, components[j]);
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

	private void addAxis(ConcurrentHashMap<Identifier,Float> pubdata2, Component ax){
		Axis p2;
		if (ax.isAnalog()) {
			p2 = new AnalogAxis(pubdata2, ax);
		} else {
			if (ax.getIdentifier() == Component.Identifier.Axis.POV) {
				p2 = new DigitalHat(pubdata2, ax);
			} else {     
				p2 = new DigitalAxis(pubdata2, ax);
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
 * by name from map. We look specifically for controllers with "USB Joystick" or we get keyboard, mouse, etc as well.
 * @param pubdata2
 */
public void ControllerReader(ConcurrentHashMap<Identifier, Float> pubdata2) {
	if( DEBUG )
		System.out.println("Controller version: " + Version.getVersion());
	ControllerEnvironment ce = ControllerEnvironment.getDefaultEnvironment();
	Controller[] ca = ce.getControllers();
	for(int i =0;i<ca.length;i++){
		if( ca[i].getName().contains("USB Joystick"))
			makeController(pubdata2, ca[i]);
	}
	ThreadPoolManager.getInstance().init(new String[] {"SYSTEM"}, true);
	ThreadPoolManager.getInstance().spin(new Runnable() {
		public void run(){
			try {
				while(true){
					for(Iterator<ControllerManager> i=controllers.iterator();i.hasNext();){
						try {
							ControllerManager cw = (ControllerManager)i.next();
							cw.poll();
						} catch (Exception e) {
							e.printStackTrace();
						}
					}
					Thread.sleep(HEARTBEATMS);
				}
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}, "SYSTEM");
}

private void makeController(ConcurrentHashMap<Identifier, Float> pubdata2, Controller c) {
	Controller[] subControllers = c.getControllers();
	if (subControllers.length == 0 ) {
		createController(pubdata2, c);
	} else {
		for(int i=0;i<subControllers.length;i++){
			makeController(pubdata2, subControllers[i]);
		}
	}
}

private void createController(ConcurrentHashMap<Identifier, Float> pubdata2, Controller c){
	controllers.add(new ControllerManager(pubdata2, c));
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
