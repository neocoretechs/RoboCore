package com.neocoretechs.robocore;

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
import org.ros.node.topic.Subscriber;
import org.ros.internal.loader.CommandLineLoader;


/**
 * Publishes user constructed Twist messages on the ardrone and ground unit topics or direct motor speed differential
 * drive params on absolute/cmd_vel. Associate PS3 controller axis made up of components like digital hats and
 * analog axis, digital axis. 
 * Identifiers:
 * 
 *x		Left stick (left/right)
 *y		Left stick (up/down)
 *rx		N/A	 
 *ry		N/A	 
 *z	 	Right stick (left/right)
 *rz		Right stick (up/down)
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
 * @author jg
 */
public class PS3CombinedPubs extends AbstractNodeMain  {
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
	
	public PS3CombinedPubs(String host, InetSocketAddress master) {
		this.host = host;
		this.master = master;
		NodeConfiguration nc = build();
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, nc);
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	    
	}
	
	public PS3CombinedPubs(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	}
	
	public PS3CombinedPubs() {}
	
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
			if( !pubdata.isEmpty() ) {
			  if(pubdata.containsKey(Component.Identifier.Axis.Y) &&
				 pubdata.containsKey(Component.Identifier.Axis.RZ))  {
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
		    if(pubdata.containsKey(Component.Identifier.Axis.POV)) {
				  Float data = pubdata.get(Component.Identifier.Axis.POV);
					if (data == Component.POV.OFF){
						//if(DEBUG)System.out.println("POV OFF");
					} else if ( data == Component.POV.UP) {
						if(DEBUG)System.out.println("POV Up");
						std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						forpub.publish(val);
					} else if ( data == Component.POV.UP_RIGHT) {
						if(DEBUG)System.out.println("POV Up_Right");
						std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						uppub.publish(val);
					} else if ( data == Component.POV.RIGHT) {
						if(DEBUG)System.out.println("POV Right");
						std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						gorpub.publish(val);
					} else if ( data == Component.POV.DOWN_RIGHT) {
						if(DEBUG)System.out.println("POV Down Right");
						std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						downpub.publish(val);
					} else if ( data == Component.POV.DOWN) {
						if(DEBUG)System.out.println("POV Down");
						std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						bakpub.publish(val);
					} else if ( data == Component.POV.DOWN_LEFT) {
						if(DEBUG)System.out.println("POV Down left");
						std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						downpub.publish(val);
					} else if ( data == Component.POV.LEFT) {
						if(DEBUG)System.out.println("POV Left");
						std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						golpub.publish(val);
					} else if ( data == Component.POV.UP_LEFT) {
						if(DEBUG)System.out.println("POV Up Left");
						std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						uppub.publish(val);
					} else { // should never happen
						if(DEBUG)System.out.println("POV IMPOSSIBLE!!");
					}		 
			  } // POV
			  if(pubdata.containsKey(Component.Identifier.Button.BASE)) { // left trigger
					 Float data = pubdata.get(Component.Identifier.Button.BASE);
					 if( data != 0) {
						 if( DEBUG )System.out.println("Base:"+data);
						 std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						 spinlpub.publish(val);
					 }
			  } 
			  if(pubdata.containsKey(Component.Identifier.Button.BASE2)) {
					Float data = pubdata.get(Component.Identifier.Button.BASE2);
					if( data != 0) {
						if( DEBUG )System.out.println("Base2:"+data);
						std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
						spinrpub.publish(val);
					}
			  }
			  if(pubdata.containsKey(Component.Identifier.Button.BASE3)) {
					Float data = pubdata.get(Component.Identifier.Button.BASE3);
					if( data != 0) {
						if( DEBUG )System.out.println("Base3:"+data);
					}
			  } 
			  if(pubdata.containsKey(Component.Identifier.Button.BASE4)) {
					Float data = pubdata.get(Component.Identifier.Button.BASE4);
					if( data != 0) {
						if( DEBUG ) System.out.println("Base4:"+data);
					}
			  }
			  if(pubdata.containsKey(Component.Identifier.Button.BASE5)) {
					Float data = pubdata.get(Component.Identifier.Button.BASE5);
					if( data != 0) {
						if( DEBUG )System.out.println("Base5:"+data);
					}
			  }
			  if(pubdata.containsKey(Component.Identifier.Button.BASE6)) {
				  Float data = pubdata.get(Component.Identifier.Button.BASE6);
				  if( data != 0) {
					  if(DEBUG)System.out.println("Base6:"+data);
				  }
			  }
			  if(pubdata.containsKey(Component.Identifier.Button.TRIGGER)) {
				  Float data = pubdata.get(Component.Identifier.Button.TRIGGER);
				  if( data != 0){
					  if(DEBUG)System.out.println("Trigger:"+data);
				  }
			  }
			  if(pubdata.containsKey(Component.Identifier.Button.THUMB)) { // circle
				  Float data = pubdata.get(Component.Identifier.Button.THUMB);
				  if( data != 0) {
					  if(DEBUG)System.out.println("Thumb:"+data);
					  std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
					  tofpub.publish(val);
				  }
			  }
			  if(pubdata.containsKey(Component.Identifier.Button.THUMB2)) { // X
				  Float data = pubdata.get(Component.Identifier.Button.THUMB2);
				  if( data != 0) {
					  if( DEBUG )System.out.println("Thumb2:"+data);
					  std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
					  lanpub.publish(val);
				  }
			  }
			  if(pubdata.containsKey(Component.Identifier.Button.TOP)) {
				  Float data = pubdata.get(Component.Identifier.Button.TOP);
				  if( data != 0)System.out.println("Top:"+data);
			  }
			  if(pubdata.containsKey(Component.Identifier.Button.TOP2)) {  //left bumper
				  Float data = pubdata.get(Component.Identifier.Button.TOP2);
				  if( data != 0) {
					  if( DEBUG )System.out.println("Top2:"+data);
					  std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
					  freezepub.publish(val);
				  }
			  }
			  if(pubdata.containsKey(Component.Identifier.Button.PINKIE)) { // right bumper
				  Float data = pubdata.get(Component.Identifier.Button.PINKIE);
				  if( data != 0) {
					  if( DEBUG )System.out.println("Pinkie:"+data);
					  std_msgs.Empty val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Empty._TYPE);
					  hovpub.publish(val);
				  }
			  }
		  } // pubdata not empty
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
	public boolean shouldRun = true;
	
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
