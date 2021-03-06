package com.neocoretechs.robocore.marlinspike;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.neocoretechs.robocore.config.Robot;
import com.neocoretechs.robocore.config.RobotInterface;
import com.neocoretechs.robocore.config.TypedWrapper;
import com.neocoretechs.robocore.machine.bridge.TopicListInterface;

/**
 * Parse configs and allocate the necessary number of control elements for one or more Marlinspike boards
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class MarlinspikeManager {
	private static boolean DEBUG = true;
	RobotInterface robot;
	String hostName;
	int leftSlot = -1;
	int leftChannel = -1;
	int rightSlot = -1;
	int rightChannel = -1;
	TypedWrapper[] lun;
	TypedWrapper[] wheel;
	TypedWrapper[] pid;
	//Object[] nodeNames; // one of these for each subscriber to serve AsynchDemuxer and DataPortInterface
	//Object[] controllers; // one of these for each AsynchDemuxer and DataPort
	/**
	 * deviceToType - NodeDeviceDemuxer -> <DeviceName, i.e. "LeftWheel"-> TypeSlotChannelEnable>
	 */
	ConcurrentHashMap<NodeDeviceDemuxer, Map<String, TypeSlotChannelEnable>> deviceToType = new ConcurrentHashMap<NodeDeviceDemuxer, Map<String,TypeSlotChannelEnable>>();
	NodeDeviceDemuxer[] nodeDeviceDemuxerByLUN;
	/**
	 * 
	 * @param lun
	 * @param wheel
	 * @param pid 
	 */
	public MarlinspikeManager(RobotInterface robot2) {
		this.robot = robot2;
		this.lun = robot2.getLUN();
		this.wheel = robot2.getWHEEL();
		this.pid = robot2.getPID();
		this.hostName = robot2.getHostName();
		//nodeNames = aggregate(lun, "NodeName");
		//controllers = aggregate(lun,"Controller");
	}
	/**
	 * NodeNames are the subscription topics for the ROS bus to send commands to the Marlinspikes
	 * attached to specific controllers. 
	 * @return The array of Strings representing unique NodeNames from configs file in no particular order.
	 */
	//public Object[] getnodeNames() {
	//	return nodeNames;
	//}
	/**
	 * Controllers are the device names with attached Marlinspikes. In particular
	 * the tty ports that will have DataPorts assigned to read/write the serial data from Marlinspike.
	 * There will be one for each AsynchDemuxer.
	 * @return The unique array of Strings of ports in no particular order.
	 */
	//public Object getControllers() {
	//	return controllers;
	//}
	/**
	 * Create the controllers from the lun properties. Aggregate them by port.
	 * @param override true to ignore node-based limitations on config params
	 * @throws IOException
	 */
	private void createControllers(boolean override) throws IOException {
		NodeDeviceDemuxer ndd = null;
		nodeDeviceDemuxerByLUN = new NodeDeviceDemuxer[lun.length];
		for(int i = 0; i < lun.length; i++) {
			if(override || hostName.equals(lun[i].get("NodeName"))) {
				String name = (String)lun[i].get("Name");
				String controller = (String)lun[i].get("Controller");
				// NodeDeviceDemuxer identity is Controller, or tty, and our NodeName check makes them unique to this node
				// assuming the config is properly done
				ndd = new NodeDeviceDemuxer((String) lun[i].get("NodeName"), name, controller);
				nodeDeviceDemuxerByLUN[i] = ndd;
				Map<String, TypeSlotChannelEnable> nameToTypeMap = deviceToType.get(ndd);
				if(nameToTypeMap == null) {
					nameToTypeMap = new ConcurrentHashMap<String, TypeSlotChannelEnable>();
					deviceToType.put(ndd, nameToTypeMap);
				}
				// map within a map, nameToTypeMap, 
				// has deviceName, demuxer, indexed by name of device so "LeftWheel" can retrieve 
				// pins. etc. 
				Optional<Object> dir = Optional.ofNullable(lun[i].get("Direction"));
				Optional<Object> enable = Optional.ofNullable(lun[i].get("EnablePin"));
				int ienable = enable.isPresent() ? Integer.parseInt((String)enable.get()) : 0;
				// if pin2 is present we assume pin1 is present and we are using a 2 PWM pin type
				// if enable pin is not present we are probably using a smart controller, and we want our own
				// constructor for TypeSlotChannelEnable
				TypeSlotChannelEnable tsce = null;
				if(dir.isPresent()) {
					tsce = new TypeSlotChannelEnable((String)lun[i].get("Type"), 
							Integer.parseInt((String)lun[i].get("Slot")), 
							Integer.parseInt((String)lun[i].get("Channel")), 
							ienable, Integer.parseInt((String) dir.get()));
				} else {
					tsce = new TypeSlotChannelEnable((String)lun[i].get("Type"), 
							Integer.parseInt((String)lun[i].get("Slot")), 
							Integer.parseInt((String)lun[i].get("Channel")), 
							ienable);
				}
				nameToTypeMap.put(name, tsce);
			}
		}
		if(deviceToType.isEmpty())
			throw new IOException("No configuration information found for any controller for this node:"+hostName);
	}
	
	/**
	 * Imperatively activate the asynchDemuxers and DataPorts now that we have the controller data from luns.
	 * The map forEach directive will iterate the keys of deviceToType, which are NodeDeviceDemuxers in which the
	 * AsynchDemuxer will be instantiated and the DataPort will connect to the board assuming all goes well.
	 * @param config The initial M codes and parameters to configure the controller entries extracted from the configuration created with configureMarlinspike.
	 * @throws RuntimeException If we cant connect to the port.
	 */
	private void activateControllers() {
		deviceToType.forEach((key,value)->{
			try {
				if(DEBUG)
					System.out.printf("%s activating Marlinspike %s %s%n",this.getClass().getName(),key,value);
				key.activateMarlinspikes(this, deviceToType);
			} catch (IOException e) {
				throw new RuntimeException(e);
			}
		});
	}
	
	private void configControllers(ArrayList<String> config) {
		deviceToType.forEach((key,value)->{
			try {
				if(DEBUG)
					System.out.printf("%s Configuring Marlinspike %s %s%n",this.getClass().getName(),key,value);
				key.getAsynchDemuxer().config(config);
			} catch (IOException e) {
				throw new RuntimeException(e);
			}
		});
	}
	/**
	 * Perform the final configuration and issue the commands to the Marlinspike via the retrieved properties.<p/>
	 * Calls createControllers as first operation.
	 * @param override true to ignore node-specific parameter in loading configuration
	 * @param activate Optional parameter true to call activateControllers, which in override true may be undesirable, at the end of configuring, it calls activateControllers as final operation if activate is true.
	 * @throws IOException
	 */
	public void configureMarlinspike(boolean override, boolean... activate) throws IOException {
		createControllers(override);
		if(activate.length > 0 && activate[0])
			activateControllers();
		NodeDeviceDemuxer ndd = null;
		ArrayList<String> config = new ArrayList<String>();
		// We have to sift back through the lun properties to get the additional optional fields
		for(int i = 0; i < lun.length; i++) {
			if(hostName.equals(lun[i].get("NodeName"))) {
				String name = (String)lun[i].get("Name");
				String controller = (String)lun[i].get("Controller");
				// Extract the NodeDeviceDemuxer key from deviceToType by comparing to controller value we just extracted
				// this should be the activated instance with valid ASynchDemuxer and DataPort connected to tty, etc.
				Stream<Object> nddx = deviceToType.entrySet().stream().
						filter(entry -> controller.equals(entry.getKey().getDevice()) && name.equals(entry.getKey().getDeviceName())).map(Map.Entry::getKey);
				ndd = (NodeDeviceDemuxer) nddx.findFirst().get();
				Map<String, TypeSlotChannelEnable> nameToTypeMap = deviceToType.get(ndd);
				TypeSlotChannelEnable tsce = nameToTypeMap.get(name);
				Optional<Object> pin1 = Optional.ofNullable(lun[i].get("PWMPin1"));
				// if pin 1 is NOT present we assume we are using a type with NO pwm pins
				Optional<Object> pin0 = Optional.ofNullable(lun[i].get("PWMPin0"));
				Optional<Object> enc = Optional.ofNullable(lun[i].get("EncoderPin"));
				if(DEBUG) {
					System.out.printf("%s: Controller Name:%s device:%s ndd:%s tsce:%s%n",this.getClass().getName(), name, controller, ndd, tsce);
				}
				int ipin0=0,ipin1=0,ienc=0;
				if(pin0.isPresent())
					ipin0 = Integer.parseInt((String) pin0.get());
				if(pin1.isPresent())
					ipin1 = Integer.parseInt((String) pin1.get());
				if(enc.isPresent())
					ienc = Integer.parseInt((String) enc.get());
				String M10Gen = null;
				try {
					M10Gen = tsce.genM10();
					if(DEBUG)
						System.out.printf("%s: Controller Name:%s device:%s generating M10:%s%n",this.getClass().getName(), name, controller, M10Gen);
					config.add(M10Gen);
					StringBuilder sb = new StringBuilder(tsce.genTypeAndSlot()).append(tsce.genDrivePins(ipin0, ipin1)).append(tsce.genChannelDirDefaultEncoder(ienc));
					if(DEBUG) {
						System.out.printf("%s: Controller Name:%s device:%s generating config%s%n",this.getClass().getName(), name, controller,sb.toString());
					}
					config.add(sb.toString());
				} catch(NoSuchElementException nse) {
				}
			}
		}
		if(activate.length > 0 && activate[0] && !config.isEmpty())
			configControllers(config);
	}
	

	/**
	 * Locate the proper AsynchDemuxer by name of device we want to write to, then add the write to that AsynchDemuxer.
	 * @param name The name of the device in the config file, such as "LeftWheel", "RightWheel"
	 * @param req The request to queue
	 */
	public void addWrite(String name, String req) {
		AsynchDemuxer.addWrite(getNodeDeviceDemuxer(name).asynchDemuxer, req);
	}
	/**
	 * Get the class with methods that talk to the Marlinspike board by the descriptive name of the device
	 * as it appears in the config.
	 * @param name One of "Leftwheel", "RightWheel" etc.
	 * @return the MarlinspikeControlInterface with methods to communicate with the board
	 */
	public MarlinspikeControlInterface getMarlinspikeControl(String name) throws NoSuchElementException {
		Stream<Object> nddx = deviceToType.entrySet().stream().
				filter(entry -> name.equals(entry.getKey().getDeviceName())).map(Map.Entry::getKey);
		try {
			NodeDeviceDemuxer ndd = (NodeDeviceDemuxer) nddx.findAny().get();
			return ndd.getMarlinspikeControl();
		} catch(NullPointerException npe) {
			throw new NoSuchElementException("The device "+name+" was not found in the collection");
		}
	}
	/**
	 * Aggregate the numerical LUNS or AXIS etc by unique property
	 * @param wrapper The TypeWrapper array in config (LUN[], AXIS[], WHEEL[], etc).
	 * @param prop The property in the LUN to return unique values of
	 * @return the array of Sting properties retrieved
	 */
	private Object[] aggregate(TypedWrapper[] wrapper, String prop) {
		return Stream.of(wrapper).flatMap(map -> map.entrySet().stream()).filter(map->map.getKey().equals(prop)).distinct()
				.toArray();
				//.collect(Collectors.toMap(p -> p.getKey(), p -> p.getValue())).values().toArray();
	    		//.collect(Collectors.toList());
	}
	/**
	 * Get the set of NodeDeviceDemuxer which contains AsynchDemuxers for this node based on the set of
	 * TypeSlotChannelEnable.<p/>
	 * getNodeDeviceDemuxerByType(getTypeSlotChannelEnableByType(String type))
	 * @param tsce The set retrieved from one of the getTypeSlotChannelEnable methods
	 * @return The list of NodeDeviceDemuxer associated with the passed list of tsce
	 */
	//ConcurrentHashMap<NodeDeviceDemuxer, Map<String, TypeSlotChannelEnable>> deviceToType 
	public Collection<NodeDeviceDemuxer> getNodeDeviceDemuxerByType(Collection<TypeSlotChannelEnable> tsce) {
		return Stream.of(deviceToType).flatMap(map -> map.entrySet().stream()).filter(map->tsce.containsAll(map.getValue().values()))
				.map(e -> e.getKey() ).collect(Collectors.toCollection(ArrayList::new));
	}
	/**
	 * @param type One of SmartController, H-Bridge, SplitBridge, SwitchBridge, PWM etc.
	 * @return All devices of a particular type attached to the Marlinspike on this node
	 */
	public Collection<TypeSlotChannelEnable> getTypeSlotChannelEnableByType(String type) throws NoSuchElementException {
		Collection<TypeSlotChannelEnable> ret = new ArrayList<TypeSlotChannelEnable>();
		Iterator<Map<String,TypeSlotChannelEnable>> it = deviceToType.values().iterator();
		while(it.hasNext()){
			Map<String,TypeSlotChannelEnable> tsces = it.next();
			Iterator<TypeSlotChannelEnable> itx = tsces.values().iterator();
			while(itx.hasNext()) {
				TypeSlotChannelEnable tsce = itx.next();
				if(tsce.getType().equals(type))
					ret.add(tsce);
			}
		}
		if(ret.isEmpty())
			throw new NoSuchElementException("The type "+type+" was not found in the collection of device to type");
		return ret;
	}
	
	/**
	 * @param type One of "LiftActuator" "LeftWheel" etc.
	 * @return device of a particular name, which points to the actual device of a particular type, attached to the Marlinspike on this node
	 */
	public TypeSlotChannelEnable getTypeSlotChannelEnableByName(String name) throws NoSuchElementException {
		Iterator<Map<String,TypeSlotChannelEnable>> it = deviceToType.values().iterator();
		while(it.hasNext()){
			Map<String,TypeSlotChannelEnable> tsces = it.next();
			TypeSlotChannelEnable ret = tsces.get(name);
				return ret;
		}
		throw new NoSuchElementException("The name "+name+" was not found in the collection of device to type");
	}
	/**
	 * SmartController, H-Bridge, SplitBridge, SwitchBridge, PWM etc.
	 * @return All the devices attached to all the Marlinspikes on this node
	 */
	public Collection<TypeSlotChannelEnable> getTypeSlotChannelEnable() {
		Collection<TypeSlotChannelEnable> tsce = new ArrayList<TypeSlotChannelEnable>();
		Stream.of(deviceToType).flatMap(map -> map.entrySet().stream())
				.map(map->map.getValue().values())
				.forEach(tsce::addAll);
		return tsce;
	}
	
	public NodeDeviceDemuxer getNDDByLUN(int lun) {
		return nodeDeviceDemuxerByLUN[lun];
	}
	/**
	 * Get the NodeDeviceDemuxer from DeviceToType
	 * @param string The DeviceName, i.e. "LeftWheel"
	 * @return
	 */
	public NodeDeviceDemuxer getNodeDeviceDemuxer(String name) throws NoSuchElementException  {
		Stream<Object> nddx = deviceToType.entrySet().stream().
				filter(entry -> name.equals(entry.getKey().getDeviceName())).map(Map.Entry::getKey);
		try {
			return (NodeDeviceDemuxer) nddx.findAny().get();
		} catch(NullPointerException nse) {
			throw new NoSuchElementException("The device "+name+" was not found in the collection");
		}
	}

	
	public String toString() {
		return String.format("Controller LeftSlot=%d, Left Channel=%d, RightSlot=%d Right Channel=%d\r\n",
				leftSlot, leftChannel, rightSlot, rightChannel);
	}
	
	public static void main(String[] args) throws IOException {
		RobotInterface robot = new Robot();
		MarlinspikeManager mm = new MarlinspikeManager(robot);
		mm.createControllers(true);
		System.out.println("NodeDeviceDemuxer for LeftWheel:"+mm.getNodeDeviceDemuxer("LeftWheel"));
		System.out.println("NodeDeviceDemuxer for RightWheel:"+mm.getNodeDeviceDemuxer("RightWheel"));
		System.out.println("NodeDeviceDemuxer for BoomActuator:"+mm.getNodeDeviceDemuxer("BoomActuator"));
		System.out.println("NodeDeviceDemuxer for LEDDriver:"+mm.getNodeDeviceDemuxer("LEDDriver"));
		try {
			System.out.println("MarlinspikeControl for BogusWheel:"+mm.getNodeDeviceDemuxer("BogusWheel"));
		} catch(NoSuchElementException nse) { System.out.println("passed");}
		Stream.of(mm.deviceToType).flatMap(map -> map.entrySet().stream()).forEach(e -> System.out.println(e));
		Collection<TypeSlotChannelEnable> tsce = mm.getTypeSlotChannelEnable();
		System.out.println("-----");
		for(TypeSlotChannelEnable tt : tsce)
			System.out.println(tt);
		System.out.println("-----");
		Collection<NodeDeviceDemuxer> listNodeDeviceDemuxer = mm.getNodeDeviceDemuxerByType(tsce);
		System.out.println("-----");
		for(NodeDeviceDemuxer ndd : listNodeDeviceDemuxer)
			System.out.println(ndd);
		System.out.println("-----");
		System.out.println("MarlinspikeControl for LeftWheel:"+mm.getMarlinspikeControl("LeftWheel"));
		System.out.println("MarlinspikeControl for RightWheel:"+mm.getMarlinspikeControl("RightWheel"));
		try {
		System.out.println("MarlinspikeControl for BogusWheel:"+mm.getMarlinspikeControl("BogusWheel"));
		} catch(NoSuchElementException nse) { System.out.println("passed");}
		System.out.println("-----");
		//for(Object n : mm.aggregate(mm.lun, "NodeName"))
		//	System.out.println(n);
		//System.out.println("----");
		//for(Object n : mm.aggregate(mm.lun,"Controller"))
		//	System.out.println(n);
		//System.out.println("----");
		for(int i = 0; i < mm.lun.length; i++) {
			System.out.println(i+".)"+mm.lun[i].get("Name")+","+mm.lun[i].get("NodeName")+","+mm.lun[i].get("Controller")+","+
		mm.lun[i].get("Type")+","+mm.lun[i].get("Slot")+","+mm.lun[i].get("PWMPin0")+","+mm.lun[i].get("Channel")+","+
					mm.lun[i].get("EnablePin")+","+mm.lun[i].get("Direction")+","+mm.lun[i].get("EncoderPin"));
		}
	}


}
