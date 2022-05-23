package com.neocoretechs.robocore.marlinspike;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Enumeration;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.neocoretechs.robocore.config.DeviceEntry;
import com.neocoretechs.robocore.config.Robot;
import com.neocoretechs.robocore.config.RobotInterface;
import com.neocoretechs.robocore.config.TypedWrapper;
import com.neocoretechs.robocore.marlinspike.TypeSlotChannelEnable.typeNames;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;
import com.neocoretechs.robocore.serialreader.MarlinspikeDataPort;
import com.neocoretechs.robocore.serialreader.marlinspikeport.Pins;

/**
 * Each individual robot uses this class to manage its collection of attributes initially parsed from config file, including
 * LUN, WHEEL, PID. Further configuration is performed to deliver needed services.<p/>
 * Parse configs and allocate the necessary number of control elements for one or more Marlinspike boards.<p/>
 * We create the collection of devices and {@link NodeDeviceDemuxer} with their {@link AsynchDemuxer} to talk to the attached Marlinspikes.
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class MarlinspikeManager {
	private static boolean DEBUG = false;
	RobotInterface robot;
	String hostName;
	TypedWrapper[] lun;
	TypedWrapper[] wheel;
	TypedWrapper[] pid;
	//Object[] nodeNames; // one of these for each subscriber to serve AsynchDemuxer and DataPortInterface
	//Object[] controllers; // one of these for each AsynchDemuxer and DataPort
	/**
	 * deviceToType - [NodeDeviceDemuxer by Controller i.e. /dev/tty2] -> <Name of device, i.e. "LeftWheel"-> TypeSlotChannelEnable>
	 */
	ConcurrentHashMap<NodeDeviceDemuxer, Map<String, TypeSlotChannelEnable>> deviceToType = new ConcurrentHashMap<NodeDeviceDemuxer, Map<String,TypeSlotChannelEnable>>();
	/**
	 * devices - canonical list of devices by properties file device name, such as "LeftWheel". Will also carry NodeDeviceDemuxer ref
	 */
	ArrayList<DeviceEntry> devices = new ArrayList<DeviceEntry>();
	
	/**
	 * Each individual robot uses this class to manage its collection of attributes initially parsed from config file, including
	 * LUN, WHEEL, PID. Further configuration is performed to deliver needed services.
	 * @param robot The RobotInterface from which parsed config directives are acquired.
	 */
	public MarlinspikeManager(RobotInterface robot) {
		this.robot = robot;
		this.lun = robot.getLUN();
		this.wheel = robot.getWHEEL();
		this.pid = robot.getPID();
		this.hostName = robot.getHostName();
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
	 * Perform the final configuration and issue the commands to the Marlinspike via the retrieved properties.<p/>
	 * Calls createControllers as first operation.
	 * @param override true to ignore node-specific parameter in loading configuration
	 * @param activate Optional parameter true to call activateControllers, which in override true may be undesirable, at the end of configuring, it calls activateControllers as final operation if activate is true.
	 * @throws IOException
	 */
	public synchronized void configureMarlinspike(boolean override, boolean activate) throws IOException {
		createControllers(override, activate);
	}
	
	/**
	 * Create the controllers from the lun properties. Aggregate them by port.<p/>
	 * Each unique NODENAME and CONTROLLER for each LUN should have its own demuxer.<p/>
	 * So each unique serial port attached to microcontroller, for example, needs it own message handing demuxer.<p/>
	 * @param override true to ignore node-based limitations on config params. All params loaded regardless of node name for testing, for example.
	 * @param activate 
	 * @throws IOException
	 */
	private synchronized void createControllers(boolean override, boolean activate) throws IOException {
		NodeDeviceDemuxer ndd = null;
		for(int i = 0; i < lun.length; i++) {
			if(override || hostName.equals(lun[i].get("NodeName"))) {
				String name = (String)lun[i].get("Name");
				if(name == null)
					throw new IOException("Must specify Name paramater in configuration file for host "+hostName);
				String controller = (String)lun[i].get("Controller");
				if(controller == null)
					throw new IOException("Must specify Controller paramater in configuration file for host "+hostName+" Name:"+name);
				DeviceEntry deviceEntry = new DeviceEntry(name, (String) lun[i].get("NodeName"), i, controller);
				devices.add(deviceEntry);
				// NodeDeviceDemuxer identity is Controller, or tty, and our NodeName check makes them unique to this node
				// assuming the config is properly done
				NodeDeviceDemuxer nddx = new NodeDeviceDemuxer((String) lun[i].get("NodeName"), name, controller);
				Map<String, TypeSlotChannelEnable> nameToTypeMap = null; // map value
				Iterator<Map.Entry<NodeDeviceDemuxer, Map<String, TypeSlotChannelEnable>>> iterator = deviceToType.entrySet().iterator();  
		        boolean isKeyPresent = false; 
		        // Iterate over the HashMap
		        while (iterator.hasNext()) {
		        	Map.Entry<NodeDeviceDemuxer, Map<String, TypeSlotChannelEnable>> entry = iterator.next();
		            if(nddx.equals(entry.getKey())) {
		                isKeyPresent = true;
		                ndd = entry.getKey();
		                nameToTypeMap = entry.getValue();
		                break;
		            }
		        }
		        if(!isKeyPresent) { // use new values
		        	//nameToTypeMap = deviceToType.get(ndd);
		        	// Have we created a demuxer for this device controller already?
		        	//if(nameToTypeMap == null) {
					nameToTypeMap = new ConcurrentHashMap<String, TypeSlotChannelEnable>();
					ndd = nddx; // our template with key, now permanent
					deviceToType.put(ndd, nameToTypeMap);
					// activate the new demuxer, but only once
					if(activate)
						activateMarlinspike(ndd);
				}
		        deviceEntry.setNodeDeviceDemuxer(ndd);
				// map within a map, nameToTypeMap, 
				// has deviceName, demuxer, indexed by name of device so "LeftWheel" can retrieve 
				// pins. etc. 
				Optional<Object> dir = Optional.ofNullable(lun[i].get("Direction"));
				Optional<Object> enable = Optional.ofNullable(lun[i].get("EnablePin"));
				int ienable = enable.isPresent() ? Integer.parseInt((String)enable.get()) : 0;
				// standalone controls?
				// if pin2 is present we assume pin1 is present and we are using a 2 PWM pin type
				// if enable pin is not present we are probably using a smart controller, and we want our own
				// constructor for TypeSlotChannelEnable
				TypeSlotChannelEnable tsce = null;
				String type = (String)lun[i].get("Type");
				if(type == null)
					throw new IOException("Must specify Type paramater in configuration file for host "+hostName+" Name:"+name+" Controller:"+controller);
				typeNames eType = null;
				//eType = typeNames.valueOf(type) will fail because we use mixed case names;
				for(typeNames et : typeNames.values()) {
					if(et.name.equals(type)) {
						eType = et;
						break;
					}		
				}
				if(eType == null) {
					//Arrays.toString(typeNames.values()) gives enum types, not values of names
					StringBuilder sb = new StringBuilder();
					for(typeNames et : typeNames.values())
						sb.append(et.val()+" ");
					throw new IOException("Type paramater in configuration file for host "+hostName+" Name:"+name+" Controller:"+controller+" Type:"+type+" must be one of "+sb.toString());
				}
				if(!type.endsWith("Pin")) { // such as InputPin, OutputPin. i.e. a more sophisticated control with multiple constructor args
					String slot = (String)lun[i].get("Slot");
					if(slot == null)
						throw new IOException("Must specify Slot paramater in configuration file for host "+hostName+" Name:"+name+" Controller:"+controller+" Type:"+type);
					String channel = (String)lun[i].get("Channel");
					if(channel == null)
						throw new IOException("Must specify Channel paramater in configuration file for host "+hostName+" Name:"+name+" Controller:"+controller+" Type:"+type);
					if(dir.isPresent()) {
						tsce = new TypeSlotChannelEnable(eType, 
								Integer.parseInt(slot), 
								Integer.parseInt(channel), 
								ienable, Integer.parseInt((String) dir.get()));
					} else {
						tsce = new TypeSlotChannelEnable(eType, 
								Integer.parseInt(slot), 
								Integer.parseInt(channel), 
								ienable);
					}
				} else { // type of Pin, so get actual pin number and construct TypeSlotChannelEnable accordingly
					String pin = (String)lun[i].get("Pin");
					if(pin == null)
						throw new IOException("Must specify Pin paramater in configuration file for host "+hostName+" Name:"+name+" Controller:"+controller+" Type:"+type);
					tsce = new TypeSlotChannelEnable(eType, Integer.parseInt(pin));
					//Toggle:True,true,TRUE False,false,FALSE or absent
					Optional<Object> oToggle = Optional.ofNullable(lun[i].get("Toggle"));
					if(oToggle.isPresent() && Boolean.parseBoolean((String)lun[i].get("Toggle")))
							tsce.setPinToggle();
				}
				// general min and max values
				Optional<Object> ominValue = Optional.ofNullable(lun[i].get("Min"));
				Optional<Object> omaxValue = Optional.ofNullable(lun[i].get("Max"));
				if(ominValue.isPresent())
					tsce.setMinValue(Integer.parseInt((String)lun[i].get("Min")));
				if(omaxValue.isPresent())
					tsce.setMaxValue(Integer.parseInt((String)lun[i].get("Max")));
				nameToTypeMap.put(name, tsce);
				// Configure the demuxer with the type/slot/channel and aggregate the init commands for final init
				if(activate)
					configureMarlinspike(ndd, lun[i], tsce);
			}
		}
		if(deviceToType.isEmpty())
			throw new IOException("No configuration information found for any controller for this node:"+hostName);
		Enumeration<NodeDeviceDemuxer> k = deviceToType.keys();
		while(k.hasMoreElements()) {
			if(DEBUG)
				System.out.printf("%s preparing to init %s%n",  this.getClass().getName(), k);
			k.nextElement().init();
		}
	}
	
	/**
	 * Active the asynchDemuxer for the given Marlinspike if it has not been previously
	 * activated. We must ensure that 1 demuxer/device is activated for a particular physical port
	 * and that subsequent attempts at activation are met with an assignment 
	 * to an existing instance of asynchDemuxer.
	 * @param marlinspikeManager 
	 * @param deviceToType The mapping of all NodeDeviceDemuxer to all the TypeSlotChannels
	 * @throws IOException If we attempt to re-use a port, we box up the runtime exception with the IOException
	 */
	public synchronized void activateMarlinspike(NodeDeviceDemuxer ndd) throws IOException {
		if(DEBUG)
			System.out.printf("%s.activateMarlinspike preparing to initialize %s%n",this.getClass().getName(), ndd);
		AsynchDemuxer asynchDemuxer = new AsynchDemuxer(this);
		if(ndd.getDevice().equals("MarlinspikeDataPort"))
			asynchDemuxer.connect(new MarlinspikeDataPort());
		else
			asynchDemuxer.connect(new ByteSerialDataPort(ndd.getDevice()));
		ndd.setAsynchDemuxer(asynchDemuxer);
		ndd.setMarlinspikeControl(new MarlinspikeControl(asynchDemuxer));
	}
	
	/**
	 * Internal configuration generator that sets up initial commands to Marlinspike controller
	 * based on configuration, to prepare to send them to the attached controller.<p/>
	 * Called as final phase of initialization pipeline.
	 * @param ndd NodeDeviceDemuxer that handles IO
	 * @param lun Logical unit from configuration file
	 * @param tsce The type, slot, and channel designator that defines hierarchy of Marlinspike controller devices
	 * @throws IOException If commands fail in send or confirmation
	 */
	private synchronized void configureMarlinspike(NodeDeviceDemuxer ndd, TypedWrapper lun, TypeSlotChannelEnable tsce) throws IOException {
		Optional<Object> pin1 = Optional.ofNullable(lun.get("SignalPin1"));
		Optional<Object> pin0 = Optional.ofNullable(lun.get("SignalPin0"));
		Optional<Object> enc = Optional.ofNullable(lun.get("EncoderPin"));
		Optional<Object> encType = Optional.ofNullable(lun.get("EncoderType"));
		Optional<Object> encCount = Optional.ofNullable(lun.get("EncoderCount"));
		Optional<Object> encInterrupt = Optional.ofNullable(lun.get("EncoderInterrupt"));
		Optional<Object> aPollRate = Optional.ofNullable(lun.get("AnalogPollRate"));
		Optional<Object> aChangeDelta = Optional.ofNullable(lun.get("AnalogChangeDelta"));
		
		if(aPollRate.isPresent())
			Pins.setAnalogPollRate(Integer.parseInt((String)aPollRate.get()));
		if(aChangeDelta.isPresent())
			Pins.setAnalogChangeDelta(Integer.parseInt((String)aChangeDelta.get()));		
		int ipin0=0,ipin1=0,ienc=0;
		if(pin0.isPresent())
			ipin0 = Integer.parseInt((String) pin0.get());
		if(pin1.isPresent())
			ipin1 = Integer.parseInt((String) pin1.get());
		if(enc.isPresent()) {
			ienc = Integer.parseInt((String) enc.get());
			tsce.setEncoderPin(ienc);
			if(encType.isPresent() && encType.get().equals("Analog")) {
				int iencCount = 1;
				if(encCount.isPresent())
					iencCount = Integer.parseInt((String)encCount.get());
				int iencLoRange = 0, iencHiRange = 0;
				if(Optional.ofNullable(lun.get("EncoderLoRange")).isPresent())
					iencLoRange = Integer.parseInt((String) lun.get("EncoderLoRange"));
				if(Optional.ofNullable(lun.get("EncoderHiRange")).isPresent())
					iencHiRange = Integer.parseInt((String) lun.get("EncoderHiRange"));
				int iencInterrupt = 0;
				if(encInterrupt.isPresent())
					iencInterrupt = Integer.parseInt((String) encInterrupt.get());
				tsce.setAnalogEncoder(iencCount, iencLoRange, iencHiRange, iencInterrupt);
			} else {
				if(encType.isPresent() && encType.get().equals("Digital")) {
					int iencCount = 1;
					int iencState = 1; // high
					String encState = null;
					if(encCount.isPresent())
						iencCount = Integer.parseInt((String)encCount.get());
					if(Optional.ofNullable(lun.get("EncoderState")).isPresent()) {
						encState = (String) lun.get("EncoderState");
						iencState = encState.equals("LOW") ? 0 : 1;
					}
					int iencInterrupt = 0;
					if(encInterrupt.isPresent())
						iencInterrupt = Integer.parseInt((String) encInterrupt.get());
					tsce.setDigitalEncoder(iencCount, iencState, iencInterrupt);
				}
			}
		}

		List<String> M10Gen = null;
		try {
			M10Gen = tsce.genM10(ipin0, ipin1);
			if(M10Gen.size() > 0)
				ndd.addInit(M10Gen);
			if(DEBUG) {
				System.out.printf("%s: Controller tsce:%s generating config:%s%n",this.getClass().getName(),tsce,M10Gen);
			}
		} catch(NoSuchElementException nse) {
		}
	}
	
	/**
	 * Locate the proper AsynchDemuxer by name of device we want to write to, then add the write to that AsynchDemuxer.
	 * @param name The name of the device in the config file, such as "LeftWheel", "RightWheel"
	 * @param req The request to queue
	 */
	public synchronized void addWrite(String name, String req) {
		AsynchDemuxer.addWrite(getNodeDeviceDemuxer(name).asynchDemuxer, req);
	}
	
	/**
	 * Get the class with methods that talk to the Marlinspike board by the descriptive name of the device
	 * as it appears in the config.
	 * @param name One of "Leftwheel", "RightWheel" etc.
	 * @return the MarlinspikeControlInterface with methods to communicate with the board
	 */
	public synchronized MarlinspikeControlInterface getMarlinspikeControl(String name) throws NoSuchElementException {
			NodeDeviceDemuxer ndd = (NodeDeviceDemuxer) getNodeDeviceDemuxer(name);
			return ndd.getMarlinspikeControl();
	}
	
	/**
	 * Aggregate the numerical LUNS or AXIS etc by unique property
	 * @param wrapper The TypeWrapper array in config (LUN[], AXIS[], WHEEL[], etc).
	 * @param prop The property in the LUN to return unique values of
	 * @return the array of Sting properties retrieved
	 */
	private synchronized Object[] aggregate(TypedWrapper[] wrapper, String prop) {
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
	public synchronized Collection<NodeDeviceDemuxer> getNodeDeviceDemuxerByType(Collection<TypeSlotChannelEnable> tsce) {
		return Stream.of(deviceToType).flatMap(map -> map.entrySet().stream()).filter(map->tsce.containsAll(map.getValue().values()))
				.map(e -> e.getKey() ).collect(Collectors.toCollection(ArrayList::new));
	}
	
	/**
	 * @param type One of SmartController, H-Bridge, SplitBridge, SwitchBridge, PWM etc.
	 * @return All devices of a particular type attached to the Marlinspike on this node
	 */
	public synchronized Collection<TypeSlotChannelEnable> getTypeSlotChannelEnableByType(String type) throws NoSuchElementException {
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
	public synchronized TypeSlotChannelEnable getTypeSlotChannelEnableByName(String name) throws NoSuchElementException {
		Iterator<Map<String,TypeSlotChannelEnable>> it = deviceToType.values().iterator();
		while(it.hasNext()){
			Map<String,TypeSlotChannelEnable> tsces = it.next();
			TypeSlotChannelEnable ret = tsces.get(name);
			if(ret != null)
				return ret;
		}
		throw new NoSuchElementException("The name "+name+" was not found in the collection of device to type");
	}
	
	/**
	 * SmartController, H-Bridge, SplitBridge, SwitchBridge, PWM etc.
	 * @return All the devices attached to all the Marlinspikes on this node
	 */
	public synchronized Collection<TypeSlotChannelEnable> getTypeSlotChannelEnable() {
		Collection<TypeSlotChannelEnable> tsce = new ArrayList<TypeSlotChannelEnable>();
		Stream.of(deviceToType).flatMap(map -> map.entrySet().stream())
				.map(map->map.getValue().values())
				.forEach(tsce::addAll);
		return tsce;
	}
	
	/**
	 * Get the NodeDeviceDemuxer from DeviceToType
	 * @param string The DeviceName, i.e. "LeftWheel"
	 * @return
	 */
	public synchronized NodeDeviceDemuxer getNodeDeviceDemuxer(String name) throws NoSuchElementException  {
		try {
			return devices.get(devices.indexOf(new DeviceEntry(name))).getNodeDeviceDemuxer();
		} catch(NullPointerException | NoSuchElementException | IndexOutOfBoundsException nse) {
			throw new NoSuchElementException("The device "+name+" was not found in the collection");
		}
	}
	/**
	 * Return the NodeDeviceDemuxer KeySet as ArrayList collection
	 * @return the Collection of unique NodeDeviceDemuxer by Control of LUN
	 */
	public synchronized Collection<NodeDeviceDemuxer> getNodeDeviceDemuxers() {
		return new ArrayList<NodeDeviceDemuxer>(deviceToType.keySet());
	}
	
	/**
	 * @return the devices
	 */
	public synchronized ArrayList<DeviceEntry> getDevices() {
		return devices;
	}

	public String toString() {
		return String.format("%s Robot %s host %s\r\n",this.getClass().getName(), robot, hostName);
	}
	
	public static void main(String[] args) throws IOException {
		System.out.println(Arrays.toString((typeNames.values())));
		RobotInterface robot = new Robot();
		MarlinspikeManager mm = new MarlinspikeManager(robot);
		mm.createControllers(true, true);
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
		mm.lun[i].get("Type")+","+mm.lun[i].get("Slot")+","+mm.lun[i].get("SignalPin0")+","+mm.lun[i].get("Channel")+","+
					mm.lun[i].get("EnablePin")+","+mm.lun[i].get("Direction")+","+mm.lun[i].get("EncoderPin"));
		}
	}


}
