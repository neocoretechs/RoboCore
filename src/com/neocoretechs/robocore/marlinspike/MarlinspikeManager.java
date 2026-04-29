package com.neocoretechs.robocore.marlinspike;

import java.io.IOException;
import java.io.Serializable;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.neocoretechs.robocore.config.DeviceEntry;
import com.neocoretechs.robocore.config.RobotInterface;
import com.neocoretechs.robocore.config.SlotEntry;
import com.neocoretechs.robocore.config.TypedWrapper;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.TypeSlotChannelEnable.typeNames;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;
import com.neocoretechs.robocore.serialreader.MarlinspikeDataPort;
import com.neocoretechs.robocore.serialreader.marlinspikeport.Pins;
import com.neocoretechs.robocore.serialreader.marlinspikeport.control.AbstractMotorControl;

/**
 * Each individual robot uses this class to manage its collection of attributes initially parsed from config file, including
 * LUN, WHEEL, PID. Further configuration is performed to deliver needed services.<p/>
 * Parse configs and allocate the necessary number of control elements for one or more Marlinspike boards.<p/>
 * We create the collection of devices and {@link NodeDevice} with their {@link AsynchDemuxer} to talk to the attached Marlinspikes.
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class MarlinspikeManager implements Serializable {
	private static final long serialVersionUID = 1L;
	private static boolean DEBUG = false;
	RobotInterface robot;
	String hostName;
	TypedWrapper[] lun;
	TypedWrapper[] wheel;
	TypedWrapper[] pid;
	transient AsynchDemuxer asynchDemuxer;
	/**
	 * deviceToType  [Name of device, i.e. "LeftWheel", TypeSlotChannelEnable]
	 */
	private ConcurrentHashMap<DeviceEntry, TypeSlotChannelEnable> deviceToType = new ConcurrentHashMap<DeviceEntry,TypeSlotChannelEnable>();
	/**
	 * devices - canonical list of devices by properties file device name, such as "LeftWheel". Will also carry NodeDeviceDemuxer ref
	 */
	private ArrayList<DeviceEntry> devices = new ArrayList<DeviceEntry>();
	/**
	 * startup - the array of startup M-codes to initialize Marlinspike
	 */
	private ArrayList<String> startup = new ArrayList<String>();
	/**
	 * By slot, LUN and TypeSlotChannelEnable
	 */
	private ConcurrentHashMap<SlotEntry, ArrayList<TypeSlotChannelEnable>> slotToType = new ConcurrentHashMap<SlotEntry, ArrayList<TypeSlotChannelEnable>>();
	/**
	 * slots
	 */
	private ArrayList<SlotEntry> slots = new ArrayList<SlotEntry>();
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
	}
	
	public AsynchDemuxer getDemuxer() {
		return asynchDemuxer;
	}
	
	/**
	 * Create the controllers from the lun properties. Aggregate them by port.<p/>
	 * Each unique NODENAME and CONTROLLER for each LUN should have its own demuxer.<p/>
	 * So each unique serial port attached to microcontroller, for example, needs it own message handing demuxer.<p/>
	 * @param override true to ignore node-based limitations on config params. All params loaded regardless of node name for testing, for example.
	 * @throws IOException
	 */
	public void createControllers(boolean override) throws IOException {
		for(int i = 0; i < lun.length; i++) {
			if(override || hostName.equals(lun[i].get("NodeName"))) {
				// general min and max values
				Optional<Object> ominValue = Optional.ofNullable(lun[i].get("Min"));
				Optional<Object> omaxValue = Optional.ofNullable(lun[i].get("Max"));
				String name = (String)lun[i].get("Name");
				if(name == null)
					throw new IOException("Must specify Name parameter in configuration file for host "+hostName);
				String controllerInst = (String)lun[i].get("Controller");
				DeviceEntry deviceEntry = new DeviceEntry(name, (String) lun[i].get("NodeName"), i, controllerInst);
				devices.add(deviceEntry);
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
					throw new IOException("Must specify Type paramater in configuration file for host "+hostName+" Name:"+name+" Controller:"+controllerInst);
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
					throw new IOException("Type paramater in configuration file for host "+hostName+" Name:"+name+" Controller:"+controllerInst+" Type:"+type+" must be one of "+sb.toString());
				}
				if(!type.endsWith("Pin")) { // such as InputPin, OutputPin. i.e. a more sophisticated control with multiple constructor args
					String slot = (String)lun[i].get("Slot");
					if(slot == null)
						throw new IOException("Must specify Slot paramater in configuration file for host "+hostName+" Name:"+name+" Controller:"+controllerInst+" Type:"+type);
					String channel = (String)lun[i].get("Channel");
					if(channel == null)
						throw new IOException("Must specify Channel paramater in configuration file for host "+hostName+" Name:"+name+" Controller:"+controllerInst+" Type:"+type);
					if(dir.isPresent()) {
						tsce = new TypeSlotChannelEnable(eType, 
							Integer.parseInt(slot), 
							Integer.parseInt(channel), 
							ienable, Integer.parseInt((String)dir.get()));
					} else {
						tsce = new TypeSlotChannelEnable(eType, 
								Integer.parseInt(slot), 
								Integer.parseInt(channel), 
								ienable);
					}
					SlotEntry sslot = new SlotEntry("slot"+slot,(String) lun[i].get("NodeName"), controllerInst);
					if(!slots.contains(sslot))
						slots.add(sslot);
					ArrayList<TypeSlotChannelEnable> tsceList = slotToType.get(sslot);
					if(tsceList == null) {
						tsceList = new ArrayList<TypeSlotChannelEnable>();
						slotToType.put(sslot,tsceList);
					}
					tsceList.add(tsce);
				} else { // type of Pin, so get actual pin number and construct TypeSlotChannelEnable accordingly
					String pin = (String)lun[i].get("Pin");
					if(pin == null)
						throw new IOException("Must specify Pin paramater in configuration file for host "+hostName+" Name:"+name+" Controller:"+controllerInst+" Type:"+type);
					tsce = new TypeSlotChannelEnable(eType, Integer.parseInt(pin));
					//Toggle:True,true,TRUE False,false,FALSE or absent
					Optional<Object> oToggle = Optional.ofNullable(lun[i].get("Toggle"));
					if(oToggle.isPresent() && Boolean.parseBoolean((String)lun[i].get("Toggle")))
							tsce.setPinToggle();
				}
				if(ominValue.isPresent())
					tsce.setMinValue(Integer.parseInt((String)lun[i].get("Min")));
				if(omaxValue.isPresent())
					tsce.setMaxValue(Integer.parseInt((String)lun[i].get("Max")));
				deviceToType.put(deviceEntry, tsce);
				// Configure the demuxer with the type/slot/channel and aggregate the init commands for final init
				configureMarlinspike(deviceEntry, lun[i], tsce);
			}
		}
		if(deviceToType.isEmpty())
			throw new IOException("No configuration information found for any controller for this node:"+hostName);
	}
	
	/**
	 * Active the asynchDemuxer for the given Marlinspike if it has not been previously
	 * activated. We must ensure that 1 demuxer/device is activated for a particular physical port
	 * and that subsequent attempts at activation are met with an assignment 
	 * to an existing instance of asynchDemuxer.
	 * @param deviceEntry or SlotEntry The mapping of configs
	 * @throws IOException If we attempt to re-use a port, we box up the runtime exception with the IOException
	 */
	private void activateMarlinspike(SlotEntry ndd) throws IOException {
		if(DEBUG)
			System.out.printf("%s.activateMarlinspike preparing to initialize %s%n",this.getClass().getName(), ndd);
		ndd.setMarlinspikeControl(new MarlinspikeControl(asynchDemuxer));
	}
	
	/**
	 * Add the parameter to the startup collection, if the entry already exists, reject it such that
	 * all entries are unique. There is currently no known use case where a duplicate startup directive needs issued.
	 * @param m10Gen
	 */
	private void addInit(List<String> m10Gen) {
		for(String mElem : m10Gen) {
			if(!startup.contains(mElem))
				startup.add(mElem);
		}
	}
	
	private void init() throws IOException {
		if(DEBUG)
			System.out.printf("%s.init queue:%d%n",this.getClass().getName(),startup.size());
		if(startup.size() > 0)
			asynchDemuxer.config(startup);
	}
	/**
	 * Internal configuration generator that sets up initial commands to Marlinspike controller
	 * based on configuration, to prepare to send them to the attached controller.<p/>
	 * Called as final phase of initialization pipeline.
	 * @param ndd DeviceEntry
	 * @param lun Logical unit from configuration file
	 * @param tsce The type, slot, and channel designator that defines hierarchy of Marlinspike controller devices
	 * @throws IOException If commands fail in send or confirmation
	 */
	private void configureMarlinspike(DeviceEntry ndd, TypedWrapper lun, TypeSlotChannelEnable tsce) throws IOException {
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
		int ienc=0;
		tsce.setPin0(pin0);
		tsce.setPin1(pin1);
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
	}
	/**
	 * Configure the {@link AsynchDemuxer} to handle traffic from the Marlinspike SBC or Userspace process
	 * simulating an SBC and calling GPIO activation libs etc, depending on whether the DataPort config is MarlinspikeDataPort
	 * or an actual tty port.<p>
	 * Call this from the node with attached Marlinspike, it will use the named Robot configured in the ParameterTree.<p>
	 * It will assign the AsynchDemuxer to a {@link MarlinspikeControl} for each {@link DeviceEntry}
	 * if the device control class is null, otherwise it will use the control class to instantiate an {@link AbstractMotorControl}<p>
	 * Most of what happens here is to compensate for the non-serializable members of the Robot configuration and
	 * to set up the Marlinspike.
	 * @throws IOException 
	 */
	public void configureDemuxer() throws IOException {
		asynchDemuxer = new AsynchDemuxer(this);
		try {
			if(robot.getDataPort().equals("MarlinspikeDataPort"))
				asynchDemuxer.connect(new MarlinspikeDataPort());
			else
				asynchDemuxer.connect(new ByteSerialDataPort(robot.getDataPort()));
		} catch(IOException ioe) {
			throw new RuntimeException(ioe);
		}
		devices.forEach(e->{
			MarlinspikeControlInterface controller;
			if(e.getControlClass() == null) {
				controller = new MarlinspikeControl(asynchDemuxer);
			} else {
				// extract max power value to instantiate MarlinspikeControlInterface AbstractMotorControl
				TypeSlotChannelEnable tsce = deviceToType.get(e);
				try {
					Class<?> controlClass = Class.forName(e.getControlClass());
					controller = (MarlinspikeControlInterface) controlClass.getConstructor(Integer.class).newInstance(tsce.maxValue);
				} catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException | NoSuchMethodException | ClassNotFoundException e1) {
					throw new RuntimeException(e1);
				}
			}
			try {
				e.setMarlinspikeControl(controller);
				activateMarlinspike(e);
			} catch (IOException e1) {
				throw new RuntimeException(e1);
			}
		});
		slots.forEach(e->{
			MarlinspikeControlInterface controller;
			if(e.getControlClass() == null) {
				controller = new MarlinspikeControl(asynchDemuxer);
			} else {
				// extract max power value to instantiate MarlinspikeControlInterface AbstractMotorControl
				TypeSlotChannelEnable tsce = slotToType.get(e).getFirst();
				try {
					Class<?> controlClass = Class.forName(e.getControlClass());
					controller = (MarlinspikeControlInterface) controlClass.getConstructor(Integer.class).newInstance(tsce.maxValue);
				} catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException | NoSuchMethodException | ClassNotFoundException e1) {
					throw new RuntimeException(e1);
				}
			}
			try {
				e.setMarlinspikeControl(controller);
				activateMarlinspike(e);
			} catch (IOException e1) {
				throw new RuntimeException(e1);
			}
		});
		for(TypeSlotChannelEnable tsce : deviceToType.values()) {
			List<String> M10Gen = null;
			M10Gen = tsce.genM10();
			if(M10Gen.size() > 0) {
				addInit(M10Gen);
				if(DEBUG) 
					System.out.printf("%s: Controller tsce:%s generating config:%s%n",this.getClass().getName(),tsce,M10Gen);
				if(robot.getPowerScale() > 0)
					addInit(M6Gen(tsce.getSlot()));
			}
		}
		init();
	}
	

	/**
	 * M6 - Power scale factor by slot
	 * @param slot the slot
	 * @return ArrayList of String configure M6
	 */
	private List<String> M6Gen(int slot) {
		ArrayList<String> m6 = new ArrayList<String>();
		StringBuilder sb = new StringBuilder(topicNames.M6.val());
		sb.append(" Z").append(slot).append(" S").append(robot.getPowerScale());
		m6.add(sb.toString());
		return m6;
	}

	/**
	 * Get the class with methods that talk to the Marlinspike board by the descriptive name of the device
	 * as it appears in the config. If not found in devices, try to return a slot
	 * @param name One of "Leftwheel", "RightWheel" etc.
	 * @return the MarlinspikeControlInterface with methods to communicate with the board
	 */
	public MarlinspikeControlInterface getMarlinspikeControl(String name) throws NoSuchElementException {
		int devIndex = -1;
		for(int i = 0; i < devices.size(); i++) {
			if(devices.get(i).getName().equals(name)) {
				devIndex = i;
				break;
			}
		}
		if(devIndex == -1) {
			for(int i = 0; i < slots.size(); i++) {
				if(DEBUG)
					System.out.println("Slot:"+slots.get(i));
				if(slots.get(i).getName().equals(name)) {
					devIndex = i;
					break;
				}
			}
			if(devIndex == -1)
				throw new NoSuchElementException(name+" not found in DeviceEntry or SlotEntry list");
			SlotEntry sdd = slots.get(devIndex);
			if(DEBUG)
				System.out.println("Slot "+name+" getMarlinSpikeControl:"+sdd);
			return sdd.getMarlinspikeControl();
		}
		DeviceEntry ndd = devices.get(devIndex);
		if(DEBUG)
			System.out.println("Device "+name+" getMarlinSpikeControl:"+ndd);
		return ndd.getMarlinspikeControl();
	}
	
	/**
	 * Get the set of DeviceEntry which contains 
	 * TypeSlotChannelEnable.<p/>
	 * getDeviceEntryByType(getTypeSlotChannelEnableByType(String type))
	 * @param tsce The set retrieved from one of the getTypeSlotChannelEnable methods
	 * @return The list of DeviceEntry associated with the passed list of tsce
	 */
	public Collection<DeviceEntry> getDeviceEntryByType(Collection<TypeSlotChannelEnable> tsce) {
		return Stream.of(deviceToType).flatMap(map -> map.entrySet().stream()).filter(map->tsce.contains(map.getValue()))
				.map(e -> e.getKey() ).collect(Collectors.toCollection(ArrayList::new));
	}
		
	/**
	 * @param type One of "LiftActuator" "LeftWheel" etc.
	 * @return device of a particular name, which points to the actual device of a particular type, attached to the Marlinspike on this node
	 */
	public TypeSlotChannelEnable getTypeSlotChannelEnableByName(String name) throws NoSuchElementException {
		Iterator<Entry<DeviceEntry, TypeSlotChannelEnable>> it = deviceToType.entrySet().iterator();
		while(it.hasNext()) {
			Entry<DeviceEntry, TypeSlotChannelEnable> ret = it.next();
			if(ret != null && ret.getKey().getName().equals(name))
				return ret.getValue();
		}
		// type a search through slots
		Iterator<Entry<SlotEntry, ArrayList<TypeSlotChannelEnable>>> its = slotToType.entrySet().iterator();
		while(its.hasNext()) {
			Entry<SlotEntry, ArrayList<TypeSlotChannelEnable>> ret = its.next();
			if(ret != null && ret.getKey().getName().equals(name))
				return ret.getValue().getFirst();
		}
		throw new NoSuchElementException("The name "+name+" was not found in the collection of device to type or slot to type array"
				+ "");
	}
	
	/**
	 * SmartController, H-Bridge, SplitBridge, SwitchBridge, PWM etc.
	 * @return All the devices attached to all the Marlinspikes on this node
	 */
	public Collection<TypeSlotChannelEnable> getTypeSlotChannelEnable() {
		return deviceToType.values();
	}
	
	/**
	 * Get the TypeSlotChannelEnable by DeviceEntry if it exists
	 * @param deviceEntry
	 * @return
	 */
	public TypeSlotChannelEnable getTypeSlotChannelEnable(DeviceEntry deviceEntry) {
		return deviceToType.get(deviceEntry);
	}
	
	/**
	 * @return the devices
	 */
	public ArrayList<DeviceEntry> getDevices() {
		return devices;
	}
	
	/**
	 * Get the table of slots to types
	 * @return the table of slots to types
	 */
	public ConcurrentHashMap<SlotEntry,ArrayList<TypeSlotChannelEnable>> getSlots() {
		return slotToType;
	}
	/**
	 * Return the TypeSlotchannelEnable LUN's by slot
	 * @param slot the slotname (number)
	 * @return the ArrayList of TypeSlotChannelEnables for the slot according to properties
	 */
	public ArrayList<TypeSlotChannelEnable> getSlot(String slot) {
		for(Map.Entry<SlotEntry,ArrayList<TypeSlotChannelEnable>> kv : slotToType.entrySet()) {
			if(kv.getKey().getName().equals(slot))
				return kv.getValue();
		}
		return null;
	}

	public String toString() {
		return String.format("%s Robot %s host %s\r\n",this.getClass().getName(), robot, hostName);
	}
	
	public static void main(String[] args) throws IOException {
		/*
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
		*/
	}

}
