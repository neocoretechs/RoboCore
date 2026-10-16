package com.neocoretechs.robocore.marlinspike;

import java.io.IOException;
import java.io.Serializable;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.neocoretechs.robocore.config.Robot;
import com.neocoretechs.robocore.config.RobotInterface;
import com.neocoretechs.robocore.config.SlotEntry;
import com.neocoretechs.robocore.config.DeviceEntry;
import com.neocoretechs.robocore.config.TypedWrapper;
import com.neocoretechs.robocore.config.StickAxis;
import com.neocoretechs.robocore.config.TriggerAxis;
import com.neocoretechs.robocore.config.POVAxis;
import com.neocoretechs.robocore.config.Button;

import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.TypeSlotChannelEnable.typeNames;
import com.neocoretechs.robocore.propulsion.DrivenWheelInterface;
import com.neocoretechs.robocore.propulsion.RobotDiffDrive;
import com.neocoretechs.robocore.propulsion.RobotDiffDriveInterface;
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
	private static boolean DEBUG = true;
	RobotInterface robot;
	String hostName;
	TypedWrapper[] lun;
	TypedWrapper[] wheel;
	TypedWrapper[] pid;
	TypedWrapper[] axis;
	TypedWrapper[] button;
	transient AsynchDemuxer asynchDemuxer;

	/**
	 * startup - the array of startup M-codes to initialize Marlinspike
	 */
	private ArrayList<String> startup = new ArrayList<String>();
	/**
	 * deviceToType  [Name of device, i.e. "BoomActuator", TypeSlotChannelEnable], DEviceEntry, SlotEntry subclass, will have 1 TypeSlotChannelEnable for pin
	 * otherwise 1 or more for slots with multiple channels
	 * By slot, LUN and TypeSlotChannelEnable
	 */
	private ConcurrentHashMap<DeviceEntry, ArrayList<TypeSlotChannelEnable>> deviceToType = new ConcurrentHashMap<DeviceEntry, ArrayList<TypeSlotChannelEnable>>();
	
	/**
	 * Each individual robot uses this class to manage its collection of attributes initially parsed from config file, including
	 * LUN, WHEEL, PID. Further configuration is performed to deliver needed services.
	 * @param robot The RobotInterface from which parsed config directives are acquired.
	 * @throws IOException 
	 */
	public MarlinspikeManager(RobotInterface robot) throws IOException {
		this.robot = robot;
		this.lun = robot.getLUN();
		this.wheel = robot.getWHEEL();
		this.pid = robot.getPID();
		this.axis = robot.getAXIS();
		this.button = robot.getBUTTON();
		this.hostName = robot.getHostName();
		configureDemuxer();
	}
	
	public AsynchDemuxer getDemuxer() {
		return asynchDemuxer;
	}
	
	/**
	 * Create the controllers from the lun properties. Aggregate them by port.<p>
	 * Each unique NODENAME and CONTROLLER for each LUN should have its own demuxer.<p>
	 * So each unique serial port attached to microcontroller, for example, needs it own message handing demuxer.<p>
	 * @param override true to ignore node-based limitations on config params. All params loaded regardless of node name for testing, for example.
	 * @throws IOException
	 */
	public void createControllers(boolean override) throws IOException {
		if(DEBUG)
			System.out.printf("%s.createControllers(%b) LUN length=%d%n",this.getClass().getName(), override, lun.length);
		for(int i = 0; i < lun.length; i++) {
			Optional<Object> nodeName = Optional.ofNullable(lun[i].get("NodeName"));
			if(nodeName.isEmpty())
				throw new IOException("Must specify NodeName parameter in configuration file for host "+hostName);
			Optional<Object> name = Optional.ofNullable(lun[i].get("Name"));
			if(name.isEmpty())
				throw new IOException("Must specify Name parameter in configuration file for host "+hostName);
			if(DEBUG)
				System.out.printf("%s.createControllers(%b) len=%s LUN[%d] HostName=%s NodeName=%s name=%s%n",this.getClass().getName(), override, lun.length, i, hostName, nodeName.get(), name.get());
			if(override || hostName.equals(nodeName.get())) {
				// general min and max values
				Optional<Integer> ominValue = Optional.ofNullable(lun[i].get("MinValue")).filter(String.class::isInstance).map(e -> Integer.parseInt((String) e));
				Optional<Integer> omaxValue = Optional.ofNullable(lun[i].get("MaxValue")).filter(String.class::isInstance).map(e -> Integer.parseInt((String) e));
				Optional<Object> controllerInst = Optional.ofNullable(lun[i].get("Controller"));
				MarlinspikeControlInterface controller = null;
				DeviceEntry deviceEntry = null;
				Optional<Integer> dir = Optional.ofNullable(lun[i].get("Direction")).filter(String.class::isInstance).map(e -> Integer.parseInt((String) e));
				Optional<Integer> enable = Optional.ofNullable(lun[i].get("EnablePin")).filter(String.class::isInstance).map(e -> Integer.parseInt((String) e));
				int ienable = enable.isPresent() ? enable.get() : 0;
				TypeSlotChannelEnable tsce = null;
				Optional<Object> type = Optional.ofNullable(lun[i].get("Type"));
				if(type.isEmpty())
					throw new IOException("Must specify Type parameter in configuration file for host "+hostName+" Name:"+name.get()+" Controller:"+controllerInst);
				typeNames eType = null;
				//eType = typeNames.valueOf(type) will fail because we use mixed case names;
				for(typeNames et : typeNames.values()) {
					if(et.name.equals((String)type.get())) {
						eType = et;
						break;
					}
				}
				if(eType == null) {
					//Arrays.toString(typeNames.values()) gives enum types, not values of names
					StringBuilder sb = new StringBuilder();
					for(typeNames et : typeNames.values())
						sb.append(et.val()+" ");
					throw new IOException("Type parameter in configuration file for host "+hostName+" Name:"+name.get()+" Controller:"+controllerInst+" Type:"+type.get()+" must be one of "+sb.toString());
				}
				// If the type is not ending in "Pin", its expected to have an associated slot. A driver has a slot that allows it to be referenced
				// in configs. A slot can have multiple channels and thus entries in the properties
				// Not a type of 'Pin', such as InputPin, OutputPin. i.e. a more sophisticated control with multiple constructor args, and
				if(!((String)type.get()).endsWith("Pin")) {
					Optional<Integer> slot = Optional.ofNullable(lun[i].get("Slot")).filter(String.class::isInstance).map(e -> Integer.parseInt((String) e));
					if(slot.isEmpty())
						throw new IOException("Must specify Slot parameter in configuration file for host "+hostName+" Name:"+name.get()+" Controller:"+controllerInst+" Type:"+type.get());
					Optional<Integer> ochannel = Optional.ofNullable(lun[i].get("Channel")).filter(String.class::isInstance).map(e -> Integer.parseInt((String) e));
					if(ochannel.isEmpty())
						throw new IOException("Must specify Channel parameter in configuration file for host "+hostName+" Name:"+name.get()+" Controller:"+controllerInst+" Type:"+type.get());
					if(((String)name.get()).endsWith("Wheel")) {
						int maxValue = 1000;
						if(omaxValue.isPresent())
							maxValue = omaxValue.get();
						controller = createController(controllerInst, maxValue);
						// create a new template
						if(robot.getDiffDrive() == null) {
							deviceEntry = new RobotDiffDrive(robot, (String)name.get(), (String)nodeName.get(), i, controller, slot.get(), lun, wheel, pid, axis);
							robot.setDiffDrive((RobotDiffDriveInterface) deviceEntry);
						} else
							deviceEntry = (DeviceEntry) robot.getDiffDrive();
						// see if we have it already
						ArrayList<TypeSlotChannelEnable> sl = deviceToType.get(deviceEntry);
						if(sl == null) {
							sl = new ArrayList<TypeSlotChannelEnable>();
							deviceToType.put(deviceEntry, sl);
						} else {
							// add to this control if we dont have the slot already
							if(((SlotEntry)deviceEntry).getMarlinspikeControl(slot.get()) == null)
									((SlotEntry)deviceEntry).setMarlinspikeControl(controller, (String)name.get(), slot.get());
						}
						int ti = sl.indexOf(tsce);
						if(ti == -1) {
							tsce = RobotDiffDrive.setDrive(lun, wheel, pid, (String)name.get(), i, slot.get(), ochannel.get());
							sl.add(tsce);
							configureMarlinspike(deviceEntry, lun[i], tsce);
						} else {
							throw new IOException("Already parsed "+tsce+" found at index "+ti+" in deviceToType list.");		
						}
					} else {
						// Non-wheel slotted controller deviceEntry that contains SlotHandler(s)
						if(dir.isPresent()) {
							tsce = new TypeSlotChannelEnable(eType, slot.get(), ochannel.get(), ienable, dir.get());
						} else {
							tsce = new TypeSlotChannelEnable(eType, slot.get(), ochannel.get(), ienable);
						}
						if(ominValue.isPresent())
							tsce.setMinValue(ominValue.get());
						if(omaxValue.isPresent())
							tsce.setMaxValue(omaxValue.get());
						controller = createController(controllerInst, tsce.getMaxValue());
						// creates SlotHandler
						deviceEntry = new SlotEntry((String)name.get(), (String)nodeName.get(), i, controller, slot.get());
						ArrayList<TypeSlotChannelEnable> sl = deviceToType.get(deviceEntry);
						if(sl == null) {
							sl = new ArrayList<TypeSlotChannelEnable>();
							deviceToType.put(deviceEntry, sl);
						} else {
							// add to this control possibly
							if(((SlotEntry)deviceEntry).getMarlinspikeControl(slot.get()) == null)
									((SlotEntry)deviceEntry).setMarlinspikeControl(controller, (String)name.get(), slot.get());
						}
						int ti = sl.indexOf(tsce);
						if(ti == -1) {
							sl.add(tsce);
							configureMarlinspike(deviceEntry, lun[i], tsce);
						} else {
							throw new IOException("Already parsed "+tsce+" found at index "+ti+" in deviceToType list.");		
						}
					}
				} else { // type of Pin, so get actual pin number and construct TypeSlotChannelEnable accordingly
					Optional<Integer> pin = Optional.ofNullable(lun[i].get("Pin")).filter(String.class::isInstance).map(e -> Integer.parseInt((String) e));
					if(pin.isEmpty())
						throw new IOException("Must specify Pin parameter in configuration file for host "+hostName+" Name:"+name.get()+" Controller:"+controllerInst+" Type:"+type.get());
					tsce = new TypeSlotChannelEnable(eType, pin.get());
					if(ominValue.isPresent())
						tsce.setMinValue(ominValue.get());
					if(omaxValue.isPresent())
						tsce.setMaxValue(omaxValue.get());
					//Toggle:True,true,TRUE False,false,FALSE or absent
					Optional<Object> oToggle = Optional.ofNullable(lun[i].get("Toggle"));
					if(oToggle.isPresent() && Boolean.parseBoolean((String)oToggle.get()))
						tsce.setPinToggle();
					controller = createController(controllerInst, tsce.getMaxValue());
					deviceEntry = new DeviceEntry((String)name.get(), (String)nodeName.get(), i, controller);
					ArrayList<TypeSlotChannelEnable> sl = deviceToType.get(deviceEntry);
					if(sl == null)
						sl = new ArrayList<TypeSlotChannelEnable>();
					else 
						throw new IOException("Already parsed deviceEntry"+deviceEntry+" for "+tsce+" found at in deviceToType list.");		
					sl.add(tsce);
					deviceToType.put(deviceEntry, sl);
					deviceEntry.setMarlinspikeControl(controller);
					configureMarlinspike(deviceEntry, lun[i], tsce);
				}
				// configure the manual controls if any
				configureManualControl(deviceEntry, axis[i], button[i]);
				// Configure the demuxer with the type/slot/channel and aggregate the init commands for final init
				// tsce gen M10
				List<String> M10Gen = tsce.genM10();
				if(M10Gen.size() > 0) {
					addInit(M10Gen);
					if(DEBUG) 
						System.out.printf("%s: Controller tsce:%s generating config:%s%n",this.getClass().getName(),tsce,M10Gen);
					if(robot.getPowerScale() > 0)
						addInit(M6Gen(tsce.getSlot()));
				}
				init();
			}
		}
		if(deviceToType.isEmpty())
			throw new IOException("No configuration information found for any controller for this node:"+hostName);
	}
	
	/**
	 * Create a {@link MarlinspikeControl} based on whether controllerInst is is null or names a class that takes the maxValue in ctor.
	 * If it didnt exist in configs (Optional.isEmpty) create a default MarlinspikeControl
	 * @param controllerInst the class name from config or null
	 * @param maxValue the maxValue to pass to newIstance ctor
	 * @return The MarlinspikeControl instance
	 * @throws IOException if InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException | NoSuchMethodException | ClassNotFoundException
	 */
	public MarlinspikeControlInterface createController(Optional<Object> controllerInst, int maxValue) throws IOException {
		if(DEBUG)
			System.out.printf("%s:createController %s maxValue:%d%n",this.getClass().getName(),controllerInst,maxValue);
		MarlinspikeControlInterface controller;
		if(controllerInst.isEmpty()) {
			controller = new MarlinspikeControl(asynchDemuxer);
		} else {
			// extract max power value to instantiate MarlinspikeControlInterface AbstractMotorControl
			try {
				Class<?> controlClass = Class.forName((String) controllerInst.get());
				controller = (MarlinspikeControlInterface) controlClass.getConstructor(Integer.class).newInstance(maxValue);
			} catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException | NoSuchMethodException | ClassNotFoundException e1) {
				throw new IOException(e1);
			}
		}
		return controller;
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
	 * based on configuration, to prepare to send them to the attached controller.<p>
	 * Called as final phase of initialization pipeline.
	 * @param ndd DeviceEntry
	 * @param lun Logical unit from configuration file
	 * @param tsce The type, slot, and channel designator that defines hierarchy of Marlinspike controller devices
	 * @throws IOException If commands fail in send or confirmation
	 */
	private void configureMarlinspike(DeviceEntry ndd, TypedWrapper lun, TypeSlotChannelEnable tsce) throws IOException {
		if(DEBUG)
			System.out.printf("%s.configureMarlinspike for DeviceEntry %s TypedWrapper %s TypeSlotChannelEnable %s%n",this.getClass().getName(), ndd, lun, tsce);
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
	 * Configure the manual control for the DeviceEntry, if any. call setManulaControl on DeviceEntry with new AxisInterface generic.
	 * @param de DeviceEntry target
	 * @param axis element at AXIS array index of DeviceEntry
	 * @param buttonelelment at BUTTON array index of DeviceEntry
	 */
	public void configureManualControl(DeviceEntry de, TypedWrapper axis, TypedWrapper button) {
		if(axis == null) {
			throw new RuntimeException(String.format("%s.publishPeripheral can not get the axis  %s based on configs.%n",this.getClass().getName(),de));
		}
		String axisType = (String) axis.get("AxisType");
		if(axisType == null) {
			throw new RuntimeException(String.format("%s.publishPeripheral NO axis type attribute device %s%n", this.getClass().getName(), de));
		}
		if(DEBUG) {
			System.out.printf("%s.publishPeripheral axis type %s attribute device %s%n", this.getClass().getName(), axisType, de);
		}
		switch(axisType) {
		case "Stick":
			Optional<Integer> x = Optional.ofNullable(axis.get("AxisX")).filter(String.class::isInstance).map(e -> Integer.parseInt((String) e));
			if(x.isEmpty())
				throw new RuntimeException(String.format("%s AxisX missing from config for %s%n",this.getClass().getName(), de));
			Optional<Integer> y = Optional.ofNullable(axis.get("AxisY")).filter(String.class::isInstance).map(e -> Integer.parseInt((String) e));
			if(y.isEmpty())
				throw new RuntimeException(String.format("%s AxisY missing from config for %s%n",this.getClass().getName(), de));
			de.setManualcontrol(new StickAxis(new int[]{x.get(),y.get()}));
			break;
		case "Trigger":
			Optional<Integer> ax = Optional.ofNullable(axis.get("Axis")).filter(String.class::isInstance).map(e -> Integer.parseInt((String) e));
			if(ax.isEmpty())
				throw new RuntimeException(String.format("%s Axis missing from config for %s%n",this.getClass().getName(), de));
			de.setManualcontrol(new TriggerAxis(new int[]{ax.get()}));
			break;
		case "Button":
			ax = Optional.ofNullable(axis.get("Axis")).filter(String.class::isInstance).map(e -> Integer.parseInt((String) e));
			if(ax.isEmpty())
				throw new RuntimeException(String.format("%s Axis missing from config for %s%n",this.getClass().getName(), de));
			de.setManualcontrol(new Button(new int[]{ax.get()}));
			break;
		case "POV":
			de.setManualcontrol(new POVAxis());
			break;
		default:
			throw new RuntimeException("Unknown Axis type in configuration:"+axisType+" for LUN "+de);
		}
	}
	/**
	 * Configure the {@link AsynchDemuxer} to handle traffic from the Marlinspike SBC or Userspace process
	 * simulating an SBC and calling GPIO activation libs etc, depending on whether the DataPort config is MarlinspikeDataPort
	 * or an actual tty port.<p>
	 * Calls init() before returning.<p>
	 * Calls connect() in asynchDemuxer, then clears Marlinspike settings on the microcontroller<p>
	 * Call this from the node with attached Marlinspike, it will use the named Robot configured in the ParameterTree.<p>
	 * It will connect the AsynchDemuxer.
	 *  to a {@link MarlinspikeControl} for each {@link DeviceEntry}
	 * if the device control class is null, otherwise it will use the control class to instantiate an {@link AbstractMotorControl}<p>
	 * Most of what happens here is to compensate for the non-serializable members of the Robot configuration and
	 * to set up the Marlinspike.
	 * @throws IOException 
	 */
	private void configureDemuxer() throws IOException {
		this.asynchDemuxer = new AsynchDemuxer(this);
		try {
			if(robot.getDataPort().equals("MarlinspikeDataPort"))
				this.asynchDemuxer.connect(new MarlinspikeDataPort());
			else
				this.asynchDemuxer.connect(new ByteSerialDataPort(robot.getDataPort()));
		} catch(IOException ioe) {
			throw new RuntimeException(ioe);
		}
		// send the codes to reset Marlinspike pins and controllers
		MarlinspikeControl.clearMarlinspikeSettings(this.asynchDemuxer);
		if(DEBUG)
			System.out.printf("%s.configureDemuxer  %s%n",this.getClass().getName(), this.asynchDemuxer);
	}
	
	/**
	 * Set the speed values for the differential drive, regardless of number of wheels.<p>
	 * Assume the primary drive at slot 0.
	 * @param speedVals The array of speed values ordered left/right/left/right for the number of wheel pairs
	 * @throws IOException 
	 */
	public void setDiffDriveSpeed(int[] speedVals) throws IOException {
		// now we have all the channels for the slot 0 controller
		if(DEBUG)
			System.out.printf("%s.setDiffDriveSpeed  diff drive=%s%n",this.getClass().getName(), robot.getDiffDrive());
		SlotHandler slotHandler = robot.getDiffDrive().getMarlinspikeControl(0);
		if(DEBUG)
			System.out.printf("%s.setDiffDriveSpeed slotToHandler slotHandler=%s%n",this.getClass().getName(), slotHandler);
		slotHandler.setSpeed((Robot) robot, speedVals);
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
	 * @param name One of "LEDDriver", "DiffDrive" etc.
	 * @return the MarlinspikeControlInterface with methods to communicate with the board
	 */
	public MarlinspikeControlInterface getMarlinspikeControl(String name) throws NoSuchElementException {
		Iterator<DeviceEntry> it = deviceToType.keys().asIterator();
		while(it.hasNext()) {
			DeviceEntry dd = it.next();
			if(dd.getName().equals(name)) {
				if(DEBUG)
					System.out.println("Device "+name+" getMarlinSpikeControl:"+dd);
				return dd.getMarlinspikeControl();
			}
		}
		throw new NoSuchElementException(name+" not found in DeviceEntry list");
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
	 * @param type One of "LiftActuator", "LEDPin", etc.<p> 
	 * "Wheel" ending entries are aggregated under "DiffDrive".<p>
	 * @return device of a particular name, which points to the actual device of a particular type, attached to the Marlinspike on this node
	 */
	public ArrayList<TypeSlotChannelEnable> getTypeSlotChannelEnableByName(String name) throws NoSuchElementException {
		Iterator<Entry<DeviceEntry, ArrayList<TypeSlotChannelEnable>>> it = deviceToType.entrySet().iterator();
		while(it.hasNext()) {
			Entry<DeviceEntry, ArrayList<TypeSlotChannelEnable>> ret = it.next();
			if(ret != null && ret.getKey().getName().equals(name))
				return ret.getValue();
		}
		// type a search through slots
		Iterator<Entry<DeviceEntry, ArrayList<TypeSlotChannelEnable>>> its = deviceToType.entrySet().iterator();
		while(its.hasNext()) {
			Entry<DeviceEntry, ArrayList<TypeSlotChannelEnable>> ret = its.next();
			if(ret != null && ret.getKey().getName().equals(name))
				return ret.getValue();
		}
		throw new NoSuchElementException("The name "+name+" was not found in the collection of device to type or slot to type array");
	}
	
	/**
	 * SmartController, H-Bridge, SplitBridge, SwitchBridge, PWM etc.
	 * @return All the devices attached to all the Marlinspikes on this node
	 */
	public Collection<ArrayList<TypeSlotChannelEnable>> getTypeSlotChannelEnable() {
		return deviceToType.values();
	}
	
	/**
	 * Get the TypeSlotChannelEnable by DeviceEntry if it exists
	 * @param deviceEntry
	 * @return
	 */
	public ArrayList<TypeSlotChannelEnable> getTypeSlotChannelEnable(DeviceEntry deviceEntry) {
		return deviceToType.get(deviceEntry);
	}
	
	
	/**
	 * Get the table of slots to types
	 * @return the table of slots to types
	 */
	public ConcurrentHashMap<DeviceEntry,ArrayList<TypeSlotChannelEnable>> getDeviceEntries() {
		return deviceToType;
	}
	/**
	 * Return the TypeSlotchannelEnable LUN's by slot. Look for subclass of SlotEntry in deviceToType.
	 * Then acquire the SlotHandler for the given slot using SlotEntry.getMarlinspikeControl.
	 * if it comes back non-null, meaning the slot was found, get the value for the deviceToType key and return it.
	 * Otherwise, iterate the deviceToType entries until we exhaust them and return null, meaning the given
	 * slot is not in use.
	 * @param slot the slot number
	 * @return the ArrayList of TypeSlotChannelEnables for the slot according to properties, or null if no slot
	 */
	public ArrayList<TypeSlotChannelEnable> getSlot(int slot) {
		for(Map.Entry<DeviceEntry,ArrayList<TypeSlotChannelEnable>> kv : deviceToType.entrySet()) {
			if(kv.getKey() instanceof SlotEntry) {
				SlotHandler sh = ((SlotEntry)(kv.getKey())).getMarlinspikeControl(slot);
				// if it contains the slot we are looking for, we can then retrieve the associated array of TypeSlotChannelEnable
				if(sh != null) {
					return kv.getValue();
				}
			}
		}
		return null;
	}
	
	public ConcurrentHashMap<DeviceEntry, ArrayList<TypeSlotChannelEnable>> getDevices() {
		return deviceToType;
	}
	
	public String toString() {
		return String.format("%s Robot %s host %s\r\n",this.getClass().getName(), robot, hostName);
	}
	
	public static void main(String[] args) throws IOException {	
		System.out.println(Arrays.toString((typeNames.values())));
		RobotInterface robot = new Robot();
		MarlinspikeManager mm = new MarlinspikeManager(robot);
		mm.createControllers(true);
		System.out.println("Control for DiffDrive:"+mm.getMarlinspikeControl("DiffDrive"));
		System.out.println("Control for BoomActuator:"+mm.getMarlinspikeControl("BoomActuator"));
		System.out.println("Control for LEDDriver:"+mm.getMarlinspikeControl("LEDDriver"));
		try {
			System.out.println("Control for BogusWheel:"+mm.getMarlinspikeControl("BogusWheel"));
		} catch(NoSuchElementException nse) { System.out.println("passed");}
		Stream.of(mm.deviceToType).flatMap(map -> map.entrySet().stream()).forEach(e -> System.out.println(e));
		Collection<ArrayList<TypeSlotChannelEnable>> tsce = mm.getTypeSlotChannelEnable();
		System.out.println("---1--");
		for(ArrayList<TypeSlotChannelEnable> tta : tsce) {
			System.out.println("+++++");
			for(TypeSlotChannelEnable tt: tta)
				System.out.println(tt);
		}
		System.out.println("---1--");
		System.out.println("---2--");
		for(ArrayList<TypeSlotChannelEnable> tta : tsce) {
			Collection<DeviceEntry> listNodeDevice = mm.getDeviceEntryByType(tta);
			System.out.println("+++++");
			for(DeviceEntry ndd : listNodeDevice)
				System.out.println(ndd);
		}
		System.out.println("---2--");
		System.out.println("MarlinspikeControl for DiffDrive:"+mm.getMarlinspikeControl("DiffDrive"));
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
