package com.neocoretechs.robocore.config;

import java.io.IOException;
import java.io.Serializable;

import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import com.neocoretechs.robocore.PID.IMUSetpointInfo;
import com.neocoretechs.robocore.PID.MotionPIDController;
import com.neocoretechs.robocore.PID.PIDParameterInterface;
import com.neocoretechs.robocore.PID.SetpointInfoInterface;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.MarlinspikeManager;
import com.neocoretechs.robocore.propulsion.RobotDiffDrive;
import com.neocoretechs.robocore.propulsion.RobotDiffDriveInterface;
/**
 * This class represents a centralized configuration management instrument.
 * <pre>
 * Read the paramaterized properties from the properties class. Config as follows by LUN, logical unit number:<p>
 * DataPort:/dev/ttyACM0
 * PowerScale:0
 * LUN[0].Name:LeftWheel 
 * LUN[0].NodeName:Control1
 * LUN[0].Type:H-Bridge 
 * LUN[0].Slot:0 
 * LUN[0].SignalPin0:8 
 * LUN[0].Channel:1 
 * LUN[0].EnablePin:24 
 * LUN[0].Direction:0 
 * LUN[0].EncoderPin:68 
 * LUN[0].MinValue:-1000 
 * LUN[0].MaxValue:1000 
 * PID[0].MotorPIDRate:1 
 * PID[0].MotorKp:1.0 
 * PID[0].MotorKd:1.0 
 * PID[0].MotorKi:1.0 
 * PID[0].MotorKo:1.0 
 * AXIS[0].AxisType:Stick
 * AXIS[0].StickType:Left/Right/Any - TBI
 * AXIS[0].AxisX:0 
 * AXIS[0].AxisY:2
 * </pre>
 * Also, BUTTON, WHEEL, etc as needed.<p>
 * The handler method for the slot is defined by annotation in the {@link com.neocoretechs.robocore.propulsion.MotionController}.
 * Motor controls are aggregated by slot. In the Marlinspike configurations, the motor controller are assigned a slot
 * from 0-9 and the controls such as G5 are first identified by slot. So G5 Z0 P300 Q300 would set the power
 * levels of the motor controller at slot 0 to P, channel 1, level 300, Q, channel 2, level 300. If it had a third wheel
 * there would be an entry in the configs for that at channel 3 slot 0 and the G5 could be G5 Z0 P300 Q300 R300, etc up to
 * 10 wheels at Y.<p>
 * Its possible that in a 4 wheel vehicle, wheel diameter can vary slightly as in, say, the drive wheels of a tracked vehicle
 * and if the tracks are driven by 4 motors with 2 sets of cogs differing in size front to back the rotation rates will
 * have to by synchronized despite the differing 'wheel' sizes. The track vary slightly, etc.<p>
 * Parameters that would seem to warrant global status have been repeated so as to appear from the
 * perspective of each individual component, and if necessary, appear in aggregate as global.<p>
 * If we initialize a diff drive, we expect PID control settings, and IMU settings even if one never appears.
 * Global power scale can divide power level by divisor to make device more indoor safe.
 * @see com.neocoretechs.robocore.propulsion.MotionController
 * @author Jonathan Groff (C) NeoCoreTechs 2021,2026
 *
 */
public class Robot implements RobotInterface, Serializable {
	public static boolean DEBUG = false;
	private static final long serialVersionUID = 1L;
	private int powerScale;
	private String robotName = "UNDEFINED";
	private int temperatureThreshold;
	private MotionPIDController motionPIDController;
	private RobotDiffDriveInterface robotDrive;
	private IMUSetpointInfo IMUSetpoint;
	private String dataPort = "/dev/ttyACM0";
	// These arrays are by 'channel'
	private TypedWrapper[] LUN;
	private TypedWrapper[] WHEEL;
	private TypedWrapper[] PID;
	private TypedWrapper[] AXIS;
	private TypedWrapper[] BUTTON;
	private Map<String, Map<Integer, Map<String, Object>>> globalConfigs;
	AsynchDemuxer asynchDemuxer;
	MarlinspikeManager marlinspikeManager = null;
	HashMap<String, Boolean> isOperating = new HashMap<String, Boolean>();
	boolean[] isActive;
	
	public Robot(String robotName) throws IOException {
		this.robotName = robotName;
		try {
			globalConfigs = Props.collectivizeProps();
		} catch (IllegalAccessException e) {
			throw new RuntimeException(e);
		}
		if(globalConfigs == null)
			throw new RuntimeException("Global property configuration failed in starting Robot, possibly data directory:"+Props.dataDirectory+" or property file:"+Props.getPropFile()+" is invalid");
		extractLUN();
		extractWHEEL();
		extractPID();
		extractAXIS();
		extractBUTTON();
		powerScale = Props.toInt("PowerScale"); // div power by divisor
		dataPort = Props.toString("DataPort");
		if(WHEEL.length > 0) {
			robotDrive = new RobotDiffDrive(LUN, WHEEL, AXIS, PID);
			//kp, ki, kd, ko, pidRate (hz)
			motionPIDController = new MotionPIDController(Props.toFloat("CrosstrackKp"), 
					Props.toFloat("CrosstrackKd"), 
					Props.toFloat("CrosstrackKi"), 
					Props.toFloat("CrosstrackKo"), 
					Props.toInt("CrosstrackPIDRate"));
		}
		IMUSetpoint = new IMUSetpointInfo();
		IMUSetpoint.setMaximum(Props.toFloat("MaxIMUDeviationDegrees")); // max deviation allowed from course
		IMUSetpoint.setMinimum(Props.toFloat("MinIMUDeviationDegrees")); // min deviation
		temperatureThreshold = Props.toInt("TemperatureThreshold");//40 C 104 F
	}

	public Robot() {}

	public void configureMarlinspike() throws IOException {
		marlinspikeManager = new MarlinspikeManager(this);
		marlinspikeManager.createControllers(true);
		isActive = new boolean[marlinspikeManager.getDevices().size()];
		// the collection of NodeDeviceDemuxer will be accumulated based on the node name entries in the properties file,
		// if it matched the name of this host the entry is included in the collection. 
		// In this way only entries that apply to Marlinspikes attached to this host are utilized.
		for(DeviceEntry ndd: marlinspikeManager.getDevices()) {
			if(DEBUG)
				System.out.printf("%s Setting DeviceName %s to non-operational status%n", this.getClass().getName(), ndd.getName());	
			isOperating.put(ndd.getName(), false);
		}
	}
	
	public String getDataPort() {
		return dataPort;
	}
	
	public MarlinspikeManager getManager() {
		return marlinspikeManager;
	}
	
	public HashMap<String, Boolean> getOperating() {
		return isOperating;
	}
	
	public boolean[] active() {
		return isActive;
	}
	
	private void extractBUTTON() {
		//Map<String, Map<Integer, Map<String, Object>>> globalConfigs;
		 Map<Integer, Map<String, Object>> button = globalConfigs.get("BUTTON");
		 if(button == null) {
			 BUTTON = new TypedWrapper[0];
			 return;
		 }
		 if(DEBUG)
			 System.out.println("BUTTON size:"+button.size());
		 BUTTON = new TypedWrapper[button.size()];
		 Set<Integer> buttonChannels = button.keySet();
		 if(DEBUG)
			 System.out.println("BUTTONs channels:"+buttonChannels.size());
		 Object[] buttonOChannels = buttonChannels.toArray();
		 Arrays.sort(buttonOChannels); // make sure of order
		 if(DEBUG)
			 System.out.println("ButtonOChannels sorted size:"+buttonOChannels.length);
		 for(Object buttonAChannels: buttonOChannels) {
			 Map<String, Object> sortedButtonProp = button.get(buttonAChannels); //buttonAChannels sorted integer channels of LUNs/AXIS/BUTTON
			 if(DEBUG)
				 System.out.println(buttonAChannels+" index of number of SortedButtonProp:"+sortedButtonProp.size());
			 BUTTON[(int)buttonAChannels] = new TypedWrapper(sortedButtonProp);
		 }	
	}
	
	private void extractAXIS() {
		//Map<String, Map<Integer, Map<String, Object>>> globalConfigs;
		 Map<Integer, Map<String, Object>> axis = globalConfigs.get("AXIS");
		 if(axis == null) {
			 AXIS = new TypedWrapper[0];
			 return;
		 }
		 if(DEBUG)
			 System.out.println("AXIS size:"+axis.size());
		 AXIS = new TypedWrapper[axis.size()];
		 Set<Integer> axisChannels = axis.keySet();
		 if(DEBUG)
			 System.out.println("AXISs channels:"+axisChannels.size());
		 Object[] axisOChannels = axisChannels.toArray();
		 Arrays.sort(axisOChannels); // make sure of order
		 if(DEBUG)
			 System.out.println("AxisOChannels sorted size:"+axisOChannels.length);
		 for(Object axisAChannels: axisOChannels) {
			 Map<String, Object> sortedAxisProp = axis.get(axisAChannels); //axisAChannels sorted integer channels of LUNs
			 if(DEBUG)
				 System.out.println(axisAChannels+" index of number of SortedAxisProp:"+sortedAxisProp.size());
			 AXIS[(int)axisAChannels] = new TypedWrapper(sortedAxisProp);
		 }	
	}

	private void extractPID() {
		//Map<String, Map<Integer, Map<String, Object>>> globalConfigs;
		 Map<Integer, Map<String, Object>> pids = globalConfigs.get("PID");
		 if(pids == null) {
			 PID = new TypedWrapper[0];
			 return;
		 }
		 PID = new TypedWrapper[pids.size()];
		 Set<Integer> pidsChannels = pids.keySet();
		 Object[] pidsOChannels = pidsChannels.toArray();
		 Arrays.sort(pidsOChannels); // make sure of order
		 for(Object pidsAChannels: pidsOChannels) {
			 Map<String, Object> sortedPidProp = pids.get(pidsAChannels); //lunsAChannels sorted integer channels of LUNs
			 PID[(int)pidsAChannels] = new TypedWrapper(sortedPidProp);
		 }	
	}
	
	private void extractWHEEL() {
		//Map<String, Map<Integer, Map<String, Object>>> globalConfigs;
		 Map<Integer, Map<String, Object>> wheels = globalConfigs.get("WHEEL");
		 if(wheels == null) {
			 WHEEL = new TypedWrapper[0];
			 return;
		 }
		 if(DEBUG)
			 System.out.println("Wheel size:"+wheels.size());
		 WHEEL = new TypedWrapper[wheels.size()];
		 Set<Integer> wheelsChannels = wheels.keySet();
		 if(DEBUG)
			 System.out.println("Wheels channels:"+wheelsChannels.size());
		 Object[] wheelsOChannels = wheelsChannels.toArray();
		 Arrays.sort(wheelsOChannels); // make sure of order
		 if(DEBUG)
			 System.out.println("WheelsOChannels sorted size:"+wheelsOChannels.length);
		 for(Object wheelsAChannels: wheelsOChannels) {
			 Map<String, Object> sortedWheelProp = wheels.get(wheelsAChannels); //wheelsAChannels sorted
			 if(DEBUG)
				 System.out.println(wheelsAChannels+" of number of SortedWheelProp:"+sortedWheelProp.size());
			 WHEEL[(int)wheelsAChannels] = new TypedWrapper(sortedWheelProp);
		 }	
	}
	/**
	 * Build that array of logical unit numbers, or 'channels' to talk to Marlinspike(s)
	 */
	private void extractLUN() {
		//Map<String, Map<Integer, Map<String, Object>>> globalConfigs;
		 Map<Integer, Map<String, Object>> luns = globalConfigs.get("LUN");
		 if(luns == null)
			 throw new RuntimeException("No LUNs defined in configuration, hence, nothing to configure.");
		 if(DEBUG)
			 System.out.println("LUN size:"+luns.size());
		 LUN = new TypedWrapper[luns.size()];
		 Set<Integer> lunsChannels = luns.keySet();
		 if(DEBUG)
			 System.out.println("LUNs channels:"+lunsChannels.size());
		 Object[] lunsOChannels = lunsChannels.toArray();
		 Arrays.sort(lunsOChannels); // make sure of order
		 if(DEBUG)
			 System.out.println("lunsOChannels sorted size:"+lunsOChannels.length);
		 for(Object lunsAChannels: lunsOChannels) {
			 Map<String, Object> sortedLunProp = luns.get(lunsAChannels); //lunsAChannels sorted integer channels of LUNs
			 if(DEBUG)
				 System.out.println(lunsAChannels+" of number of SortedLunProp:"+sortedLunProp.size());
			 LUN[(int)lunsAChannels] = new TypedWrapper(sortedLunProp);
		 }
	}
	/**
	 * Get the lun number of the lun with "Name:" property named name
	 * @return -1 if no such name exists
	 */
	@Override
	public int getLUN(String name) {
		int len = LUN.length;
        return IntStream.range(0, len).filter(i -> LUN[i].get("Name").equals(name)).findFirst().orElse(-1);          
	}
	
	@Override
	public String getNameByLUN(int lun) {
		return (String) LUN[lun].get("Name");
	}
	
	@Override
	public String getSlotByName(String name) {
		return (String) LUN[getLUN(name)].get("Slot");
	}
	
	@Override
	public RobotDiffDriveInterface getDiffDrive() {
		return robotDrive;
	}

	@Override
	public PIDParameterInterface getMotionPIDController() {
		return motionPIDController;
	}

	@Override
	public PIDParameterInterface getLeftMotorPIDController() {
		return robotDrive.getLeftWheel().getPIDController();
	}

	@Override
	public SetpointInfoInterface getLeftDistanceSetpointInfo() {
		return robotDrive.getLeftWheel().getTickSetpointInfo();
	}

	@Override
	public SetpointInfoInterface getIMUSetpointInfo() {
		return IMUSetpoint;
	}

	@Override
	public SetpointInfoInterface getLeftSpeedSetpointInfo() {
		return robotDrive.getLeftWheel().getSpeedsetPointInfo();
	}

	@Override
	public SetpointInfoInterface getLeftTickSetpointInfo() {
		return robotDrive.getLeftWheel().getTickSetpointInfo();
	}

	@Override
	public PIDParameterInterface getRightMotorPIDController() {
		return robotDrive.getRightWheel().getPIDController(); // for now, both the same one due to limited function
	}

	@Override
	public SetpointInfoInterface getRightDistanceSetpointInfo() {
		return robotDrive.getRightWheel().getTickSetpointInfo();
	}

	@Override
	public SetpointInfoInterface getRightSpeedSetpointInfo() {
		return robotDrive.getRightWheel().getSpeedsetPointInfo();
	}

	@Override
	public SetpointInfoInterface getRightTickSetpointInfo() {
		return robotDrive.getRightWheel().getTickSetpointInfo();
	}
	
	@Override
	public int getPowerScale() {
		return powerScale;
	}
	
	@Override
	public float getTemperatureThreshold() {
		return temperatureThreshold;
	}
	/**
	 * @return the controller
	 */
	public String getDataport() {
		return dataPort;
	}
	

	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append(String.format("Robot %s%nPower Scale=%b Drive: %s%nMotion Controller:%s%n IMU:%s%n",
				getName(),
				powerScale,
				robotDrive == null ? "NULL" : robotDrive.toString(),
				motionPIDController == null ? "NULL" : motionPIDController.toString(),
					IMUSetpoint == null ? "NULL" : IMUSetpoint.toString()));
		for(int i = 0; i < LUN.length; i++) {
			Set<Entry<String, Object>> x = LUN[i].entrySet();
			Iterator<Entry<String, Object>> it = x.iterator();
			while(it.hasNext()) {
				Entry<String, Object> e = it.next();
				sb.append(e.getKey());
				sb.append(":");
				sb.append(e.getValue());
				sb.append("\r\n");
			}
		}
		for(int i = 0; i < WHEEL.length; i++) {
			Set<Entry<String, Object>> x = WHEEL[i].entrySet();
			Iterator<Entry<String, Object>> it = x.iterator();
			while(it.hasNext()) {
				Entry<String, Object> e = it.next();
				sb.append(e.getKey());
				sb.append(":");
				sb.append(e.getValue());
				sb.append("\r\n");
			}
		}
		for(int i = 0; i < PID.length; i++) {
			Set<Entry<String, Object>> x = PID[i].entrySet();
			Iterator<Entry<String, Object>> it = x.iterator();
			while(it.hasNext()) {
				Entry<String, Object> e = it.next();
				sb.append(e.getKey());
				sb.append(":");
				sb.append(e.getValue());
				sb.append("\r\n");
			}
		}
		for(int i = 0; i < AXIS.length; i++) {
			Set<Entry<String, Object>> x = AXIS[i].entrySet();
			Iterator<Entry<String, Object>> it = x.iterator();
			while(it.hasNext()) {
				Entry<String, Object> e = it.next();
				sb.append(e.getKey());
				sb.append(":");
				sb.append(e.getValue());
				sb.append("\r\n");
			}
		}
		for(int i = 0; i < BUTTON.length; i++) {
			Set<Entry<String, Object>> x = BUTTON[i].entrySet();
			Iterator<Entry<String, Object>> it = x.iterator();
			while(it.hasNext()) {
				Entry<String, Object> e = it.next();
				sb.append(e.getKey());
				sb.append(":");
				sb.append(e.getValue());
				sb.append("\r\n");
			}
		}
		return sb.toString();
	}

	@Override
	public String getName() {
		return Props.toString("Name");
	}

	@Override
	public TypedWrapper[] getLUN() {
		return LUN;
	}

	@Override
	public TypedWrapper[] getWHEEL() {
		return WHEEL;
	}

	@Override
	public TypedWrapper[] getPID() {
		return PID;
	}

	@Override
	public TypedWrapper[] getAXIS() {
		return AXIS;
	}

	@Override
	public TypedWrapper[] getBUTTON() {
		return BUTTON;
	}

	public Map<String, Map<Integer, Map<String, Object>>> getGlobalConfigs() {
		return globalConfigs;
	}

	@Override
	public String getHostName() {
		return robotName;
	}

}
