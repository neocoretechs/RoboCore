package com.neocoretechs.robocore.config;

import java.io.Serializable;
import java.util.Arrays;
import java.util.Collection;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentHashMap.KeySetView;

import com.neocoretechs.robocore.PID.IMUSetpointInfo;
import com.neocoretechs.robocore.PID.MotionPIDController;
import com.neocoretechs.robocore.PID.PIDParameterInterface;
import com.neocoretechs.robocore.PID.SetpointInfoInterface;
import com.neocoretechs.robocore.affectors.AffectorInterface;
import com.neocoretechs.robocore.affectors.Affectors;
import com.neocoretechs.robocore.propulsion.RobotDiffDrive;
import com.neocoretechs.robocore.propulsion.RobotDiffDriveInterface;
/**
 * This class represents a centralized configuration management instrument.
 * Read the paramaterized properties from the properties class. Config as follows by LUN, logical unit number:<p/>
 * LUN[0].Name:LeftWheel <br/>
 * LUN[0].NodeName:Control1 <br/>
 * LUN[0].Controller:/dev/ttyACM0 <br/>
 * LUN[0].Type:H-Bridge <br/>
 * LUN[0].Slot:0 <br/>
 * LUN[0].PWMPin0:8 <br/>
 * LUN[0].Channel:1 <br/>
 * LUN[0].EnablePin:24 <br/>
 * LUN[0].Direction:0 <br/>
 * LUN[0].EncoderPin:68 <br/>
 * LUN[0].MinValue:-1000 <br/>
 * LUN[0].MaxValue:1000 <br/>
 * PID[0].MotorPIDRate:1 <br/>
 * PID[0].MotorKp:1.0 <br/>
 * PID[0].MotorKd:1.0 <br/>
 * PID[0].MotorKi:1.0 <br/>
 * PID[0].MotorKo:1.0 <br/>
 * AXIS[0].AxisType:Stick <br/>
 * AXIS[0].AxisX:0 <br/>
 * AXIS[0].AxisY:2 <br/>
 * Also, BUTTON, WHEEL, etc as needed.<p/>
 * It may seem odd that values are redundant. For instance, wheel track would seem absolute, as would wheel diameter.
 * However, its possible that in a 4 wheel vehicle, diameter can vary slightly as in, say, the drive wheels of a tracked vehicle
 * and if the tracks are driven by 4 motors with 2 sets of cogs differing in size front to back the rotation rates wil
 * have to by synchronized despite the differing 'wheel' sizes. So to may the track vary slightly, and other instances I cant yet forsee.<p/>
 * With that in mind the parameters that would seem to warrant global status have been repeated so as to appear from the
 * perspective of each individual component, and if necessary, appear in aggregate as global. 
 * @author Jonathan Groff (C) NeoCoreTechs 2021
 *
 */
public class Robot implements RobotInterface, Serializable {
	public static boolean DEBUG = false;
	private static final long serialVersionUID = 1L;
	private boolean indoor = Props.toBoolean("IsIndoor"); // div power by ten indoor mode
	private int temperatureThreshold = Props.toInt("TemperatureThreshold");//40 C 104 F
	private MotionPIDController motionPIDController;
	private RobotDiffDriveInterface robotDrive;
	private IMUSetpointInfo IMUSetpoint;
	private AffectorInterface affectors;
	// These arrays are by 'channel'
	private TypedWrapper[] LUN;
	private TypedWrapper[] WHEEL;
	private TypedWrapper[] PID;
	private TypedWrapper[] AXIS;
	private TypedWrapper[] BUTTON;
	private Map<String, Map<Integer, Map<String, Object>>> globalConfigs;
	public Robot() {
		try {
			globalConfigs = Props.collectivizeProps();
		} catch (IllegalAccessException e) {
			throw new RuntimeException(e);
		}
		extractLUN();
		extractWHEEL();
		extractPID();
		extractAXIS();
		extractBUTTON();
		robotDrive = new RobotDiffDrive(LUN, WHEEL, AXIS, PID);
		//kp, ki, kd, ko, pidRate (hz)
		motionPIDController = new MotionPIDController(Props.toFloat("CrosstrackKp"), 
														Props.toFloat("CrosstrackKd"), 
														Props.toFloat("CrosstrackKi"), 
														Props.toFloat("CrosstrackKo"), 
														Props.toInt("CrosstrackPIDRate"));
		IMUSetpoint = new IMUSetpointInfo();
		IMUSetpoint.setMaximum(Props.toFloat("MaxIMUDeviationDegrees")); // max deviation allowed from course
		IMUSetpoint.setMinimum(Props.toFloat("MinIMUDeviationDegrees")); // min deviation
		affectors = new Affectors(LUN, AXIS, PID);
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
		 WHEEL = new TypedWrapper[wheels.size()];
		 Set<Integer> wheelsChannels = wheels.keySet();
		 Object[] wheelsOChannels = wheelsChannels.toArray();
		 Arrays.sort(wheelsOChannels); // make sure of order
		 for(Object wheelsAChannels: wheelsOChannels) {
			 Map<String, Object> sortedWheelProp = wheels.get(wheelsAChannels); //wheelsAChannels sorted
			 WHEEL[(int)wheelsAChannels] = new TypedWrapper(sortedWheelProp);
		 }	
	}
	/**
	 * Build that array of logical unit numbers, or 'channels' to talk to Marlinspike(s)
	 */
	private void extractLUN() {
		//Map<String, Map<Integer, Map<String, Object>>> globalConfigs;
		 Map<Integer, Map<String, Object>> luns = globalConfigs.get("LUN");
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
	public boolean isIndoor() {
		return indoor;
	}
	
	@Override
	public float getTemperatureThreshold() {
		return temperatureThreshold;
	}
	

	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append(String.format("Robot %s\r\nIndoor=%b Drive: %s\r\nMotion Controller:%s\r\n IMU:%s\r\nAffectors:%s",
				getName(),
				indoor,
				robotDrive == null ? "NULL" : robotDrive.toString(),
				motionPIDController == null ? "NULL" : motionPIDController.toString(),
					IMUSetpoint == null ? "NULL" : IMUSetpoint.toString(),
							affectors == null ? "NULL" : affectors.toString()));
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
	public AffectorInterface getAffectors() {
		return affectors;
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

}
