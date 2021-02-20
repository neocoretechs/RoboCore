package com.neocoretechs.robocore.config;

import java.io.Serializable;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Arrays;
import java.util.Collection;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import com.neocoretechs.robocore.PID.IMUSetpointInfo;
import com.neocoretechs.robocore.PID.MotionPIDController;
import com.neocoretechs.robocore.PID.PIDParameterInterface;
import com.neocoretechs.robocore.PID.SetpointInfoInterface;
import com.neocoretechs.robocore.affectors.AffectorInterface;
import com.neocoretechs.robocore.affectors.Affectors;
import com.neocoretechs.robocore.propulsion.RobotDiffDrive;
import com.neocoretechs.robocore.propulsion.RobotDiffDriveInterface;
/**
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
 * @author groff
 *
 */
public class Robot implements RobotInterface, Serializable {
	private static final long serialVersionUID = 1L;
	MotionPIDController motionPIDController;
	RobotDiffDriveInterface robotDrive;
	IMUSetpointInfo IMUSetpoint;
	AffectorInterface affectors;
	// These arrays are by 'channel'
	private TypedWrapper[] LUN;
	private TypedWrapper[] PID;
	private TypedWrapper[] AXIS;
	private Map<String, Map<Integer, Map<String, Object>>> globalConfigs;
	public Robot() {
		try {
			globalConfigs = Props.collectivizeProps();
		} catch (IllegalAccessException e) {
			throw new RuntimeException(e);
		}
		extractLUN();
		extractPID();
		extractAXIS();
		robotDrive = new RobotDiffDrive(LUN, AXIS, PID);
		//kp, ki, kd, ko, pidRate (hz)
		motionPIDController = new MotionPIDController(Props.toFloat("CrosstrackKp"), 
														Props.toFloat("CrosstrackKd"), 
														Props.toFloat("CrosstrackKi"), 
														Props.toFloat("CrosstrackKo"), 
														Props.toInt("CrosstrackPIDRate"));
		IMUSetpoint = new IMUSetpointInfo();
		IMUSetpoint.setMaximum(Props.toFloat("MaxIMUDeviationDegrees")); // max deviation allowed from course
		IMUSetpoint.setMinimum(Props.toFloat("MinIMUDeviationDegrees")); // min deviation
		affectors = new Affectors();
	}

	private void extractAXIS() {
		//Map<String, Map<Integer, Map<String, Object>>> globalConfigs;
		 Map<Integer, Map<String, Object>> axis = globalConfigs.get("AXIS");
		 AXIS = new TypedWrapper[axis.size()];
		 Set<Integer> axisChannels = axis.keySet();
		 Object[] axisOChannels = axisChannels.toArray();
		 Arrays.sort(axisOChannels); // make sure of order
		 for(Object axisAChannels: axisOChannels) {
			 Map<String, Object> sortedAxisProp = axis.get(axisAChannels); //lunsAChannels sorted integer channels of LUNs
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
	/**
	 * Build that array of logical unit numbers, or 'channels' to talk to Marlinspike(s)
	 */
	private void extractLUN() {
		//Map<String, Map<Integer, Map<String, Object>>> globalConfigs;
		 Map<Integer, Map<String, Object>> luns = globalConfigs.get("LUN");
		 LUN = new TypedWrapper[luns.size()];
		 Set<Integer> lunsChannels = luns.keySet();
		 Object[] lunsOChannels = lunsChannels.toArray();
		 Arrays.sort(lunsOChannels); // make sure of order
		 for(Object lunsAChannels: lunsOChannels) {
			 Map<String, Object> sortedLunProp = luns.get(lunsAChannels); //lunsAChannels sorted integer channels of LUNs
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
	
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append(String.format("Robot %s\r\nDrive: %s\r\nMotion Controller:%s\r\n IMU:%s\r\nAffectors:%s",
				getName(),
				robotDrive == null ? "NULL" : robotDrive.toString(),
				motionPIDController == null ? "NULL" : motionPIDController.toString(),
					IMUSetpoint == null ? "NULL" : IMUSetpoint.toString(),
							affectors == null ? "NULL" : affectors.toString()));
		Set<Entry<String, Map<Integer, Map<String, Object>>>> props = globalConfigs.entrySet();
		for(Entry<String, Map<Integer, Map<String, Object>>> e : props ) {
			sb.append(e.getKey());
			sb.append("\r\n");
			Map<Integer, Map<String, Object>> prop1 = e.getValue();
			Set<Integer> iset = prop1.keySet();
			Set<Entry<Integer, Map<String, Object>>> jset = prop1.entrySet();
			Iterator<Entry<Integer, Map<String, Object>>> it = jset.iterator();
			for(Integer i: iset) {
				sb.append("[");
				sb.append(i);
				sb.append("].");
				Entry<Integer, Map<String, Object>> elem = it.next();
				Map<String,Object> o = elem.getValue();
				Set<String> s = o.keySet();
				Collection<Object> c = o.values();
				Iterator<String> its = s.iterator();
				Iterator<Object> ito = c.iterator();
				while(its.hasNext()) {
					sb.append(its.next());
					sb.append(":");
					sb.append(ito.next());
					sb.append("\r\n");
				}
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

}
