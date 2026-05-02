package com.neocoretechs.robocore.config;

import java.io.IOException;
import java.util.HashMap;

import com.neocoretechs.robocore.PID.PIDParameterInterface;
import com.neocoretechs.robocore.PID.SetpointInfoInterface;
import com.neocoretechs.robocore.marlinspike.MarlinspikeManager;
import com.neocoretechs.robocore.propulsion.RobotDiffDriveInterface;

/**
 * This is the top level interface that describes the configuration of a Robot as it is
 * used in this framework.<br>
 * Fundamentally, it is a collection of PID controller setpoints, configurations as to the physical locations of 
 * various hardware controller slots and channels, and some PID controllers that are handed into the
 * various subsystems.<br>
 * This class represents a centralized configuration management instrument.
 * <pre>
 * Read the parameterized properties from the properties class. Config as follows by LUN, logical unit number:<p>
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
 * @see Robot
 * @author Jonathan Groff (C) NeoCoreTechs 2021,2026
 *
 */
public interface RobotInterface {
	public boolean[] active();
	public String getName(); // The name of the Robot. Each one is special and unique.
	public String getHostName(); // the individual processor in the robot internal network.
	public String getDataPort();
	public MarlinspikeManager getManager();
	public void configureMarlinspike() throws IOException;
	public HashMap<String, Boolean> getOperating();
	public RobotDiffDriveInterface getDiffDrive(); // embodies different differential drive characteristics
	public PIDParameterInterface getMotionPIDController(); // PID control parameters for trave by the different diff drives
	public PIDParameterInterface getLeftMotorPIDController(); // motor PID control accounting for different drivers and propulsion motors
	public PIDParameterInterface getRightMotorPIDController(); // motor PID control accounting for different drivers and propulsion motors
	public SetpointInfoInterface getLeftDistanceSetpointInfo(); // The wheel diameters and baselines from the diff drive affecting travel
	public SetpointInfoInterface getRightDistanceSetpointInfo(); // The wheel diameters and baselines from the diff drive affecting travel
	public SetpointInfoInterface getIMUSetpointInfo(); // IMU navigation and its interaction with the above different drive characteristics
	public SetpointInfoInterface getLeftSpeedSetpointInfo(); // Maximum speeds and indoor/outdoor characteristics
	public SetpointInfoInterface getRightSpeedSetpointInfo(); // Maximum speeds and indoor/outdoor characteristics
	public SetpointInfoInterface getLeftTickSetpointInfo(); // Ticks and odometry related to the differing diff drives
	public SetpointInfoInterface getRightTickSetpointInfo(); // Ticks and odometry related to the differing diff drives
	public int getPowerScale();
	public float getTemperatureThreshold();
	public TypedWrapper[] getLUN();
	public TypedWrapper[] getWHEEL();
	public TypedWrapper[] getPID();
	public TypedWrapper[] getAXIS();
	public TypedWrapper[] getBUTTON();
	public int getLUN(String name);
	public String getNameByLUN(int lun);
	public String getSlotByName(String name);
}
