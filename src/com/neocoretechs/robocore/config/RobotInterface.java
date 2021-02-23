package com.neocoretechs.robocore.config;

import com.neocoretechs.robocore.PID.PIDParameterInterface;
import com.neocoretechs.robocore.PID.SetpointInfoInterface;
import com.neocoretechs.robocore.affectors.AffectorInterface;
import com.neocoretechs.robocore.propulsion.RobotDiffDriveInterface;

/**
 * This is the top level interface that describes the essence of a Robot as it is
 * used in this framework.<br/>
 * Fundamentally, it is a collection of PID controller setpoints, configurations as to the physical locations of 
 * various hardware controller slots and channels, and some PID controllers that are handed into the
 * various subsystems.<br/>
 * 'Essence' may be a bit of a stretch, but certainly these parameters are the primary ones that differ
 * from robot to robot that use this common framework based on RosJavaLite and the MarlinSpike
 * realtime unit.
 * 
 * @author Jonathan Groff (C) NeoCoreTechs 2020
 *
 */
public interface RobotInterface {
	public String getName(); // The name of the Robot. Each one is special and unique.
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
	public AffectorInterface getAffectors();
	public boolean isIndoor();
	public float getTemperatureThreshold();
	public TypedWrapper[] getLUN();
	public TypedWrapper[] getWHEEL();
	public TypedWrapper[] getPID();
	public TypedWrapper[] getAXIS();
	public TypedWrapper[] getBUTTON();
}
