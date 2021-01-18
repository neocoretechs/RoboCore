package com.neocoretechs.robocore.config;

import com.neocoretechs.robocore.PID.IMUSetpointInfo;
import com.neocoretechs.robocore.PID.MotionPIDController;
import com.neocoretechs.robocore.PID.PIDParameterInterface;
import com.neocoretechs.robocore.PID.SetpointInfoInterface;
import com.neocoretechs.robocore.affectors.AffectorInterface;
import com.neocoretechs.robocore.propulsion.RobotDiffDrive;
import com.neocoretechs.robocore.propulsion.RobotDiffDriveInterface;

public class Robot implements RobotInterface {
	MotionPIDController motionPIDController;
	RobotDiffDriveInterface robotDrive;
	IMUSetpointInfo IMUSetpoint;
	public Robot() {
		robotDrive = new RobotDiffDrive();
		//kp, ki, kd, ko, pidRate (hz)
		motionPIDController = new MotionPIDController(Props.toFloat("CrosstrackKp"), 
														Props.toFloat("CrosstrackKd"), 
														Props.toFloat("CrosstrackKi"), 
														Props.toFloat("CrosstrackKo"), 
														Props.toInt("CrosstrackPIDRate"));
		IMUSetpoint = new IMUSetpointInfo();
		IMUSetpoint.setMaximum(Props.toFloat("MaxIMUDeviationDegrees")); // max deviation allowed from course
		IMUSetpoint.setMinimum(Props.toFloat("MinIMUDeviationDegrees")); // min deviation
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
		return String.format("Robot %s\r\nDrive: %s\r\nMotion Controller:%s\r\n IMU:%s",
				getName(),
				robotDrive == null ? "NULL" : robotDrive.toString(),
				motionPIDController == null ? "NULL" : motionPIDController.toString(),
					IMUSetpoint == null ? "NULL" : IMUSetpoint.toString());
	}

	@Override
	public String getName() {
		return Props.toString("Name");
	}

	@Override
	public AffectorInterface getAffectors() {
		// TODO Auto-generated method stub
		return null;
	}

}
