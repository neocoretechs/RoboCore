package com.neocoretechs.robocore;

import com.neocoretechs.robocore.PID.IMUSetpointInfo;
import com.neocoretechs.robocore.PID.MotionPIDController;
import com.neocoretechs.robocore.PID.PIDParameterInterface;
import com.neocoretechs.robocore.PID.SetpointInfoInterface;
import com.neocoretechs.robocore.propulsion.ROSCOE1;
import com.neocoretechs.robocore.propulsion.ROSCOE2;
import com.neocoretechs.robocore.propulsion.RobotDiffDriveInterface;

public class Robot implements RobotInterface {
	MotionPIDController motionPIDController;
	RobotDiffDriveInterface robot;
	IMUSetpointInfo IMUSetpoint;
	public Robot(int robotType) {
		switch(robotType) {
		case 1:
			robot= new ROSCOE1(0, 1, 2, ROSCOE1.wheelTrack); // controller slot, left channel, right channel
			break;
		default:
			robot= new ROSCOE2(0, 1, 2, ROSCOE2.wheelTrack); // controller slot, left channel, right channel
			break;
		}
		//kp, ki, kd, ko, pidRate (hz)
		motionPIDController = new MotionPIDController(1.5f, 0.9f, 3.0f, 1.0f, 1);
		IMUSetpoint = new IMUSetpointInfo();
		IMUSetpoint.setMaximum(45); // max deviation allowed from course
	}

	@Override
	public RobotDiffDriveInterface getDiffDrive() {
		return robot;
	}

	@Override
	public PIDParameterInterface getMotionPIDController() {
		return motionPIDController;
	}

	@Override
	public PIDParameterInterface getLeftMotorPIDController() {
		return robot.getLeftWheel().getPIDController();
	}

	@Override
	public SetpointInfoInterface getLeftDistanceSetpointInfo() {
		return robot.getLeftWheel().getTickSetpointInfo();
	}

	@Override
	public SetpointInfoInterface getIMUSetpointInfo() {
		return IMUSetpoint;
	}

	@Override
	public SetpointInfoInterface getLeftSpeedSetpointInfo() {
		return robot.getLeftWheel().getSpeedsetPointInfo();
	}

	@Override
	public SetpointInfoInterface getLeftTickSetpointInfo() {
		return robot.getLeftWheel().getTickSetpointInfo();
	}

	@Override
	public PIDParameterInterface getRightMotorPIDController() {
		return robot.getRightWheel().getPIDController(); // for now, both the same one due to limited function
	}

	@Override
	public SetpointInfoInterface getRightDistanceSetpointInfo() {
		return robot.getRightWheel().getTickSetpointInfo();
	}

	@Override
	public SetpointInfoInterface getRightSpeedSetpointInfo() {
		return robot.getRightWheel().getSpeedsetPointInfo();
	}

	@Override
	public SetpointInfoInterface getRightTickSetpointInfo() {
		return robot.getRightWheel().getTickSetpointInfo();
	}
	
	public String toString() {
		return String.format("Robot %s\r\nMotion Controller:%s\r\n IMU:%s", 
				robot == null ? "NULL" : robot.toString(),
				motionPIDController == null ? "NULL" : motionPIDController.toString(),
					IMUSetpoint == null ? "NULL" : IMUSetpoint.toString());
	}

}
