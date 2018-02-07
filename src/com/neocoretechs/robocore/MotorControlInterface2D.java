package com.neocoretechs.robocore;

import java.io.IOException;

import com.neocoretechs.robocore.MotorControl.TwistInfo;

/**
 * This interface could be the cut between land/sea/space robots. 
 * The method params represent sensor fusion of IMU, range, and other sensors to determine baseline safe movement.
 * An abstraction of motor control interfaces.
 * @author jg Copyright (C) NeoCoreTechs 2017,2018
 *
 */
public interface MotorControlInterface2D {
	public boolean move2DRelative(float yawIMURads, int yawTargetDegrees, int targetDistance, float[] accelDeltas, int[] ranges) throws IOException;
	public boolean move2DAbsolute(float yawIMURads, int yawTargetDegrees, int targetDistance, float[] accelDeltas, int[] ranges) throws IOException;
	public void commandStop() throws IOException;
	public TwistInfo moveRobotAbsolute(float yawIMURads, int yawTargetDegrees, int targetDistance) throws IOException;
	public TwistInfo moveRobotRelative(float yawIMURads, int yawTargetDegrees, int targetDistance) throws IOException;
	public int[] setMotorArcSpeed(float lin, float ang) throws IOException;
	public void setAbsoluteMotorSpeed(int channel1Speed, int channel2Speed) throws IOException;
	public void updateSpeed(int leftWheelSpeed, int rightWheelSpeed) throws IOException;
}
