package com.neocoretechs.robocore;

import java.io.IOException;

import com.neocoretechs.robocore.MotorControl.TwistInfo;

/**
 * This interface is the cut between ARDrone land/sea/space (basically anything outside of standard air motor control)
 * Preliminary sensor fusion of IMU, ultrasonic range, and accelerometer deltas to determine baseline safe movement.
 * The purpose of this contract is to abstract various embedded and client server motor control interfaces.
 * @author jg
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
