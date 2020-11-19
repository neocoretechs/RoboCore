package com.neocoretechs.robocore.propulsion;

import java.io.IOException;

/**
 * This interface could be the cut between land/sea/space robots. 
 * The method params represent sensor fusion of IMU, range, and other sensors to determine baseline safe movement.
 * Multiple motor controllers occupy virtual 'slots' that correspond to hardware such as bridges.
 * Each controller 'slot' can accommodate several motor control channels.
 * Each channel receives a value representing a power level.
 * We address the left and right motor in a presumed multichannel controller at the same time to reduce latency.
 * Propulsion motors are addressed via controller slot, controller slot channel, and value.
 * An abstraction of motor control interfaces.
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2017,2018,2020
 *
 */
public interface MotorControlInterface2D {
	public boolean move2DRelative(float yawIMURads, int yawTargetDegrees, int targetDistance, float[] accelDeltas, int[] ranges) throws IOException;
	public boolean move2DAbsolute(float yawIMURads, int yawTargetDegrees, int targetDistance, float[] accelDeltas, int[] ranges) throws IOException;
	public void commandStop() throws IOException;
	//public TwistInfo moveRobotAbsolute(TwistInfo twistInfo, float yawIMURads, int yawTargetDegrees, int targetDistance) throws IOException;
	//public TwistInfo moveRobotRelative(TwistInfo twistInfo, float yawIMURads, int yawTargetDegrees, int targetDistance) throws IOException;
	//public int[] setMotorArcSpeed(int slot1, int channel1, int slot2, int channel2, float lin, float ang) throws IOException;
	public void setAbsoluteMotorSpeed(int slot1, int channel1, int channel1Speed, int slot2, int channel2, int channel2Speed) throws IOException;
	void updateSpeed(int slot1, int channel1, int leftWheelSpeed, int slot2, int channel2, int rightWheelSpeed) throws IOException;
}
