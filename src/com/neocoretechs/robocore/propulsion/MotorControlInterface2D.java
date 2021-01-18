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
	public String reportAllControllerStatus() throws IOException;
	public String reportSystemId() throws IOException;
	public void commandStop() throws IOException;
	public String commandReset() throws IOException;
	public void setAbsoluteDiffDriveSpeed(int slot1, int channel1, int channel1Speed, int slot2, int channel2, int channel2Speed) throws IOException;
	public void setAbsolutePWMLevel(int slot1, int channel1, int leftWheelSpeed, int slot2, int channel2, int rightWheelSpeed) throws IOException;
	public void setAbsolutePWMLevel(int slot, int channel, int pwmLevel)throws IOException;
	public void setAffectorDriveSpeed(int slot1, int channel1, int affectorSpeed) throws IOException;
}
