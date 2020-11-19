package com.neocoretechs.robocore.propulsion;

public interface RobotDiffDriveInterface {
	public DrivenWheelInterface getLeftWheel();
	public DrivenWheelInterface getRightWheel();
	public void setDriveWheels(DrivenWheelInterface leftWheels, DrivenWheelInterface rightWheel);
	public int getLeftWheelChannel();
	public int getRightWheelChannel();
	public int getControllerSlot();
}
