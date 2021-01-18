package com.neocoretechs.robocore.propulsion;
/**
 * Interface to represent a differential drive for a robot. Presumes 2 driven wheels
 * of type DrivenWheelInterface. Also has methods to allow communication with realtime component by
 * specifying the parameters of the physical or virtual controller the drives the
 * underlying physical propulsion system.
 * @author Jonathan Groff (C) NeoCoreTechs 2020
 *
 */
public interface RobotDiffDriveInterface {
	public DrivenWheelInterface getLeftWheel();
	public DrivenWheelInterface getRightWheel();
	public void setDriveWheels(DrivenWheelInterface leftWheels, DrivenWheelInterface rightWheel);
	public float getWheelTrack();
	public boolean isIndoor(); // determines whether power should be reduced for safe operation
	public int getLeftWheelChannel();
	public int getRightWheelChannel();
	public int getControllerSlot();
	public String getControllerAxisPropertyName();
	public int getControllerAxisX(); //derived by Props.toInt(getControllerAxisPropertyName()+"X");
	public int getControllerAxisY(); //derived by Props.toInt(getControllerAxisPropertyName()+"Y");
}
