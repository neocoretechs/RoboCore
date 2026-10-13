package com.neocoretechs.robocore.propulsion;

import java.io.IOException;
import java.util.ArrayList;

import com.neocoretechs.robocore.config.Robot;
import com.neocoretechs.robocore.marlinspike.SlotHandler;

/**
 * Interface to represent a differential drive for a robot. Presumes 2 driven wheels
 * of type DrivenWheelInterface. Also has methods to allow communication with realtime component by
 * specifying the parameters of the physical or virtual controller the drives the
 * underlying physical propulsion system.<p/>
 * The concept of slots and channels for the controllers allows you to allocate configuration to one or more physical 
 * cards/logical controllers. For instance, one microcontroller might have one slot with a logical controller defined having
 * 2 channels for left and right, or be allocated across to different physical cards with each having the same slot and channel.
 * @author Jonathan Groff (C) NeoCoreTechs 2020
 *
 */
public interface RobotDiffDriveInterface {
	public DrivenWheelInterface getWheel(ArrayList<DrivenWheelInterface> slots, int chan);
	public int getControllerAxisX(); //derived by Props.toInt(getControllerAxisPropertyName()+"X");
	public int getControllerAxisY(); //derived by Props.toInt(getControllerAxisPropertyName()+"Y");
	public ArrayList<DrivenWheelInterface> getWheels(int slot);
	public int getChannels();
	public ArrayList<ArrayList<DrivenWheelInterface>> getWheels();
	public SlotHandler getSlotHandler(int slot);
	public void createControllers(Robot robot) throws IOException;
}
