package com.neocoretechs.robocore;

import java.io.IOException;

import com.neocoretechs.robocore.propulsion.MotorControlInterface2D;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;

/**
 * The MEGA control endpoint that controls serial data.
 * This class talks to the serial drivers that communicate with the attached USB SBC. In fact, this is code that talks to the
 * RS232 board that converts to TTL that talks to the SBC that runs the embedded code that manages
 * the comm to the motor controller.
 * The relevant methods generate Twist messages that can be multiplexed to the Ros bus
 * and sent further on.
 * Increasing levels of complexity of motion control with options to move in polar arcs, set absolute speeds, etc.
 * Absolute and relative motion with the fusion of IMU data if desired.
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2017,2020
 *
 */
public class MegaControl implements MotorControlInterface2D, PWMControlInterface {
	public static boolean DEBUG = true;
	//float yawIMURads; = twistInfo.imuTheta
	int yawTargetDegrees;
	int targetDistance;
	int targetTime; 
	float[] accelDeltas;
	int[] ranges;
	boolean init = true;
	Object mutex = new Object();

	/* Stop the robot if it hasn't received a movement command in this number of milliseconds */
	public static int AUTO_STOP_INTERVAL = 2000;
	long lastMotorCommand = AUTO_STOP_INTERVAL;
	/* The base and odometry frames */
	//String baseFrame = "/base_link";
	//String odomFrame = "/odom";
	
	protected static boolean moving = false; // is the base in motion?

	public MegaControl() {}
	
	public synchronized void setAbsoluteDiffDriveSpeed(int slot1, int channel1, int leftWheelSpeed, int slot2, int channel2, int rightWheelSpeed) throws IOException {
		String motorCommand1 = "G5 Z"+slot1+" C"+channel1+" P"+String.valueOf(leftWheelSpeed);
		ByteSerialDataPort.getInstance().writeLine(motorCommand1);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}
		String motorCommand2 = "G5 Z"+slot2+" C"+channel2+" P"+String.valueOf(rightWheelSpeed);
		ByteSerialDataPort.getInstance().writeLine(motorCommand2);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}
	}
	
	public synchronized void setAbsolutePWMLevel(int slot, int channel, int pwmLevel) throws IOException {
		String pwmCommand1 = "G5 Z"+slot+" C"+channel+" X"+pwmLevel;
		ByteSerialDataPort.getInstance().writeLine(pwmCommand1);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}
	}
	
	@Override
	public void commandStop() throws IOException {
		String motorCommand1 = "M799";
		ByteSerialDataPort.getInstance().writeLine(motorCommand1);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}
		
	}

	@Override
	public void setAbsolutePWMLevel(int slot1, int channel1, int leftWheelSpeed, int slot2, int channel2, int rightWheelSpeed) throws IOException {
		String motorCommand1 = "G5 Z"+slot1+" C"+channel1+" X"+String.valueOf(leftWheelSpeed);
		ByteSerialDataPort.getInstance().writeLine(motorCommand1);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}
		String motorCommand2 = "G5 Z"+slot2+" C"+channel2+" X"+String.valueOf(rightWheelSpeed);
		ByteSerialDataPort.getInstance().writeLine(motorCommand2);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}
		
	}
	public static void main(String[] args) throws Exception {
		if( args.length < 1 ) {
			System.out.println("Usage: java -cp <classpath> com.neocoretechs.robocore.MegaControl");
		}
		MegaControl mc = new MegaControl();
		//mc.setMotorSpeed(100.0f, 1f);
		/*
		 * TODO: make this something
		mc.moveRobotAbsolute(.5f, 45, 100, 1);
		mc.moveRobotAbsolute(-.25f, 45, 100, 1);
		mc.moveRobotAbsolute(.25f, 45, 100, 1);
		mc.moveRobotAbsolute(-.25f, 225, 100, 1);
		mc.moveRobotAbsolute(.0f, 0, 100, 1);
		*/
		
		//mc.moveRobotRelative(.5f, 45, 100);
		//mc.moveRobotRelative(-.25f, 45, 100);
		//mc.moveRobotRelative(.25f, 45, 100);
		//mc.moveRobotRelative(-.25f, 225, 100);
		//mc.moveRobotRelative(.0f, 0, 100);

	}

}



