package com.neocoretechs.robocore.test;

import com.neocoretechs.robocore.MotorControl;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;

public class MotorTest2 {
	public static void main(String[] args) throws Exception {
		MotorControl motoCon = new MotorControl();
		String motorCommand = "M0"; // turn off realtime output
		ByteSerialDataPort.getInstance().writeLine(motorCommand);
		// config smart controller channel 1 default direction
		motorCommand = "M2 C1 E0";
		ByteSerialDataPort.getInstance().writeLine(motorCommand);
		Thread.sleep(100); // wait for uc slowness
		// config smart controller channel 2, invert direction
		motorCommand = "M2 C2 E1"; 
		ByteSerialDataPort.getInstance().writeLine(motorCommand);
		Thread.sleep(100); // wait for uc slowness
		motorCommand = "M6 S10"; //scale by 10 to slow default speed
		ByteSerialDataPort.getInstance().writeLine(motorCommand);
		Thread.sleep(100); // wait for uc slowness
		motorCommand = "M33 P22 D60 E0"; //link the ultrasonic sensor pointing forward(E0) on pin 22 to stop motor at distance 60cm
		ByteSerialDataPort.getInstance().writeLine(motorCommand);
		// use the higher level MotorControl to send commands to motor, thus testing that portion
		Thread.sleep(100); // wait for uc slowness
		for(int i = 0 ; i < 5000; i++) {
			motoCon.setAbsoluteMotorSpeed(500, 500); // half power channel 1,2 / 10
			System.out.println("Forward "+i);
			motoCon.setAbsoluteMotorSpeed(0,0);
			System.out.println("stop1 "+i);
			motoCon.setAbsoluteMotorSpeed(-500, -500);
			System.out.println("Reverse "+i);
			motoCon.setAbsoluteMotorSpeed(0,0);
			System.out.println("stop2 "+i);
		}
	}
}
