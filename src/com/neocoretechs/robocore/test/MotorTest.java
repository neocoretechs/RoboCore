package com.neocoretechs.robocore.test;

import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;

public class MotorTest {
	public static void main(String[] args) throws Exception {
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
		Thread.sleep(100); // wait for uc slowness
		for(int i = 0 ; i < 5000; i++) {
			motorCommand = "G5 C1 P500"; // half power channel 1 / 10
			ByteSerialDataPort.getInstance().writeLine(motorCommand);
			//Thread.sleep(100);
			motorCommand = "G5 C2 P500"; // half power channel 1 / 10
			ByteSerialDataPort.getInstance().writeLine(motorCommand);
			System.out.println("Forward "+i);
			//Thread.sleep(100);
			motorCommand = "G5 C1 P0"; // stop
			ByteSerialDataPort.getInstance().writeLine(motorCommand);
			//Thread.sleep(100);
			motorCommand = "G5 C2 P0"; // stop
			ByteSerialDataPort.getInstance().writeLine(motorCommand);
			System.out.println("stop1 "+i);
			//Thread.sleep(100);
			motorCommand = "G5 C1 P-500"; // half power channel 1 / 10 reverse
			ByteSerialDataPort.getInstance().writeLine(motorCommand);
			//Thread.sleep(100);
			motorCommand = "G5 C2 P-500"; // half power channel 1 / 10 reverse
			ByteSerialDataPort.getInstance().writeLine(motorCommand);
			System.out.println("Reverse "+i);
			//Thread.sleep(100);
			motorCommand = "G5 C1 P0"; // stop
			ByteSerialDataPort.getInstance().writeLine(motorCommand);
			//Thread.sleep(100);
			motorCommand = "G5 C2 P0"; // stop
			ByteSerialDataPort.getInstance().writeLine(motorCommand);
			System.out.println("stop2 "+i);
			//Thread.sleep(100);
			//Thread.sleep(15);
		}
	}
}
