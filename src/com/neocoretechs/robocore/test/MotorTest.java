package com.neocoretechs.robocore.test;

import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;

public class MotorTest {
	public static void main(String[] args) throws Exception {
		AsynchDemuxer ad = new AsynchDemuxer();
		ad.connect(ByteSerialDataPort.getInstance());
		String motorCommand = "M0"; // turn off realtime output
		AsynchDemuxer.addWrite(ad,motorCommand);
		// config smart controller channel 1 default direction
		motorCommand = "M2 C1 E0";
		AsynchDemuxer.addWrite(ad,motorCommand);
		// config smart controller channel 2, invert direction
		motorCommand = "M2 C2 E1"; 
		AsynchDemuxer.addWrite(ad,motorCommand);
		motorCommand = "M6 S10"; //scale by 10 to slow default speed
		AsynchDemuxer.addWrite(ad,motorCommand);
		motorCommand = "M33 P22 D60 E0"; //link the ultrasonic sensor pointing forward(E0) on pin 22 to stop motor at distance 60cm
		AsynchDemuxer.addWrite(ad,motorCommand);
		for(int i = 0 ; i < 5000; i++) {
			motorCommand = "G5 C1 P500"; // half power channel 1 / 10
			AsynchDemuxer.addWrite(ad,motorCommand);
			motorCommand = "G5 C2 P500"; // half power channel 1 / 10
			AsynchDemuxer.addWrite(ad,motorCommand);
			System.out.println("Forward "+i);
			motorCommand = "G5 C1 P0"; // stop
			AsynchDemuxer.addWrite(ad,motorCommand);
			motorCommand = "G5 C2 P0"; // stop
			AsynchDemuxer.addWrite(ad,motorCommand);
			System.out.println("stop1 "+i);
			//Thread.sleep(100);
			motorCommand = "G5 C1 P-500"; // half power channel 1 / 10 reverse
			AsynchDemuxer.addWrite(ad,motorCommand);
			//Thread.sleep(100);
			motorCommand = "G5 C2 P-500"; // half power channel 1 / 10 reverse
			AsynchDemuxer.addWrite(ad,motorCommand);
			System.out.println("Reverse "+i);
			motorCommand = "G5 C1 P0"; // stop
			AsynchDemuxer.addWrite(ad,motorCommand);
			motorCommand = "G5 C2 P0"; // stop
			AsynchDemuxer.addWrite(ad,motorCommand);
			System.out.println("stop2 "+i);
		}
	}
}
