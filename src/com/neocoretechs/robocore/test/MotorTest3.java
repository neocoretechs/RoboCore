package com.neocoretechs.robocore.test;

import com.neocoretechs.robocore.MotorControl;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;

public class MotorTest3 {
	public static void main(String[] args) throws Exception {
		AsynchDemuxer.getInstance().config(); // will read /home/pi/startup.gcode and begin demux of real time Mega2560 data
		MotorControl motoCon = new MotorControl();
		for(int i = 0 ; i < 5000; i++) {
			motoCon.setAbsoluteMotorSpeed(500, 0); // half power channel 1,2 / 10
			System.out.println("Forward "+i);
			Thread.sleep(85);
			motoCon.commandStop();
			System.out.println("stop1 "+i);
			Thread.sleep(85);
			motoCon.setAbsoluteMotorSpeed(-500, 0);
			Thread.sleep(85);
			System.out.println("Reverse "+i);
			motoCon.commandStop();
			System.out.println("stop2 "+i);
			Thread.sleep(85);
		}
	}
}
