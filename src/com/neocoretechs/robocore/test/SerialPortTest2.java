package com.neocoretechs.robocore.test;

import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;

public class SerialPortTest2 {
	public static void main(String[] args) throws Exception {
		String motorCommand = "M301 P22";
		ByteSerialDataPort.getInstance().writeLine(motorCommand);
		for(int i = 0 ; i < 5000; i++) {
			System.out.println("read "+i);
			String in1 = ByteSerialDataPort.getInstance().readLine();
			System.out.println(in1);
			while( !in1.equals("<ultrasonic>")) {// soak up status headers
				System.out.println("POSSIBLE ERROR AT "+i+" "+in1);
				in1 = ByteSerialDataPort.getInstance().readLine();
			}
			System.out.println(in1); 
			in1 = ByteSerialDataPort.getInstance().readLine();
			if( in1.charAt(0) != '1') 
				System.out.println("ERROR AT "+i);
			System.out.println(in1);
			in1 = ByteSerialDataPort.getInstance().readLine();
			// this one should always start with "2"
			if( in1.charAt(0) != '2') 
				System.out.println("ERROR AT "+i);
			System.out.println(in1);
			in1 = ByteSerialDataPort.getInstance().readLine();
			while( !in1.equals("</ultrasonic>")) {// soak up status headers
				System.out.println("POSSIBLE ERROR AT "+i+" "+in1);
				in1 = ByteSerialDataPort.getInstance().readLine();
			}
			//Thread.sleep(15);
		}
	}
}
