package com.neocoretechs.robocore.test;

import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;

public class SerialPortTest {
	public static void main(String[] args) throws Exception {
		String motorCommand = "M700";
		for(int i = 0 ; i < 5000; i++) {
			ByteSerialDataPort.getInstance().writeLine(motorCommand);
			System.out.println("wrote "+i);
			String in1 = ByteSerialDataPort.getInstance().readLine();
			System.out.println(in1);
			while( !in1.startsWith("<status>")) {// soak up status headers
				System.out.println("POSSIBLE ERROR skipping:"+in1);
				in1 = ByteSerialDataPort.getInstance().readLine();
			}
			// got status header, soak up variable lines, look for markers
			boolean compiled = false;
			boolean me = false;
			while( in1.charAt(0) == '<') {
				in1 = ByteSerialDataPort.getInstance().readLine();
				if(in1.contains("Compiled")) {
					compiled = true;
				}
				if( in1.contains("Groff")) {
					me = true;
				}
				System.out.println(in1);
				if( in1.startsWith("</status>")) break;
			}
			if( !compiled || !me )
				System.out.println("ERROR AT "+i);
			//Thread.sleep(15);
		}
	}
}
