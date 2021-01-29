package com.neocoretechs.robocore.test;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;

public class SerialPortTest {
	static AsynchDemuxer ad = new AsynchDemuxer();
	private static int THRESHOLD = 50;
	public static void main(String[] args) throws Exception {	
		ByteSerialDataPort.getInstance().connect(true);
		command("M115");
		command("M700");
		command("M701");
		command("M702");
		command("M703");
		command("M704");
		command("M705");
		command("M706");
	}
	
	public static void command(String code)throws Exception{
		ByteSerialDataPort.getInstance().writeLine(code);
		System.out.println("wrote "+code);
		Thread.sleep(1);
		int lines = 0;
		String line = null;
		while(!ad.isLineTerminal((line=ByteSerialDataPort.getInstance().readLine()))) {
			System.out.println(lines+".) "+line);
			++lines;
			if(lines > THRESHOLD ) {
				System.out.println("TOO MANY LINES RETURNED FOR"+code);
				break;
			}
		}
		System.out.println(lines+" lines returned for "+code);
	}
}
