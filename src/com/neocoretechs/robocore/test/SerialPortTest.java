package com.neocoretechs.robocore.test;

import java.util.List;

import com.neocoretechs.robocore.machine.bridge.FileIOUtilities;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;

public class SerialPortTest {
	static AsynchDemuxer ad = new AsynchDemuxer();
	private static int THRESHOLD = 50;
	public static void main(String[] args) throws Exception {	
		ByteSerialDataPort.getInstance().connect(true);
		List<String> starts = FileIOUtilities.getConfig();
		for(String s : starts) {
			System.out.println("Startup GCode:"+s);
			command(s);
		}
		command("M115");
		command("M700");
		command("M701");
		command("M702");
		command("M703");
		command("M704");
		command("M705");
		command("M706");
		for(int i = 0; i < 10; i++) {
			command("M798 Z"+i); //motor controller status per channel
		}
	}
	
	public static void command(String code)throws Exception{
		ByteSerialDataPort.getInstance().writeLine(code);
		System.out.println("wrote "+code);
		Thread.sleep(1);
		int lines = 0;
		String line = null;
		long tim0 = System.currentTimeMillis();
		while(!ad.isLineTerminal((line=ByteSerialDataPort.getInstance().readLine()))) {
			System.out.println(lines+".) "+line);
			++lines;
			if(lines > THRESHOLD ) {
				System.out.println("TOO MANY LINES RETURNED FOR "+code);
				break;
			}
		}
		System.out.println(lines+" lines returned for "+code+" in "+(System.currentTimeMillis()-tim0)+" .ms");
	}
}
