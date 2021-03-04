package com.neocoretechs.robocore.test;

import java.util.List;

import com.neocoretechs.robocore.machine.bridge.FileIOUtilities;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;
/**
 * Call with the first command line argument the name of the serial data port.
 * Use the main method of ByteSerialDataPort if an enumeration of all data ports is required.
 * @author Jonathan Groff (C) NeoCoreTechs 2021
 *
 */
@Deprecated
public class SerialPortTest {
	static AsynchDemuxer ad = new AsynchDemuxer();
	ByteSerialDataPort bsdp;
	private static int THRESHOLD = 50;
	public static void main(String[] args) throws Exception {
		SerialPortTest spt = new SerialPortTest();
		spt.bsdp = new ByteSerialDataPort(args[0]);
		spt.bsdp.connect(true);
		List<String> starts = FileIOUtilities.getConfig();
		for(String s : starts) {
			System.out.println("Startup GCode:"+s);
			spt.command(s);
		}
		spt.command("M115");
		spt.command("M700");
		spt.command("M701");
		spt.command("M702");
		spt.command("M703");
		spt.command("M704");
		spt.command("M705");
		spt.command("M706");
		for(int i = 0; i < 10; i++) {
			spt.command("M798 Z"+i); //motor controller status per channel
		}
	}
	
	public void command(String code)throws Exception{
		bsdp.writeLine(code);
		System.out.println("wrote "+code);
		Thread.sleep(1);
		int lines = 0;
		String line = null;
		long tim0 = System.currentTimeMillis();
		while(!ad.isLineTerminal((line=bsdp.readLine()))) {
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
