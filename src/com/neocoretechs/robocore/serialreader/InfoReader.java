package com.neocoretechs.robocore.serialreader;

import java.io.FileReader;
import java.io.IOException;
import java.nio.CharBuffer;


/**
 * Gather the cpuinfo to determine machine type, which dictates port configurations
 * We also gather the serial number of the CPU and the hardware revision
 * @author jg (C) NeoCoreTechs 2017
 *
 */
public final class InfoReader {
	/**
	 * Get the hardware type, revision, and serial from /proc/cpuinfo (linux only of course).
	 * @return 3 element string array of hardware type, revision, and serial
	 * @throws IOException If the format is janky
	 */
	public static String[] readInfo() throws IOException {
		FileReader fr = new FileReader("/proc/cpuinfo");
		CharBuffer barg = CharBuffer.allocate(2048);
		while( fr.read(barg) != -1);
		fr.close();
		String bargs = new String(barg.array());
		//
		int hardPos = bargs.indexOf("Hardware");
		if( hardPos == -1)
			throw new IOException("Can't find Hardware type in cpuinfo");
		int colPos = bargs.indexOf(':',hardPos)+1;
		if( colPos == -1) {
			throw new IOException("Can't find Hardware type in cpuinfo");
		}
		String bhard = bargs.substring(colPos+1);
		//
		int revPos = bargs.indexOf("Revision");
		if( revPos == -1)
			throw new IOException("Can't find Hardware revision in cpuinfo");
		colPos = bargs.indexOf(':',hardPos)+1;
		if( colPos == -1) {
			throw new IOException("Can't find Hardware revision in cpuinfo");
		}
		String brev = bargs.substring(colPos+1);
		//
		// May not have serial, Odroid C2 does not
		String bser = "000000000000000";
		int serPos = bargs.indexOf("Serial");
		if( serPos != -1) {
			colPos = bargs.indexOf(':',serPos)+1;
			if( colPos != -1) {
				bser = bargs.substring(colPos+1);
			}
		}
		//
		return new String[]{bhard, brev, bser};
	
	}
	public static void main(String[] args) throws Exception{
		System.out.println(InfoReader.readInfo());
	}
}
