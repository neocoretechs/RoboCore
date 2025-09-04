package com.neocoretechs.robocore.serialreader;

import java.io.IOException;

/**
 * Main interface contract for port that reads/writes data from attached device or standalone process.
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020
 *
 */
public interface DataPortInterface {
	public void connect(boolean writeable) throws IOException;
	public boolean isConnected() throws IOException;
	public int read() throws IOException;
	public void write(int c) throws IOException;
	public void close();
	public String readLine();
	public int bytesToRead() throws IOException;
	public void writeLine(String output) throws IOException;
	public String getPortName();
	public String stringSettings();

}
