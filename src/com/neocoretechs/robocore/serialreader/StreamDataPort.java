package com.neocoretechs.robocore.serialreader;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;

/**
 * Functions as any DataPort, but reads from stream.
 * Provides raw data to/from and abstracted IO channel.
 * Intent is to read a file from upload stream and extract vals, putting them to the device
 * @author jg
 *
 */
public class StreamDataPort implements DataPortCommandInterface {
	protected String port;
	protected InputStream fin = null;
	protected OutputStream fout = null;
	protected BufferedReader br = null;
	protected BufferedWriter bw = null;
	protected boolean writeable = false;
	
	public StreamDataPort() {}
	
	public StreamDataPort(InputStream in, OutputStream out) {
		port = "Stream";
		fin = in;
		fout = out;
	}
	/**
	 * Assume stream connect is handled elsewhere before we get here, so do little
	 */
	@Override
	public void connect(boolean writeable) throws IOException { 
		this.writeable = writeable;
	}

	@Override
	public int read() throws IOException {
		return fin.read();
	}
	
	@Override
	public void write(int c) throws IOException {
		if( !writeable )
			throw new IOException("Attempted write on read-only resource "+port);
        fout.write(c);
	}
	
	@Override
	public void close() {
		if( br != null ) {
			try {
				br.close();
			} catch (IOException e) {}
			br = null;
		} else {
			if( fin != null ) {
				try {
					fin.close();
				} catch (IOException e) {}
				fin = null;
			}
		}
		if( bw != null ) {
			try {
				bw.flush();
				bw.close();
				bw = null;
			} catch (IOException e) {}
		} else {
			if( fout != null ) {
				try {
					fout.flush();
					fout.close();
					fout = null;
				} catch (IOException e) {}
			}
		}
	}

	@Override
	public String readLine() {
		//if( Props.DEBUG ) System.out.println("IO_filePort getInputLine");
		if( br == null ) {
			br = new BufferedReader(new InputStreamReader(fin));
		}
		try {
			String s = br.readLine();
			//if( Props.DEBUG ) System.out.println("Got "+s);
			return s;
		} catch (IOException e) {
			return null;
		}
	}

	@Override
	public int bytesToRead() throws IOException {
		return fin.available();
	}

	@Override
	public void writeLine(String output) throws IOException {
		if( !writeable )
			throw new IOException("Attempted write on read-only resource "+port);
		if( bw == null ) {
			bw = new BufferedWriter(new OutputStreamWriter(fout));
		}
		bw.write(output);
	}

	@Override
	public String getPortName() {
		return port;
	}

	@Override
	public String stringSettings() {
		return port;
	}

	@Override
	public String sendCommand(String command) throws IOException {
		if( !writeable )
			throw new IOException("Attempted write on read-only resource "+port);
        fout.write(command.getBytes());
        return null;
	}

}
