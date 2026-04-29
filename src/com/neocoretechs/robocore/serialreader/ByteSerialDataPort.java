package com.neocoretechs.robocore.serialreader;

import com.fazecast.jSerialComm.SerialPort;
import com.fazecast.jSerialComm.SerialPortTimeoutException;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Arrays;

import com.neocoretechs.robocore.SynchronizedThreadManager;

/**
 * Class that interfaces with the serial data port on the SBC. It uses linux userspace drivers such as jSerialComm
 * to effect transmission and reception. Two asynchronous threads run; one for transmit, one for receive.<p>
 * After creating the port with proper parameters such as baud rate, parity, data and stop bits, call
 * the connect method to begin communication.<p>
 * @see com.fazecast.jSerialComm.SerialPort
 * @see DataPortCommandInterface
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021,2026
 */
public class ByteSerialDataPort implements DataPortCommandInterface {
	private static boolean DEBUG = false;
	private static boolean PORTDEBUG = false;
	private SerialPort serialPort;
	private OutputStream outStream;
	private InputStream inStream;
	private boolean connected = false;
	private static int READ_DELAY = 2; // ms delay between character
	private static int WRITE_DELAY = 2; // ms delay
	private static int READ_TIMEOUT = 200;
	// serial settings
	//Port=/dev/ttyACM0
	//PortSettings=115200,n,8,1
	private String portName = "/dev/ttyACM0";
	//private static String portName = "/dev/ttyS1";
	private int baud = 115200;
	private int datab = 8;
	private int stopb = 1;
	private int parityb = 0;
	//
	private Object readMx = new Object();// mutex
	private Object writeMx = new Object();
	private boolean EOT = false;

	private int[] readBuffer = new int[32767];
	private int[] writeBuffer = new int[32767];
	private int readBufferHead = 0;
	private int readBufferTail = 0;
	private int writeBufferHead = 0;
	private int writeBufferTail = 0;

	public ByteSerialDataPort()  {
		if( DEBUG ) 
			System.out.println("ByteSerialDataPort "+portName+" baud="+baud+" databits="+datab+" stopbits="+stopb+" parity="+parityb);
	}

	public ByteSerialDataPort(String tportName) {
		portName = tportName;
		if( DEBUG ) 
			System.out.println("ByteSerialDataPort "+portName+" baud="+baud+" databits="+datab+" stopbits="+stopb+" parity="+parityb);
	}

	public ByteSerialDataPort(String tportName, int tbaud, int tdatab, int tstopb, int tparityb) {
		portName = tportName;
		baud = tbaud;
		datab = tdatab;
		stopb = tstopb;
		parityb = tparityb;
		//connect(true);
		//clear();
		if( DEBUG ) 
			System.out.println("ByteSerialDataPort "+portName+" baud="+baud+" databits="+datab+" stopbits="+stopb+" parity="+parityb);
	}
	@Override	    
	public void connect(boolean writeable) throws IOException {
		SynchronizedThreadManager.getInstance().init(new String[] {"SYSTEM"});
		System.out.println("\nUsing Library Version v" + SerialPort.getVersion());
		SerialPort.allowPortOpenForEnumeration();
		SerialPort.autoCleanupAtShutdown();
		SerialPort.addShutdownHook(new Thread() { public void run() { System.out.println("\nRunning shutdown hook"); } });
		serialPort = SerialPort.getCommPort(portName);
		serialPort.allowElevatedPermissionsRequest();
		System.out.println("\nPre-setting RTS: " + (serialPort.setRTS() ? "Success" : "Failure"));
		boolean openedSuccessfully = serialPort.openPort(0);
		System.out.println("\nOpening " + serialPort.getSystemPortName() + ": " + serialPort.getDescriptivePortName() + " - " + serialPort.getPortDescription() + ": " + openedSuccessfully);
		if (!openedSuccessfully)
		{
			System.out.println("Error code was " + serialPort.getLastErrorCode() + " at Line " + serialPort.getLastErrorLocation());
			return;
		}
		// Disable hardware flow control and DTR if Pico resets on open
		serialPort.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);
		serialPort.clearDTR();
		serialPort.clearRTS();
		// Set semi-blocking read with a modest timeout (ms)
		serialPort.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING, READ_TIMEOUT, 0);
		SerialPort.autoCleanupAtShutdown();
		serialPort.setBaudRate(baud);
		serialPort.setNumDataBits(datab);
		switch(stopb) {
			case 1 -> serialPort.setNumStopBits(SerialPort.ONE_STOP_BIT);
			case 2 -> serialPort.setNumStopBits(SerialPort.TWO_STOP_BITS);
		}
		switch(parityb) {
			case 0 -> serialPort.setParity(SerialPort.NO_PARITY);
			case 1 -> serialPort.setParity(SerialPort.ODD_PARITY);
			case 2 -> serialPort.setParity(SerialPort.EVEN_PARITY);
		}
		// Open the input and output streams for the connection. If they won't
		// open, close the port before throwing an exception.
		/*
		inStream = serialPort.getInputStream();
		if( inStream == null ) {
			throw new IOException("Cant get InputStream for port "+portName);
		}   
		SerialReader readThread = new SerialReader(inStream);
		SynchronizedThreadManager.getInstance().spin(readThread, "SYSTEM");
		while(!readThread.isRunning)
			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {}

		if( writeable) {
			outStream = serialPort.getOutputStream();
			if( outStream == null ) {
				throw new IOException("Cant get OutputStream for port "+portName);
			}
			//(new Thread(new SerialWriter(outStream))).start();
			SerialWriter writeThread = new SerialWriter(outStream);
			SynchronizedThreadManager.getInstance().spin(writeThread, "SYSTEM");
			while(!writeThread.isRunning)
				try {
					Thread.sleep(1);
				} catch (InterruptedException e) {}
		}
		*/
		connected = true;
		if( PORTDEBUG ) 
			System.out.println("Connected to "+portName);
	}

	@Override
	public boolean isConnected() {
		return connected;
	}
	
	@Override
	public void close() {
		serialPort.closePort();
	}

	@Override	    
	public int bytesToRead() throws IOException {
		synchronized(readMx) {
			return inStream.available();
		}
	}
	@Override
	public void write(int c) throws IOException {
		if( PORTDEBUG ) 
			System.out.print("<"+c+">");
    	synchronized(writeMx) {
    		checkWriteBuffer();
    		writeBuffer[writeBufferTail++] = (byte)( c & 0xFF);
       		writeMx.notify();
    	}
	}

	@Override
	public void writeLine(String bytesToWrite) throws IOException {
		if( PORTDEBUG ) 
			System.out.println(this.getClass().getName()+".writeLine:"+bytesToWrite);
		synchronized(writeMx) {
			byte[] bytes = bytesToWrite.getBytes();
			for(int i = 0 ; i < bytes.length; i++) {
				checkWriteBuffer();
				writeBuffer[writeBufferTail++] = bytes[i];
				writeMx.notify();
			}
			checkWriteBuffer();
			writeBuffer[writeBufferTail++] = '\r';
			//writeBuffer[writeBufferTail++] = '\n';	
			//writeBuffer[writeBufferTail++] = -1;
			//checkWriteBuffer();
			writeMx.notify();
		}
		//try {
		//	Thread.sleep(15);
		//} catch (InterruptedException e) {}

	}

	private void checkWriteBuffer() {
		synchronized(writeMx) {
			if( writeBufferTail >= writeBuffer.length) {	
				writeBufferTail = 0;
			}
		}
	}
	@Override
	public int read() throws IOException {
		if( PORTDEBUG ) 
			System.out.println(this.getClass().getName()+".read");
		synchronized(readMx) {
			try {
				if( readBufferHead == readBufferTail )
					readMx.wait();
				if( readBufferHead == readBuffer.length)
					readBufferHead = 0;
			} catch (InterruptedException e) {
			}
			if( PORTDEBUG ) 
				System.out.println("readBufferHead="+readBufferHead+" readBufferTail="+readBufferTail+" = "+readBuffer[readBufferHead]);
			return readBuffer[readBufferHead++];
		}
	}
	/**
	 * Read with thrown MachineNotReadyException on timeout
	 * @param timeout
	 * @return
	 * @throws IOException
	 */
	private int read(long timeout) throws IOException {
		if( PORTDEBUG ) 
			System.out.println(this.getClass().getName()+".read");
		synchronized(readMx) {
			try {
				if( readBufferHead == readBufferTail )
					readMx.wait(timeout);
				if( readBufferHead == readBuffer.length)
					readBufferHead = 0;
			} catch (InterruptedException e) {}
			if( PORTDEBUG ) 
				System.out.println(this.getClass().getName()+".read readBufferHead="+readBufferHead+" readBufferTail="+readBufferTail+" = "+readBuffer[readBufferHead]);
			return readBuffer[readBufferHead++];
		}
	}
	@Override	
	/**
	 * Mutex wait on inputLine
	 * @return
	 */
	public String readLine() {
		if( PORTDEBUG ) 
			System.out.println(this.getClass().getName()+".readLine");
		int c = -1;
		StringBuffer sb = new StringBuffer();
		try {
			while( c != '\r' && c != '\n' && c != 0 || sb.length() <= 1 ) {
				c = read();
				if( PORTDEBUG )
					System.out.print("["+Character.toChars(c)[0]+"]");
				if( c != -1 && c != '\r' && c != '\n' && c != 0)
					sb.append(Character.toString((char)c));
			}
		} catch (IOException e) {
			System.out.println("IOException reading line:"+sb.toString());
			return null;
		}
		if( PORTDEBUG )
			System.out.println(this.getClass().getName()+".readLine, exiting with:"+sb.toString());
		return sb.toString();
	}

	/**
	 * Mutex wait on inputLine
	 * @return
	 * @throws IOException 
	 * @Exception MachineNotReadyException on timeout
	 */
	public String readLine(long timeout) throws IOException {
		if( PORTDEBUG ) 
			System.out.println(this.getClass().getName()+".readLine("+timeout+")");
		int c = -1;
		StringBuffer sb = new StringBuffer();
		try {
			while( c != '\r' && c != '\n' && c != 0 || sb.length() <= 1 ) {
				c = read(timeout);
				if( PORTDEBUG )
					System.out.print("["+Character.toChars(c)[0]+"]");
				// not an 'r' but we got an 'n' must have dropped the 'r'?
				if( c != -1 && c != '\r' && c != '\n' && c != 0)
					sb.append(Character.toString((char)c));
			}
		} catch (IOException e) {
			System.out.println("IOException "+e+" reading line:"+sb.toString());
			return null;
		}
		if( PORTDEBUG )
			System.out.println(this.getClass().getName()+".readLine("+timeout+"), exiting with:"+sb.toString());
		return sb.toString();
	}
	/**
	 * pacman the jizzle in the inputstream
	 */
	public void clear() {
		synchronized(readMx) {
			readBufferHead = readBufferTail = 0;
			try {
				int navail = inStream.available();
				if( PORTDEBUG )
					System.out.println(this.getClass().getName()+".clear Clearing "+navail+" from input");
				for(int i = 0; i < navail; i++) inStream.read();
			} catch (IOException e) {
				e.printStackTrace();
			}
			EOT = false;
		}
	}

	@Override
	public ArrayList<String> sendCommand(String command) throws IOException {
		byte[] buffer = new byte[1024];
		ArrayList<String> ret = new ArrayList<String>();
		command += "\r";
		byte[] bcomm = command.getBytes();
		int blen = bcomm.length;
		int wlen = 0;
		int tlen = 0;
		while(tlen < blen) {
			wlen = serialPort.writeBytes(bcomm, blen, tlen);
			if(wlen > 0 && tlen < blen) {
				tlen += wlen;
				blen -= wlen;
				try {
					Thread.sleep(1);
				} catch (InterruptedException e) {
					break;
				}
			}
		}
		int num = 0;
		while((num = serialPort.readBytes(buffer, buffer.length)) <= 0) {
			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {
				break;
			}
		}
		ret.add(new String(Arrays.copyOf(buffer, num)));
		return ret;
	}
	/*
	public ArrayList<String> sendCommand(String command) throws IOException {
		writeLine(command);
		ArrayList<String> ret = new ArrayList<String>();
		while(bytesToRead() <= 0) {
			try {
				Thread.sleep(0,50000);
			} catch (InterruptedException e) {}
		}
		while(bytesToRead() > 0) {
			if(READ_TIMEOUT != -1)
				ret.add(readLine(READ_TIMEOUT));
			else
				ret.add(readLine());
		}
		return ret;
	}
	*/
	public String getPortName() { return portName; }
	public int getBaudRate() { return baud; }
	public int getDataBits() { return datab; }
	public int getStopBits() { return stopb; }
	public int getParity() { return parityb; }
	public synchronized int getHandshake() { return (serialPort == null ? -1 : serialPort.getFlowControlSettings()); }
	public boolean isEOT() { return EOT; }

	/**
	 * Data about machine and port settings
	 */
	public String stringSettings() {
		String msg = "ByteSerialDataPort\n";
		msg = msg + "Port Name = " + getPortName() + "\n";
		msg = msg + "Port BaudRate = " + getBaudRate() + "\n";
		msg = msg + "Port Parity = " + getParity() + "\n";
		msg = msg + "Port DataBits = " + getDataBits() + "\n";
		msg = msg + "Port StopBits = " + getStopBits() + "\n";
		msg = msg + "Port ReadTimeout = 5500\n";
		msg = msg + "Port WriteTimeout = 5500\n";
		msg = msg + "Port Handshake = " + getHandshake();
		return msg;
	}

	/** */
	public class SerialReader implements Runnable 
	{
		InputStream in;
		public volatile boolean shouldRun = true;
		public volatile boolean isRunning = false;
		public SerialReader(InputStream in)
		{
			this.in = in;
		}

		public void run ()
		{
			int inChar = -1;
			isRunning = true;
			while (shouldRun)
			{
				try {
					inChar = this.in.read();
					if(PORTDEBUG)
						System.out.print((char)inChar);
					if( inChar == 255 ) {
						EOT = true;
						inChar = -1;
						if(PORTDEBUG) 
							System.out.println("<EOT>");
					} else {
						EOT = false;
					}
				} catch(IOException ioe) {
					if(!(ioe instanceof SerialPortTimeoutException))
						System.out.println(ioe);
					continue;
				}

				//System.out.print(inChar+"="+Character.toString((char)inChar)+" ");
				//if( Props.DEBUG ) System.out.println("\n-----");
				synchronized(readMx) {
					if( readBufferTail == readBuffer.length)
						readBufferTail = 0;
					readBuffer[readBufferTail++] = inChar;
					if( readBufferTail == readBufferHead )
						System.out.println("Possible buffer overrun "+readBufferHead+" "+readBufferTail);
					readMx.notify();
				}
				try {
					Thread.sleep(READ_DELAY);
				} catch (InterruptedException e) {}
			}
			isRunning = false;
		}
	}

	public class SerialWriter implements Runnable 
	{
		OutputStream out;
		public volatile boolean shouldRun = true;
		public volatile boolean isRunning = false;
		public SerialWriter( OutputStream out )
		{
			this.out = out;
		}

		public void run ()
		{
			isRunning = true;
			while(shouldRun)
			{
				try
				{                
					synchronized(writeMx) {   
						if( writeBufferHead == writeBufferTail ) {
							//System.out.println("Enter wait writer:"+writeBufferHead+" "+writeBufferTail);
							writeMx.wait();
							//System.out.println("Leave wait writer:"+writeBufferHead+" "+writeBufferTail);
						}
						if( writeBufferHead == writeBuffer.length)
							writeBufferHead = 0;
						if(PORTDEBUG)
							System.out.print("<"+(char)(writeBuffer[writeBufferHead])+"@"+writeBufferHead+">");
						this.out.write(writeBuffer[writeBufferHead++]);
						writeMx.notify();
					}
	           		Thread.sleep(WRITE_DELAY);
				}
				catch ( IOException ioe ) {
					System.out.println("Write exception on serial write:"+ioe);
				} 
				catch (InterruptedException e) {}
			}
			isRunning = false;
		}
	}
}
