package com.neocoretechs.robocore.serialreader;

import com.fazecast.jSerialComm.SerialPort;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.math.BigDecimal;
import java.math.RoundingMode;
import com.neocoretechs.robocore.SynchronizedFixedThreadPoolManager;

/**
 * Uses the serial UART mode of the URM037 serial ultrasonic sensor.
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2021
 *
 */
public class UltrasonicSerialDataPort implements DataPortInterface {
	private static boolean DEBUG = true;
	private static boolean PORTDEBUG = true;
	private static boolean INFO = true;
	private SerialPort serialPort;
	private OutputStream outStream;
	private InputStream inStream;
	// serial settings
	// PortSettings=9600,n,8,1
	// On RasPi its /dev/ttyAMA0 or /dev/ttyS0, on OdroidC2, ttyS0 is hardwired console so we use ttyS1 on header
	// On C1 we have ttyS2 so basically, if we have ttyS2 use that, if we have a ttyS1, use it, otherwise, use ttyS0
	// 
	private static String portName = "/dev/ttyS1";
	private static int baud = 9600;
	private static int datab = 8;
	private static int stopb = 1;
	private static int parityb = 0;
	//
	private static Object readMx = new Object();// mutex
	private static Object writeMx = new Object();
	private static boolean EOT = false;

	private static int[] readBuffer = new int[32]; // leave room for 4 readings
	private static int[] writeBuffer = new int[32];// and 4 writings
	private static int readBufferHead = 0;
	private static int readBufferTail = 0;
	private static int writeBufferHead = 0;
	private static int writeBufferTail = 0;

	private static volatile UltrasonicSerialDataPort instance = null;
	private static Object mutex = new Object();

	static {
		SynchronizedFixedThreadPoolManager.init(2, Integer.MAX_VALUE, new String[] {"URM37"+portName});
	}

	// URM37 request distance every 25 ms max
	public static final byte[] REQUEST_DISTANCE = {(byte)0x22,(byte)0x0,(byte)0x0,(byte)0x22};
	// return is format 0x22, high byte, low byte ,checksum

	/**
	 * Try to determine port, if we cant through cpuinfo, use default
	 * @return An instance of UltrasonicSerialDataPort, singleton for this class
	 */
	public static UltrasonicSerialDataPort getInstance() {
		synchronized(mutex) {
			if( instance == null ) {
				try {
					instance = new UltrasonicSerialDataPort(portName, baud, datab, stopb, parityb);									 
				} catch (IOException e) {
					System.out.println("Could not initialize UltrasonicSerialDataPort of:"+portName+" because "+e);
					e.printStackTrace();
					throw new RuntimeException(e);
				}
			}
			return instance;
		}
	}

	public static UltrasonicSerialDataPort getInstance(String sportName) throws IOException {
		synchronized(mutex) {
			if( instance == null ) {
				try {
					instance = new UltrasonicSerialDataPort(sportName, baud, datab, stopb, parityb);									 
				} catch (IOException e) {
					System.out.println("Could not initialize UltrasonicSerialDataPort of:"+portName+" because "+e);
					e.printStackTrace();
					throw new RuntimeException(e);
				}
			}
			return instance;
		}
	}

	private UltrasonicSerialDataPort(String tportName, int tbaud, int tdatab, int tstopb, int tparityb) throws IOException {
		portName = tportName;
		baud = tbaud;
		datab = tdatab;
		stopb = tstopb;
		parityb = tparityb;
		if( DEBUG || INFO) 
			System.out.println("UltrasonicSerialDataPort "+portName+" baud="+baud+" databits="+datab+" stopbits="+stopb+" parity="+parityb);
		connect(true);
		//clear();
	}

	public String getPortName() { return portName; }
	public int getBaudRate() { return baud; }
	public int getDataBits() { return datab; }
	public int getStopBits() { return stopb; }
	public int getParity() { return parityb; }
	public int getHandshake() { return (serialPort == null ? -1 : serialPort.getFlowControlSettings()); }
	public boolean isEOT() { return EOT; }

	public void connect(boolean writeable) throws IOException {
		serialPort = SerialPort.getCommPort(portName);
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
		inStream = serialPort.getInputStream();
		if( inStream == null ) {
			throw new IOException("Cant get InputStream for port "+portName);
		}

		//(new Thread(new SerialReader(inStream))).start();
		SerialReader readThread = new SerialReader(inStream);
		SynchronizedFixedThreadPoolManager.spin(readThread, "URM37"+portName);
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
			SynchronizedFixedThreadPoolManager.spin(writeThread, "URM37"+portName);
			while(!writeThread.isRunning)
				try {
					Thread.sleep(1);
				} catch (InterruptedException e) {}
		}

		if( PORTDEBUG ) 
			System.out.println("Connected:"+stringSettings());
	}

	@Override
	public void close() {
		serialPort.closePort();
	}


	public int bytesToRead() throws IOException {
		return inStream.available();
	}

	@Override
	public void write(int c) throws IOException {
		synchronized(writeMx) {
			checkWriteBuffer();
			writeBuffer[writeBufferTail++] = (byte)( c & 0xFF);
			writeMx.notify();
		}
	}

	private void checkWriteBuffer() {
		synchronized(writeMx) {
			if( writeBufferTail >= writeBuffer.length) {	
				writeBufferTail = 0;
			}
		}
	}

	private void checkReadBuffer() {
		synchronized(readMx) {
			try {
				if( readBufferHead == readBufferTail )
					readMx.wait();
				if( readBufferHead == readBuffer.length)
					readBufferHead = 0;
			} catch (InterruptedException e) {}
		}
	}

	@Override
	public int read() throws IOException {
		synchronized(readMx) {
			checkReadBuffer();
			return readBuffer[readBufferHead++];
		}
	}
	/**
	 */
	private byte[] signalRead() throws IOException {
		byte[] DISTANCE_RETURN = new byte[4];
		//if( Props.DEBUG ) System.out.println("write "+c);
		// Build and send serial register write command.
		//if( DEBUG )
		//	System.out.println("Setting up read registers from signalRead..");
		for(int i = 0; i < REQUEST_DISTANCE.length; i++)
			write(REQUEST_DISTANCE[i]); // Start byte	
		//if( DEBUG )
		//		System.out.println("Reading response header in signalRead");
		try {
			Thread.sleep(30);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		for(int i = 0; i < DISTANCE_RETURN.length; i++)
			DISTANCE_RETURN[i] = (byte)( read() & 0xFF);
		// check checksum
		if((DISTANCE_RETURN[0]+DISTANCE_RETURN[1]+DISTANCE_RETURN[2]) != DISTANCE_RETURN[3])
			throw new IOException("Bad checksum:"+Integer.toHexString(DISTANCE_RETURN[3]));
		return DISTANCE_RETURN;
	}


	/**
	 * pacman the jizzle in the inputstream
	 */
	public void clear() {
		synchronized(readMx) {
			readBufferHead = readBufferTail = 0;
			try {
				int navail = inStream.available();
				//if( Props.DEBUG )
				//	System.out.println("Clearing "+navail+" from input");
				for(int i = 0; i < navail; i++) inStream.read();
			} catch (IOException e) {
				e.printStackTrace();
			}
			EOT = false;
		}
	}

	/**
	 * Read distsance from sensor
	 * @return The distance in current setting/mode, or 65536 if error
	 * @throws IOException
	 */
	public int readDistance() throws IOException {
		byte[] data = signalRead();
		if(DEBUG) {
			System.out.println();
			System.out.print("Read:");
			for(byte b: data) 
				System.out.print(Integer.toHexString(b)+" ");
			System.out.println();
		}
		int dist = (((int)data[1] & 255)<<8) | ((int)data[2] & 255);
		return dist;
	}


	/**
	 * Sets the serial port parameters
	 * @param parityb 
	 * @param stopb 
	 * @param datab 
	 * @param baud 
	 * @throws UnsupportedCommOperationException 
	 */
	private void setSerialPortParameters(int baud, int datab, int stopb, int parityb) throws IOException {
		//if( Props.DEBUG ) System.out.println("Setting serial port "+baud+" "+datab+" "+stopb+" "+parityb);

		// Set serial port
		// serialPort.setSerialPortParams(57600,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);
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
		serialPort.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);


		//SerialPort.RtsEnable = true;
		//SerialPort.ReadBufferSize = 4096;
		//SerialPort.WriteBufferSize = 512;
		//SerialPort.ReceivedBytesThreshold = 1;
		//SerialPort.ReadTimeout = 5500;
		//SerialPort.WriteTimeout = 5500;
	}


	/**
	 * Data about machine and port settings
	 */
	public String stringSettings() {
		StringBuilder sb = new StringBuilder("URM37SerialDataPort\n");
		sb.append("Port Name = ");
		sb.append(getPortName());
		sb.append("\n");
		sb.append("Port BaudRate = ");
		sb.append(getBaudRate());
		sb.append("\n");
		sb.append("Port Parity = ");
		sb.append(getParity());
		sb.append("\n");
		sb.append("Port DataBits = ");
		sb.append(getDataBits());
		sb.append("\n");
		sb.append("Port StopBits = ");
		sb.append(getStopBits());
		sb.append("\n");
		sb.append("Port ReadTimeout = 5500\n");
		sb.append("Port WriteTimeout = 5500\n");
		sb.append("Port Handshake = ");
		sb.append(getHandshake());
		return sb.toString();
	}

	public static double round(double value, int places) {
		if (places < 0) throw new IllegalArgumentException();
		BigDecimal bd = new BigDecimal(value);
		bd = bd.setScale(places, RoundingMode.HALF_UP);
		return bd.doubleValue();
	}
	/** */
	public static class SerialReader implements Runnable 
	{
		InputStream in;
		public static volatile boolean shouldRun = true;
		public volatile boolean isRunning = false;
		public SerialReader(InputStream in)
		{
			this.in = in;
		}

		public void run ()
		{
			int inChar = -1;
			isRunning = true;
			while (SerialReader.shouldRun)
			{
				try {
					inChar = this.in.read();
					//System.out.println("SR="+inChar+" "+(char)inChar);
					// rxtx returns -1 on timeout of port
					if( inChar == 255 ) {
						EOT = true;
						inChar = -1;
						//if(Props.DEBUG) System.out.println("EOT signaled...");
					} else {
						EOT = false;
					}
				} catch(IOException ioe) {
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
					Thread.sleep(2);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			isRunning = false;
		}
	}

	/** */
	public static class SerialWriter implements Runnable 
	{
		OutputStream out;
		public static boolean shouldRun = true;
		public boolean isRunning = false;
		public SerialWriter( OutputStream out )
		{
			this.out = out;
		}

		public void run ()
		{
			isRunning = true;
			while(SerialWriter.shouldRun)
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
						//System.out.print("["+(char)(writeBuffer[writeBufferHead])+"@"+writeBufferHead+"]");
						this.out.write(writeBuffer[writeBufferHead++]);
						writeMx.notify();
					}
					Thread.sleep(2);
				}
				catch ( IOException ioe ) {
					System.out.println("Write exception on serial write:"+ioe);
				} 
				catch (InterruptedException e) {
				}

			}
			isRunning = false;
		}
	}

	@Override
	public String readLine() {
		throw new RuntimeException("refactor! not applicable method");
	}

	@Override
	public void writeLine(String output) throws IOException {
		throw new RuntimeException("refactor! not applicable method");

	}

}
