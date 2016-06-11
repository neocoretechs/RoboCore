package com.neocoretechs.robocore.serialreader;

import gnu.io.CommPortIdentifier;
import gnu.io.NoSuchPortException;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;
import gnu.io.UnsupportedCommOperationException;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Enumeration;

import com.neocoretechs.robocore.ThreadPoolManager;

public class ByteSerialDataPort implements DataPortInterface {
		private static boolean DEBUG = true;
	    private SerialPort serialPort;
	    private OutputStream outStream;
	    private InputStream inStream;
		// serial settings
		//Port=/dev/ttyACM0
		//PortSettings=115200,n,8,1
	    private static String portName = "/dev/ttyACM0";
	    private static int baud = 115200;
	    private static int datab = 8;
	    private static int stopb = 1;
	    private static int parityb = 0;
	    //
	    private static Object readMx = new Object();// mutex
	    private static Object writeMx = new Object();
	    private static boolean EOT = false;
	 
	    private static int[] readBuffer = new int[32767];
	    private static int[] writeBuffer = new int[32767];
	    private static int readBufferHead = 0;
	    private static int readBufferTail = 0;
	    private static int writeBufferHead = 0;
	    private static int writeBufferTail = 0;
	    private static ByteSerialDataPort instance = null;
	    private static Object mutex = new Object();
	    public static ByteSerialDataPort getInstance() {
	    	synchronized(mutex) {
	    	if( instance == null ) {
	    		try {
					instance = new ByteSerialDataPort(portName, baud, datab, stopb, parityb);
													 
				} catch (IOException e) {
					System.out.println("Could not initialize ByteSerialDataPort:"+e);
					e.printStackTrace();
					throw new RuntimeException(e);
				}
	    	}
	    	return instance;
	    	}
	    }
	    
	    private ByteSerialDataPort(String tportName, int tbaud, int tdatab, int tstopb, int tparityb) throws IOException {
	    	portName = tportName;
	    	baud = tbaud;
	    	datab = tdatab;
	    	stopb = tstopb;
	    	parityb = tparityb;
	    	connect(true);
	    	if( DEBUG ) 
	    		System.out.println("ByteSerialDataPort "+portName+" baud="+baud+" databits="+datab+" stopbits="+stopb+" parity="+parityb);
	    }
	    
	    public void connect(boolean writeable) throws IOException {
	    	
	    	//if( Props.DEBUG ) System.out.println("Trying connect to serial port "+portName);
	        try {
	            // Obtain a CommPortIdentifier object for the port you want to open
	            CommPortIdentifier portId =
	                    CommPortIdentifier.getPortIdentifier(portName);
	            if( portId == null ) {
	            	throw new IOException("Cant get CommPortIdentifier for "+portName);
	            }
	            //if ( portId.isCurrentlyOwned() )
	            //{
	            //    if( Props.DEBUG ) System.out.println("Error: Port is currently in use");
	            //}   
	            serialPort =
	                    (SerialPort) portId.open("", 5500);
	            if( serialPort == null ) {
	            	throw new IOException("Cant open SerialPort "+portName);
	            }
	            //if (! (serialPort instanceof SerialPort) )
	            //{
	            //	err
	            //}
	            // Set the parameters of the connection.
	            setSerialPortParameters(baud, datab, stopb, parityb);
	 
	            // Open the input and output streams for the connection. If they won't
	            // open, close the port before throwing an exception.
	            inStream = serialPort.getInputStream();
	            if( inStream == null ) {
	            	throw new IOException("Cant get InputStream for port "+portName);
	            }   
	            //(new Thread(new SerialReader(inStream))).start();
	            ThreadPoolManager.getInstance().spin((new SerialReader(inStream)), "SYSTEM");
	            
	            if( writeable) {
	                outStream = serialPort.getOutputStream();
		            if( outStream == null ) {
		            	throw new IOException("Cant get OutputStream for port "+portName);
		            }
		            //(new Thread(new SerialWriter(outStream))).start();
		            ThreadPoolManager.getInstance().spin((new SerialWriter(outStream)), "SYSTEM");
		            
	            }          
	        } catch (NoSuchPortException | PortInUseException | UnsupportedCommOperationException e) {
	        	if( serialPort != null ) {
	        		serialPort.close();
	        	}
	            throw new IOException(e);
	        }
	        //if( Props.DEBUG ) System.out.println("Connected to "+portName);
	    }
	    
	    public void close() {
	    	if( serialPort != null)
	    		serialPort.close();
	    }
	    
	    public int bytesToRead() throws IOException {
	    	return inStream.available();
	    }
	 
	    public void write(int c) throws IOException {
	    	//if( Props.DEBUG ) System.out.println("write "+c);
	    	synchronized(writeMx) {
	    		checkWriteBuffer();
	    		writeBuffer[writeBufferTail++] = c;
	    		checkWriteBuffer();
    			writeBuffer[writeBufferTail++] = -1;
	    		writeMx.notify();
	    	}
	    }
	    
	    public void writeLine(String bytesToWrite) throws IOException {
	    	//if( Props.DEBUG ) System.out.println("writeLine "+bytesToWrite);
	    	synchronized(writeMx) {
	    		byte[] bytes = bytesToWrite.getBytes();
	    		for(int i = 0 ; i < bytes.length; i++) {
	    			checkWriteBuffer();
	    			writeBuffer[writeBufferTail++] = bytes[i];
	    		}
    			writeBuffer[writeBufferTail++] = 13;
    			checkWriteBuffer();
    			writeBuffer[writeBufferTail++] = -1;
    			checkWriteBuffer();
	    		writeMx.notify();
	    	}
	    }
	    
	    private void checkWriteBuffer() {
	    	if( writeBufferTail >= writeBuffer.length) {	
    				writeBufferTail = 0;
			}
	    }
	    
	    public int read() throws IOException {
	    	//if( Props.DEBUG ) System.out.println("read");
	    	synchronized(readMx) {
	    		try {
	    			if( readBufferHead == readBuffer.length)
	    				readBufferHead = 0;
	    			if( readBufferHead == readBufferTail )
	    				readMx.wait();
				} catch (InterruptedException e) {
				}
	    		//if( Props.DEBUG ) System.out.println("readBufferHead="+readBufferHead+" readBufferTail="+readBufferTail+" = "+readBuffer[readBufferHead]);
	    		return readBuffer[readBufferHead++];
	    	}
	    	//return inStream.read();
	    }
	    /**
	     * Read with thrown MachineNotReadyException on timeout
	     * @param timeout
	     * @return
	     * @throws IOException
	     */
	    private int read(long timeout) throws IOException {
	    	//if( Props.DEBUG ) System.out.println("read");
	    	synchronized(readMx) {
	    		try {
	    			if( readBufferHead == readBuffer.length)
	    				readBufferHead = 0;
	    			if( readBufferHead == readBufferTail )
	    				readMx.wait(timeout);
				} catch (InterruptedException e) {
				}
	    		// if we waited and nothing came back after timeout, machine no go
	    		if( readBufferHead == readBufferTail ) {
	    			throw new IOException("Serial port not responding..");
	    		}
	    		//if( Props.DEBUG ) System.out.println("readBufferHead="+readBufferHead+" readBufferTail="+readBufferTail+" = "+readBuffer[readBufferHead]);
	    		return readBuffer[readBufferHead++];
	    	}
	    }
	    /**
	     * Mutex wait on inputLine
	     * @return
	     */
	    public String readLine() {
	    	int c = -1;
	    	StringBuffer sb = new StringBuffer();
	    	try {
				while( (c = read()) != -1 && c != 10 && c != 13) {
					sb.append((char)c);
				}
			} catch (IOException e) {
			}
	    	if( c == -1 )
	    		return null;
	    	return sb.toString();
	    }
	    
	    /**
	     * Mutex wait on inputLine
	     * @return
	     * @throws IOException 
	     * @Exception MachineNotReadyException on timeout
	     */
	    public String readLine(long timeout) throws IOException {
	    	int c = -1;
	    	StringBuffer sb = new StringBuffer();
			while( (c = read(timeout)) != -1 && c != 10 && c != 13) {
					sb.append((char)c);
			}
	    	if( c == -1 )
	    		return null;
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
					//if( Props.DEBUG )
					//	System.out.println("Clearing "+navail+" from input");
					for(int i = 0; i < navail; i++) inStream.read();
				} catch (IOException e) {
					e.printStackTrace();
				}
	    		EOT = false;
	    	}
	    }
	    
	    public String getPortName() { return portName; }
	    public int getBaudRate() { return baud; }
	    public int getDataBits() { return datab; }
	    public int getStopBits() { return stopb; }
	    public int getParity() { return parityb; }
	    public int getHandshake() { return (serialPort == null ? -1 : serialPort.getFlowControlMode()); }
	    public boolean isEOT() { return EOT; }
	    
	    /**
	     * Sets the serial port parameters
	     * @param parityb 
	     * @param stopb 
	     * @param datab 
	     * @param baud 
	     * @throws UnsupportedCommOperationException 
	     */
	    private void setSerialPortParameters(int baud, int datab, int stopb, int parityb) throws IOException, UnsupportedCommOperationException {
	    	//if( Props.DEBUG ) System.out.println("Setting serial port "+baud+" "+datab+" "+stopb+" "+parityb);

	        // Set serial port
	    	// serialPort.setSerialPortParams(57600,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);
	    	
	        serialPort.setSerialPortParams(baud, datab, stopb, parityb);
	 
	        serialPort.setFlowControlMode(SerialPort.FLOWCONTROL_NONE);
	            
	        //serialPort.setFlowControlMode( 
	        //    		  SerialPort.FLOWCONTROL_RTSCTS_IN | 
	        //    		  SerialPort.FLOWCONTROL_RTSCTS_OUT);
	            
	        //serialPort.setFlowControlMode(SerialPort.FLOWCONTROL_XONXOFF_IN |  SerialPort.FLOWCONTROL_XONXOFF_OUT);
	        serialPort.setDTR(false);
	        serialPort.setRTS(false);
	        //serialPort.enableReceiveThreshold(1);
	        serialPort.disableReceiveTimeout();
	        //SerialPort.RtsEnable = true;
	        //SerialPort.ReadBufferSize = 4096;
	        //SerialPort.WriteBufferSize = 512;
	        //SerialPort.ReceivedBytesThreshold = 1;
	        //SerialPort.ReadTimeout = 5500;
	        //SerialPort.WriteTimeout = 5500;
	    }
	    
	    public static Enumeration getPortIdentifiers() {
	    	return CommPortIdentifier.getPortIdentifiers();
	    }
	    /**
	     * Data about machine and port settings
	     */
	    public String stringSettings()
	    {
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
	        public static class SerialReader implements Runnable 
	        {
	            InputStream in;
	            public static boolean shouldRun = true;
	            public SerialReader(InputStream in)
	            {
	                this.in = in;
	            }
	            
	            public void run ()
	            {
	                int inChar = -1;
	                while (SerialReader.shouldRun)
					{
						try {
							inChar = this.in.read();
							//System.out.println("SerialReader="+inChar+" "+(char)inChar);
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
					}            
	            }
	        }

	        /** */
	        public static class SerialWriter implements Runnable 
	        {
	            OutputStream out;
	            public static boolean shouldRun = true;
	            public SerialWriter( OutputStream out )
	            {
	                this.out = out;
	            }
	            
	            public void run ()
	            {
	                while(SerialWriter.shouldRun)
					{
	                	try
	                	{                
	                		synchronized(writeMx) {
	                			if( writeBufferHead == writeBuffer.length)
	                				writeBufferHead = 0;
	                			if( writeBufferHead == writeBufferTail ) {
	                				System.out.println("Enter wait writer:"+writeBufferHead+" "+writeBufferTail);
	                    			writeMx.wait();
	                    			System.out.println("Leave wait writer:"+writeBufferHead+" "+writeBufferTail);
	                			}
	                			//System.out.println("SerialWriter="+(char)(writeBuffer[writeBufferHead]));
	                			this.out.write(writeBuffer[writeBufferHead++]);
	                			writeMx.notify();
	                		}
	                	}
	                	catch ( IOException ioe ) {
							System.out.println("Write exception on serial write:"+ioe);
	                	} 
	                	catch (InterruptedException e) {
	                	}

					}
	            }
	        }
	        
}
