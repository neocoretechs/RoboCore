package com.neocoretechs.robocore.serialreader;

import gnu.io.CommPortIdentifier;
import gnu.io.CommPortOwnershipListener;
import gnu.io.NoSuchPortException;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;
import gnu.io.UnsupportedCommOperationException;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Enumeration;

import com.neocoretechs.robocore.ThreadPoolManager;
/**
 * Singleton class that interfaces with the serial data port on the SBC. It uses GNU io RXTX
 * to effect transmission and reception.
 * @author groff
 *
 */
public class ByteSerialDataPort implements DataPortInterface {
		private static boolean DEBUG = true;
	    private SerialPort serialPort;
	    private OutputStream outStream;
	    private InputStream inStream;
		// serial settings
		//Port=/dev/ttyACM0
		//PortSettings=115200,n,8,1
	    private static String portName = "/dev/ttyACM0";
	    //private static String portName = "/dev/ttyAMA0";
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
	    
	    private static volatile ByteSerialDataPort instance = null;
	    private boolean portOwned = false;
	    
	    public static ByteSerialDataPort getInstance() {
	    	if( instance == null ) {
	    		synchronized(ByteSerialDataPort.class) { 
	    			try {
	    				instance = new ByteSerialDataPort(portName, baud, datab, stopb, parityb);											 
	    			} catch (IOException e) {
	    				System.out.println("Could not initialize ByteSerialDataPort:"+e);
	    				e.printStackTrace();
	    				throw new RuntimeException(e);
	    			}
	    		}
	    	}
	    	return instance;
	    }
	    
	    private ByteSerialDataPort(String tportName, int tbaud, int tdatab, int tstopb, int tparityb) throws IOException {
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
	    
	    public synchronized void connect(boolean writeable) throws IOException {
	    	portOwned = false;
	    	//if( Props.DEBUG ) System.out.println("Trying connect to serial port "+portName);
	        try {
	            // Obtain a CommPortIdentifier object for the port you want to open
	            CommPortIdentifier portId =
	                    CommPortIdentifier.getPortIdentifier(portName);
	            if( portId == null ) {
	            	throw new IOException("Cant get CommPortIdentifier for "+portName);
	            }
	        	// Add ownership event listener
	            portId.addPortOwnershipListener(new SerialOwnershipHandler());

	            serialPort =
	                    (SerialPort) portId.open("", 5500);
	            if( serialPort == null ) {
	            	throw new IOException("Cant open SerialPort "+portName);
	            }
	            
	            while(!portOwned)
					try {
						Thread.sleep(1);
					} catch (InterruptedException e1) {}
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
	            SerialReader readThread = new SerialReader(inStream);
	            ThreadPoolManager.getInstance().spin(readThread, "SYSTEM");
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
		            ThreadPoolManager.getInstance().spin(writeThread, "SYSTEM");
		            while(!writeThread.isRunning)
						try {
							Thread.sleep(1);
						} catch (InterruptedException e) {}
	            }
	            
	        } catch (NoSuchPortException | PortInUseException | UnsupportedCommOperationException e) {
	        	if( serialPort != null ) {
	        		serialPort.close();
	        	}
	            throw new IOException(e);
	        }
	        //if( Props.DEBUG ) System.out.println("Connected to "+portName);
	    }
	    
	    public synchronized void close() {
	    	if( serialPort != null)
	    		serialPort.close();
	    }
	    
	    public synchronized int bytesToRead() throws IOException {
	    	return inStream.available();
	    }
	 
	    public synchronized void write(int c) throws IOException {
	    	//if( Props.DEBUG ) System.out.println("write "+c);
	    	synchronized(writeMx) {
	    		checkWriteBuffer();
	    		writeBuffer[writeBufferTail++] = c;
	    		//checkWriteBuffer();
    			//writeBuffer[writeBufferTail++] = -1;
	    		writeMx.notify();
	    	}
	    }
	    
	    public synchronized void writeLine(String bytesToWrite) throws IOException {
	    	//if( Props.DEBUG ) System.out.println("writeLine "+bytesToWrite);
	    	synchronized(writeMx) {
	    		byte[] bytes = bytesToWrite.getBytes();
	    		for(int i = 0 ; i < bytes.length; i++) {
	    			checkWriteBuffer();
	    			writeBuffer[writeBufferTail++] = bytes[i];
	    			writeMx.notify();
	    		}
	    		checkWriteBuffer();
    			writeBuffer[writeBufferTail++] = 13;	
    			//writeBuffer[writeBufferTail++] = -1;
    			//checkWriteBuffer();
	    		writeMx.notify();
	    	}
	    	//try {
			//	Thread.sleep(15);
			//} catch (InterruptedException e) {}

	    }
	    
	    private synchronized void checkWriteBuffer() {
	    	if( writeBufferTail >= writeBuffer.length) {	
    				writeBufferTail = 0;
			}
	    }
	    
	    public synchronized int read() throws IOException {
	    	//if( Props.DEBUG ) System.out.println("read");
	    	synchronized(readMx) {
	    		try {
	    			if( readBufferHead == readBufferTail )
	    				readMx.wait();
	    			if( readBufferHead == readBuffer.length)
	    				readBufferHead = 0;
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
	    private synchronized int read(long timeout) throws IOException {
	    	//if( Props.DEBUG ) System.out.println("read");
	    	synchronized(readMx) {
	    		try {
	    			if( readBufferHead == readBufferTail )
	    				readMx.wait(timeout);
	      			if( readBufferHead == readBuffer.length)
	    				readBufferHead = 0;
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
	    public synchronized String readLine() {
	    	int c = -1;
	    	StringBuffer sb = new StringBuffer();
	    	try {
				while( c != '\r' && c != '\n' || sb.length() <= 1 ) {
					c = read();
					//if( DEBUG )
					//	System.out.print("["+Character.toChars(c)[0]+"]");
					if( c != -1 && c != '\r' && c != '\n')
						sb.append(Character.toString((char)c));
				}
			} catch (IOException e) {
				System.out.println("IOException reading line:"+sb.toString());
				return null;
			}
	    	//if( DEBUG )
	    	//	System.out.println("readLine:"+sb.toString());
	    	return sb.toString();
	    }
	    
	    /**
	     * Mutex wait on inputLine
	     * @return
	     * @throws IOException 
	     * @Exception MachineNotReadyException on timeout
	     */
	    public synchronized String readLine(long timeout) throws IOException {
	    	int c = -1;
	    	StringBuffer sb = new StringBuffer();
	    	try {
				while( c != '\r' ) {
					c = read(timeout);
					//if( DEBUG )
					//	System.out.print("["+Character.toChars(c)[0]+"]");
					if( c != -1 ) {
						// not an 'r' but we got an 'n' must have dropped the 'r'?
						if( c == '\n')
							return sb.toString();
						sb.append((char)c);
					}
				}
				// got an 'r', must get an 'n' anything else is bad 
				while( c != '\n' ) {
					c = read(timeout);
					//if( DEBUG )
					//	System.out.print("["+Character.toChars(c)[0]+"]");
					// got an 'r' but no 'n', must have dropped the 'n'?
					if( c != -1 ) {
							break;
					}
				}
			} catch (IOException e) {
				System.out.println("IOException "+e+" reading line:"+sb.toString());
				return null;
			}
	    	return sb.toString();
	    }
	    /**
	     * pacman the jizzle in the inputstream
	     */
	    public synchronized void clear() {
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
	    public synchronized int getHandshake() { return (serialPort == null ? -1 : serialPort.getFlowControlMode()); }
	    public boolean isEOT() { return EOT; }
	    
	    /**
	     * Sets the serial port parameters
	     * @param parityb 
	     * @param stopb 
	     * @param datab 
	     * @param baud 
	     * @throws UnsupportedCommOperationException 
	     */
	    private synchronized void setSerialPortParameters(int baud, int datab, int stopb, int parityb) throws IOException, UnsupportedCommOperationException {
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
	    
	    public synchronized static Enumeration getPortIdentifiers() {
	    	return CommPortIdentifier.getPortIdentifiers();
	    }
	    /**
	     * Data about machine and port settings
	     */
	    public synchronized String stringSettings() {
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
					}
	                isRunning = false;
	            }
	        }

	        /** */
	        public static class SerialWriter implements Runnable 
	        {
	            OutputStream out;
	            public static volatile boolean shouldRun = true;
	            public volatile boolean isRunning = false;
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
	        
	        class SerialOwnershipHandler implements CommPortOwnershipListener
	        {
	            public void ownershipChange(int type) {
	                switch (type) {
	                    case CommPortOwnershipListener.PORT_OWNED:
	                        //System.out.println("We got the port");
	                    	portOwned = true;
	                        break;
	                    case CommPortOwnershipListener.PORT_UNOWNED:
	                        //System.out.println("We've just lost our port ownership");
	                    	portOwned = false;
	                        break;
	                    case CommPortOwnershipListener.PORT_OWNERSHIP_REQUESTED:
	                        //System.out.println("Someone is asking our port's ownership");
	                        break;
	                }
	            }
	        }
	        
}
