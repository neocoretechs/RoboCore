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
 * Class that interfaces with the serial data port on the SBC. It uses GNU io RXTX
 * to effect transmission and reception.
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class ByteSerialDataPort implements DataPortInterface {
		private static boolean DEBUG = false;
		private static boolean PORTDEBUG = false;
	    private SerialPort serialPort;
        CommPortIdentifier portId = null;
	    private OutputStream outStream;
	    private InputStream inStream;
		// serial settings
		//Port=/dev/ttyACM0
		//PortSettings=115200,n,8,1
	    private String portName = "/dev/ttyACM0";
	    //private static String portName = "/dev/ttyAMA0";
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
	    
	    private int portOwned = CommPortOwnershipListener.PORT_UNOWNED; 
	    
	    
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
	    
	    public void connect(boolean writeable) throws IOException {
	    	if(portOwned == CommPortOwnershipListener.PORT_OWNED) {
	    		throw new IOException(Thread.currentThread().getName()+" requesting ownership of port already owned by.."+portId.getCurrentOwner());
	    	}
	    	if(portOwned == CommPortOwnershipListener.PORT_OWNERSHIP_REQUESTED) {
	    		throw new IOException(Thread.currentThread().getName()+" requesting ownership of port whose ownership has already been requested by "+portId.getCurrentOwner());
	    	}
	        try {
	            // Obtain a CommPortIdentifier object for the port you want to open
	        	portId = CommPortIdentifier.getPortIdentifier(portName);
	            if( portId == null ) {
	            	throw new IOException("Cant get CommPortIdentifier for "+portName);
	            }
	        	// Add ownership event listener
	            portId.addPortOwnershipListener(new SerialOwnershipHandler());
	        	if( PORTDEBUG ) 
		    		System.out.println("Trying connect to serial port "+portName);
	            serialPort =
	                    (SerialPort) portId.open("", 5500);
	            if( serialPort == null ) {
	            	throw new IOException("Cant open SerialPort "+portName);
	            }
	            long portTime = System.nanoTime();
	            while(portOwned != CommPortOwnershipListener.PORT_OWNED)
					try {
						Thread.sleep(0,1);
					} catch (InterruptedException e1) {}
	            if(DEBUG)
	            	System.out.println("Took port ownership in "+(System.nanoTime()-portTime)+" .ns");
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
	        if( PORTDEBUG ) 
	        	System.out.println("Connected to "+portName);
	    }
	    
	    public void close() {
	    	if( serialPort != null)
	    		serialPort.close();
	    }
	    
	    public int bytesToRead() throws IOException {
	    	synchronized(readMx) {
	    		return inStream.available();
	    	}
	    }
	 
	    public void write(int c) throws IOException {
	    	if( PORTDEBUG ) 
	    		System.out.print("<"+c+">");
	    	synchronized(writeMx) {
	    		checkWriteBuffer();
	    		writeBuffer[writeBufferTail++] = c;
	    		//checkWriteBuffer();
    			//writeBuffer[writeBufferTail++] = -1;
	    		writeMx.notify();
	    	}
	    }
	    
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
    			writeBuffer[writeBufferTail++] = '\n';	
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
	    	//return inStream.read();
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
				} catch (InterruptedException e) {
				}
	    		// if we waited and nothing came back after timeout, machine no go
	    		if( readBufferHead == readBufferTail ) {
	    			throw new IOException("Serial port not responding..");
	    		}
	    		if( PORTDEBUG ) 
	    			System.out.println(this.getClass().getName()+".read readBufferHead="+readBufferHead+" readBufferTail="+readBufferTail+" = "+readBuffer[readBufferHead]);
	    		return readBuffer[readBufferHead++];
	    	}
	    }
	    	
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
				while( c != '\r' && c != '\n' || sb.length() <= 1 ) {
					c = read();
					if( PORTDEBUG )
						System.out.print("["+Character.toChars(c)[0]+"]");
					if( c != -1 && c != '\r' && c != '\n')
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
				while( c != '\r' ) {
					c = read(timeout);
					if( PORTDEBUG )
						System.out.print("["+Character.toChars(c)[0]+"]");
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
					if( PORTDEBUG )
						System.out.print("["+Character.toChars(c)[0]+"]");
					// got an 'r' but no 'n', must have dropped the 'n'?
					if( c != -1 ) {
							break;
					}
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
							// rxtx returns -1 on timeout of port
							if( inChar == 255 ) {
								EOT = true;
								inChar = -1;
								if(PORTDEBUG) 
									System.out.println("<EOT>");
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
	            	portOwned = type;
	            	/*
	                switch (type) {
	                    case CommPortOwnershipListener.PORT_OWNED:1
	                        //System.out.println("We got the port");
	                    case CommPortOwnershipListener.PORT_UNOWNED:2
	                        //System.out.println("We've just lost our port ownership");
	                    case CommPortOwnershipListener.PORT_OWNERSHIP_REQUESTED:3
	                        //System.out.println("Someone is asking our port's ownership");
	                }
	                */
	            }
	        }
	        
	        public static void main(String[] args) {
	        	Enumeration portEnum = getPortIdentifiers();
	        	// iterate through, looking for the port
	        	while (portEnum.hasMoreElements()) {
	        		CommPortIdentifier currPortId = (CommPortIdentifier)portEnum.nextElement();
	        		System.out.printf("Found port %s type %d (Serial, parallel, I2C, Raw, 485) Owner %s%n", currPortId.getName(),currPortId.getPortType(),
	        				(currPortId.isCurrentlyOwned() ? currPortId.getCurrentOwner() : "NOT owned"));
	        	}
	        }
	        
}
