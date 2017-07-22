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

public class IMUSerialDataPort implements DataPortInterface {
		private static boolean DEBUG = true;
	    private SerialPort serialPort;
	    private OutputStream outStream;
	    private InputStream inStream;
		// serial settings
		//Port=/dev/ttyACM0
		//PortSettings=115200,n,8,1
	    private static String portName = "/dev/ttyS0";
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
	    
	    private static IMUSerialDataPort instance = null;
	    private static Object mutex = new Object();
	    private boolean portOwned = false;
	    
	    //BNO055
	    //Page id register definition
	    public static final byte BNO055_ID              = (byte)0xA0;
	    public static final byte BNO055_PAGE_ID_ADDR    = (byte)0x07;
	    // PAGE0 REGISTER DEFINITION START
	    public static final byte BNO055_CHIP_ID_ADDR    = (byte)0x00;
	    public static final byte BNO055_ACCEL_REV_ID_ADDR  = (byte)0x01;
	    public static final byte BNO055_MAG_REV_ID_ADDR    = (byte)0x02;
	    public static final byte BNO055_GYRO_REV_ID_ADDR   = (byte)0x03;
	    public static final byte BNO055_SW_REV_ID_LSB_ADDR = (byte)0x04;
	    public static final byte BNO055_SW_REV_ID_MSB_ADDR = (byte)0x05;
	    public static final byte BNO055_BL_REV_ID_ADDR     = (byte)0X06;
	    // BNO055 modes
	    public static final byte BNO055_OPR_MODE_ADDR = (byte)0x3D;
	    public static final byte BNO055_PWR_MODE_ADDR = (byte)0x3E;
	    public static final byte BNO055_SYS_TRIGGER_ADDR = (byte)0x3F;
	    public static final byte BNO055_TEMP_SOURCE_ADDR = (byte)0x40;
	    
	    public static final byte POWER_MODE_NORMAL = (byte)0x00;
	    
	    // Operation mode settings
	    public static final byte OPERATION_MODE_CONFIG                = (byte)0x00;
	    public static final byte OPERATION_MODE_ACCONLY               = (byte)0x01;
	    public static final byte OPERATION_MODE_MAGONLY               = (byte)0x02;
	    public static final byte OPERATION_MODE_GYRONLY               = (byte)0x03;
	    public static final byte OPERATION_MODE_ACCMAG                = (byte)0x04;
	 	public static final byte OPERATION_MODE_ACCGYRO               = (byte)0x05;
	    public static final byte OPERATION_MODE_MAGGYRO               = (byte)0x06;
	    public static final byte OPERATION_MODE_AMG                   = (byte)0x07;
	    public static final byte OPERATION_MODE_IMUPLUS               = (byte)0x08;
	    public static final byte OPERATION_MODE_COMPASS               = (byte)0x09;
	    public static final byte OPERATION_MODE_M4G                   = (byte)0x0A;
	    public static final byte OPERATION_MODE_NDOF_FMC_OFF          = (byte)0x0B;
	    public static final byte OPERATION_MODE_NDOF                  = (byte)0x0C;

	    
	    public static IMUSerialDataPort getInstance() {
	    	synchronized(mutex) {
	    	if( instance == null ) {
	    		try {
					instance = new IMUSerialDataPort(portName, baud, datab, stopb, parityb);
													 
				} catch (IOException e) {
					System.out.println("Could not initialize IMUSerialDataPort:"+e);
					e.printStackTrace();
					throw new RuntimeException(e);
				}
	    	}
	    	return instance;
	    	}
	    }
	    
	    private IMUSerialDataPort(String tportName, int tbaud, int tdatab, int tstopb, int tparityb) throws IOException {
	    	portName = tportName;
	    	baud = tbaud;
	    	datab = tdatab;
	    	stopb = tstopb;
	    	parityb = tparityb;
	    	connect(true);
	    	//clear();
	    	if( DEBUG ) 
	    		System.out.println("IMUSerialDataPort "+portName+" baud="+baud+" databits="+datab+" stopbits="+stopb+" parity="+parityb);
	    }
	    
	    public void connect(boolean writeable) throws IOException {
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
	        //
	        // set up BNO055
	    	// throw away command
	        //write(BNO055_PAGE_ID_ADDR, new byte[]{(byte)0x00}, false);
	        // set config mode
	    	set_mode(OPERATION_MODE_CONFIG);
	        // now read without kruft
	        write(BNO055_PAGE_ID_ADDR, new byte[]{(byte)0x00}, true);
	        if( DEBUG )
	        	System.out.println("Read chip id..");
	        // Check the chip ID
	        byte[] bno_id = read(BNO055_CHIP_ID_ADDR,(byte)0x01);
	        if( DEBUG) {
	        	System.out.printf("Read chip ID: %02x\r\n",bno_id[0]);
	        }
	        if(bno_id[0] != BNO055_ID) {
	             return;
	        }
	    	reset();
	    	// Set to normal power mode.
	        write(BNO055_PWR_MODE_ADDR, new byte[] {POWER_MODE_NORMAL}, false);
	        // Default to internal oscillator.
	        write(BNO055_SYS_TRIGGER_ADDR, new byte[]{(byte)0x0}, false);
	        //Enter normal operation mode.
	        set_mode(OPERATION_MODE_NDOF);
	        //if( Props.DEBUG ) System.out.println("Connected to "+portName);
	    }
	    
	    public void close() {
	    	if( serialPort != null)
	    		serialPort.close();
	    }
	    
	    public int bytesToRead() throws IOException {
	    	return inStream.available();
	    }
	    /**
	     * Acknowledge Response:
		 * Byte 1 Byte 2
		 * Response Header Status
		 * 0xEE 0x01: WRITE_SUCCESS
		 * 0x03: WRITE_FAIL
		 * 0x04: REGMAP_INVALID_ADDRESS
		 * 0x05: REGMAP_WRITE_DISABLED
		 * 0x06: WRONG_START_BYTE
		 * 0x07: BUS_OVER_RUN_ERROR
		 * 0X08: MAX_LENGTH_ERROR
		 * 0x09: MIN_LENGTH_ERROR
		 * 0x0A: RECEIVE_CHARACTER_TIMEOUT
	     * @param address
	     * @param data
	     * @param ack
	     * @throws IOException
	     */
	    public void write(byte address, byte[] data, boolean ack) throws IOException {
	    	//if( Props.DEBUG ) System.out.println("write "+c);
	    	// Build and send serial register write command.
	    	synchronized(writeMx) {
	    		checkWriteBuffer();
	    		writeBuffer[writeBufferTail++] = (byte) 0xAA; // Start byte
	    		checkWriteBuffer();
	    		writeBuffer[writeBufferTail++] = (byte) 0x00;  // Write
	    		checkWriteBuffer();
	    		writeBuffer[writeBufferTail++] =  (byte) (address & 0xFF);
	    		checkWriteBuffer();
	    		writeBuffer[writeBufferTail++] =  (byte) (data.length & 0xFF);
	    		for(int i = 0; i < data.length; i++) {
	    			checkWriteBuffer();
	    			writeBuffer[writeBufferTail++] =  (byte) (data[i] & 0xFF);
	    		}
	    		writeMx.notify();
	    	}
	    	if( !ack )
	    		return;
	    	// wait for buffer to empty
	    	//if( DEBUG )
	    	//	System.out.println("Wait for write buffer to empty");
	    	
	    	//synchronized(writeMx) {
	    	//	while(writeBufferHead != writeBufferTail) {
	    	//		try {
	    	//			Thread.sleep(1);
	    	//		} catch (InterruptedException e) {}
	    	//	}
	    	//}
	    	// read the ack
	    	//try {
			//	Thread.sleep(30);
			//} catch (InterruptedException e) {}
	    	//if( DEBUG )
	    	//	System.out.println("Preparing to receive ACK");
	    	
	    	byte resp;
	    	synchronized( readMx) {
	    		checkReadBuffer();
	    		resp = (byte) readBuffer[readBufferHead++];
	    		if( resp != (byte)0xEE ) {
	    			System.out.printf("Received unexpected response in write ACK: %02x while looking for ACK byte 'ee'\r\n", resp);
	    			return;
	    		}
	    		if( DEBUG )
	    			System.out.println("Found ack header");
	    		checkReadBuffer();
	    		resp = (byte) readBuffer[readBufferHead++];
	    		if( resp != (byte)0x01 ) {
	    				System.out.printf("Error from write ACK:\r\n",resp);
	    				return;
	    		}
	    	}
	    	if( DEBUG ) {
	    		System.out.println("ACK successful");
	    	}
	    }
	    
		@Override
		public void write(int c) throws IOException {
	    	synchronized(writeMx) {
	    		writeBuffer[writeBufferTail++] = (byte)( c & 0xFF);
	    		checkWriteBuffer();
	       		writeMx.notify();
	    	}
		}
		
	    private void checkWriteBuffer() {
	    	if( writeBufferTail >= writeBuffer.length) {	
    				writeBufferTail = 0;
			}
	    }
	    /**
		 * 0xEE - error, 0xBB - success
		 * byte 2: returned from method, 0 if success
		 * 0x02: READ_FAIL
		 * 0x04: REGMAP_INVALID_ADDRESS
		 * 0x05: REGMAP_WRITE_DISABLED
		 * 0x06: WRONG_START_BYTE
		 * 0x07: BUS_OVER_RUN_ERROR
		 * 0X08: MAX_LENGTH_ERROR
		 * 0x09: MIN_LENGTH_ERROR
		 * 0x0A: RECEIVE_CHARACTER_TIMEOUT
		 */
	    private byte signalRead(byte address, byte length) throws IOException {
		    	//if( Props.DEBUG ) System.out.println("write "+c);
		    	// Build and send serial register write command.
	    	if( DEBUG )
	    		System.out.println("Sending header to signalRead");
		    	synchronized(writeMx) {
		    		checkWriteBuffer();
		    		writeBuffer[writeBufferTail++] = (byte) 0xAA; // Start byte
		    		checkWriteBuffer();
		    		writeBuffer[writeBufferTail++] = (byte) 0x01;  // Read
		    		checkWriteBuffer();
		    		writeBuffer[writeBufferTail++] =  (byte) (address & 0xFF);
		    		checkWriteBuffer();
		    		writeBuffer[writeBufferTail++] =  (byte) (length & 0xFF);
		    		writeMx.notify();
		    	}
		      	if( DEBUG )
		    		System.out.println("wait for empty buffer");
		    	// wait for buffer to empty
		    	synchronized(writeMx) {
		    		while(writeBufferHead != writeBufferTail) {
		    			try {
		    				Thread.sleep(1);
		    			} catch (InterruptedException e) {}
		    		}
		    	}
		      	if( DEBUG )
		    		System.out.println("Reading response header in signalRead");
		    	byte resp;
		    	synchronized( readMx) {
		    		checkReadBuffer();
		    		resp = (byte) readBuffer[readBufferHead++];
		    		if( resp == (byte)0xBB )
		    			return 0;
		    		// read fail is EE, otherwise confustion
		    		if( DEBUG )
		    			System.out.println("Did NOT receive expected 'bb' response in signalRead");
		    		if( resp != (byte)0xEE ) {
		    			System.out.printf("Received unexpected response in signalRead: %02x while looking for error byte 'ee'\r\n", resp);
		    			return (byte)0xFF;
		    		} else {
		    			checkReadBuffer();
		    			resp = (byte) readBuffer[readBufferHead++];
		    			if( DEBUG ) {
		    				System.out.println("Recovered error code from signalRead");
		    			}
		    		}
		    	}
		    	// should have the error code
		    	System.out.printf("Bad response from IMU: %02x\r\n",resp);
		    	return resp;
	    }
	        
	    /**
	     * Read Success Response:
		 * Byte 1 Byte 2 Byte 3 Byte (n+2)
		 * ResponseByte length Data 1 Data n
		 * 0xBB <..>
		 * Read Failure or Acknowledge Response:
		 * Byte 1 Byte 2
		 * Response Header Status
		 * 0xEE 0x02: READ_FAIL
		 * 0x04: REGMAP_INVALID_ADDRESS
		 * 0x05: REGMAP_WRITE_DISABLED
		 * 0x06: WRONG_START_BYTE
		 * 0x07: BUS_OVER_RUN_ERROR
		 * 0X08: MAX_LENGTH_ERROR
		 * 0x09: MIN_LENGTH_ERROR
		 * 0x0A: RECEIVE_CHARACTER_TIMEOUT
	     */
	    public byte[] read(byte address, byte length) throws IOException {
	    	//if( Props.DEBUG ) System.out.println("read");
	    	// Build and send serial register read command.
	    	int retry = 0;
	    	byte resp;
	    	while( (resp = signalRead(address, length)) == (byte)0x07 && retry++ < 10) {
	    		try {
					Thread.sleep(1);
				} catch (InterruptedException e) {}
	    	}
	    	if( resp != 0 ) {
	    		System.out.println("Bad response for signalRead, exiting read..."+resp);
	    		return null;
	    	}
            // Verify register read succeeded.
	    	synchronized(readMx) {
	    		checkReadBuffer();
	    		//if( Props.DEBUG ) System.out.println("readBufferHead="+readBufferHead+" readBufferTail="+readBufferTail+" = "+readBuffer[readBufferHead]);
	    		if( (resp = (byte) readBuffer[readBufferHead++]) != (byte)0xBB) 
	                 System.out.printf("Register read error: %02x looking for 'bb'\r\n", resp);
	            // Read the returned bytes.
	    		checkReadBuffer();
	            byte blen = (byte) readBuffer[readBufferHead++];          	
	            if( blen == 0 || blen != length) {
	                System.out.println("Timeout waiting to read data, is the BNO055 connected?");
	                System.out.printf("Received: %d but looking for %d bytes.\r\n", blen, length);
	            }
	            byte[] bout = new byte[blen];
	            for(int i = 0; i < blen; i++) {
	            	checkReadBuffer();
	            	bout[i] = (byte) readBuffer[readBufferHead++];
	            }
	            return bout;
	    	}
	    	//return inStream.read();
	    }
	    
	    private void checkReadBuffer() {
    		try {
    			if( readBufferHead == readBuffer.length)
    				readBufferHead = 0;
    			if( readBufferHead == readBufferTail )
    				readMx.wait();
			} catch (InterruptedException e) {}
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
	    
	    private void set_mode(byte mode) throws IOException {
	    	if( DEBUG )
	    		System.out.printf("Setting mode:%02x\r\n",mode);
	       //Set operation mode for BNO055 sensor.  Mode should be a value from
	       //table 3-3 and 3-5 of the datasheet:
	       // http://www.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
	       // 
	       write(BNO055_OPR_MODE_ADDR, new byte[]{(byte)(mode & (byte)0xFF)}, false);
	       // Delay for 30 milliseconds (datasheet recommends 19ms, but a little more
	       // can't hurt and the kernel is going to spend some unknown amount of time too).
	       try {
			Thread.sleep(30);
	       } catch (InterruptedException e) {}
	       if( DEBUG )
	    		System.out.println("Exiting set_mode");
	    }

	    private void reset() throws IOException {
	    	if( DEBUG )
	    		System.out.println("entering device reset");
	    	 write(BNO055_SYS_TRIGGER_ADDR, new byte[]{(byte)0x20}, false);
	         // Wait 650ms after reset for chip to be ready (as suggested in datasheet).
	         try {
				Thread.sleep(650);
			} catch (InterruptedException e) {}
	         // Set to normal power mode.
	         write(BNO055_PWR_MODE_ADDR, new byte[]{(byte)POWER_MODE_NORMAL}, false);
	         // Default to internal oscillator.
	         write(BNO055_SYS_TRIGGER_ADDR, new byte[]{(byte)0x0}, false);
	         // Enter normal operation mode.
	         set_mode(OPERATION_MODE_NDOF);
		    if( DEBUG )
		    	System.out.println("exiting device reset");  
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
		    String msg = "IMUSerialDataPort\n";
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
	            public boolean isRunning = false;
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
	                			if( writeBufferHead == writeBuffer.length)
	                				writeBufferHead = 0;
	                			if( writeBufferHead == writeBufferTail ) {
	                				//System.out.println("Enter wait writer:"+writeBufferHead+" "+writeBufferTail);
	                    			writeMx.wait();
	                    			//System.out.println("Leave wait writer:"+writeBufferHead+" "+writeBufferTail);
	                			}
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


			@Override
			public String readLine() {
				throw new RuntimeException("refactor! not applicable method");
			}

			@Override
			public void writeLine(String output) throws IOException {
				throw new RuntimeException("refactor! not applicable method");
				
			}

			@Override
			public int read() throws IOException {
				throw new RuntimeException("wrong method, need address etc!");
			}
	        
}
