package com.neocoretechs.robocore.serialreader;

import gnu.io.CommPortIdentifier;
import gnu.io.CommPortOwnershipListener;
import gnu.io.NoSuchPortException;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;
import gnu.io.UnsupportedCommOperationException;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.Enumeration;

import com.neocoretechs.robocore.ThreadPoolManager;

/**
 * Uses the serial UART mode of the BNO055 Bosch 9 axis sensor fusion package and presents a series of methods to read
 * The accelerometer, gyro, magnetometer, fused Euler angle data, and temperature.
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2017,2018,2019,2020,2021
 *
 */
public class IMUSerialDataPort implements DataPortInterface {
		private static boolean DEBUG = false;
		private static boolean PORTDEBUG = false;
		private static boolean INFO = true;
	    private SerialPort serialPort;
	    private OutputStream outStream;
	    private InputStream inStream;
		// serial settings
		// PortSettings=115200,n,8,1
	    // On RasPi its /dev/ttyS0, on OdroidC2, ttyS0 is hardwired console so we use ttyS1 on header
	    // On C1 we have ttyS2 so basically, if we have ttyS2 use that, if we have a ttyS1, use it, otherwise, use ttyS0
	    // 
	    private static String portName = "/dev/ttyS0";
	    private static int baud = 115200;
	    private static int datab = 8;
	    private static int stopb = 1;
	    private static int parityb = 0;
	    //
	    private static Object readMx = new Object();// mutex
	    private static Object writeMx = new Object();
	    private static boolean EOT = false;
	 
	    private static int[] readBuffer = new int[512];
	    private static int[] writeBuffer = new int[256];
	    private static int readBufferHead = 0;
	    private static int readBufferTail = 0;
	    private static int writeBufferHead = 0;
	    private static int writeBufferTail = 0;
	    
	    private static volatile IMUSerialDataPort instance = null;
	    private static Object mutex = new Object();
	    private int portOwned = CommPortOwnershipListener.PORT_UNOWNED;
	    private CommPortIdentifier portId = null;
	    
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
	    // Accel data register
	    public static final byte BNO055_ACCEL_DATA_X_LSB_ADDR = (byte)0x08;
	    // Mag data register
	    public static final byte BNO055_MAG_DATA_X_LSB_ADDR = (byte)0x0E;
	    // Gyro register
	    public static final byte BNO055_GYRO_DATA_X_LSB_ADDR = (byte)0x14;
	    // Euler register
	    public static final byte BNO055_EULER_H_LSB_ADDR = (byte)0x1A;
	    // Temperature data register
	    public static final byte BNO055_TEMP_ADDR  = (byte)0x34;
	    // Quaternion data register
	    public static final byte BNO055_QUATERNION_DATA_W_LSB_ADDR = (byte)0x20;
	    //
	    public static final byte POWER_MODE_NORMAL = (byte)0x00;
	    // Calibration register
	    public static final byte BNO055_CALIB_STAT_ADDR = (byte)0X35;
	    
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
	    
	    public static final byte ACCEL_OFFSET_X_LSB_ADDR              = (byte)0x55;
	    
	    // Axis remap registers
	    public static final byte BNO055_AXIS_MAP_CONFIG_ADDR          = (byte)0x41;
	    public static final byte BNO055_AXIS_MAP_SIGN_ADDR            = (byte)0x42;
	    //Axis remap values
	    public static final byte AXIS_REMAP_X                         = (byte)0x00;
	    public static final byte AXIS_REMAP_Y                         = (byte)0x01;
	    public static final byte AXIS_REMAP_Z                         = (byte)0x02;
	    public static final byte AXIS_REMAP_POSITIVE                  = (byte)0x00;
	    public static final byte AXIS_REMAP_NEGATIVE                  = (byte)0x01;
	    
	    // Status registers
	    public static final byte BNO055_SELFTEST_RESULT_ADDR          = (byte)0x36;
	    public static final byte BNO055_INTR_STAT_ADDR                = (byte)0x37;

	    public static final byte BNO055_SYS_CLK_STAT_ADDR             = (byte)0x38;
	    public static final byte BNO055_SYS_STAT_ADDR                 = (byte)0x39;
	    public static final byte BNO055_SYS_ERR_ADDR                  = (byte)0x3A;

	    
		private String CALIBRATION_FILE = "/var/local/calibration.json";
		
		// Calibration tolerances for various sensors
		private int SYSTEM_CAL = 3;
		private int ACC_CAL = 3;
		private int GYRO_CAL = 3;
		private int MAG_CAL = 3;
		// precision for position readout, number of decimal places
		private int IMU_TOL = 3;

	    /**
	     * Try to determine port, if we cant through cpuinfo, use default
	     * @return An instance of IMUSerialDataPort, singleton for this class
	     */
	    public static IMUSerialDataPort getInstance() {
	    	synchronized(mutex) {
	    	if( instance == null ) {
	    		try {
	    			CommPortIdentifier cpi  = null;
	    			try {
	    				System.setProperty("gnu.io.rxtx.SerialPorts", "/dev/ttyS2");
	    				cpi = CommPortIdentifier.getPortIdentifier("/dev/ttyS2");
	    				portName = "/dev/ttyS2";
	    			} catch(NoSuchPortException nspe) {
	    				try {
	    					System.setProperty("gnu.io.rxtx.SerialPorts", "/dev/ttyS1");
	    					cpi = CommPortIdentifier.getPortIdentifier("/dev/ttyS1"); 
	    					portName = "/dev/ttyS1";
	    				} catch(NoSuchPortException nspe2) {
	    					try {
	    						System.setProperty("gnu.io.rxtx.SerialPorts", "/dev/ttyS0");
								cpi = CommPortIdentifier.getPortIdentifier("/dev/ttyS0");
								portName = "/dev/ttyS0";
							} catch (NoSuchPortException e) {
								System.out.println("Could not initialize IMUSerialDataPort of /dev/ttyS0, /dev/ttyS1, or /dev/ttyS2:"+e);
								e.printStackTrace();
								throw new RuntimeException(e);
							}
	    				}	
	    			}
					instance = new IMUSerialDataPort(portName, baud, datab, stopb, parityb);									 
				} catch (IOException e) {
					System.out.println("Could not initialize IMUSerialDataPort of:"+portName+" because "+e);
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
	    	if( DEBUG || INFO) 
	    		System.out.println("IMUSerialDataPort "+portName+" baud="+baud+" databits="+datab+" stopbits="+stopb+" parity="+parityb);
	    	connect(true);
	    	//clear();
	    }
	    
		public void setCalibrationFile(String calibrationFile) { CALIBRATION_FILE = calibrationFile; }
		public void setSYSTEM_CAL(int sYSTEM_CAL) {	SYSTEM_CAL = sYSTEM_CAL; }
		public void setACC_CAL(int aCC_CAL) { ACC_CAL = aCC_CAL; }
		public void setGYRO_CAL(int gYRO_CAL) { GYRO_CAL = gYRO_CAL; }
		public void setMAG_CAL(int mAG_CAL) {MAG_CAL = mAG_CAL;}
		public void setIMU_TOL(int imu_TOL) {IMU_TOL = imu_TOL;}
	    public String getPortName() { return portName; }
	    public int getBaudRate() { return baud; }
	    public int getDataBits() { return datab; }
	    public int getStopBits() { return stopb; }
	    public int getParity() { return parityb; }
	    public int getHandshake() { return (serialPort == null ? -1 : serialPort.getFlowControlMode()); }
	    public boolean isEOT() { return EOT; }
	    
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
	        //
	        // set up BNO055
	        // set config mode
	    	set_mode(OPERATION_MODE_CONFIG);
	        // now read without kruft
	        while(write(BNO055_PAGE_ID_ADDR, new byte[]{(byte)0x00}, true)) {
	        	try {
					Thread.sleep(100);
				} catch (InterruptedException e) {}
	        }
	        readChipId();
	    	reset();
	    	// obtain calibration data if we can
	    	byte[] calData = null;
	    	try {
	    		calData = readCalibration();
	    		set_mode(OPERATION_MODE_CONFIG);
	    		setCalibration(calData);
	    	} catch(FileNotFoundException fnfe) {}
	    	setNormalPowerNDOFMode();
	    	//reportCalibrationStatus();
	        if( PORTDEBUG ) 
	        	System.out.println("Connected to "+portName+" and BNO055 IMU is ready!");
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
		 * 0x03: WRITE_FAIL - check connection, protocol settings and operation mode
		 * 0x04: REGMAP_INVALID_ADDRESS - check if the register is addressable
		 * 0x05: REGMAP_WRITE_DISABLED - check register property, ie read only
		 * 0x06: WRONG_START_BYTE - Check if first byte sent is 0xAA
		 * 0x07: BUS_OVER_RUN_ERROR - resend the command
		 * 0X08: MAX_LENGTH_ERROR - split command so single frame < 128 bytes
		 * 0x09: MIN_LENGTH_ERROR - send a valid frame
		 * 0x0A: RECEIVE_CHARACTER_TIMEOUT - decrease waiting time between sending of 2 bytes of 1 frame
	     * @param address
	     * @param data
	     * @param ack
	     * @return true if a command retry is necessary due to buffer overrun
	     * @throws IOException
	     */
	    public boolean write(byte address, byte[] data, boolean ack) throws IOException {
	    	//if( Props.DEBUG ) System.out.println("write "+c);
	    	// Build and send serial register write command.
	    	write((byte) 0xAA); // Start byte
	    	write((byte) 0x00);  // Write
	    	write((byte) (address & 0xFF));
	    	write((byte) (data.length & 0xFF));
	    	for(int i = 0; i < data.length; i++) {
	    			write((byte) (data[i] & 0xFF));
	    	}
	        if( ack ) {
	        	byte resp;
	        	while((resp = (byte)(read() & 0xFF)) == (byte)0xEE) {
	        		try {
						Thread.sleep(1);
					} catch (InterruptedException e) {}
	        	}
	        	//if( resp != (byte)0xEE ) {
	    		//	throw new IOException(String.format("Response header corrupted in write ACK: %02x while looking for ACK byte 'ee'\r\n", resp));			
	        	//}
	        	//if( DEBUG )
	        	//	System.out.println("Found ack header");
	        	// we always use ACK except for reset, but it seems to send back the ee
	        	//resp = (byte)( read() & 0xFF);
	        	if( resp != (byte)0x01 ) {
	        		switch(resp) {
	        		case ((byte)0x03): 
	        			throw new IOException(String.format("Error from write ACK:%02x WRITE_FAIL - check connection, protocol settings and operation mode\r\n",resp));
	        		case((byte)0x04):
	        			throw new IOException(String.format("Error from write ACK:%02x REGMAP_INVALID_ADDRESS - check if the register is addressable\r\n",resp));
	        		case((byte)0x05): 
	        			throw new IOException(String.format("Error from write ACK:%02x REGMAP_WRITE_DISABLED - check register property, ie read only\r\n",resp));
	        		case((byte)0x06): 
	        			throw new IOException(String.format("Error from write ACK:%02x WRONG_START_BYTE - Check if first byte sent is 0xAA\r\n",resp));
	        		case((byte)0x07):
	        			//System.out.println(String.format("Error from write ACK:%02x BUS_OVER_RUN_ERROR - resend the command\r\n",resp));
	        			return true;
	        		case((byte)0X08):
	        			throw new IOException(String.format("Error from write ACK:%02x MAX_LENGTH_ERROR - split command so single frame < 128 bytes\r\n",resp));
	        		case((byte)0x09): 
	        			throw new IOException(String.format("Error from write ACK:%02x MIN_LENGTH_ERROR - send a valid frame\r\n",resp));
	        		case((byte)0x0A):
	        			throw new IOException(String.format("Error from write ACK:%02x RECEIVE_CHARACTER_TIMEOUT - decrease waiting time between sending of 2 bytes of 1 frame\r\n",resp));
	        		default:
	    			throw new IOException(String.format("UnknownError from write ACK:%02x\r\n",resp));
	        		}
	        	}
	        }
	        if( DEBUG ) {
        		System.out.println("ACK successful");
	        }
	        return false;
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
	    	//if( DEBUG )
	    	//	System.out.println("Setting up read registers from signalRead..");
		    write((byte) 0xAA); // Start byte
		    write((byte) 0x01);  // Read
		    write((byte) (address & 0xFF));
		    write((byte) (length & 0xFF));		
		    //if( DEBUG )
		    //		System.out.println("Reading response header in signalRead");
		    byte resp;
		    resp = (byte)( read() & 0xFF);
		    if( resp == (byte)0xBB )
		    		return 0;
		    // read fail is EE, otherwise confustion
		    //if( DEBUG )
		    //	System.out.println("Did NOT receive expected 0xBB response in signalRead");
		    if( resp != (byte)0xEE ) {
		    	if(DEBUG)
		    		System.out.printf("Received unexpected response in signalRead: %02x while looking for error byte 'ee'\r\n", resp);
		    	return (byte)0x07; // lets call this bus overrun from chip
		    } else {
		    	resp = (byte)( read() & 0xFF);
		    	//if( DEBUG ) {
		    	//	System.out.println("Recovered error code from signalRead");
		    	//}
		    }
		    // should have the error code
		    if(DEBUG)
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
		 * 0x03 : WRITE_FAIL - check connection protocol setting and operation of BNO
		 * 0x04 : REGMAP_INVALID_ADDRESS - Check that register is addressable
		 * 0x06 : START BYTE IS NOT 0xAA - Check that the start byte is 0xAA
		 * 0x07 : BUS_OVER_RUN_ERROR - BNO was not able to clear buffer, re-send command
		 * 0x08 : MAX_LENGTH_ERR - Max length of data is > 128 0x80 split frame
		 * 0x09 : MIN_LENGTH_ERR - Min length of data is < 1, send valid frame
		 * 0x0A : RECIEVE_CHARACTER_TIMEOUT - If next character does not arrive within 100ms, decrease wait time between bytes
	     */
	    public byte[] read(byte address, byte length) throws IOException {
	    	//if( Props.DEBUG ) System.out.println("read");
	    	// Build and send serial register read command.
	    	int retry = 0;
	    	byte resp;
	    	while( (resp = signalRead(address, length)) == (byte)0x07 && retry++ < 10) {
	    		try {
					Thread.sleep(100);
				} catch (InterruptedException e) {}
	    	}
	    	if( resp != 0 ) {
	       		switch(resp) {
        		case ((byte)0x03): 
        			throw new IOException(String.format("Error from write ACK:%02x WRITE_FAIL - check connection, protocol settings and operation mode\r\n",resp));
        		case((byte)0x04):
        			throw new IOException(String.format("Error from write ACK:%02x REGMAP_INVALID_ADDRESS - check if the register is addressable\r\n",resp));
        		case((byte)0x06): 
        			throw new IOException(String.format("Error from write ACK:%02x WRONG_START_BYTE - Check if first byte sent is 0xAA\r\n",resp));
        		case((byte)0x07):
        			throw new IOException(String.format("Error from write ACK:%02x BUS_OVER_RUN_ERROR - resend the command\r\n",resp));
        		case((byte)0X08):
        			throw new IOException(String.format("Error from write ACK:%02x MAX_LENGTH_ERROR - split command so single frame < 128 bytes\r\n",resp));
        		case((byte)0x09): 
        			throw new IOException(String.format("Error from write ACK:%02x MIN_LENGTH_ERROR - send a valid frame\r\n",resp));
        		case((byte)0x0A):
        			throw new IOException(String.format("Error from write ACK:%02x RECEIVE_CHARACTER_TIMEOUT - decrease waiting time between sending of 2 bytes of 1 frame to < 100ms\r\n",resp));
        		default:
        			throw new IOException("Bad response for signalRead after " + retry +" retries, exiting read with response:"+resp);
	       		}
	    	}
	    	// Returning with 0 from signalRead means we found 0xBB
	    	// next byte is length, then data 1..n
	    	//if( Props.DEBUG ) System.out.println("readBufferHead="+readBufferHead+" readBufferTail="+readBufferTail+" = "+readBuffer[readBufferHead]);
	    	byte blen = (byte)( read() & 0xFF);
	    	//if( DEBUG )
	    	//	System.out.printf("Recovered length byte: %02x\r\n", blen);
	        // Read the returned bytes.	
	        if( blen == 0 || blen != length) {
	                throw new IOException(String.format("Received length byte: %d but read requested %d bytes.\r\n", blen, length));
	        }
	        byte[] bout = new byte[blen];
	        for(int i = 0; i < blen; i++) {
	            	bout[i] = (byte)( read() & 0xFF);
	        }
	        return bout;
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
	     * Set the operation mode for sensor
	     * @param mode
	     * @throws IOException
	     */
	    private void set_mode(byte mode) throws IOException {
	    	if( DEBUG )
	    		System.out.printf("Setting mode:%02x\r\n",mode);
	       //Set operation mode for BNO055 sensor.  Mode should be a value from
	       //table 3-3 and 3-5 of the datasheet:
	       // http://www.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
	       // 
	       while(write(BNO055_OPR_MODE_ADDR, new byte[]{(byte)(mode & (byte)0xFF)}, true)) {
	         	 try {
						Thread.sleep(100);
				} catch (InterruptedException e) {}
	       }
	       // Delay for 30 milliseconds (datasheet recommends 19ms, but a little more
	       // can't hurt and the kernel is going to spend some unknown amount of time too).
	       try {
			Thread.sleep(30);
	       } catch (InterruptedException e) {}
	       //if( DEBUG )
	    	//	System.out.println("Exiting set_mode");
	    }

	    private void reset() throws IOException {
	    	if( DEBUG )
	    		System.out.println("entering device reset");
	    	 write(BNO055_SYS_TRIGGER_ADDR, new byte[]{(byte)0x20}, false);
	         // Wait 650ms after reset for chip to be ready (as suggested in datasheet).
	         try {
				Thread.sleep(650);
			} catch (InterruptedException e) {}
		    //if( DEBUG )
		    //	System.out.println("exiting device reset");  
	    }
	    
	    /**
	     * Set the sensor fusion NDOF mode
	     * @throws IOException
	     */
	    private void setNormalPowerNDOFMode() throws IOException {
	         // Set to normal power mode.
	         while(write(BNO055_PWR_MODE_ADDR, new byte[]{(byte)POWER_MODE_NORMAL}, true)) {
	        	 try {
					Thread.sleep(100);
				} catch (InterruptedException e) {}
	         }
	         // Default to internal oscillator.
	         while(write(BNO055_SYS_TRIGGER_ADDR, new byte[]{(byte)0x0}, true)) {
	           	 try {
						Thread.sleep(100);
				 } catch (InterruptedException e) {}
	         }
	         // Enter normal operation mode.
	         set_mode(OPERATION_MODE_NDOF);
	    }
	    
	    private void readChipId() throws IOException {
	        //if( DEBUG )
	        //	System.out.println("Read chip id..");
	        // Check the chip ID
	        byte[] bno_id = read(BNO055_CHIP_ID_ADDR,(byte)0x01);
	        if( DEBUG) {
	        	System.out.printf("Read chip ID: %02x\r\n",bno_id[0]);
	        }
	        if(bno_id[0] != BNO055_ID) {
	        	if(DEBUG)
	        		System.out.printf("Recovered chip Id did NOT match expected byte!:%02x\r\n",bno_id[0]);
	        	close();
	            throw new IOException("BNO055 chip id did not match expected value");
	        }
	    }
	    
	    /**
	     * Read the calibration status of the sensors and return a 4 tuple with
	     * calibration status as follows:
	     * - System, 3=fully calibrated, 0=not calibrated
	     * - Gyroscope, 3=fully calibrated, 0=not calibrated
	     * - Accelerometer, 3=fully calibrated, 0=not calibrated
	     * - Magnetometer, 3=fully calibrated, 0=not calibrated
	     * According to docs, one should ignore those device readings with 0 calibration status.
	     * If the system or accel is 0, readings from fusion are considered worthless.
	     * @return A 4 byte array representing the above calibration status
	     * @throws IOException 
	     */
	    public byte[] getCalibrationStatus() throws IOException {
	        // Return the calibration status register value.
	        byte[] cal_status = read(BNO055_CALIB_STAT_ADDR, (byte)0x01);
	        byte sys = (byte) ((byte)(cal_status[0] >> 6) & (byte)0x03);
	        byte gyro = (byte) (((byte)cal_status[0] >> 4) & (byte)0x03);
	        byte accel = (byte) (((byte)cal_status[0] >> 2) & (byte)0x03);
	        byte mag = (byte) ((byte)cal_status[0] & (byte)0x03);
	        // Return the results as a tuple of all 3 values.
	        return new byte[]{sys, gyro, accel, mag};
	    }
	    /**
	     * Attempt to reset calibration by generating a reset then reading calibration data and calibrating
	     * @throws IOException
	     */
	    public void resetCalibration() throws IOException {
	        // set up BNO055
	        // set config mode
	    	set_mode(OPERATION_MODE_CONFIG);
	        // now read without kruft
	        while(write(BNO055_PAGE_ID_ADDR, new byte[]{(byte)0x00}, true)) {
	          	try {
						Thread.sleep(100);
				} catch (InterruptedException e) {}
	        }
	        readChipId();
	    	reset();
	    	// obtain calibration data if we can
	    	byte[] calData = null;
	    	try {
	    		calData = readCalibration();
	    		set_mode(OPERATION_MODE_CONFIG);
	    		setCalibration(calData);
	    	} catch(FileNotFoundException fnfe) {}
	    	setNormalPowerNDOFMode();
	        if( DEBUG ) 
	        	System.out.println("BNO055 IMU is re-calibrated!");
	    }
	    /* Convert the value to an appropriate range (section 3.6.4) */
	    
	    /**
	     * Read accelerometer
	     * @return int array of x,y,z accel values
	     * @throws IOException
	     */
	    public int[] readAccel() throws IOException {
	    	byte[] data = read(BNO055_ACCEL_DATA_X_LSB_ADDR, (byte)6);
	    	int c = 0;
	    	while( data == null ) {
	    		if(++c == 5) return null;
		    	reset();
		    	setNormalPowerNDOFMode();
		    	data = read(BNO055_ACCEL_DATA_X_LSB_ADDR, (byte)6);
	    	}
	    	short xAccl = (short)((data[0] & 0xFF)
					| ((data[1] << 8) & 0xFF00));
			short yAccl = (short)((data[2] & 0xFF)
					| ((data[3] << 8) & 0xFF00));
			short zAccl = (short)((data[4] & 0xFF)
					| ((data[5] << 8) & 0xFF00));
	    	// Convert the data
			/* 1m/s^2 = 100 LSB, to get readings in mg div by 1 */
	    	return new int[]{xAccl/100, yAccl/100, zAccl/100};
	    }
	    
	    /**
	     * Read the magnetometer sensor data
	     * @return X/Y/Z magnetic field strength of axis in microteslas
	     * @throws IOException
	     */
	    public int[] readMag() throws IOException {
	    	// Read 6 bytes of data from address 0x0E(14)
	    	// xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
	    	byte[] data = read(BNO055_MAG_DATA_X_LSB_ADDR, (byte)6);
	    	int c = 0;
	    	while( data == null ) {
	    		if(++c == 5) return null;
		    	reset();
		    	setNormalPowerNDOFMode();
		    	data = read(BNO055_MAG_DATA_X_LSB_ADDR, (byte)6);
	    	}
	    	// Convert the data
	    	short xMag = (short)((data[0] & 0xFF)
						| ((data[1] << 8) & 0xFF00));
			short yMag = (short)((data[2] & 0xFF)
						| ((data[3] << 8) & 0xFF00));
			short zMag = (short)((data[4] & 0xFF)
						| ((data[5] << 8) & 0xFF00));
	    	return new int[]{xMag/16, yMag/16, zMag/16};
	    }
	    
	    /**
	     * Read the gyroscope sensor data
	     * @return x/y/z/ gyro reading
	     * @throws IOException
	     */
	    public int[] readGyro() throws IOException {
	    	// Read 6 bytes of data from address 0x14(20)
	    	// xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
	    	byte[] data = read(BNO055_GYRO_DATA_X_LSB_ADDR, (byte)6);
	    	int c = 0;
	    	while( data == null ) {
	    		if(++c == 5) return null;
		    	reset();
		    	setNormalPowerNDOFMode();
		    	data = read(BNO055_GYRO_DATA_X_LSB_ADDR, (byte)6);
	    	}
	    	short xGyro = (short)((data[0] & 0xFF)
						| ((data[1] << 8) & 0xFF00));
			short yGyro = (short)((data[2] & 0xFF)
						| ((data[3] << 8) & 0xFF00));
			short zGyro = (short)((data[4] & 0xFF)
						| ((data[5] << 8) & 0xFF00));
	    	// Convert the data, 900 is RPS, radians per second, 16 is DPS, degrees per second
	    	//return new int[]{xGyro/900, yGyro/900, zGyro/900};
	    	return new int[]{xGyro/16, yGyro/16, zGyro/16};
	    }
	    
	    /**
	     * Return euler orientation data from the sensor
	     * @return the Yaw/Pitch/Roll integer array, or null if read error from sensor
	     * @throws IOException
	     */
	    public double[] readEuler() throws IOException {
	        //Return the current absolute orientation as a tuple of heading, roll, and pitch euler angles in degrees.
	    	byte[] data = read(BNO055_EULER_H_LSB_ADDR, (byte)6);
	    	int c = 0;
	    	while( data == null ) {
	    		if(++c == 5) return null;
		    	reset();
		    	setNormalPowerNDOFMode();
		    	data = read(BNO055_EULER_H_LSB_ADDR, (byte)6);
	    	}
	    	short heading = (short)((data[0] & 0xFF)
					| ((data[1] << 8) & 0xFF00));
			short roll = (short)((data[2] & 0xFF)
					| ((data[3] << 8) & 0xFF00));
			short pitch = (short)((data[4] & 0xFF)
					| ((data[5] << 8) & 0xFF00));
	        return new double[]{round(((double)heading)/16.0, IMU_TOL), round(((double)roll)/16.0, IMU_TOL), round(((double)pitch)/16.0, IMU_TOL)};

	    }
	    
	    /**
	     * Read temperature from sensor
	     * @return The +/- temperature in current setting/mode, or Integer.MAX_VALUE if error
	     * @throws IOException
	     */
	    public int readTemperature() throws IOException {
	    	byte[] data = read(BNO055_TEMP_ADDR, (byte)1);
	    	int c = 0;
	    	while( data == null ) {
	    		if(++c == 5)  return Integer.MAX_VALUE;
		    	reset();
		    	setNormalPowerNDOFMode();
		    	data = read(BNO055_TEMP_ADDR, (byte)1);
	    	}
	    	int temp = data[0];
	    	if(temp > 127)
	            temp -= 256;
	    	return temp;
	    }
	    
	    /**
	     * Return the current orientation as a tuple of X, Y, Z, W quaternion values.
	     */
	    public double[] readQuaternion() throws IOException {
	    	byte[] data = read(BNO055_QUATERNION_DATA_W_LSB_ADDR, (byte)8);
	    	int c = 0;
	    	while( data == null ) {
	    		if(++c == 5) return null;
		    	reset();
		    	setNormalPowerNDOFMode();
		    	data = read(BNO055_QUATERNION_DATA_W_LSB_ADDR, (byte)8);
	    	}
	    	short w = (short)((data[0] & 0xFF)
					| ((data[1] << 8) & 0xFF00));
			short x = (short)((data[2] & 0xFF)
					| ((data[3] << 8) & 0xFF00));
			short y = (short)((data[4] & 0xFF)
					| ((data[5] << 8) & 0xFF00));
			short z = (short)((data[6] & 0xFF)
					| ((data[7] << 8) & 0xFF00));
	    	// Scale values, see 3.6.5.5 in the datasheet.
	    	double scale = (1.0 / (1<<14));
	    	return new double[]{round(x*scale, IMU_TOL), round(y*scale, IMU_TOL), round(z*scale, IMU_TOL), round(w*scale, IMU_TOL)};
	    }
		/**
		 * We are going to look for the calibration data in a form generated by the Adafruit webGl calibration
		 * process, which is rendered as a single line of JSON if we can find it, otherwise we will hand out relatively worthless
		 * answers upon each reset. Will will presume to have moved it to /var/local as a common reference point.
		 * @throws IOException 
		 */
		public byte[] readCalibration() throws FileNotFoundException, IOException {
					FileReader fis = new FileReader(CALIBRATION_FILE);
					BufferedReader br = new BufferedReader(fis);
					String s = br.readLine();
					br.close();
					fis.close();
					if( DEBUG )
						System.out.println("Calibration:"+s);
					String[] tokes = s.substring(1,s.length()-1).split(",");
					if( tokes.length != 22) {
						throw new IOException("Configuration file contains "+tokes.length+" items where 22 were expected!");
					}
					byte[] cb = new byte[22];
					for(int i = 0; i < cb.length; i++) {
						cb[i] = Integer.valueOf(tokes[i].trim()).byteValue();
						if(DEBUG)
							System.out.println("Calb "+i+":"+tokes[i].trim());
					}
					return cb;
		}
		/**
		 * Write calibration data in JSON as the Adafruit do.
		 * @param calb
		 * @throws FileNotFoundException
		 * @throws IOException
		 */
		public void writeCalibration(byte[] calb) throws FileNotFoundException, IOException {
			if( calb.length != 22) {
				throw new IOException("Configuration data contains "+calb.length+" items where 22 were expected!");
			}
			FileWriter fis = new FileWriter(CALIBRATION_FILE);
			BufferedWriter br = new BufferedWriter(fis);
			br.write("[");
			for(int i = 0; i < calb.length; i++) {
				System.out.println("Calb "+i+":"+calb[i]);
				br.write(String.valueOf((int)calb[i] & 0xFF));
				if( i != calb.length-1)
					br.write(", ");
			}
			br.write("]");
			br.close();
			fis.close();
		}
		/**
		 * Return the sensor's calibration data and return it as an array of
	     * 22 bytes. Can be saved and then reloaded with the set_calibration function
	     * to quickly calibrate from a previously calculated set of calibration data.
	     * Assume we switched to configuration mode, as mentioned in section 3.10.4 of datasheet.
		 * @throws IOException 
		 */
		private byte[] getCalibration() throws IOException {
	        //self._config_mode()
	        // Read the 22 bytes of calibration data and convert it to a list (from
	        // a bytearray) so it's more easily serialized should the caller want to
	        // store it.
	        return read(ACCEL_OFFSET_X_LSB_ADDR, (byte)22);
	        //Go back to normal operation mode.
	        //self._operation_mode()
		}
		/**
		 * Set the sensor's calibration data using a list of 22 bytes that
	     * represent the sensor offsets and calibration data.  This data should be
	     * a value that was previously retrieved with getCalibration (and then
	     * perhaps persisted to disk or other location until needed again).
		 * @param data
		 */
	    private void setCalibration(byte[] data) throws IOException {
	        // Check that 22 bytes were passed in with calibration data.
	    	if( data.length != 22)
	    		throw new IOException("Calibration data expected 22 bytes but got "+data.length);
	        //Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
	        //self._config_mode()
	        // Set the 22 bytes of calibration data.
	        while(write(ACCEL_OFFSET_X_LSB_ADDR, data, true)) {
	          	 try {
						Thread.sleep(100);
					} catch (InterruptedException e) {}
	        }
	        // Go back to normal operation mode.
	        //self._operation_mode()
	    }
	    /**
	     * Report the current calibration status.
	     * @return the 4 byte array for system status, gyro, accel, mag 0-3
	     * @throws IOException
	     */
	    public byte[] reportCalibrationStatus(byte[] stat) throws IOException {
	    	//byte[] stat = getCalibrationStatus();
	    	// system, gyro, accel, mag
	    	if( stat[0] != 3 )
	    		System.out.println("System status is less than full calibration at:"+stat[0]+", results will be "+
	    				(stat[0] == 0 ? "UNUSABLE" : stat[0] == 1 ? "INACCURATE" : stat[0] == 2 ? "MARGINAL" : "UNKNOWN"));
	    	if( stat[1] != 3 )
	    		System.out.println("Gyro calibration less than optimal at:"+stat[1]+", results will be "+
	    				(stat[1] == 0 ? "UNUSABLE" : stat[1] == 1 ? "INACCURATE" : stat[1] == 2 ? "MARGINAL" : "UNKNOWN"));
	    	if( stat[2] != 3 )
	    		System.out.println("Accelerometer calibration less than optimal at:"+stat[2]+", results will be "+
	    				(stat[2] == 0 ? "UNUSABLE" : stat[2] == 1 ? "INACCURATE" : stat[2] == 2 ? "MARGINAL" : "UNKNOWN"));
	    	if( stat[3] != 3 )
	    		System.out.println("Magnetometer calibration less than optimal at:"+stat[3]+", results will be "+
	    				(stat[3] == 0 ? "UNUSABLE" : stat[3] == 1 ? "INACCURATE" : stat[3] == 2 ? "MARGINAL" : "UNKNOWN"));
	    	if( stat[0] == 3 && stat[1] == 3 && stat[2] == 3 && stat[3] == 3)
	    		System.out.println("All sensors calibrated!");
	    	return stat;
	    }
	    
	    /**
	     * Report the current calibration status.
	     * @return the 4 byte array for system status, gyro, accel, mag 0-3
	     * @throws IOException
	     */
	    public String formatCalibrationStatus(byte[] stat) throws IOException {
	    	StringBuilder sb = new StringBuilder();
	    	//byte[] stat = getCalibrationStatus();
	    	// system, gyro, accel, mag
	    	if( stat[0] != 3 ) {
	    		sb.append("System status is less than full calibration at:");
	    		sb.append(stat[0]);
	    		sb.append(", results will be ");
	    		sb.append(stat[0] == 0 ? "UNUSABLE" : stat[0] == 1 ? "INACCURATE" : stat[0] == 2 ? "MARGINAL" : "UNKNOWN");
	    		sb.append("\r\n");
	    	}
	    	if( stat[1] != 3 ) {
	    		sb.append("Gyro calibration less than optimal at:");
	    		sb.append(stat[1]);
	    		sb.append(", results will be ");
	    		sb.append(stat[1] == 0 ? "UNUSABLE" : stat[1] == 1 ? "INACCURATE" : stat[1] == 2 ? "MARGINAL" : "UNKNOWN");
	    		sb.append("\r\n");
	    	}
	    	if( stat[2] != 3 ) {
	    		sb.append("Accelerometer calibration less than optimal at:");
	    		sb.append(stat[2]);
	    		sb.append(", results will be ");
	    		sb.append(stat[2] == 0 ? "UNUSABLE" : stat[2] == 1 ? "INACCURATE" : stat[2] == 2 ? "MARGINAL" : "UNKNOWN");
	    		sb.append("\r\n");
	    	}
	    	if( stat[3] != 3 ) {
	    		sb.append("Magnetometer calibration less than optimal at:");
	    		sb.append(stat[3]);
	    		sb.append(", results will be ");
	    		sb.append(stat[3] == 0 ? "UNUSABLE" : stat[3] == 1 ? "INACCURATE" : stat[3] == 2 ? "MARGINAL" : "UNKNOWN");
	    		sb.append("\r\n");
	    	}
	    	if( stat[0] == 3 && stat[1] == 3 && stat[2] == 3 && stat[3] == 3) {
	    		sb.append("All sensors calibrated!");
	    		sb.append("\r\n");
	    	}
	    	return sb.toString();
	    }
	    /**
	     * Perform interactive calibration. The steps are:
	     * 1.) For gyro leave flat for a few seconds, this reading usually self corrects immediately
	     * 2.) For accel, rotate the sensor through increments of 15 degrees roll from the flat position, pausing for a few seconds between each
	     * 3.) For mag, wave the sensor through a figure 8 rotation of 8 to 12 inches until stable. This reading is the most tedious.
	     * Once all elements reach 3 a write is performed storing a file for next reset and the method exits.
	     * @throws IOException
	     */
	    public String calibrate(byte[] stat) throws IOException {
	    	StringBuilder sb = new StringBuilder();
	    	//byte[] stat = reportCalibrationStatus();
    		sb.append("When status reaches target, the message:<< CALIBRATION ACHIEVED! >> and file "+CALIBRATION_FILE+" will appear and process is complete");
    		sb.append("\r\n");
    		sb.append("1.) For gyro leave flat for a few seconds, this reading usually self corrects immediately.");
    		sb.append("\r\n");
    		sb.append("2.) For accel, rotate the sensor through increments of 15 degrees. Roll from the flat position, pausing for a few seconds between each rotation.");
    		sb.append("\r\n");
    		sb.append("3.) For mag, wave the sensor through a figure 8 rotation of 8 to 12 inches until stable.");
    		sb.append("\r\n");
    		sb.append("Once all elements reach tolerance, a write is performed storing a file for next reset and the method exits!");
    		sb.append("\r\n");
	    	if( stat[0] != SYSTEM_CAL || stat[1] != GYRO_CAL || stat[2] != ACC_CAL || stat[3] != MAG_CAL ){
	    		sb.append("PERFORM CALIBRATION KATA NOW!");
	    		sb.append("\r\n");
	    		sb.append("When all status reaches target, message appears, file is written, and process completes");
	    		sb.append("\r\n");
	    		sb.append("1.) For gyro leave flat for a few seconds, reading self corrects");
	    		sb.append("\r\n");
	    		sb.append("2.) For accel, rotate the sensor through increments of 15 degrees from the flat position.");
	    		sb.append("\r\n");
	    		sb.append("3.) For mag, wave the sensor through a figure 8 rotation..");
	    		sb.append("\r\n");
	    		sb.append("-v-----v-----v-----v-----v-----v-----v-----v-----v-----v---");
	    		sb.append("\r\n");
	    		sb.append("!!! SYSTEM status target:"+SYSTEM_CAL+" CURRENT:"+stat[0]);
	    		sb.append("\r\n");
	    		sb.append("!!!1.) GYRO status target:"+GYRO_CAL+" CURRENT:"+stat[1]);
	    		sb.append("\r\n");
	    		sb.append("!!!2.) ACCEL status target:"+ACC_CAL+" CURRENT:"+stat[2]);
	    		sb.append("\r\n");
	    		sb.append("!!!3.) MAG status target:"+MAG_CAL+" CURRENT:"+stat[3]);
	    		sb.append("\r\n");
	    		sb.append("-^-----^-----^-----^-----^-----^-----^-----^-----^-----^---");
	    		sb.append("\r\n");
	    	} else {
	    		// loop exits when all status reaches 3, then we write after retrieving calibration bytes
	    		sb.append("<< CALIBRATION ACHIEVED! >>");
	    		sb.append("\r\n");
	    		set_mode(OPERATION_MODE_CONFIG);
	    		writeCalibration(getCalibration());
	    		setNormalPowerNDOFMode();
	    	}
	    	return sb.toString();
	    }
	    
	    /**
	     * Return a tuple with the axis remap register values.  This will return
	     * 6 values with the following meaning:
	     * - X axis remap (a value of AXIS_REMAP_X, AXIS_REMAP_Y, or AXIS_REMAP_Z.
	     *                   which indicates that the physical X axis of the chip
	     *                   is remapped to a different axis)
	     *    - Y axis remap (see above)
	     *    - Z axis remap (see above)
	     *    - X axis sign (a value of AXIS_REMAP_POSITIVE or AXIS_REMAP_NEGATIVE
	     *                   which indicates if the X axis values should be positive/
	     *                   normal or negative/inverted.  The default is positive.)
	     *    - Y axis sign (see above)
	     *    - Z axis sign (see above)
	     *  Note that by default the axis orientation of the BNO chip looks like
	     *  the following (taken from section 3.4, page 24 of the datasheet).  Notice
	     *  the dot in the corner that corresponds to the dot on the BNO chip:
	     *                     | Z axis
	     *                     |
	     *                     |   / X axis
	     *                 ____|__/____
	     *    Y axis     / *   | /    /|
	     *    _________ /______|/    //
	     *             /___________ //
	     *            |____________|/
	     * @throws IOException 
	     */
	     private byte[] getAxisRemap() throws IOException {
	        // Get the axis remap register value.
	        byte[] mapConfig = read(BNO055_AXIS_MAP_CONFIG_ADDR, (byte)1);
	        byte z = (byte) ((mapConfig[0] >> 4) & (byte)0x03);
	        byte y = (byte) ((mapConfig[0] >> 2) & (byte)0x03);
	        byte x = (byte) (mapConfig[0] & (byte)0x03);
	        // Get the axis remap sign register value.
	        byte[] signConfig = read(BNO055_AXIS_MAP_SIGN_ADDR, (byte)1);
	        byte x_sign = (byte) ((signConfig[0] >> 2) & (byte)0x01);
	        byte y_sign = (byte) ((signConfig[0] >> 1) & (byte)0x01);
	        byte z_sign = (byte) (signConfig[0] & (byte)0x01);
	        // Return the results as a tuple of all 3 values.
	        return new  byte[]{x, y, z, x_sign, y_sign, z_sign};
	     }
	    /**
	     *  Set axis remap for each axis.  The x, y, z parameter values should
	     *  be set to one of AXIS_REMAP_X, AXIS_REMAP_Y, or AXIS_REMAP_Z and will
	     *  change the BNO's axis to represent another axis.  Note that two axis
	     *  cannot be mapped to the same axis, so the x, y, z params should be a
	     *  unique combination of AXIS_REMAP_X, AXIS_REMAP_Y, AXIS_REMAP_Z values.
	     *  The x_sign, y_sign, z_sign values represent if the axis should be positive
	     *  or negative (inverted).
	     *  See the get_axis_remap documentation for information on the orientation
	     *  of the axis on the chip, and consult section 3.4 of the datasheet.
	     * @throws IOException 
	     */
	     private void setAxisRemap(byte x, byte y, byte z, byte x_sign, byte y_sign, byte z_sign) throws IOException {    
	        // Switch to configuration mode.
	    	set_mode(OPERATION_MODE_CONFIG);
	        // Set the axis remap register value.
	        byte map_config = (byte)0x00;
	        map_config |= (z & (byte)0x03) << 4;
	        map_config |= (y & (byte)0x03) << 2;
	        map_config |= x & (byte)0x03;
	        while(write(BNO055_AXIS_MAP_CONFIG_ADDR,new byte[]{ map_config}, true)) {
	          	 try {
						Thread.sleep(100);
					} catch (InterruptedException e) {}
	        }
	        // Set the axis remap sign register value.
	        byte sign_config = (byte)0x00;
	        sign_config |= (x_sign & (byte)0x01) << 2;
	        sign_config |= (y_sign & (byte)0x01) << 1;
	        sign_config |= z_sign & (byte)0x01;
	        while(write(BNO055_AXIS_MAP_SIGN_ADDR, new byte[]{sign_config}, true)) {
	          	 try {
						Thread.sleep(10);
				} catch (InterruptedException e) {}
	        }
	        // Go back to normal operation mode.
	        setNormalPowerNDOFMode();
	     }
	    /**
	     * 	Set axis remap for each axis.  The x, y, z parameter values should
	     *  be set to one of AXIS_REMAP_X, AXIS_REMAP_Y, or AXIS_REMAP_Z and will
	     *  change the BNO's axis to represent another axis.  Note that two axis
	     *  cannot be mapped to the same axis, so the x, y, z params should be a
	     *  unique combination of AXIS_REMAP_X, AXIS_REMAP_Y, AXIS_REMAP_Z values.
	     *  In this overload, the axis default to all positive.
	     * @param x
	     * @param y
	     * @param z
	     * @throws IOException
	     */
	    private void setAxisRemap(byte x, byte y, byte z) throws IOException {
	    	setAxisRemap(x, y, z, AXIS_REMAP_POSITIVE, AXIS_REMAP_POSITIVE, AXIS_REMAP_POSITIVE);
	    }
	    
	    /**
	     *     Return a tuple with status information.  Three values will be returned:
	     *     - System status register value with the following meaning:
	     *         0 = Idle
	     *         1 = System Error
	     *         2 = Initializing Peripherals
	     *         3 = System Initialization
	     *         4 = Executing Self-Test
	     *         5 = Sensor fusion algorithm running
	     *         6 = System running without fusion algorithms
	     *     - Self test result register value with the following meaning:
	     *         Bit value: 1 = test passed, 0 = test failed
	     *         Bit 0 = Accelerometer self test
	     *         Bit 1 = Magnetometer self test
	     *         Bit 2 = Gyroscope self test
	     *         Bit 3 = MCU self test
	     *         Value of 0x0F = all good!
	     *     - System error register value with the following meaning:
	     *         0 = No error
	     *         1 = Peripheral initialization error
	     *         2 = System initialization error
	     *         3 = Self test result failed
	     *         4 = Register map value out of range
	     *         5 = Register map address out of range
	     *         6 = Register map write error
	     *         7 = BNO low power mode not available for selected operation mode
	     *         8 = Accelerometer power mode not available
	     *         9 = Fusion algorithm configuration error
	     *        10 = Sensor configuration error
	     *   If run_self_test is passed in as False then no self test is performed and
	     *   None will be returned for the self test result.  Note that running a
	     *   self test requires going into config mode which will stop the fusion
	     *   engine from running.
	     * @return The three byte tuple of status, self test, error where self-test is 0 for fail or not run 
	     * @param run_self_test true to execute self test
	     * @throws IOException 
	     */
	    public byte[] getSystemStatus(boolean run_self_test) throws IOException {
	    	byte[] selfTest = new byte[1];
	    	byte[] status = new byte[1];
	    	byte[] error = new byte[1];
	        if(run_self_test) {
	            // Switch to configuration mode if running self test.
	        	set_mode(OPERATION_MODE_CONFIG);
	            // Perform a self test.
	            byte[] sysTrigger = read(BNO055_SYS_TRIGGER_ADDR, (byte)1);
	            sysTrigger[0] |= (byte)0x1;
	            while(write(BNO055_SYS_TRIGGER_ADDR, sysTrigger, true)) {
	            	try {
						Thread.sleep(100);
					} catch (InterruptedException e) {}
	            }
	            // Wait for self test to finish.
	            try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {}
	            // Read test result.
	            selfTest = read(BNO055_SELFTEST_RESULT_ADDR, (byte)1);
	            // Go back to operation mode.
	            setNormalPowerNDOFMode();
	        }
	        // Now read status and error registers.
	        status = read(BNO055_SYS_STAT_ADDR, (byte)1);
	        error = read(BNO055_SYS_ERR_ADDR, (byte)1);
	        // Return the results as a tuple of all 3 values.
	        return new byte[]{status[0], selfTest[0], error[0]};
	    }
	    public byte[] getSystemStatus() throws IOException { return getSystemStatus(true); }
	    /**
	     * Display status from test.
	     * @param sysStat the 3 byte tuple from getSystemStatus()
	     */
	    public String displaySystemStatus(byte[] sysStat) {
	    	StringBuilder sb = new StringBuilder("System status:");
	    	switch(sysStat[0]) {
	    	case(0): 
	    		sb.append("Idle\r\n");
	    		break;
	    	case(1):
	    		sb.append("System Error\r\n");
	    		break;
	    	case(2):
	    		sb.append("Initializing Peripherals\r\n");
	    		break;       
	    	case(3):
	    		sb.append("System Initialization\r\n");
	    		break;
	    	case(4):
	    		sb.append("Executing Self-Test\r\n");
	    		break;
	    	case (5):
	    		sb.append("Sensor fusion algorithm running\r\n");
	    		break;
	    	case (6):
	    		sb.append("System running without fusion algorithms\r\n");
	    		break;
	    	default:
	    		sb.append("Unknown system status ");
	    		sb.append(sysStat[0]);
	    		sb.append("\r\n");
	    		break;
	    	}
	    	// self test
	    	sb.append("Self test:\r\n");
	    	//Bit 0 = Accelerometer self test
	    	sb.append(" Accel:");
	    	sb.append((sysStat[1] & (byte)0x01) > 0 ? "PASS" : "FAIL or not run");
	    	//Bit 1 = Magnetometer self test
	    	sb.append(" Mag:"+((sysStat[1] & (byte)0x02) > 0 ? "PASS" : "FAIL or not run"));
	    	//Bit 2 = Gyroscope self test
	    	sb.append(" Gyro:"+((sysStat[1] & (byte)0x04) > 0 ? "PASS" : "FAIL or not run"));
	    	//Bit 3 = MCU self test
	    	sb.append(" MCU:");
	    	sb.append((sysStat[1] & (byte)0x08) > 0 ? "PASS" : "FAIL or not run");
	    	sb.append("\r\n");
	    	// system error
	    	sb.append("System error:");
	    	switch(sysStat[2]) {
	    	case(0): 
	    		sb.append("NO ERROR\r\n");
	    		break;
	    	case(1):
	    		sb.append("Peripheral initialization error\r\n");
	    		break;
	    	case(2):
	    		sb.append("System initialization error\r\n");
	    		break;       
	    	case(3):
	    		sb.append("Self test result failed\r\n");
	    		break;
	    	case(4):
	    		sb.append("Register map value out of range\r\n");
	    		break;
	    	case (5):
	    		sb.append("Register map address out of range\r\n");
	    		break;
	    	case (6):
	    		sb.append("Register map write error\r\n");
	    		break;
	    	case (7):
	    		sb.append("BNO low power mode not available for selected operation mode\r\n");
	    		break;
	    	case (8):
	    		sb.append("Accelerometer power mode not available\r\n");
	    		break;
	    	case (9):
	    		sb.append("Fusion algorithm configuration error\r\n");
	    		break;
	    	case (10):
	    		sb.append("Sensor configuration error\r\n");
	    		break;
	    	default:
	    		sb.append("Unknown system error value ");
	    		sb.append(sysStat[2]);
	    		sb.append("\r\n");
	    		break;
	    	}
	    	return sb.toString();
	    }
	    
	    /**
	     *  Return a tuple with revision information about the BNO055 chip.  Will
	     *   return 5 values:
	     *   - Software revision
	     *   - Bootloader version
	     *   - Accelerometer ID
	     *   - Magnetometer ID
	     *   - Gyro ID
	     * @return
	     * @throws IOException 
	     */
	    public byte[] getRevision() throws IOException {  
	        // Read revision values.
	        byte[] accel = read(BNO055_ACCEL_REV_ID_ADDR, (byte)1);
	        byte[] mag = read(BNO055_MAG_REV_ID_ADDR, (byte)1);
	        byte[] gyro = read(BNO055_GYRO_REV_ID_ADDR, (byte)1);
	        byte[] bl = read(BNO055_BL_REV_ID_ADDR, (byte)1);
	        byte[] sw_lsb = read(BNO055_SW_REV_ID_LSB_ADDR, (byte)1);
	        byte[] sw_msb = read(BNO055_SW_REV_ID_MSB_ADDR, (byte)1);
	        byte sw = (byte) (((sw_msb[0] << 8) | sw_lsb[0]) & 0xFFFF);
	        // Return the results as a tuple of all 5 values.
	        return new byte[]{sw, bl[0], accel[0], mag[0], gyro[0]};
	    }
	    
	    public String displayRevision(byte[] revs) {
	    	StringBuilder sb = new StringBuilder();
	    	sb.append("BNO055 Software rev.:");
	    	sb.append(Integer.valueOf(revs[0] & 0xff));
	    	sb.append(" bootloader:");
	    	sb.append(Integer.valueOf(revs[1] & 0xff));
	    	sb.append(" Accel Id:");
	    	sb.append(Integer.valueOf(revs[2] & 0xff));
	    	sb.append(" Mag Id:");
	    	sb.append(Integer.valueOf(revs[3] & 0xff));
	    	sb.append(" Gyro Id:");
	    	sb.append(Integer.valueOf(revs[4] & 0xff));
	    	return sb.toString();
	    }
        /**
         * Set if an external crystal is being used by passing True, otherwise
         * use the internal oscillator by passing False (the default behavior).
         * @throws IOException 
         */
	    public void setExternalCrystal(boolean external_crystal) throws IOException {
	        // Switch to configuration mode.
	    	set_mode(OPERATION_MODE_CONFIG);
	        // Set the clock bit appropriately in the SYS_TRIGGER register.
	        if(external_crystal) {
	            write(BNO055_SYS_TRIGGER_ADDR, new byte[]{(byte)0x80}, true);
	        } else {
	            write(BNO055_SYS_TRIGGER_ADDR, new byte[]{0x00}, true);
	        }
	        // Go back to normal operation mode.
	        setNormalPowerNDOFMode();
	    }
	        
	    
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
	    	StringBuilder sb = new StringBuilder("IMUSerialDataPort\n");
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

			@Override
			public String readLine() {
				throw new RuntimeException("refactor! not applicable method");
			}

			@Override
			public void writeLine(String output) throws IOException {
				throw new RuntimeException("refactor! not applicable method");
				
			}
	        
}
