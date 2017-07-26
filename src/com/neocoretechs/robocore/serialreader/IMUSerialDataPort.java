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
 * Uses the serial UART mode of the BNO055 Bosch 9 axis sensor fusion package and presents a series of methods to read
 * The accelerometer, gyro, magnetometer, fused Euler angle data, and temperature.
 * Copyright NeoCoreTechs 2017
 * @author jg
 *
 */
public class IMUSerialDataPort implements DataPortInterface {
		private static boolean DEBUG = true;
	    private SerialPort serialPort;
	    private OutputStream outStream;
	    private InputStream inStream;
		// serial settings
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
	 
	    private static int[] readBuffer = new int[256];
	    private static int[] writeBuffer = new int[256];
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
	        // set config mode
	    	set_mode(OPERATION_MODE_CONFIG);
	        // now read without kruft
	        write(BNO055_PAGE_ID_ADDR, new byte[]{(byte)0x00}, true);
	        readChipId();
	    	reset();
	    	setNormalPowerNDOFMode();
	        if( DEBUG ) 
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
	    	write((byte) 0xAA); // Start byte
	    	write((byte) 0x00);  // Write
	    	write((byte) (address & 0xFF));
	    	write((byte) (data.length & 0xFF));
	    	for(int i = 0; i < data.length; i++) {
	    			write((byte) (data[i] & 0xFF));
	    	}
	    	
	    	byte resp;
	    	resp = (byte)(read() & 0xFF);
	    	if( resp != (byte)0xEE ) {
	    			System.out.printf("Received unexpected response in write ACK: %02x while looking for ACK byte 'ee'\r\n", resp);
	    			return;
	    	}
	    	//if( DEBUG )
	    	//	System.out.println("Found ack header");
	    	// we always use ACK except for reset, but it seems to send back the ee
	        if( !ack ) {
		    		if( DEBUG )
		    			System.out.println("return from write without ACK..");
		    		return;
		    }
	        resp = (byte)( read() & 0xFF);
	    	if( resp != (byte)0x01 ) {
	    				System.out.printf("Error from write ACK:%02x\r\n",resp);
	    				return;
	    	}
	    	//if( DEBUG ) {
	    	//	System.out.println("ACK successful");
	    	//}
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
	    	if( writeBufferTail >= writeBuffer.length) {	
    				writeBufferTail = 0;
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
	    	synchronized( readMx) {
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
		    //	System.out.println("Did NOT receive expected 'bb' response in signalRead");
		    if( resp != (byte)0xEE ) {
		    	System.out.printf("Received unexpected response in signalRead: %02x while looking for error byte 'ee'\r\n", resp);
		    	return (byte)0x07; // lets call this bus overrun from chip
		    } else {
		    	resp = (byte)( read() & 0xFF);
		    	//if( DEBUG ) {
		    	//	System.out.println("Recovered error code from signalRead");
		    	//}
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
					Thread.sleep(5);
				} catch (InterruptedException e) {}
	    	}
	    	if( resp != 0 ) {
	    		System.out.println("Bad response for signalRead after " + retry +" retries, exiting read with response:"+resp);
	    		return null;
	    	}
	    	// Returning with 0 from signalRead means we found 0xBB
	    	// next byte is length, then data 1..n
	    	//if( Props.DEBUG ) System.out.println("readBufferHead="+readBufferHead+" readBufferTail="+readBufferTail+" = "+readBuffer[readBufferHead]);
	    	byte blen = (byte)( read() & 0xFF);
	    	//if( DEBUG )
	    	//	System.out.printf("Recovered length byte: %02x\r\n", blen);
	        // Read the returned bytes.	
	        if( blen == 0 || blen != length) {
	                System.out.printf("Received length byte: %d but read requested %d bytes.\r\n", blen, length);
	                return null;
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
	       write(BNO055_OPR_MODE_ADDR, new byte[]{(byte)(mode & (byte)0xFF)}, true);
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
	         write(BNO055_PWR_MODE_ADDR, new byte[]{(byte)POWER_MODE_NORMAL}, true);
	         // Default to internal oscillator.
	         write(BNO055_SYS_TRIGGER_ADDR, new byte[]{(byte)0x0}, true);
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
	        	System.out.printf("Recovered chip Id did NOT match expected byte!:%02x\r\n",bno_id[0]);
	        	close();
	            throw new IOException("BNO055 chip id did not match expected value");
	        }
	    }
	    
	    /**
	     * Read accelerometer
	     * @return int array of x,y,z accel values
	     * @throws IOException
	     */
	    public int[] readAccel() throws IOException {
	    	byte[] data = read(BNO055_ACCEL_DATA_X_LSB_ADDR, (byte)6);
	    	if( data == null )
	    		return null;
	    	// Convert the data
	    	int xAccl = ((data[1] & (byte)0xFF) * 256 + (byte)(data[0] & (byte)0xFF)) ;
	    	if(xAccl > 32767) {
	    		xAccl -= 65536;
	    	}	
	    	int yAccl = ((data[3] & (byte)0xFF) * 256 + (byte)(data[2] & (byte)0xFF)) ;
	    	if(yAccl > 32767) {
	    		yAccl -= 65536;
	    	}
	    	int zAccl = ((data[5] & (byte)0xFF) * 256 + (byte)(data[4] & (byte)0xFF)) ;
	    	if(zAccl > 32767) {
	    		zAccl -= 65536;
	    	}
	    	return new int[]{xAccl, yAccl, zAccl};
	    }
	    
	    /**
	     * Read the magnetometer sensor data
	     * @return x/Y/Z magnetic field strength of axis in microteslas
	     * @throws IOException
	     */
	    public int[] readMag() throws IOException {
	    	// Read 6 bytes of data from address 0x0E(14)
	    	// xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
	    	byte[] data = read(BNO055_MAG_DATA_X_LSB_ADDR, (byte)6);
	    	if( data == null )
	    		return null;
	    	// Convert the data
	    	int xMag = ((data[1] & (byte)0xFF) * 256 + (byte)(data[0] & (byte)0xFF));
	    	if(xMag > 32767) {
	    		xMag -= 65536;
	    	}	
	    	int yMag = ((data[3] & 0xFF) * 256 + (byte)(data[2] & (byte)0xFF)) ;
	    	if(yMag > 32767) {
	    		yMag -= 65536;
	    	}
	    	int zMag = ((data[5] & 0xFF) * 256 + (byte)(data[4] & (byte)0xFF)) ;
	    	if(zMag > 32767) {
	    		zMag -= 65536;
	    	}	
	    	return new int[]{xMag, yMag, zMag};
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
	    	if( data == null)
	    		return null;
	    	// Convert the data
	    	int xGyro = ((data[1] & (byte)0xFF) * 256 + (byte)(data[0] & (byte)0xFF)) ;
	    	if(xGyro > 32767) {
	    		xGyro -= 65536;
	    	}
	    	int yGyro = ((data[3] & (byte)0xFF) * 256 + (byte)(data[2] & (byte)0xFF)) ;
	    	if(yGyro > 32767) {
	    		yGyro -= 65536;
	    	}
	    	int zGyro = ((data[5] & (byte)0xFF) * 256 + (byte)(data[4] & (byte)0xFF)) ;
	    	if(zGyro > 32767) {
	    		zGyro -= 65536;
	    	}
	    	return new int[]{xGyro, yGyro, zGyro};
	    }
	    
	    /**
	     * Return euler orientation data from the sensor
	     * @return the Yaw/Pitch/Roll integer array, or null if read error from sensor
	     * @throws IOException
	     */
	    public double[] readEuler() throws IOException {
	        //Return the current absolute orientation as a tuple of heading, roll, and pitch euler angles in degrees.
	    	byte[] data = read(BNO055_EULER_H_LSB_ADDR, (byte)6);
	    	if( data == null )
	    		return null;
	        int heading = ((data[1] & (byte)0xFF) * 256 + (byte)(data[0] & (byte)0xFF));
	    	if(heading > 32767) {
	    		heading -= 65536;
	    	}
	       	int roll = ((data[3] & (byte)0xFF) * 256 + (byte)(data[2] & (byte)0xFF)) ;
	    	if(roll > 32767) {
	    		roll -= 65536;
	    	}
	    	int pitch = ((data[5] & (byte)0xFF) * 256 + (byte)(data[4] & (byte)0xFF)) ;
	    	if(pitch > 32767) {
	    		pitch -= 65536;
	    	}
	        return new double[]{heading/16.0, roll/16.0, pitch/16.0};

	    }
	    
	    /**
	     * Read temperature from sensor
	     * @return The +/- temperature in current setting/mode, or Integer.MAX_VALUE if error
	     * @throws IOException
	     */
	    public int readTemperature() throws IOException {
	    	byte[] data = read(BNO055_TEMP_ADDR, (byte)1);
	    	if( data == null )
	    		return Integer.MAX_VALUE;
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
	    	if( data == null )
	    		return null;
	        int w = ((data[1] & (byte)0xFF) * 256 + (byte)(data[0] & (byte)0xFF));
	    	if(w > 32767) {
	    		w -= 65536;
	    	}
	       	int x = ((data[3] & (byte)0xFF) * 256 + (byte)(data[2] & (byte)0xFF)) ;
	    	if(x > 32767) {
	    		x -= 65536;
	    	}
	    	int y = ((data[5] & (byte)0xFF) * 256 + (byte)(data[4] & (byte)0xFF)) ;
	    	if(y > 32767) {
	    		y -= 65536;
	    	}
	    	int z = ((data[7] & (byte)0xFF) * 256 + (byte)(data[6] & (byte)0xFF)) ;
	    	if(z > 32767) {
	    		z -= 65536;
	    	}	
	    	// Scale values, see 3.6.5.5 in the datasheet.
	    	double scale = (1.0 / (1<<14));
	    	return new double[]{x*scale, y*scale, z*scale, w*scale};
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


			@Override
			public String readLine() {
				throw new RuntimeException("refactor! not applicable method");
			}

			@Override
			public void writeLine(String output) throws IOException {
				throw new RuntimeException("refactor! not applicable method");
				
			}

	
	        
}
