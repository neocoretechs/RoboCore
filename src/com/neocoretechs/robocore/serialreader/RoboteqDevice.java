package com.neocoretechs.robocore.serialreader;

import java.io.IOException;

public class RoboteqDevice extends AbstractSmartMotorControl {

	static final int ROBOTEQ_DEFAULT_TIMEOUT  =   1000;
	static final int ROBOTEQ_BUFFER_SIZE      =   64;
	static final int ROBOTEQ_COMMAND_BUFFER_SIZE =20;

	static final int ROBOTEQ_QUERY_CHAR        =  0x05;
	static final int ROBOTEQ_ACK_CHAR          =  0x06;

	static final int ROBOTEQ_OK                =  0;
	static final int ROBOTEQ_TIMEOUT           =  -1;
	static final int ROBOTEQ_ERROR              = -2;
	static final int ROBOTEQ_BAD_COMMAND       =  -3;
	static final int ROBOTEQ_BAD_RESPONSE      =  -4;
	static final int ROBOTEQ_BUFFER_OVER       =  -5;

	static final int ROBOTEQ_FAULT_OVERHEAT    =  0x01;
	static final int ROBOTEQ_FAULT_OVERVOLTAGE =  0x02;
	static final int ROBOTEQ_FAULT_UNDERVOLTAGE=  0x04;
	static final int ROBOTEQ_FAULT_SHORT       =  0x08;
	static final int ROBOTEQ_FAULT_ESTOP       =  0x10;
	static final int ROBOTEQ_FAULT_SCRIPT      =  0x20;
	static final int ROBOTEQ_FAULT_MOSFET      =  0x40;
	static final int ROBOTEQ_FAULT_CONFIG      =  0x80;

	static final int ROBOTEQ_STATUS_SERIAL     =  0x01;
	static final int ROBOTEQ_STATUS_PULSE      =  0x02;
	static final int ROBOTEQ_STATUS_ANALOG     =  0x04;
	static final int ROBOTEQ_STATUS_POWER_OFF  =  0x08;
	static final int ROBOTEQ_STATUS_STALL      =  0x10;
	static final int ROBOTEQ_STATUS_LIMIT      =  0x20;
	static final int ROBOTEQ_SCRIPT_RUN        =  0x80;
	
	private ByteSerialDataPort m_Serial;
	private int m_Timeout;
	private String command;// = new char[ROBOTEQ_COMMAND_BUFFER_SIZE];

	
	public RoboteqDevice(int maxPower) throws IOException {
		super(1000);
		m_Timeout = ROBOTEQ_DEFAULT_TIMEOUT;
		setChannels(2);
		m_Serial = new ByteSerialDataPort("/dev/ttyS2",115200, 8, 1, 0);
		m_Serial.connect(true);
	}
		
	@Override
    /**
     * send motor power command (!G)
     *
     * @param ch channel
     * @param p power level (-1000, 1000)
     * @return ROBOTEQ_OK if successful 
     */
	public int commandMotorPower(int ch, int p) throws IOException {
		/*
		* Command the motor to spin. May reset current direction. Encoders reset regardless if present. Checks for ultrasonic shutdown if present.
		* ch - channel. max is controller dependent
		* p - power level -1000 to 10000
		*/
		// check shutdown override
		if( MOTORSHUTDOWN )
			return ROBOTEQ_OK;
		if( ch < 1 || ch > getChannels() )
			return ROBOTEQ_BAD_COMMAND;
		// add offset for min power, 0 is default if not set by M5
		//if( p != 0 )
		//	p += minMotorPower[ch-1];
		//if( p > MAXMOTORPOWER ) // cap it at max
		//	p = MAXMOTORPOWER;
		//if( MOTORPOWERSCALE != 0 )
		//	p /= MOTORPOWERSCALE;
		motorSpeed[ch-1] = p;
			
		if( p < 0 )  // and we want to go backward
				currentDirection[ch-1] = 0; // set new direction value
		else // dir is 1 forward
				currentDirection[ch-1] = 1;		
		// New command resets encoder count
		resetEncoders();
		// if power 0, we are stopping anyway
		if( checkUltrasonicShutdown() )
			return ROBOTEQ_OK;
		// If this wheel is mirrored, invert power
		if( defaultDirection[ch-1] != 0)
			p = -p;
		if( p != 0 && Math.abs(p) < motorSpeed[ch-1]) {
			if(p > 0 )
				p = minMotorPower[ch-1];
			else
				p = -minMotorPower[ch-1];
		}
		if( Math.abs(p) > MAXMOTORPOWER ) { // cap it at max
			if(p > 0)
				p = MAXMOTORPOWER;
			else
				p = -MAXMOTORPOWER;
		}
		// Scale motor power if necessary and save it in channel speed array with proper sign for later use
		if( MOTORPOWERSCALE != 0 ) {
			p /= MOTORPOWERSCALE;
		}
		command = String.format("!G %02d %d\r", ch, p);
		fault_flag = 0;
		return sendCommand(command);
	}

	private int sendCommand(String commandx) {
		if (this.m_Serial == null)
			return ROBOTEQ_ERROR;
		if(commandx.length() <= 0 || commandx.length() >= ROBOTEQ_COMMAND_BUFFER_SIZE){
			return ROBOTEQ_BAD_COMMAND;
		}	
		for(int i = 0 ; i < commandx.length(); i++) {
			try {
				this.m_Serial.write(commandx.charAt(i));
				this.m_Serial.read(); // absorb echo
			//this.m_Serial.flush();
				Thread.sleep(1);
			} catch (InterruptedException | IOException e) {
				e.printStackTrace();
			}
		}
		//this->m_Serial->flush();
		char[] buffer = new char[ROBOTEQ_BUFFER_SIZE];
		int res = this.readResponse(buffer);

		if (res < 1)
			return ROBOTEQ_BAD_RESPONSE;

		// Check Command Status
		if (buffer[0] == '+') {
			return ROBOTEQ_OK;
		} else {
			return ROBOTEQ_BAD_COMMAND;
		}
	}
	
	private int readResponse(char[] buf) {
		int inByte;
		int index = 0;
		try {
		for(int i = 0; i < 10; i++) {
			if (m_Serial.bytesToRead() > 0) {
				while(m_Serial.bytesToRead() > 0) {
					inByte = m_Serial.read();
					buf[index++] = (char) inByte;
					if (index > buf.length) {
						// out of buffer space
						return ROBOTEQ_BUFFER_OVER;
					}
					if (inByte == 0x0D) {
						return index;
					}
				}
			}
			Thread.sleep(10);
		}
		} catch (IOException | InterruptedException e) {
			e.printStackTrace();
		}
		// timeout
		return ROBOTEQ_TIMEOUT;
	}


	private int sendQuery(String commandx, char[] response) {
		try {
			for(int i = 0; i < commandx.length(); i++) {
				m_Serial.write(commandx.charAt(i));
				m_Serial.read();
			}
		} catch (IOException e) {
			e.printStackTrace();
			return -1;
		}
		return readResponse(response);
	}

	@Override
    /**
     * send emergency stop command (!EX)
     * note: you have to reset the controller after this sending command
     *
     * @return ROBOTEQ_OK if successful
     */
	public int commandEmergencyStop(int status) throws IOException {
		//sprintf(command, "!EX\r");
		for(int ch = 0; ch < getChannels(); ch++) {
			command = String.format("!G %02d %d\r", ch+1, 0);
			this.sendCommand(command);
		}
		fault_flag = 16;
		resetSpeeds();
		resetEncoders();
		return status;
	}
	
	int commandReset() {
		command = "%RESET 321654987\r";
		return this.sendCommand(command);
	}

	int commandBrushlessCounter() {
		command = "^BLFB 0\r";
		return this.sendCommand(command);
	}
	
	@Override
    /**
     * check if controller is connected
     *
     * @return ROBOTEQ_OK if connected
     * @throws IOException 
     */
	public int isConnected() throws IOException {
		if (this.m_Serial == null)
			return 0;
		m_Serial.write(ROBOTEQ_QUERY_CHAR);
		//m_Serial.flush();
		int inByte;
		for(int i = 0; i < 10; i++) {
			if (m_Serial.bytesToRead() > 0) {
				while(m_Serial.bytesToRead() > 0) {
					inByte = m_Serial.read();
					if (inByte == ROBOTEQ_ACK_CHAR) {
							return 1;
					}
				}
			}
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		// timeout
		return 0;
	}

	@Override
	public String getDriverInfo(int ch) {
		try {
			if( isConnected() == 0) {
				return String.format("Roboteq smart Controller channel %d is not connected.%n", ch);
			}
		} catch (IOException e) {
			e.printStackTrace();
			return String.format("Roboteq smart Controller channel %d threw exception%s.%n", ch, e.getMessage());
		}
		return String.format("Roboteq smart Controller Voltage:%s Amps:%s Fault:%s Status:%s%n",queryBatteryVoltage() , queryBatteryAmps(), queryFaultFlag(), queryStatusFlag());
	}

	@Override
	/**
	*f1 = overheat
	*f2 = overvoltage
	*f3 = undervoltage
	*f4 = short circuit
	*f5 = emergency stop
	*f6 = Sepex excitation fault
	*f7 = MOSFET failure
	*f8 = startup configuration fault
	*/
	public int queryFaultFlag() {
		// Query: ?FF
		// Response: FF=<status>
		int fault = -1;
		//memset(buffer, NULL, ROBOTEQ_BUFFER_SIZE);
		char[] buffer = new char[ROBOTEQ_BUFFER_SIZE];
		int res = 0;
		if((res = this.sendQuery("?FF\r", buffer)) < 0)
			return res;
		if (res < 4)
			return ROBOTEQ_BAD_RESPONSE;
		// Parse Response
		fault = buffer[4] - 48;
		if (fault < 1)
			return ROBOTEQ_BAD_RESPONSE;
		return fault | fault_flag;
	}

	@Override
	/**
	*f1 = Serial mode
	*f2 = Pulse mode
	*f3 = Analog mode
	*f4 = Power stage off
	*f5 = Stall detected
	*f6 = At limit
	*f7 = Unused
	*f8 = MicroBasic script running
	*/
	public int queryStatusFlag() {
		// Query: ?FS
		// Response: FS=<status>
		int status = -1;
		//memset(buffer, NULL, ROBOTEQ_BUFFER_SIZE);
		char[] buffer = new char[ROBOTEQ_BUFFER_SIZE];
		int res;
		if ((res = this.sendQuery("?FS\r", buffer)) < 0)
			return res;
		if (res < 4)
			return ROBOTEQ_BAD_RESPONSE;
		// Parse Response
		status = buffer[4] - 48;
		//if (sscanf((char*)buffer, "FS=%i", &status) < 1)
		if(status < 1)
			return ROBOTEQ_BAD_RESPONSE;
		return status;
	}
	
	int queryFirmware() {
		// Query: ?FID
		// Response: FID=<firmware>
		//memset(buf, NULL, bufSize);
		char[] buf = new char[100];
		return this.sendQuery("?FID\r",buf);
		// TODO: Parse response
	}
	/**
	 * Query: ?M [ch]
	 * Response: M=<motor power>
	 * @param ch channel
	 * @return
	 */
	int queryMotorPower(int ch) {
		int p;
		int res;
		// Build Query Command
		command = String.format("?M %d\r", ch);
		char[] buffer = new char[ROBOTEQ_BUFFER_SIZE];
		// Send Query
		if ((res = this.sendQuery("?BA\r", buffer)) < 0)
			return res;
		if (res < 4)
			return ROBOTEQ_BAD_RESPONSE;
		String x = new String(buffer);
		x = x.substring(2);
		p = Integer.parseInt(x);
		// Parse Response
		//if (sscanf((char*)buffer, "M=%i", &p) < 1) {
		if( p < 0) {
			return ROBOTEQ_BAD_RESPONSE;
		}
		return p;
	}
	/**
	* Query: ?BA
	* Response: BA=<ch1*10>:<ch2*10>
	* @return total amperage at both channels totaled
	*/
	int queryBatteryAmps() {
		int ch1, ch2;
		int res;
		// Send Query
		char[] buffer = new char[ROBOTEQ_BUFFER_SIZE];
		if ((res = this.sendQuery("?BA\r", buffer)) < 0)
			return res;
		if (res < 4)
			return ROBOTEQ_BAD_RESPONSE;
		String x = new String(buffer);
		x = x.substring(3);
		String[] amps = x.split(":");
		// Parse Response
		//if (sscanf((char*)buffer, "BA=%i:%i", &ch1, &ch2) < 2) {
		if(amps.length < 2) {
			return ROBOTEQ_BAD_RESPONSE;
		}
		ch1 = Integer.parseInt(amps[0]);
		ch2 = Integer.parseInt(amps[1]);
		// Return total amps (ch1 + ch2)
		return ch1+ch2;
	}
	/**
	 * Query: ?BA [ch]
	 * Response: BA=<ch*10>
	 */
	int queryBatteryAmps(int ch) {
		int amps;
		int res;
		// Build Query Command
		char[] buffer = new char[ROBOTEQ_BUFFER_SIZE];
		command = String.format("?BA %d\r", ch);
		if ((res = this.sendQuery(command, buffer)) < 0)
			return res;
		if (res < 4)
			return ROBOTEQ_BAD_RESPONSE;
		String x = new String(buffer);
		x = x.substring(3);
		amps = Integer.parseInt(x);
		// Parse Response
		if(amps < 0) {
			return ROBOTEQ_BAD_RESPONSE;
		}
		return amps;
	}
	/**
	 * 	Query: ?V 2 (2 = main battery voltage)
	 * Response: V=<voltage>*10
	 * @return
	 */
	int queryBatteryVoltage() {
		int voltage = -1;
		//memset(buffer, NULL, ROBOTEQ_BUFFER_SIZE);
		char[] buffer = new char[ROBOTEQ_BUFFER_SIZE];
		int res;
		if ((res = this.sendQuery("?V 2\r", buffer)) < 0)
			return res;
		if (res < 4)
			return ROBOTEQ_BAD_RESPONSE;
		String x = new String(buffer);
		x = x.substring(2);
		voltage = Integer.parseInt(x);
		// Parse Response
		if(voltage < 0) {
			return ROBOTEQ_BAD_RESPONSE;
		}
		//if (sscanf((char*)buffer, "V=%i", &voltage) != 1)
		return voltage;
	}
	/**
	* Query: ?S [ch]
	* Response: S=[speed]
	* Encoder speed in RPM
	* To report RPM accurately, the correct Pulses per Revolution (PPR) must be stored in the encoder configuration.
	*/
	int queryEncoderSpeed(int ch){
		int speed;
		int res;
		char[] buffer = new char[ROBOTEQ_BUFFER_SIZE];
		// Build Query Command
		command = String.format("?S %d\r", ch);
		if ((res = this.sendQuery(command, buffer)) < 0)
			return res;
		if (res < 3)
			return ROBOTEQ_BAD_RESPONSE;
		String x = new String(buffer);
		x = x.substring(2);
		speed = Integer.parseInt(x);
		//if (sscanf((char*)buffer, "S=%i", &speed) < 1) {
		if(speed < 0) {
			return ROBOTEQ_BAD_RESPONSE;
		}
		return speed;
	}
	/**
	* Query: ?SR [ch]
	* Response: SR=[speed]
	* Returns the measured motor speed as a ratio of the Max RPM configuration parameter 
	*/
	int queryEncoderRelativeSpeed(int ch) {
		int speed;
		int res;
		char[] buffer = new char[ROBOTEQ_BUFFER_SIZE];
		// Build Query Command
		command = String.format("?SR %d\r", ch);
		if ((res = this.sendQuery(command, buffer)) < 0)
			return res;
		if (res < 3)
			return ROBOTEQ_BAD_RESPONSE;
		String x = new String(buffer);
		x = x.substring(3);
		speed = Integer.parseInt(x);
		if (res < 3)
			return ROBOTEQ_BAD_RESPONSE;
		// Parse Response
		//if (sscanf((char*)buffer, "SR=%i", &speed) < 1) {
		if(speed < 1) {
			return ROBOTEQ_BAD_RESPONSE;
		}
		return speed;
	}
	/**
	* Query: ?CB [ch] or ?C ch
	* Response: CB=[speed]? or C=[speed]
	* On brushless motor controllers, returns the running total of Hall sensor transition value as
	* an absolute number. The counter is a 32-bit counter with a range of +/- 2000000000
	* counts.
	*/
	int queryBrushlessCounter(int ch) {
		int speed;
		int res;
		char[] buffer = new char[ROBOTEQ_BUFFER_SIZE];
		// Build Query Command
		command = String.format("?C %d\r", ch);
		if ((res = this.sendQuery(command, buffer)) < 0)
			return res;
		if (res < 3)
			return ROBOTEQ_BAD_RESPONSE;
		String x = new String(buffer);
		x = x.substring(2);
		speed = Integer.parseInt(x);
		// Parse Response
		//if (sscanf((char*)buffer, "C=%i", &speed) < 1) {
		if(speed < 1) {
			return ROBOTEQ_BAD_RESPONSE;
		}
		return speed;
	}
	/**
	* Query: ?CBR [ch] or ?CR
	* Response: CBR=[speed] or CR=[speed]?
	* On brushless motor controllers, returns the number of Hall sensor transition value that
	* have been measured from the last time this query was made.
	*/
	int queryBrushlessCounterRelative(int ch) {
		int speed;
		int res;
		char[] buffer = new char[ROBOTEQ_BUFFER_SIZE];
		// Build Query Command
		command = String.format("?CR %d\r", ch);
		if ((res = this.sendQuery(command, buffer)) < 0)
			return res;
		if (res < 3)
			return ROBOTEQ_BAD_RESPONSE;
		String x = new String(buffer);
		x = x.substring(3);
		speed = Integer.parseInt(x);
		// Parse Response
		//if (sscanf((char*)buffer, "CR=%i", &speed) < 1) {
		if(speed < 1) {
			return ROBOTEQ_BAD_RESPONSE;
		}
		return speed;
	}
	/**
	* Query: ?BS [ch]
	* Response: BS=[speed]
	* To report RPM accurately, the correct number of motor poles must be
	* loaded in the BLPOL configuration parameter.
	*/
	int queryBrushlessSpeed(int ch) {
		int speed;
		int res;
		char[] buffer = new char[ROBOTEQ_BUFFER_SIZE];
		// Build Query Command
		command = String.format("?BS %d\r", ch);
		if ((res = this.sendQuery(command, buffer)) < 0)
			return res;
		if (res < 3)
			return ROBOTEQ_BAD_RESPONSE;
		String x = new String(buffer);
		x = x.substring(3);
		speed = Integer.parseInt(x);
		// Parse Response
		if(speed < 1) {
			return ROBOTEQ_BAD_RESPONSE;
		}
		// Parse Response
		//if (sscanf((char*)buffer, "BS=%i", &speed) < 1) {
		return speed;
	}
	/**
	* Query: ?BSR [ch]
	* Response: BSR=[speed]
	* On brushless motor controllers, returns the measured motor speed as a ratio of the Max RPM configuration parameter
	*/
	int queryBrushlessSpeedRelative(int ch) {
		int speed;
		int res;
		char[] buffer = new char[ROBOTEQ_BUFFER_SIZE];
		// Build Query Command
		command = String.format("?BSR %d\r", ch);
		if ((res = this.sendQuery(command, buffer)) < 0)
			return res;
		if (res < 3)
			return ROBOTEQ_BAD_RESPONSE;
		String x = new String(buffer);
		x = x.substring(4);
		speed = Integer.parseInt(x);
		// Parse Response
		if(speed < 1) {
			return ROBOTEQ_BAD_RESPONSE;
		}
		// Parse Response
		//if (sscanf((char*)buffer, "BSR=%i", &speed) < 1) {
		return speed;
	}
	/**
	*  Query: ?TM
	* Response: TM=[time]
	* On RTC units, 32 bit seconds
	*/
	int queryTime() {
		int speed;
		int res;
		char[] buffer = new char[ROBOTEQ_BUFFER_SIZE];
		if ((res = this.sendQuery("?TM\r", buffer)) < 0)
			return res;
		if (res < 3)
			return ROBOTEQ_BAD_RESPONSE;
		String x = new String(buffer);
		x = x.substring(3);
		speed = Integer.parseInt(x);
		// Parse Response
		//if (sscanf((char*)buffer, "TM=%i", &speed) < 1) {
		if(speed < 1) {
			return ROBOTEQ_BAD_RESPONSE;
		}
		return speed;
	}
	
	int setEncoderPulsePerRotation(int ch, int ppr) {
		command = String.format("^EPPR %02d %d\r", ch, ppr);
		return this.sendCommand(command);
	}

	int getEncoderPulsePerRotation(int ch) {
		// TODO: not implmented
		return ROBOTEQ_OK;
	}

	int setMotorAmpLimit(int ch, int a){
		command = String.format("^ALIM %d %d", ch, a);
		return this.sendCommand(command);
	}

	int getMotorAmpLimit(int ch){
		// TODO: not implmented
		return ROBOTEQ_OK;
	}

	int loadConfiguration() {
		command = "%%EELD\r";
		return this.sendCommand(command);
	}

	int saveConfiguration() {
		command = "%%EESAV\r";
		return this.sendCommand(command);
	}
}
