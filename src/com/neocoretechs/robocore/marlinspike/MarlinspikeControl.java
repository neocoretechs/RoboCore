package com.neocoretechs.robocore.marlinspike;

import java.io.IOException;

import com.neocoretechs.robocore.config.Robot;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;

/**
 * The realtime control endpoint.<p/>
 * The 2 critical elements that must be provided to sustain the operations of this class are;
 * 1.) An {@code AsynchDemuxer} asynchronous demuxxer that can process the messages coming from 
 * the Marlinspike realtime subsystem.<p/>
 * 2.) A {@code DataportInterface} source of data to be demuxxed.
 * The data port also serves, when writable, to receive directives that initiate responses to be demuxxed.<p/>
 * This class talks to the serial drivers that communicate with the attached USB SBC  
 * that runs the embedded code that manages the motor controller in the Marlinspike realtime subsystem.<p/>
 * Alternatively, a MarlinspikeDataPort may substitute for an attached SBC. The MarlinspikeDataPort uses the
 * GPIO functions under Linux SBC and the header attached to the board serving the Ros node.<p/>
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2017,2020
 */
public class MarlinspikeControl implements MarlinspikeControlInterface {
	public static boolean DEBUG = false;
	//float yawIMURads; = twistInfo.imuTheta
	int yawTargetDegrees;
	int targetDistance;
	int targetTime; 
	float[] accelDeltas;
	int[] ranges;
	boolean init = true;
	Object mutex = new Object();
	AsynchDemuxer asynchDemuxer;

	/* Stop the robot if it hasn't received a movement command in this number of milliseconds */
	public static int AUTO_STOP_INTERVAL = 2000;
	long lastMotorCommand = AUTO_STOP_INTERVAL;
	/* The base and odometry frames */
	//String baseFrame = "/base_link";
	//String odomFrame = "/odom";
	
	protected static boolean moving = false; // is the base in motion?

	public MarlinspikeControl(AsynchDemuxer asynchDemuxer) { this.asynchDemuxer = asynchDemuxer; }
	
	/**
	 * Single channel device control
	 * @param deviceName The deviceName as it appears in the configuration 
	 * @param deviceLevel the value to set the level of the device
	 * @throws IOException
	 */
	public synchronized void setDeviceLevel(String deviceName, int deviceLevel) throws IOException {
		if(DEBUG) 
			System.out.println(this.getClass().getName()+" Device:"+deviceName+" deviceLevel:"+deviceLevel+" Thread:"+Thread.currentThread().getName()+" Id:"+Thread.currentThread().getId());
		TypeSlotChannelEnable tsce = asynchDemuxer.getNameToTypeSlotChannel(deviceName);
		if(DEBUG) 
			System.out.println(this.getClass().getName()+" Device:"+deviceName+" deviceLevel:"+deviceLevel+" slot:"+tsce.getSlot()+" channel:"+tsce.channel+" Thread:"+Thread.currentThread().getName()+" Id:"+Thread.currentThread().getId());
		//String affectorCommand = "G5 Z"+tsce.slot+" C"+tsce.channel+" P"+String.valueOf(deviceLevel);
		AsynchDemuxer.addWrite(asynchDemuxer, tsce.genActivate(deviceLevel));
	}

	/**
	 * Generate a series of requests to query the Marlinspike realtime subsystem and retrieve status
	 * reports via various M codes issued to the realtime processing loop.<p>
	 * We issue the commands directly through the serial USB or other port or call the standalone method 
	 * and wait for status messages to be returned from the Marlinspike.<p/>
	 * These reports are delineated by XML type headers and have various levels of structure, so a handler
	 * is used in a separate thread for each 'topic' that corresponds to a header from the status payload from the Marlinspike.<p/>
	 * These processing threads get the asynchronously returned data by demultiplexing them from a circular blocking queue
	 * populated by the main run method of (@code AsynchDemuxer} which reads from the Marlinspike using the third party
	 * open source RxTx library and its own read and write threads, or processes the return from the MarlinspikeDataPort.<p/>
	 * Once each handler has demuxxed the data from the queue, it creates a series of {@code MachineReading} instances which
	 * serve to order and give further structure to the retrieved data.<p/>
	 * Each topic handler has it own queue of MachineReading instances and so in this way the realtime data which may come back
	 * in all sorts of order and at various times is categorized and ordered.<p/>
	 * The other purpose of the MachineReading and the {@code MachineBridge} arbiter that mitigates the intermediate MachineReading
	 * processing is XML formatting using JAXB and suppling a JQuery ready set of XML compliant structures for web facing
	 * applications.
	 */
	public synchronized String reportAllControllerStatus() throws IOException {
		//getSystemStatus();
		//getAssignedPins();
		//getDigitalPinSetting();
		//getAnalogPinSetting();
		//getUltrasonicPinSetting();
		//getPWMPinSetting();
		//getMotorControlSetting();
		getControllerStatus();
		//getPWMControlSetting();
		if(DEBUG)
			System.out.println(this.getClass().getName()+".reportAllControllerStatus");
		return "Ok";
	}
	
	  // 
    // Report methods. The sequence is to issue the M-code to the MarlinSpike. The returned data will
    // include the proper <headers> which are 'demuxxed' and the correct MachineReadings are created from
    // the retrieved data and added to the queues in each MachineBridge instance for that topic
    // as they are retrieved from the MarlinSpike.<br/>
    // After issuing each M-code, call one of these methods to acquire the queue with the MachineReadings 
    // and call toString on them to build the proper output buffer for each topic, then do whatever with the String
    // payload.
    //
	public synchronized String reportSystemId() throws IOException {
		String statCommand1 = "M115"; // system id
		AsynchDemuxer.addWrite(asynchDemuxer, statCommand1);
		return "Ok";
	}
    /**
     * M700
     */
    public synchronized void getSystemStatus() throws IOException {
		String statCommand1 = "M700"; // report status
		AsynchDemuxer.addWrite(asynchDemuxer, statCommand1);
    }
    /**
     * M701
     */
    public synchronized void getDigitalPinSetting() throws IOException {
		String statCommand1 = "M701"; // report status
		AsynchDemuxer.addWrite(asynchDemuxer, statCommand1);
    }
    /**
     * M702
     */
    public synchronized void getAnalogPinSetting() throws IOException {
		String statCommand1 = "M702"; // report status
		AsynchDemuxer.addWrite(asynchDemuxer, statCommand1);
    }
    /**
     * M703
     */
    public synchronized void getUltrasonicPinSetting() throws IOException {
		String statCommand1 = "M703"; // report status
		AsynchDemuxer.addWrite(asynchDemuxer, statCommand1);
    }
    /**
     * M704
     */
    public synchronized void getPWMPinSetting() throws IOException {
		String statCommand1 = "M704"; // report status
		AsynchDemuxer.addWrite(asynchDemuxer, statCommand1);
    }
    /**
     * M706
     * @throws IOException 
     */
    public synchronized void getAssignedPins() throws IOException {
		String statCommand1 = "M706"; // report all pins in use
		AsynchDemuxer.addWrite(asynchDemuxer,statCommand1);	
    }
    /**
     * M705
     * @throws IOException 
     */
    public synchronized void getMotorControlSetting() throws IOException {
		String statCommand1 = "M705"; // report all pins in use
		AsynchDemuxer.addWrite(asynchDemuxer, statCommand1);
    }
    /**
     * M798 Z<slot> X
     * @throws IOException 
     */
    public synchronized void getPWMControlSetting() throws IOException {
    	for(int i = 0; i < 10; i++) {
    		String statCommand1 = "M798 Z"+i+" X"; // report all pins in use
    		AsynchDemuxer.addWrite(asynchDemuxer,statCommand1);
    	}
    }
    /**
     * M999 Reset MCU
     * @throws IOException 
     */
    public synchronized String commandReset() throws IOException {
    	for(int i = 0; i < 10; i++) {
    		String statCommand1 = "M999"; // report all pins in use
    		AsynchDemuxer.addWrite(asynchDemuxer,statCommand1);
    	}
    	return "Ok";
    }
    
	@Override
	public void commandPWM(String req) {
		AsynchDemuxer.addWrite(asynchDemuxer, req);
	}
    /**
     * M798 Z<slot>
     * @return A String payload of the status of each of the assigned motor controllers.
     * @throws IOException 
     */
    public synchronized void getControllerStatus() throws IOException {
    	for(int i = 0; i < 10; i++) {
			if(DEBUG)
				System.out.println(this.getClass().getName()+".reportAllControllerSatus controller in use in slot"+i);
    		String statCommand1 = "M798 Z"+i; // report all pins in use
    		AsynchDemuxer.addWrite(asynchDemuxer, statCommand1);
    	}
    }
    
	
	@Override
	public void commandStop() throws IOException {
		String motorCommand1 = "M799";
		AsynchDemuxer.addWrite(asynchDemuxer, motorCommand1);	
	}

	
	public static void main(String[] args) throws Exception {
		if( args.length < 1 ) {
			System.out.println("Usage: java -cp <classpath> com.neocoretechs.robocore.MegaControl deviceName deviceLevel");
		}
		Robot robot = new Robot();
		MarlinspikeManager mm = new MarlinspikeManager(robot);
		AsynchDemuxer asynchDemuxer = new AsynchDemuxer(mm);
		asynchDemuxer.connect(new ByteSerialDataPort());
		MarlinspikeControl mc = new MarlinspikeControl(asynchDemuxer);
		// set the absolute speed of the diff drive controller in slot 0 to 100 on channel 1 and 
		// 1 on channel 2
		mc.setDeviceLevel(args[0],Integer.parseInt(args[1]));

	}


}



