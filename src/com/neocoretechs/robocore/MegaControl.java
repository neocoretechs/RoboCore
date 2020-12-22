package com.neocoretechs.robocore;

import java.io.IOException;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineBridge;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.propulsion.MotorControlInterface2D;


/**
 * The MEGA control endpoint that controls serial data.
 * The 2 critical elements that must be provided to sustain the operations of this class are;
 * 1.) An {@code AsynchDemuxer} asynchronous demuxxer that can process the messages coming from 
 * the Marlinspike realtime subsystem.
 * 2.) A {@code DataportInterface} source of data to be demuxxed.
 * The data port also serves, when writable, to receive directives that initiate responses to be demuxxed.<p/>
 * This class talks to the serial drivers that communicate with the attached USB SBC. 
 * In fact, this is code that talks to the
 * RS232 board that converts to TTL that talks to the SBC that runs the embedded code that manages
 * the comm to the motor controller in the Marlinspike realtime subsystem, for example.<p/>
 * The relevant methods generate Twist messages that can be multiplexed to the Ros bus
 * and sent further on.
 * Increasing levels of complexity of motion control with options to move in polar arcs, set absolute speeds, etc.
 * Absolute and relative motion with the fusion of IMU data if desired.
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2017,2020
 *
 */
public class MegaControl implements MotorControlInterface2D, PWMControlInterface {
	public static boolean DEBUG = true;
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

	public MegaControl(AsynchDemuxer asynchDemuxer) { this.asynchDemuxer = asynchDemuxer; }
	
	public synchronized void setAbsoluteDiffDriveSpeed(int slot1, int channel1, int leftWheelSpeed, int slot2, int channel2, int rightWheelSpeed) throws IOException {
		if(DEBUG) 
			System.out.println(this.getClass().getName()+".setAbsoluteDiffDriveSpeed BEGIN writing slot:"+slot1+" channel:"+channel1+" left wheel spd:"+leftWheelSpeed+
					"slot "+slot2+" channel:"+channel2+" right wheel speed "+rightWheelSpeed);
		String motorCommand1 = "G5 Z"+slot1+" C"+channel1+" P"+String.valueOf(leftWheelSpeed);
		asynchDemuxer.getDataPort().writeLine(motorCommand1);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}
		String motorCommand2 = "G5 Z"+slot2+" C"+channel2+" P"+String.valueOf(rightWheelSpeed);
		asynchDemuxer.getDataPort().writeLine(motorCommand2);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}
		if(DEBUG) 
			System.out.println(this.getClass().getName()+".setAbsoluteDiffDriveSpeed END writing slot:"+slot1+" channel:"+channel1+" left wheel spd:"+leftWheelSpeed+
					"slot "+slot2+" channel:"+channel2+" right wheel speed "+rightWheelSpeed);
	}
	/**
	 * Generate a series of requests to query the Marlinspike realtime subsystem and retrieve status
	 * reports via various M codes issued to the realtime processing loop.<p>
	 * We issue the commands directly through the serial USB or other port and wait for status messages to be
	 * returned from the Marlinspike.<p/>
	 * These reports are delineated by XML type headers and have various levels of structure, so a handler
	 * is used in a separate thread for each 'topic' that corresponds to a header from the status payload from the Marlinspike.<p/>
	 * These processing threads get the asynchronously returned data by demultiplexing them from a circular blocking queue
	 * populated by the main run method of (@code AsynchDemuxer} which reads from the Marlinspike using the third party
	 * open source RxTx library and its own read and write threads.<p/>
	 * Once each handler has demuxxed the data from the queue, it creates a series of {@code MachineReading} instances which
	 * serve to order and give further structure to the retrieved data.<p/>
	 * Each topic handler has it own queue of MachineReading instances and so in this way the realtime data which may come back
	 * in all sorts of order and at various times is categorized and ordered.<p/>
	 * The other purpose of the MachineReading and the {@code MachineBridge} arbiter that mitigates the intermediate MachineReading
	 * processing is XML formatting using JAXB and suppling a JQuery ready set of XML compliant structures for web facing
	 * applications.
	 */
	public synchronized String reportAllControllerStatus() throws IOException {
		asynchDemuxer.clearLineBuffer();
		if(DEBUG)
			System.out.println(this.getClass().getName()+".reportAllControllerStatus System status");
		StringBuilder sb = new StringBuilder();
		sb.append("System status:\r\n");
		// <a> header returned from MarlinSpike
		sb.append(getSystemStatus());
		sb.append("\r\n");
		if(DEBUG)
			System.out.println(this.getClass().getName()+".reportAllControllerStatus pins in use");
		//
		sb.append("All pins in use:\r\n");
		// <assignedpins> header returned from MarlinSpike
		sb.append(getAssignedPins());
		sb.append("\r\n");
		//
		if(DEBUG)
			System.out.println(this.getClass().getName()+".reportAllControllerStatus controllers in use");
		sb.append("\r\nAll controllers in use:\r\n");
		// <motorcontrolsetting> header returned from MarlinSpike
		sb.append(getMotorControlSetting());
		sb.append("\r\n");
		//
		// <controllerstatus> header
		sb.append(getControllerStatus());
		//
		sb.append("\r\nPWM controllers in use:\r\n");
		// <pwmcontrolsetting>
		sb.append(getPWMControlSetting()+"\r\n");
		if(DEBUG)
			System.out.println(this.getClass().getName()+".reportAllControllerStatus returning:\r\n"+sb.toString());
		return sb.toString();
	}
	
	public String getMachineReadingsFromBridge(String group) throws IOException {
			MachineReading mr;
			StringBuilder sb = new StringBuilder();
			MachineBridge mb = asynchDemuxer.getMachineBridge(group);
			while((mr = mb.waitForNewReading()) != MachineReading.EMPTYREADING) {
				if(mr == null)
					throw new IOException("PREMATURE END OF MACHINEREADINGS!");
				sb.append(mr.toString());
				sb.append("\r\n");
			}
			return sb.toString();
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
    /**
     * M700
     * @return A string payload of robot overall status
     */
    public synchronized String getSystemStatus() throws IOException {
		String statCommand1 = "M700"; // report status
		asynchDemuxer.getDataPort().writeLine(statCommand1);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}		
    	return getMachineReadingsFromBridge(topicNames.STATUS.val());
    }
    /**
     * M706
     * @return A String payload of all assigned pins (if any), comma separated.
     * @throws IOException 
     */
    public synchronized String getAssignedPins() throws IOException {
		String statCommand1 = "M706"; // report all pins in use
		asynchDemuxer.getDataPort().writeLine(statCommand1);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}		
    	return getMachineReadingsFromBridge(topicNames.ASSIGNEDPINS.val());
    }
    /**
     * M705
     * @return A String payload of motor controller configurations (if any), each one a multiline report.
     * @throws IOException 
     */
    public synchronized String getMotorControlSetting() throws IOException {
		String statCommand1 = "M705"; // report all pins in use
		asynchDemuxer.getDataPort().writeLine(statCommand1);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}		
    	return getMachineReadingsFromBridge(topicNames.MOTORCONTROLSETTING.val());
    }
    /**
     * M798 Z<slot> X
     * @return A String payload of PWM controller status (if any), each one a multiline report.
     * @throws IOException 
     */
    public synchronized String getPWMControlSetting() throws IOException {
    	StringBuilder sb = new StringBuilder();
    	for(int i = 0; i < 10; i++) {
    		sb.append("\r\nPWM Controller in use in slot:"+i+"\r\n");
    		String statCommand1 = "M798 Z"+i+" X"; // report all pins in use
    		asynchDemuxer.getDataPort().writeLine(statCommand1);
    		try {
    			Thread.sleep(100);
    		} catch (InterruptedException e) {}
    		sb.append(getMachineReadingsFromBridge(topicNames.CONTROLLERSTATUS.val()));
    		sb.append("---");
    	}
    	return sb.toString();
    }
    /**
     * M798 Z<slot>
     * @return A String payload of the status of each of the assigned motor controllers.
     * @throws IOException 
     */
    public synchronized String getControllerStatus() throws IOException {
       	StringBuilder sb = new StringBuilder();
    	for(int i = 0; i < 10; i++) {
    		sb.append("\r\nController in use in slot:"+i+"\r\n");
			if(DEBUG)
				System.out.println(this.getClass().getName()+".reportAllControllerSatus controller in use in slot"+i);
    		String statCommand1 = "M798 Z"+i; // report all pins in use
    		asynchDemuxer.getDataPort().writeLine(statCommand1);
    		try {
    			Thread.sleep(100);
    		} catch (InterruptedException e) {}
    		sb.append(getMachineReadingsFromBridge(topicNames.CONTROLLERSTATUS.val()));
    		sb.append("---");
    	}
    	return sb.toString();
    }
	public synchronized void setAbsolutePWMLevel(int slot, int channel, int pwmLevel) throws IOException {
		String pwmCommand1 = "G5 Z"+slot+" C"+channel+" X"+pwmLevel;
		asynchDemuxer.getDataPort().writeLine(pwmCommand1);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}
	}
	
	@Override
	public void commandStop() throws IOException {
		String motorCommand1 = "M799";
		asynchDemuxer.getDataPort().writeLine(motorCommand1);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}
		
	}

	@Override
	public void setAbsolutePWMLevel(int slot1, int channel1, int leftWheelSpeed, int slot2, int channel2, int rightWheelSpeed) throws IOException {
		String motorCommand1 = "G5 Z"+slot1+" C"+channel1+" X"+String.valueOf(leftWheelSpeed);
		asynchDemuxer.getDataPort().writeLine(motorCommand1);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}
		String motorCommand2 = "G5 Z"+slot2+" C"+channel2+" X"+String.valueOf(rightWheelSpeed);
		asynchDemuxer.getDataPort().writeLine(motorCommand2);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}
		
	}
	public static void main(String[] args) throws Exception {
		if( args.length < 1 ) {
			System.out.println("Usage: java -cp <classpath> com.neocoretechs.robocore.MegaControl");
		}
		AsynchDemuxer asynchDemuxer = new AsynchDemuxer();
		asynchDemuxer.connect(com.neocoretechs.robocore.serialreader.ByteSerialDataPort.getInstance());
		MegaControl mc = new MegaControl(asynchDemuxer);
		// set the absolute speed of the diff drive controller in slot 0 to 100 on channel 1 and 
		// 1 on channel 2
		mc.setAbsoluteDiffDriveSpeed(0, 1, 100, 0, 2, 1);

	}

}



