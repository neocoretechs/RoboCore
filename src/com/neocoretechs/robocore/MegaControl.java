package com.neocoretechs.robocore;

import java.io.IOException;

import com.neocoretechs.robocore.PID.AbstractPIDController;
import com.neocoretechs.robocore.propulsion.DrivenWheelInterface;
import com.neocoretechs.robocore.propulsion.MotorControlInterface2D;
import com.neocoretechs.robocore.propulsion.TwistInfo;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;

/**
 * The MEGA control endpoint that controls serial data.
 * This class talks to the serial drivers that communicate with the attached USB SBC. In fact, this is code that talks to the
 * RS232 board that converts to TTL that talks to the SBC that runs the embedded code that manages
 * the comm to the motor controller.
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

	/* Stop the robot if it hasn't received a movement command in this number of milliseconds */
	public static int AUTO_STOP_INTERVAL = 2000;
	long lastMotorCommand = AUTO_STOP_INTERVAL;
	/* The base and odometry frames */
	//String baseFrame = "/base_link";
	//String odomFrame = "/odom";
	


	protected static boolean moving = false; // is the base in motion?



	float theta_offset;     /* offset from from global theta to local theta */


	
	//int nextOdom = 0;
	DrivenWheelInterface leftWheel, rightWheel;
	OdomInfo odomInfo;
	private float yawIMURads;

	
	public MegaControl() { 
		init(); 
	}

	
	public synchronized void setAbsoluteMotorSpeed(int slot1, int channel1, int ch1, int slot2, int channel2, int ch2) throws IOException {

		/* Reset the auto stop timer */
		lastMotorCommand = System.currentTimeMillis();
		/* Indicate that we are moving */
		moving = true;
		// Set the target speeds in wheel rotation command units -1000, 1000 and if indoor mode div by ten
		if(DrivenWheelInterface.indoor) {
			leftWheel.setTargetSpeed(ch1/10);
			rightWheel.setTargetSpeed(ch1/10);
		} else {
			leftWheel.setTargetSpeed(ch1);
			rightWheel.setTargetSpeed(ch1);
		}
		if( DEBUG )
			System.out.println("Absolute Motor L:"+leftWheel.getTargetSpeed()+" R:"+rightWheel.getTargetSpeed());
		/* Convert speeds to ticks per frame */
		leftWheel.setTargetTicksPerFrame(leftWheel.SpeedToTicks((float) leftWheel.getTargetSpeed()));
		rightWheel.setTargetTicksPerFrame(rightWheel.SpeedToTicks((float) rightWheel.getTargetSpeed()));
		/* Read the encoders */
		//leftWheel.Encoder = 0;//encoders.YAxisGetCount();
		//rightWheel.Encoder = 0;//encoders.XAxisGetCount();

		/* Record the time that the readings were taken */
		odomInfo.lastOdomTime = System.currentTimeMillis();
		//odomInfo.encoderStamp = nh.now;

		/* Compute PID update for each motor */
		leftWheel.getPIDController().doPID(leftWheel);
		rightWheel.getPIDController().doPID(rightWheel);

		updateSpeed(slot1, channel1, (int)leftWheel.getTargetSpeed(), slot2, channel2, (int)rightWheel.getTargetSpeed());
	}
	
	public void setForward() throws IOException {}
	public void setReverse() throws IOException {}

	
	@Override
	public synchronized void updateSpeed(int slot1, int channel1, int leftWheelSpeed, int slot2, int channel2, int rightWheelSpeed) throws IOException {
		String motorCommand1 = "G5 Z"+slot1+" C"+channel1+" P"+String.valueOf(leftWheelSpeed);
		ByteSerialDataPort.getInstance().writeLine(motorCommand1);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}
		String motorCommand2 = "G5 Z"+slot2+" C"+channel2+" P"+String.valueOf(rightWheelSpeed);
		ByteSerialDataPort.getInstance().writeLine(motorCommand2);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}
	}
	
	public synchronized void updatePWMLevel(int slot, int channel, int pwmLevel) throws IOException {
		String pwmCommand1 = "G5 Z"+slot+" C"+channel+" X"+pwmLevel;
		ByteSerialDataPort.getInstance().writeLine(pwmCommand1);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}
	}
	
	private synchronized void clearPID() {
		moving = false;
		leftWheel.getPIDController().clearPID();
		rightWheel.getPIDController().clearPID();
	}

	private synchronized void clearAll() {
		clearPID();
		//encoders.XAxisReset();
		//encoders.YAxisReset();
	}
	/**
	 * Hard coded to slot 0, channel, slot 0, channel 2 setting power to 0. Could use M799 or M799x to shut down all
	 * calling commandEmergencyStop for each active controller channel. here we call Slot 0 channel 1, Slot 0 channel 2.
	 */
	public synchronized void commandStop() throws IOException {
		setAbsoluteMotorSpeed(0, 1, 0, 0, 2, 0);
	}

	protected synchronized void init() {
		leftWheel = new DrivenWheelInterface() {

			@Override
			public void setTargetSpeed(float t) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void setTargetMM(float t) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void setTargetTicksPerFrame(float t) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public float getTargetSpeed() {
				// TODO Auto-generated method stub
				return 0;
			}

			@Override
			public float getTargetMM() {
				// TODO Auto-generated method stub
				return 0;
			}

			@Override
			public float getTargetTicksPerFrame() {
				// TODO Auto-generated method stub
				return 0;
			}

			@Override
			public int SpeedToTicks(float v) {
				// TODO Auto-generated method stub
				return 0;
			}

			@Override
			public void setprevX(float t) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void setTwistInfo(TwistInfo t) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public AbstractPIDController getPIDController() {
				// TODO Auto-generated method stub
				return null;
			}

			@Override
			public TwistInfo getTwistInfo() {
				// TODO Auto-generated method stub
				return null;
			}
		};
		rightWheel = new DrivenWheelInterface() {

			@Override
			public void setTargetSpeed(float t) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void setTargetMM(float t) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void setTargetTicksPerFrame(float t) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public float getTargetSpeed() {
				// TODO Auto-generated method stub
				return 0;
			}

			@Override
			public float getTargetMM() {
				// TODO Auto-generated method stub
				return 0;
			}

			@Override
			public float getTargetTicksPerFrame() {
				// TODO Auto-generated method stub
				return 0;
			}

			@Override
			public int SpeedToTicks(float v) {
				// TODO Auto-generated method stub
				return 0;
			}

			@Override
			public void setprevX(float t) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void setTwistInfo(TwistInfo t) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public AbstractPIDController getPIDController() {
				// TODO Auto-generated method stub
				return null;
			}

			@Override
			public TwistInfo getTwistInfo() {
				// TODO Auto-generated method stub
				return null;
			}
		};
		odomInfo = new OdomInfo();
		/* Set the target speeds in meters per second */
		leftWheel.setTargetSpeed(0.0f);
		rightWheel.setTargetSpeed(0.0f);

		/* Convert speeds to encoder ticks per frame */
		leftWheel.setTargetTicksPerFrame(leftWheel.SpeedToTicks((float) leftWheel.getTargetSpeed()));
		rightWheel.setTargetTicksPerFrame(rightWheel.SpeedToTicks((float) rightWheel.getTargetSpeed()));

		/* Zero out the encoder counts */
		//encoders.XAxisReset();
		//encoders.YAxisReset();

		/* Initialize the ROS node */
		//nh.initNode();

		//nextPID =  (int)leftWheel.getPIDController().PID_INTERVAL;
		//nextOdom =  (int) ODOM_INTERVAL;

		/* Advertise the Odometry publisher */
		//nh.advertise(odomPub);
		//nh.advertise(logPub);

		/* Activate the Twist subscriber */
		//nh.subscribe(cmdVelSub);
	}
	
	public static void main(String[] args) throws Exception {
		if( args.length < 1 ) {
			System.out.println("Usage: java -cp <classpath> com.neocoretechs.robocore.MegaControl");
		}
		MegaControl mc = new MegaControl();
		//mc.setMotorSpeed(100.0f, 1f);
		/*
		 * TODO: make this something
		mc.moveRobotAbsolute(.5f, 45, 100, 1);
		mc.moveRobotAbsolute(-.25f, 45, 100, 1);
		mc.moveRobotAbsolute(.25f, 45, 100, 1);
		mc.moveRobotAbsolute(-.25f, 225, 100, 1);
		mc.moveRobotAbsolute(.0f, 0, 100, 1);
		*/
		
		//mc.moveRobotRelative(.5f, 45, 100);
		//mc.moveRobotRelative(-.25f, 45, 100);
		//mc.moveRobotRelative(.25f, 45, 100);
		//mc.moveRobotRelative(-.25f, 225, 100);
		//mc.moveRobotRelative(.0f, 0, 100);

	}
	/**
	 * @throws IOException 
	 * 
	 */
	@Override
	public void setAbsolutePWMLevel(int slot1, int channel2, int level3) throws IOException {
		updatePWMLevel(slot1, channel2, level3); 
	}


	/**
	 * Move absolute with some rudimentary checks for excessive angular momentum and objects too close
	 * and reduce motor rate by intermittent commandStop until values are acceptable.
	 * this is called by the chain or responsibility emanating from the cmd_vel topic.
	 * @throws IOException 
	 */
	@Override
	public synchronized boolean move2DAbsolute(float yawIMURads, int yawTargetDegrees, int targetDistance, float[] accelDeltas, int[] ranges) throws IOException {
		if( DEBUG )
			System.out.println("Motion control Move 2D Abs: "+yawTargetDegrees);
			this.yawIMURads = yawIMURads;
			this.yawTargetDegrees = yawTargetDegrees;
			this.targetDistance = targetDistance;
			this.accelDeltas = accelDeltas;
			this.ranges = ranges;
		if( yawTargetDegrees == 0 && targetDistance == 0) {
			if( DEBUG )
				System.out.println("0,0 command stop");
			commandStop();
			return false;
		}
		// if any deltas > about 100, there is lateral force sufficient to raise wheels
		// if any ranges close than about 200mm
		if( accelDeltas[0] > 500.0f || accelDeltas[1] > 500.0f || accelDeltas[2] > 500.0f) {
			if( DEBUG )
				System.out.println("MegaControl issuing stop based on accel deltas");
				commandStop(); // pick up further motion on subsequent commands when accel deltas are nicer
				return false;
		}
		if( ranges[0] < 225 || ranges[1] < 200 ) {
			if( ranges[0] < 200 || ranges[1] < 100 ) {
				if( DEBUG )
				System.out.println("<<<<<<Backing up!");
				this.targetDistance = -50;
				this.yawTargetDegrees = 0;
			} else {
				if( DEBUG )
				System.out.println("Altering course for obstacle");
				this.yawTargetDegrees += 30;
				if( this.yawTargetDegrees > 360 ) this.yawTargetDegrees -= 360;
				this.targetDistance = 0;
			}
		}
		moveRobotAbsolute(this.yawIMURads, this.yawTargetDegrees, this.targetDistance);	
		return true;
	}
	/**
	 * Move relative with some rudimentary checks for excessive angular momentum and objects too close
	 * and reduce motor rate by intermittent commandStop until values are acceptable.
	 * this is called by the chain or responsibility emanating from the cmd_vel topic.
	 * @throws IOException 
	 */
	@Override
	public synchronized boolean move2DRelative(float yawIMURads, int yawTargetDegrees, int targetDistance, float[] accelDeltas, int[] ranges) throws IOException {
		if( DEBUG )
			System.out.println("Motion control Move 2D Rel: "+yawTargetDegrees);
			this.yawIMURads = yawIMURads;
			this.yawTargetDegrees = yawTargetDegrees;
			this.targetDistance = targetDistance;
			this.accelDeltas = accelDeltas;
			this.ranges = ranges;
		if( yawTargetDegrees == 0 && targetDistance == 0) {
			if( DEBUG )
				System.out.println("0,0 command stop");
			commandStop();
			return false;
		}
		// if any deltas > about 100, there is lateral force sufficient to raise wheels
		// if any ranges close than about 200mm
		if( accelDeltas[0] > 500.0f || accelDeltas[1] > 500.0f || accelDeltas[2] > 500.0f) {
			if( DEBUG )
				System.out.println("MegaControl issuing stop based on accel deltas "+accelDeltas[0]+" "+accelDeltas[1]+" "+accelDeltas[2]);
				commandStop(); // pick up further motion on subsequent commands when accel deltas are nicer
				return false;
		} 
		// something in front, turn 30 degrees in place
		if( ranges[0] < 225 || ranges[1] < 200 ) {
			if( ranges[0] < 200 || ranges[1] < 100 ) {
				if( DEBUG )
					System.out.println("<<<<<<Backing up!");
				this.targetDistance = -50;
				this.yawTargetDegrees = 0;
			} else {
				if( DEBUG )
					System.out.println("MegaControl issuing 30 degr. rotation based on obstacle detection "+ranges[0]+" "+ranges[1]);
				if( yawTargetDegrees < 30 )
					this.yawTargetDegrees = 30;
			}
			this.targetDistance = 0;
		}
		if( DEBUG )
			System.out.println("MegaControl moving robot relative:"+this.yawIMURads+" "+this.yawTargetDegrees+" "+this.targetDistance+" "+this.targetTime);
		moveRobotRelative(this.yawIMURads, this.yawTargetDegrees, this.targetDistance);	
		return true;
	}


	/* A struct to hold Odometry info */
	class OdomInfo {
		long lastOdomTime;    // last millis() time odometry was calculated
		float linearX;	         // total linear x distance traveled
		float linearY;	         // total linear y distance traveled
		float angularZ;		 // total angular distance traveled
	}


}



