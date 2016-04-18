package com.neocoretechs.robocore;

import java.io.IOException;

import com.microcaliperdevices.saje.io.machine.dataport.ByteSerialDataPort;
/**
 * The motor control endpoint that controls serial data.
 * This class talks to the serial drivers that communicate with the attached USB SBC. In fact, this is code that talks to the
 * RS232 board that converts to TTL that talks to the SBC that runs the embedded code that manages
 * the comm to the motor controller.
 * The relevant methods generate Twist messages that can be multiplexed to the Ros bus
 * and sent further on.
 * @author jg
 *
 */
public class MotorControl implements MotorControlInterface2D {
	float yawIMURads;
	int yawTargetDegrees;
	int targetDistance;
	int targetTime; 
	float[] accelDeltas;
	int[] ranges;
	boolean init = true;
	Object mutex = new Object();
	//public static int MAXOUTPUT = 1000; // normal
	public static int MAXOUTPUT = 50; // indoor
	/* Stop the robot if it hasn't received a movement command in this number of milliseconds */
	public static int AUTO_STOP_INTERVAL = 2000;
	long lastMotorCommand = AUTO_STOP_INTERVAL;
	/* The base and odometry frames */
	//String baseFrame = "/base_link";
	//String odomFrame = "/odom";
	float TWOPI = (float) (Math.PI * 2.0f);
	/* PID Parameters */
	int Kp = 20;
	int Kd = 12;
	int Ki = 0;
	int Ko = 50;
	
	/* Define the robot parameters */
	// so ticks are IMU data in mm/s, I hope. In static form its locked to diameter but here, as IMU data, 
	// it is variable based on desired speed.
	int cpr = 500; // ticks per revolution, if IMU reports .5 m/s as 500 mm/s then 500, else we have to dynamically change based on speed
	float wheelDiameter = 406.4f; // millimeters, 16"
	float wheelTrack = 304.8f; // millimeters, 12"
	float ticksPerMeter = (float) (cpr / (Math.PI * wheelDiameter));
	boolean moving = false; // is the base in motion?

	float total_mm;
	float clicks_per_mm;
	float last_theta;
	float theta_offset;     /* offset from from global theta to local theta */
	float CLICKS_PER_MM = 2.0f; // 

	/* Rate at which PID loop is updated */
	int PID_RATE = 30;     // Hz
	float PID_INTERVAL = (float) (1000.0 / PID_RATE);

	/* Counters to track update rates for PID and Odometry */
	int nextPID = 0;
	//int nextOdom = 0;

	SetPointInfo leftWheel, rightWheel;
	OdomInfo odomInfo;
	TwistInfo twistInfo;
	
	public MotorControl() { 
		init(); 
	}

	
	/* Convert meters per second to ticks per time frame */
	private int SpeedToTicks(float v) {
		return (int) (v * cpr / (PID_RATE * Math.PI * wheelDiameter));
	}

	/**
	 *  The function computes motor speeds.
	 *  results in leftWheel.targetSpeed and rightWheel.targetSpeed
	 * so X is the arc radius, normally we will set as a call to
	 * setMotorSpeed as 0 to turn in place or a value of arc radius
	 * modified by current speed to make a gentle curve.
	 * if theta is 0, move linear in x
	 * if x and theta not 0 its rotation about a point in space
	 *  @param x The radius from the center point of the arc about which we are rotating, if 0 turn in place
	 *  @param th The 2PI polar measure of arc segment we are traversing, if 0 pure forward/back motion
	 * @throws IOException 
	 */
	public void setMotorSpeed(float x, float th) throws IOException {

		float spd_left, spd_right;
  
		/* Reset the auto stop timer */
		lastMotorCommand = System.currentTimeMillis();

		if (x == 0 && th == 0) {
			moving = false;
			String motorCommand = "G5 C1 P0";
			ByteSerialDataPort.getInstance().writeLine(motorCommand);
			motorCommand = "G5 C2 P0";
			ByteSerialDataPort.getInstance().writeLine(motorCommand);
			return;
		}

		/* Indicate that we are moving */
		moving = true;

		if (x == 0) {
			// Turn in place
			if( th < 0 ) { // left
				spd_right = (float) (th * wheelTrack / 2.0);
			} else {
				spd_right = -((float) (th * wheelTrack / 2.0));
			}
			spd_left = -spd_right;
		} else {
			if (th == 0) {	
				// Pure forward/backward motion
				spd_left = spd_right = x;
			} else {
				// Rotation about a point in space
				if( th < 0 ) { // left
					spd_left = (float) (x - th * wheelTrack / 2.0);
					spd_right = (float) (x + th * wheelTrack / 2.0);
				} else {
					spd_right = (float) (x - th * wheelTrack / 2.0);
					spd_left = (float) (x + th * wheelTrack / 2.0);
				}
			}
		}

		/* Set the target speeds in meters per second */
		leftWheel.TargetSpeed = spd_left;
		rightWheel.TargetSpeed = spd_right;

		/* Convert speeds to ticks per frame */
		leftWheel.TargetTicksPerFrame = SpeedToTicks((float) leftWheel.TargetSpeed);
		rightWheel.TargetTicksPerFrame = SpeedToTicks((float) rightWheel.TargetSpeed);

		updateSpeed();
	}

	/**
	 *  PID routine to compute the next motor commands 
	 */
	private void doPID(SetPointInfo p) {
		/*
		int Perror;
		int output;
		Perror = (int) (p.TargetTicksPerFrame - (p.X - p.prevX));

		// Derivative error is the delta Perror
		output = (Kp * Perror + Kd * (Perror - p.PrevErr) + Ki * p.Ierror) / Ko;
		p.PrevErr = (int) Perror;
		//p.PrevEnc = p.Encoder;

		output += p.output;
		 
		if (output >= MAXOUTPUT)
			output = MAXOUTPUT;
		else if (output <= -MAXOUTPUT)
			output = -MAXOUTPUT;
		else
			p.Ierror += Perror;

		p.output = (int) output;
		*/
		if (p.TargetSpeed >= MAXOUTPUT)
			p.TargetSpeed = MAXOUTPUT;
		else if (p.TargetSpeed <= -MAXOUTPUT)
			p.TargetSpeed = -MAXOUTPUT;	
	}

	/* Read the encoder values and call the PID routine */
	protected void updateSpeed() throws IOException {
		/* Read the encoders */
		//leftWheel.Encoder = 0;//encoders.YAxisGetCount();
		//rightWheel.Encoder = 0;//encoders.XAxisGetCount();

		/* Record the time that the readings were taken */
		odomInfo.lastOdomTime = System.currentTimeMillis();
		//odomInfo.encoderStamp = nh.now;

		/* If we're not moving there is nothing more to do */
		if (!moving)
			return;

		/* Compute PID update for each motor */
		doPID(leftWheel);
		doPID(rightWheel);

		/* Set the motor speeds accordingly */
		System.out.println("Motor:"+leftWheel.TargetSpeed+" "+rightWheel.TargetSpeed);
		// we invert channel 1 since its a mirror of the orientation of channel 2
		leftWheel.TargetSpeed = -leftWheel.TargetSpeed;
		
		String motorCommand = "G5 C1 P"+String.valueOf((int)leftWheel.TargetSpeed);
		ByteSerialDataPort.getInstance().writeLine(motorCommand);
		
		motorCommand = "G5 C2 P"+String.valueOf((int)rightWheel.TargetSpeed);
		ByteSerialDataPort.getInstance().writeLine(motorCommand);
	}

	private void clearPID() {
		moving = false;
		leftWheel.PrevErr = 0;
		leftWheel.Ierror = 0;
		leftWheel.output = 0;
		rightWheel.PrevErr = 0;
		rightWheel.Ierror = 0;
		rightWheel.output = 0;
	}

	private void clearAll() {
		clearPID();
		//encoders.XAxisReset();
		//encoders.YAxisReset();
	}
	
	public void commandStop() throws IOException {
		setMotorSpeed(0.0f, 0.0f);
	}

	private void init() {
		leftWheel = new SetPointInfo();
		rightWheel = new SetPointInfo();
		odomInfo = new OdomInfo();
		twistInfo = new TwistInfo();
		/* Set the target speeds in meters per second */
		leftWheel.TargetSpeed = 0.0f;
		rightWheel.TargetSpeed = 0.0f;

		/* Convert speeds to encoder ticks per frame */
		leftWheel.TargetTicksPerFrame = SpeedToTicks((float) leftWheel.TargetSpeed);
		rightWheel.TargetTicksPerFrame = SpeedToTicks((float) rightWheel.TargetSpeed);

		/* Zero out the encoder counts */
		//encoders.XAxisReset();
		//encoders.YAxisReset();

		/* Initialize the ROS node */
		//nh.initNode();

		nextPID =  (int) PID_INTERVAL;
		//nextOdom =  (int) ODOM_INTERVAL;

		/* Advertise the Odometry publisher */
		//nh.advertise(odomPub);
		//nh.advertise(logPub);

		/* Activate the Twist subscriber */
		//nh.subscribe(cmdVelSub);
	}
	
	public static void main(String[] args) throws Exception {
		if( args.length < 1 ) {
			System.out.println("Usage: java -cp <classpath> com.neocoretechs.robocore.MotorControl");
		}
		MotorControl mc = new MotorControl();
		//mc.setMotorSpeed(100.0f, 1f);
		/*
		mc.moveRobotAbsolute(.5f, 45, 100, 1);
		mc.moveRobotAbsolute(-.25f, 45, 100, 1);
		mc.moveRobotAbsolute(.25f, 45, 100, 1);
		mc.moveRobotAbsolute(-.25f, 225, 100, 1);
		mc.moveRobotAbsolute(.0f, 0, 100, 1);
		*/
		
		mc.moveRobotRelative(.5f, 45, 100);
		mc.moveRobotRelative(-.25f, 45, 100);
		mc.moveRobotRelative(.25f, 45, 100);
		mc.moveRobotRelative(-.25f, 225, 100);
		mc.moveRobotRelative(.0f, 0, 100);

	}
	/**
	 * Move the robot to an absolute pose of 0-360 degrees and attempt to travel the target distance in the target time
	 * when the IMU and target coincide distance and time produce linear motion
	 * otherwise when turning distance determines the radius of the arc segment described by the difference
	 * of the current IMU and target and time becomes roughly the speed at which to make such traversal
	 * @param yawIMURads Yaw is delivered in radians -1.0 to 1.0
	 * @param yawTargetDegrees target final pose is in degrees for easier human input
	 * @param targetDistance target distance to travel in mm
	 * @param targetTime time in which to travel desired distance in seconds
	 * 	 * @return The Twist message with twistInfo.robotTheta as the value to turn (X,Y,deltaTheta,IMUTheta,wheelTheta,yawDegrees as well)
	 * @throws IOException 
	 */
	public TwistInfo moveRobotAbsolute(float yawIMURads, int yawTargetDegrees, int targetDistance) throws IOException {

	        //while (true) {

	                //leftWheel.targetMM = (float)(left_velocity) / clicks_per_mm;
	                //rightWheel.targetMM = (float)(right_velocity) / clicks_per_mm;

	                //avg_mm = (float) ((leftWheel.targetMM + rightWheel.targetMM) / 2.0);
	                //total_mm += avg_mm;
	                // wheel_theta is travel of each wheel with desired velocity, if same angle is 0, forward
	                //twistInfo.wheelTheta = (leftWheel.targetMM - rightWheel.targetMM) / wheelTrack;
	        		
	        		twistInfo.wheelTheta = (float)yawTargetDegrees * 0.0174532925f; // degr to radians
	                /* read the YAW value from the imu struct and convert to radians */
	                twistInfo.imuTheta = (float) (yawIMURads * Math.PI);
	                if( twistInfo.imuTheta < 0.0f ) {
	                	twistInfo.imuTheta += TWOPI;
	                }
	                twistInfo.yawDegrees = yawDegrees(twistInfo.imuTheta);
	                
	                /* calculate rotation rate-of-change  */
	                twistInfo.deltaTheta = twistInfo.imuTheta - last_theta;
	                last_theta = twistInfo.imuTheta;

	                // this will generate the +- value to turn us properly
	                twistInfo.robotTheta = twistInfo.wheelTheta - twistInfo.imuTheta;
	                if( twistInfo.robotTheta > TWOPI )
	                	twistInfo.robotTheta -= TWOPI;
	                if( twistInfo.robotTheta < 0)
	                	twistInfo.robotTheta += TWOPI;
	                    
	                twistInfo.X += (float)(targetDistance * Math.sin(twistInfo.robotTheta)); 
	                twistInfo.Y += (float)(targetDistance * Math.cos(twistInfo.robotTheta)); 
	                // so X is the arc radius, normally we will set as a call to
	                // setMotorSpeed as 0 to turn in place or a value of arc radius
	                // modified by current speed to make a gentle curve.
	                // if theta is 0, move linear
	                //stasis(fabs(wheel_theta),fabs(imu_theta));
	                System.out.println(twistInfo);
	                setMotorSpeed(targetDistance, twistInfo.robotTheta);
	                try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
	                return twistInfo;//twistInfo.robotTheta;
	        //}
	}
	
	/**
	 * Move the robot to an relative pose of 0-360 degrees and attempt to travel the target distance in the target time
	 * Here, targetDegrees can be negative
	 * when the IMU and target coincide distance and time produce linear motion
	 * otherwise when turning distance determines the radius of the arc segment described by the difference
	 * of the current IMU and target and time becomes roughly the speed at which to make such traversal
	 * @param yawIMURads Yaw is delivered in radians -1.0 to 1.0
	 * @param yawTargetDegrees target final pose is in degrees for easier human input and represents motion relative to current pose vs absolute position
	 * @param targetDistance target distance to travel in mm
	 * @param targetTime time in which to travel desired distance in seconds
	 * @return The Twist message with twistInfo.robotTheta as the value to turn (X,Y,deltaTheta,IMUTheta,wheelTheta,yawDegrees as well)
	 * @throws IOException 
	 */
	public TwistInfo moveRobotRelative(float yawIMURads, int yawTargetDegrees, int targetDistance) throws IOException {
	        //while (true) {

	                //leftWheel.targetMM = (float)(left_velocity) / clicks_per_mm;
	                //rightWheel.targetMM = (float)(right_velocity) / clicks_per_mm;

	                //avg_mm = (float) ((leftWheel.targetMM + rightWheel.targetMM) / 2.0);
	                //total_mm += avg_mm;
	                // wheel_theta is travel of each wheel with desired velocity, if same angle is 0, forward
	                //twistInfo.wheelTheta = (leftWheel.targetMM - rightWheel.targetMM) / wheelTrack;
	        		
	        		twistInfo.wheelTheta = (float)yawTargetDegrees * 0.0174532925f; // degr to radians
	                /* read the YAW value from the imu struct and convert to radians */
	                twistInfo.imuTheta = (float) (yawIMURads * Math.PI);
	                if( twistInfo.imuTheta < 0.0f ) {
	                	twistInfo.imuTheta += TWOPI;
	                }
	                twistInfo.yawDegrees = yawDegrees(twistInfo.imuTheta);
	                
	                /* calculate rotation rate-of-change  */
	                twistInfo.deltaTheta = twistInfo.imuTheta - last_theta;
	                last_theta = twistInfo.imuTheta;

	                twistInfo.robotTheta  = twistInfo.imuTheta + twistInfo.wheelTheta;
	                // this will generate the value to turn us properly
	                if( twistInfo.robotTheta > TWOPI )
	                	twistInfo.robotTheta -= TWOPI;
	                if( twistInfo.robotTheta < 0)
	                	twistInfo.robotTheta += TWOPI;

	                twistInfo.X += (float)(targetDistance * Math.sin(twistInfo.robotTheta)); 
	                twistInfo.Y += (float)(targetDistance * Math.cos(twistInfo.robotTheta)); 
	                // so X is the arc radius, normally we will set as a call to
	                // setMotorSpeed as 0 to turn in place or a value of arc radius
	                // modified by current speed to make a gentle curve.
	                // if theta is 0, move linear in x
	                // if x and theta not 0 its rotation about a point in space
	                //stasis(fabs(wheel_theta),fabs(imu_theta));
	                System.out.println(twistInfo);
	                setMotorSpeed(targetDistance, twistInfo.wheelTheta);
	                try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
	                return twistInfo;//twistInfo.robotTheta;
	        //}
	}
	
	public static float yawDegrees(float yaw) {
		float yaw_degrees = (float) (yaw * 180.0 / Math.PI); // conversion to degrees
		if( yaw_degrees < 0 ) yaw_degrees += 360.0; // convert negative to positive angles
		return yaw_degrees;
	}

	/**
	 * Move absolute with some rudimentary checks for excessive angular momentum and objects too close
	 * and reduce motor rate by intermittent commandStop until values are acceptable.
	 * The ARDrone US seems to flatten everything to 0 when objects 200mm closer or less
	 * this is called by the chain or responsibility emanating from the cmd_vel topic.
	 * @throws IOException 
	 */
	@Override
	public boolean move2DAbsolute(float yawIMURads, int yawTargetDegrees, int targetDistance, float[] accelDeltas, int[] ranges) throws IOException {
		System.out.println("Motion control Move 2D Abs: "+yawTargetDegrees);
		synchronized( mutex ) {
			this.yawIMURads = yawIMURads;
			this.yawTargetDegrees = yawTargetDegrees;
			this.targetDistance = targetDistance;
			this.accelDeltas = accelDeltas;
			this.ranges = ranges;
		}
		if( yawTargetDegrees == 0 && targetDistance == 0) {
			System.out.println("0,0 command stop");
			commandStop();
			return false;
		}
		// if any deltas > about 100, there is lateral force sufficient to raise wheels
		// if any ranges close than about 200mm
		if( accelDeltas[0] > 500.0f || accelDeltas[1] > 500.0f || accelDeltas[2] > 500.0f) {
				System.out.println("MotorControl issuing stop based on accel deltas");
				commandStop(); // pick up further motion on subsequent commands when accel deltas are nicer
				return false;
		}
		if( ranges[0] < 225 || ranges[1] < 200 ) {
			if( ranges[0] < 200 || ranges[1] < 100 ) {
				System.out.println("<<<<<<Backing up!");
				this.targetDistance = -50;
				this.yawTargetDegrees = 0;
			} else {
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
	 * The ARDrone US seems to flatten everything to 0 when objects 200mm closer or less
	 * this is called by the chain or responsibility emanating from the cmd_vel topic.
	 * @throws IOException 
	 */
	@Override
	public boolean move2DRelative(float yawIMURads, int yawTargetDegrees, int targetDistance, float[] accelDeltas, int[] ranges) throws IOException {
		System.out.println("Motion control Move 2D Rel: "+yawTargetDegrees);
		synchronized( mutex ) {
			this.yawIMURads = yawIMURads;
			this.yawTargetDegrees = yawTargetDegrees;
			this.targetDistance = targetDistance;
			this.accelDeltas = accelDeltas;
			this.ranges = ranges;
		}
		if( yawTargetDegrees == 0 && targetDistance == 0) {
			System.out.println("0,0 command stop");
			commandStop();
			return false;
		}
		// if any deltas > about 100, there is lateral force sufficient to raise wheels
		// if any ranges close than about 200mm
		if( accelDeltas[0] > 500.0f || accelDeltas[1] > 500.0f || accelDeltas[2] > 500.0f) {
			System.out.println("MotorControl issuing stop based on accel deltas "+accelDeltas[0]+" "+accelDeltas[1]+" "+accelDeltas[2]);
				commandStop(); // pick up further motion on subsequent commands when accel deltas are nicer
				return false;
		} 
		// something in front, turn 30 degrees in place
		if( ranges[0] < 225 || ranges[1] < 200 ) {
			if( ranges[0] < 200 || ranges[1] < 100 ) {
				System.out.println("<<<<<<Backing up!");
				this.targetDistance = -50;
				this.yawTargetDegrees = 0;
			} else {
				System.out.println("MotorControl issuing 30 degr. rotation based on obstacle detection "+ranges[0]+" "+ranges[1]);
				if( yawTargetDegrees < 30 )
					this.yawTargetDegrees = 30;
			}
			this.targetDistance = 0;
		}
		System.out.println("MotorControl moving robot relative:"+this.yawIMURads+" "+this.yawTargetDegrees+" "+this.targetDistance+" "+this.targetTime);
		moveRobotRelative(this.yawIMURads, this.yawTargetDegrees, this.targetDistance);	
		return true;
	}


         
	class TwistInfo {
		float yawDegrees;
		float wheelTheta;/* change in theta calculated from wheels */
		float imuTheta; /* global theta from IMU, 2 PI radians */
		float deltaTheta;/* change in theta calculated from IMU, current imu - prev imu */
		float robotTheta;/* bot heading in radians minus theta_offset */
		float X; /* bot X position in mm */
		float Y;/* bot Y position in mm */
		public String toString() {
         return "Yaw d:"+yawDegrees+" Yaw target"+wheelTheta+
        		" IMU t:"+imuTheta+" Delta t:"+deltaTheta+" Robot t:"+robotTheta+" X:"+X+" Y:"+Y;
		}
	}
	/* A struct to hold Odometry info */
	class OdomInfo {
		long lastOdomTime;    // last millis() time odometry was calculated
		float linearX;	         // total linear x distance traveled
		float linearY;	         // total linear y distance traveled
		float angularZ;		 // total angular distance traveled
	}

	/* Setpoint Info For a Motor */
	class SetPointInfo {
		float TargetSpeed;            // target speed in m/s
		float targetMM;
		float TargetTicksPerFrame;    // target speed in ticks per frame
		float X;
		float prevX;
		int PrevErr;                   // last error
		int Ierror;                    // integrated error
		int output;                    // last motor setting
	}
	}



