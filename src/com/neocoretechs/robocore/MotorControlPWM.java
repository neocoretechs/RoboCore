package com.neocoretechs.robocore;

import java.io.IOException;

import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;
/**
 * This class extends MotorControl to provide a lower level command set for
 * PWM signals directed at a driver vs a motor controller.
 * We will be issuing signals to drive pins setting forward/reverse high/low
 * and PWM signals from a range of 0 to 255 with 0 being stop.
 * Instead of the high level G codes which send commands via UART
 * to a smart controller we will generate M codes (M41, M42, M45) to
 * directly drive pins on a 'dumb' H bridge driver.
 * @author jg
 *
 */
public class MotorControlPWM extends MotorControl {
	private static boolean DEBUG = true;
	private int leftSpeed = 0;
	private int rightSpeed = 0;
	private int leftPWMPin = 8;
	private int rightPWMPin = 10;
	private int leftDirPin = 24;
	private int rightDirPin = 22;
	private int scale = 4; // map the 0-1000 to 0-255
	
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
	@Override
	public synchronized void setMotorSpeed(float x, float th) throws IOException {

		float spd_left, spd_right;
  
		/* Reset the auto stop timer */
		lastMotorCommand = System.currentTimeMillis();

		if (x == 0 && th == 0) {
			moving = false;
			String motorCommand = "M45 P"+String.valueOf(leftPWMPin)+" S0";
			ByteSerialDataPort.getInstance().writeLine(motorCommand);
			if( DEBUG )
				System.out.println(motorCommand);
			motorCommand = "M45 P"+String.valueOf(rightPWMPin)+" S0";
			ByteSerialDataPort.getInstance().writeLine(motorCommand);
			if( DEBUG )
				System.out.println(motorCommand);
			return;
		}

		/* Indicate that we are moving */
		moving = true;

		if (x == 0) {
			if( th == 32767 ) {
				setForward();
				return;
			} else {
				if( th == -32767 ) {
					setReverse();
					return;
				}
			}
			// Turn in place, rotate the proper wheel forward, the other backward for a spin
			if( th < 0 ) { // left
				//spd_right = (float) (th * wheelTrack / 2.0);
				spd_right = th;
			} else {
				//spd_right = -((float) (th * wheelTrack / 2.0));
				spd_right = -th;
			}	
			spd_left = -spd_right;
		} else {
			if (th == 0) {	
				// Pure forward/backward motion
				spd_left = spd_right = x;
			} else {
				// Rotation about a point in space
				// Calculate Drive Turn output due to X input
				if (x > 0) {
				  // Forward
				  spd_left = ( th > 0 ) ? MAXOUTPUT : (MAXOUTPUT + th);
				  spd_right = ( th > 0 ) ? (MAXOUTPUT - th) : MAXOUTPUT;
				} else {
				  // Reverse
				  spd_left = (th > 0 ) ? (MAXOUTPUT - th) : MAXOUTPUT;
				  spd_right = (th > 0 ) ? MAXOUTPUT : (MAXOUTPUT + th);
				}

				// Scale Drive output due to X input (throttle)
				spd_left = spd_left * x / MAXOUTPUT;
				spd_right = spd_right *  x / MAXOUTPUT;
				
				// Now calculate pivot amount
				// - Strength of pivot (nPivSpeed) based on  X input
				// - Blending of pivot vs drive (fPivScale) based on Y input
				nPivSpeed = (int) th;
				// if th beyond pivlimit scale in 
				fPivScale = (float) ((Math.abs(x)>fPivYLimit)? 0.0 : (1.0 - Math.abs(x)/fPivYLimit));

				// Calculate final mix of Drive and Pivot
				/* Set the target speeds in meters per second */
				spd_left = (float) ((1.0-fPivScale)*spd_left + fPivScale*( nPivSpeed));
				spd_right = (float) ((1.0-fPivScale)*spd_right + fPivScale*(nPivSpeed));
			}
		}

		// Calculate final mix of Drive and Pivot
		// Set the target speeds in wheel rotation command units -1000, 1000 and if indoor mode div by ten
		leftWheel.TargetSpeed = indoor ? spd_left/10 : spd_left;
		rightWheel.TargetSpeed = indoor ? spd_right/10 : spd_right;
		if( DEBUG )
			System.out.println("Linear x:"+x+" angular z:"+th+" Motor L:"+leftWheel.TargetSpeed+" R:"+rightWheel.TargetSpeed);
		/* Convert speeds to ticks per frame */
		leftWheel.TargetTicksPerFrame = SpeedToTicks((float) leftWheel.TargetSpeed);
		rightWheel.TargetTicksPerFrame = SpeedToTicks((float) rightWheel.TargetSpeed);

		updateSpeed();
		}

	public void setForward() throws IOException {
		String motorCommand = "M41 P"+String.valueOf(leftDirPin); // forward, set pin high
		ByteSerialDataPort.getInstance().writeLine(motorCommand);
		if( DEBUG )
			System.out.println(motorCommand);
		motorCommand = "M41 P"+String.valueOf(rightDirPin); // forward, set pin high
		ByteSerialDataPort.getInstance().writeLine(motorCommand);
		if( DEBUG )
				System.out.println(motorCommand);
	
	}
	
	public void setReverse() throws IOException {
		String motorCommand = "M42 P"+String.valueOf(leftDirPin); // reverse
		ByteSerialDataPort.getInstance().writeLine(motorCommand);
		if( DEBUG )
			System.out.println(motorCommand);
		motorCommand = "M42 P"+String.valueOf(rightDirPin); // reverse
		ByteSerialDataPort.getInstance().writeLine(motorCommand);
		if( DEBUG )
				System.out.println(motorCommand);
	}
	@Override
	protected synchronized void updateSpeed() throws IOException {

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
		//if( DEBUG )
		//	System.out.println("Motor:"+leftWheel.TargetSpeed+" "+rightWheel.TargetSpeed);
		//
		// we invert channel 1 since its a mirror of the orientation of channel 2
		// WE ONLY NEED THIS WHERE THE MOTOR WAS NOT DESIGNED FOR THE SIDE ITS ON
		// BUT FLIPPED FROM THE OTHER SIDE
		//leftWheel.TargetSpeed = -leftWheel.TargetSpeed;
		//
		// scale the usual motor range of -1000 to 1000 to 0 to 255 with pin state changes for direction
		leftWheel.TargetSpeed /= scale;
		rightWheel.TargetSpeed /= scale;
		if( leftWheel.TargetSpeed < 0 ) { // reverse
			//String motorCommand = "M42 P"+String.valueOf(leftDirPin); // reverse
			//ByteSerialDataPort.getInstance().writeLine(motorCommand);	
			// reverse and currently reversing
			String motorCommand = "M45 P"+String.valueOf(leftPWMPin)+" S"+String.valueOf((int)-leftWheel.TargetSpeed);
			ByteSerialDataPort.getInstance().writeLine(motorCommand);
			if( DEBUG )
				System.out.println(motorCommand);
		} else {
			 // forward/stop
				//String motorCommand = "M41 P"+String.valueOf(leftDirPin); // forward, set pin high
				//ByteSerialDataPort.getInstance().writeLine(motorCommand);
				//if( DEBUG )
				//	System.out.println(motorCommand);
				// forward and currently forwarding
				String motorCommand = "M45 P"+String.valueOf(leftPWMPin)+" S"+String.valueOf((int)leftWheel.TargetSpeed);
				ByteSerialDataPort.getInstance().writeLine(motorCommand);
				if( DEBUG )
					System.out.println(motorCommand);
			
		}
		
		if( rightWheel.TargetSpeed < 0 ) {
			//String motorCommand = "M42 P"+String.valueOf(rightDirPin); // reverse
			//ByteSerialDataPort.getInstance().writeLine(motorCommand);
			//if( DEBUG )
			//		System.out.println(motorCommand);
			String motorCommand = "M45 P"+String.valueOf(rightPWMPin)+" S"+String.valueOf((int)-rightWheel.TargetSpeed);
			ByteSerialDataPort.getInstance().writeLine(motorCommand);
			if( DEBUG )
				System.out.println(motorCommand);
		} else {
				//String motorCommand = "M41 P"+String.valueOf(rightDirPin); // forward, set pin high
				//ByteSerialDataPort.getInstance().writeLine(motorCommand);
				//if( DEBUG )
				//		System.out.println(motorCommand);
				String motorCommand = "M45 P"+String.valueOf(rightPWMPin)+" S"+String.valueOf((int)rightWheel.TargetSpeed);
				ByteSerialDataPort.getInstance().writeLine(motorCommand);
				if( DEBUG )
					System.out.println(motorCommand);
		}
		// store current speed values
		leftSpeed = (int) leftWheel.TargetSpeed;
		rightSpeed = (int) rightWheel.TargetSpeed;
	}
	
	@Override
	protected void init() {
		super.init();	
		wheelDiameter = 300.0f; // millimeters, 12"
		wheelTrack = 500.0f; // millimeters, 20"
		ticksPerMeter = (float) (cpr / (Math.PI * wheelDiameter));
		indoor = false;
	}
}
