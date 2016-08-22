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
	public synchronized int[] setMotorSpeed(float x, float th) throws IOException {

		//float spd_left, spd_right;
  
		/* Reset the auto stop timer */
		lastMotorCommand = System.currentTimeMillis();

		if (x == 0 && th == 0) {
			moving = false;
			if( DEBUG )
				System.out.println("STOP!!");
			return new int[]{ 0,0 };
		}
		/* Indicate that we are moving */
		moving = true;
		return new int[]{(int)x/scale, (int)th/scale};
	}
	
	
	private void setLeftForward() throws IOException {
		String motorCommand = "M41 P"+String.valueOf(leftDirPin); // forward, set pin high
		ByteSerialDataPort.getInstance().writeLine(motorCommand);
		if( DEBUG )
			System.out.println(motorCommand);
	}
	
	private void setRightForward() throws IOException {
		String motorCommand = "M41 P"+String.valueOf(rightDirPin); // forward, set pin high
		ByteSerialDataPort.getInstance().writeLine(motorCommand);
		if( DEBUG )
				System.out.println(motorCommand);

	}
	
	private void setLeftReverse() throws IOException {
		String motorCommand = "M42 P"+String.valueOf(leftDirPin); // reverse
		ByteSerialDataPort.getInstance().writeLine(motorCommand);
		if( DEBUG )
			System.out.println(motorCommand);
	}
	
	private void setRightReverse() throws IOException {
		String motorCommand = "M42 P"+String.valueOf(rightDirPin); // reverse
		ByteSerialDataPort.getInstance().writeLine(motorCommand);
		if( DEBUG )
			System.out.println(motorCommand);
	}

	@Override
	public void updateSpeed(int leftWheelSpeed, int rightWheelSpeed) throws IOException {
		if(leftSpeed <= 0) { // previous speed
			if( leftWheelSpeed > 0 ) // now going forward
				setLeftForward();
		} else {
			if(leftWheelSpeed <= 0 )
				setLeftReverse();
		}
		if(rightSpeed <= 0) { // previous speed
			if( rightWheelSpeed > 0 ) // now going forward
				setRightForward();
		} else {
			if( rightWheelSpeed <= 0)
				setRightReverse();
		}
		if(leftWheelSpeed < 0) leftWheelSpeed = -leftWheelSpeed;
		if(rightWheelSpeed < 0) rightWheelSpeed = -rightWheelSpeed;
		String motorCommand = "M45 P"+String.valueOf(leftPWMPin)+" S"+String.valueOf(leftWheelSpeed);
		ByteSerialDataPort.getInstance().writeLine(motorCommand);
		if( DEBUG )
			System.out.println(motorCommand);
		motorCommand = "M45 P"+String.valueOf(rightPWMPin)+" S"+String.valueOf(rightWheelSpeed);
		ByteSerialDataPort.getInstance().writeLine(motorCommand);

		leftSpeed = leftWheelSpeed;
		rightSpeed = rightWheelSpeed;
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
