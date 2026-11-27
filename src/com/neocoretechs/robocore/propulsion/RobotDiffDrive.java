package com.neocoretechs.robocore.propulsion;

import java.io.IOException;
import java.io.Serializable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.stream.Collectors;

import com.neocoretechs.robocore.PID.MotionPIDController;
import com.neocoretechs.robocore.config.DeviceEntry;
import com.neocoretechs.robocore.config.RobotInterface;
import com.neocoretechs.robocore.config.SlotEntry;
import com.neocoretechs.robocore.config.TypedWrapper;

import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;

import com.neocoretechs.robocore.marlinspike.MarlinspikeControlInterface;
import com.neocoretechs.robocore.marlinspike.TypeSlotChannelEnable;
import com.neocoretechs.robocore.marlinspike.TypeSlotChannelEnable.typeNames;

import com.neocoretechs.robocore.propulsion.MotionController.InternalPayload;

/**
* Configuration for generic robot differential drive.
* Based on the Marlinspike specification, there can be up to 10 AbstractMotorControl "slots".<p>
* Each "slot" can have up to 10 PWM pins (wheels) defined<p>
* So if you are powering an Ogre from the famous microgame "GEV", you can have 5 track banks on each side of the tank with each bank containing 10 tracks<p>
* This would translate to slots 0-9 each of which has channels 1-10 <p>
* However on the pico you are limited to 8 "slices" for PWM designations each with two A and B channels. so you
* would have to power 5 tracks on each side of the Ogre and switch them as they got knocked out by GEVs or
* artillery.<p>
* typical uae pattern - if(activation(InternalPayload) == 0) int[] lr = calcMotorPower(euler, payload); commandMotorPower(lr[0], lr[1]) 
* @author Jonathan Groff (C) NeoCoreTechs 2020,2021
*/
public class RobotDiffDrive extends SlotEntry implements RobotDiffDriveInterface, Serializable {
	public static boolean DEBUG = false;
	private static final long serialVersionUID = 1L;
	RobotInterface robot;
	TypedWrapper[] LUN;
	TypedWrapper[] WHEEL;
	TypedWrapper[] AXIS;
	TypedWrapper[] PID;
	int channels = 0;
	private boolean shouldMove = true;
	private int targetPitch;
	private int targetDist;
	private float targetYaw;
	public static float WHEELBASE = 1000.0f;
	private boolean DEBUGBEARING;
	private float compassHeading;
	
	// slope speed change over time analysis
	private CircularBlockingDeque<Float> speedQueueL = new CircularBlockingDeque<Float>(5);
	private CircularBlockingDeque<Float> speedQueueR = new CircularBlockingDeque<Float>(5);
	private float maxSpeedSlope = 100;
	static boolean wasPid = true; // used to determine crossing from geometric to PID to preload integral windup
	static boolean holdBearing = false; // hold steering to current bearing
	static boolean outputNeg = false; // used to prevent integral windup by resetting ITerm when crossing track
	/**
	 * Controller, devicename, slot goes into the initial slothandler entry, which is first wheel encountered.
	 * This sets up the sort order in the deviceToType mapping based on nodename, name
	 * @param deviceName in this case topic name
	 * @param NodeName
	 * @param LUN last LUN
	 * @param controller
	 * @param slot
	 */
	public RobotDiffDrive(RobotInterface robot2, String deviceName, String NodeName, int LUN, MarlinspikeControlInterface controller, int slot, TypedWrapper[] lUN, TypedWrapper[] wHEEL, TypedWrapper[] aXIS, TypedWrapper[] pID) {
		super(robot2, deviceName, NodeName, LUN, controller, slot);
		this.robot = robot2;
		this.LUN = lUN;
		this.WHEEL = wHEEL;
		this.AXIS = aXIS;
		this.PID = pID;
		Object[] wheels = getWheels().toArray();
		// wheelbase refers to max speed, both wheels averaged channels 1 and 2, 1 left, 2 right
		WHEELBASE = (((DrivenWheelInterface) wheels[0]).getSpeedsetPointInfo().getMaximum()+((DrivenWheelInterface) wheels[1]).getSpeedsetPointInfo().getMaximum())/2.0f;
	}

	/**
	 * Static helper method to parse and populate the fields for a {@link RobotWheel} instance.
	 * @param LUN {@link TypedWrapper} for LUN properties
	 * @param WHEEL wheel props
	 * @param PID pid props
	 * @param name Name property to match
	 * @param wheelLun LUN index to use
	 * @param slot slot prop
	 * @param chan channel prop
	 * @return the instantiated RobotWheel
	 */
	public static RobotWheel setDrive(TypedWrapper[] LUN, TypedWrapper[] WHEEL, TypedWrapper[] PID, String name, int wheelLun, int slot, int chan) {
		Optional<Float> wheelTrack = Optional.empty();// Props.toFloat("WheelTrackMM"); // millimeters
		Optional<Float> wheelDiameter = Optional.empty(); //Props.toFloat("WheelDiameterMM"); // millimeters, 16"
		Optional<Integer> ticksPerRevolution = Optional.empty();
		Optional<Float> motorKp = Optional.empty();
		Optional<Float> motorKd = Optional.empty();
		Optional<Float> motorKi = Optional.empty();
		Optional<Float> motorKo = Optional.empty();
		Optional<Integer> motorPIDRate = Optional.empty();
		Optional<Integer> minSpeed = Optional.empty();
		Optional<Integer> maxSpeed = Optional.empty();
		Optional<String> type = Optional.empty();
		Optional<Integer> enablePin = Optional.empty();
		
		type = Optional.ofNullable(LUN[wheelLun].get("Type")).filter(String.class::isInstance).map(e -> String.valueOf(e));
		if(!type.get().equals("SmartController"))
			enablePin = Optional.ofNullable(LUN[wheelLun].get("EnablePin")).filter(String.class::isInstance)
				.map(e -> Integer.parseInt((String) e));
		
		minSpeed = Optional.ofNullable(LUN[wheelLun].get("MinValue")).filter(String.class::isInstance)
				.map(e -> Integer.parseInt((String) e));
		maxSpeed = Optional.ofNullable(LUN[wheelLun].get("MaxValue")).filter(String.class::isInstance)
				.map(e -> Integer.parseInt((String) e));

		wheelTrack = Optional.ofNullable(WHEEL[wheelLun].get("WheelTrackMM")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		wheelDiameter = Optional.ofNullable(WHEEL[wheelLun].get("WheelDiameterMM")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		ticksPerRevolution = Optional.ofNullable(WHEEL[wheelLun].get("TicksPerRevolution")).filter(String.class::isInstance)
				.map(e -> Integer.parseInt((String) e));

		motorKp = Optional.ofNullable(PID[wheelLun].get("MotorKp")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		motorKd = Optional.ofNullable(PID[wheelLun].get("MotorKd")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		motorKi = Optional.ofNullable(PID[wheelLun].get("MotorKi")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		motorKo = Optional.ofNullable(PID[wheelLun].get("MotorKo")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		motorPIDRate = Optional.ofNullable(PID[wheelLun].get("MotorPIDRate")).filter(String.class::isInstance)
				.map(e -> Integer.parseInt((String) e));

		try {
			typeNames eType = null;
			//eType = typeNames.valueOf(type) will fail because we use mixed case names;
			for(typeNames et : typeNames.values()) {
				if(et.name.equals((String)type.get())) {
					eType = et;
					break;
				}
			}
			if(eType == null)
				throw new RuntimeException("Type of:"+(String)type.get()+" did not resolve to a legitimate type for "+name+" slot "+slot+" channel "+chan);
			
			return new RobotWheel(name, eType, slot, chan, (type.get().equals("SmartController") ? -1 : enablePin.get()),
					wheelDiameter.get(), 
					wheelTrack.get(), 
					minSpeed.get(), 
					maxSpeed.get(), 
					ticksPerRevolution.get(), 
					motorKp.get(), motorKd.get(), 
					motorKi.get(), motorKo.get(),
					motorPIDRate.get());
		} catch(NoSuchElementException nse) {
			if(!type.get().equals("SmartController"))
				System.out.println("enable="+enablePin.get());
			System.out.println("wheelDiameter="+wheelDiameter.get());
			System.out.println("wheelTrack="+wheelTrack.get());
			System.out.println("minSpeed="+minSpeed.get());
			System.out.println("maxSpeed="+maxSpeed.get());
			System.out.println("ticksPerRevolution="+ticksPerRevolution.get());
			System.out.println("motorKp="+motorKp.get());
			System.out.println("motorKd="+motorKd.get());
			System.out.println("motorKi="+motorKi.get());
			System.out.println("motorKo="+motorKo.get());
			System.out.println("motorPIDRate="+motorPIDRate.get());
			throw new RuntimeException("Cannot configure robot wheel "+name+" LUN "+wheelLun+" slot:"+slot+" channel:"+chan);
		}
	}
	
	@Override
	/**
	 * Return the wheel channels for a given slot
	 * @param slot target slot
	 * @return ArrayList of channels of DriveWheelInterface for this slot or null if slot wasnt found
	 */
	public ArrayList<DrivenWheelInterface> getWheels(int slot) {
		ArrayList<TypeSlotChannelEnable> atsce = robot.getManager().getSlot(slot);
		if(atsce == null)
			return null;
		ArrayList<DrivenWheelInterface> rslot = null;
		if(atsce.getFirst() instanceof DrivenWheelInterface) {
			rslot = new ArrayList<DrivenWheelInterface>(atsce.size());
			for(TypeSlotChannelEnable tsce: atsce)
				rslot.add((RobotWheel)tsce);
		}
		return rslot;
	}

	/**
	 * Get the wheel for the given slot row and channel
	 * @param slot The array of channels for this slot from getWheels
	 * @param chan the target channel
	 * @return RobotWheel or null if slots is null or channel cant be found in row for slot.
	 */
	public static DrivenWheelInterface getWheel(ArrayList<DrivenWheelInterface> slot, int chan) {
		if(slot == null)
			return null;
		if(chan <= 0 || chan > 10)
			throw new RuntimeException("Channel must be from 1-10");
		for(int rsl2 = 0; rsl2 < slot.size(); rsl2++) {
			if(slot.get(rsl2).getChannel() == chan)
				return slot.get(rsl2);
		}
		return null;
	}
	
	/**
	 * Handle button pivot, e-stop etc here.
	 * recall, matches() scales value by 1000, PowerScale reduces in Marlinspike pico code
	 * @param payload flow published button/axis
	 * @return 0 - no action, 1 softStop, 2 left pivot, 3 right pivot
	 * @throws IOException 
	 */
	public int activation(InternalPayload payload) throws IOException {
		// check for emergency stop, on X or A or green or lower button
		int[] buttons = payload.getButtons();
		if( buttons[6] != 0 ) {
			softStop();
			return 1;
		}
		if( buttons[7] != 0 ) { // left pivot
			Integer[] xy = (Integer[]) getManualControl().matches(payload.getAxis());
			commandMotorPower( xy[1], -xy[1]);
			if(DEBUG)
				System.out.printf("Stick Left pivot speedL=%d|speedR=%d\n",xy[1],xy[1]);
			return 2;
		} else {
			if( buttons[5] != 0 ) { // right pivot
				Integer[] xy = (Integer[]) getManualControl().matches(payload.getAxis());
				commandMotorPower(-xy[1], xy[1]);
				if(DEBUG)
					System.out.printf("Stick Right pivot speedL=%d|speedR=%d\n",-xy[1],xy[1]);
				return 3;
			}
		}
		Integer[] xy = (Integer[]) getManualControl().matches(payload.getAxis());
		int[] motorPower = calcMotorPower(xy, compassHeading, (buttons[4] != 0));
		commandMotorPower(motorPower[0],motorPower[1]);
		return 0;
	}
	
	@Override
	public void commandMotorPower(int speedL, int speedR) throws IOException {
		getMarlinspikeControl().setDeviceLevels(getTopicName(), new int[] {speedL, speedR});
	}
	
	private void softStop() {
		robot.getManager().getDevices().keySet().forEach(e-> {
			DeviceEntry ndd = (DeviceEntry)e;
			try {
				ndd.getMarlinspikeControl().commandStop();
			} catch (IOException ex) {
				throw new RuntimeException(ex);
			}
		});
		robot.getOperating().replaceAll((key, value) -> false);
	}
	/**
	 * If slope is negative and coordinate is neg acceleration in reverse
	 * If slope is pos and coord is negative reverse deceleration
	 * 
	 * @param speed
	 * @param speedQueue
	 * @return
	 */
	private float slope(float speed, CircularBlockingDeque<Float> speedQueue) {
		speedQueue.addLast(speed);
		int n = speedQueue.length();
		double m, c, sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
		for (int i = 0; i < n; i++) {
			sum_x += i;//x[i];
			sum_y += speedQueue.get(i);//[i];
			sum_xy += i * speedQueue.get(i);//[i] * y[i];
			sum_x2 += Math.pow(i, 2);//pow(x[i], 2);
		}
		m = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - Math.pow(sum_x, 2));
		//c = (sum_y - m * sum_x) / n;
		//System.out.println("m = " + m);
		//System.out.println("c = " + c);
		if(m == 0)
			return speedQueue.get(n-1);
		if(m < 0) {
			if(m < -maxSpeedSlope) {
				if(n == 1) {
					speedQueue.set(0,speedQueue.get(1)/2); // divide it in half, we only have 1 point
				} else {
					// slope neg and coord neg, reverse accel
					if(speedQueue.get(n-1) < 0) {
						// diff of last point and next to last
						float diff = speedQueue.get(n-1)-speedQueue.get(n-2);
						// split the diff
						speedQueue.set(n-1, speedQueue.get(n-2)+(diff/2.0f));
					} else {
						// slope neg and coord pos, forward decel
						// diff of next to last and last
						float diff = speedQueue.get(n-2)-speedQueue.get(n-1);
						// split the diff
						speedQueue.set(n-1, speedQueue.get(n-2)-(diff/2.0f));
					}
				}
			}
		} else {
			// slope pos
			if(m > maxSpeedSlope) {
				if(n == 1) {
					speedQueue.set(0,speedQueue.get(1)/2); // divide it in half, we only have 1 point
				} else {
					// slope pos and coord neg, reverse decel
					if(speedQueue.get(n-1) < 0) {
						// diff of next to last and last
						float diff = speedQueue.get(n-2)-speedQueue.get(n-1);
						// split the diff
						speedQueue.set(n-1, speedQueue.get(n-2)-(diff/2.0f));
					} else {
						// slope pos and coord pos, forward accel
						// diff of last and next to last
						float diff = speedQueue.get(n-1)-speedQueue.get(n-2);
						// split the diff
						speedQueue.set(n-1, speedQueue.get(n-2)+(diff/2.0f));
					}
				}
			}
		}
		return speedQueue.get(n-1);
	}

	/**
	 * 		
	 * If holding an inertially guided course, check for the tolerance of error proportion
	 * from setpoint to current heading, if within a lower tolerance, use the pid values of the
	 * currently computed course deviation to
	 * alter the motor speed to reduce cross track error and become critically damped.
	 * If we are at the higher tolerance, compute a solution based on triangles that brings us on a tangent course
	 * to the crosstrack. The base of a right triangle is the speed plus half wheelbase, the hypotenuse is computed from
	 * the chord of the magnitude error offset from track and the base (speed). The ratio of base to hypotenuse is used
	 * based on required distance traveled for each wheel and the opposite wheel is slowed by that proportion
	 * just as we do for a solution of arcs.
	 * The chord leg of the triangle is computed based on half wheelbase as radius via 2*R*sin(theta/2) with theta
	 * being crosstrack deviation.
	 * @param euler euler packet from motion control
	 * @param radius
	 * @param arcin
	 * @param arcout
	 * @param speedL left speed
	 * @param speedR right speed
	 * @return left/right speed calculated
	 */
	private float[] followIMUSetpoint(float compassHeading, float radius, float arcin, float arcout, float speedL, float speedR) {
		if(DEBUG || DEBUGBEARING)
			System.out.printf("Inertial Setpoint=%f | Hold=%b ", robot.getIMUSetpointInfo().getDesiredTarget(), holdBearing);
		// In auto
		// Point at which geometric solution begins/disengages is IMU maximumCourseDeviation from configuration
		float PID_THRESHOLD  = robot.getIMUSetpointInfo().getMaximum()/2; // point at which PID engages/disengages
		if( Math.abs(robot.getMotionPIDController().getError()) <= robot.getIMUSetpointInfo().getMaximum() ) {
			if( robot.getMotionPIDController().getError() < 0.0f ) { // decrease left wheel power goal
				// If in lower bound use PID, between lower and middle use triangle solution, above that use arc
				if( Math.abs(robot.getMotionPIDController().getError()) <= PID_THRESHOLD ) {
					speedL = rightPid(radius, speedL);
				} else {
					speedL = rightAngle(radius, speedL);
				}
			} else {
				if( robot.getMotionPIDController().getError() > 0.0f ) { // decrease right wheel power goal
					if( robot.getMotionPIDController().getError() <= PID_THRESHOLD) {
						speedR = leftPid(radius, speedR);
					} else {
						speedR = leftAngle(radius, speedR);
					}
				} else {
					// we are critically damped, set integral to 0
					//ITerm = 0;
					robot.getMotionPIDController().setITerm(0);
				}
			}
			if(DEBUG || DEBUGBEARING)
				System.out.printf("<="+robot.getIMUSetpointInfo().getMaximum()+" degrees Speed=%f|IMU=%f|speedL=%d|speedR=%d|Hold=%b\n",radius,compassHeading,(int)speedL,(int)speedR,holdBearing);
		} else {
			// Exceeded tolerance of triangle solution, proceed to polar geometric solution in arcs
			robot.getMotionPIDController().setITerm(0);//ITerm = 0;
			wasPid = false;
			arcin = (float) (Math.abs(robot.getMotionPIDController().getError()/360.0) * (2.0 * Math.PI) * radius);
			arcout = (float) (Math.abs(robot.getMotionPIDController().getError()/360.0) * (2.0 * Math.PI) * (radius + WHEELBASE));
			// error is difference in degrees between setpoint and input
			if( robot.getMotionPIDController().getError() < 0.0f )
				speedL *= (arcin/arcout);
			else
				if( robot.getMotionPIDController().getError() > 0.0f )
					speedR *= (arcin/arcout);
			if(DEBUG || DEBUGBEARING)
				System.out.printf(">"+robot.getIMUSetpointInfo().getMaximum()+" degrees Speed=%f|IMU=%f|arcin=%f|arcout=%f|speedL=%d|speedR=%d|Hold=%b\n",radius,compassHeading,arcin,arcout,(int)speedL,(int)speedR,holdBearing);
		}
		return new float[] {speedL, speedR};
	}
	/**
	 * Increase power of right wheel based on PID results
	 * If we hit max we are going to decrease power to left wheel by splitting the difference
	 * of leftover speed on right applied to left wheel decrease. whether these are equivalent in reality is
	 * left to evaluation.
	 * @param speed the scaled velocity
	 */
	private float rightPid(float speed, float speedL) {
		/*
		if( !outputNeg ) { // prevent integral windup by checking when track is crossed via sign of course diff changing
			ITerm = 0;
		}
		outputNeg = true;
		// see if we are entering from the geometric realm, if so preload integral
		if( !wasPid ) {
			wasPid = true;
			// get rid of old integral and replace with our preload windup
			// Since we are in negative offsets we will subtract the maximum offset that is threshold value 
			pidOutput = PTerm + DTerm + (-PID_THRESHOLD * ki);
			// Now set up integral to reflect transition state
			ITerm = -PID_THRESHOLD;
		}
		 */
		// Reduce speed of left wheel to affect turn, speed reduction may result in negative values turning wheel backwards if necessary
		// scale it by speed
		speedL -= ( Math.abs(robot.getMotionPIDController().getOutput()) * (speed/350) );
		/*
		// goal is to increase power of right wheel
		// We scale it by speed with the assumption that at full speed,
		// the values from PID represent the ones necessary to work at top speed of 1000 or SPEEDLIMIT.
		speedR += ( Math.abs(pidOutput) * (Math.abs(axes[2])*3.2) );
		// if we cap at max, use the difference to retard opposite
		if( speedR > SPEEDLIMIT ) {
			// over limit, split the difference
			float speeDif = speedR - SPEEDLIMIT;
			speedR = SPEEDLIMIT;
			// left is decreased by the amount right went over max
			speedL -= speeDif;
			if( speedL < 0.0 ) speedL = 0.0f;
			// and the robot is pivoting at max speed if everything was capped
		}
		 */
		return speedL;
	}
	/**
	 * Increase power of left wheel based on PID results. If we max out we are going to apply the leftover
	 * power over maximum to decreasing the right wheel speed.
	 * @param speed The scaled velocity
	 */
	private float leftPid(float speed, float speedR) {
		/*
		if( outputNeg ) {
			ITerm = 0;
		}
		outputNeg = false;
		// see if we are entering from the geometric realm, if so preload integral
		if( !wasPid ) {
			wasPid = true;
			// get rid of old integral and replace with our preload windup
			// Since we are in positive offsets we will add the maximum threshold value for windup.
			pidOutput = PTerm + DTerm + (PID_THRESHOLD * ki);
			// Now set up integral to reflect transition state
			ITerm = PID_THRESHOLD;
		}
		 */
		// Reduce speed of right wheel to affect turn, speed reduction may result in negative values turning wheel backwards if necessary
		// scale it by speed
		speedR -= ( Math.abs(robot.getMotionPIDController().getOutput()) * (speed/350) );
		/*
		// goal is net increase of left wheel power
		// We scale it by speed with the assumption that at full speed,
		// the values from PID represent the ones necessary to work at top speed of 1000 or SPEEDLIMIT.
		speedL +=( Math.abs(pidOutput) * (Math.abs(axes[2])*3.2) );
		if( speedL > SPEEDLIMIT ) {
			// over limit, split the difference
			float speeDif = speedL - SPEEDLIMIT;
			speedL = SPEEDLIMIT;
			speedR -= speeDif;
			if( speedR < 0.0) speedR = 0.0f;
		}
		 */
		return speedR;
	}
	/**
	 * Apply a solution in triangles as we do with solution in arcs to reduce left wheel speed.
	 * This presupposes we are within tolerance.
	 * @param radius velocity-based value of turn radius component
	 */
	private float rightAngle(float radius, float speedL) {
		// Mitigate integral windup
		robot.getMotionPIDController().setITerm(0);//ITerm = 0;
		wasPid = false;
		// compute chord of course deviation, this is short leg of triangle.
		// chord is 2Rsin(theta/2) ( we convert to radians first)
		double chord = 2 * ((WHEELBASE/2)*2.7) * Math.sin((Math.abs(robot.getMotionPIDController().getError()/360.0) * (2.0 * Math.PI))/2);
		// make the base of the triangle radius , the do a simple pythagorean deal and take the 
		// square root of sum of square of base and leg
		double hypot = Math.sqrt((chord*chord) + ((radius+(WHEELBASE/2))*(radius+(WHEELBASE/2))));
		// decrease the power on the opposite side by ratio of base to hypotenuse, since one wheel needs to 
		// travel the length of base and the other needs to travel length of hypotenuse
		if(DEBUG)
			System.out.printf(" RIGHTANGLE=%f|chord=%f|hypot=%f|radius/hypot=%f|Hold=%b\n",radius,chord,hypot,((radius+(WHEELBASE/2))/hypot),holdBearing);
		speedL *= ((radius+(WHEELBASE/2))/hypot);
		return speedL;
	}
	/**
	 * Apply a solution in triangles as we do with solution in arcs to reduce right wheel speed.
	 * This presupposes we are within tolerance.
	 * @param radius Velocity based component of turning radius
	 */
	private float leftAngle(float radius, float speedR) {
		// Mitigate integral windup
		robot.getMotionPIDController().setITerm(0);//ITerm = 0;
		wasPid = false;
		// compute chord of course deviation, this is short leg of triangle.
		// chord is 2Rsin(theta/2), R is half wheeltrack so we assume our robot is roughly 'square'
		double chord = 2 * ((WHEELBASE/2)*2.7) * Math.sin((Math.abs(robot.getMotionPIDController().getError()/360.0) * (2.0 * Math.PI))/2);
		// make the base of the triangle radius , the do a simple pythagorean deal and take the 
		// square root of sum of square of base and leg
		double hypot = Math.sqrt((chord*chord) + ((radius+(WHEELBASE/2))*(radius+(WHEELBASE/2))));
		// decrease the power on the opposite side by ratio of base to hypotenuse, since one wheel needs to 
		// travel the length of base and the other needs to travel length of hypotenuse
		if(DEBUG)
			System.out.printf(" LEFTANGLE=%f|chord=%f|hypot=%f|Hold=%b\n",radius,chord,hypot,holdBearing);
		speedR *= ((radius+(WHEELBASE/2))/hypot);
		return speedR;
	}

	/**
	 * Attempt to travel the target distance in the target time arriving at a final target degrees on the IMU
	 * Here, targetDegrees can be negative
	 * when the IMU and target coincide distance and time produce linear motion
	 * otherwise when turning distance determines the radius of the arc segment described by the difference
	 * of the current IMU and target and time becomes roughly the speed at which to make such traversal
	 * @param yawIMUDegrees Yaw is delivered in degrees
	 * @param yawTargetDegrees target final pose is in degrees for easier human input and represents motion relative to current pose vs absolute position
	 * @param targetDistance target distance to travel in CM, if 0 perform a pivot
	 * @return the left and right integer speed array
	 * @throws IOException 
	 */
	public synchronized int[] moveRobotRelative(float yawIMUDegrees, float yawTargetDegrees, int targetDistance) throws IOException {
		//leftWheel.targetMM = (float)(left_velocity) / clicks_per_mm;
		//rightWheel.targetMM = (float)(right_velocity) / clicks_per_mm;
		//avg_mm = (float) ((leftWheel.targetMM + rightWheel.targetMM) / 2.0);
		//total_mm += avg_mm;
		//wheel_theta is travel of each wheel with desired velocity, if same angle is 0, forward
		//twistInfo.wheelTheta = (leftWheel.targetMM - rightWheel.targetMM) / wheelTrack;
		//float yawDegrees = 0.0f;
		//float deltaTheta = 0.0f;
		float wheelTheta = 0.0f;
		float imuTheta = 0.0f;
		float robotTheta = 0.0f;
		wheelTheta = ((float)yawTargetDegrees * 0.0174532925f); // degr to radians
		/* read the YAW value from the imu struct and convert to radians */
		imuTheta = ((float) (yawIMUDegrees * 0.0174532925f));
		// if final theta negative, turn left
		robotTheta = imuTheta - wheelTheta;
		robotTheta = (float) Math.atan2(Math.sin(robotTheta), Math.cos(robotTheta));
		/* calculate rotation rate-of-change
		deltaTheta = imuTheta - last_theta;
		last_theta = imuTheta;
		robotTheta = imuTheta + wheelTheta;
		// this will generate the value to turn us properly
		if( robotTheta > TWOPI )
			robotTheta -= TWOPI;
		if( robotTheta < 0)
			robotTheta += TWOPI;
		float X =  (float)(targetDistance * Math.sin(robotTheta)); 
		float Y = (float)(targetDistance * Math.cos(robotTheta)); 
		*/
		// so X is the arc radius, normally we will set as a call to
		// setMotorSpeed as 0 to turn in place or a value of arc radius
		// modified by current speed to make a gentle curve.
		// if theta is 0, move linear in x
		return setMotorArcSpeed(targetDistance, robotTheta);
	}

	/**
	 * The function computes motor speeds. Channels are hardwired 1 left 2 right per config and we assume slot 0 is the first in the collection
	 * additional wheel and channels will reflect these.
	 * results in leftWheel.targetSpeed and rightWheel.targetSpeed
	 * so X is the arc radius, normally we will set as a call to
	 * setMotorSpeed as 0 to turn in place or a value of arc radius
	 * modified by current speed to make a gentle curve.
	 * if theta is 0, move linear in x
	 * if x and theta not 0 its rotation about a point in space
	 * @param x The radius from the center point of the arc about which we are rotating, if 0 turn in place
	 * @param th The 2PI polar measure of arc segment we are traversing, if 0 pure forward/back motion
	 * @return int 2 element array of final wheel speed left, right
	 * @throws IOException 
	 */
	private synchronized int[] setMotorArcSpeed(float x, float th) throws IOException {
		DrivenWheelInterface[] wheels = new DrivenWheelInterface[2];
		getWheels().toArray(wheels);
		float spd_left, spd_right;

		/* Reset the auto stop timer */
		//motorControl.lastMotorCommand = System.currentTimeMillis();

		/* Indicate that we are moving */
		//moving = true;

		if (x == 0) {
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
					spd_left = ( th > 0 ) ? wheels[0].getSpeedsetPointInfo().getMaximum() : (wheels[0].getSpeedsetPointInfo().getMaximum() + th);
					spd_right = ( th > 0 ) ? (wheels[1].getSpeedsetPointInfo().getMaximum() - th) : wheels[1].getSpeedsetPointInfo().getMaximum();
				} else {
					// Reverse
					spd_left = (th > 0 ) ? (wheels[0].getSpeedsetPointInfo().getMaximum() - th) : wheels[0].getSpeedsetPointInfo().getMaximum();
					spd_right = (th > 0 ) ? wheels[1].getSpeedsetPointInfo().getMaximum() : (wheels[1].getSpeedsetPointInfo().getMaximum() + th);
				}

				// Scale Drive output due to X input (throttle)
				spd_left = spd_left * x / wheels[0].getSpeedsetPointInfo().getMaximum();
				spd_right = spd_right * x / wheels[1].getSpeedsetPointInfo().getMaximum();

				// Now calculate pivot amount
				// - Strength of pivot (nPivSpeed) based on  X input
				// - Blending of pivot vs drive (fPivScale) based on Y input
				float fPivYLimit = 250.0f;
				int nPivSpeed = (int) th;
				// if th beyond pivlimit scale in 
				float fPivScale = (float) ((Math.abs(x)>fPivYLimit)? 0.0 : (1.0 - Math.abs(x)/fPivYLimit));

				// Calculate final mix of Drive and Pivot
				/* Set the target speeds in meters per second */
				spd_left = (float) ((1.0-fPivScale)*spd_left + fPivScale*( nPivSpeed));
				spd_right = (float) ((1.0-fPivScale)*spd_right + fPivScale*(-nPivSpeed));
			}
		}

		// Calculate final mix of Drive and Pivot
		// Set the target speeds in wheel rotation command units 
		wheels[0].getSpeedsetPointInfo().setTarget(spd_left);
		wheels[1].getSpeedsetPointInfo().setTarget(spd_left);
		if( DEBUG )
			System.out.println("Linear x:"+x+" angular z:"+th+" Motor L:"+wheels[0].getSpeedsetPointInfo().getTarget()+
					" R:"+wheels[1].getSpeedsetPointInfo().getTarget());
		/* Convert speeds to ticks per frame */
		//robot.getDiffDrive().getLeftWheel().getSpeedsetPointInfo().SpeedToTicks(robot.getLeftMotorPIDController(), robot.getDiffDrive().getLeftWheel());
		//robot.getDiffDrive().getRightWheel().getSpeedsetPointInfo().SpeedToTicks(robot.getRightMotorPIDController(), robot.getDiffDrive().getRightWheel());
		//leftWheel.TargetTicksPerFrame = SpeedToTicks((float) leftWheel.TargetSpeed);
		//rightWheel.TargetTicksPerFrame = SpeedToTicks((float) rightWheel.TargetSpeed);
		/* Read the encoders */
		//leftWheel.Encoder = 0;//encoders.YAxisGetCount();
		//rightWheel.Encoder = 0;//encoders.XAxisGetCount();

		/* Record the time that the readings were taken */
		//robot.getDiffDrive().odomInfo.lastOdomTime = System.currentTimeMillis();
		//odomInfo.encoderStamp = nh.now;

		/* Compute PID update for each motor */
		wheels[0].getPIDController().Compute(wheels[0].getSpeedsetPointInfo());
		wheels[1].getPIDController().Compute(wheels[1].getSpeedsetPointInfo());

		/* Set the motor speeds accordingly */
		//if( DEBUG )
		//	System.out.println("Motor:"+leftWheel.TargetSpeed+" "+rightWheel.TargetSpeed);
		// call to motor controller to update speed using absolute terms
		//motorControl.updateSpeed(slot1, channel1, (int)leftWheel.TargetSpeed, slot2, channel2, (int)rightWheel.TargetSpeed);
		return new int[]{(int) wheels[0].getSpeedsetPointInfo().getTarget(),(int) wheels[1].getSpeedsetPointInfo().getTarget()};
	}
	/**
	 * button[4] ! = 0 for hold bearing
	 * compassHeading = euler.compassHeading
	 * @param euler
	 * @param buttonaxes
	 * @return
	 */
	public int[] calcMotorPower(Integer[] xy, float compassHeading, boolean imuBearing) {
		float speedL,speedR;
		// get radian measure of left stick x,y
		if(DEBUG)
			System.out.printf(" controller Axes:%s%n",Arrays.toString(xy));
		float radians = (float) Math.atan2((double)xy[0],(double)xy[1]);
		float stickDegrees = (float) (radians * (180.0 / Math.PI)); // convert to degrees

		// horizontal axis is 0 to -180/180 degrees, we want 0 at the top 
		//<pre>
		//         -90	
		//          |
		//-180/180 --- 0
		//          |
		//         90
		//</pre>
		stickDegrees += 90; // fixes 2 right quadrants and top left quadrant, orients 0 at top, 180 at bottom, 0 to -90 top left.
		if(stickDegrees > 180.0 ) // fixes bottom left quadrant, sets -90 to -180
			stickDegrees -= 360.0;

		// we have absolute stickdegrees, offset the current IMU by that quantity to get new desired course
		float offsetDegrees = (float) (compassHeading - stickDegrees);
		// reduce the angle  
		offsetDegrees =  offsetDegrees % 360;
		//
		if (offsetDegrees <= -180.0)
			offsetDegrees += 360.0;
		if (offsetDegrees >= 180)  
			offsetDegrees -= 360;
		// force it to be the positive remainder, so that 0 <= angle < 360  
		//offsetDegrees = (offsetDegrees + 360) % 360;  
		// force into the minimum absolute value residue class, so that -180 < angle <= 180  
		//if (offsetDegrees > 180)  
		//      offsetDegrees -= 360;

		//if( DEBUG )
		//	System.out.println("Axes:"+axes[0]+","+axes[2]+" deg:"+bearingDegrees+" rad:"+radians);
		// Button 4 - triangle. Establish setpoint for autonomous course following.
		// If we are pressing the button, dont update the setpoint with IMU yaw, this has the effect of locking
		// the course onto the absolute track relative to the way the robot is pointing offset by stick position
		// rather than a continuous update of stick offset added to current orientation. so if you want the robot to track
		// straight forward on its current orientation, hold triangle/top and push stick forward or back.
		if( imuBearing ) {   
			// is it initial button press? then log current bearing of IMU to hold to
			if( !holdBearing ) {
				holdBearing = true;
				outputNeg = false;
				// Setpoint is desired target yaw angle (compass heading) from IMU,vs heading minus the joystick offset from 0
				robot.getIMUSetpointInfo().setDesiredTarget(compassHeading);
				robot.getIMUSetpointInfo().setTarget((float)0);
				robot.getMotionPIDController().clearPID();
				wasPid = true; // start with PID control until we get out of tolerance
				if(DEBUG);
				System.out.println("Stick absolute deg set:"+robot.getIMUSetpointInfo());	
			}
		} else {
			holdBearing = false;
			// joystick setting becomes desired target yaw angle heading and IMU is disregarded
			robot.getIMUSetpointInfo().setDesiredTarget(stickDegrees);
			robot.getIMUSetpointInfo().setTarget((float)0); 
		}

		// eulers.compassHeading is Input: target yaw angle from IMU. If IMU is delivering a 0, then our delta below will just be desiredTarget.
		robot.getIMUSetpointInfo().setTarget(compassHeading); 
		//
		// perform the PID processing
		// IMUSetpointInfo looks like this:
		// public void setTarget(float t) { yawAngle = t;	}
		// public float getTarget() { return yawAngle; }
		// public void setDesiredTarget(float t) { desiredYawAngle = t; }
		// public float getDesiredTarget() { return desiredYawAngle; }
		// public float delta() { return desiredYawAngle - yawAngle; }
		//
		// MotionPidController.Compute(SetPointInfo) does this with SetPointInfo before it sets PID terms:
		// SetPointInfo.setPrevErr(error);
		// error = SetPointInfo.delta();
		// error =  error % 360; // angle or 360 - angle
		// if (error <= -180.0)
		//     error += 360.0;
		// if (error >= 180)  
		//     error -= 360;  
		//
		((MotionPIDController) robot.getMotionPIDController()).Compute(robot.getIMUSetpointInfo());

		// Speed may be plus or minus, this determines a left or right turn as the quantity is added to desired speed
		// of wheels left and right. The speed for each wheel is modified by desired turn radius, which is based on speed,
		// or on the computed derivation from desired course if a straight line course is set.
		// the xy array should come out resolved via the matches method from our AxisInterface
		speedR = (float)xy[1];
		speedL = (float)xy[0];
		// PTerm has the positive or negative difference in degrees (left or right offset)
		// between IMU heading and joystick, we have to calc the scaling factors for wheel speed then apply.
		// We determine the radius of the turn potential by the forward speed desired (joystick Y)
		// For instance if at 0 we can spin at radius .5 of wheelbase if necessary, while at speed 500 out of 1000
		// we can turn at radius 1, which is 90 degrees with inner wheel stationary, and at 1000 out of 1000
		// we turn inner radius 1 and outer radius 2 robot widths as defined by WHEELBASE, which is a gradual arc.
		// degrees/180 * PI * (r + WHEELBASE) = outer wheel arc length
		// degrees/180 * PI * r = inner wheel arc length
		// we compare the arc lengths as a ratio.
		// We use the speed in the range of the stick Y range (0 to 1) times a scale, and establish that as a scale factor to 
		// add to wheelbase, so at max speed we extend the radius by one robot length; joystick y + WHEELBASE making 
		// inner radius 1 'robot length unit' and outer radius 1 + WHEELBASE units.
		// We dont really need absolute measurements, we just assign the values based on stick Y and a scale factor
		// and the same scale factor to 1 'robot width unit'. We are multiplying the stick Y by 1000 and WHEELBASE by 1000
		// but in reality we are using radius 0 to 1, and then 0 to 1 plus WHEELBASE scaled to finer granularity.
		// This assumes that at half speed we can still make a 90 degree turn.
		// We use value of IMU vs desired course to turn left or right via
		// a plus or minus value from the Compute method set in the variable called PTerm.
		float radius = Math.abs(speedR);
		float arcin = 0, arcout = 0;
		//
		if( holdBearing ) {
			if(DEBUG ) {
				System.out.printf("HOLDING BEARING %s\n",robot.getMotionPIDController().toString());
				System.out.printf("Stick deg=%f|Offset deg=%f|IMU=%f|arcin=%f|arcout=%f|speedL=%d|speedR=%d\n",stickDegrees,offsetDegrees,compassHeading,arcin,arcout,(int)speedL,(int)speedR);
			}
			float[] sp = followIMUSetpoint(compassHeading, radius, arcin, arcout, speedL, speedR);
			speedL = sp[0];
			speedR = sp[1];
		} else {
			// manual steering mode, use tight radii and a human integrator
			// Compute arc length on inner and outer wheel
			arcin = (float) (Math.abs(robot.getMotionPIDController().getError()/360.0) * (2.0 * Math.PI) * radius);
			arcout = (float) (Math.abs(robot.getMotionPIDController().getError()/360.0) * (2.0 * Math.PI) * (radius + WHEELBASE));
			// error is difference in degrees between setpoint and input
			// Stick position will override any IMU tolerances
			if( stickDegrees != -180.0 && stickDegrees != 180 && stickDegrees != 0.0) { // if we want to go straight forward or backwards dont bother computing any ratios
				if( robot.getMotionPIDController().getError() < 0.0f )
					speedL *= (arcin/arcout);
				else
					if( robot.getMotionPIDController().getError() > 0.0f )
						speedR *= (arcin/arcout);
			}
			speedL = slope(speedL, speedQueueL);
			speedR = slope(speedR, speedQueueR);
			if(DEBUG) {
				System.out.printf("MANUAL BEARING %s\n",robot.getMotionPIDController().toString());
				System.out.printf("Stick deg=%f|Offset deg=%f|IMU=%f|arcin=%f|arcout=%f|speedL=%d|speedR=%d\n",stickDegrees,offsetDegrees,compassHeading,arcin,arcout,(int)speedL,(int)speedR);
			}
		}
		return new int[] {(int)speedL,(int)speedR};
	}

	@Override
	public ArrayList<DrivenWheelInterface> getWheels() {
		 ArrayList<TypeSlotChannelEnable> atsce = robot.getManager().getTypeSlotChannelEnable(this);
		 return atsce.stream().filter(RobotWheel.class::isInstance).map(RobotWheel.class::cast).collect(Collectors.toCollection(ArrayList::new));
	}	
	
	@Override
	public String toString() {
		StringBuffer sb = new StringBuffer();
		ArrayList<DrivenWheelInterface> wheels = getWheels();
		for(int sl = 0; sl < wheels.size(); sl++) {
			DrivenWheelInterface rslot = wheels.get(sl);
				sb.append(sl+1);
				sb.append(".) ");
				sb.append(rslot.toString());
				sb.append("Controller=");
				sb.append(getManualControl());
				sb.append("\r\n");
		}
		return sb.toString();
	}

	@Override
	public int getChannels() {
		return channels;
	}

	@Override
	public void setCompassHeading(float compassheading) {
		this.compassHeading = compassheading;	
	}


}
