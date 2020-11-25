package com.neocoretechs.robocore.PID;

public class MotionPIDController extends AbstractPIDController {
	/**
	 * Construct with PID constants, integration rate, and maximum windup
	 * @param kp
	 * @param kd
	 * @param ki
	 * @param ko
	 * @param pidRate
	 * @param maximum
	 */
	public MotionPIDController(float kp, float kd, float ki, float ko, int pidRate) {
		super(kp, kd, ki, ko, pidRate);
	}
	/**
	 * Calculate the PID values, PTerm contains the proportion times scalar
	 * DTerm is the derivative, ITerm is the integral of error variable which we
	 * may or may not use. If we do, maximum should probably be set to no more than
	 * 45 degrees in the IMU setpoint.
	 * pidOutput is output.<br/>
	 * @param ppi The SetpointInfoInterface of type IMU
	 */
	@Override
	public void Compute(SetpointInfoInterface ppi) {
		
	   //if(!inAuto) return;
	   //long now = System.currentTimeMillis();
	   //long timeChange = (now - lastTime);
	   //if(timeChange >= SampleTime) {
	      ppi.setPrevErr(error);
	      // Compute all the working error variables
	      //error = Setpoint - Input OR error =  Desired yaw angle - yaw angle;
	      error = ppi.delta();
	      // This is specific to pid control of heading 
	      // reduce the angle  
	      error =  error % 360; // angle or 360 - angle
	      //force it to be the positive remainder, so that 0 <= angle < 360  
	      //error = (error + 360) % 360;  
	      //force into the minimum absolute value residue class, so that -180 < angle <= 180  
	      if (error <= -180.0)
	            error += 360.0;
	      if (error >= 180)  
	         error -= 360;  
	      //
	      //float dInput = (Input - lastInput);
	      // Compute PID Output, proportion
	      PTerm = (Kp * error) ;//+ ITerm - kd * dInput;
	      // derivative
	      DTerm = Kd * (error - ppi.getPrevErr());
	      // integral
	      ITerm += error; // integrate the error
	      ITerm = (Ki * ITerm); // then scale it, this limits windup
	      //ITerm += (ki * error);
	      // clamp the I term to prevent reset windup
	      if( ITerm > 0 ) {
	    	  if(ITerm > ppi.getMaximum()) 
	    		  ITerm = ppi.getMaximum();
	    	  else 
	    		  if(ITerm < ppi.getMinimum()) 
	    			  ITerm = ppi.getMinimum();
	      } else {
	    	  if(-ITerm > ppi.getMaximum() )
	    		  ITerm = -ppi.getMaximum();
	    	  else
	    		  if(-ITerm < ppi.getMinimum())
	    			  ITerm = -ppi.getMinimum();
	      }
	      //lastInput = Input;
	      //lastTime = now;
	      //error is positive if current_heading > bearing (compass direction)
	      // To Use the PD form, set ki to 0.0 default, thus eliminating integrator
	      output = (PTerm /*+ ITerm*/ + DTerm);
	   //}
	}
	
	public String toString() {
		return String.format("Motion PID Error = %f, Output = %f | DTerm = %f | ITerm = %f | PTerm = %f | Motion PID Constants: %s",
				error,output, DTerm, ITerm, PTerm,super.toString());
	}

}
