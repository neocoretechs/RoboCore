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
	 * Calculate the PID values, Perror contains the proportion times scalar
	 * Derror is the derivative, Ierror is the integral of error variable which we
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
	      ppi.setPrevErr(output);
	      // Compute all the working error variables
	      //output = Setpoint - Input;
	      //output =  Desired yaw angle - yaw angle;
	      output = ppi.delta();
	      // This is specific to pid control of heading 
	      // reduce the angle  
	      output =  output % 360; // angle or 360 - angle
	      //force it to be the positive remainder, so that 0 <= angle < 360  
	      //error = (error + 360) % 360;  
	      //force into the minimum absolute value residue class, so that -180 < angle <= 180  
	      if (output <= -180.0)
	            output += 360.0;
	      if (output >= 180)  
	         output -= 360;  
	      //
	      //float dInput = (Input - lastInput);
	      // Compute PID Output, proportion
	      Perror = (Kp * output) ;//+ ITerm - kd * dInput;
	      // derivative
	      Derror = Kd * (Perror - ppi.getPrevErr());
	      // integral
	      Ierror += output; // integrate the error
	      Ierror = (Ki * Ierror); // then scale it, this limits windup
	      //ITerm += (ki * error);
	      // clamp the I term to prevent reset windup
	      if( Ierror > 0 ) {
	    	  if(Ierror > ppi.getMaximum()) 
	    		  Ierror = ppi.getMaximum();
	    	  else 
	    		  if(Ierror < -ppi.getMaximum()) 
	    			  Ierror = -ppi.getMaximum();
	      } else {
	    	  if(-Ierror > ppi.getMaximum() )
	    		  Ierror = -ppi.getMaximum();
	    	  else
	    		  if(-Ierror < -ppi.getMaximum())
	    			  Ierror = -ppi.getMaximum();
	      }
	      //lastInput = Input;
	      //lastTime = now;
	      //error is positive if current_heading > bearing (compass direction)
	      // To Use the PD form, set ki to 0.0 default, thus eliminating integrator
	      output = (Perror /*+ ITerm*/ + Derror);
	   //}
	}
	
	public String toString() {
		return String.format("Motion PID Constants: %s\r\nMotion PID Output = %f | DTerm = %f | ITerm = %f | PTerm = %f ",
				super.toString(),output, Derror, Ierror, Perror);
	}

}
