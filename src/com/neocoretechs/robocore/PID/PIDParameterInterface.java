package com.neocoretechs.robocore.PID;
/**
 * Manipulate the parameters necessary for PID processing.
 * @author groff
 *
 */
public interface PIDParameterInterface {

	public void setPerror(float i); 	 // Proportion
	public void setIerror(float i);      // integrated error
	public void setDerror(float i);		 // Derivative
	public void setOutput(float i);		 // result
	public void setKp(float i);
	public void setKd(float i);
	public void setKi(float i);
	public void setKo(float i);
	public void setPIDRate(int i);     // Hz Rate at which PID loop is updated 
	public void clearPID();
	public void Compute(SetpointInfoInterface p);

	public float getPerror();					// Proportion
	public float getIerror();                   // integrated error
	public float getDerror();					// Derivative
	public float getOutput();					// result
	public float getKp();
	public float getKd();
	public float getKi();
	public float getKo();
	public int getPIDRate();
	public float getPIDInterval(SetpointInfoInterface p);
	
	/**
	 * Currently we are using a non time based control, this is for reference.
	 * @param Kp
	 * @param Ki
	 * @param Kd
	 */
	/*public void SetTunings(float Kp, float Ki, float Kd);
	{
	  float SampleTimeInSec = ((float)SampleTime)/1000.0f;
	  kp = Kp;
	  ki = Ki * SampleTimeInSec;
	  kd = Kd / SampleTimeInSec;
	}*/
	 /**
	  * If we used a time based control.
	  * @param NewSampleTime
	  */
	/*public void SetSampleTime(int NewSampleTime);
	{
	   if( NewSampleTime > 0) {
	      float ratio  = (float)NewSampleTime / (float)SampleTime;
	      ki *= ratio;
	      kd /= ratio;
	      SampleTime = NewSampleTime;
	   }
	}*/
}
