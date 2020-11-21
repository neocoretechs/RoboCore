package com.neocoretechs.robocore.PID;

public abstract class AbstractPIDController implements PIDParameterInterface {

	float Perror;
	float output;
	float Ierror;
	float Derror;
	/* PID Parameters */
	float Kp;// = 20;
	float Kd;// = 12;
	float Ki;// = 0;
	float Ko;// = 50;
	int PID_RATE;// = 30;     // Hz
	/* Rate at which PID loop is updated */
	public float getPIDInterval(SetpointInfoInterface spi) {return spi.getMaximum() / (float)getPIDRate(); }

	/* Counters to track update rates for PID and Odometry */
	int nextPID = 0;

	@Override
	public float getPerror() { return Perror; }
	@Override
	public float getIerror() {return Ierror;}
	@Override
	public float getDerror() {return Derror;}
	@Override
	public float getOutput() {return output;}

	@Override
	public void setPerror(float i) {Perror = i;}					// Proportion
	@Override
	public void setIerror(float i) {Ierror = i;}                    // integrated error
	@Override
	public void setDerror(float i) {Derror = i; }					// Derivative
	@Override
	public void setOutput(float i) {output = i;}
	@Override
	public float getKp() {return Kp;}
	@Override
	public void setKp(float kp2) {Kp = kp2;}
	@Override
	public float getKd() {return Kd;}
	@Override
	public void setKd(float kd) {Kd = kd;}
	@Override
	public float getKi() {return Ki;}
	@Override
	public void setKi(float ki) {Ki = ki;}
	@Override
	public float getKo() {return Ko;}
	@Override
	public void setKo(float ko) {Ko = ko;}
	@Override
	public int getPIDRate() {return PID_RATE;}
	@Override
	public void setPIDRate(int pID_RATE) {PID_RATE = pID_RATE;}

	/**
	 * 
	 * @param kp
	 * @param kd
	 * @param ki
	 * @param ko
	 * @param pidRate
	 * @param maximum
	 */
	public AbstractPIDController(float kp, float kd, float ki, float ko, int pidRate) {
		setKp(kp);
		setKd(kd);
		setKi(ki);
		setKo(ko);
		setPIDRate(pidRate);
	}

	public void clearPID() {
		Perror = 0;
		Ierror = 0;
		Derror = 0;
		output = 0;
	}
	
	public abstract void Compute(SetpointInfoInterface ppi);

}
