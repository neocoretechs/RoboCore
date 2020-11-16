package com.neocoretechs.robocore;

public class PIDController {
	int Perror;
	int output;
	int Ierror;
	/* PID Parameters */
	int Kp = 20;
	int Kd = 12;
	int Ki = 0;
	int Ko = 50;
	// - fPivYLimt  : The threshold at which the pivot action starts
	//	                This threshold is measured in units on the Y-axis
	//	                away from the X-axis (Y=0). A greater value will assign
	//	                more of the joystick's range to pivot actions.
	//	                Allowable range: (0..+1000)
	float fPivYLimit = 250.0f;

	int     nPivSpeed;      // Pivot Speed                          (-1000..+999)
	float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )
	int PID_RATE = 30;     // Hz
	/* Rate at which PID loop is updated */
	float PID_INTERVAL = (float) (1000.0 / PID_RATE);

	/* Counters to track update rates for PID and Odometry */
	int nextPID = 0;
	public void setPrevErr(int i) {Perror = i;}                  // last error
	public void setIerror(int i) {Ierror = i;}                    // integrated error
	public void setOutput(int i) {output = i;}
	public PIDController() {
		// TODO Auto-generated constructor stub
	}
	/**
	 *  PID routine to compute the next motor commands 
	 */
	protected synchronized void doPID(SetpointInfo p) {
		/*
	
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
		if (p.getTargetSpeed() >= SetpointInfo.MAXOUTPUT)
			p.setTargetSpeed(SetpointInfo.MAXOUTPUT);
		else if (p.getTargetSpeed() <= -SetpointInfo.MAXOUTPUT)
			p.setTargetSpeed(-SetpointInfo.MAXOUTPUT);	
		}
	public void clearPID() {
		Perror = 0;
		Ierror = 0;
		output = 0;
	}

}
