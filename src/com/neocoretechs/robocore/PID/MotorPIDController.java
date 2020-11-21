package com.neocoretechs.robocore.PID;
/**
 * Erzatz pid controller which really just caps the output of each motor command. 
 * @author groff
 *
 */
public class MotorPIDController extends AbstractPIDController {

	// 
	// - fPivYLimt  : The threshold at which the pivot action starts
	//	                This threshold is measured in units on the Y-axis
	//	                away from the X-axis (Y=0). A greater value will assign
	//	                more of the joystick's range to pivot actions.
	//	                Allowable range: (0..+1000)
	//float fPivYLimit = 250.0f;
	//int     nPivSpeed;      // Pivot Speed                          (-1000..+999)
	//float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )
	/* Counters to track update rates for PID and Odometry */
	/**
	 * Invoke Ctor with PID constants.
	 * @param kp
	 * @param kd
	 * @param ki
	 * @param ko
	 * @param pidRate
	 */
	public MotorPIDController(float kp, float kd, float ki, float ko, int pidRate) {
		super(kp, kd, ki, ko, pidRate);
	}
	/**
	 *  PID routine to compute the next motor commands 
	 */
	public synchronized void Compute(SetpointInfoInterface p) {
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
		if (p.getTarget() >= p.getMaximum())
			p.setTarget(p.getMaximum());
		else 
			if (p.getTarget() <= -p.getMaximum())
				p.setTarget(-p.getMaximum());	
	}



}
