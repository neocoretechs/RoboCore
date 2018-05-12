package com.neocoretechs.robocore.test.MotionViewer;
import javax.swing.*;
import javax.swing.border.*;

import java.text.*;
import java.awt.*;

/**
 * This class is used to display both the true final position of the robot,
 * and the dead-reckoned position.
 *
 * @author Various
 * @version 1.
 */
class PositionPanel extends JPanel
{
    /**
     * The default constructor sets up all fields of the display, and
     * initially shows the time and all coordinates as 0, and the angles
     * as 90 degrees.
     */
    public PositionPanel(){
      /*
    * Set up the border for this panel
    */
        TitledBorder  tBorder = new TitledBorder("Actual Position ");
        setBorder(tBorder);
        setForeground(tBorder.getTitleColor());

   /*
    * Use a "Grid Bag Layout" for this panel.  It will be laid
    * out like this:
    * X:          <actual X>   
    * Y:          <actual Y>   
    * Theta(DTerm):      <actual deg>  
	* Speed L/R:  <actual L Vel> <actual R final Vel>
	* ITerm:	<integral of PID>
	* PID:		<PID value>
	* Err:		<Course deviation err>
    */
        GridBagLayout      gbl      = new GridBagLayout();
        GridBagConstraints gbc = new GridBagConstraints();
        setLayout(gbl);
        gbc.anchor=gbc.NORTHWEST;


   /*
    * Set up the "X" label
    */
        gbc.fill=gbc.HORIZONTAL;
        JLabel xJLabel = new JLabel("X:");
        gbc.gridx=0;
        gbc.gridy=1;
        gbc.weightx=0;
        gbc.weighty=1;
        gbl.setConstraints(xJLabel, gbc);
        add(xJLabel);

   /*
    * Set up the "Y" label
    */
        JLabel yJLabel = new JLabel("Y:");
        gbc.fill=gbc.HORIZONTAL;
        gbc.gridx=0;
        gbc.gridy=2;
        gbc.weightx=0;
        gbc.weighty=1;
        gbl.setConstraints(yJLabel, gbc);
        add(yJLabel);

   /*
    * Set up the "Theta" label
    */
        JLabel aJLabel = new JLabel("Theta (DTerm):");
        gbc.fill=gbc.HORIZONTAL;
        gbc.gridx=0;
        gbc.gridy=3;
        gbc.weightx=0;
        gbc.weighty=1;
        gbl.setConstraints(aJLabel, gbc);
        add(aJLabel);

    /*
     * Set up the Speed label
     */
        JLabel vJLabel = new JLabel("Speed L/R:");
        gbc.fill=gbc.HORIZONTAL;
        gbc.gridx=0;
        gbc.gridy=4;
        gbc.weightx=0;
        gbc.weighty=1;
        gbl.setConstraints(vJLabel, gbc);
        add(vJLabel);

    /*
     * Set up the Iterm label
     */
        JLabel aposJLabel = new JLabel("ITerm:");
        //aposJLabel.setHorizontalAlignment(JLabel.RIGHT);
        gbc.fill=gbc.HORIZONTAL;
        gbc.gridx=0;
        gbc.gridy=5;
        gbc.weightx=0;
        gbc.weighty=1;
        gbl.setConstraints(aposJLabel, gbc);
        add(aposJLabel);


    /*
     * Set up the PID label
     */
        JLabel dmJLabel = new JLabel("PID:");
        //dmJLabel.setHorizontalAlignment(JLabel.RIGHT);
        gbc.fill=gbc.HORIZONTAL;
        gbc.gridx=0;
        gbc.gridy=6;
        gbc.weightx=0;
        gbc.weighty=1;
        gbl.setConstraints(dmJLabel, gbc);
        add(dmJLabel);
        
     /*
      * Set up the Err label
      */
        JLabel dJLabel = new JLabel("Err:");
        //dJLabel.setHorizontalAlignment(JLabel.RIGHT);
        gbc.fill=gbc.HORIZONTAL;
        gbc.gridx=0;
        gbc.gridy=7;
        gbc.weightx=0;
        gbc.weighty=1;
        gbl.setConstraints(dJLabel, gbc);
        add(dJLabel);
        
     /*
      * Set up the Speed label
      */
        JLabel sJLabel = new JLabel("Speed:");
        //dJLabel.setHorizontalAlignment(JLabel.RIGHT);
        gbc.fill=gbc.HORIZONTAL;
        gbc.gridx=0;
        gbc.gridy=8;
        gbc.weightx=0;
        gbc.weighty=1;
        gbl.setConstraints(sJLabel, gbc);
        add(sJLabel);
        
     /*
      * Set up the Speed label
      */
        JLabel iJLabel = new JLabel("IMU:");
        //dJLabel.setHorizontalAlignment(JLabel.RIGHT);
        gbc.fill=gbc.HORIZONTAL;
        gbc.gridx=0;
        gbc.gridy=9;
        gbc.weightx=0;
        gbc.weighty=1;
        gbl.setConstraints(iJLabel, gbc);
        add(iJLabel);

   /*
    * Set up fields for data points.
    * Set up the text field for the actual X coordinate
    */
        xField = new JTextField(8);
        xField.setEditable(false);
        xField.setHorizontalAlignment(JTextField.RIGHT);
        xField.setBorder(null);
        gbc.fill=gbc.NONE;
        gbc.gridx=1;
        gbc.gridy=1;
        gbc.weightx=1;
        gbc.weighty=1;
        gbl.setConstraints(xField, gbc);
        add(xField);

   /*
    * Set up the text field for the actual Y coordinate
    */
        yField = new JTextField(8);
        yField.setEditable(false);
        yField.setHorizontalAlignment(JTextField.RIGHT);
        yField.setBorder(null);
        gbc.fill=gbc.NONE;
        gbc.gridx=1;
        gbc.gridy=2;
        gbc.weightx=1;
        gbc.weighty=1;
        gbl.setConstraints(yField, gbc);
        add(yField);

   /*
    * Set up the text field for the actual angle (theta)
    */
        aField = new JTextField(8);
        aField.setEditable(false);
        aField.setHorizontalAlignment(JTextField.RIGHT);
        aField.setBorder(null);
        gbc.fill=gbc.NONE;
        gbc.gridx=1;
        gbc.gridy=3;
        gbc.weightx=1;
        gbc.weighty=1;
        gbl.setConstraints(aField, gbc);
        add(aField);

   /*
    * Set up the text field for the d
    
        dxField = new JTextField(8);
        dxField.setEditable(false);
        dxField.setHorizontalAlignment(JTextField.RIGHT);
        dxField.setBorder(null);
        gbc.fill=gbc.NONE;
        gbc.gridx=2;
        gbc.gridy=1;
        gbc.weightx=1;
        gbc.weighty=1;
        gbl.setConstraints(dxField, gbc);
        add(dxField);
    */
   /*
    * Set up the text field for the 
    
        dyField = new JTextField(8);
        dyField.setEditable(false);
        dyField.setHorizontalAlignment(JTextField.RIGHT);
        dyField.setBorder(null);
        gbc.fill=gbc.NONE;
        gbc.gridx=2;
        gbc.gridy=2;
        gbc.weightx=1;
        gbc.weighty=1;
        gbl.setConstraints(dyField, gbc);
        add(dyField);
	*/        

   /*
    * Set up the text fields for left and right velocities
    */
        vLeftField = new JTextField(8);
        vLeftField.setEditable(false);
        vLeftField.setHorizontalAlignment(JTextField.RIGHT);
        vLeftField.setBorder(null);
        gbc.fill=gbc.NONE;
        gbc.gridx=1;
        gbc.gridy=4;
        gbc.weightx=1;
        gbc.weighty=1;
        gbl.setConstraints(vLeftField, gbc);
        add(vLeftField);

        vRightField = new JTextField(8);
        vRightField.setEditable(false);
        vRightField.setHorizontalAlignment(JTextField.RIGHT);
        vRightField.setBorder(null);
        gbc.fill=gbc.NONE;
        gbc.gridx=2;
        gbc.gridy=4;
        gbc.weightx=1;
        gbc.weighty=1;
        gbl.setConstraints(vRightField, gbc);
        add(vRightField);
    /*
     * Set up the text field for the ITerm
     */
        itermField = new JTextField(8);
        itermField.setEditable(false);
        itermField.setHorizontalAlignment(JTextField.RIGHT);
        itermField.setBorder(null);
        gbc.fill=gbc.NONE;
        gbc.gridx=1;
        gbc.gridy=5;
        gbc.weightx=1;
        gbc.weighty=1;
        gbl.setConstraints(itermField, gbc);
        add(itermField);


   /*
    * Set up the text field for the PID value
    */
        pidField = new JTextField(8);
        pidField.setEditable(false);
        pidField.setHorizontalAlignment(JTextField.RIGHT);
        pidField.setBorder(null);
        gbc.fill=gbc.NONE;
        gbc.gridx=1;
        gbc.gridy=6;
        gbc.weightx=1;
        gbc.weighty=1;
        gbl.setConstraints(pidField, gbc);
        add(pidField);

    /*
     * Set up the text field for the err
     */ 
        errField = new JTextField(8);
        errField.setEditable(false);
        errField.setHorizontalAlignment(JTextField.RIGHT);
        errField.setBorder(null);
        gbc.fill=gbc.NONE;
        gbc.gridx=1;
        gbc.gridy=7;
        gbc.weightx=1;
        gbc.weighty=1;
        gbl.setConstraints(errField, gbc);
        add(errField);

     /*
      * Set up the text field for the speed
      */
        spdField = new JTextField(8);
        spdField.setEditable(false);
        spdField.setHorizontalAlignment(JTextField.RIGHT);
        spdField.setBorder(null);
        gbc.fill=gbc.NONE;
        gbc.gridx=1;
        gbc.gridy=8;
        gbc.weightx=1;
        gbc.weighty=1;
        gbl.setConstraints(spdField, gbc);
        add(spdField);
  
      /*
       * Set up the text field for the IMU
       */
       imuField = new JTextField(8);
       imuField.setEditable(false);
       imuField.setHorizontalAlignment(JTextField.RIGHT);
       imuField.setBorder(null);
       gbc.fill=gbc.NONE;
       gbc.gridx=1;
       gbc.gridy=9;
       gbc.weightx=1;
       gbc.weighty=1;
       gbl.setConstraints(imuField, gbc);
       add(imuField);
   /*
    * Set up the text field for the dead-reckoned with mean theta angle (theta)
    */
        /*
        dmaField = new JTextField(8);
        dmaField.setEditable(false);
        dmaField.setHorizontalAlignment(JTextField.RIGHT);
        dmaField.setBorder(null);
        gbc.fill=gbc.NONE;
        gbc.gridx=4;
        gbc.gridy=3;
        gbc.weightx=1;
        gbc.weighty=1;
        gbl.setConstraints(dmaField, gbc);
        add(dmaField);
        */


   /*
    * Create formatters for the coordinates and time (2 decimal places)
    * and the angles (1 decimal place, and the degrees symbol).
    */
        df = new DecimalFormat("####0.0000");
        dfs = new DecimalFormat("+####0.00;-####0.00");
        da = new DecimalFormat("####0.0000\u00b0");
        das = new DecimalFormat("+####0.0\u00b0;-####0.0\u00b0");

   /*
    * Initialize all displayed values to 0, except the angles,
    * which are set to 90 degrees.
    */
        setComputedValues(0.0, 0.0, 90.0, 
				                90.0, 0,
				                0.0, 0.0, 0.0,
								0.0, 0.0);
    }


    /* Convert from radians to degrees, and coerce to range (-180, 180] */
    private double degree(double a){
        double d = a*180.0/Math.PI;
        if(d<0)
           d = -d;
        d = d - 360.0*Math.floor(d/360.0);
        if(a<0)
           d=360.0-d;
        if(d>180.0)
           d-=360;
        return d;
    }


    /**
     * Display both the true and dead-reckoned positions. theta is dterm, or robot orientation, err is course deviation
     * from center track.
     * (location and direction).
     * runData[0][numSteps] = err;
     * runData[1][numSteps] = left;
     * runData[2][numSteps] = right;
     * runData[3][numSteps] = dterm;
     * runData[4][numSteps] = iterm;
     * runData[5][numSteps] = pid;
     * runData[6][numSteps] = speed;
     * @param  
     */
    void setComputedValues(double x, double y, double theta,
                           double err, double speed,
						   double pid, double iterm, double imu,
                           double vLeft, double vRight)
    {
        xField.setText(df.format(x));
        yField.setText(df.format(y));
        aField.setText(da.format(degree(theta)));

        //dxField.setText(df.format(xDR) + " ("+dfs.format(xDR-x)+")");
        //dyField.setText(df.format(yDR) + " ("+dfs.format(yDR-y)+")");
        errField.setText(da.format(err));//+ 
				                //" ("+das.format(degree(thetaDR-theta))+")");
        spdField.setText(df.format(speed));

        pidField.setText(df.format(pid));// + " ("+dfs.format(xDRM-x)+")");
        itermField.setText(df.format(iterm));// + " ("+dfs.format(yDRM-y)+")");
        //dmaField.setText(da.format(degree(thetaDRM)));//+ 
				                //" ("+das.format(degree(thetaDRM-theta))+")");
        imuField.setText(da.format(imu));

        vLeftField.setText(df.format(vLeft));
        vRightField.setText(df.format(vRight));
    }

    /** Formatter for the coordinates and time (2 decimal places). */
    DecimalFormat df;

    /** Formatter for the errors (2 decimal places, allows shows + or -). */
    DecimalFormat dfs;

    /** Formatter for the angles (1 decimal place, and the "&deg;" symbol). */
    DecimalFormat da;

    /** Formatter for the angle errors (1 decimal place, allows include + or - 
		 * and the "&deg;" symbol). */
    DecimalFormat das;

   /** Field for displaying the "true" X coordinate. */
    JTextField xField;

    /** Field for displaying the "true" Y coordinate. */
    JTextField yField;
    /** Field for displaying the "true" direction. */
    JTextField aField;

    /** Field for displaying the dead-reckoned X coordinate. */
    JTextField dxField;

    /** Field for displaying the dead-reckoned Y coordinate. */
    JTextField dyField;

    /** Field for displaying the dead-reckoned direction. */
    JTextField errField;

    /** Field for displaying the dead-reckoned X coordinate. */
    JTextField pidField;
    
    /** Field for displaying the dead-reckoned Y coordinate. */
    JTextField spdField;

    /** Field for displaying the dead-reckoned Y coordinate. */
    JTextField itermField;

    /** Field for displaying the dead-reckoned direction. */
    JTextField dmaField;

    /** IMU heading reading */
    JTextField imuField;
    
    /** Field for displaying the final velocities. */
    JTextField vLeftField;
    JTextField vRightField;
}
