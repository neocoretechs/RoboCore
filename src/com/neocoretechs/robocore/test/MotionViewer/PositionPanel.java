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
        TitledBorder  tBorder = new TitledBorder("Actual and Dead " +
                                                  "Reckoned Position " +
																									"(DR2 - mean theta)");
        setBorder(tBorder);
        setForeground(tBorder.getTitleColor());

   /*
    * Use a "Grid Bag Layout" for this panel.  It will be laid
    * out like this:
    * X:          <actual X>    <dr X>    <dr X err>   <dr2 X>   <dr2 X err>
    * Y:          <actual Y>    <dr Y>    <dr Y err>   <dr2 Y>   <dr2 Y err>
    * Theta:      <actual deg>  <dr deg>  <dr deg err> <dr2 deg> <dr2 deg err>
		* Final Vel:  <actual L Vel><actual R final Vel>
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
        JLabel aJLabel = new JLabel("Theta:");
        gbc.fill=gbc.HORIZONTAL;
        gbc.gridx=0;
        gbc.gridy=3;
        gbc.weightx=0;
        gbc.weighty=1;
        gbl.setConstraints(aJLabel, gbc);
        add(aJLabel);

    /*
     * Set up the "Velocity" label
     */
        JLabel vJLabel = new JLabel("Final Vel L/R:");
        gbc.fill=gbc.HORIZONTAL;
        gbc.gridx=0;
        gbc.gridy=4;
        gbc.weightx=0;
        gbc.weighty=1;
        gbl.setConstraints(vJLabel, gbc);
        add(vJLabel);

    /*
     * Set up the "Actual Pos" label
     */
        JLabel aposJLabel = new JLabel("Actual");
        aposJLabel.setHorizontalAlignment(JLabel.RIGHT);
        gbc.fill=gbc.HORIZONTAL;
        gbc.gridx=1;
        gbc.gridy=0;
        gbc.weightx=0;
        gbc.weighty=1;
        gbl.setConstraints(aposJLabel, gbc);
        add(aposJLabel);

    /*
     * Set up the "Dead Rec Pos" label
     */
        JLabel dJLabel = new JLabel("DR (Err)");
        dJLabel.setHorizontalAlignment(JLabel.RIGHT);
        gbc.fill=gbc.HORIZONTAL;
        gbc.gridx=2;
        gbc.gridy=0;
        gbc.weightx=0;
        gbc.weighty=1;
        gbl.setConstraints(dJLabel, gbc);
        add(dJLabel);

    /*
     * Set up the "Dead Rec w/ mean Pos" label
     */
        JLabel dmJLabel = new JLabel("DR2 (Err)");
        dmJLabel.setHorizontalAlignment(JLabel.RIGHT);
        gbc.fill=gbc.HORIZONTAL;
        gbc.gridx=4;
        gbc.gridy=0;
        gbc.weightx=0;
        gbc.weighty=1;
        gbl.setConstraints(dmJLabel, gbc);
        add(dmJLabel);

   /*
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
    * Set up the text field for the dead-reckoned X coordinate
    */
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

   /*
    * Set up the text field for the dead-reckoned Y coordinate
    */
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

   /*
    * Set up the text field for the dead-reckoned angle (theta)
    */
        daField = new JTextField(8);
        daField.setEditable(false);
        daField.setHorizontalAlignment(JTextField.RIGHT);
        daField.setBorder(null);
        gbc.fill=gbc.NONE;
        gbc.gridx=2;
        gbc.gridy=3;
        gbc.weightx=1;
        gbc.weighty=1;
        gbl.setConstraints(daField, gbc);
        add(daField);

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
    * Set up the text field for the dead-reckoned with mean theta X coordinate
    */
        dmxField = new JTextField(8);
        dmxField.setEditable(false);
        dmxField.setHorizontalAlignment(JTextField.RIGHT);
        dmxField.setBorder(null);
        gbc.fill=gbc.NONE;
        gbc.gridx=4;
        gbc.gridy=1;
        gbc.weightx=1;
        gbc.weighty=1;
        gbl.setConstraints(dmxField, gbc);
        add(dmxField);

   /*
    * Set up the text field for the dead-reckoned with mean theta Y coordinate
    */
        dmyField = new JTextField(8);
        dmyField.setEditable(false);
        dmyField.setHorizontalAlignment(JTextField.RIGHT);
        dmyField.setBorder(null);
        gbc.fill=gbc.NONE;
        gbc.gridx=4;
        gbc.gridy=2;
        gbc.weightx=1;
        gbc.weighty=1;
        gbl.setConstraints(dmyField, gbc);
        add(dmyField);

   /*
    * Set up the text field for the dead-reckoned with mean theta angle (theta)
    */
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


   /*
    * Create formatters for the coordinates and time (2 decimal places)
    * and the angles (1 decimal place, and the degrees symbol).
    */
        df = new DecimalFormat("####0.00");
        dfs = new DecimalFormat("+####0.00;-####0.00");
        da = new DecimalFormat("####0.0\u00b0");
        das = new DecimalFormat("+####0.0\u00b0;-####0.0\u00b0");

   /*
    * Initialize all displayed values to 0, except the angles,
    * which are set to 90 degrees.
    */
        setComputedValues(0.0, 0.0, 90.0, 
				                  0.0, 0.0, 90.0, 
				                  0.0, 0.0, 90.0, 
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
     * Display both the true and dead-reckoned positions
     * (location and direction).
     */
    void setComputedValues(double x, double y, double theta,
                           double xDR, double yDR, double thetaDR,
													 double xDRM, double yDRM, double thetaDRM,
                           double vLeft, double vRight)
    {
        xField.setText(df.format(x));
        yField.setText(df.format(y));
        aField.setText(da.format(degree(theta)));

        dxField.setText(df.format(xDR) + " ("+dfs.format(xDR-x)+")");
        dyField.setText(df.format(yDR) + " ("+dfs.format(yDR-y)+")");
        daField.setText(da.format(degree(thetaDR))+ 
				                " ("+das.format(degree(thetaDR-theta))+")");


        dmxField.setText(df.format(xDRM) + " ("+dfs.format(xDRM-x)+")");
        dmyField.setText(df.format(yDRM) + " ("+dfs.format(yDRM-y)+")");
        dmaField.setText(da.format(degree(thetaDRM))+ 
				                " ("+das.format(degree(thetaDRM-theta))+")");

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
    JTextField daField;

    /** Field for displaying the dead-reckoned X coordinate. */
    JTextField dmxField;

    /** Field for displaying the dead-reckoned Y coordinate. */
    JTextField dmyField;

    /** Field for displaying the dead-reckoned direction. */
    JTextField dmaField;

    /** Field for displaying the final velocities. */
    JTextField vLeftField;
    JTextField vRightField;
}
