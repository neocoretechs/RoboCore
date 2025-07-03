package com.neocoretechs.robocore.test.MotionViewer;
import javax.swing.*;
import javax.swing.event.*;

import java.awt.*;
import java.text.*;
import java.util.*;

/**
 * JFBSlider creates a new control element, by combining a slider,
 * with a textbox showing the current setting of the slider.  The limits
 * of the slider are set as floating-point numbers, and the number
 * of decimal places dislayed in the feedback window is adjustable.
 *
 * @author <a href="mailto:MikeGauland@users.sourceforge.net">Michael
 *     Gauland</a>
 * @version 1.
 */

public class JFBSlider extends JPanel implements ChangeListener {
    /**
     * All JFBSliders use this font for the feedback window.
     */
    static final Font textFont = new Font("Monospaced", Font.PLAIN, 12);

    /**
     * All JFBSliders use this font for the slider labels.
     */
    static final Font sliderFont = new Font("Serif", Font.BOLD, 10);

    /** Horizontal padding on the outside edges of the JFBSlider. */
    static final Dimension outerPad = new Dimension(10,1);

    /** Horizontal adding between the elements of the JFBSlider. */
    static final Dimension innerPad = new Dimension(5,1);

    /** The lower limit of the control range. */
    double lowerLimit;

    /** The upper limit of the control range. */
    double upperLimit;

    /** The number of steps in the slider. */
    int steps;

    /** The actual slider control. */
    JSlider theSlider;

    /** The feedback text field. */
    JTextField theTextField = new JTextField();

    /** Controls the formatting of the feedback number. */
    NumberFormat formatter = NumberFormat.getInstance();

    /** The units for the control (e.g., "&deg;") can be specified. */
    String units = "";

    /**
     * This constructor requires
     * @param name A string which will become the label of the slider.
     * @param min The minimum floating-point value of the control.
     * @param max The maximum floating-point value of the control.
     * @param numSteps The number of discrete steps along the slider.
     * @param digits The number of digits to the right of the decimal
     *     point to display in the feedback window.
     * @param units A string representing measurement units that
     *     will be displayed with the value in the feedback window.
     */
    public JFBSlider(String name, double min, double max, int numSteps,
              int digits, String units) {
        super();
        this.units = units;
        setup(name, min, max, numSteps, digits);
    }
   
    /**
     * This constructor does not require a units string.
     * @param name A string which will become the label of the slider.
     * @param min The minimum floating-point value of the control.
     * @param max The maximum floating-point value of the control.
     * @param numSteps The number of discrete steps along the slider.
     * @param digits The number of digits to the right of the decimal
     *     point to display in the feedback window.
     * @param units A string representing measurement units that
     *     will be displayed with the value in the feedback window.
     */
    public JFBSlider(String name, double min, double max, int numSteps,
              int digits) {
        super();

        setup(name, min, max, numSteps, digits);
    }

    /**
     * This is a private function to help the two constructors set up
     * the control.  It is only called during contruction.
     *
     * The parameters are the same as for the constructors.
     */
    private void setup(String name, double min, double max, int numSteps,
                       int digits)
    {
        /* Set the limits and number of steps for the slider: */
        lowerLimit = min;
        upperLimit = max;
        steps = numSteps;

        /*
         * Set up the number formatter.  By setting both "...Min..."
         * and "...Max..." to the same value, the same number of
         * fractional digits will always be displayed.
         */
        formatter.setMinimumFractionDigits(digits);
        formatter.setMaximumFractionDigits(digits);

        /*
         * Create the slider, and set the font.
         */
        theSlider = new JSlider(0, steps);
        theSlider.setFont(sliderFont);

        /*
         * Create a HashTable for the slider labels, then add 
         * strings for to label the limits and center of the
         * slider.
         */
        Hashtable labelStrings = new Hashtable();

        // Label the minimum (left limit)
        JLabel label = new JLabel(formatter.format(min));
        label.setFont(sliderFont);
        labelStrings.put(Integer.valueOf(0), label);

        // Label the midpoint
        label = new JLabel(formatter.format((min + max)/2));
        label.setFont(sliderFont);
        labelStrings.put(Integer.valueOf(steps/2), label);

        // Label the maximum (right limit)
        label = new JLabel(formatter.format(max));
        label.setFont(sliderFont);
        labelStrings.put(Integer.valueOf(steps), label);

        /*
         * Install the labels on the slider, and turn them on.
         */
        theSlider.setLabelTable(labelStrings);
        theSlider.setPaintLabels(true);

        /*
         * Create the label for the slider, using the name string
         * passed in to the constructor.
         */
        JLabel theLabel = new JLabel(name);
        theLabel.setHorizontalTextPosition(JLabel.RIGHT);
        theLabel.setLabelFor(theSlider);

        /*
         * Set the layout or the JFBSlider, and install the
         * label an the slider, with horizontal padding.
         */
        this.setLayout(new BoxLayout(this, BoxLayout.X_AXIS));

        this.add(Box.createRigidArea(outerPad));
        this.add(theLabel);
      
        this.add(Box.createRigidArea(innerPad));
        this.add(theSlider);

        /*
         * Set up the text field.  Note that it cannot be edited.  It
         * would be cool to allow a user to enter a value into the box,
         * as an alternative to setting the slider, but that is not
         * implemented in this class.
         */
        theTextField.setEditable(false);                  
        theTextField.setHorizontalAlignment(JTextField.RIGHT);
        theTextField.setFont(textFont);
        {
            /*
             * Set the width of the text box to be wide enough to
             * show the full range of values.
             */
            int len1 = formatter.format(min).length();
            int len2 = formatter.format(max).length();
                  
            int places = (len1 >= len2) ? len1 : len2;

            places += units.length();

            theTextField.setColumns(places + 1);  // 1 extra space for clarity
        }

        /*
         * Set the maximum size to the preferred size, so the box
         * won't stretch to fill large windows.
         */
        theTextField.setMaximumSize(theTextField.getPreferredSize());

        /*
         * Add the text box, with padding, to the JFBSlider panel.
         */
        this.add(Box.createRigidArea(innerPad));
        this.add(theTextField);
        this.add(Box.createRigidArea(outerPad));

        /*
         * Add this object as a ChangeListener to the slider.  Whenever
         * the slider moves, we need to update the text box.
         */
        theSlider.addChangeListener(this);

        /*
         * Force a change so the text box will be correct.
         */
        setValue(getValue());
    }

    /**
     * The client can add tick marks to the JFBSlider, as
     * with any JSlider object.
     *
     * @param major The spacing between major tick marks.
     * @param minor The spacing between minor tick marks.
     */
    public void addTicks(int major, int minor) {
        theSlider.setMajorTickSpacing(major);
        theSlider.setMinorTickSpacing(minor);
        theSlider.setPaintTicks(true);
    }

    /**
     * This is called when the slider value is set, and just
     * updates the feedback text box.
     *
     * @param e The event, which is ignored, since we know
     *     it must have come from the slider.
     */
    public void stateChanged(ChangeEvent e) {
        theTextField.setText(formatter.format(getValue()) + units);
    }  

    /**
     * Add a ChangeListener to the slider. This lets the client
     * react to changes to the control (which is, after all, the point
     * of having a control at all, isn't it?).
     *
     * @param l The ChangeListener object to add.
     */
    public void addChangeListener(ChangeListener l) {
        theSlider.addChangeListener(l);
    }

    /**
     * This returns the value of the slider, scaled based on the
     * floating-point min and max values supplied at instantiation.
     */
    public double getValue() {
        return lowerLimit + theSlider.getValue() *
            (upperLimit - lowerLimit) / steps;
    }

    /**
     * Force a new value on the control, by setting the slider to
     * the point closest to the supplied floating-point value.  This
     * will reslt in all ChangeListeners being called, including this
     * object, so this function needn't update the feedback text box.
     *
     * @param value The new value for the control.
     */
    public void setValue(double value) {
        theSlider.setValue((int)Math.round(((value - lowerLimit) /
                                            (upperLimit-lowerLimit)*steps)));
    }

    /**
     * When a client is notified that the control changed (through
     * the ChangeListener mechanism), it can pass the event to this
     * function to determine whether this control generated the message.
     */
    public boolean isEventSource(ChangeEvent e) {
        return e.getSource() == theSlider;
    }

    /**
     * Returns the context of the feedback text field.  This can be useful
     * when a client wants to print or display the control value, without
     * having to format it.
     */
    public String getText() {
        return theTextField.getText();
    }  
}
