package com.neocoretechs.robocore.test.MotionViewer;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.Graphics;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.concurrent.CountDownLatch;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JTabbedPane;
import javax.swing.SwingUtilities;
import javax.swing.border.LineBorder;
import javax.swing.border.TitledBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import com.neocoretechs.robocore.config.Robot;
import com.neocoretechs.robocore.config.RobotInterface;
import com.neocoretechs.robocore.propulsion.MotionController;



/**
 * This is to top-level class for the app.  It demonstrates the errors
 * that occur with "dead-reckoning" navigation.
 * To run it use: java -cp RoboCore.jar com.neocoretechs.robocore.test.MotionApp
 * Original version by
 * @author <a href="mailto:MikeGauland@users.sourceforge.net">Michael
 *     Gauland</a>
 *
 * Model for acceleration added by
 * @author Jing YE, a student at the University of Melbourne
 *
 * Revision to use (theta_old+theta_new)/2 and show error amounts by
 * @author <a href="mailto:tdbrown@uiuc.edu">Tom Brown</a>
 *
 * @version 1.3
 * This version:
 * @author Jonathan Groff (C) NeoCoreTechs 2021
 *
 */
public class MotionApp implements ChangeListener, DrawInterface
{
	public static String setPoint;
	public static int numSteps;
    // These variables are used for the controls in the applet.

    /** Adjustment for the initial position "X" coordinate. */
    private JFBSlider controlPositionX = new JFBSlider("X:", -10, 10, 200, 1);

    /** Adjustment for the initial position "Y" coordinate. */
    private JFBSlider controlPositionY = new JFBSlider("Y:", -10, 10, 200, 1);

    /** Adjustment for the initial direction. */
    private JFBSlider controlPositionTheta = new JFBSlider("Coord:", 0, numSteps,
                                                           numSteps, 0);

    /** Adjustment for the velocity of the left wheel. */
    private JFBSlider controlVelocityLeft = new JFBSlider("L: ", -5.00, 5.00,
                                                          100, 2);

    /** Adjustment for the velocity of the right wheel. */
    private JFBSlider controlVelocityRight = new JFBSlider("R: ", -5.00, 5.00,
                                                           100, 2);

    /** Adjustment for the acceleration of the left wheel. */
    private JFBSlider controlAccelerationLeft= new JFBSlider("L: ", -1.00, 1.00,
                                                          200, 2);

    /** Adjustment for the acceleration of the right wheel. */
    private JFBSlider controlAccelerationRight= new JFBSlider("R: ",-1.00, 1.00,
                                                           200, 2);

    /** Adjustment for the width of the robot's body. */
    private JFBSlider controlBodyWidth = new JFBSlider("Width: ", 0.1, 2.5,
                                                       120, 2);

    /** Adjustment for the duration of the simulation. */
    private JFBSlider controlDuration = new JFBSlider("Duration: " , 0.0, 30.0,
                                                      300, 1);

    /** Adjustment for the interval between dead-reckonings. */
    private JFBSlider controlDeadReckoningInterval = new JFBSlider("Interval:",
                                                                   0.0, 1.0,
                                                                   100, 2);

    // END of controls variables.

    // These variables will contain the display elements.

    /** Display group for the control tabs. */
    private JTabbedPane paneControls = new JTabbedPane();

    /** Control group for the robot body settings. */
    private static PlayerFrame controlsRobot;// = new PlayerFrame();

    
	static class PlayerFrame extends JPanel {
		JFrame frame;
		public PlayerFrame() {
			frame = new JFrame("Player");
			//---
			//displayPanel = new PlayerFrame();
			//frame.getContentPane().add(displayPanel, BorderLayout.CENTER);
			//displayPanel.setVisible(true);
			//---
			//frame.getContentPane().add(this, BorderLayout.CENTER);
			frame.add(this, BorderLayout.CENTER);
			// Remove window title and borders
	        //frame.setUndecorated(true);
	        // Make frame topmost
	        //frame.setAlwaysOnTop(true);
	        // Disable Alt+F4 on Windows
	        //frame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
	        // Make frame full-screen
	        frame.setExtendedState(Frame.MAXIMIZED_BOTH);
	        // Display frame
			this.setVisible(true);
			frame.pack();
			//frame.setSize(new Dimension(640, 480));
			frame.setVisible(true);
		}
		private java.awt.Image lastFrame;
		public void setLastFrame(java.awt.Image lf) {
			if( lf == null ) return;
			if( lastFrame == null ) {
				lastFrame = lf;
				return;
			}
			synchronized(lastFrame) {
				//lastFrame.getGraphics().dispose();
				lastFrame = lf; 
			} 
		}
		//public void paint(Graphics g) {
		public void paintComponent(Graphics g) {
			super.paintComponent(g);
			if( lastFrame == null )
				return;
			synchronized(lastFrame) {
				g.drawImage(lastFrame, 0, 0, lastFrame.getWidth(this), lastFrame.getHeight(this), this);
				lastFrame.flush();
			}
		}
	}
    /** Control group for timing settings. */
    private JPanel controlsTiming = new JPanel();

    /** Control group for the speed settings. */
    private JPanel controlsSpeed = new JPanel();

    /** The canvas for displaying the robot's tracks. */
    private FloatCanvas paneTracks = new FloatCanvas();

    /** Display group for the position information (text). */
    private PositionPanel panePosition;

    // END of display elements variables.

    // These variables contain spacings used in the display.

    /** Constant padding vertical padding of 3 pixels. */
    private final Dimension vpad3 = new Dimension(1,3);

    /** Constant padding vertical padding of 5 pixels. */
    private final Dimension vpad5 = new Dimension(1,5);

    /** Constant padding horizontal padding of 5 pixels. */
    private final Dimension hpad5 = new Dimension(5,1);

    // END of display spacings variables.

    // These variables are used for storing robot attributes used
    // in the simulation, and for storing the results of the simulation
    // (e.g., true position of each wheel).

    /** The duration of the simulation, in seconds. */
    public static double      simulationTime = 2.0;

    /** The time between dead-reckoning calculations, in seconds. */
    private double      deadReckoningInterval = 1;//0.01;

    /** This is the object used to simulate the robot's movements. */
    private IdealDrive  theWheels = new IdealDrive();

    /** The number of steps used for displaying the true path and
     *  the dead-reckoned path.   numSteps is computed and used
     *  throughout the class as the number of steps for displaying the
     *  true path.  maxDRSegments are used to limit the number
     *  of segments we use for the DR track.   maxNumSteps could
     *  benefit from going a bit bigger than the 250 that is wired in,
     *  but the number of cases for which we need more steps
     *  is small and we wish to avoid excessive memory consumption.
     *  When more than maxNumSteps are required, the path will tend
     *  to exhibit places where it makes sharp bends rather than
     *  smooth changes in direction.   These bends are artifact of
     *  the graphics routines, not errors in the position calculations.
     */
    private static int         maxNumSteps   = 10000;
    //private int         numSteps      = 100;
    private static int         maxDRSegments = 10000;

    /**
     * A temporary variable used to determine the time between each
     * simulated leg used for drawing the true path.
     */
    private double      stepSize;

    /** Temporary storage for the robot's position. */
    private Position    pos = new Position();

    /** This variable is used to store the  path of the center of the robot,
     * which is then used to derive the position of the wheels.
     */
    public static Position    plotData[] = new Position[maxNumSteps + 1];

    /** Storage for the path of the center of the robot, which will
     * be used for drawing the track.
     */
    static FPoint      centerPoints[] = new FPoint[maxNumSteps + 1];

    /** Storage for the path of the left wheel of the robot, which will
     * be used for drawing the track.
     */
    private static FPoint      leftPoints[] = new FPoint[maxNumSteps + 1];

    /** Storage for the path of the right wheel of the robot, which will
     * be used for drawing the track.
     */
    private static FPoint      rightPoints[] = new FPoint[maxNumSteps + 1];

    /** Storage for the dead-reckoned path of the robot. */
    private static Position    deadReckonPos[] = new Position[maxDRSegments+1];

    /** Storage for the dead-reckoned path of the robot using the mean of theta. */
    private static Position    deadReckonMeanPos[] = new Position[maxDRSegments+1];

    /** Storage for the true path of the robot. */
    private static Position    trueReckonPos[] = new Position[maxDRSegments+1];
    
    /** Log data from run. err, left speed, right speed, dterm, iterm, pid, target speed, imu */
    public static double[][] runData = new double[8][maxNumSteps+1];
    
    /** current position of data slider */
    public static double currentPosition = 0;

    /** Temporary variable. */
    private int         nSegment;
	private Robot robot;

    // END of simulation variables.
    public MotionApp() throws IOException {
    	init();
    }
    /**
     * Initialize the app.
     * @throws IOException 
     */
    public void init() throws IOException {
    /*
    * Set up controls for Robot tab, which includes:
    *     Initial Position
    *         Slider for X coordinate
    *         Slider for Y coordinate
    *         Slider for direction (theta)
    *     Body Settings
    *         Body Width
    */
    robot = new Robot();
    controlsRobot = new PlayerFrame();
   // Basic set up for the tab:
        controlsRobot.setLayout(new BoxLayout(controlsRobot,
                                              BoxLayout.Y_AXIS));

        controlsRobot.setBorder(new LineBorder(Color.magenta, 1));
        controlsRobot.add(Box.createRigidArea(vpad5));

   // Create a panel for the Initial Position controls:
        JPanel theSliderPanel = new JPanel();
        theSliderPanel.setLayout(new BoxLayout(theSliderPanel,
                                               BoxLayout.Y_AXIS));
        theSliderPanel.setBorder(new TitledBorder("Initial Position"));
        System.out.println("added slider "+theSliderPanel);

   /*
    * Add tick marks for the initial position sliders, and
    * add this object as a "ChangeListener" to each.
    */
        controlPositionX.addTicks(10,5);
        controlPositionX.addChangeListener(this);

        controlPositionY.addTicks(10,5);
        controlPositionY.addChangeListener(this);

        controlPositionTheta.steps = numSteps;
        controlPositionTheta.addTicks(10,5);
        controlPositionTheta.addChangeListener(this);

   /*
    * Add the sliders to the panel, with some spacing
    * between them, then add the panel to the robot panel.
    * Insert some spacing as well.
    */
        theSliderPanel.add(Box.createRigidArea(vpad3));
        theSliderPanel.add(controlPositionX);
        theSliderPanel.add(Box.createRigidArea(vpad3));
        theSliderPanel.add(controlPositionY);
        theSliderPanel.add(Box.createRigidArea(vpad3));
        theSliderPanel.add(controlPositionTheta);
        // body width
        theSliderPanel.add(Box.createRigidArea(vpad3));
        controlBodyWidth.addTicks(5,5);
        controlBodyWidth.addChangeListener(this);
        theSliderPanel.add(controlBodyWidth);
        //
        System.out.println("adding slider to main panel"+controlsRobot);
        controlsRobot.add(theSliderPanel);
        controlsRobot.add(Box.createRigidArea(vpad3));

   /*
    * Create a panel for the Body Settings controls
    * (of which "Width" is currently the only one).  Add
    * tick marks to the Body Width control, add this object
    * as a change listener, and add the new panel to the
    * robot tab panel.
    
        JPanel theBodyPanel = new JPanel();
        theBodyPanel.setLayout(new BoxLayout(theBodyPanel,
                                             BoxLayout.Y_AXIS));
        theBodyPanel.setBorder(new TitledBorder("Body Settings"));

        controlBodyWidth.addTicks(5,5);
        controlBodyWidth.addChangeListener(this);

        theBodyPanel.add(controlBodyWidth);
        System.out.println("added panel "+theBodyPanel);
        controlsRobot.add(theBodyPanel);
        controlsRobot.add(Box.createRigidArea(vpad3));
        System.out.println("added panel to main panel "+controlsRobot);
    */

   /*
    * Set up the controls for Speed tab, which include
    * sliders for the velocity and acceleration of each
    * wheel.
    */
   // Basic setup for the panel:
        controlsSpeed.setLayout(new BoxLayout(controlsSpeed,
                                              BoxLayout.Y_AXIS));
        controlsSpeed.setBorder(new LineBorder(Color.red, 1));
        controlsSpeed.add(Box.createRigidArea(vpad5));

   /*
    * Create a panel for the velocity sliders.  Add the
    * controls to this panel, then add the panel to the Speed
    * tab panel.
    */
        JPanel theVelocityPanel = new JPanel();
        theVelocityPanel.setLayout(new BoxLayout(theVelocityPanel,
                                                 BoxLayout.Y_AXIS));
        theVelocityPanel.setBorder(new TitledBorder("Velocities"));

        controlVelocityLeft.addTicks(10,5);
        controlVelocityLeft.addChangeListener(this);

        controlVelocityRight.addTicks(10,5);
        controlVelocityRight.addChangeListener(this);

   /*
    * Create a panel for the Acceleration sliders.  Add the
    * controls to this panel, then add the panel to the Speed
    * tab panel.
    */

        JPanel theAccelerationPanel = new JPanel();
        theAccelerationPanel.setLayout(new BoxLayout(theAccelerationPanel,
                                                     BoxLayout.Y_AXIS));
        theAccelerationPanel.setBorder(new TitledBorder("Accelerations"));

        controlAccelerationLeft.addTicks(10,5);
        controlAccelerationLeft.addChangeListener(this);

        controlAccelerationRight.addTicks(10,5);
        controlAccelerationRight.addChangeListener(this);


        theVelocityPanel.add(Box.createRigidArea(vpad3));
        theVelocityPanel.add(controlVelocityRight);
        theVelocityPanel.add(Box.createRigidArea(vpad3));
        theVelocityPanel.add(controlVelocityLeft);
        theVelocityPanel.add(Box.createRigidArea(vpad3));

        theAccelerationPanel.add(Box.createRigidArea(vpad3));
        theAccelerationPanel.add(controlAccelerationRight);
        theAccelerationPanel.add(Box.createRigidArea(vpad3));
        theAccelerationPanel.add(controlAccelerationLeft);
        theAccelerationPanel.add(Box.createRigidArea(vpad3));

        controlsSpeed.add(theVelocityPanel);
        controlsSpeed.add(theAccelerationPanel);
        System.out.println("speed control panel set up "+controlsSpeed);

   /*
    * Set up the controls for the Timing tab, which
    * consist of two panels:  one for the simlulation
    * duration slider, and one for the dead-reckoning
    * interval slider.
    */
   // Basic panel setup:
        controlsTiming.setLayout(new BoxLayout(controlsTiming,
                                               BoxLayout.Y_AXIS));
        controlsTiming.setBorder(new LineBorder(Color.cyan, 1));
        controlsTiming.add(Box.createRigidArea(vpad5));

   /*
    * Create a panel for the Simulation section.  Add the
    * duration slider control, and add the panel to the
    * Timing tab panel.
    */
        JPanel theTimePanel = new JPanel();
        theTimePanel.setLayout(new BoxLayout(theTimePanel,
                                             BoxLayout.Y_AXIS));
        theTimePanel.setBorder(new TitledBorder("Simulation"));

        controlDuration.addTicks(10, 5);
        controlDuration.addChangeListener(this);

        theTimePanel.add(controlDuration);
        controlsTiming.add(theTimePanel);
        System.out.println("added timing panel "+controlsTiming);

   /*
    * Create a panel for the Dead-Reckoning section.  Add the
    * interval slider control, and add the panel to the
    * Timing tab panel.
    */
        JPanel theDeadReckoningPanel = new JPanel();
        theDeadReckoningPanel.setLayout(new BoxLayout(theDeadReckoningPanel,
                                                      BoxLayout.Y_AXIS));
        theDeadReckoningPanel.setBorder(new TitledBorder("Dead Reckoning"));

        theDeadReckoningPanel.add(controlDeadReckoningInterval);

        controlDeadReckoningInterval.addTicks(10,5);
        controlDeadReckoningInterval.addChangeListener(this);

        controlsTiming.add(theDeadReckoningPanel);
        System.out.println("dead reckoning panel added to timing controls "+theDeadReckoningPanel);

   /*
    * Add the tab panels to the Controls panel.
    */
        paneControls.addTab("Robot", controlsRobot);
        paneControls.addTab("Speed", controlsSpeed);
        paneControls.addTab("Timing", controlsTiming);
        System.out.println("pane controls updated "+paneControls);

   /*
    * Create a panel for displaying the robot's path:
    */
        JPanel canvasPanel = new JPanel(new GridLayout());
        canvasPanel.add(paneTracks);
        canvasPanel.setBorder(new LineBorder(Color.blue, 1));
        System.out.println("panel for path display created "+canvasPanel);
   // Create & setup the new panel:
        panePosition = new PositionPanel();

   /*
    * Set up the layout for the main window.  Set up the
    * layout for and insert the canvas panl, controls pane,
    * and position feedback pane.
    */
        GridBagLayout      gbl = new GridBagLayout();
        GridBagConstraints gbc = new GridBagConstraints();
        //getContentPane().setLayout(gbl);
        controlsRobot.setLayout(gbl);
        System.out.println("main panel layout set to "+gbl);
        // Add the canvas pane to the main window
        gbc.anchor=GridBagConstraints.NORTHWEST;
        gbc.fill=GridBagConstraints.BOTH;
        gbc.gridx=0;
        gbc.gridy=0;
        gbc.gridheight=2;
        gbc.gridwidth=1;
        gbc.weightx=1;
        gbc.weighty=1;
        gbl.setConstraints(canvasPanel, gbc);
        //getContentPane().add(canvasPanel);
        controlsRobot.add(canvasPanel);
        System.out.println("canvas panel added "+canvasPanel);

        // Add the controls pane to the main window
        gbc.fill=GridBagConstraints.HORIZONTAL;
        gbc.gridx=1;
        gbc.gridy=0;
        gbc.gridheight=1;
        gbc.gridwidth=1;
        gbc.weightx=0;
        gbc.weighty=0;
        gbl.setConstraints(paneControls, gbc);
        //getContentPane().add(paneControls);
        controlsRobot.frame.add(paneControls);
        System.out.println("pane controls added "+paneControls);

   // Add the position feedback pane to the main window
        gbc.fill=GridBagConstraints.HORIZONTAL;
        gbc.gridx=1;
        gbc.gridy=1;
        gbc.gridheight=1;
        gbc.gridwidth=1;
        gbc.weightx=0;
        gbc.weighty=0;
        gbl.setConstraints(panePosition, gbc);
        //getContentPane().add(panePosition);
        controlsRobot.add(panePosition);
        System.out.println("pane position added "+panePosition);

   /*
    * Set all the controls to known initial values
    */
        controlPositionX.setValue(0);
        controlPositionY.setValue(0);
        controlPositionTheta.setValue(0);
        controlVelocityLeft.setValue(2.5);
        controlVelocityRight.setValue(2.4);
        controlAccelerationLeft.setValue(0.0);
        controlAccelerationRight.setValue(0.0);
        controlBodyWidth.setValue(1.0);
        controlDuration.setValue(simulationTime);
        controlDeadReckoningInterval.setValue(0.1);
        controlDeadReckoningInterval.setValue(0.5);

   /*
    * Now that the controls have been set, calculate and
    * display the position data.
    */
        System.out.println("computing position data...");
        computePositionData();

   /*
    * Finally, install this object as a paint function for the
    * canvas pane.
    */
        paneTracks.installPaintFunc(this);
    }

    /**
     * Handle "ChangeEvents", which occur when a slider changes
     * value.  Use the "isEventSource" functions for each
     * "JFBSlider" object to determine which control generated the
     * event.  For most events, we just get the new value, and pass
     * it to the appropriate function to set the corresponding
     * control.
     */
    public void stateChanged(ChangeEvent e) {
        if (controlPositionX.isEventSource(e)) {
            theWheels.setX0(controlPositionX.getValue());
        } else if (controlPositionY.isEventSource(e)) {
            theWheels.setY0(controlPositionY.getValue());
        } else if (controlPositionTheta.isEventSource(e)) {
            currentPosition = controlPositionTheta.getValue();
        } else if (controlVelocityLeft.isEventSource(e)) {
            theWheels.setVelocityLeft(controlVelocityLeft.getValue());
        } else if (controlVelocityRight.isEventSource(e)) {
            theWheels.setVelocityRight(controlVelocityRight.getValue());
        } else if (controlAccelerationLeft.isEventSource(e)) {
            theWheels.setAccelerationLeft(controlAccelerationLeft.getValue());
        } else if (controlAccelerationRight.isEventSource(e)) {
            theWheels.setAccelerationRight(controlAccelerationRight.getValue());
        } else if (controlBodyWidth.isEventSource(e)) {
            theWheels.setBodyWidth(controlBodyWidth.getValue());
        } else if (controlDuration.isEventSource(e)) {
            simulationTime = controlDuration.getValue();
        } else if (controlDeadReckoningInterval.isEventSource(e)) {
            double t = controlDeadReckoningInterval.getValue();

            /*
             * Since a dead-reckoning interval of 0 is not
             * possible, set it to 0.1 instead.  I did this
             * instead of setting the minimum limit of the control
             * to 0.1, because it gives the control a nice, even
             * midpoint label.
             */
              if (t == 0) {
                   t = 0.1;
                   controlDeadReckoningInterval.setValue(t);
              };
              deadReckoningInterval = t;
        } else {
            System.out.println(e.toString());
        }

        /*
         * Whatever the event, update the position feedback
         * readouts, and redraw the paths.
         */
        computePositionData();
        updatePositionValues();
        paneTracks.repaint();
    }


    /**
     * Paint the applet&mdash;redraw the scales and the paths.
     *
     * @param g The graphics context in which to draw.
     */
    public void paint(Graphics g) {
        //Draw a Rectangle around the applet's display area.
        //g.drawRect(0, 0, getSize().width - 1, getSize().height - 1);
        g.drawRect(0, 0, controlsRobot.getSize().width-1, controlsRobot.getSize().height-1);

        // Redraw all the contents of the applet window.
        controlsRobot.paintComponents(g);
    }


    /**
     * Update the true and dead-reckoned fields in the
     * position feedback pane.
     * runData[0][numSteps] = err;
     * runData[1][numSteps] = left;
     * runData[2][numSteps] = right;
     * runData[3][numSteps] = dterm;
     * runData[4][numSteps] = iterm;
     * runData[5][numSteps] = pid;
     * [6] = speed
     * [7] = imu
     */
    public void updatePositionValues(){

        if(panePosition==null)
            return;  // do nothing
        Position pos  = theWheels.positionAt(currentPosition);
        double vLeft  = theWheels.getVelocityLeft(currentPosition);
        double vRight = theWheels.getVelocityRight(currentPosition);

        panePosition.setComputedValues(
                                      pos.x,
                                      pos.y,
                                      pos.theta,
                                      runData[0][(int) currentPosition],
                                      runData[6][(int) currentPosition],
                                      runData[5][(int) currentPosition],
                                      runData[4][(int) currentPosition],
                                      runData[7][(int) currentPosition],
                                      vLeft,
                                      vRight
                                      );
    }


    /**
     * Calculate the true and dead-reckoned position data, which
     * will be used for displaying the tracks and the information
     * in the position feedback pane.
     */
    public void computePositionData(){

        //  compute the true position data

        //numSteps = theWheels.getSimpsonIntervals(simulationTime);
        //if(numSteps<30)
        //   numSteps=30;
        //else if(numSteps>maxNumSteps)
         //  numSteps=maxNumSteps;
        stepSize = 1; //simulationTime / numSteps;
        //for (int i=0; i<=numSteps; i++) {
        //    pos = theWheels.positionAt(i * stepSize);
        //    plotData[i] = pos;
        //    centerPoints[i] = pos;
        //    leftPoints[i] = IdealDrive.LeftWheelLoc(pos);
        //    rightPoints[i] = IdealDrive.RightWheelLoc(pos);
        //}

        //compute the dead-reckoned position data

        nSegment = numSteps;//(int)Math.floor(simulationTime/deadReckoningInterval);
        
        double deltaT = deadReckoningInterval;
        /*
        if(simulationTime-nSegment*deltaT>0.01){
            // the simulation time does not come out to an even
            // multiple of deadReckoningIntervals
            nSegment++;
        }

        if(nSegment>maxDRSegments){
            // the dead-reckoning interval results in too many segments
            // (more than that for which we wish to allocate memory),
            // so we need to make some adjustments.  Unfortunately,
            // the user will not see the results of the dead-reckoning
            // that he requested, but at this point is is using so fine
            // an interval that it doesn't matter anymore.
            nSegment=maxDRSegments;
            deltaT=simulationTime/maxDRSegments;
        }
         */
        deadReckonPos[0] = theWheels.positionAt(0);
        deadReckonMeanPos[0] = deadReckonPos[0];
        trueReckonPos[0] = deadReckonPos[0];
        double trueTime=0;
        double vLeft0, vLeft1, vRight0, vRight1;
        vLeft1  = theWheels.getVelocityLeft(0.0);
        vRight1 = theWheels.getVelocityRight(0.0);
        for (int iSegment=1; iSegment<=nSegment; iSegment++){
        	
            if(iSegment==nSegment){
                // we are on the last segment.  recall the simulation time
                // isn't necessarily an even multiple of the dead-reckoning
                // interval (both values are arbitrary user inputs), so this
                // last segment could be a little shorter than all the previous
                // ones.    we recompute it just in case.
                deltaT=simulationTime-(iSegment-1)*deltaT;
            }
            trueTime+=deltaT;
            vLeft0  = vLeft1;
            vRight0 = vRight1;
            vLeft1  = theWheels.getVelocityLeft(trueTime);
            vRight1 = theWheels.getVelocityRight(trueTime);
            // Dead-reckon a new position:
            // sLeft and sRight are the displacements of the wheels (may be <0)
            // for the next segment, theta is the new orientation (at the end
            // of the segment).  This usage follows the results that are
            // obtained with the standard dead-reckoning approach.
            double sLeft  = deltaT*(vLeft0+vLeft1)/2.0;
            double sRight = deltaT*(vRight0+vRight1)/2.0;
            double sMean  = (sRight+sLeft)/2;
            double wTrack = theWheels.getBodyWidth();
            double theta = deadReckonPos[iSegment-1].theta +
                (sRight-sLeft)/wTrack;
						double theta_mean = (theta+deadReckonPos[iSegment-1].theta) / 2;

            deadReckonPos[iSegment] =
                new Position( deadReckonPos[iSegment-1].x +
                              sMean*Math.cos(theta),
                              deadReckonPos[iSegment-1].y +
                              sMean*Math.sin(theta),
                              theta
                    );

            deadReckonMeanPos[iSegment] =
                new Position( deadReckonMeanPos[iSegment-1].x +
                              sMean*Math.cos(theta_mean),
                              deadReckonMeanPos[iSegment-1].y +
                              sMean*Math.sin(theta_mean),
                              theta
                    );
             
            trueReckonPos[iSegment]=theWheels.positionAt(iSegment/*trueTime*/);
        }
    }

    /**
     * Draw the tracks in the FloatCanvas pane.  This function
     * is installed in the pane, and will be called whenever that
     * pane is redrawn.
     */
    public void drawFunc(FloatCanvas theFloatCanvas) {
        int i;
        Color shadow = new Color(232, 232, 232);
        FPoint fpd[] = new FPoint[3];

   /*
    * Find the min and max X and Y values, and set the limits
    * of the FloatCanvas based on them.
    */
        double xMin = plotData[0].x, xMax = xMin;
        double yMin = plotData[0].y, yMax = yMin;

        for (i=0; i<=numSteps; i++) {
            if (plotData[i].x < xMin) {
                xMin = plotData[i].x;
            } else if (plotData[i].x > xMax) {
                xMax = plotData[i].x;
            }

            if (plotData[i].y < yMin) {
                yMin = plotData[i].y;
            } else if (plotData[i].y > yMax) {
                yMax = plotData[i].y;
            }
        }
   // Pad the limits with enough space for the robot
        double bodyWidth = theWheels.getBodyWidth();
        xMin -= bodyWidth;
        xMax += bodyWidth;
        yMin -= bodyWidth;
        yMax += bodyWidth;

   // Use the padded limits to set the range of the FloatCanvas
        theFloatCanvas.setLimits(xMin, xMax, yMin, yMax);
        /*
   // Draw the dead-reckoned wheel tracks.
   FPoint dPoly[] = new FPoint[5];
        dPoly[0] = IdealDrive.LeftWheelLoc(deadReckonPos[0]);
        dPoly[1] = IdealDrive.RightWheelLoc(deadReckonPos[0]);
        for (int iSegment=1; iSegment<=nSegment; iSegment++){
            dPoly[2] = IdealDrive.RightWheelLoc(deadReckonPos[iSegment]);
            dPoly[3] = IdealDrive.LeftWheelLoc(deadReckonPos[iSegment]);
            dPoly[4]=dPoly[0];
            theFloatCanvas.fillPolygon(dPoly, shadow);
            theFloatCanvas.drawPolygon(dPoly, Color.white);
            dPoly[0] = dPoly[3];
            dPoly[1] = dPoly[2];
        }

        // draw the triangle at the end of the dead-reckoned track
        fpd[0] = IdealDrive.LeftWheelLoc( deadReckonPos[nSegment]);
        fpd[1] = IdealDrive.NoseLoc(      deadReckonPos[nSegment]);
        fpd[2] = IdealDrive.RightWheelLoc(deadReckonPos[nSegment]);
        theFloatCanvas.fillPolygon(fpd,shadow);


        // Draw the triangles for the dead-reckoned positions
        for (int iSegment=1; iSegment<=nSegment; iSegment++){
            fpd[0] = IdealDrive.LeftWheelLoc( deadReckonPos[iSegment]);
            fpd[1] = IdealDrive.NoseLoc(      deadReckonPos[iSegment]);
            fpd[2] = IdealDrive.RightWheelLoc(deadReckonPos[iSegment]);
            theFloatCanvas.drawPolygon(fpd, Color.darkGray);

            fpd[0] = IdealDrive.LeftWheelLoc( deadReckonMeanPos[iSegment]);
            fpd[1] = IdealDrive.NoseLoc(      deadReckonMeanPos[iSegment]);
            fpd[2] = IdealDrive.RightWheelLoc(deadReckonMeanPos[iSegment]);
            theFloatCanvas.drawPolygon(fpd, Color.cyan);
        }
	*/
        // Draw the shadow for inertial heading intended
        FPoint dPoly[] = new FPoint[5];
        dPoly[0] = leftPoints[0];
        dPoly[1] = rightPoints[0];
        dPoly[2] = new FPoint(numSteps,rightPoints[0].y);
        dPoly[3] = new FPoint(numSteps,leftPoints[0].y);
        dPoly[4]=dPoly[0];
        theFloatCanvas.fillPolygon(dPoly, shadow);
        theFloatCanvas.drawPolygon(dPoly, Color.white);
        
		// Draw the blue, green and red lines tracing the true track
        theFloatCanvas.drawPolyline(centerPoints, numSteps+1);
        theFloatCanvas.drawPolyline(leftPoints,   numSteps+1, Color.green);
        theFloatCanvas.drawPolyline(rightPoints,  numSteps+1, Color.red);
		float PID_THRESHOLD  = robot.getIMUSetpointInfo().getMaximum()/2; // point at which PID engages/disengages
		float TRIANGLE_THRESHOLD = robot.getIMUSetpointInfo().getMaximum();
        // Draw the triangles for the true positions
        for (int iSegment=0; iSegment<=numSteps/*nSegment*/; iSegment++){
            fpd[0] = IdealDrive.LeftWheelLoc( trueReckonPos[iSegment]);
            fpd[1] = IdealDrive.NoseLoc(      trueReckonPos[iSegment]);
            fpd[2] = IdealDrive.RightWheelLoc(trueReckonPos[iSegment]);
            if( iSegment == currentPosition )
            	theFloatCanvas.fillPolygon(fpd, Color.red);
            if( Math.abs(runData[0][iSegment]) <= PID_THRESHOLD)
            	theFloatCanvas.drawPolygon(fpd, Color.blue);
            else
            	if( Math.abs(runData[0][iSegment]) <= TRIANGLE_THRESHOLD)
            		theFloatCanvas.drawPolygon(fpd, Color.DARK_GRAY);
            	else
            		theFloatCanvas.drawPolygon(fpd, Color.MAGENTA); // arc solution
        }
    }
    
    public static void main(String[] args) throws Exception {
    	setPoint = args[0];
       	CountDownLatch latch = new CountDownLatch(1);
    	new Thread(new fileReader(latch)).start();
    	latch.await();
		SwingUtilities.invokeLater(new Runnable() {
		    public void run() {
		    	try {
					new MotionApp();
				} catch (IOException e) {
					throw new RuntimeException(e);
				}
		        
		    }
		});
    }

    static class fileReader implements Runnable {
    	private CountDownLatch latch;
		public fileReader(CountDownLatch latch) {
			this.latch = latch;
		}

		@Override
    	public void run() {
    		try {
    			FileReader fis = new FileReader("motion.log");
    			BufferedReader br = new BufferedReader(fis);
    			String s = "";
    			float err=0, left, right, dterm, pterm, pid, speed = 0, imu = 0;
    			numSteps = 0;
    			double baseline = (IdealDrive.bodyWidth * Math.cos((2.0 * Math.PI)))*100;
    			while(s != null) {
    				s = br.readLine();
    				if(s != null && s.startsWith("Inertial Setpoint="+setPoint) ) {
    					int pl = s.indexOf("speedL=");
    					// newline if it went into triangle solutions
    					// if we found SPEEDL initially, we will also find 'SPEED'
    					if( pl == -1 ) {
    						// if we did NOT find SPEEDL initially, look for xxxxANGLE=
    						int pa = s.indexOf("ANGLE="); // this has speed for this log line
    						if( pa != -1 ) {
    							speed = Float.valueOf(s.substring(pa+6,s.indexOf("|",pa+6)));
    						} else {
    							System.out.println("***"+numSteps+" CANT PROCESS SPEED LOOKING FOR xxxxANGLE="+s);
    						}
    						s = br.readLine(); // try again on next line
    						pl = s.indexOf("speedL=");
    					} else {
     						int pa = s.indexOf("Speed="); // this has speed for this log line
    						if( pa != -1 ) {
    							speed = Float.valueOf(s.substring(pa+6,s.indexOf("|",pa+6)));
    						} else {
    							System.out.println("***"+numSteps+" CANT PROCESS SPEED LOOKING FOR Speed="+s);
    						}
    						// extract IMU heading reading
      						int pi = s.indexOf("IMU=");
    						if( pi != -1 ) {
    							imu = Float.valueOf(s.substring(pi+4,s.indexOf("|",pi+4)));
    						} else {
    							System.out.println("***"+numSteps+" CANT PROCESS IMU LOOKING FOR IMU="+s);
    						}
    					}
    					if( pl != -1 ) {
    	  					System.out.println(s);
    						left = Float.valueOf(s.substring(pl+7,s.indexOf("|",pl+7)));
    						int pr = s.indexOf("speedR=");
    						right = Float.valueOf(s.substring(pr+7,s.indexOf("|",pr+7)));
    						// extract IMU heading reading
      						int pi = s.indexOf("IMU=");
    						if( pi != -1 ) {
    							imu = Float.valueOf(s.substring(pi+4,s.indexOf("|",pi+4)));
    						} else {
    							System.out.println("***"+numSteps+" CANT PROCESS IMU LOOKING FOR IMU="+s);
    						}
    						// next line should be OUTPUT
    						s = br.readLine();
    						int p1 = s.indexOf("Err = ");
    						if( p1 != -1 ) {
    							err = Float.valueOf(s.substring(p1+6,s.indexOf(" ",p1+6)));
    						} else {
    							System.out.println("***"+numSteps+" CANT FIND ERR ENTRY="+s);
    						}
    						int p2 = s.indexOf("DTerm = ");
    						if( p2 != -1) {
    							dterm = Float.valueOf(s.substring(p2+8,s.indexOf(" ",p2+8)));
    							int p3 = s.indexOf("Output = ");
    							pterm = Float.valueOf(s.substring(p3+8,s.indexOf(" ",p3+9)));
    							int p4 = s.indexOf("PID=");
    							pid = Float.valueOf(s.substring(p4+4,s.indexOf(" ",p4+4)));
    							System.out.println(s+" "+err+" "+left+" "+right+" "+dterm+" "+pterm+" "+pid);
    							runData[0][numSteps] = err;
    							runData[1][numSteps] = left;
    							runData[2][numSteps] = right;
    							runData[3][numSteps] = dterm;
    							runData[4][numSteps] = pterm;
    							runData[5][numSteps] = pid;
    							runData[6][numSteps] = speed;
    							runData[7][numSteps] = imu;
    							// Calculate chord of offset error as radius
    							// chord is 2Rsin(theta/2) ( we convert to radians first)
    							//double chord = 2 * (IdealDrive.bodyWidth/2) * Math.sin((Math.abs(err/360.0) * (2.0 * Math.PI))/2);
    							// step will increment as time along the X axis, chord is R
    							//double x = numSteps + chord * Math.sin((err/360.0) * (2.0 * Math.PI));
    							double y = 0;
    							if( err > 0 )
    								// convert to get result above baseline
    								y = baseline+(baseline-((IdealDrive.bodyWidth * Math.cos(((err/360.0) * (2.0 * Math.PI))))*100));
    							else
    								y = (IdealDrive.bodyWidth * Math.cos((err/360.0) * (2.0 * Math.PI)))*100;
    							centerPoints[numSteps] =  new FPoint(numSteps,y);
    							Position pos = new Position();
    							pos.set(centerPoints[numSteps],((dterm/360.0) * (2.0 * Math.PI)));
    							plotData[numSteps] = pos;
    					        leftPoints[numSteps] = IdealDrive.LeftWheelLoc(pos);
    					        rightPoints[numSteps] = IdealDrive.RightWheelLoc(pos);
    							System.out.println(numSteps+"="+pos+" l/r locs:"+leftPoints[numSteps]+","+rightPoints[numSteps]);
    							++numSteps;
    						} else {
    							System.out.println("***"+numSteps+" REJECTED CANT FIND PID LINE="+s);
    						}
    						
    					}
    				}
    			}
    			--numSteps;
    			simulationTime = numSteps;
    			System.out.println("Done..");
    			br.close();
    			fis.close();
    			latch.countDown();
    		} catch (Exception e) {
    			// TODO Auto-generated catch block
    			e.printStackTrace();
    		}
    	}
    	
    }
}

