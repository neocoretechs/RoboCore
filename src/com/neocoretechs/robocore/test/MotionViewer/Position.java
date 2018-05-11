package com.neocoretechs.robocore.test.MotionViewer;
/**
 * Position describes where the robot is, by extending the FPoint class to 
 * add the direction the robot is facing.
 * 
 * @author <a href="mailto:MikeGauland@users.sourceforge.net">Michael
 *     Gauland</a>
 * @version 1.
 */
class Position extends FPoint {
    /**
     * The direction the robot is facing.  Since this class provides no
     * functions that operate on theta, it is entirely up to the client
     * whether theta is in degrees or radians.
     */
    double theta;

    Position() {
	// Rely on FPoint default constructor; Java will set theta to 0.
    }
   
    /**
     * Constructor to initialize the object with x and y coordinates, and
     * the direction.
     *
     * @param newX The initial X coordinate.
     * @param newY The initial Y coordinate.
     * @param newTheta The initial direction.
     */
    Position (double newX, double newY, double newTheta){
        super.set(newX, newY);
        theta = newTheta;
    }
    
    /**
     * Set the location and direction of the robot.
     *
     * @param newLocation The coordinates of the new position.
     * @param newTheta The direction the robot is now facing.
     */
    public void set(FPoint newLocation, double newTheta) {
        super.set(newLocation);
        theta = newTheta;
    }
    
    @Override
    public String toString() {
    	return super.toString()+" theta="+theta;
    }
}