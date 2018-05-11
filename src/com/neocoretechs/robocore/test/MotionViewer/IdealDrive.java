/**
 * IdealDrive simulates a differential-drive robot, with no slippage,
 * and other simplifications.
 *
 * All linear measurements (i.e., body width, distance traveled, etc.) are
 * assumed to be in the same units (e.g., meters), as are all time settings
 * in the simulation (e.g., duration).  Velocity is also expressed in these
 * same units (i.e., if you assume the velocity is in meters/second, then the
 * duration of the simulation is expressed in seconds.).
 *
 * @author <a href="mailto:MikeGauland@users.sourceforge.net">
 * Michael Gauland</a>
 * @version 2.
 *
 * Addition of Simpson's Rule implementation to treat acceleration,
 * contributed by Jing YE, Univeristy of Melbourne, Jan 2001
 */

package com.neocoretechs.robocore.test.MotionViewer;
public class IdealDrive {
    /** the starting position, for simulations. */
    Position initialPos = new Position();

    /**
     * The width of the robot's body--i.e., the distance
     * between the drive wheels.
     */
    public static double bodyWidth = 0.7;

    /**
     * The velocity of the left drive wheel.
     * <P>See the note at the top of this file for information about
     * measurement units.</P>
     */
    double velocityLeft = 0.0;

    /**
     * The velocity of the right drive wheel.
     * <P>See the note at the top of this file for information about
     * measurement units.</P>
     */
    double velocityRight = 0.0;

    /**
     * The acceleration of the left drive wheel
     */
    double accelerationLeft = 0.0;

    /**
     * The acceleration of the right drive wheel
     */
    double accelerationRight = 0.1;


    /**
     * Since the sine of the starting angle is used whenever we calculate the
     * true position of the robot at a given time, it will be calculated only
     * when the angle is set.
     */
    double sinTheta0;

    /**
     * Since the cosine of the starting angle is used whenever we calculate the
     * true position of the robot at a given time, it will be calculated only
     * when the angle is set.
     */
    double cosTheta0;


    /**
     * A variable that caches the time.
     */
    double cachedTime = 0;

    /**
     * The maximum-error criterion used for selecting intervals
     *
     */
    double maxAllowableError = 0.01;

    /**
     * The default constructor does nothing, leaving all variables at their
     * default values.
     */
    IdealDrive() {
    }

    public static void main(String [] args){

        IdealDrive ix = new IdealDrive();

        ix.setVelocityRight(2.5);
        ix.setVelocityLeft(2.0);
        ix.setAccelerationRight(0.70);
        ix.setAccelerationLeft(0.89);
        ix.setBodyWidth(0.1);
        for(int i=120; i<130; i++){
           int n = ix.getSimpsonIntervals(i/10.0);
           System.out.println("time, n intervals): "+(i/10.0)+",  "+n);
        }
    }




    /**
     * To facilitate debugging, this function generates a string representing
     * the settings of the robot.
     */
    public String toString() {
        String result = new String();

        result = "Start at: (" + initialPos.x + ", " +
            initialPos.y + "); facing " +
            (initialPos.theta * 180 / Math.PI) + "\nBody: " +
            bodyWidth + "; Vels: " + velocityLeft + ", " + velocityRight;
        return result;
    }

    /** Set the initial position of the robot. */
    public void setInitialLocation(FPoint fp) {
        initialPos.set(fp);
    }

    /** Set the X coordinate only of the initial position. */
    public void setX0(double x0) {
        initialPos.x = x0;
    }

    /** Set the Y coordinate only of the initial position. */
    public void setY0(double y0) {
        initialPos.y = y0;
    }

    /**
     * Set the initial facing angle of the robot.  The angle is expressed
     * in degrees, measured counterclockwise fro East (i.e., 0&deg;
     * is facing right, 90&deg; is straight up, etc.).
     */
    public void setTheta0(double degrees) {
        // Convert from degrees to radians
        initialPos.theta = degrees * Math.PI / 180.0;

        // Store the sin and cos of the angle, for use in later calculations
        sinTheta0 = Math.sin(initialPos.theta);
        cosTheta0 = Math.cos(initialPos.theta);
    }

    /**
     * Set the width of the robot's body.
     * <P>See the note at the top of this file for information about
     * measurement units.</P>
     */
    public void setBodyWidth(double width) {
        bodyWidth = width;
    }

    /**
     * Return the width of the robot's body.
     * <P>See the note at the top of this file for information about
     * measurement units.</P>
     */
    public double getBodyWidth() {
        return bodyWidth;
    }

    /**
     * Set the velocity of the left drive wheel.
     * <P>See the note at the top of this file for information about
     * measurement units.</P>
     */
    public void setVelocityLeft(double velocity) {
        velocityLeft = velocity;
    }

    /**
     * Set the velocity of the right drive wheel.
     * <P>See the note at the top of this file for information
     * about measurement units.</P>
     */
    public void setVelocityRight(double velocity) {
        velocityRight = velocity;
    }

    /**
     * Return the velocity of the left drive wheel.
     * runData[0][numSteps] = err;
     * runData[1][numSteps] = left;
     * runData[2][numSteps] = right;
     * runData[3][numSteps] = dterm;
     * runData[4][numSteps] = iterm;
     * runData[5][numSteps] = pid;
     */
    public double getVelocityLeft(double t) {
        //return velocityLeft+t*accelerationLeft;
    	return MotionApp.runData[1][(int) t];
    }

    /**
     * Return the velocity of the right drive wheel.
     * <P>See the note at the top of this file for information
     * about measurement units.</P>
     */
    public double getVelocityRight(double t) {
        //return velocityRight+t*accelerationRight;
    	return MotionApp.runData[2][(int) t];
    }

    /**
     * Set the acceleration of the left drive wheel.
     */

    public void setAccelerationLeft(double acceleration) {
            accelerationLeft = acceleration;
    }

    /**
     * Set the acceleration of the right drive wheel.
     */
    public void setAccelerationRight(double acceleration) {
            accelerationRight = acceleration;
    }

    /**
     * Return the acceleration of the left drive wheel.
     */
    public double getAccelerationLeft() {
            return accelerationLeft;
    }

    /**
     * Return the acceleration of the right drive wheel.
     */
    public double getAccelerationRight() {
            return accelerationRight;
    }

    public void setMaximumAllowableError(double maxAllowableError){
            this.maxAllowableError=maxAllowableError;
    }

    public double getMaximumAllowableError(){
            return maxAllowableError;
    }

    public double getAbsoluteDeltaTheta(double t){
      /**
       *  deltaTheta lets us determine the total change in the
       *  system's orientation over a period of time from 0 to t.
       *  orientation is computed using a second degree polynomial
       *  for theta(t) = C*t^2 + D*t + theta0.
       *  If theta(t) is monotonically increasing or decreasing,
       *  then the total delta t is just a matter of evaluating
       *  it at f(t) and f(0) and taking the absolute value of
       *  the differences.  But suppose the robot starts out turning
       *  to the right, but over time (due to acceleration of the slower
       *  right wheel) ends up turning to the left.   This function
       *  is concerned about the total of the absolute delta thetas
       *  over both parts of the turn.
       */

      double C = (accelerationRight - accelerationLeft) /(2 * bodyWidth);
      double D = (velocityRight - velocityLeft) / bodyWidth;

      double ft = C*t*t+D*t;

      if(Math.abs(C)<1.0e-6){
         // the function theta(t) is essentially a straight line,
         // so there is no inflexion point and function is monotonically
         // increasing or decreasing, we simply return |f(t)-f(0)|
         return Math.abs(ft);
      }

      double x=D/(2*C);  // there is an inflection-point/extrema at t=x
      if(x<0 || x>t){
         // the inflection point is outside the range [0, t]
         // so again, we ignore it
         return Math.abs(ft);
      }

      // the is an inflection point in the interval, so we return
      // | f(t)-f(x) |  +  | f(x)-f(0) |
      double fx = C*x*x+D*x;
      return Math.abs(ft-fx) + Math.abs(fx);
   }




     /*  To use Simpson's Rule for approximating the value of an
      *  integral of f(x) over the interval [0, t], we need to
      *  select enough sub-intervals so that the absolute error
      *  is smaller than some arbitrary bound.   Simpson's Rule
      *  provides a technique for estimating the upper limit of the
      *  magnitude of the error based on the 4th derivative of f(x)
      *
      *     If |f4(x)| <= M for all x in interval a <= x <= b,
      *     and h = (b-a)/n  then
      *
      *          Error <= M * h^4*(b-a)/180
      *
      *      where n is the number of subintervals.
      *
      *  For our position functions x(t)=(Ax+B)cos(Cx^2 +Dx +E), and
      *  y(t)=(Ax+B)sin(Cx^2+Dx+E), the 4th derivatives get a little involved,
      *  but fortunately we can take advantage of certain simplifications.
      *  The 4th derivative of of the function f(x) = (Ax+B)cos(Cx^2 + Dx + E) is
      *
      *      f4(x) = 4Asin(Cx^2 + Dx + E)(2Cx + D)^3
      *            - 24Acos(Cx^2 + Dx + E) (2Cx+ D)C
      *            + (Ax+b)cos(Cx^2 + Dx + E) (2Cx + D) ^4
      *            + 12(Ax+B)sin(Cx^2 + Dx + E) (2Cx+D)^2 * C
      *            - 12(Ax+B)cos(Cx^2 + Dx + E)C^2
      *
      *  Because  |sin(x)| <= 1, |cos(x)|<=1, we know that the
      *  maximum contribution of the absolute value of the trig functions,
      *  no matter what the value of x, will be 1.  So we use that value,
      *  replacing all trig functions with 1.   Now in the expression
      *  (Ax+B)*sin(x), Ax+B may be less than zero.  But at the same
      *  time sin(x) may also be negative, giving us a positive
      *  value for (Ax+B)*sin(x).   Since we are interested in the
      *  absolute, worst case, maximum value of the 4th derivative over
      *  the interval, we take assume that each term makes a positive contribution
      *  the f4(x) but taking absolute values.
      *
      *       M =  4|A(2Cx + D)^3| + 24|A(2Cx+ D)C|
      *         +   |(Ax+b)(2Cx + D) ^4|  + 12|(Ax+B)(2Cx+D)^2 * C|
      *         + 12|(Ax+B)C^2|
      *
      *  The same rule applies for g(x) = (Ax+B)sin(Cx^2 + Dx + E)
      */

      public int getSimpsonIntervals(double A, double B, double C, double D, double x){

           //precompute some values ..
           double term1   = A*x + B;
           double term2   = 2*C *x + D;
           double term2P2 = term2*term2;
           double term2P3 = term2P2 * term2;
           double term2P4 = term2P2 * term2P2;
           double xP5     = Math.pow(x, 5);

           double Mcos =   Math.abs(4*A * term2P3)
                         + Math.abs(24 * A*C*term2)
                         + Math.abs(term1 * term2P4)
                         + Math.abs(12 * term1 * term2P2 * C)
                         + Math.abs(12 * term1 * C *C );

           double Msin =   Math.abs(4 * term2P3)
                         + Math.abs(24 * A * term2 * C)
                         + Math.abs(term1 * term2P4)
                         + Math.abs(12 * term1 * term2P2 * C)
                         + Math.abs(12 * term1 * C * C);

           double ncos = Math.abs(Mcos * xP5 /(maxAllowableError*180));
           double nsin = Math.abs(Msin * xP5 /(maxAllowableError*180));

           double n = ncos;

           if (nsin > ncos) {
                   n = nsin;
           }

           n = Math.pow(n, 0.25);

           int N =(int) Math.ceil(n);

           if ((N%2) == 1) {
                   N += 1;
           }

           if (N < 4) {
                   N = 4;
           }

           return N;
   }

    public int getSimpsonIntervals(double x){
           double A = (accelerationLeft + accelerationRight) /2;
           double B = (velocityLeft + velocityRight) /2;
           double C = (accelerationRight - accelerationLeft) /(2 * bodyWidth);
           double D = (velocityRight - velocityLeft) / bodyWidth;
           return getSimpsonIntervals(A, B, C, D, x);
    }



    /**
     * Calculate the position of the robot at a given point in time.
     * Acceleration is also considered in the calculations.
     * <P>See the note at the top of this file for information about
     * measurement units.</P>
     */

    public Position positionAt(double t) {

          /**
           *  Contributed by Jing YE, Univeristy of Melbourne.
           *
           *  When acceleration is zero (wheel speeds are constant),
           *  there is a closed-form solution for both orientation and
           *  position as a function of time.  But when speeds change,
           *  there is no close-form solution for position.
           *  All we know are the derivatives x'(t), y'(t),
           *  So we need a numerical method to integrate them.
           *  In this case, ee use Simpon's Rule from elementary calculus.
           */


           if (t == 0) {
                 cachedTime = 0;
           }
           /*
           double A = (accelerationLeft + accelerationRight) /2;
           double B = (velocityLeft + velocityRight) /2;
           double C = (accelerationRight - accelerationLeft) /(2 * bodyWidth);
           double D = (velocityRight - velocityLeft) / bodyWidth;

           double theta0 = initialPos.theta;

           double theta = C*t*t + D*t + theta0;

           double start = 0;
           double end = t;

           double finalX = 0;
           double finalY = 0;


           //Start of Simpson's rule...
           int simpsonIntervals = getSimpsonIntervals(A, B, C, D, t);
           double deltaT = t / simpsonIntervals;

           //f(x0)
           finalX += (A*start + B)*(Math.cos(C*start*start + D*start + theta0));
           finalY += (A*start + B)*(Math.sin(C*start*start + D*start + theta0));

           //4*f(x1) + 2*f(x2) + ... + 4*f(xn-1)
           for (int i = 1; i < simpsonIntervals; i++) {
                   start += deltaT;
                   if ((i%2) == 1) {
                           finalX += 4 * (A*start + B) *
                                   (Math.cos(C*start*start + D*start + theta0));
                           finalY += 4 * (A*start + B) *
                                   (Math.sin(C*start*start + D*start + theta0));
                   } else {
                           finalX += 2 * (A*start + B) *
                                   (Math.cos(C*start*start + D*start + theta0));
                           finalY += 2 * (A*start + B) *
                                   (Math.sin(C*start*start + D*start + theta0));
                   }
           }

           //f(xn)
           finalX += (A*end + B) * (Math.cos(C*end*end + D*end + theta0));
           finalY += (A*end + B) * (Math.sin(C*end*end + D*end + theta0));

           finalX = deltaT*finalX/3.0d;
           finalY = deltaT*finalY/3.0d;

           //End of simpson's rule...
	*/
           cachedTime = t;

           FPoint location = new FPoint();

           location.x = MotionApp.centerPoints[(int) t].x;//finalX;
           location.y = MotionApp.centerPoints[(int) t].y;//finalY;

           location = location.add(initialPos);

           return new Position(location.x, location.y, MotionApp.plotData[(int) t].theta);
    }

    /**
     * Use dead-reckoning, as described in Gary Lucas' paper, to
     * estimate where the robot will be after time t.
     
    public Position DeadReckonAt(double t) {
        
         // First, calculate the distance traveled by each wheel:
        double distanceLeft = velocityLeft * t;
        double distanceRight = velocityRight * t;

        //
        // Now, calculate the final angle, and use that to estimate
        // the final position.  See Gary Lucas' paper for derivations
        // of the equations.
        ///
        double theta = initialPos.theta + (distanceRight - distanceLeft)
            / bodyWidth;
        double x = initialPos.x + (distanceRight + distanceLeft)
            / 2 * Math.cos(theta);
        double y = initialPos.y + (distanceRight + distanceLeft)
            / 2 * Math.sin(theta);

        return new Position(x, y, theta);
    }
	*/

    /**
     * Given the position of the robot (i.e., location and direction),
     * return the location of the left wheel.  This is used for
     * drawing the robot, or its path.
     */
    public static FPoint LeftWheelLoc(Position p) {
        FPoint result = new FPoint();

        result.x = p.x + bodyWidth/2 * Math.cos(p.theta + Math.PI/2);
        result.y = p.y + bodyWidth/2 * Math.sin(p.theta + Math.PI/2);

        return result;
    }

    /**
     * Given the position of the robot (i.e., location and direction),
     * return the location of the right wheel.  This is used for
     * drawing the robot, or its path.
     */
    public static FPoint RightWheelLoc(Position p) {
        FPoint result = new FPoint();

        result.x = p.x + bodyWidth/2 * Math.cos(p.theta - Math.PI/2);
        result.y = p.y + bodyWidth/2 * Math.sin(p.theta - Math.PI/2);

        return result;
    }

    /**
     * Given the position of the robot (i.e., location and direction),
     * return the location of its nose (i.e., the "front".  This is used
     * for drawing the robot, or its path.
     */
    public static FPoint NoseLoc(Position p) {
        FPoint result = new FPoint();

        /*
         * "bodyWidth/2" is the distance from the centerpoint of the drive
         * wheels, to the "nose".  This can be changed if you want the
         * triangle representing the robot to have a different shape,
         * without affecting the rest of the code.
         */
        result.x = p.x + bodyWidth/2 * Math.cos(p.theta);
        result.y = p.y + bodyWidth/2 * Math.sin(p.theta);

        return result;
    }
}
