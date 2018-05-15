/**
 * FPoint represents a point on the floating-point Cartesian plane
 * (i.e., it is an (x, y) pair, with x and y represented as doubles.
 *
 * @author <a href="mailto:MikeGauland@users.sourceforge.net">Michael
 *     Gauland</a>
 * @version 1.
 */
package com.neocoretechs.robocore.test.MotionViewer;
public class FPoint {
    /** The horizontal coordinate. */
    public double x;

    /** The vertical coordinate. */
    public double y;

    /** Default constructor; both coordinates set to Java default of 0.0. */
    FPoint() {
    }

    /** Constructor, taking a double for each coordinate. */
    FPoint(double x, double y) {
        set(x, y);
    }

    /*
     * Copy another FPoint object.
     */
    public void set(FPoint fp) {
        this.x = fp.x;
        this.y = fp.y;
    }

    /*
     * Set both coordinates, specified as doubles.
     */
    public void set(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Add the same value to both coordinates.
     */
    FPoint add(double k) {
        return new FPoint( this.x + k, this.y + k);
    }

    /**
     * Sub one FPoint to another.  This can be used to
     * sub separate offsets to each coordinate.
     */
    FPoint sub(FPoint fp) {
        return new FPoint(this.x - fp.x, this.y - fp.y);
    }
   
    /**
     * Sub the same value to both coordinates.
     */
    FPoint sub(double k) {
        return new FPoint( this.x - k, this.y - k);
    }

    /**
     * Add one FPoint to another.  This can be used to
     * add separate offsets to each coordinate.
     */
    FPoint add(FPoint fp) {
        return new FPoint(this.x + fp.x, this.y + fp.y);
    }
    /**
     * Multiply both coordinate by the same value.
     */
    FPoint scale(double k) {
        return new FPoint(this.x * k, this.y * k);
    }

    /**
     * Multiple each coordinate by different values.
     */
    FPoint scale(FPoint fp) {
        return new FPoint(this.x * fp.x, this.y * fp.y);
    }
   
    /**
     * Returns the coordinates as a string, for printing or display.
     */
    public String toString() {
        return new String("(" + x + ", " + y + ")");
    }
}
