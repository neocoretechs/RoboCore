/**
 * DrawInterface allows a class to be installed into a FloatCanvas,
 * so that the object will be asked to draw itself whenever the canvas
 * is redrawn.
 *
 * @author <a href="mailto:MikeGauland@users.sourceforge.net">Michael
 *     Gauland</a>
 * @version 1.
 * @see FloatCanvas
 */
package com.neocoretechs.robocore.test.MotionViewer;
public interface DrawInterface {
    /** The implementation of drawFunc() should draw the object.     */
    void drawFunc(FloatCanvas fc);
}

