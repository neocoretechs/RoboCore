package com.neocoretechs.robocore.test.MotionViewer;
import java.awt.Canvas;
import java.awt.event.*;
import java.awt.Graphics;
import java.awt.*;

/**
 * FloatCanvas implements a floating-point cartesian plane.
 *
 * @author <a href="mailto:MikeGauland@users.sourceforge.net">
 * Michael Gauland</a>
 * @version 1.
 */
public class FloatCanvas extends Canvas implements ComponentListener {
    /**
     * All FloatCanvas objects are displayed using this font,
     * which is initialized to 8-point "Monospaced".
     *
     * <P><FONT SIZE=-1><I>(This could be a class variable, since all
     * FloatCanvas objects use the same font, but then it doesn't seem to
     * show up in the generated documents.)</I></FONT></P>
     */
    Font labelFont = new Font("Monospaced", Font.PLAIN, 8);

    /**
     * Metrics for the label font are stored here, but it cannot
     * be obtained until an object is instantiated, since we need
     * a graphics context to access the "getFontMetrics()" function.
     *
     * <P><FONT SIZE=-1><I>(This could be a class variable, since all
     * FloatCanvas objects use the same font, but then it doesn't seem to
     * show up in the generated documents.)</I></FONT></P>
     */
    FontMetrics fontMetric;

    /**
     * We'll grab the font height once, since we need it often, and it
     * won't change.
     *
     * <P><FONT SIZE=-1><I>(This could be a class variable, since all
     * FloatCanvas objects use the same font, but then it doesn't seem to
     * show up in the generated documents.)</I></FONT></P>
     */
    int fontHeight;

    /**
     * This will be set to the area used for the plot&mdash;i.e., the
     * graphics area, minus the border area used for the axis labels.
     * It is used to set the clipping region while drawing or plotting
     * to the graph.
     */
    Rectangle plotRect = new Rectangle();

    /**
     * For smoother updates, data is plotted to this off-screen image,
     * then displayed all at once.
     */
    Image imageBuffer = null;

    /**
     * This variable stores one object that can be drawn in the canvas.
     * For MotionApp, that will be the MotionApp object, which will
     * draw the robot's paths in the canvas.  This could be changed to an
     * array or collection, to allow mutliple objects to be drawn onto the
     * same canvas.
     *
     * <P>The drawFunc() function for this object is called whenever the
     * FloatCanvas is drawn or refreshed.
     */
    DrawInterface theInterface;

    /** The minimum X value in the plot area */
    double xMin = -1;

    /** The maximum X value in the plot area */
    double xMax = +1;

    /** The minimum Y value in the plot area */
    double yMin = -1;

    /** The maximum Y value in the plot area */
    double yMax = +1;

    /**
     * The scaling to apply to convert from floating-point coordinates
     * to screen pixels.  It will be recalculated when the window is
     * resized, or any of xMin, xMax, yMin, or yMax change.
     */
    double scaleFactor = 1;

    /**
     * Space between the top of the plot area and the top of the
     * graphics context.
     */
    static final int topMargin = 1;

    /**
     * Space between the bottom of the plot area, and the bottom of the
     * graphics context.  This must leave room for the X-axis labels.
     */
    static final int bottomMargin = 20;

    /**
     * Space between the left edge of the plot area, and the left edge of
     * the graphics context.  This must leave room for the Y-axis labels.
     */
    static final int leftMargin = 30;

    /**
     * Space between the right edge of the plot area, and the right edge of
     * the graphics context.
     */
    static final int rightMargin = 1;

    /**
     * The length of the tic marks identifying the limits of the plot area.
     */
    static final int ticLength = 2;

    /** Default background color for a FloatCanvas object. */
    static final Color defaultBackground = Color.white;

    /** Default foreground color for a FloatCanvas object. */
    static final Color defaultForeground = Color.blue;

    /** Default color for scales. */
    static final Color defaultScaleColor = Color.red;

    /**
     * Default constructor--set background and foreground colors to
     * defaults, and add the object as a "ComponentListener" of its
     * superclass.
     */
    FloatCanvas() {
        super();
        setBackground(defaultBackground);
        setForeground(defaultForeground);
        super.addComponentListener(this);
    }

    /**
     * getGraphics() overrides the superclass getGraphics function.  When
     * plotting the clients drawFunc() function, we draw the image off-screen,
     * then display the buffered image all at once, to reduce flicker.
    */
    public Graphics getGraphics() {
   /*
    * If we're drawing off-screen, the imageBuffer member variable
    * will reference the off-screen graphics context we are drawing to.
    * If it is null, then we are not drawing off-screen, and should
    * use the superclass's getGraphics() function instead.
    */
        if (imageBuffer != null) {
            return imageBuffer.getGraphics();
        } else {
            return super.getGraphics();
        }
    }

    /**
     * getPlotGraphics() retrieves the graphics conext through getGraphics(),
     * then sets the clipping area so that anything drawn outside the plot
     * area (delimited by the floating-point values xMin, xMax, yMin, and yMax)
     * will be clipped.
     */
    public Graphics getPlotGraphics() {
        Graphics g = this.getGraphics();
        g.setClip(plotRect);
        return g;
    }

    /**
     * Required for the ComponentListener interface, but doesn't do
     * anything.
     */
    public void componentResized(ComponentEvent e) {
    }

    /**
     * Required for the ComponentListener interface, but doesn't do
     * anything.
     */
    public void componentMoved(ComponentEvent e) {
    }

    /**
     * Required for the ComponentListener interface, but doesn't do
     * anything.
     */
    public void componentHidden(ComponentEvent e) {
    }

    /**
     * Required for the ComponentListener interface, but doesn't do
     * anything.
     */
    public void componentShown(ComponentEvent e) {
    }

    /**
     * We need to grab the font metrics, but even though the font won't
     * change, we can't grab it until we have a graphics context from which
     * to call getFontMetrics().  Therefore, we can't call it from the
     * FloatCanvas constructor; instead, we'll call this function the
     * first time we need to use the font information.  After that, the
     * member variables fontMetric and fontHeight will be non-null, and
     * this function will not be called a second time.
     */
    void grabFontInfo() {
   Graphics g = getGraphics();

        fontMetric = g.getFontMetrics(labelFont);
        fontHeight = fontMetric.getAscent();
    }

    /**
     * setLimits() sets the floating-point boundariesof the plot area,
     * rounded out to one decimal place (e.g., 1.00 would be treated as 1.00,
     * but 1.01 would become 1.1).
     */
    public void setLimits(double xMin, double xMax, double yMin, double yMax) {
   /*
    * The (yuck!) hard-coded "10.0"s round the limits "outwards"
    * to one decimal place.  This works for the MotionApp applet,
    * but should really be more flexible.
    */
        this.xMin = Math.floor(xMin * 10.0) / 10.0;
        this.xMax = Math.ceil(xMax * 10.0) / 10.0;
        this.yMin = Math.floor(yMin * 10.0) / 10.0;
        this.yMax = Math.ceil(yMax * 10.0) / 10.0;

   /*
    * After changing the limits of the plot area, we need to recalculate
    * the scale factor for converting from floating-point values to
    * display pixels.
    */
        setScaleFactor();
    }

    /**
     * setScaleFactor() calculates the scaling necessary to fit the full
     * X and Y ranges into the graphics context.  To avoid distorting the
     * image, we will calculate the scale factors independently, but use the
     * smaller for both axes.
     */
    void setScaleFactor() {
   /*
    * For each axis, we want to represent a range of [Min, Max], over
    * the allotted number of pixels.  Note that the number of pixels
    * is determined by the top and bottom (or left and right) margins,
    * plus the "magic number" 2, which allows for the plot area border.
    */
        double yScale = (getSize().height -
                         (bottomMargin + topMargin + 2)) / (yMax - yMin);

        double xScale = (getSize().width -
                         (leftMargin + rightMargin + 2)) / (xMax - xMin);

        scaleFactor = (xScale < yScale) ? xScale : yScale;
    }

    /**
     * Translate and scale the point, to convert it to a pixel location
     * in the graphics context.
     */
    Point scalePoint(FPoint fp) {
        return new Point( (int)Math.round((fp.x - xMin) * scaleFactor
                                          + leftMargin),
                          (int)Math.round(getSize().height - bottomMargin -
                                          (fp.y - yMin) * scaleFactor) );
    }

    /**
     * Plot the point in the current foreground color.
     */
    public void plot(FPoint fp) {
        plot(fp, getForeground());
    }

    /**
     * Plot the point in the specified color.
     */
    public void plot(FPoint fp, Color c) {
        /*
         * Get the graphics context, set to clip to the plot area.
         */
        Graphics g = getPlotGraphics();

        g.setColor(c);                   // Set the color
        Point p = scalePoint(fp);        // Convert to a pixel location

        /*
         * Draw the point as a tiny circle. When I tried to make it a
         * single pixel, it didn't show up on one system (Mac or Windows NT;
         * I don't remember which).
         */
        g.fillOval(p.x-1, p.y-1, 3, 3);
    }


    /**
     * This is similar to the "drawPolyline()" method of the Graphics
     * class, but uses floating-point coordinates.
     */
    public void drawPolyline(FPoint fp[], int numPoints, Color c) {
        int x[] = new int[numPoints];  // Translated X values
        int y[] = new int[numPoints];  // Translated Y values
        int i;                         // Loop index
        Point p = new Point();         // Temporary pixel coordinates

        /*
         * Convert each floating-point coordinate pair to
         * a pixel coordinate in the Graphics context.
         *
         * Note:  the "scaleFactor" member variable must have been
         * correctly set for this routine to work.  Currently, scaleFactor
         * is recalculated whenever the Graphics size, or any of xMin,
         * xMax, yMin, or yMax is changed.
         */
        for (i=0; i<numPoints; i++) {
            p = scalePoint(fp[i]);
            x[i] = p.x;
            y[i] = p.y;
        }

        /*
         * Get the graphics context, with the clipping region set to
         * the plot area; set the color; and draw the integer points.
         */
        Graphics g = getPlotGraphics();
        g.setColor(c);
        g.drawPolyline(x,y,numPoints);
    }

    /**
     * Draw a floating-point "polyline" in the current foreground color.
     */
    public void drawPolyline(FPoint fp[], int numPoints) {
        drawPolyline(fp, numPoints, getForeground());
    }


    /**
     * Provide the functionality of the Graphics class's "drawPolygon()"
     * method, using floating-point coordinates.
     */
    public void drawPolygon(FPoint fp[], Color c) {
        int numPoints = fp.length;       // The number of points provided
        int x[] = new int[numPoints];    // Translated X values
        int y[] = new int[numPoints];    // Translated Y values
        int i;                           // Loop index
        Point p = new Point();           // Temporary pixel coordinates

        /*
         * Translate all the points from floating-point coordinaates
         * to pixel coordinates.
         *
         * Note:  the "scaleFactor" member variable must have been
         * correctly set for this routine to work.  Currently, scaleFactor
         * is recalculated whenever the Graphics size, or any of xMin,
         * xMax, yMin, or yMax is changed.
         */
        for (i=0; i<numPoints; i++) {
            p = scalePoint(fp[i]);
            x[i] = p.x;
            y[i] = p.y;
        }

        /*
         * Get the graphics context, with the clipping region set to
         * the plot area; set the color; and draw the integer polygon.
         */
        Graphics g = getPlotGraphics();
        g.setColor(c);
        g.drawPolygon(x,y,numPoints);
    }

    /**
     * Draw a floating-point polygon, using the current foreground color.
     */
    public void drawPolygon(FPoint fp[]) {
        drawPolygon(fp, getForeground());
    }

    /**
     * This is similar to the "fillPolygon()" method of the Graphics
     * class, but uses floating-point coordinates.
     */
    public void fillPolygon(FPoint fp[], Color c) {
        int numPoints = fp.length;     // Number of points provided
        int x[] = new int[numPoints];  // Translated X values
        int y[] = new int[numPoints];  // Translated Y values
        int i;                         // Loop index
        Point p = new Point();         // Temporary pixel coordinates

        /*
         * Convert each floating-point coordinate pair to
         * a pixel coordinate in the Graphics context.
         *
         * Note:  the "scaleFactor" member variable must have been
         * correctly set for this routine to work.  Currently, scaleFactor
         * is recalculated whenever the Graphics size, or any of xMin,
         * xMax, yMin, or yMax is changed.
         */
        for (i=0; i<numPoints; i++) {
            p = scalePoint(fp[i]);
            x[i] = p.x;
            y[i] = p.y;
        }

        /*
         * Get the graphics context, with the clipping region set to the
         * plot area; set the color; and "fillPolygon()" the integer points.
         */
        Graphics g = getPlotGraphics();
        g.setColor(c);
        g.fillPolygon(x,y,numPoints);
    }

    /**
     * Draw a filled polygon using floating-point coordinates, and
     * the current foreground color.
     */
    public void fillPolygon(FPoint fp[]) {
        fillPolygon(fp, getForeground());
    }

    /**
     * Draw a line in a specified color, given a pair of floating-point
     * coordinates.
     */
    public void line(FPoint fp1, FPoint fp2, Color c) {
        Graphics g = getPlotGraphics();       // Get the Graphics context
        g.setColor(c);                        // Set the foreground color

        Point p1 = scalePoint(fp1);           // Convert the points to
        Point p2 = scalePoint(fp2);           // pixel coordinates.

        g.drawLine(p1.x, p1.y, p2.x, p2.y);   // Use the Graphics function.
    }


    /**
     * Draw a line, using floating-point coordinates, in the current
     * foreground color.
     */
    public void line(FPoint fp1, FPoint fp2) {
        line(fp1, fp2, getForeground());
    }

    /**
     * Draw the a label along the Y axis, marking the floating-point value.
     */
    void drawYLabel(double value) {
        /*
         * Get the pixel coordinate of the point on the Y axis
         */
        int ticLoc = scalePoint(new FPoint(0,value)).y;

        /*
         * Generate a label string from the floating point value.
         */
        String s = Double.toString(value);

        /*
         * The label is located lower than the tic mark, by half the
         * height of the string.
         */
        int labelLoc = ticLoc + fontHeight/2 +1;

        /*
         * If the label is at or near the top of the window, force
         * it down so that it will be visible.
         */
        int labelLimit = fontHeight + 1;
        if (labelLoc < labelLimit) labelLoc = labelLimit;

        /*
         * Get the graphics context, draw the tic mark, and draw
         * the label string.
         */
        Graphics g = getGraphics();
        g.drawLine(leftMargin - ticLength, ticLoc, leftMargin, ticLoc);
        g.drawString(s, leftMargin - fontMetric.stringWidth(s) -
                     (ticLength + 2),  // 2 pixels between string & line
                     labelLoc);
    }

    /**
     * Draw the a label along the X axis, marking the floating-point value.
     */
    void drawXLabel(double value) {
        /*
         * Get the pixel coordinate of the point on the X axis
         */
        int ticLoc = scalePoint(new FPoint(value,0)).x;

        /*
         * Generate a label string from the floating point value.
         */
        String s = Double.toString(value);

        /*
         * The label starts to the left of the tic mark, so it will be
         * centered on the tic.
         */
        int labelLoc = ticLoc - fontMetric.stringWidth(s)/2;

        /*
         * If the label is at or near the right of the window, force
         * it left so that it will be visible.
         */
        int labelLimit = getSize().width - rightMargin -
            fontMetric.stringWidth(s) - 3;
        if (labelLoc > labelLimit) labelLoc = labelLimit;

        /*
         * Get the graphics context, draw the tic mark, and draw
         * the label string.
         */
        Graphics g = getGraphics();
        g.drawLine(ticLoc, getSize().height - bottomMargin,
                   ticLoc, getSize().height - bottomMargin + ticLength);
        g.drawString(s, labelLoc, getSize().height - bottomMargin +
                     fontHeight + ticLength + 2); // 2 pixels btw tic & string
    }

    /**
     * Draw the X and Y axes, and label them.
     */
    public void drawScales() {
        Graphics g = getGraphics();

        /*
         * If this is the first time we are drawing the labels, grab
         * the font data.  This will only need to be done once.
         */
        if (fontMetric == null) {
            grabFontInfo();
        }

        /*
         * Along with the axes, we draw a line across the top and right
         * edges of the plot area.  It's not really necessary, but it
         * looks good, and is easy to do.
         */
        g.setColor(defaultScaleColor);
        g.drawRect(leftMargin, topMargin,
                   getSize().width - (leftMargin + rightMargin + 1),
                   getSize().height - (topMargin + bottomMargin + 1));

        /*
         * Draw the labels for the min and max points to be plotted.
         */
        drawYLabel(yMin);
        drawYLabel(yMax);
        drawXLabel(xMin);
        drawXLabel(xMax);
    }

    /**
     * Intall the callback function for drawing the contents of
     * the canvas.  As currently implemented, only the latest
     * function installed will be used (any previously-installed
     * functions will be lost).  This would be fairly straightforward
     * to change, should a future implementation need to allow more than
     * one object to draw to the canvas.
     */
    public void installPaintFunc(DrawInterface i) {
        theInterface = i;
    }

    /**
     * Painting consists of drawing the installed object (via the
     * callback to an off-screen buffer, adding the axes and labels,
     * then displaying the off-screen buffer.
     */
    public void paint(Graphics g) {
        // If there is no client function installed, do nothing:
        if (theInterface != null) {
            super.paint(g);  // Clear the canvas

            /*
             * Create an off-screen buffer, the same size as the
             * graphics context.
             */
            imageBuffer = createImage(getSize().width, getSize().height);

            /*
             * Draw a border around the graphics context.
             */
            plotRect.setBounds(leftMargin, 0,
                               getSize().width - leftMargin,
                               getSize().height - bottomMargin);

            /*
             * Invoke the client's draw function:
             */
            theInterface.drawFunc(this);

            drawScales();                       // Draw the scales & labels
            g.drawImage(imageBuffer,0,0,null);  // Display the buffer
            imageBuffer = null;                 // Release the buffer
        }
    }

    /**
     * To repaint the FloatCanvas, get the graphics context, and
     * pass it on to pain().
     */
    public void repaint() {
        paint(getGraphics());
    }
}

