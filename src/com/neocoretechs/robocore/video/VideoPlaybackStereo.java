package com.neocoretechs.robocore.video;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.image.BufferedImage;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import java.net.Socket;
import java.nio.ByteBuffer;
import java.text.DateFormat;
import java.text.DecimalFormat;
import java.util.Date;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.stream.Stream;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;

//import com.neocoretechs.machinevision.CannyEdgeDetector;
import com.neocoretechs.relatrix.client.asynch.AsynchRelatrixClientTransaction;
import com.neocoretechs.relatrix.client.RemoteStream;
import com.neocoretechs.rocksack.TransactionId;

//import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;

/**
 * Create a panel and retrieve video images from the image database, then
 * display them to the panel OR Start an MPEG streaming server and await connections OR write out a series of image files.
 * The function depends on remapped command line param "__mode" either "display" or directory name
 * Demonstrates how we can retrieve images from the data store then pump it to a display.
 * @author Jonathan Groff (C) NeoCoreTechs 2021
 *
 */
public class VideoPlaybackStereo  {
	private static boolean DEBUG = false;
	private static final boolean SAMPLERATE = true; // display pubs per second
	public static AsynchRelatrixClientTransaction rkvc;
	public static TransactionId xid;
    private static BufferedImage imagel = null;
    private static BufferedImage imager = null;
    private static PlayerFrame displayPanel1;
    private static PlayerFrame displayPanel2;
    private static Object mutex = new Object();

    private static ByteBuffer cbl;
    private static byte[] bufferl = new byte[0];
    private static ByteBuffer cbr;
    static byte[] bufferr = new byte[0];
    
    double eulers[] = new double[]{0.0,0.0,0.0};
   
	private static String mode = "display";
	private static String outDir = "/";
	private static int frames = 0;
	private static int[] ibuf = null;
    private static JFrame frame = null;
    //CircularBlockingDeque<java.awt.Image> queue = new CircularBlockingDeque<java.awt.Image>(30);
    //CircularBlockingDeque<byte[]> bqueue = new CircularBlockingDeque<byte[]>(30);
    private static byte[][] bqueue;
	private static int sequenceNumber,lastSequenceNumber;
	static long time1;
	static long timdiff = 0;
	static long timlast = 0;
	static long realtimdiff = 0;
	static long realtimlast = 0;
	static long samplesPer = 0;
	
	public static void main(String[] args) {
		try {
			if(args.length != 3 && args.length != 4) {
				System.out.println("usage: java com.neocoretechs.robocore.video.VideoPlaybackStereo [local node] [remote node] [server port] <previous hours to display>");
				System.exit(1);
			}
			rkvc = new AsynchRelatrixClientTransaction(args[0], args[1], Integer.parseInt(args[2]));
			xid = rkvc.getTransactionId();
		} catch (IOException e2) {
			throw new RuntimeException(e2);
		}
		if( mode.equals("display")) {
			//if( DEBUG )
				System.out.println("Invoking thread for pumping frames to AWT Panel");
			
			SwingUtilities.invokeLater(new Runnable() {
			    public void run() {
					JFrame frame = new JFrame("Player");
			        displayPanel1 = new PlayerFrame();
			        frame.setLayout(new BorderLayout());
					frame.add(displayPanel1, BorderLayout.EAST);
					displayPanel1.setPreferredSize(new Dimension(640,480));
					displayPanel1.setVisible(true);
			        displayPanel2 = new PlayerFrame();
					frame.add(displayPanel2, BorderLayout.WEST);
					displayPanel2.setPreferredSize(new Dimension(640,480));
					displayPanel2.setVisible(true);
					frame.pack();
					frame.setSize(new Dimension(1320, 520));
					frame.setVisible(true);
			        displayPanel1.paintPanel();
			        displayPanel2.paintPanel();
			    }
			});
			
			while(displayPanel1 == null || displayPanel2 == null /*||
					displayPanel2.aField == null || displayPanel2.yField == null || displayPanel2.xField == null ||
					displayPanel1.aField == null || displayPanel1.yField == null || displayPanel1.xField == null*/)
				try {
					Thread.sleep(1);
				} catch (InterruptedException e) {}
		//}
		
		try {
			CompletableFuture<Stream> sstream;
		    Stream stream = null;
		    if(args.length == 4) {
		    	long ptimh = Long.parseLong(args[3]);
		    	long ptim = System.currentTimeMillis() - (ptimh*3600000L);
		    	CompletableFuture<Object> r = rkvc.lastKey(xid,Long.class);
		    	Long lastTim = (Long) r.get();
		    	//long ptim = (ptimh*3600000L);
		    	//Long firstTim = (Long)rkvc.firstKey(Long.class);
		    	System.out.println("From:"+new Date(ptim)+" To:"+new Date(lastTim));
		    	sstream = rkvc.findSubStream(xid, '*', Integer.class, '?',ptim,lastTim);
		    } else {
		    	sstream =  rkvc.findStream(xid,'?', '?', '?');
		    }
		    try {
				stream = sstream.get();
			} catch (InterruptedException | ExecutionException e2) {
				e2.printStackTrace();
			}
		    //stream = (Stream<Comparable[]>) Relatrix.findStream("?", "?", "?", true);
			//Map<Object, Map<Object, Map<Object, Long>>> nameCount = stream.collect(Collectors.groupingBy(b -> b[0].toString(),
		    //		Collectors.groupingBy(d -> d[1].toString(), 
		    //		Collectors.groupingBy(e -> e[2].toString(), Collectors.counting()))));
	        //nameCount.forEach((name, count) -> {
	        //    System.out.println(name + ":" + count);
	        //});	        
		    //stream = (Stream<Comparable[]>) Relatrix.findStream("?", "?", "?");
		    //stream.flatMap(e -> Stream.of(e))
		    //	.forEach((l,m,n) -> System.out.println("Element B:"+l));
		    	//.forEach(g -> System.out.println("Element B:"+g));
			//Iterator<?> it = Relatrix.findSet("?", "?", "?");
			//RemoteStream it = rkvc.entrySetStream(java.lang.Long.class);
			stream.forEach(e -> {
				// try to push frames in realtime, capped at 1 second max wait since we detect motion
				// if the recorded time between the frames is longer than the realtime display, slow
				// the display down to match the difference in recorded frame time to the difference in real time (capped at 1 sec).
				// We want to avoid playing faster than realtime.
					Long tim = (long)((Comparable[]) e)[0];
					if(timdiff == 0) {
						timdiff = tim;
						realtimdiff = System.currentTimeMillis();
					} else {
						timdiff = tim-timlast;
						realtimdiff = System.currentTimeMillis() - realtimlast;
						if(timdiff > realtimdiff) {
							try {
								// cap it at 1 sec
								if((timdiff - realtimdiff) > 1000)
									Thread.sleep(1000);
								else
									Thread.sleep(timdiff - realtimdiff);
							} catch (InterruptedException e1) {
							}
						}
					}
					//if(DEBUG)
						System.out.println("timdiff="+timdiff+" relatimdiff="+realtimdiff);
					timlast = tim;
					realtimlast = System.currentTimeMillis();
					//Entry e = (Entry) rkvc.next(it);
					StereoscopicImageBytes sib = (StereoscopicImageBytes) ((Comparable[]) e)[2];
					synchronized(mutex) {
						bufferl = sib.getLeft(); // 3 byte BGR
						bufferr = sib.getRight(); // 3 byte BGR	
						if( mode.equals("display")) {
							try {
								InputStream in = new ByteArrayInputStream(bufferl);
								imagel = ImageIO.read(in);
								in.close();
								/*
								CannyEdgeDetector detect = new CannyEdgeDetector();
								detect.setLowThreshold(.75f);
								detect.setHighThreshold(1f);
								detect.setGaussianKernelWidth(32);
								detect.setGaussianKernelRadius(3);
								detect.setSourceImage(imagel);
								detect.process();
								imagel = detect.getEdgesImage();
								*/
							} catch (IOException e1) {
								System.out.println("Could not convert LEFT image payload due to:"+e1.getMessage());
								return;
							}
							try {
								InputStream in = new ByteArrayInputStream(bufferr);
								imager = ImageIO.read(in);
								in.close();
							} catch (IOException e1) {
								System.out.println("Could not convert RIGHT image payload due to:"+e1.getMessage());
								return;
							}
						} else {
							if( mode.equals("display_server")) {
								bqueue = new byte[][]{bufferl, bufferr};
							} 
						}

	        			displayPanel1.setLastFrame((java.awt.Image)imagel);
	        			displayPanel2.setLastFrame((java.awt.Image)imager);
	    				displayPanel2.setComputedValues(String.valueOf(((Comparable[]) e)[1]), (long)((Comparable[]) e)[0],(double) 0);
	        			displayPanel1.setComputedValues(String.valueOf(((Comparable[]) e)[1]), (long)((Comparable[]) e)[0], (double)0); // values from db for time, yaw
	        			//displayPanel.lastFrame = displayPanel.createImage(new MemoryImageSource(newImage.imageWidth
	        			//		, newImage.imageHeight, buffer, 0, newImage.imageWidth));
	        			displayPanel1.invalidate();
	        			displayPanel2.invalidate();
	        			displayPanel1.updateUI();
	        			displayPanel2.updateUI();
					}
					++sequenceNumber; // we want to inc seq regardless to see how many we drop	
				//}
		});
		System.out.println("End of retrieval. sequence="+sequenceNumber);
		} catch(IllegalArgumentException | InterruptedException | ExecutionException iae) {
			iae.printStackTrace();
			return;
		}
		
		} // run
	
		
	} // main

	/**
	 * Pump the HTTP image payload to the socket connected client, most likely a browser.
	 * Designed to be spun up in its own thread.
	 */
	class StandardImageConnection implements Runnable {
		private static final String BOUNDARY = "reposition";

		private Socket client;
		//private DataInputStream in;
		//private DataOutputStream out;
		private OutputStream out;

		public StandardImageConnection(Socket client_socket)
		{
			client = client_socket;
			try {
				//in = new DataInputStream(client.getInputStream());
				//out = new DataOutputStream(new BufferedOutputStream(client.getOutputStream()));
				out = client.getOutputStream();
			} catch (IOException ie) {
				System.err.println("Unable to start new connection: " + ie);
				try {
					client.close();
				} catch (IOException ie2) {
				}
				return;
			}

		}

	    public void run()
		{
			try {
				out.write("HTTP/1.0 200 OK\r\n".getBytes());
				out.write("Server: RoboCore Image server\r\n".getBytes());
				out.write(("Content-Type: multipart/x-mixed-replace;boundary=" + BOUNDARY + "\r\n").getBytes());
				out.write("\r\n".getBytes());
				out.write(("--" + BOUNDARY + "\n").getBytes());
				while (true) {
						out.write("Content-type: image/jpeg\n\n".getBytes());
						byte[][] b = bqueue;//.takeFirst();
        				out.write(b[0], 0, b[0].length);
						out.write(("--" + BOUNDARY + "\n").getBytes());
						out.write("Content-type: image/jpeg\n\n".getBytes());
        				out.write(b[1], 0, b[1].length);
						out.write(("--" + BOUNDARY + "\n").getBytes());
						out.flush();
				}
			} catch (Exception ie) {
				try {
					out.close();
					client.close();
				} catch (IOException ie2) {
				}
			}
		}
	}
	
	static class PlayerFrame extends JPanel {
		JFrame frame;
		public PlayerFrame() {
			//frame = new JFrame("Player");
			//---
			//displayPanel = new PlayerFrame();
			//frame.getContentPane().add(displayPanel, BorderLayout.CENTER);
			//displayPanel.setVisible(true);
			//---
			//frame.add(this, BorderLayout.CENTER);
			// Remove window title and borders
	        //frame.setUndecorated(true);
	        // Make frame topmost
	        //frame.setAlwaysOnTop(true);
	        // Disable Alt+F4 on Windows
	        //frame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
	        // Make frame full-screen
	        //frame.setExtendedState(Frame.MAXIMIZED_BOTH);
	        // Display frame
			//this.setVisible(true);
			//frame.pack();
			//frame.setSize(new Dimension(640, 480));
			//frame.setVisible(true);
		}
		private java.awt.Image lastFrame;
		public void setLastFrame(java.awt.Image lf) {
			if( lf == null ) return;
			if( lastFrame == null ) {
				lastFrame = lf;
				return;
			}
			synchronized(lastFrame) {
				lastFrame = lf; 
			} 
		}

		public void paintComponent(Graphics g) {
			super.paintComponent(g);
			if( lastFrame == null )
				return;
			synchronized(lastFrame) {
				g.drawImage(lastFrame, 0, 0, lastFrame.getWidth(this), lastFrame.getHeight(this), this);
				lastFrame.flush();
			}
		}
		
		public void paintPanel(){
		   /*
		    * Use a "Grid Bag Layout" for this panel.  It will be laid
		    * out like this:
		    * Yaw:<actual Yaw> Roll:<actual roll> Pitch:<actual pitch>
		    */
		        GridBagLayout      gbl      = new GridBagLayout();
		        GridBagConstraints gbc = new GridBagConstraints();
		        setLayout(gbl);
		        gbc.anchor=gbc.LAST_LINE_START;


		   /*
		    * Set up the "Yaw" label
		    */
		        gbc.fill=gbc.HORIZONTAL;
		        JLabel xJLabel = new JLabel("Yaw:");
		        gbc.gridx=1;
		        gbc.gridy=1;
		        gbc.weightx=0;
		        gbc.weighty=1;
		        gbl.setConstraints(xJLabel, gbc);
		        add(xJLabel);

		   /*
		    * Set up the "Time" label
		    */
		        JLabel yJLabel = new JLabel("Time:");
		        gbc.fill=gbc.HORIZONTAL;
		        gbc.gridx=2;
		        gbc.gridy=1;
		        gbc.weightx=0;
		        gbc.weighty=1;
		        gbl.setConstraints(yJLabel, gbc);
		        add(yJLabel);

		   /*
		    * Set up the "Pitch" label
		    */
		        JLabel aJLabel = new JLabel("Pitch:");
		        gbc.fill=gbc.HORIZONTAL;
		        gbc.gridx=3;
		        gbc.gridy=1;
		        gbc.weightx=0;
		        gbc.weighty=1;
		        gbl.setConstraints(aJLabel, gbc);
		        add(aJLabel);

		   
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
		        yField = new JTextField(18);
		        yField.setEditable(false);
		        yField.setHorizontalAlignment(JTextField.RIGHT);
		        yField.setBorder(null);
		        gbc.fill=gbc.NONE;
		        gbc.gridx=2;
		        gbc.gridy=1;
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
		        gbc.gridx=3;
		        gbc.gridy=1;
		        gbc.weightx=1;
		        gbc.weighty=1;
		        gbl.setConstraints(aField, gbc);
		        add(aField);


		   /*
		    * Create formatters for the coordinates and time (2 decimal places)
		    * and the angles (1 decimal place, and the degrees symbol).
		    */
		        dt = DateFormat.getTimeInstance();
		        df = DateFormat.getDateInstance();
		        dfs = new DecimalFormat("+####0.00;-####0.00");
		        da = new DecimalFormat("####0.0\u00b0");
		        das = new DecimalFormat("+####0.0\u00b0;-####0.0\u00b0");

		   /*
		    * Initialize all displayed values to 0, except the angles,
		    * which are set to 90 degrees.
		    */
		        setComputedValues(String.valueOf(0.0), 0, 0.0);
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
		    void setComputedValues(String string, long time, double pitch)
		    {
		        xField.setText(/*da.format(*/string/*)*/);
		        yField.setText(dt.format(time)+" "+df.format(time));
		        aField.setText(da.format(pitch));

		    }

		    /** Formatter for the time. */
		    DateFormat dt,df;

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

		}
	
}

