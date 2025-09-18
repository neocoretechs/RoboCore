package com.neocoretechs.robocore.video;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.Graphics;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.image.BufferedImage;
import java.awt.image.WritableRaster;
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.text.DecimalFormat;
import java.util.Map;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;
import javax.swing.border.Border;
import javax.swing.border.TitledBorder;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.robocore.SynchronizedThreadManager;

//import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;


/**
 * Create a panel and receive published video images on the Ros bus from /sensor_mmsgs/Image, then
 * display them to the panel OR Start an MPEG streaming server and await connections OR write out a series of image files.
 * The function depends on remapped command line param "__mode" either "display" or directory name
 * Demonstrates how we can manipulate the image buffer to publish to ROS or create
 * a bufferedimage from that payload, then pump it to a display.
 * @author jg (C) NeoCoreTechs 2017
 *
 */
public class VideoListener extends AbstractNodeMain 
{
	private static boolean DEBUG = false;
	private static final boolean SAMPLERATE = true; // display pubs per second

    private BufferedImage image = null;
    private PlayerFrame displayPanel;
    private Object mutex = new Object();
    private Object navMutex = new Object();
    ByteBuffer cb;
    byte[] buffer = new byte[0];
    
    double eulers[] = new double[]{0.0,0.0,0.0};
   
	String mode = "display";
	String outDir = "/";
	int frames = 0;
	int[] ibuf = null;
	// http
    int port = 8000;
    //CircularBlockingDeque<java.awt.Image> queue = new CircularBlockingDeque<java.awt.Image>(30);
    //CircularBlockingDeque<byte[]> bqueue = new CircularBlockingDeque<byte[]>(30);
    byte[] bqueue;
	private int sequenceNumber,lastSequenceNumber;
	long time1;
	
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_video");
	}
	@Override
	public void onStart(final ConnectedNode connectedNode) {

		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		if( remaps.containsKey("__mode") )
			mode = remaps.get("__mode");
		if( mode.equals("display")) {
			if( DEBUG )
				System.out.println("Pumping frames to AWT Panel");
			SwingUtilities.invokeLater(new Runnable() {
			    public void run() {
			        displayPanel = new PlayerFrame();
			        displayPanel.paintPanel();
			    }
			});
			SynchronizedThreadManager.getInstance().init(new String[] {"VIDEOLISTENER"});
			SynchronizedThreadManager.getInstance().spin(new Runnable() {
				@Override
				public void run() {
			        while(true) {
			        	//java.awt.Image dimage=null;
						//try {
							//dimage = queue.takeFirst();
						//} catch (InterruptedException e) {
							//e.printStackTrace();
						//}
			        	if( image == null )
							try {
								Thread.sleep(1);
							} catch (InterruptedException e) {}
			        	else {
			        		synchronized(mutex) {
			        		displayPanel.setLastFrame((java.awt.Image)/*d*/image);
			        		synchronized(navMutex) {
			        			displayPanel.setComputedValues(eulers[0], eulers[1], eulers[2]);
			        		}
			        	//displayPanel.lastFrame = displayPanel.createImage(new MemoryImageSource(newImage.imageWidth
						//		, newImage.imageHeight, buffer, 0, newImage.imageWidth));
			        		displayPanel.invalidate();
			        		displayPanel.updateUI();
			        		}
			        	}
			        }
				}
			}, "VIDEOLISTENER");
		} else {
			// Spin up a server to server multipart image types
			SynchronizedThreadManager.getInstance().init(new String[] {"VIDEOLISTENER"});
			if( mode.equals("display_server") ) {
				if( DEBUG )
					System.out.println("Pumping frames as MJPEG via HTTP on 127.0.0.1:"+port);
				SynchronizedThreadManager.getInstance().spin(new Runnable() {
					@Override
					public void run() {
				        while(true) {
				        	ServerSocket ssocket = null;
				        	try {
				        		ssocket = new ServerSocket(port);
				        		while (true) {
				        			Socket client_socket = ssocket.accept();
				        			if( DEBUG )
				        				System.out.println("Connection established from "+client_socket);
				        			SynchronizedThreadManager.getInstance().spin(new StandardImageConnection(client_socket), "VIDEOLISTENER");
				        		}
				        	} catch (Exception e) {
				        		System.err.println("Exception occurred: " + e);
				        	}
				        	finally {
				        		if(ssocket != null)
				        			try {
				        				ssocket.close();
				        			} catch (IOException e) {}
				        	}
				        }
					}
				}, "VIDEOLISTENER");		
			}/* else { // mode has output directory
				
				if( mode.equals("rtsp_server") ) {
					if( DEBUG )
						System.out.println("Pumping frames to RTSP server");
					try {
						RtspServer.init();
					} catch (Exception e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				} else {
					// if mode is not display, or display_server, look for output file directory
					outDir = remaps.get("__mode");
					if( DEBUG )
						System.out.println("Sending video files to :"+outDir);
				}
			}*/
		}
		//final Subscriber<sensor_msgs.Image> imgsub =
		//		connectedNode.newSubscriber("/sensor_msgs/Image", sensor_msgs.Image._TYPE);
		final Subscriber<stereo_msgs.StereoImage> imgsub =
				connectedNode.newSubscriber("/stereo_msgs/StereoImage", stereo_msgs.StereoImage._TYPE);
		final Subscriber<sensor_msgs.Imu> subsimu = 
				connectedNode.newSubscriber("/sensor_msgs/Imu", sensor_msgs.Imu._TYPE);
		/**
		 * Image extraction from bus, then image processing, then on to display section.
		 */
		//imgsub.addMessageListener(new MessageListener<sensor_msgs.Image>() {
		//@Override
		//public void onNewMessage(sensor_msgs.Image img) {
		imgsub.addMessageListener(new MessageListener<stereo_msgs.StereoImage>() {
			@Override
			public void onNewMessage(stereo_msgs.StereoImage img) {
			long slew = System.currentTimeMillis() - time1;
			if( SAMPLERATE && slew >= 1000) {
				time1 = System.currentTimeMillis();
				System.out.println("Samples per second:"+(sequenceNumber-lastSequenceNumber)+". Slew rate="+(slew-1000));
				lastSequenceNumber = sequenceNumber;
			}
			synchronized(buffer) {
				cb = img.getData(); // left image, right image is in getData2
				buffer = cb.array(); // 3 byte BGR
			}
			//IntBuffer ib = cb.toByteBuffer().asIntBuffer();
			if( DEBUG ) {
				System.out.println("New image "+img.getWidth()+","+img.getHeight()+" size:"+buffer.length/*ib.limit()*/);
			}
			//image = new BufferedImage(img.getWidth(), img.getHeight(), BufferedImage.TYPE_3BYTE_BGR);
			//image = new BufferedImage(img.getWidth(), img.getHeight(), BufferedImage.TYPE_4BYTE_ABGR);
			/*
			image = new BufferedImage(img.getWidth(), img.getHeight(), BufferedImage.TYPE_INT_ARGB);
			WritableRaster raster = (WritableRaster) image.getRaster();
			int ibsize = img.getHeight() * img.getWidth();
			if( ibuf == null )
				ibuf = new int[ibsize * 4];
			int iup = 0;
			int ip = 0;
			for(int i = 0; i < ibsize; i++) {
				int ialph = 255; //(ib.get(i) >> 24) & 0x0ff;
				//int ired = (ib.get(i) >> 16) & 0x0ff; 
				//int igreen = (ib.get(i) >> 8 ) & 0x0ff;
				//int iblue = (ib.get(i) & 0x0ff);
				int iblue = buffer[ip++];
				int igreen = buffer[ip++];
				int ired = buffer[ip++];
				ibuf[iup++] = ired;
				ibuf[iup++] = igreen;
				ibuf[iup++] = iblue;
				ibuf[iup++] = ialph;
			}
			//System.out.println(ibuf.length+" "+raster.getWidth()+" "+raster.getHeight()+" "+raster.getMinX()+" "+raster.getMinY());
		    raster.setPixels(0, 0, img.getWidth(), img.getHeight(), ibuf);
			*/
			
			if( mode.equals("display")) {
				try {
					synchronized(buffer) {
						InputStream in = new ByteArrayInputStream(buffer);
						synchronized(mutex) {
						image = ImageIO.read(in);
						}
						in.close();
					}
				} catch (IOException e1) {
					System.out.println("Could not convert image payload due to:"+e1.getMessage());
					return;
				}
				//displayPanel.setLastFrame((java.awt.Image)image);
				//displayPanel.setLastFrame(displayPanel.createImage(new MemoryImageSource(newImage.imageWidth
				//		, newImage.imageHeight, buffer, 0, newImage.imageWidth)));
				//displayPanel.invalidate();
				//displayPanel.updateUI();
				//queue.addLast(image);
				System.gc();
			} else {
				if( mode.equals("display_server")) {
					//bqueue.addLast(buffer);
					bqueue = buffer;
				} /*else { 
					if( mode.equals("rtsp_server")){
						RtspServer.queue.addLast(buffer);
					} else {
						// write file for each frame
						// if its a jpeg payload this is inefficient but still format independent
						InputStream in = new ByteArrayInputStream(buffer);
						try {
							image = ImageIO.read(in);
						} catch (IOException e1) {
							System.out.println("Could not convert image payload due to:"+e1.getMessage());
							return;
						}
						File f = new File(outDir+"/roscoe"+(++frames)+".jpg");
						try {
							ImageIO.write(image, "jpg", f);
							if( DEBUG )
								System.out.println("Wrote: roscoe"+frames+".jpg");
						} catch (IOException e) {
							System.out.println("Cant write image roscoe"+frames+".jpg");
							return;
						}	
					}
				}*/
			}
			++sequenceNumber; // we want to inc seq regardless to see how many we drop	
		}
		});
		
		subsimu.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
			@Override
			public void onNewMessage(sensor_msgs.Imu message) {
				synchronized(navMutex) {
					eulers = message.getOrientationCovariance();
					//System.out.println("Nav:Orientation X:"+orientation.getX()+" Y:"+orientation.getY()+" Z:"+orientation.getZ()+" W:"+orientation.getW());
					if(DEBUG)
					System.out.println("Nav:Eulers "+eulers[0]+" "+eulers[1]+" "+eulers[2]);
				}
			}

		});
	}
	
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
						byte[] b = bqueue;//.takeFirst();
        				out.write(b, 0, b.length);
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
	
	class PlayerFrame extends JPanel {
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
	        frame.setUndecorated(true);
	        // Make frame topmost
	        frame.setAlwaysOnTop(true);
	        // Disable Alt+F4 on Windows
	        frame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
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
		    * Set up the "Roll" label
		    */
		        JLabel yJLabel = new JLabel("Roll:");
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
		        yField = new JTextField(8);
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
		        df = new DecimalFormat("####0.00");
		        dfs = new DecimalFormat("+####0.00;-####0.00");
		        da = new DecimalFormat("####0.0\u00b0");
		        das = new DecimalFormat("+####0.0\u00b0;-####0.0\u00b0");

		   /*
		    * Initialize all displayed values to 0, except the angles,
		    * which are set to 90 degrees.
		    */
		        setComputedValues(0.0, 0.0, 0.0);
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
		    void setComputedValues(double yaw, double roll, double pitch)
		    {
		        xField.setText(da.format(yaw));
		        yField.setText(da.format(roll));
		        aField.setText(da.format(pitch));

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

		}
	
}

