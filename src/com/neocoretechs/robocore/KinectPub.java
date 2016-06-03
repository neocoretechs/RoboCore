package com.neocoretechs.robocore;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ShortBuffer;

//import javax.imageio.ImageIO;
//import java.awt.image.BufferedImage;


import java.util.ArrayList;
import java.util.List;

import javax.imageio.ImageIO;

import org.openkinect.Context;
import org.openkinect.DepthFormat;
import org.openkinect.Device;
//import org.openkinect.Acceleration;
import org.openkinect.Image;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
//import org.openkinect.LEDStatus;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

	/**
	 * Spin a kinect reader thread via KinectReader class that populates a queue of depth maps that we
	 * process during the publishing loop to pass down the ROS publishing pipeline. We decouple the depth
	 * map acquisition to prevent conflict with ROS publishing loop.
	 * @author jg
	 *
	 */
	public class KinectPub extends AbstractNodeMain
	{
		private static boolean DEBUG = true;
		private Context context = null;
		private Device device = null;
		private static KinectReader kinectReader = new KinectReader();
		ShortBuffer data;
		@Override
		public GraphName getDefaultNodeName() {
			return GraphName.of("pubs_kinect");
		}
		
		@Override
		public void onStart(final ConnectedNode connectedNode) {
		
  			//final Publisher<sensor_msgs.PointCloud> kinpub  = connectedNode.newPublisher("robocore/kinect", sensor_msgs.PointCloud._TYPE);
  			final Publisher<std_msgs.UInt16MultiArray> kinpub  = connectedNode.newPublisher("robocore/kinect", std_msgs.UInt16MultiArray._TYPE);
	    	/**
	    	 * Main publishing loop. Essentially we are publishing the data in whatever state its in, using the
	    	 * mutex appropriate to establish critical sections. A sleep follows each publication to keep the bus arbitrated
	    	 * This CancellableLoop will be canceled automatically when the node shuts down
	    	 */
	    	connectedNode.executeCancellableLoop(new CancellableLoop() {
	    		final int width = 640, height = 480;
	    		private int sequenceNumber;
	    		private short[] sbuf = new short[width*height];
	    		
	  			//List<geometry_msgs.Point32> point;
	  			
	    		@Override
	    		protected void setup() {
	    			sequenceNumber = 0;
	    	       
	    		}

	    		@Override
	    		protected void loop() throws InterruptedException {
	    			boolean found = false;
	    			//device.light(LEDStatus.LED_GREEN);
	    			//device.tilt(-5);
	    			/*
	    			FileInputStream ftex = null;
	    			try {
	    			ftex = new FileInputStream(new File(System.getProperty("user.home") + File.separator + "kinect.depth"));
	    			} catch(IOException ioe){ ioe.printStackTrace();}
	    			*/
	    			//point = new ArrayList<geometry_msgs.Point32>(width*height);
	    			ShortBuffer data = kinectReader.getDepth();	
	    		       
	    			if( DEBUG )
	    					System.out.println("kinect Publisher loop.."+sequenceNumber);
	    		
	    			//String home = System.getProperty("user.home");
	    			//BufferedImage bi = null;
	    			/*
	    			try {		
	    				ftex.read(buf);
					//	bi = ImageIO.read(fbi);
					} catch (IOException e) {
					//	bi = null;
					}
					*/
	    			//if( bi != null && bi.getWidth() == width || bi.getHeight() == height) {
	    			//ByteBuffer bdata = ByteBuffer.wrap(buf);
	    			//ShortBuffer data = bdata.asShortBuffer();
	    			//int ix = 0;
	    			if( data != null ) {
	    				/*
	    				for(int y=0; y<height; y++) {
	    					for(int x=0; x<width; x++) {
	    						//int offset = (y*width+x);
	    						//short d = (short) bi.getRGB(x,y);
	    						short d = data.get( ix++ );
	    						if( d != 0 ) found = true;
	    						geometry_msgs.Point32 pt =  connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Point32._TYPE);
	    						pt.setX(x);
	    						pt.setY(y);
	    						pt.setZ(d);
	    						point.add(pt);
	    					}
	    				}
	    				sensor_msgs.PointCloud points =  kinpub.newMessage();
	    				points.setPoints(point);
	    				*/
	    				std_msgs.UInt16MultiArray points = kinpub.newMessage();
	    				data.get(sbuf); // array method call not supported for direct buffer
	    				points.setData(sbuf);
	    				
	    				kinpub.publish(points);
	    				
	    				if( DEBUG ) {
	    					System.out.println("Publish Depth map #"+String.valueOf(++sequenceNumber)+" map:"+points.getData().length);
	    					//if(!found) {
	    					//	System.out.println("NO NON ZERO ELEMENTS FOUND TO PUBLISH!");
	    					//}
	    				}
	    				//try {
	    				//	ftex.flush();
	    				//    ftex.close();
	    				//} catch (IOException e) {
	    				//	e.printStackTrace();
	    				//}
	    			} else {
	    				// no image in the buffer, nothing published, give it a chance to catch up
	    				Thread.sleep(50);
	    				if( DEBUG ) {
	    					System.out.println("No depth map..");
	    				}
	    			} 
	    			
	    			
	    		} // loop
	    		
	        
	        //String home = System.getProperty("user.home");
	        //System.out.println("Writing "+home);
	        //ImageIO.write(color, "jpg", new File(home + File.separator + "kinect.color.jpg"));
	        //System.out.println("Written, color..");
	        //ImageIO.write(depth, "jpg", new File(home + File.separator + "kinect.depth.jpg"));
	        //System.out.println("Written, depth");
	    		
		});// cancellable loop 
	        
	 } // onStart
	
	@Override
	public void onShutdownComplete(final Node shutNode) {
		if( DEBUG )
			System.out.println("Kinect reader shutdown..");
		kinectReader.shutdown();
	}

	
}
