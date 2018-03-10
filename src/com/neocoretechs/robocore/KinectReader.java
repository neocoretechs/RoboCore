package com.neocoretechs.robocore;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.util.concurrent.ArrayBlockingQueue;

import javax.imageio.ImageIO;

import org.openkinect.Context;
import org.openkinect.DepthFormat;
import org.openkinect.Device;
import org.openkinect.Image;
import org.openkinect.LEDStatus;
/**
 * Start a thread which populates a blocking queue with successive kinect depth buffers
 * of ShortBuffer type. The intent is to decouple ROS publishing loop from kinect processing pipeline which seem
 * to be incompatible in the local environment. Uses thread group 'kinect''.
 * @author jg
 *
 */
public class KinectReader implements Runnable{
	private static boolean DEBUG = true;
	private static volatile boolean shouldRun = true;
	//private Device device = null;
	//private Context context = null;
	private static int sequenceNumber = 0;
	public static final int width = 640, height = 480;
	
	public static ArrayBlockingQueue<ShortBuffer> depths = new ArrayBlockingQueue<ShortBuffer>(1024);
	
	public KinectReader() {
		ThreadPoolManager.init(new String[]{"kinect"});
		ThreadPoolManager.getInstance().spin(this, "kinect");
	}
	/*
	private void setup() {
	     context = Context.getContext();
	        
	        if( context == null ) {
	        	System.out.println("The Kinect device context has returned null!");
	        	return;
	        }

	        if(context.devices() < 1) {
	            System.out.println("No Kinect devices found in the current context, there may be an installation error.");
	            return;
	        }
	        if(DEBUG)
	        	System.out.println("Devices:"+context.devices());

	        device = context.getDevice(0);
	        
	        if( device == null ) {
	        	System.out.println("The Kinect Device returned from the context is null!");
	        	System.out.println("May need modprobe to resolve user space driver conflict, see docs");
	        	return;
	        }
	}
	*/
	/**
	 * Poll the queue for a depth map, return null if the queue is empty
	 * @return
	 * @throws InterruptedException
	 */
	public ShortBuffer getDepth() throws InterruptedException {
		return depths.take();
	}
	
	
	/**
	 * Signal run method loop that we are done, which causes cleanup of device context
	 */
	public void shutdown() {
		shouldRun = false;
	}
	

	
	//public static void main(String[] args) throws Exception {
	public void run() {	
	     Context context = Context.getContext();
	        
	        if( context == null ) {
	        	System.out.println("The Kinect device context has returned null!");
	        	return;
	        }

	        if(context.devices() < 1) {
	            System.out.println("No Kinect devices found in the current context, there may be an installation error.");
	            return;
	        }
	        if(DEBUG)
	        	System.out.println("Devices:"+context.devices());

	        Device device = context.getDevice(0);
	        
	        if( device == null ) {
	        	System.out.println("The Kinect Device returned from the context is null!");
	        	System.out.println("May need modprobe to resolve user space driver conflict, see docs");
	        	return;
	        }
	        //final short[] sbuff = new short[width*height];
	
			
		while(shouldRun) {
			if( DEBUG )
				System.out.println("kinect Reader loop..");
	
			device.depth(new Image() {
				public void data(ByteBuffer image) {
					ShortBuffer data = image.asShortBuffer();
					try {
						//if( depths.remainingCapacity() <= 1 )
						//	System.out.println("WARNING:Kinect reader queue nearing capacity...");
						depths.put(data);
						
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
				
			}, DepthFormat.RAW_11);
			
			while(context.processEvents())
				try {
					Thread.sleep(50);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			
			device.depth(null);
		
		} // shouldRun
		// If we shut down processor dispose of device context
		if( DEBUG )
			System.out.println("Shutting down kinect reader..");
		device.dispose();
	}
	

}
