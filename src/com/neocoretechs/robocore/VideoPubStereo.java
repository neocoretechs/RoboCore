package com.neocoretechs.robocore;

import java.nio.ByteBuffer;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.message.Time;

import au.edu.jcu.v4l4j.CaptureCallback;
import au.edu.jcu.v4l4j.DeviceInfo;
import au.edu.jcu.v4l4j.FrameGrabber;
import au.edu.jcu.v4l4j.ImageFormat;
import au.edu.jcu.v4l4j.V4L4JConstants;
import au.edu.jcu.v4l4j.VideoDevice;
import au.edu.jcu.v4l4j.VideoFrame;
import au.edu.jcu.v4l4j.exceptions.StateException;
import au.edu.jcu.v4l4j.exceptions.V4L4JException;

/**
 * This class takes a series of stereoscopic images from V4L4j and remuxxes them onto the ROS bus.
 * The images are encoded jpeg via capture class, then the byte payload is sent downline.
 * Changing the capture class in initFrameGrabber of embedded videocapL class can change the format.
 * @author jg Copyright (C) NeoCoreTechs 2017,2018
 *
 */
public class VideoPubStereo extends AbstractNodeMain {
	private static final boolean DEBUG = false;
	private static final boolean SAMPLERATE = true; // display pubs per second


	boolean imageReadyL = false;
	boolean imageReadyR = false;
	
	private int lastSequenceNumber;
	long time1;
	
	Time tst;

	int imwidth = 640, imheight = 480;
	
	Object vidMutexL = new Object();
	Object vidMutexR = new Object();
	
	private static int width = 640, height = 480, std = V4L4JConstants.STANDARD_WEBCAM, channel = 0;
	private static String deviceL = "/dev/video0";
	private static String deviceR = "/dev/video1";
	private VideoDevice videoDeviceL;
	private VideoDevice videoDeviceR;
	private FrameGrabber frameGrabberL;
	private FrameGrabber frameGrabberR;

	ByteBuffer bbL;
	ByteBuffer bbR;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("pubs_stereovideo");
	}

	/**
	 * 
	 */
	@Override
	public void onStart(final ConnectedNode connectedNode) {

		//final Log log = connectedNode.getLog();
		final Publisher<stereo_msgs.StereoImage> imgpub =
		connectedNode.newPublisher("/stereo_msgs/StereoImage", stereo_msgs.StereoImage._TYPE);
		//final Publisher<sensor_msgs.Image> rimgpub =
		//		connectedNode.newPublisher("/sensor_msgs/rImage", sensor_msgs.Image._TYPE);
		// caminfopub has camera info
		//final Publisher<sensor_msgs.CameraInfo> caminfopub =
		//connectedNode.newPublisher("/sensor_msgs/CameraInfo", sensor_msgs.CameraInfo._TYPE);
	
	/**
	 * Main publishing loop. Essentially we are publishing the data in whatever state its in, using the
	 * mutex appropriate to establish critical sections. A sleep follows each publication to keep the bus arbitrated
	 * This CancellableLoop will be canceled automatically when the node shuts down
	 */
	connectedNode.executeCancellableLoop(new CancellableLoop() {
		private int sequenceNumber;
		videocapL vidcapL;
		videocapR vidcapR;
		@Override
		protected void setup() {
			sequenceNumber = 0;
			
			try {
				vidcapL = new videocapL();
				vidcapL.initFrameGrabber();
			} catch (V4L4JException e1) {
				System.out.println("Error setting up capture for left camera");
				e1.printStackTrace();
				// cleanup and exit
				vidcapL.cleanupCapture();
				return;
			}
			
			try {
				vidcapR = new videocapR();
				vidcapR.initFrameGrabber();
			} catch (V4L4JException e1) {
				System.out.println("Error setting up capture for right camera");
				e1.printStackTrace();
				// cleanup and exit
				vidcapR.cleanupCapture();
				return;
			}
		}

		@Override
		protected void loop() throws InterruptedException {
			
			std_msgs.Header imghead = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
			imghead.setSeq(sequenceNumber);
			tst = connectedNode.getCurrentTime();
			imghead.setStamp(tst);
			imghead.setFrameId(tst.toString());
			//sensor_msgs.Image imagemess = imgpub.newMessage();
			//stereo_msgs.StereoImage imagemess = imgpub.newMessage();
			stereo_msgs.StereoImage imagemess = connectedNode.getTopicMessageFactory().newFromType(stereo_msgs.StereoImage._TYPE);
			/*
			std_msgs.Header rimghead = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
			rimghead.setSeq(sequenceNumber);
			tst = connectedNode.getCurrentTime();
			rimghead.setStamp(tst);
			rimghead.setFrameId(tst.toString());
			sensor_msgs.Image rimagemess = rimgpub.newMessage();
 			*/
						
			if( imageReadyL && imageReadyR ) {
				if( SAMPLERATE && System.currentTimeMillis() - time1 >= 1000) {
					time1 = System.currentTimeMillis();
					System.out.println("Samples per second:"+(sequenceNumber-lastSequenceNumber));
					lastSequenceNumber = sequenceNumber;
				}
				if( DEBUG )
					System.out.println(sequenceNumber+":Added frame "+imwidth+","+imheight);
				
				synchronized(vidMutexL) {
					imagemess.setData(bbL);
					imagemess.setData2(bbR);
					imagemess.setEncoding("JPG");
					imagemess.setWidth(imwidth);
					imagemess.setHeight(imheight);
					imagemess.setStep(imwidth);
					imagemess.setIsBigendian((byte)0);
					imagemess.setHeader(imghead);
					imgpub.publish(imagemess);
					//
				
					/*
					rimagemess.setData(bbR);
					rimagemess.setEncoding("JPG");
					rimagemess.setWidth(imwidth);
					rimagemess.setHeight(imheight);
					rimagemess.setStep(imwidth);
					rimagemess.setIsBigendian((byte)0);
					rimagemess.setHeader(rimghead);
					rimgpub.publish(rimagemess);
					*/
					//
					imageReadyL = false;
					imageReadyR = false;
					if( DEBUG )
						System.out.println("Pub. Image:"+sequenceNumber);	
				}
				//
				//caminfomsg.setHeader(imghead);
				//caminfomsg.setWidth(imwidth);
				//caminfomsg.setHeight(imheight);
				//caminfomsg.setDistortionModel("plumb_bob");
				//caminfomsg.setK(K);
				//caminfomsg.setP(P);
				//caminfopub.publish(caminfomsg);
				//System.out.println("Pub cam:"+imagemess);
			} else {
				if(DEBUG)
					System.out.println("Image(s) not ready "+sequenceNumber);
				Thread.sleep(1);
				++lastSequenceNumber; // if no good, up the last sequence to compensate for sequence increment
			}
			++sequenceNumber; // we want to inc seq regardless to see how many we drop	
		}
	});
	}
	
	/**
	 *  * A typical <code>FrameGrabber</code> use case is as follows:
	 * <br>
	 * Create the video deviceL and frame grabber:
	 * <code><br><br>
	 * //create a new video deviceL<br>
	 * VideoDevice vd = new VideoDevice("/dev/video0");<br>
	 * // vd.setThreadFactory(myThreadFactory); // set your own thread factory if required.
	 * <br>//Create an instance of FrameGrabber
	 * <br>FrameGrabber f = vd.getJPEGFrameGrabber(320, 240, 0, 0, 80);
	 * <br>//If supported, this frame grabber will capture frames and convert them
	 * <br>//to JPEG before delivering them to your application. 
	 * <br>
	 * <br>//Instantiate an object that implements the {@link CaptureCallback}
	 * <br>//interface which will receive captured frame
	 * <br>myCallbackObject = new MyCallbackObjectclass();
	 * <br>
	 * <br>//pass it to the frame grabber.
	 * <br>f.setCaptureCallback(myCallbackObject);
	 * <br>
	 * <br>//Now start the capture
	 * <br>f.startCapture();
	 * <br>
	 * <br>//At this stage, myCallbackObject.nextFrame() will be called every time
	 * <br>//a new frame is available (see next paragraph).
	 * <br>
	 * <br>//When appropriate, stop the capture
	 * <br>f.stopCapture();
	 * <br>//Release the frame grabber
	 * <br>vd.releaseFrameGrabber();
	 * <br>//Release the video deviceL
	 * <br>vd.release();
	 * </code><br><br>
	 * In myCallbackObject, when the capture is started, the following method is
	 * called every time a new frame is available :<br>
	 * <code>
	 * <br>public void nextFrame(VideoFrame frame) {
	 * <br>&nbsp;&nbsp; //do something useful with frame. But make sure you recycle
	 * <br>&nbsp;&nbsp; //it when dealt with, so v4l4j can re-use it later on
	 * <br>&nbsp;&nbsp; frame.recycle();
	 * <br>}<br>
	 * </code>
	 *
	 */
	class videocapL implements CaptureCallback {
		
	private void initFrameGrabber() throws V4L4JException {
		videoDeviceL = new VideoDevice(deviceL);
		DeviceInfo deviceInfoL=videoDeviceL.getDeviceInfo();
		if (deviceInfoL.getFormatList().getNativeFormats().isEmpty()) {
		    System.out.println("Couldn't detect native format for the left camera device.");
		} else {
			for(ImageFormat fi : deviceInfoL.getFormatList().getNativeFormats() )
				System.out.println(fi);
		}
		  //ImageFormat imageFormat=deviceInfo.getFormatList().getNativeFormat(0);
		
		frameGrabberL = videoDeviceL.getJPEGFrameGrabber(width, height, channel, std, 80);
		frameGrabberL.setCaptureCallback(this);
		width = frameGrabberL.getWidth();
		height = frameGrabberL.getHeight();
		System.out.println("Starting capture of left camera at " + width + "x" + height);
		frameGrabberL.startCapture();
	}
	
	private void cleanupCapture() {
		try {
			frameGrabberL.stopCapture();
		}
		catch (StateException ex) {
			// the frame grabber may be already stopped, so we just ignore
			// any exception and simply continue.
		}
		// release the frame grabber and video deviceL
		videoDeviceL.releaseFrameGrabber();
		videoDeviceL.release();
	}
	
	@Override
	public void nextFrame(VideoFrame frame) {
		// This method is called when a new frame is ready.
		// Don't forget to recycle it when done dealing with the frame.
		// draw the new frame onto the JLabel
		synchronized(vidMutexL) {
			//bi = frame.getBufferedImage();
			bbL = ByteBuffer.wrap(frame.getBytes());
		}
		imageReadyL = true;
		// recycle the frame
		frame.recycle();
	}
	
	@Override
	public void exceptionReceived(V4L4JException e) {
		// This method is called by v4l4j if an exception
		// occurs while waiting for a new frame to be ready.
		// The exception is available through e.getCause()
		e.printStackTrace();
	}
	}
	
	class videocapR implements CaptureCallback {
		
	private void initFrameGrabber() throws V4L4JException {
		videoDeviceR = new VideoDevice(deviceR);
		DeviceInfo deviceInfoR=videoDeviceR.getDeviceInfo();
		if (deviceInfoR.getFormatList().getNativeFormats().isEmpty()) {
		    System.out.println("Couldn't detect native format for the right camera device.");
		} else {
			for(ImageFormat fi : deviceInfoR.getFormatList().getNativeFormats() )
				System.out.println(fi);
		}
		frameGrabberR = videoDeviceR.getJPEGFrameGrabber(width, height, channel, std, 80);
		frameGrabberR.setCaptureCallback(this);
		//width = frameGrabberR.getWidth();
		//height = frameGrabberR.getHeight();
		System.out.println("Starting capture of right camera at " + width + "x" + height);
		frameGrabberR.startCapture();
	}
	
	private void cleanupCapture() {
		try {
			frameGrabberR.stopCapture();
		}
		catch (StateException ex) {
			// the frame grabber may be already stopped, so we just ignore
			// any exception and simply continue.
		}
		// release the frame grabber and video deviceL
		videoDeviceR.releaseFrameGrabber();
		videoDeviceR.release();
	}
	
	@Override
	public void nextFrame(VideoFrame frame) {
		// This method is called when a new frame is ready.
		// Don't forget to recycle it when done dealing with the frame.
		// draw the new frame onto the JLabel
		synchronized(vidMutexR) {
			bbR = ByteBuffer.wrap(frame.getBytes());
		}
		imageReadyR = true;
		// recycle the frame
		frame.recycle();
	}
	
	@Override
	public void exceptionReceived(V4L4JException e) {
		// This method is called by v4l4j if an exception
		// occurs while waiting for a new frame to be ready.
		// The exception is available through e.getCause()
		e.printStackTrace();
	}
	}
}
