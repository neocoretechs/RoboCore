package com.neocoretechs.robocore.video;

import java.nio.ByteBuffer;
import java.util.List;
import java.util.Map;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;

import org.ros.message.Time;

import au.edu.jcu.v4l4j.CaptureCallback;
import au.edu.jcu.v4l4j.DeviceInfo;
import au.edu.jcu.v4l4j.FrameGrabber;
import au.edu.jcu.v4l4j.ImageFormat;
import au.edu.jcu.v4l4j.ImageFormatList;
import au.edu.jcu.v4l4j.ResolutionInfo;
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
	CircularBlockingDeque<byte[]> bqueueL = new CircularBlockingDeque<byte[]>(5);
	CircularBlockingDeque<byte[]> bqueueR = new CircularBlockingDeque<byte[]>(5);

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
	private ImageFormat videoBestImageFormatL = null;
	private ImageFormat videoBestImageFormatR = null;

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
		// Optionally get video devices from command line __left:= and __right:= params
		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		if( remaps.containsKey("__left") )
			deviceL = remaps.get("__left");
		if( remaps.containsKey("__right") )
			deviceR = remaps.get("__right");
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
						
			//if( imageReadyL && imageReadyR ) {
				if( SAMPLERATE && System.currentTimeMillis() - time1 >= 1000) {
					time1 = System.currentTimeMillis();
					System.out.println("Samples per second:"+(sequenceNumber-lastSequenceNumber));
					lastSequenceNumber = sequenceNumber;
				}
				if( DEBUG )
					System.out.println(sequenceNumber+":Added frame "+imwidth+","+imheight);
				
				//synchronized(vidMutexL) {
					//imagemess.setData(bbL);
					//imagemess.setData2(bbR);
					imagemess.setData(ByteBuffer.wrap(bqueueL.take()));
					imagemess.setData2(ByteBuffer.wrap(bqueueR.take()));
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
				//}
				//
				//caminfomsg.setHeader(imghead);
				//caminfomsg.setWidth(imwidth);
				//caminfomsg.setHeight(imheight);
				//caminfomsg.setDistortionModel("plumb_bob");
				//caminfomsg.setK(K);
				//caminfomsg.setP(P);
				//caminfopub.publish(caminfomsg);
				//System.out.println("Pub cam:"+imagemess);
			//} else {
			//	if(DEBUG)
			//		System.out.println("Image(s) not ready "+sequenceNumber);
			//	Thread.sleep(1);
			//	++lastSequenceNumber; // if no good, up the last sequence to compensate for sequence increment
			//}
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
		System.out.println("Video Device Name LEFT: "+deviceInfoL.getName());
		System.out.println("Device file: "+deviceInfoL.getDeviceFile());
		System.out.println("Supported formats:"); 
		if (deviceInfoL.getFormatList().getNativeFormats().isEmpty()) {
		    System.out.println("Couldn't detect native format for the camera device on the LEFT.");
		} else {
			for(ImageFormat fi : deviceInfoL.getFormatList().getNativeFormats() )
				System.out.println("\t"+fi.toNiceString());
		}
		videoBestImageFormatL = getVideoBestImageFormat(deviceInfoL);
		  //ImageFormat imageFormat=deviceInfo.getFormatList().getNativeFormat(0);
		
		//deviceInfoL.frameGrabberL = videoDeviceL.getJPEGFrameGrabber(width, height, channel, std, V4L4JConstants.MAX_JPEG_QUALITY);
		frameGrabberL = videoDeviceL.getJPEGFrameGrabber(width, height, channel, std, 80, videoBestImageFormatL);
		frameGrabberL.setCaptureCallback(this);
		width = frameGrabberL.getWidth();
		height = frameGrabberL.getHeight();
		System.out.println("Starting capture of camera on the LEFT at " + width + "x" + height);
		frameGrabberL.startCapture();
	}
	
	/**
	 * Release the video device after stopping capture.
	 * The frame grabber may be already stopped, so we just ignore
	 * any exception and simply continue.
	 */
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
	
	/**
	 * This method is called when a new frame is ready.
	 * Synchronized around video mutex, the left byte buffer is the frame wrapped by ByteBuffer.wrap.
	 * The frame is recycled after wrapping.
	 */
	@Override
	public void nextFrame(VideoFrame frame) {
		//synchronized(vidMutexL) {
			//bi = frame.getBufferedImage();
			//bbL = ByteBuffer.wrap(frame.getBytes());
		//}
		bqueueL.add(frame.getBytes());
		imageReadyL = true;
		// recycle the frame
		frame.recycle();
	}
	
	/**
	 * This method is called by v4l4j if an exception
	 * occurs while waiting for a new frame to be ready.
	 * The exception is available through e.getCause()
	 */
	@Override
	public void exceptionReceived(V4L4JException e) {
		e.printStackTrace();
	}
	}
	
	/**
	 * This is the right grabber class. They are organized in this manner due to sets of global resources being required for
	 * each global video device.
	 *
	 */
	class videocapR implements CaptureCallback {
		
	private void initFrameGrabber() throws V4L4JException {
		videoDeviceR = new VideoDevice(deviceR);
		DeviceInfo deviceInfoR=videoDeviceR.getDeviceInfo();
		System.out.println("Video Device Name RIGHT: "+deviceInfoR.getName());
		System.out.println("Device file: "+deviceInfoR.getDeviceFile());
		System.out.println("Supported formats:"); 
		if (deviceInfoR.getFormatList().getNativeFormats().isEmpty()) {
		    System.out.println("Couldn't detect native format for the camera device on the RIGHT.");
		} else {
			for(ImageFormat fi : deviceInfoR.getFormatList().getNativeFormats() )
				System.out.println(fi);
		}
		videoBestImageFormatR = getVideoBestImageFormat(deviceInfoR);
		//frameGrabberR = videoDeviceR.getJPEGFrameGrabber(width, height, channel, std, V4L4JConstants.MAX_JPEG_QUALITY);
		frameGrabberR = videoDeviceR.getJPEGFrameGrabber(width, height, channel, std, 80, videoBestImageFormatR);
		frameGrabberR.setCaptureCallback(this);
		//width = frameGrabberR.getWidth();
		//height = frameGrabberR.getHeight();
		System.out.println("Starting capture of camera on the RIGHT at " + width + "x" + height);
		frameGrabberR.startCapture();
	}
	/**
	 * Release the video device after stopping capture.
	 * The frame grabber may be already stopped, so we just ignore
	 * any exception and simply continue.
	 */
	private void cleanupCapture() {
		try {
			frameGrabberR.stopCapture();
		}
		catch (StateException ex) {
		}
		// release the frame grabber and video deviceL
		videoDeviceR.releaseFrameGrabber();
		videoDeviceR.release();
	}
	
	/**
	 * This method is called when a new frame is ready.
	 * Synchronized around video mutex, the right byte buffer is the frame wrapped by ByteBuffer.wrap.
	 * The frame is recycled after wrapping.
	 */
	@Override
	public void nextFrame(VideoFrame frame) {
		// This method is called when a new frame is ready.
		// Don't forget to recycle it when done dealing with the frame.
		//synchronized(vidMutexR) {
		//	bbR = ByteBuffer.wrap(frame.getBytes());
		//}
		bqueueR.add(frame.getBytes());
		imageReadyR = true;
		// recycle the frame
		frame.recycle();
	}
	
	/**
	 * This method is called by v4l4j if an exception
	 * occurs while waiting for a new frame to be ready.
	 * The exception is available through e.getCause()
	 */
	@Override
	public void exceptionReceived(V4L4JException e) {
		e.printStackTrace();
	}
	}
	
	private static final String[] BEST_FORMATS = new String[] {
			// MJPEG and JPEG are the best match because there is no need to
			// recalculate too much, hardware will deliver what we need
			"MJPEG", "JPEG",
			// 24-bit formats where every pixel can be stored in 3 bytes
			"BGR24", "RGB24",
			// 32-bit formats where every pixel can be stored in 4 bytes
			"BGR32", "RGB32",
			// next are YUV formats where every 2 pixels can be written in 4 bytes
			"YU", "UY", "YV", "UV"
		};

	private static ImageFormat getVideoBestImageFormat(DeviceInfo device) throws V4L4JException {

		if (device == null) {
			throw new IllegalArgumentException("Device must not be null!");
		}

		ImageFormatList formatsList = device.getFormatList();
		List<ImageFormat> formats = formatsList.getNativeFormats();//getJPEGEncodableFormats();

		int min = Integer.MAX_VALUE;
		ImageFormat bestFormat = null;

		for (ImageFormat format : formats) {

			ResolutionInfo info = format.getResolutionInfo();
			ResolutionInfo.Type type = info.getType();
			String name = format.getName();

			// skip unsupported resolution type

			switch (type) {
				case UNSUPPORTED:
				case DISCRETE:
				case STEPWISE:
					break;
				default:
					throw new V4L4JException("Unknown resolution type " + type);
			}

			System.out.println("Testing "+name+" type="+type);

			for (int i = 0; i < BEST_FORMATS.length; i++) {
				if (name.startsWith(BEST_FORMATS[i]) && i < min) {
					min = i;
					bestFormat = format;
				}
			}
		}

		System.out.println("Best image format match "+bestFormat);

		if (bestFormat == null) {
			throw new V4L4JException("No suitable image format detected");
		}

		return bestFormat;
	}
}
