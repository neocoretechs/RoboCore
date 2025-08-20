package com.neocoretechs.robocore.video;

import java.nio.ByteBuffer;
import java.util.List;
import java.util.Map;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import org.ros.message.Time;



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

	private int lastSequenceNumber;
	long time1;
	Time tst;
	int imwidth = 640, imheight = 480;
	
	private static int width = 640, height = 480;
	private FrameGrabber frameGrabber;
	
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
		long handle;
		@Override
		protected void setup() {
			sequenceNumber = 0;
			frameGrabber = new FrameGrabber();
			handle = frameGrabber.startCapture(false);
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
			byte[][] images = frameGrabber.getStereoJPEGFrames(handle);
			imagemess.setData(ByteBuffer.wrap(images[0]));
			imagemess.setData2(ByteBuffer.wrap(images[1]));
			imagemess.setEncoding("JPG");
			imagemess.setWidth(imwidth);
			imagemess.setHeight(imheight);
			imagemess.setStep(imwidth);
			imagemess.setIsBigendian((byte)0);
			imagemess.setHeader(imghead);
			imgpub.publish(imagemess);
				
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


}
