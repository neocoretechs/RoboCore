/**
 * AR Drone driver for ROS
 * Based on the JavaDrone project and rosjava and ardrone_utd and the original Japanese version and some other
 * assorted code.
 * @author jg
 */
package com.neocoretechs.robocore;

import geometry_msgs.Quaternion;

import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLConnection;
import java.nio.ByteBuffer;

import javax.imageio.ImageIO;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.Time;

/**
 * This class takes a series of images from the UV4l and remuxxed onto the ROS bus.
 * @author jg
 *
 */
public class VideoPub extends AbstractNodeMain  {
	private boolean DEBUG = true;
	//double phi, theta, psi;
	float rangeTop; // Ultrasonic sensor
	
	byte[] bbuf = null;// RGB buffer

	boolean imageReady = false;
	
	boolean started = true;
	boolean videohoriz = true;
	boolean emergency = false;

	int visionDistance, visionAngle, visionX, visionY;
	
	boolean isTemp = true;
	boolean isPress = true;
	boolean isTag = false;
	boolean isVision = false;
	String visionName;
	
	Time tst;
	//int imwidth = 672, imheight = 418;
	int imwidth = 640, imheight = 480;

	
	Object vidMutex = new Object();

	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("videopub");
	}

	/**
	 * 
	 * Start the main processing pipeline. We subscribe to an external ranging message robocore/range to augment the
	 * ultrasonic range and supply a complete point cloud to the range/ultrasonic/ardrone channel. If we have ultrasonics
	 * they will be in the first elements of the point cloud array in X in addition to element 0 X being the ARDrone ranger
	 */
	@Override
	public void onStart(final ConnectedNode connectedNode) {

		//final Log log = connectedNode.getLog();
		final Publisher<sensor_msgs.Image> imgpub =
		connectedNode.newPublisher("ardrone/image_raw", sensor_msgs.Image._TYPE);
		// caminfopub has camera info
		//final Publisher<sensor_msgs.CameraInfo> caminfopub =
		//connectedNode.newPublisher("ardrone/camera_info", sensor_msgs.CameraInfo._TYPE);
	
		
	/**
	 * Main publishing loop. Essentially we are publishing the data in whatever state its in, using the
	 * mutex appropriate to establish critical sections. A sleep follows each publication to keep the bus arbitrated
	 * This CancellableLoop will be canceled automatically when the node shuts down
	 */
	connectedNode.executeCancellableLoop(new CancellableLoop() {
		private int sequenceNumber;
		byte[] bbuf;
		BufferedImage bi = null;
		byte[] byteChunk = new byte[8192];
		ByteArrayOutputStream baos = new ByteArrayOutputStream();
		@Override
		protected void setup() {
			sequenceNumber = 0;
		}

		@Override
		protected void loop() throws InterruptedException {
			std_msgs.Header imghead = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
	
			imghead.setSeq(sequenceNumber);
			tst = connectedNode.getCurrentTime();
			imghead.setStamp(tst);
			imghead.setFrameId(tst.toString());
			sensor_msgs.Image imagemess = imgpub.newMessage();
 
			try {
				//ByteArrayOutputStream os = new ByteArrayOutputStream();
				//JPEGImageEncoder encoder = JPEGCodec.createJPEGEncoder(os);
				URL iurl = new URL("http://172.16.0.102:9090/stream/snapshot.jpeg");
				InputStream istream = iurl.openStream();
				
				if( istream == null) {
					System.out.println("Returned URL image stream null "+sequenceNumber);
					imageReady = false;
				} else {
					int n;
					baos.reset();
					while ( (n = istream.read(byteChunk)) > 0 ) {
						//System.out.println(n+" read ");
						baos.write(byteChunk, 0, n);
					}
					//bi = ImageIO.read(istream);
					imageReady = true;
					baos.flush();
					bbuf = baos.toByteArray();
				}
				//os.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}//
				if( DEBUG )
					if( imageReady )
						System.out.println("Added frame "+imwidth+","+imheight+" "+bbuf.length);
					else
						System.out.println("Image not ready "+sequenceNumber++);
			//} catch(IllegalStateException ise) {
				// buffer full;
				//System.out.println("Video buffer full!");
				//vidbuf.clear();
			//}

			if( bbuf != null && imageReady) {
				synchronized(vidMutex) {		
				//sensor_msgs.CameraInfo caminfomsg = caminfopub.newMessage();
            	//System.out.println("Image:"+newImage.imageWidth+","+newImage.imageHeight+" queue:"+list.size());
					imagemess.setData(ByteBuffer.wrap(bbuf));
				}
				imagemess.setEncoding("JPG"/*"8UC3"*/);
				
				imagemess.setWidth(imwidth);
				imagemess.setHeight(imheight);
				imagemess.setStep(imwidth);
				imagemess.setIsBigendian((byte)0);
				imagemess.setHeader(imghead);
				//
				//caminfomsg.setHeader(imghead);
				//caminfomsg.setWidth(imwidth);
				//caminfomsg.setHeight(imheight);
				//caminfomsg.setDistortionModel("plumb_bob");
				//caminfomsg.setK(K);
				//caminfomsg.setP(P);
				
				imgpub.publish(imagemess);
				imageReady = false;
				if( DEBUG )
					System.out.println("Pub. Image:"+sequenceNumber);
				//caminfopub.publish(caminfomsg);
				//System.out.println("Pub cam:"+imagemess);
				sequenceNumber++;  	
			}
			
			Thread.sleep(1);		
		}
	});
	}
	
}
