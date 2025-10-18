package com.neocoretechs.robocore.video;

import java.io.IOException;
import java.nio.ByteBuffer;

import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.atomic.AtomicInteger;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.relatrix.Relation;
import com.neocoretechs.relatrix.client.asynch.AsynchRelatrixClientTransaction;
import com.neocoretechs.relatrix.key.NoIndex;

import com.neocoretechs.robocore.SynchronizedThreadManager;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.rocksack.TransactionId;

/**
 * Create a database and receive published video images on the Ros bus from /stereo_msgs/StereoImage, then
 * store them
 * The function can use remapped command line param "__commitRate" int value, to change
 * the default of 100 images initiating a checkpoint transaction.<p/>
 * Image storage must be stopped and started via the service at uri cmd_store. The payload of the
 * request is either "begin" or "end".<p/> 
 * Demonstrates how we can manipulate the image buffer to store images in the categorical database.<p/>
 * @author Jonathan Groff (C) NeoCoreTechs 2021
 *
 */
public class VideoRecorderStereo extends AbstractNodeMain 
{
	private static boolean DEBUG = false;
	private static boolean DEBUGDIFF = true;
	private static final boolean SAMPLERATE = true; // display pubs per second

    private Object mutex = new Object();

    ByteBuffer cbl;
    byte[] bufferl = new byte[0];
    ByteBuffer cbr;
    byte[] bufferr = new byte[0];
	
    float compassHeading;
    float roll;
    float pitch;
    float temperature;
    
	String outDir = "/";
	int frames = 0;

	private int sequenceNumber,lastSequenceNumber;
	long time1;
	protected static boolean shouldStore = true;
	private static int MAXIMUM = 50000;
	int commitRate = 500;
	AtomicInteger commitCnt = new AtomicInteger();
	TransactionId xid;
	AsynchRelatrixClientTransaction session = null;

	static {
		SynchronizedThreadManager.getInstance().init(new String[] {"VIDEORECORDER"});
	}
	CircularBlockingDeque<TimedImage> queue = new CircularBlockingDeque<TimedImage>(30);
	

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_storevideo");
	}
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		try {
			session = connectedNode.getRelatrixClient();
			xid = session.getTransactionId();
		} catch (IOException e1) {
			e1.printStackTrace();
		}

		final Subscriber<stereo_msgs.StereoImage> imgsub =
				connectedNode.newSubscriber("/stereo_msgs/StereoImage", stereo_msgs.StereoImage._TYPE);

		if( remaps.containsKey("__commitRate") )
			commitRate = Integer.parseInt(remaps.get("__commitRate"));

		SynchronizedThreadManager.getInstance().spin(new Runnable() {
			@Override
			public void run() {
				if(DEBUG)
					System.out.println("Entering storage loop");
				while(shouldStore) {
					try {
						TimedImage ti = queue.takeFirst();
						CompletableFuture<Relation> r = session.store(xid,System.currentTimeMillis(),sequenceNumber,NoIndex.create(ti));
						r.get();
					} catch (InterruptedException | ExecutionException e) {
						e.printStackTrace();
					}
					if(commitCnt.get() >= commitRate) {
						commitCnt.set(0);
						System.out.println("Committing at sequence "+sequenceNumber);
						session.commit(xid);
						if(MAXIMUM > 0 && sequenceNumber >= MAXIMUM)
							shouldStore = false;
					}
				}
			}
		}, "VIDEORECORDER");

		/**
		 * Image extraction from bus, then image processing, then on to display section.
		 */
		imgsub.addMessageListener(new MessageListener<stereo_msgs.StereoImage>() {
			@Override
			public void onNewMessage(stereo_msgs.StereoImage img) {
				long slew = System.currentTimeMillis() - time1;
				if( SAMPLERATE && slew >= 1000) {
					time1 = System.currentTimeMillis();
					System.out.println("Frames per second:"+(sequenceNumber-lastSequenceNumber)+" Storing:"+shouldStore+". Slew rate="+(slew-1000)+" seq:"+sequenceNumber);
					lastSequenceNumber = sequenceNumber;
				}
				if(sequenceNumber < MAXIMUM) {
					TimedImage ti = new TimedImage(img,System.nanoTime());
					queue.addLast(ti);
					commitCnt.getAndIncrement();
					++sequenceNumber; // we want to inc seq regardless to see how many we drop	
				} else {
					System.out.println("Storage maximum reached...");
					System.exit(0);
				}
			}
		});

	}
	
}

