package com.neocoretechs.robocore.video;

import java.io.IOException;
import java.nio.ByteBuffer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.relatrix.client.asynch.AsynchRelatrixClientTransaction;
import com.neocoretechs.relatrix.key.NoIndex;

import com.neocoretechs.rknn4j.image.Instance;
import com.neocoretechs.rknn4j.image.detect_result;
import com.neocoretechs.rknn4j.image.detect_result.Rectangle;
import com.neocoretechs.rknn4j.image.detect_result_group;
import com.neocoretechs.rknn4j.runtime.Model;

import com.neocoretechs.robocore.SynchronizedFixedThreadPoolManager;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;

import com.neocoretechs.rocksack.Alias;
import com.neocoretechs.rocksack.TransactionId;

import stereo_msgs.StereoImage;

/**
 * Perform a series of data acquisition and sensor fusion operations by acting as subscriber to the topics of
 * /stereo_msgs/ObjectDetect, /stereo_msgs/Imu and /sereo_msgs/StereoImage <p>
 * Once fused, data is passed to the model runner for evaluation via the publishing loop.
 * The internal TimedImage class performs a toJson on its images, timestamp and eulers angles.
 * @author Jonathan Groff (C) NeoCoreTechs 2021,2025
 *
 */
public class VideoObjectRecog extends AbstractNodeMain 
{
	private static boolean DEBUG = false;
	private static boolean DEBUGDIFF = false;
	private static boolean DEBUGJSON = true;
	private static boolean DEBUGDISPARITY = true;
	private static boolean SAVE_DETECTIONS = false;
	private static final boolean SAMPLERATE = true; // display pubs per second
	AsynchRelatrixClientTransaction session = null;
	TransactionId xid = null;
	Alias tensorAlias = null;
    ByteBuffer cbl;
    byte[] bufferl = new byte[0];
    ByteBuffer cbr;
    byte[] bufferr = new byte[0];
    int[] prevBuffer = new int[0];
    static boolean imageReadyL = false;
    static boolean imageReadyR = false;
   
	String outDir = "/";
	int frames = 0;
    CircularBlockingDeque<TimedImage> queue = new CircularBlockingDeque<TimedImage>(30);
  
	public long sequenceNumber = 0;
	public long sequenceNumber2 = 0;
	private long lastSequenceNumber;
	private long lastSequenceNumber2;
	long time1 = System.currentTimeMillis();
	long time2 = System.currentTimeMillis();
	protected static boolean shouldStore = false;
	int commitRate = 500;
	
	// NPU constants
	long ctx; // context for NPU
	private static String MODELS_FILE[] = {"yolov5s-640-640.rknn","yolo11s.rknn","ssd_inception_v2.rknn"};
	private static String LABELS_FILE[] = {"coco_80_labels_list.txt","coco_80_labels_list.txt","coco_labels_list.txt"};
	static int MODEL = 1;
	static String MODEL_FILE;
	static String LABEL_FILE;
	boolean wantFloat = false;
	static boolean RESIZE_TO_ORIG = false; // resize model image back to original cam capture size, false for YOLO, true for Inception
	
	int[] dimsImage = new int[] {640,480};
 
	String[] labels = null;
	static String MODEL_DIR = "/etc/model/RK3588/";
	//
	Model model = new Model();

	// InceptSSD specific constants
	float[][] boxPriors;
	//
	// z = f*(B/xl-xr)
	final static float f = 3.7f; // focal length mm
	final static float B = 100.0f; // baseline mm
	final static float IOU_THRESHOLD = .40f;
	
	double eulers[] = new double[]{0.0,0.0,0.0}; // set from loop
	long eulerTime = 0L;
	
	Publisher<stereo_msgs.StereoImage> imgpubstore = null;
	String detectAndStore = null; // look for particular object and send to storage channel
	
	static String threadGroup = "VIDEOCLIENT";
	static {
		SynchronizedFixedThreadPoolManager.init(2, Integer.MAX_VALUE, new String[] {threadGroup});
	}
	
	final class TimedImage implements Comparable {
		long sequence;
		long time;
		byte[] leftImage;
		byte[] rightImage;
		double eulersCorrelated[] = new double[]{0.0,0.0,0.0};
		public TimedImage(byte[] leftImage, byte[] rightImage, long sequence) {
			this.leftImage = leftImage;
			this.rightImage = rightImage;
			this.sequence = sequence;
			this.time = System.currentTimeMillis();
			if(Math.abs(this.time-eulerTime) <= 1000)
				synchronized(eulers) {
					this.eulersCorrelated = eulers;
				}
		}
		public String toJson() throws IOException{
			Instance rimage = createImage(rightImage);
			detect_result_group rdrg = model.inference(rimage, MODEL);
			imageReadyR = true;
			Instance limage = createImage(leftImage);
			detect_result_group ldrg = model.inference(limage, MODEL);
			imageReadyL = true;
			if(ldrg.getCount() == 0 && rdrg.getCount() == 0)
				return null;
			if(ldrg.getCount() > 0 && SAVE_DETECTIONS)
				limage.saveDetections(ldrg,"leftImage"+time);
			if(rdrg.getCount() > 0 && SAVE_DETECTIONS)
				rimage.saveDetections(rdrg,"rightimage"+time);
			if(ldrg.getCount() != 0 && rdrg.getCount() != 0) {
				detect_result[] d1 = ldrg.getResults();
				detect_result[] d2 = rdrg.getResults();
				// blocks of 8 coords overlapping xmin,ymin, xmax, ymax
				Object[] regions = correlateRegions(d1, d2);
				for(int i = 0; i < regions.length; i+=2) {
					detect_result dl = d1[(int)regions[i]];
					detect_result dr = d2[(int)regions[i+1]];
					//float lmin = xlmin-xrmin;
					//float lmax = xlmax-xrmax;
					float lmin = Math.abs(dl.getBox().xmin-dr.getBox().xmin);
					float lmax = Math.abs(dl.getBox().xmax-dr.getBox().xmax);
					float z1 = (f*B)/(f*(B/lmin));
					float z2 = (f*B)/(f*(B/lmax));
					// z= (f*B)/d
					if(DEBUG || DEBUGDISPARITY)
						System.out.println("Correlation left="+dl.getBox()+" right="+dr.getBox()+" disparity:"+z1+","+z2);
				}
			}
			String slbuf = ldrg.toJson();
			String srbuf = rdrg.toJson();
			StringBuilder sb = new StringBuilder("{\r\n");
			sb.append("\"timestamp\":");
			sb.append(time);
			sb.append(",\r\n");
			if(eulersCorrelated[0] != 0) {
				sb.append("\"imu\":{\"heading\":");
				sb.append(eulersCorrelated[0]);
				sb.append(",\"roll\":");
				sb.append(eulersCorrelated[1]);
				sb.append(",\"pitch\":");
				sb.append(eulersCorrelated[2]);
				sb.append("},\r\n");
			}
			sb.append("\"LeftImage\":[");
			sb.append(slbuf);
			sb.append("\r\n],\r\n\"RightImage\":[");
			sb.append(srbuf);
			sb.append("\r\n]");
			return sb.toString();
		}
		@Override
		public int compareTo(Object o) {
			if(time > ((TimedImage)o).time)
				return 1;
			if(time < ((TimedImage)o).time)
				return -1;
			return 0;
		}
		@Override
		public int hashCode() {
			final int prime = 31;
			int result = 1;
			result = prime * result + getEnclosingInstance().hashCode();
			result = prime * result + Objects.hash(time);
			return result;
		}
		@Override
		public boolean equals(Object obj) {
			if (this == obj) {
				return true;
			}
			if (!(obj instanceof TimedImage)) {
				return false;
			}
			TimedImage other = (TimedImage) obj;
			if (!getEnclosingInstance().equals(other.getEnclosingInstance())) {
				return false;
			}
			return time == other.time;
		}
		private VideoObjectRecog getEnclosingInstance() {
			return VideoObjectRecog.this;
		}
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_storevideoclient");
	}
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		MODEL_FILE = MODELS_FILE[MODEL];
		LABEL_FILE = LABELS_FILE[MODEL];
		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		if( remaps.containsKey("__commitRate") )
			commitRate = Integer.parseInt(remaps.get("__commitRate"));
		if( remaps.containsKey("__modelDir") )
			MODEL_DIR = remaps.get("__modelDir");
		if( remaps.containsKey("__labelsFile") )
			LABEL_FILE = remaps.get("__labelsFile");
		if( remaps.containsKey("__modelFile") )
			MODEL_FILE = remaps.get("__modelFile");
		if(!MODEL_DIR.endsWith(("/")))
				MODEL_DIR += "/";
		if( remaps.containsKey("__storeDetect") )
			detectAndStore = remaps.get("__storeDetect");
		try {
			if(MODEL_FILE.contains("inception")) { // InceptionSSD
				boxPriors = Model.loadBoxPriors(MODEL_DIR+"box_priors.txt",detect_result.NUM_RESULTS);
			}
			labels = Model.loadLines(MODEL_DIR+LABEL_FILE);
			if(DEBUG)
				System.out.println("Total category labels="+labels.length);
			//initialize the NPU with the proper model file, labels, and box priors or null
			model.initNPU(MODEL_DIR+MODEL_FILE, labels, boxPriors);
		} catch (IOException e2) {
			throw new RuntimeException(e2);
		}
		try {
			session = connectedNode.getRelatrixClient();
			//dbClient.setTablespace("D:/etc/Relatrix/db/test/ai");
			//try {
				xid = session.getTransactionId();
			//} catch (IllegalAccessException | ClassNotFoundException e) {}
			//tensorAlias = new Alias("Tensors");
			//try {
			//	if(dbClient.getAlias(tensorAlias).get() == null)
			//		dbClient.setRelativeAlias(tensorAlias);
			//} catch(ExecutionException | InterruptedException ie) {}
			if(DEBUG)
				System.out.println("Relatrix transaction Id:"+xid);
		} catch(IOException ioe) {
			ioe.printStackTrace();
		}
		final Subscriber<sensor_msgs.Imu> subsimu = 
				connectedNode.newSubscriber("/sensor_msgs/Imu", sensor_msgs.Imu._TYPE);
		final Subscriber<stereo_msgs.StereoImage> imgsub =
				connectedNode.newSubscriber("/stereo_msgs/StereoImage", stereo_msgs.StereoImage._TYPE);
		
		final Publisher<stereo_msgs.StereoImage> imgpub =
				connectedNode.newPublisher("/stereo_msgs/ObjectDetect", stereo_msgs.StereoImage._TYPE);

		/**
		 * Image extraction from bus from StereoImage topic, then image processing, then pass to pipeline.
		 */
		imgsub.addMessageListener(new MessageListener<stereo_msgs.StereoImage>() {
			@Override
			public void onNewMessage(stereo_msgs.StereoImage img) {
				long slew = System.currentTimeMillis() - time2;
				if( SAMPLERATE && slew >= 1000) {
					time2 = System.currentTimeMillis();
					System.out.println("Input Frames per second:"+(sequenceNumber2-lastSequenceNumber2)+" Storing:"+shouldStore+". Slew rate="+(slew-1000));
					lastSequenceNumber2 = sequenceNumber2;
				}	
				ByteBuffer cbl = img.getData();
				byte[] bufferl = cbl.array(); // 3 byte BGR
				ByteBuffer cbr = img.getData2();
				byte[] bufferr = cbr.array(); // 3 byte BGR
				// sequenceNumber is the publishing number, it may repeat here if more images come in than are published
				try {
					queue.addLastWait(new TimedImage(bufferl, bufferr, sequenceNumber));
				} catch (InterruptedException e) {}
				++sequenceNumber2; // we want to inc seq regardless to see how many we drop	
			}
		});
		//
		// update all TimedImage in the queue that match the current timestamp to 1 ms with the current
		// IMU reading
		//
		subsimu.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
			@Override
			public void onNewMessage(sensor_msgs.Imu message) {
				synchronized(eulers) {
					eulers = message.getOrientationCovariance();
					eulerTime = System.currentTimeMillis();
					//queue.forEach(e->{
					//	e.setIMU(currentTime, eulers);
					//});
				}
			}
		});

	/**
	 * Main publishing loop. Essentially we are publishing the data in whatever state its in, using the
	 * mutex appropriate to establish critical sections. A sleep follows each publication to keep the bus arbitrated
	 * This CancellableLoop will be canceled automatically when the node shuts down
	 */
	connectedNode.executeCancellableLoop(new CancellableLoop() {
		private int sequenceNumber;
		@Override
		protected void setup() {
			sequenceNumber = 0;
		}

		@Override
		protected void loop() throws InterruptedException {
			std_msgs.Header imghead = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
			imghead.setSeq(sequenceNumber);
			Time tst = connectedNode.getCurrentTime();
			imghead.setStamp(tst);
			imghead.setFrameId(tst.toString());
			//sensor_msgs.Image imagemess = imgpub.newMessage();
			//stereo_msgs.StereoImage imagemess = imgpub.newMessage();
			stereo_msgs.StereoImage imagemess = connectedNode.getTopicMessageFactory().newFromType(stereo_msgs.StereoImage._TYPE);

			TimedImage e = queue.takeFirstNotify();
			try {
				if( SAMPLERATE && System.currentTimeMillis() - time1 >= 1000) {
					time1 = System.currentTimeMillis();
					System.out.println("Output frames per second:"+(sequenceNumber-lastSequenceNumber)/*+limage+" "+rimage*/);
					lastSequenceNumber = sequenceNumber;
				}
				if( DEBUG )
					System.out.println(sequenceNumber+":Added frame ");
				//synchronized(mutex) {
				//leftPayload = limage.detectionsToJPEGBytes(ldrg);
				//rightPayload = rimage.detectionsToJPEGBytes(rdrg);
				//storeImage(ldrg, rdrg, leftPayload, rightPayload, imagemess, sequenceNumber);
				//if(leftPayload != null && rightPayload != null) {
				// for image resize back to 640x480
				//if(RESIZE_TO_ORIG) {
				//try {
				//	leftPayload = Instance.resizeRawJPEG(leftPayload,widthHeightChannel[0],widthHeightChannel[1],widthHeightChannel[2],dimsImage[0],dimsImage[1]);
				//	rightPayload = Instance.resizeRawJPEG(rightPayload,widthHeightChannel[0],widthHeightChannel[1],widthHeightChannel[2],dimsImage[0],dimsImage[1]);
				//} catch(Exception e) {
				//	e.printStackTrace();
				//}
				//}
				String toJSON = e.toJson();
				// Did we detect anything in either image?
				if(toJSON != null) {
					if(DEBUG || DEBUGJSON)
						System.out.println(toJSON);
					imagemess.setData(ByteBuffer.wrap(toJSON.getBytes()));
					//imagemess.setEncoding("JPG");
					imagemess.setEncoding("UTF8_JSON");
					imagemess.setWidth(dimsImage[0]);
					imagemess.setHeight(dimsImage[1]);
					imagemess.setStep(dimsImage[0]);
					imagemess.setIsBigendian((byte)0);
					imagemess.setHeader(imghead);
					imgpub.publish(imagemess);
				}	
				//
				imageReadyL = false;
				imageReadyR = false;
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
			} catch (IOException ex) {
				ex.printStackTrace();
				if(DEBUG)
					System.out.println("Image(s) not ready "+sequenceNumber);
				try {
					Thread.sleep(1);
				} catch (InterruptedException e1) {}
				++lastSequenceNumber; // if no good, up the last sequence to compensate for sequence increment
			}
			++sequenceNumber; // we want to inc seq regardless to see how many we drop	
		}
	});
	}

	/**
	 * Correlate all left boxes to right boxes, if there is overlap of better than threshold, return coords in out array
	 * called by storeImage
	 * @param lbox left detection result array
	 * @param rbox right detection result array
	 * @return Object array of Integer of correlated detect_results for N result N(lbox[index]) = left N+1(rbox[index]) = right for overlap regardless of category
	 */
	Object[] correlateRegions(detect_result[] lbox, detect_result[] rbox) {
		float[][] ious = new float[lbox.length][rbox.length];
		for(int i = 0; i < lbox.length; i++) {
			for(int j = 0; j < rbox.length; j++) {
				ious[i][j] = calculateOverlap(lbox[i].getBox().xmin,lbox[i].getBox().ymin,lbox[i].getBox().xmax,lbox[i].getBox().ymax,
					rbox[j].getBox().xmin,rbox[j].getBox().ymin,rbox[j].getBox().xmax,rbox[j].getBox().ymax);
			}
		}
		if(DEBUG)
			for(int k = 0; k < ious.length; k++)
				System.out.println("IOU["+k+"]="+Arrays.toString(ious[k]));
		ArrayList<Integer> outBoxes = new ArrayList<Integer>();
		// iterate 2d array of left box to all right overlaps
		for(int k = 0; k < lbox.length; k++) {
			float bestIou = 0.0f;
			int bestNum = 0;
			for(int m = 0; m < rbox.length; m++) {
				if(ious[k][m] > IOU_THRESHOLD && ious[k][m] > bestIou) {
					bestIou = ious[k][m];
					bestNum = m;
				}
			}
			if(DEBUG)
				System.out.println("left box "+k+" BestIou="+bestIou+" bestNum="+bestNum);
			// did we get one above threshold?
			if(bestIou > 0.0f) {
				// put left box in output array
				outBoxes.add(k);
				outBoxes.add(bestNum);
			}
		}
		return outBoxes.toArray();
	}
	/**
	 * Perform the intersection over union operation IoU
	 * @param xmin0 minx region 1
	 * @param ymin0 miny region 1
	 * @param xmax0 maxx region 1
	 * @param ymax0 maxy region 1
	 * @param xmin1 minx region 2
	 * @param ymin1 miny region 2
	 * @param xmax1 maxx region 2
	 * @param ymax1 maxy region 2
	 * @return intersection amount divided by union amount of the 2 regions
	 */
	static float calculateOverlap(float xmin0, float ymin0, float xmax0, float ymax0, float xmin1, float ymin1, float xmax1, float ymax1) {
		float w = Math.max(0.0f, Math.min(xmax0, xmax1) - Math.max(xmin0, xmin1));
		float h = Math.max(0.0f, Math.min(ymax0, ymax1) - Math.max(ymin0, ymin1));
		float i = w * h;
		float u = (xmax0 - xmin0) * (ymax0 - ymin0) + (xmax1 - xmin1) * (ymax1 - ymin1) - i;
		return (u <= 0.f ? 0.f : (i / u));
	}
	
	public void storeImage(detect_result_group ldrg, detect_result_group rdrg, byte[] leftPayload, byte[] rightPayload, StereoImage imagemess, Integer sequenceNumber) {
		//
		// see if we send this detection to storage channel as well
		//
		//boolean sendl = false;
		//boolean sendr = false;
		//float areal = 0.0f;
		//float arear = 0.0f;
		//float xlmin = 0.0f;
		//float xlmax = 0.0f;
		//float xrmin = 0.0f;
		//float xrmax = 0.0f;
		if(imgpubstore != null) {
			for(detect_result dr: ldrg.getResults()) {
				if(dr.getName().toLowerCase().startsWith(detectAndStore)) {
					float prob = dr.getProbability();
					//xlmin = dr.getBox().xmin;
					//xlmax = dr.getBox().xmax;
					//areal = (dr.getBox().ymax-dr.getBox().ymin) * (xlmax-xlmin);
					//if(prob > .4) {
						//sendl = true;
					if(DEBUG)
						System.out.println("L Detected "+dr.getName().toLowerCase()+" using "+detectAndStore+" %="+prob);
						//break;
					//}
				}
			}
			if(ldrg.getCount() > 0) { // artifact?
				for(detect_result dr: rdrg.getResults()) {
					if(dr.getName().toLowerCase().startsWith(detectAndStore)) {
						float prob = dr.getProbability();
						//xrmin = dr.getBox().xmin;
						//xrmax = dr.getBox().xmax;
						//arear = (dr.getBox().ymax-dr.getBox().ymin) * (xrmax-xrmin);
						//if(prob > .4) {
							//sendr = true;
						if(DEBUG)
							System.out.println("R Detected "+dr.getName().toLowerCase()+" using "+detectAndStore+" %="+prob);
							//break;
						//}
					}
				}
			}
			//if(sendl && sendr)
			//	System.out.println("person "+areal+" "+arear+" "+Math.abs(areal-arear));
			//if(sendl && sendr && Math.abs(areal-arear) < (.1f * areal)) {
			if(ldrg.getCount() > 0 && rdrg.getCount() > 0) { // possible correlation
				imgpubstore.publish(imagemess);
				if( DEBUG )
					System.out.println("Pub. Image:"+sequenceNumber);
				if(detectAndStore != null) {
					List<byte[]> sib = new ArrayList<byte[]>();
					sib.add(leftPayload);
					sib.add(rightPayload);
					session.store(xid, Long.valueOf(System.currentTimeMillis()), Integer.valueOf(sequenceNumber), NoIndex.create(sib));
				}
			}
		}
	}
	/**
	 * Generate Instance from raw JPEG image buffer with RGA
	 * @param imgBuff
	 * @return
	 * @throws IOException
	 */
	Instance createImage(byte[] imgBuff) throws IOException {
	  	return new Instance("img",dimsImage[0],dimsImage[1],model.getChannel(),imgBuff,model.getWidth(),model.getHeight(),"img",(MODEL != 2)); // not INCEPTION_SSD
	}
	/**
	 * Translate the image Instance and detect_result_group into raw JPEG byte array
	 * @param bImage
	 * @param group
	 * @return
	 * @throws IOException
	 */
	byte[] convertImage(Instance bImage, detect_result_group group) throws IOException {
		return bImage.detectionsToJPEGBytes(group);
	}
	

}

