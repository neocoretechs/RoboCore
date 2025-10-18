package com.neocoretechs.robocore.test;

import java.io.IOException;
import java.net.InetAddress;
import java.nio.ByteBuffer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ExecutionException;

import org.json.JSONObject;

import com.neocoretechs.relatrix.Result;
import com.neocoretechs.relatrix.client.asynch.AsynchRelatrixClientTransaction;
import com.neocoretechs.relatrix.key.NoIndex;

import com.neocoretechs.rknn4j.image.Instance;
import com.neocoretechs.rknn4j.image.detect_result;
import com.neocoretechs.rknn4j.image.detect_result_group;
import com.neocoretechs.rknn4j.runtime.Model;

import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.robocore.video.TimedImage;
import com.neocoretechs.rocksack.Alias;
import com.neocoretechs.rocksack.TransactionId;

/**
 * Perform a series of data acquisition and sensor fusion operations by acting as subscriber to the topics of
 * /stereo_msgs/ObjectDetect, /stereo_msgs/Imu and /sereo_msgs/StereoImage <p>
 * Once fused, data is passed to the model runner for evaluation via the publishing loop.
 * The internal TimedImage class performs a toJson on its images, timestamp and eulers angles.
 * @author Jonathan Groff (C) NeoCoreTechs 2021,2025
 *
 */
public class RecogTest 
{
	private static boolean DEBUG = false;
	private static boolean DEBUGDIFF = false;
	private static boolean DEBUGJSON = true;
	private static boolean DEBUGDISPARITY = true;
	private static boolean SAVE_DETECTIONS = false;
	private static final int SAMPLERATE = 5; // display pubs per SAMPLERATE if > 0
	AsynchRelatrixClientTransaction session = null;
	TransactionId xid = null;
	Alias tensorAlias = null;
    ByteBuffer cbl;
    byte[] bufferl = new byte[0];
    ByteBuffer cbr;
    byte[] bufferr = new byte[0];
    
    byte[] prevBufferl = null;
    byte[] prevBufferr = null;
    
    static boolean imageReadyL = false;
    static boolean imageReadyR = false;
   
	String outDir = "/";
	int frames = 0;
 
  
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
	final static float IMAGE_DIFFERENCE_PCT = .15f;
	
	
	String detectAndStore = null; // look for particular object and send to storage channel
	String previousJSON = "";

	CircularBlockingDeque<TimedImage2> timedImageDebounce = new CircularBlockingDeque<TimedImage2>(5);
	
	final class TimedImage2 implements Comparable {
		long sequence;
		long time;
		byte[] leftImage;
		byte[] rightImage;
		double eulersCorrelated[] = new double[]{0.0,0.0,0.0};
		float rangeCorrelated = 0f;
		stereo_msgs.StereoImage stereoImage;
		detect_result_group rdrg;
		detect_result_group ldrg;
		boolean skipSegLeft = false;
		boolean skipSegRight = false;
		public TimedImage2(byte[] leftImage, byte[] rightImage, long sequence) {
			this.leftImage = leftImage;
			this.rightImage = rightImage;
			this.time = System.currentTimeMillis();
			this.sequence = sequence;
			synchronized(model) {
				try {
					Instance rimage = createImage(rightImage);
					if(prevBufferr == null)
						prevBufferr = rimage.getImageByteArray();
					if(rimage.shouldSegment(prevBufferr, IMAGE_DIFFERENCE_PCT)) {
						prevBufferr = rimage.getImageByteArray();
						rdrg = model.inference(rimage, MODEL);
						imageReadyR = true;
						if(rdrg.getCount() > 0 && SAVE_DETECTIONS && debounce(this))
							rimage.saveDetections(rdrg,"rightimage"+time);
						skipSegRight = false;
					} else
						skipSegRight = true;

					Instance limage = createImage(leftImage);
					if(prevBufferl == null)
						prevBufferl = limage.getImageByteArray();
					if(limage.shouldSegment(prevBufferl, IMAGE_DIFFERENCE_PCT)) {
						prevBufferl = limage.getImageByteArray();
						ldrg = model.inference(limage, MODEL);
						imageReadyL = true;
						if(ldrg.getCount() > 0 && SAVE_DETECTIONS && debounce(this))
							limage.saveDetections(ldrg,"leftImage"+time);
						skipSegLeft = false;
					} else
						skipSegLeft = true;
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
		
		public String toJson() throws IOException{
			String slbuf = new String();
			String srbuf = new String();
			JSONObject jo = new JSONObject();
			synchronized(model) {
				if((skipSegLeft || ldrg.getCount() == 0) && (skipSegRight || rdrg.getCount() == 0)) {
					return null;
				}
				slbuf = ldrg.toJson();
				srbuf = rdrg.toJson();
				jo.put("timestamp",time);
				if(ldrg.getCount() == 0 && rdrg.getCount() == 0) {
					return jo.toString();
				}
			}
			/*
			if(ldrg.getCount() != 0 && rdrg.getCount() != 0) {
				detect_result[] d1 = ldrg.getResults();
				detect_result[] d2 = rdrg.getResults();
				//detect overlapping xmin,ymin, xmax, ymax
				Object[] regions = correlateRegions(d1, d2);
				for(int i = 0; i < regions.length; i+=2) {
					detect_result dl = d1[(int)regions[i]];
					detect_result dr = d2[(int)regions[i+1]];
					//float lmin = xlmin-xrmin;
					//float lmax = xlmax-xrmax;
					float lmin = Math.abs(dl.getBox().xmin-dr.getBox().xmin);
					float lmax = Math.abs(dl.getBox().xmax-dr.getBox().xmax);
					float z1 = (f*B)/lmin; //(f*(B/lmin));
					float z2 = (f*B)/lmax; //(f*(B/lmax));
					// z= (f*B)/d
					if(DEBUG || DEBUGDISPARITY)
						System.out.println("Correlation left="+dl.getBox()+" right="+dr.getBox()+" disparity:"+z1+","+z2);
				}
			}
			*/
			String preJson = jo.toString();
			StringBuilder sb = new StringBuilder(preJson.substring(0,preJson.length()-1)+",");
			
			sb.append("\"LeftImage\":[");
			sb.append(slbuf);
			sb.append("\r\n],\r\n\"RightImage\":[");
			sb.append(srbuf);
			sb.append("\r\n]");
			sb.append("}\r\n");
			return sb.toString();
		}
		@Override
		public int compareTo(Object o) {
			if(time > ((TimedImage2)o).time)
				return 1;
			if(time < ((TimedImage2)o).time)
				return -1;
			return 0;
		}
		@Override
		public int hashCode() {
			final int prime = 31;
			int result = 1;
			synchronized(model) {
				result = prime * result + Objects.hash(ldrg, rangeCorrelated, rdrg);
			}
			return result;
		}
		/**
		 * We assume we tossed out anything with zero in both detections
		 * If the left number matches the right number and each occurrence of name in left and right match, they are 'equal'
		 */
		@Override
		public boolean equals(Object obj) {
			if (this == obj) {
				return true;
			}
			if (!(obj instanceof TimedImage2)) {
				return false;
			}
			int leftSame = 0;
			int rightSame = 0;
			int leftCount = 0;
			int rightCount = 0;
			synchronized(model) {
				if(ldrg != null && ((TimedImage2)obj).ldrg != null && ldrg.getCount() == ((TimedImage2)obj).ldrg.getCount()) {
					leftCount = ldrg.getCount();
					for(int i = 0; i < leftCount;i++) {
						if(ldrg.getResults() == null)
							break;
						for(int j = 0; j <((TimedImage2)obj).ldrg.getCount(); j++ ) {
							if(((TimedImage2)obj).ldrg.getResults() == null)
								break;
							if(ldrg.getResults()[i].getName().equals(((TimedImage2)obj).ldrg.getResults()[j].getName())) {
								leftSame++;
								break;
							}
						}
					}
				}
				if(rdrg != null && ((TimedImage2)obj).rdrg != null && rdrg.getCount() == ((TimedImage2)obj).rdrg.getCount()) {
					rightCount = rdrg.getCount();
					for(int i = 0; i < rightCount;i++) {
						if(rdrg.getResults() == null)
							break;
						for(int j = 0; j <((TimedImage2)obj).rdrg.getCount(); j++ ) {
							if(((TimedImage2)obj).rdrg.getResults() == null)
								break;
							if(rdrg.getResults()[i].getName().equals(((TimedImage2)obj).rdrg.getResults()[j].getName())) {
								rightSame++;
								break;
							}
						}
					}
				}
			}
			if(leftCount == 0 && rightCount == 0)
				return false;
			if(leftSame == leftCount && rightSame == rightCount)
				return true;
			return false;
		}

	}
	
	private void process(TimedImage fromDb) {
		//long slew = System.currentTimeMillis() - time2;
		// inference in ctor
		TimedImage2 candidate = new TimedImage2(fromDb.leftImage, fromDb.rightImage, sequenceNumber2);
		if(debounce(candidate)) {
			try {
				System.out.println(sequenceNumber2+" = "+candidate.toJson());
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		++sequenceNumber2; // we want to inc seq regardless to see how many we drop	
	}

	public static void main(String[] args) {
		MODEL_FILE = MODELS_FILE[MODEL];
		LABEL_FILE = LABELS_FILE[MODEL];
		RecogTest rt = new RecogTest();
		try {
			if(MODEL_FILE.contains("inception")) { // InceptionSSD
				rt.boxPriors = Model.loadBoxPriors(MODEL_DIR+"box_priors.txt",detect_result.NUM_RESULTS);
			}
			rt.labels = Model.loadLines(MODEL_DIR+LABEL_FILE);
			if(DEBUG)
				System.out.println("Total category labels="+rt.labels.length);
			//initialize the NPU with the proper model file, labels, and box priors or null
			rt.model.initNPU(MODEL_DIR+MODEL_FILE, rt.labels, rt.boxPriors);
		} catch (IOException e2) {
			throw new RuntimeException(e2);
		}
		try {
			// attach to running RosCore with embedded Relatrix server
			rt.session = new AsynchRelatrixClientTransaction(args[0], args[1], 8092);
			rt.xid = rt.session.getTransactionId();
			//tensorAlias = new Alias("Tensors");
			//try {
			//	if(dbClient.getAlias(tensorAlias).get() == null)
			//		dbClient.setRelativeAlias(tensorAlias);
			//} catch(ExecutionException | InterruptedException ie) {}
			rt.session.findStream(rt.xid, '*', '*', '?').get().forEach(e->{
				rt.process((TimedImage) ((Result)e).get());
			});
			System.out.println("Relatrix transaction Id:"+rt.xid);
		} catch(IOException ioe) {
			ioe.printStackTrace();
		} catch (InterruptedException e) {
			System.exit(1);
		} catch (ExecutionException e) {
			e.printStackTrace();
		}
	}
	/**
	 * If we have a majority of matching candidates in the circular deque, certify it as a valid detect
	 * @param candidate the image under scrutiny
	 * @return true if 60% match, if true, queue is cleared
	 */
	private boolean debounce(TimedImage2 candidate) {
		int total = 0;
		// both zero? toss it
		synchronized(model) {
			if((!candidate.skipSegLeft && candidate.ldrg.getCount() != 0) || 
					(!candidate.skipSegRight && candidate.rdrg.getCount() != 0)) {
				Iterator<TimedImage2> it = timedImageDebounce.iterator();
				while(it.hasNext()) {
					TimedImage2 ti = it.next();
					if(ti.equals(candidate))
						++total;
				}
				if(((float)total/(float)timedImageDebounce.length()) >= .6f) {
					timedImageDebounce.clear();
					timedImageDebounce.addLast(candidate);
					return true;
				}
				timedImageDebounce.addLast(candidate);
			}
		}
		if(DEBUG)
			System.out.println("Debounce total match:"+total+" from queue length:"+timedImageDebounce.size());
		return false;
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

