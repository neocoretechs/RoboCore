package com.neocoretechs.robocore;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.time.LocalDate;
import java.time.LocalTime;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.List;

import javax.imageio.ImageIO;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.machinevision.hough3d.Vector4d;
import com.neocoretechs.machinevision.hough3d.hough_settings;
import com.neocoretechs.machinevision.hough3d.octree_t;
import com.neocoretechs.machinevision.hough3d.writer_file;
import com.neocoretechs.robocore.VideoProcessor.AbstractDepth;
import com.neocoretechs.robocore.VideoProcessor.envInterface;

import diagnostic_msgs.DiagnosticStatus;
import diagnostic_msgs.KeyValue;

/**
 * Listen for pointcloud messages
 * @author jg
 *
 */
public class PointCloudSubs extends AbstractNodeMain {
	private static boolean DEBUG = false;
	String outDir = "/users/jg/workspace/robocore/motionclouds";
	static int imageWidth, imageHeight;
	protected int fileNum = 0;
	final static int leftBound = -207;
	final static int rightBound = 207;
	protected static int files = 0;
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_pointcloud");
	}
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		//final Log log = connectedNode.getLog();
		Subscriber<stereo_msgs.DisparityImage2> subsbat = connectedNode.newSubscriber("stereo_msgs/PointCloud", stereo_msgs.DisparityImage2._TYPE);
	
		subsbat.addMessageListener(new MessageListener<stereo_msgs.DisparityImage2>() {
		@Override
		public void onNewMessage(stereo_msgs.DisparityImage2 message) {
				System.out.println();
				float[] disp = message.getD();
				int goodDisp = 0;
				for(int i = 0; i < disp.length; i++)
					if( disp[i] > 0 )
						++goodDisp ;
				System.out.println("Status "+goodDisp+" points of "+disp.length+" @ "+LocalDate.now()+" "+LocalTime.now()); //2016/11/16 12:08:43
				BufferedImage imageL1 = null;
				ByteBuffer cbL;
				byte[] bufferL, datax;
				// signal commencement processing by placing the last in queue
				try {
						cbL = message.getImage().getData();
						bufferL = cbL.array();
						InputStream in = new ByteArrayInputStream(bufferL);
						imageL1 = ImageIO.read(in);
						if( imageL1.getType() != BufferedImage.TYPE_3BYTE_BGR)
							System.out.println("NOT 3 byte BGR, but:"+imageL1.getType());
						imageWidth = imageL1.getWidth();
						imageHeight = imageL1.getHeight();
						datax = (byte[]) imageL1.getData().getDataElements(0, 0, imageWidth, imageHeight, null);
						writeFile(datax, disp, "/robotEye"+fileNum);
						++fileNum;
						in.close();
				} catch (IOException e1) {
					System.out.println("Could not convert image payload due to:"+e1.getMessage());
					return;
				}
		}
		});	
	}
	/**
	 * Write the pixels from source image at depth * 2
	 * @param d List of envInterface post-processed nodes data
	 * @param filename file to write to
	 * @param compVal if >= 0 the value to compare to depth to write conditionally
	 */
	protected void writeFile(byte[] data, float[] depth, String filename) {
		DataOutputStream dos = null;
		File f = new File(outDir+filename+".asc");
		try {
			dos = new DataOutputStream(new FileOutputStream(f));
			for(int i = 0; i < imageWidth; i++) {
				for(int j = 0; j < imageHeight; j++) {
					dos.writeBytes(String.valueOf(i)); // X
						dos.writeByte(' ');
						dos.writeBytes(String.valueOf(j));// Y
						dos.writeByte(' ');
						dos.writeBytes(String.valueOf(depth[(j*imageWidth)+i]*2)); // Z
						dos.writeByte(' ');
						dos.writeBytes(String.valueOf(data[(((j*imageWidth)+i)*3)+2])); // R
						dos.writeByte(' ');
						dos.writeBytes(String.valueOf(data[(((j*imageWidth)+i)*3)+1])); // G
						dos.writeByte(' ');
						dos.writeBytes(String.valueOf(data[((j*imageWidth)+i)*3])); // B
						dos.writeByte('\r');
						dos.writeByte('\n');					
				//writer_file.line3D(dos, (int)id.getEnv()[0]/*xmin*/, (int)id.getEnv()[1]/*ymin*/, (int)(id.getDepth()*10.0d), 
				//		(int)id.getEnv()[2]/*xmax*/, (int)id.getEnv()[1]/*ymin*/, (int)(id.getDepth()*10.0d), 0, 255, 255);
				//writer_file.line3D(dos, (int)id.getEnv()[2]/*xmax*/, (int)id.getEnv()[1]/*ymin*/, (int)(id.getDepth()*10.0d), 
				//		(int)id.getEnv()[2]/*xmax*/, (int)id.getEnv()[3]/*ymax*/, (int)(id.getDepth()*10.0d), 0, 255, 255);
				//writer_file.line3D(dos, (int)id.getEnv()[2]/*xmax*/, (int)id.getEnv()[3]/*ymax*/, (int)(id.getDepth()*10.0d), 
				//		(int)id.getEnv()[0]/*xmin*/, (int)id.getEnv()[3]/*ymax*/, (int)(id.getDepth()*10.0d), 0, 255, 255);
				//writer_file.line3D(dos, (int)id.getEnv()[0]/*xmin*/, (int)id.getEnv()[3]/*ymax*/, (int)(id.getDepth()*10.0d), 
				//		(int)id.getEnv()[0]/*xmin*/, (int)id.getEnv()[1]/*ymin*/, (int)(id.getDepth()*10.0d), 0, 255, 255);
				}		
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			return;
		} catch (IOException e) {
			e.printStackTrace();
		} finally {
			try {
				if( dos != null ) {
					dos.flush();
					dos.close();
				}
			} catch (IOException e) {
			}		
		}	
	}
	/**
	 * Generate the occupancy grid with minimum depth indicators into file seq
	 * from list of IndexMax elements
	 * @param nodes
	 */
	private static void genNav2(List<envInterface> nodes) {
		// descending sort mirrors forward direction of robot movement
		   //final Comparator<envInterface> yComp = new Comparator<envInterface>() {         
			//	  @Override         
			//	  public int compare(envInterface jc1, envInterface jc2) {             
			//		      return ((int)((IndexMax)jc2).node.getMiddle().y < (int)((IndexMax)jc1).node.getMiddle().y ? -1 :                     
			//		              ((int)((IndexMax)jc2).node.getMiddle().y == (int)((IndexMax)jc1).node.getMiddle().y ? 0 : 1));           
			//	  }     
		   //};
		   final Comparator<double[]> yComp = new Comparator<double[]>() {         
				  @Override         
				  public int compare(double[] jc1, double[] jc2) {             
					      return ((int)jc2[3] < (int)jc1[3] ? -1 :                     
					              ((int)jc2[3] == (int)jc1[3] ? 0 : 1));           
				  }     
		   };
		   final Comparator<envInterface> xComp = new Comparator<envInterface>() {         
				  @Override         
				  public int compare(envInterface jc1, envInterface jc2) {             
					      return ((int)((AbstractDepth)jc2).node.getMiddle().x < (int)((AbstractDepth)jc1).node.getMiddle().x ? -1 :                     
					              ((int)((AbstractDepth)jc2).node.getMiddle().x == (int)((AbstractDepth)jc1).node.getMiddle().x ? 0 : 1));           
				  }     
		   };
		   if(nodes.size() == 0)
			   return;
		   DataOutputStream dos = null;
		   File f = new File(hough_settings.file+"NavLine"+files+hough_settings.extension);
		   try {
			   dos = new DataOutputStream(new FileOutputStream(f));
			   // col and min depth marker
			   ArrayList<double[]> splinePts = new ArrayList<double[]>();
			   Collections.sort(nodes, xComp);
			   int x = (int)((AbstractDepth)nodes.get(0)).node.getMiddle().x;
			   int iPosStart = 0;
			   int iPosEnd = 0;
			   for(int i = 0 ; i < nodes.size(); i++) {
				   if( x != (int)((AbstractDepth)nodes.get(i)).node.getMiddle().x) {
					   iPosEnd = i;
					   genColHist2(dos, iPosStart, iPosEnd, nodes, splinePts);
					   iPosStart = i;
					   x = (int)((AbstractDepth)nodes.get(i)).node.getMiddle().x;
				   }
			   }
			   iPosEnd = nodes.size();
			   genColHist2(dos, iPosStart, iPosEnd, nodes, splinePts);
			   //if(splinePts.size() == 0)
				//   return;
			   // rows
			   //Collections.sort(nodes, yComp);
			   //int y = (int)((AbstractDepth)nodes.get(0)).node.getMiddle().y;
			   //iPosStart = 0;
			   //iPosEnd = 0;
			   //for(int i = 0 ; i < nodes.size(); i++) {
				//   if( y != (int)((AbstractDepth)nodes.get(i)).node.getMiddle().y) {
				//	   iPosEnd = i;
				//	   genRow2(dos, iPosStart, iPosEnd, nodes);
				//	   iPosStart = i;
				//	   y = (int)((AbstractDepth)nodes.get(i)).node.getMiddle().y;
				//   }
			   //}
			   //iPosEnd = nodes.size();
			   /*
			   Collections.sort(splinePts, yComp);
			   int y = (int)splinePts.get(0)[3]; //y
			   iPosStart = 0;
			   iPosEnd = 0;
			   for(int i = 0 ; i < splinePts.size(); i++) {
				   if( y != (int)splinePts.get(i)[3]) {
					   iPosEnd = i;
					   genRow2(dos, iPosStart, iPosEnd, splinePts);
					   iPosStart = i;
					   y = (int)splinePts.get(i)[3];
				   }
			   }
			   iPosEnd = splinePts.size();
			   genRow2(dos, iPosStart, iPosEnd, splinePts);
			   */
		   } catch (FileNotFoundException e) {
				e.printStackTrace();
				return;  
		  } catch (IOException e) {
			e.printStackTrace();
		  } finally {
				try {
					if( dos != null ) {
						dos.flush();
						dos.close();
					}
				} catch (IOException e) {}		
		  }
		   
	}

	public static void genRow2(DataOutputStream dos, int yStart, int yEnd, List<double[]> splinePts) throws IOException{
		//if(DEBUG)
			//System.out.println("genRow from="+yStart+" to="+yEnd);
			List<double[]> subNodes = splinePts.subList(yStart, yEnd);
			//final Comparator<envInterface> xComp = new Comparator<envInterface>() {         
			//		  @Override         
			//		  public int compare(envInterface jc1, envInterface jc2) {             
			//			      return ((int)((AbstractDepth)jc2).node.getMiddle().x < (int)((AbstractDepth)jc1).node.getMiddle().x ? -1 :                     
			//			              ((int)((AbstractDepth)jc2).node.getMiddle().x == (int)((AbstractDepth)jc1).node.getMiddle().x ? 0 : 1));           
			//		  }     
			//};
			final Comparator<double[]> xComp = new Comparator<double[]>() {         
					  @Override         
					  public int compare(double[] jc1, double[] jc2) {             
						      return ((int)jc2[0] < (int)jc1[0] ? -1 :                     
						              ((int)jc2[0] == (int)jc1[0] ? 0 : 1));           
					  }     
			};
			Collections.sort(subNodes, xComp);
			// sorted now by x and y
			double zMin = Double.MAX_VALUE;
			double[] cnode = null;
			for(int i = 0; i < subNodes.size(); i++) {
				//System.out.println("Row "+(int)subNodes.get(i)[0]+" "+(int)subNodes.get(i)[1]+" "+(int)subNodes.get(i)[2]);
				if( i < subNodes.size()-1) {
					//writer_file.line3D(dos, (int)((AbstractDepth)subNodes.get(i)).node.getCentroid().x, (int)((AbstractDepth)subNodes.get(i)).node.getCentroid().y, (int)((AbstractDepth)subNodes.get(i)).depth*10,
					//		(int)((AbstractDepth)subNodes.get(i+1)).node.getCentroid().x, (int)((AbstractDepth)subNodes.get(i+1)).node.getCentroid().y, (int)((AbstractDepth)subNodes.get(i+1)).depth*10,
					//		0, 255, 255);
					writer_file.line3D(dos, (int)subNodes.get(i)[0] , (int)subNodes.get(i)[1], (int)subNodes.get(i)[2], 
							(int)subNodes.get(i+1)[0], (int)subNodes.get(i+1)[1], (int)subNodes.get(i+1)[2], 0, 255, 255);
				}
				if((int)subNodes.get(i)[1] >= 0 && (int)subNodes.get(i)[0] >= leftBound && (int)subNodes.get(i)[0] <= rightBound &&
						subNodes.get(i)[2] <= zMin) {
					zMin = subNodes.get(i)[2];
					cnode = subNodes.get(i);
				}
			}
			//System.out.println("-----------");
			//  min depth
			if( cnode != null ) {
			Vector4d cen1 = new Vector4d();
			Vector4d cen2 = new Vector4d();
			cen1.x = cnode[0] - (cnode[4]/2);
			cen1.y = cnode[1] - (cnode[4]/2);
			cen1.z = zMin;
			cen2.x = cnode[0] + (cnode[4]/2);
			cen2.y = cnode[1] + (cnode[4]/2);
			cen2.z = zMin;
			//if( DEBUG )
			//	System.out.println("genColHist env minx,y,z="+cen1+" maxx,y,z="+cen2+" centroid node="+cnode);
			// xmin, ymin, xmax, ymin
			writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen1.y, (int)cen1.z, 255, 127, 0);
			// xmax, ymin, xmax, ymax
			writer_file.line3D(dos, (int)cen2.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 255, 127, 0);
			// xmax, ymax, xmin, ymax
			writer_file.line3D(dos, (int)cen2.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen2.y, (int)cen2.z, 255, 127, 0);
			// xmin, ymax, xmin, ymin
			writer_file.line3D(dos, (int)cen1.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 255, 127, 0);
			}
	}
	public static void genColHist2(DataOutputStream dos, int xStart, int xEnd, List<envInterface> nodelA, List<double[]> splinePts) throws IOException{
		//if(DEBUG)
			//System.out.println("genColHist from="+xStart+" to="+xEnd);
			List<envInterface> subNodes = nodelA.subList(xStart, xEnd);
			final Comparator<envInterface> yComp = new Comparator<envInterface>() {         
					  @Override         
					  public int compare(envInterface jc1, envInterface jc2) {             
						      return ((int)((AbstractDepth)jc2).node.getMiddle().y < (int)((AbstractDepth)jc1).node.getMiddle().y ? -1 :                     
						              ((int)((AbstractDepth)jc2).node.getMiddle().y == (int)((AbstractDepth)jc1).node.getMiddle().y ? 0 : 1));           
					  }     
			}; 
			Collections.sort(subNodes, yComp);
			/*
			ArrayList<double[]> dpit = new ArrayList<double[]>();
			if(SMOOTHEGRID) {
				PolyBezierPathUtil pbpu = new  PolyBezierPathUtil();
				ArrayList<PolyBezierPathUtil.EPointF> epfa = new ArrayList<PolyBezierPathUtil.EPointF>();
				for(int i = 0; i < subNodes.size(); i++) {
					PolyBezierPathUtil.EPointF epf = pbpu.new EPointF(((AbstractDepth)subNodes.get(i)).depth*10.0d, ((AbstractDepth)subNodes.get(i)).node.getMiddle().y);
					epfa.add(epf);
				}
				if(epfa.size() < 2)
					return;
				Path2D.Double path = pbpu.computePathThroughKnots(epfa);
				PathIterator pit = path.getPathIterator(null);
				while(!pit.isDone()) {
					double[] coords = new double[6];
					pit.currentSegment(coords);
					dpit.add(coords);
					pit.next();
				}
			} else {
				//---if we dont want to smooth
				for(int i = 0; i < subNodes.size(); i++) {
					double[] coords = new double[6];
					coords[0] = ((AbstractDepth)subNodes.get(i)).depth*10.0d;
					coords[1] = ((AbstractDepth)subNodes.get(i)).node.getMiddle().y;
					dpit.add(coords);
				}
			}
			*/
			double zMin = Double.MAX_VALUE;
			octree_t cnode = null;
			int cpos = -1;
			// sorted now by x and y
			for(int i = 0; i < subNodes.size(); i++) {
				//double[] splinep = dpit.get(i);
				//splinep[2] = splinep[0]; // rotate X back to Z
				//splinep[0] = ((AbstractDepth)subNodes.get(i)).node.getMiddle().x;// the x, then y is index 1 as originally
				//splinep[3] = ((AbstractDepth)subNodes.get(i)).node.getMiddle().y;// we need to sort the rows on this value
				//splinep[4] = ((AbstractDepth)subNodes.get(i)).node.getSize(); // to generate box
				//splinePts.add(splinep); // collect it in our splined array to make rows later
				//if( i < subNodes.size()-1) {
					//writer_file.line3D(dos, (int)((AbstractDepth)subNodes.get(i)).node.getCentroid().x, (int)((AbstractDepth)subNodes.get(i)).node.getCentroid().y, (int)((AbstractDepth)subNodes.get(i)).depth*10,
					//		(int)((AbstractDepth)subNodes.get(i+1)).node.getCentroid().x, (int)((AbstractDepth)subNodes.get(i+1)).node.getCentroid().y, (int)((AbstractDepth)subNodes.get(i+1)).depth*10,
					//		0, 255, 255);
					//writer_file.line3D(dos, (int)splinep[0] , (int)splinep[1], (int)splinep[2], 
					//		(int)((AbstractDepth)subNodes.get(i+1)).node.getMiddle().x, (int)dpit.get(i+1)[1], (int)dpit.get(i+1)[0], 0, 255, 255);
					//if( DEBUG ) 
					//	System.out.println("genColHist line3D x="+(int)((AbstractDepth)subNodes.get(i)).node.getCentroid().x+" y="+(int)((AbstractDepth)subNodes.get(i)).node.getCentroid().y+" z="+(int)((AbstractDepth)subNodes.get(i)).depth*10+
					//			" to x="+(int)((AbstractDepth)subNodes.get(i+1)).node.getCentroid().x+" y="+(int)((AbstractDepth)subNodes.get(i+1)).node.getCentroid().y+" z="+(int)((AbstractDepth)subNodes.get(i+1)).depth*10);
					// y remains the same and we rotate our depth 90 degrees to become the x axis
					// so we get an angular measurement thats perpendicular to frontal x,y plane
					// this corresponds to the theta angle we get from cartestian_to_spherical, for some reason in this
					// orientation, vs z axis vertical, the theta value is aligned along our depth, or z axis horizontal. 
					// So if we use spherical the phi and theta are swapped in this orientation.
					//double thet1 = Math.atan2( (((AbstractDepth)subNodes.get(i)).node.getCentroid().y-((AbstractDepth)subNodes.get(i+1)).node.getCentroid().y),
					//			((((AbstractDepth)subNodes.get(i)).depth-((AbstractDepth)subNodes.get(i+1)).depth)*10) );
					//double thet1 = Math.atan2( splinep[1]-dpit.get(i+1)[1], ((splinep[2]-dpit.get(i+1)[0])*10) );
					//double degThet = Math.toDegrees(thet1);
					//if( degThet < 0.0) {
					//	    degThet += 360.0;
					//}
					//Vector4d point = new Vector4d( (((AbstractDepth)subNodes.get(i)).node.getCentroid().x-((AbstractDepth)subNodes.get(i+1)).node.getCentroid().x),
					//		(((AbstractDepth)subNodes.get(i)).node.getCentroid().y-((AbstractDepth)subNodes.get(i+1)).node.getCentroid().y),
					//			((((AbstractDepth)subNodes.get(i)).depth-((AbstractDepth)subNodes.get(i+1)).depth)*10), 1);
					//double[] deg1 = octree_t.cartesian_to_spherical(point);
					//
					//System.out.println("Column "+(int)((AbstractDepth)subNodes.get(i)).node.getMiddle().x+" depth diff "+i+" degrees ="+degThet+" node="+((AbstractDepth)subNodes.get(i)).node);
				//}
				// since we sort descending to compute in the direction of robot motion, we want the highest numbered
				// row as minimum so we know the top of an obstacle right in front of us. To this end we use <= zMin
				// so identical depths will percolate to the top
				if((int)((AbstractDepth)subNodes.get(i)).node.getMiddle().y >= 0 && 
					//(int)splinep[0] >= leftBound && (int)splinep[0] <= rightBound &&
					(int)((AbstractDepth)subNodes.get(i)).node.getMiddle().x >= leftBound &&
					(int)((AbstractDepth)subNodes.get(i)).node.getMiddle().x <= rightBound &&
					((AbstractDepth)subNodes.get(i)).depth <= zMin) {
						zMin = ((AbstractDepth)subNodes.get(i)).depth;
						//cnode = ((AbstractDepth)subNodes.get(i)).node;
						cpos = i;
				}
			}
			//  min depth
			//if( cnode != null ) {
			if( cpos != -1 ) { // there may have been 0 elements
				cnode = ((AbstractDepth)subNodes.get(cpos)).node;
				//System.out.println("Zmin="+zMin+" cpos="+cpos+" node="+cnode); //delinate column depth display
				int isize = (int) cnode.getSize();
				Vector4d cen1 = new Vector4d();
				Vector4d cen2 = new Vector4d();
				cen1.x = cnode.getMiddle().x - (isize/2);
				cen1.y = cnode.getMiddle().y - (isize/2);
				cen1.z = 0;//zMin*10.0d;//cnode.getCentroid().z - (cnode.getSize()/2);
				cen2.x = cnode.getMiddle().x + (isize/2);
				cen2.y = cnode.getMiddle().y + (isize/2);
				cen2.z = 0;//zMin*10.0d;//cnode.getCentroid().z + (cnode.getSize()/2);
				while(cen1.y < (imageHeight/2)) {
					//if( DEBUG )
					//	System.out.println("genColHist env minx,y,z="+cen1+" maxx,y,z="+cen2+" centroid node="+cnode);
					// xmin, ymin, xmax, ymin
					writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen1.y, (int)cen1.z, 0, 127, 255);
					// xmax, ymin, xmax, ymax
					writer_file.line3D(dos, (int)cen2.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 0, 127, 255);
					// xmax, ymax, xmin, ymax
					writer_file.line3D(dos, (int)cen2.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen2.y, (int)cen2.z, 0, 127, 255);
					// xmin, ymax, xmin, ymin
					writer_file.line3D(dos, (int)cen1.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 0, 127, 255);
					// xmin, ymin, xmax, ymax
					writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 0, 127, 255);
					// xmax, ymin, xmin, ymax
					writer_file.line3D(dos, (int)cen2.x, (int)cen1.y, (int)cen1.z, (int)cen1.x, (int)cen2.y, (int)cen2.z, 0, 127, 255);
					cen1.y += isize;
					cen2.y += isize;
				}
			} //else
				//System.out.println("no elements");
	}
	/**
	 * Permutation that does not rely on octree node
	 * @param nodes
	 */
	public static void genNav(List<envInterface> nodes) {
		   final Comparator<envInterface> yComp = new Comparator<envInterface>() {         
				  @Override         
				  public int compare(envInterface jc1, envInterface jc2) {
					  int y1 = (int)(jc1.getEnv()[1]+((jc1.getEnv()[3]-jc1.getEnv()[1])/2));
					  int y2 = (int)(jc2.getEnv()[1]+((jc2.getEnv()[3]-jc2.getEnv()[1])/2));
					      return (y2 < y1 ? -1 : (y2 == y1 ? 0 : 1));           
				  }     
		   };
		   final Comparator<envInterface> xComp = new Comparator<envInterface>() {         
				  @Override         
				  public int compare(envInterface jc1, envInterface jc2) {
					  int x1 = (int)(jc1.getEnv()[0]+((jc1.getEnv()[2]-jc1.getEnv()[0])/2));
					  int x2 = (int)(jc2.getEnv()[0]+((jc2.getEnv()[2]-jc2.getEnv()[0])/2));
					      return (x2 < x1 ? -1 : (x2 == x1 ? 0 : 1));           
				  }     
		   };
		   Collections.sort(nodes, yComp);
		   DataOutputStream dos = null;
		   File f = new File(hough_settings.file+"NavLine"+files+hough_settings.extension);
		   try {
			   dos = new DataOutputStream(new FileOutputStream(f));
			   int y = (int)(nodes.get(0).getEnv()[1]+((nodes.get(0).getEnv()[3]-nodes.get(0).getEnv()[1])/2));
			   int iPosStart = 0;
			   int iPosEnd = 0;
			   for(int i = 0 ; i < nodes.size(); i++) {
				   if( y != (int)(nodes.get(i).getEnv()[1]+((nodes.get(i).getEnv()[3]-nodes.get(i).getEnv()[1])/2))) {
					   iPosEnd = i;
					   genRow(dos, iPosStart, iPosEnd, nodes);
					   iPosStart = i;
					   y = (int)(nodes.get(i).getEnv()[1]+((nodes.get(i).getEnv()[3]-nodes.get(i).getEnv()[1])/2));
				   }
			   }
			   iPosEnd = nodes.size();
			   genRow(dos, iPosStart, iPosEnd, nodes);
			   // col and min depth marker
			   Collections.sort(nodes, xComp);
			   int x = (int)(nodes.get(0).getEnv()[0]+((nodes.get(0).getEnv()[2]-nodes.get(0).getEnv()[0])/2));
			   iPosStart = 0;
			   iPosEnd = 0;
			   for(int i = 0 ; i < nodes.size(); i++) {
				   if( x != nodes.get(i).getEnv()[0]+((nodes.get(i).getEnv()[2]-nodes.get(i).getEnv()[0])/2)) {
					   iPosEnd = i;
					   genColHist(dos, iPosStart, iPosEnd, nodes);
					   iPosStart = i;
					   x =(int)(nodes.get(i).getEnv()[0]+((nodes.get(i).getEnv()[2]-nodes.get(i).getEnv()[0])/2));
				   }
			   }
			   iPosEnd = nodes.size();
			   genColHist(dos, iPosStart, iPosEnd, nodes);
		   } catch (FileNotFoundException e) {
				e.printStackTrace();
				return;  
		  } catch (IOException e) {
			e.printStackTrace();
		  } finally {
				try {
					if( dos != null ) {
						dos.flush();
						dos.close();
					}
				} catch (IOException e) {}		
		  }   
	}

	public static void genRow(DataOutputStream dos, int yStart, int yEnd, List<envInterface> nodelA) throws IOException{
		//if(DEBUG)
		//	System.out.println("genRow from="+yStart+" to="+yEnd);
			List<envInterface> subNodes = nodelA.subList(yStart, yEnd);
			final Comparator<envInterface> xComp = new Comparator<envInterface>() {         
					  @Override         
					  public int compare(envInterface jc1, envInterface jc2) {
						  int x1 = (int)(jc1.getEnv()[0]+((jc1.getEnv()[2]-jc1.getEnv()[0])/2));
						  int x2 = (int)(jc2.getEnv()[0]+((jc2.getEnv()[2]-jc2.getEnv()[0])/2));
						      return (x2 < x1 ? -1 : (x2 == x1 ? 0 : 1));           
					  }     
			};
			Collections.sort(subNodes, xComp);
			// sorted now by x and y
			for(int i = 0; i < subNodes.size(); i++) {
				if( i < subNodes.size()-1) {
					int ix0 = (int)subNodes.get(i).getEnv()[0]+(((int)subNodes.get(i).getEnv()[2]-(int)subNodes.get(i).getEnv()[0])/2);
					int iy0 = (int)subNodes.get(i).getEnv()[1]+(((int)subNodes.get(i).getEnv()[3]-(int)subNodes.get(i).getEnv()[1])/2);
					int iz0 = (int)(subNodes.get(i).getDepth()*10.0d);
					int ix1 = (int)subNodes.get(i+1).getEnv()[0]+(((int)subNodes.get(i+1).getEnv()[2]-(int)subNodes.get(i+1).getEnv()[0])/2);
					int iy1 = (int)subNodes.get(i+1).getEnv()[1]+(((int)subNodes.get(i+1).getEnv()[3]-(int)subNodes.get(i+1).getEnv()[1])/2);
					int iz1 = (int)(subNodes.get(i+1).getDepth()*10.0d);
					writer_file.line3D(dos, ix0, iy0, iz0, ix1, iy1, iz1, 0, 255, 255);
				}
			}
	}
	
	public static void genColHist(DataOutputStream dos, int xStart, int xEnd, List<envInterface> nodelA) throws IOException{
		//if(DEBUG)
			//System.out.println("genColHist from="+xStart+" to="+xEnd);
			List<envInterface> subNodes = nodelA.subList(xStart, xEnd);
			final Comparator<envInterface> yComp = new Comparator<envInterface>() {         
					  @Override         
					  public int compare(envInterface jc1, envInterface jc2) {
						  int y1 = (int)(jc1.getEnv()[1]+((jc1.getEnv()[3]-jc1.getEnv()[1])/2));
						  int y2 = (int)(jc2.getEnv()[1]+((jc2.getEnv()[3]-jc2.getEnv()[1])/2));
						      return (y2 < y1 ? -1 : (y2 == y1 ? 0 : 1));           
					  }     
			};
			Collections.sort(subNodes, yComp);
			double zMin = Double.MAX_VALUE;
			int ix0 = 0;
			int iy0 = 0;
			int iz0 = 0;
			int ix1 = 0;
			int iy1 = 0;
			int iz1 = 0;
			envInterface cnode = null;
			// sorted now by x and y
			for(int i = 0; i < subNodes.size(); i++) {
				if( i < subNodes.size()-1) {
					ix0 = (int)subNodes.get(i).getEnv()[0]+(((int)subNodes.get(i).getEnv()[2]-(int)subNodes.get(i).getEnv()[0])/2);
					iy0 = (int)subNodes.get(i).getEnv()[1]+(((int)subNodes.get(i).getEnv()[3]-(int)subNodes.get(i).getEnv()[1])/2);
					iz0 = (int)(subNodes.get(i).getDepth()*10.0d);
					ix1 = (int)subNodes.get(i+1).getEnv()[0]+(((int)subNodes.get(i+1).getEnv()[2]-(int)subNodes.get(i+1).getEnv()[0])/2);
					iy1 = (int)subNodes.get(i+1).getEnv()[1]+(((int)subNodes.get(i+1).getEnv()[3]-(int)subNodes.get(i+1).getEnv()[1])/2);
					iz1 = (int)(subNodes.get(i+1).getDepth()*10.0d);
					writer_file.line3D(dos, ix0, iy0, iz0, ix1, iy1, iz1, 0, 255, 255);
					//if( DEBUG ) 
					//	System.out.println("genColHist line3D x="+ix0+" y="+iy0+" z="+iz0+" to x="+iz1+" y="+iy1+" z="+iz1);
				}
				if(iy0 >= 0 && subNodes.get(i).getDepth() < zMin) {
					zMin = subNodes.get(i).getDepth();
					cnode = subNodes.get(i);
				}
			}
			//  min depth
			if( cnode != null ) {
			Vector4d cen1 = new Vector4d();
			Vector4d cen2 = new Vector4d();
			cen1.x = cnode.getEnv()[0];
			cen1.y = cnode.getEnv()[1];
			cen1.z = cnode.getDepth()*10.0d;//cnode.getCentroid().z - (cnode.getSize()/2);
			cen2.x = cnode.getEnv()[2];
			cen2.y = cnode.getEnv()[3];
			cen2.z = cnode.getDepth()*10.0d;//cnode.getCentroid().z + (cnode.getSize()/2);
			//if( DEBUG )
			//	System.out.println("genColHist env minx,y,z="+cen1+" maxx,y,z="+cen2+" centroid node="+cnode);
			// xmin, ymin, xmax, ymin
			writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen1.y, (int)cen1.z, 0, 127, 255);
			// xmax, ymin, xmax, ymax
			writer_file.line3D(dos, (int)cen2.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 0, 127, 255);
			// xmax, ymax, xmin, ymax
			writer_file.line3D(dos, (int)cen2.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen2.y, (int)cen2.z, 0, 127, 255);
			// xmin, ymax, xmin, ymin
			writer_file.line3D(dos, (int)cen1.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 0, 127, 255);
			}
	}
}
