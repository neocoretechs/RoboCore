package com.neocoretechs.robocore;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.ros.node.ConnectedNode;
/**
* The multiarray declares a generic multi-dimensional array of a particular data type. 
* Dimensions are ordered from outer most to inner most.
* MultiArrayDimension[] dim # Array of dimension properties
* uint32 data_offset        # padding elements at front of data
* Accessors should ALWAYS be written in terms of dimension stride and specified outer-most dimension first. 
* multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
* A standard, 3-channel 640x480 image with interleaved color channels would be specified as:
* dim[0].label  = "height"
* dim[0].size   = 480
* dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
* dim[1].label  = "width"
* dim[1].size   = 640
* dim[1].stride = 3*640 = 1920
* dim[2].label  = "channel"
* dim[2].size   = 3
* dim[2].stride = 3
* multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
* 
* @author Jonathan Groff (C) NeoCoreTechs 2020
*
*/
public class RosArrayUtilities {

	public RosArrayUtilities() {
	}
	/**
 	 * The data will come in in a linear array of 3 elements * number of elements
 	 * So basically a 2D array of the 3 values, one for each controller slot/channel array[3,N] 
	 * @param connectedNode Our node transceiver info
	 * @param valBuf The linear array of values to send, which we will be transposing to our MultiAray
	 * @param label1 The label for dimension 1 of the array
	 * @param label2 The label for dimension 2 of the array
	 * @return The multi array we create
	 */
	public static std_msgs.Int32MultiArray setupInt32Array(ConnectedNode connectedNode, ArrayList<Integer> valBuf, String label1, String label2) {
		std_msgs.Int32MultiArray val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Int32MultiArray._TYPE);
		std_msgs.MultiArrayLayout vlayout = new std_msgs.MultiArrayLayout();
		List<std_msgs.MultiArrayDimension> vdim = new ArrayList<std_msgs.MultiArrayDimension>();
		std_msgs.MultiArrayDimension vdim1 = new std_msgs.MultiArrayDimension();
		std_msgs.MultiArrayDimension vdim2 = new std_msgs.MultiArrayDimension();
		vdim1.setLabel(label1);
		vdim2.setLabel(label2);
		//
		vdim1.setSize(3);
		vdim2.setSize(valBuf.size()/3);
		//
		// multiarray(i,j) = data[data_offset + dim_stride[1]*i + k]
		vdim1.setStride(valBuf.size());
		vdim2.setStride(3);
		//
		vdim.add(vdim1);
		vdim.add(vdim2);
		vlayout.setDim(vdim);
		vlayout.setDataOffset(0);
		val.setLayout(vlayout);
		int[] vali = new int[valBuf.size()];
		int i = 0;
		for( Integer inv : valBuf)
			vali[i++] = inv;
		val.setData(vali);
		return val;
	}
	/**
	 * Create a 2d unint array
	 * @param connectedNode
	 * @param valBuf
	 * @param label1
	 * @return
	 */
	public static std_msgs.UInt32MultiArray setupUInt32Array(ConnectedNode connectedNode, ArrayList<Integer> valBuf, String label1) {
		std_msgs.UInt32MultiArray val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.UInt32MultiArray._TYPE);
		std_msgs.MultiArrayLayout vlayout = new std_msgs.MultiArrayLayout();
		List<std_msgs.MultiArrayDimension> vdim = new ArrayList<std_msgs.MultiArrayDimension>();
		std_msgs.MultiArrayDimension vdim1 = new std_msgs.MultiArrayDimension();
		vdim1.setLabel(label1);
		//
		vdim1.setSize(2);
		//
		// multiarray(i,j) = data[data_offset + dim_stride[1]*i + k]
		vdim1.setStride(valBuf.size());	
		//
		vdim.add(vdim1);
		vlayout.setDim(vdim);
		vlayout.setDataOffset(0);
		val.setLayout(vlayout);
		int[] vali = new int[valBuf.size()];
		int i = 0;
		for( Integer inv : valBuf)
			vali[i++] = inv;
		val.setData(vali);
		return val;
	}
	
	public diagnostic_msgs.DiagnosticStatus setupDiagnostic(ConnectedNode connectedNode, String directive) {
		diagnostic_msgs.DiagnosticStatus statmsg = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.DiagnosticStatus._TYPE);
		String rs = null;
		statmsg.setName(directive);
		statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
		statmsg.setMessage(rs);
		diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
		List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
		li.add(kv);
		statmsg.setValues(li);
		System.out.println("Returned status "+statmsg.getMessage());
		return statmsg;
	}

}
