package com.neocoretechs.robocore;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.time.LocalDate;
import java.time.LocalTime;
import java.util.Calendar;
import java.util.Date;
import java.util.List;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.talker.VoxHumana;

import diagnostic_msgs.DiagnosticStatus;
import diagnostic_msgs.KeyValue;

/**
 * Listen for pointcloud messages
 * @author jg
 *
 */
public class PointCloudSubs extends AbstractNodeMain {
	private static boolean DEBUG = false;
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_pointcloud");
	}
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		//final Log log = connectedNode.getLog();
		Subscriber<sensor_msgs.PointCloud> subsbat = connectedNode.newSubscriber("stereo_msgs/PointCloud", sensor_msgs.PointCloud._TYPE);
	
		subsbat.addMessageListener(new MessageListener<sensor_msgs.PointCloud>() {
		@Override
		public void onNewMessage(sensor_msgs.PointCloud message) {
				System.out.println();

				System.out.println("Status "+message.getPoints().size()+" points @ "+LocalDate.now()+" "+LocalTime.now()); //2016/11/16 12:08:43
		}
		});	
	}

}
