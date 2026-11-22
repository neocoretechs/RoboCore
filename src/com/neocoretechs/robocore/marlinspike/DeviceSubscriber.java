package com.neocoretechs.robocore.marlinspike;

import java.io.IOException;
import java.util.Arrays;
import java.util.Collection;
import java.util.NoSuchElementException;
import java.util.concurrent.Flow;
import java.util.stream.Collectors;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import com.neocoretechs.robocore.config.DeviceEntry;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;

import diagnostic_msgs.DiagnosticStatus;
import std_msgs.Int32MultiArray;

	public class DeviceSubscriber implements Flow.Subscriber<Int32MultiArray> {
		public static boolean DEBUG = true;
	    private Flow.Subscription subscription;
	    
	    ConnectedNode connectedNode; 
	    Publisher<DiagnosticStatus> statpub; 
	    Collection<String> statusQueue;
	    CircularBlockingDeque<DiagnosticStatus> outgoingDiagnostics;
	    DeviceEntry de;
	    
	    public DeviceSubscriber(ConnectedNode connectedNode, Publisher<DiagnosticStatus> statpub, CircularBlockingDeque<DiagnosticStatus> diagQueue, Collection<String> statusQueue, DeviceEntry de) {
	    	this.connectedNode = connectedNode;
	    	this.statpub = statpub;
	    	this.outgoingDiagnostics = diagQueue;
	    	this.statusQueue = statusQueue;
	    	this.de = de;
	    }
	    
	    @Override
	    public void onSubscribe(Flow.Subscription subscription) {
	        this.subscription = subscription;
	        this.subscription.request(1);
	    }
	    
	    @Override
	    public void onNext(Int32MultiArray item) {
	        process(item);
	        subscription.request(1);
	    }
	    
	    protected void process(Int32MultiArray item) {
	    	int[] valch = item.getData();
	    	if(DEBUG)
	    		System.out.printf("%s Subscriber:%s process DeviceName=%s Topic=%s args:%s Thread:%s%n", this.getClass().getName(), this, de.getName(), de.getTopicName(), Arrays.toString(valch), Thread.currentThread().getName());
	    	try {
	    		MarlinspikeControlInterface control = null;
	    		control = de.getMarlinspikeControl();//robot.getManager().getMarlinspikeControl(device.getName());
	    		if(DEBUG)
	    			System.out.printf("%s Subscriber:%s process got Control %s from MarlinspikeManager%n", this.getClass().getName(), this, de, control);
	    		// set the operation
	    		control.setDeviceLevels(de.getTopicName(), valch);
	    		//
	    		if(DEBUG)
	    			System.out.printf("%s Subscriber:%s process Topic:%s DeviceName=%s speeds:%s thread:%s%n", this.getClass().getName(), this, 
	    					de.getTopicName(), de.getName(), Arrays.toString(valch), Thread.currentThread().getName());
	    	} catch (IOException e) {
	    		System.out.printf("%s Subscriber:%s ,There was a problem communicating with the controller:%s%n:",this.getClass().getName(), this, e);
	    		e.printStackTrace();
	    		synchronized(statpub) {
	    			statusQueue.add(String.format("%s Subscriber:%s process; There was a problem communicating with the controller:%s%n:",this.getClass().getName(), this, e));
	    			statusQueue.addAll(Arrays.stream(e.getStackTrace()).map(m->m.toString()).collect(Collectors.toList()));
	    			new PublishDiagnosticResponse(connectedNode, statpub, outgoingDiagnostics, de.getTopicName(), diagnostic_msgs.DiagnosticStatus.ERROR, statusQueue);
	    		}
	    	}
	    }
	    
	    @Override
	    public void onError(Throwable throwable) {
	        throwable.printStackTrace();
    		synchronized(statpub) {
    			statusQueue.add(String.format("%s Subscriber:%s process; There was a problem communicating with the controller:%s%n:",this.getClass().getName(), this, throwable));
    			statusQueue.addAll(Arrays.stream(throwable.getStackTrace()).map(m->m.toString()).collect(Collectors.toList()));
    			new PublishDiagnosticResponse(connectedNode, statpub, outgoingDiagnostics, de.getTopicName(), diagnostic_msgs.DiagnosticStatus.ERROR, statusQueue);
    		}
	    }
	    @Override
	    public void onComplete() {
	    }
	    @Override
	    public String toString() {
	    	return subscription.toString();
	    }
	    	
}
