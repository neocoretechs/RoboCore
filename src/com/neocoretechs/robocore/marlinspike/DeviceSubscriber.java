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

import com.neocoretechs.robocore.propulsion.MotionController.InternalPayload;
import com.neocoretechs.robocore.propulsion.RobotDiffDrive;

import diagnostic_msgs.DiagnosticStatus;

/**
 * Internal reactive subscriber class, one existing for each DeviceEntry, and associated subclasses SlotEntry, RobotDiffDrive, etc.<p>
 * On new data from controller, model etc, the publisher delivers the int[] payload that activates each specific device
 * via the process method.
 * @see DeviceEntry
 * @see com.neocoretechs.robocore.config.SlotEntry
 * @see com.neocoretechs.robocore.propulsion.MotionController
 * @see com.neocoretechs.robocore.propulsion.RobotDiffDrive
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2026
 * @param <T>
 */
public class DeviceSubscriber<T> implements Flow.Subscriber<T> {
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
	public void onNext(T item) {
		process(item);
		subscription.request(1);
	}

	protected void process(T item) {

		try {
			MarlinspikeControlInterface control = null;
			control = de.getMarlinspikeControl();//robot.getManager().getMarlinspikeControl(device.getName());
			if(de instanceof RobotDiffDrive) {
				((RobotDiffDrive)de).activation((InternalPayload) item);
			} else {
				if(de.getManualControl().isAxis()) {
					float[] axis = ((InternalPayload)item).getAxis();
					// set the operation
					Integer[] levels = (Integer[]) de.getManualControl().matches(axis);
					control.setDeviceLevels(de.getTopicName(), levels);
				}
				if(de.getManualControl().isButton()) {
					int[] button = ((InternalPayload)item).getButtons();
					// set the operation
					Integer[] levels = (Integer[]) de.getManualControl().matches(button);
					control.setDeviceLevels(de.getTopicName(), levels);
				}
			}
			//
			if(DEBUG)
				System.out.printf("%s Subscriber:%s process DeviceName=%s Control=%s Thread:%s%n", this.getClass().getName(), this, de, control, Thread.currentThread().getName());
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
		return this.getClass().getName()+" for device topic "+de.getTopicName();
	}

}
