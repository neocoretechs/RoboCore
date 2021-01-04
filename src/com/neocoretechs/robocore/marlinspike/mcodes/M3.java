package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
* Set HBridge PWM motor driver, map pin to channel, this will check to prevent free running motors during inactivity
* For a PWM motor control subsequent G5 commands are affected here.
* and then D a direction pin that determines forward/backward , then E, the default value of the direction pin.
* The D and E pins determine whether a HIGH or LOW determines FORWARD/BACK on the given controller channel. This is
* to compensate for 'backward' motors mounted left/right, or differences in controller design. Using a combination of
* these 2 parameters you can tune any controller/motor setup properly for forward/back.
*  Finally, W<encoder pin>  to receive hall wheel sensor signals and
* optionally PWM timer setup [R<resolution 8,9,10 bits>] [X<prescale 0-7>].
* The Timer mode (0-3) is preset to 2 in the individual driver. Page 129 in datasheet. Technically we are using a 'non PWM'
* where the 'compare output mode' is defined by 3 operating modes. Since we are unifying all the timers to use all available PWM
* pins, the common mode among them all is the 'non PWM', within which the 3 available operating modes can be chosen from.
* There are essentially three main operating modes:
* 0 - Stop
* 1 - Toggle on compare match
* 2 - Clear on match
* 3 - Set on match
* For motor operation and general purpose PWM, mode 2 the most universally applicable.
* M3 [Z<slot>] P<pin> C<channel> D<direction pin> E<default dir> W<encoder pin> [R<resolution 8,9,10 bits>] [X<prescale 0-7>]
* @author groff
*
*/
public class M3 implements Runnable {
	private boolean DEBUG;
	private boolean shouldRun = true;
	private TopicList topicList;
	AsynchDemuxer asynchDemuxer;
	private Object mutex = new Object();
	String data;
	public M3(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		this.asynchDemuxer = asynchDemuxer;
		//
		// M3 - setup bridge controller
		//
		this.topicList = new TopicList(asynchDemuxer, topicNames.M3.val(), 2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				data = asynchDemuxer.getMarlinLines().takeFirst();
				synchronized(mutex) {
					mutex.notify();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		};
		topics.put(topicNames.M3.val(), topicList);
	}
	@Override
	public void run() {
		while(shouldRun) {
			synchronized(mutex) {
				try {
					mutex.wait();
					topicList.getMachineBridge().add(MachineReading.EMPTYREADING);
					synchronized(asynchDemuxer.mutexWrite) {
						asynchDemuxer.mutexWrite.notifyAll();
					}
				} catch (InterruptedException e) {
					shouldRun = false;
				}
			}
		}
		
	}

}
