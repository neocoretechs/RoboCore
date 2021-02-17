package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M33 [Z<slot>] P<ultrasonic pin> D<min. distance in cm> [E<direction 1- forward facing, 0 - reverse facing sensor>] 
 * link Motor controller to ultrasonic sensor, the sensor must exist via M301
 * @author Jonathan Groff (C) NeoCoreTechs 2020,20201
 *
 */
public class M33 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M33(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M33.val());
	}
}
