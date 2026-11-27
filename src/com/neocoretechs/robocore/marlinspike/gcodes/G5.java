package com.neocoretechs.robocore.marlinspike.gcodes;


import java.io.Serializable;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.ActivationInterface;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.TypeSlotChannelEnable;
import com.neocoretechs.robocore.propulsion.RobotDiffDrive;
/**
 * The list of channel power levels is a variable list depending on the number of channels the device is configured with.
 * 10 possible channels, P-Y parameters in the G5 directive.
 * G5 - Absolute command motor [Z<controller>] [P<motor power -1000 to 1000>] [Q<channel2 power>] [R<channel 3 power>] ... [Y[channel 10 power]
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021,2026
 *
 */
public class G5 extends AbstractBasicResponse implements ActivationInterface, Serializable {
	private static final long serialVersionUID = 1L;
	private boolean DEBUG = false;
	private TypeSlotChannelEnable tsce;
	private static final String[] channels= {"P","Q","R","S","T","U","V","W","X","Y"};
	public G5(AsynchDemuxer asynchDemuxer) {
		//
		// G5
		//
		super(asynchDemuxer, topicNames.G5.val(), 2);
	}
	public G5(TypeSlotChannelEnable tsce) {
		this.tsce = tsce;
	}

	@Override
	public String getActivation(int... deviceLevel) {
		StringBuilder sb = new StringBuilder();
		for(int level = 0; level < deviceLevel.length; level++) {
			sb.append(channels[level]);
			sb.append(deviceLevel[level]);
			sb.append(" ");
		}
		return String.format("G5 Z%d %s%n",tsce.getSlot(),sb.toString());
	}

}
