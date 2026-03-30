package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
* Set HBridge PWM motor driver, map pin to channel, this will check to prevent free running motors during inactivity
* For a PWM motor control subsequent G5 commands are affected here.
* and then D a direction pin that determines forward/backward , then E, the default value of the direction pin.
* The D and E pins determine whether a HIGH or LOW determines FORWARD/BACK on the given controller channel. This is
* to compensate for 'backward' motors mounted left/right, or differences in controller design. Using a combination of
* these 2 parameters you can tune any controller/motor setup properly for forward/back.
* Finally, W&lt;encoder pin&gt;  to receive hall wheel sensor signals.
* M3 [Z&lt;slot&gt;] P&lt;pin&gt; C&lt;channel&gt; D&lt;direction pin&gt; E&lt;default dir&gt; W&lt;encoder pin&gt;
* @author Jonathan Groff (C) NeoCoreTechs 2020,2021
*
*/
public class M3 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M3(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M3.val());
	}
}
