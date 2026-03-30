package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
* Set Delay HBridge PWM motor driver, which creates a 200ms delay between direction changes to 
* allow the magnetic field to collapse such that the bus doesnt sag during direction change due to back EMF.<p>
* Map pin to channel, this will check to prevent free running motors during inactivity
* For a PWM motor control subsequent G5 commands are affected here.
* and then D a direction pin that determines forward/backward , then E, the default value of the direction pin.
* The D and E pins determine whether a HIGH or LOW determines FORWARD/BACK on the given controller channel. This is
* to compensate for 'backward' motors mounted left/right, or differences in controller design. Using a combination of
* these 2 parameters you can tune any controller/motor setup properly for forward/back.<p>
* Finally, W<encoder pin> to receive hall wheel sensor signals and<br>
* M3 [Z&lt;slot&gt;] P&lt;pin&gt; C&lt;channel&gt; D&lt;direction pin&gt; E&lt;default dir&gt; W&lt;encoder pin&gt;
* @author Jonathan Groff (C) NeoCoreTechs 2020,2021
*/
public class M16 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M16(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M16.val());
	}
}
