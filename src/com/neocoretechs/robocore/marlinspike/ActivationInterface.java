package com.neocoretechs.robocore.marlinspike;
/**
 * This interface is responsible for generating the actual M or G code that is passed to the Marlinspike to activate the device.<p/>
 * Typically it is implemented by the corresponding M or G code object in the mcodes or gcodes package.<p/>
 * The factory that creates the instances of the implementors is located in the typeNames enum in {@link TypeSlotChannelEnable}.
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2022
 */
public interface ActivationInterface {
	public String getActivation(int deviceLevel);
}
