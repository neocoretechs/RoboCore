package com.neocoretechs.robocore.serialreader.marlinspikeport;

import java.io.IOException;

/**
 * Handles the action atacted to the signaling of an particular interrupt implementing {@link InterruptServiceInterface}
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2022
 *
 */
public interface InterruptServiceHandlerInterface {
	public void handleInterrupt() throws IOException;
}
