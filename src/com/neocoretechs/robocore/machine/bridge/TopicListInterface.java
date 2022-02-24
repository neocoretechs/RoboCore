package com.neocoretechs.robocore.machine.bridge;

import java.util.ArrayList;
/**
 * Access to data from list of topics from the designated {@link MachineBridge}
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2021,2022
 *
 */
public interface TopicListInterface {
	/**
	 * Retrieve available data from {@link MachineBridge} into collection
	 * @param readLine collection to receive data
	 * @throws InterruptedException
	 */
	public void retrieveData(ArrayList<String> readLine) throws InterruptedException;
	
	/**
	 * Get attached {@link MachineBridge}
	 * @return
	 */
	public MachineBridge getMachineBridge();
	
	/**
	 * Return a {@link MachineReading} from attached {@link MachineBridge}
	 * @param mr
	 * @return
	 */
	public Object getResult(MachineReading mr);

}
