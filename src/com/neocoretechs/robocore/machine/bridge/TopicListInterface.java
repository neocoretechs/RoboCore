package com.neocoretechs.robocore.machine.bridge;

import java.util.ArrayList;

public interface TopicListInterface {
		public void retrieveData(ArrayList<String> readLine) throws InterruptedException;
		public MachineBridge getMachineBridge();
		public Object getResult(MachineReading mr);

}
