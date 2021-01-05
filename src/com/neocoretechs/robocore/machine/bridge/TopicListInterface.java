package com.neocoretechs.robocore.machine.bridge;

public interface TopicListInterface {
		public void retrieveData(String line) throws InterruptedException;
		public MachineBridge getMachineBridge();
		public Object getResult(MachineReading mr);

}
