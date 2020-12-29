package com.neocoretechs.robocore.machine.bridge;

public interface TopicListInterface {
		public void retrieveData(String line) throws InterruptedException;
		public MachineBridge getMachineBridge();
		public Object getResult(MachineReading mr);
		/**
		 * The request consists of the M code and the parameters, passed on to the abstract class method
		 * @param req
		 */
		public void writeRequest(String req);
}
