package com.neocoretechs.robocore.machine.bridge;

import java.io.Serializable;
import java.util.ArrayList;

public abstract class TopicList implements TopicListInterface, Serializable {
	private static final long serialVersionUID = 1L;
		MachineBridge mb;
		public TopicList(String groupName, int queueSize) {
			mb = new MachineBridge(groupName, queueSize);
		}
		@Override
		public MachineBridge getMachineBridge() { return mb; }
		@Override
		public String toString() {
			return this.getClass().getName()+" Topic:"+mb.getGroup()+" queue size:"+mb.get().length();
		}
		@Override
		public abstract void retrieveData(ArrayList<String> readLine) throws InterruptedException;

}
