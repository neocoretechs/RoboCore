package com.neocoretechs.robocore.machine.bridge;

import java.util.ArrayList;

public abstract class TopicList implements TopicListInterface {
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
