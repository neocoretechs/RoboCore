package com.neocoretechs.robocore.machine.bridge;

public abstract class TopicList implements TopicListInterface {
		MachineBridge mb;
		public TopicList(AsynchDemuxer demux, String groupName, int queueSize) {
			mb = new MachineBridge(groupName, queueSize);
		}
		@Override
		public MachineBridge getMachineBridge() { return mb; }

}
