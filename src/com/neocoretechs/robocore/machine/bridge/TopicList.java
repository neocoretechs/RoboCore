package com.neocoretechs.robocore.machine.bridge;

public abstract class TopicList implements TopicListInterface {

		AsynchDemuxer demux;
		MachineBridge mb;
		public TopicList(AsynchDemuxer demux, String groupName, int queueSize) {
			this.demux = demux;
			mb = new MachineBridge(groupName, queueSize);
		}
		@Override
		public MachineBridge getMachineBridge() { return mb; }
		/**
		 * Issue a request which is queued to outbound Marlinspike write queue, then
		 * wait an interval for a corresponding ACK response to come back which matches request.
		 * If the response, which triggers a notifyAll in the corresponding retrieveData method, does
		 * not match request or times out, toss an error.
		 * @param ad
		 * @param req
		 */
		@Override
		public void writeRequest(String req) {
			AsynchDemuxer.addWrite(demux, req);
			synchronized(demux.mutexWrite) {
				try {
					demux.mutexWrite.wait(500);
				} catch (InterruptedException e) {
					System.out.println("Timeout - No write response from Marlinspike for:"+req);
					e.printStackTrace();
				}
			}
		}

}
