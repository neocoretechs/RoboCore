package com.neocoretechs.robocore.marlinspike;

import java.util.ArrayList;

import com.neocoretechs.robocore.machine.bridge.TopicListInterface;

public interface ResponseInterface {
	public TopicListInterface getTopicList();
	public void run(ArrayList<String> readLine);
}
