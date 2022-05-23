package com.neocoretechs.robocore.serialreader.marlinspikeport;

import java.io.IOException;

public interface InterruptServiceInterface {
	public int getPin();
	public void setInterruptServiceHandler(InterruptServiceHandlerInterface ishi);
	public void service() throws IOException;
}
