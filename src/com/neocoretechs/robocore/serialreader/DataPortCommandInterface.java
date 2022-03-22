package com.neocoretechs.robocore.serialreader;

import java.io.IOException;

public interface DataPortCommandInterface extends DataPortInterface {
	public String sendCommand(String command) throws IOException;
}
