package com.neocoretechs.robocore.serialreader;

import java.io.IOException;
import java.util.ArrayList;
/**
 * Command/response contract for those dataports that support synchronous communication.
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2022
 *
 */
public interface DataPortCommandInterface extends DataPortInterface {
	public ArrayList<String> sendCommand(String command) throws IOException;
}
