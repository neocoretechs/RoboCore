/**
 * 
 */
package com.neocoretechs.robocore.machine.bridge;

import java.util.ArrayList;
import java.util.List;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAnyElement;
//import javax.xml.bind.annotation.XmlElementWrapper;
import javax.xml.bind.annotation.XmlRootElement;


/**
 * @author jg
 *
 */
@XmlRootElement(name="machineReadings")
@XmlAccessorType(XmlAccessType.FIELD)
public class RawDataSubset {
	//@XmlElementWrapper()
	@XmlAnyElement(lax=true)
	List<MachineReading> machineReadings = new ArrayList<MachineReading>();
}
