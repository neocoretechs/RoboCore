package com.neocoretechs.robocore.machine.bridge;

import java.io.Serializable;
import java.util.List;
import javax.xml.bind.annotation.adapters.XmlAdapter;

/**
 * This class is called by JAXB by installing it as an adapter to control serialization/deserialization
 * of subsets of the data. These subsets are controlled by the parameters in the class variables which require being preset.
 * Two different modes allow for different parameters to function in page mode, where page number and
 * page size are specified to return a single page oriented data structure, and offset mode for an absolute
 * start and end to be specified.
 * @author jg
 *
 */
public class RawDataXmlAdapter extends XmlAdapter<RawDataSubset,List<MachineReading>> implements Serializable {
	private static final long serialVersionUID = 1L;
	private static int page = 1;
	private static int numberPerPage = 20;
	private static PageMode pageMode = PageMode.PAGE;
	
	public static void setPage(int npage) { page = npage; }
	public static void setNumberPerPage(int npp) { numberPerPage = npp; }
	public static void setStart(int npage) { page = npage; }
	public static void setEnd(int npp) { numberPerPage = npp; }
	public static int getPage() { return page; }
	public static int getNumberPerPage() { return numberPerPage; }
	public static enum PageMode { PAGE, OFFSET } ;
	
	public static void setPageMode(PageMode mode) {
		pageMode = mode;
	}
	
	/* (non-Javadoc)
	 * @see javax.xml.bind.annotation.adapters.XmlAdapter#marshal(java.lang.Object)
	 */
	@Override
	public RawDataSubset marshal(List<MachineReading> arg0) throws Exception {
		int start = 0,end = 0;
		RawDataSubset rds = new RawDataSubset();
		switch(pageMode) {
			case OFFSET:
				start = page;
				end = numberPerPage;
				break;
			case PAGE:
				if( page == 0 ) {
					start = 0;
					end = arg0.size();
				} else {
					start = (page * numberPerPage) - numberPerPage;
					end = (page * numberPerPage);
					if( start >= arg0.size() )
						return rds;
					if( end >= arg0.size())
						end = arg0.size();
				}
				break;
			default:
				break;
		}

		for(int i = start; i < end; i++) {
			rds.machineReadings.add(arg0.get(i));
		}
		return rds;
	}

	/* (non-Javadoc)
	 * @see javax.xml.bind.annotation.adapters.XmlAdapter#unmarshal(java.lang.Object)
	 */
	@Override
	public List<MachineReading> unmarshal(RawDataSubset arg0) throws Exception {
		return arg0.machineReadings;
	}

}
