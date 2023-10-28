// ------------------------------
// ehWSN.ned - file of all NEDs for the EH-WSN project
//
// E. Elmallah: July 2023
// ------------------------------

simple WiChan {
    parameters:
        int logLevel   = default(2);	// 2 is for exchanged messages
	int wiChanAddr = default (777);
	int xCellSize  = default (10);
	int yCellSize  = default (10);
	int xGridSize  = default (parent.xGridSize);
	int yGridSize  = default (parent.yGridSize);
	int numNodes   = default (parent.numNodes);
	int maxPktLen  = default (300)   @units(bytes);	
	int chanRate   = default (20000)  @units(bps);

        // for directional reachability
	double cellReachSampleSize= default(10);         //# of sample points
	double cellReachThr    = default (0.75) @units(ratio);

    gates:
        inout  ioNode[];     // = parent.numNodes;
}

// ------------------------------

simple Node {
    parameters:
        int     logLevel  = default (5);      // 2 is for exchanged messages
	int     nodeAddr;
	int     sinkAddr  = default(111);     // for single-sink WSN
	int     bcastAddr = default(9999);    // boradcast addr
	int     wiChanAddr= parent.wiChan.wiChanAddr;

	// A destSet is a comma separated list of node addresses
	string  destSet= default(""); 		     // for AdHoc nets
	//string  destSet= default(string(sinkAddr));  // for WSNs

        double  xLoc= default(0) @units(m);
        double  yLoc= default(0) @units(m);
	double  angleMidRay= default(0)       @units(deg);  // directional
	double  angleHalfWidth= default(180)  @units(deg);
	double  txRange= default(100) @units(m);

	int  maxPktLen= parent.wiChan.maxPktLen;
	int  chanRate=  parent.wiChan.chanRate;

	// Traffic generation section: in each time slot, wait for one
	// idle maxPktTime then send numPktsPerSlot pkts, spaced by
	// intervals of length (pktIa * maxPktTime)
	//
	double  slotLen= default(10.0);	   // seconds
	int  numPktsPerSlot= default(3);   // pkts generated every time slot
	int  pktIa= default(3);		   // intera-arival time in units
	     	    			   // of maxPktTime

        // Mac parameters
	//
	int  macBackoffOrder= default(10);  //in units of maxPktLen	

        // routing parameters
	//
        string staticRoutingStr= parent.staticRoutingStr;



	// Energy parameters: initial energy is set to 2400 mWH (as in an
	//    1.2V AA battery), the bit transmission energy is set to
	//    0.01 mWH (depends on bit duration), and the default harvesting
	//    rate is set to 0.5 mWS based on harvesting 400 WH/m2 rate,
	//    15cm x 15cm solar panel, and gamma= 0.2 factor of PV sizing.
	//
	double  eInitial= default(2400)          @units(mWH);
	double  eStorageCap= default(2400)       @units(mWH);
	double  eTxPerBit= default(.01)          @units(mWH);
	double  eHarvestRate= default(0.5)       @units(mWS);
	
   gates:
        inout ioWC;
}
// ------------------------------
