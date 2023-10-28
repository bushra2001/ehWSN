// ------------------------------
// WiChan.cc - file for the EH-WSN project
//
// E. Elmallah: July 2023
// ------------------------------

#include "ehWSN.h"	// includes <omentpp.h> and other header files

// ------------------------------
// class: WiChan
class WiChan: public cSimpleModule
{
  public:
      // Module variables: logging level (for debugging), module signature
      // string (e.g., "node20   | "), and signature separator
      //
      int      logLevel;
      char     modSig[MAXWORD], modSigSep[MAXWORD];

      int  wiChanAddr;  // a fictious address used mainly for debugging
      int  xCellSize, yCellSize, xGridSize, yGridSize;
      int  xCellCount, yCellCount;   // each cell index is in [0,this count]
      int  xyCellCount;         // = xCellCount * yCellCount
      int  numNodes;            // = par("parent.numNodes")

      // A cell is considered reachable from a node (with directional antena)
      // if the node can reach at least (cellReachThr * cellReachSampleSize)
      // randomly selected points in the cell
      //
      double cellReachSampleSize;
      double cellReachThr;

      int     maxPktLen;        // in bytes
      int     chanRate;         // in bps
      double  maxPktTime;       // = (maxPktLen * 8)/chanRate

      map_int2nodeWiInfo  DB_nodeWiInfo;   // DB of all nodes wireless
                                           // info known to the channel
      map_int2set_int     DB_cellNodes;    // DB of nodes per cell
      deque_recvPkt       DB_recvPkt;      // queue of recently received pkts

      STAT stat;        // counters (statistics collected)

      virtual void initialize();
      virtual void handleMessage(cMessage *msg);
      virtual void finish();

      void moduleVarInit();		//initialize remaining variables
      void do_msg_loc (cMessage* msg);
      void do_msg_mac (cMessage* msg);

      //convert cell (x,y)-coordinate to linear index (and the converse)
      //
      int      do_xy2lin (int xIdx, int yIdx);
      pair_int do_lin2xy (int xyCellIdx);

      bool     withinAntenaRange (nodeWiInfo_t& node,
                                  double xtest, double ytest);
      double   getDist (double xi, double yi, double xj, double yj);
      bool     isCellReachable_3 (nodeWiInfo_t& node, int xyCellIdx);      
      void     update_nodeWiInfo (NodeInfo_t& nodeInfo, int gateIdx);
      void     msgBroadcast (cMessage *msg, simtime_t delay,
                             int srcAddr, set_int recvCellSet);

      void     print_nodeWiInfo (int margin, const char *prefix);
      void     print_cellNodes (int margin, const char *prefix);
      void     print_cellDist (int margin, const char *prefix);
      void     print_recvPkt (int magin, const char *prefix);
      void     print_wiChan (int margin, const char *prefix);
};
//---------- 

Define_Module(WiChan);	 // mandatory!!
//---------- 

void WiChan::initialize ()
{
  int i;
  logLevel=   par("logLevel").intValue();
  xCellSize=  par("xCellSize").intValue();
  yCellSize=  par("yCellSize").intValue();
  xGridSize=  par("xGridSize").intValue();
  yGridSize=  par("yGridSize").intValue();
  wiChanAddr= par("wiChanAddr").intValue();
  numNodes=   par("numNodes").intValue();
  maxPktLen=  par("maxPktLen").intValue();
  chanRate=   par("chanRate").intValue();

  cellReachSampleSize=  par("cellReachSampleSize").doubleValue();
  cellReachThr=         par("cellReachThr").doubleValue();

  //
  if (logLevel <= 1) {
       printf("\t ** WiChan::initialize(): getId()= %d \n", getId());
       printf("\t ** WiChan::initialize(): ioNode[] IDs \n");
       printf("\t    getBaseId(ioNode$i)= 0x%x, getBaseId(ioNode$o)= 0x%x \n",
	             gateBaseId("ioNode$i"), gateBaseId("ioNode$o") );
       for (i= 0; i < gateSize("ioNode"); i++) {
           printf("\t     ioNode[%d]$i= 0x%x, ioNode[%d]$o= 0x%x \n",
      		     i, gate("ioNode$i",i)->getId(),
		     i, gate("ioNode$o",i)->getId());
       }		     
  }

  xCellCount= xGridSize/xCellSize;	 // integer division
  yCellCount= yGridSize/yCellSize;
  xyCellCount= xCellCount * yCellCount;

  maxPktTime= (double) (maxPktLen * 8) / chanRate;

  moduleVarInit();     	   // initialize some remaining variables	  

  printf ("%s finished initialization \n", modSig);
}
// --------------------

void WiChan::handleMessage (cMessage* msg)
{

  int msgKind= (int)msg->getKind();    // can also use msg->getKind() directly
  if (msgKind == MSG_LOC_UP) {	
      do_msg_loc (msg);
  }
  else if (msgKind == MSG_MAC) {
      do_msg_mac(msg);
  }      
  delete msg;
} 
// --------------------

void WiChan::finish() {
    char buf[MAXLINE];

    newWS(1,'\n');
    print_wiChan(1, "finish(): print_wiChan()\n");

    newWS(1,'\n');
    printf ("%s finish(): stat.print()\n", modSig);    
    stat.print(modSig,1, "");
    printf("\n%s\n", modSigSep);
}

// --------------------
//
void  WiChan::moduleVarInit() {
  std::string  modSigStr, modSigSepStr;

  modSigStr   = Util_formatModSig (getFullName());
  modSigSepStr= modSigStr + "----------";

  strcpy (modSig, modSigStr.c_str());
  strcpy (modSigSep, modSigSepStr.c_str());

  DB_cellNodes.clear();	   // clear the DB of the set of nodes in each cell
  DB_recvPkt.clear();	   // clear the DB of recent received pkts
  stat.clear();		   // clear basic statistics

}

// --------------------
void WiChan::do_msg_loc (cMessage * msg)
{
    char buf[MAXLINE];
    NodeInfo_t    nodeInfo;		// defined in a *msg file
    nodeWiInfo_t  nodeWiInfo;			

    stat.c[S_MSG_LOC_RECV]++; 
    Msg_Loc *msgIn= check_and_cast< Msg_Loc* > (msg);

    Util_printMsg(msgIn, modSig, 0, "received ");

    nodeInfo = msgIn->getNodeInfo();	// get payload from the message


    // get gateIdx of the received message
    //
    int gateIdx;
    for (gateIdx= 0; gateIdx < gateSize("ioNode"); gateIdx++) {
        if (msg->arrivedOn ("ioNode$i", gateIdx)) break;
    }
    //printf ("**debug: WiChan::do_msg_loc: msg arrived on ioNode[%d]$i \n",
    //        gateIdx);

    // update map "DB_nodeWiInfo", and set "DB_cellNodes'
    //
    update_nodeWiInfo (nodeInfo, gateIdx);

    // send MSG_LOC_REPLY
    //
    Msg_Loc *msgOut= new Msg_Loc( "MSG_LOC_REPLY", MSG_LOC_REPLY);

    nodeInfo.destAddr= nodeInfo.srcAddr;
    nodeInfo.srcAddr=  wiChanAddr;
    msgOut->setNodeInfo(nodeInfo);

    send(msgOut,"ioNode$o", gateIdx);
    stat.c[S_MSG_LOC_SENT]++;
}	     

//##newpage
// --------------------
// do_msg_mac():  assume that srcAddr is truely that of the sender
// 		   node (not a forged source)

void WiChan::do_msg_mac (cMessage * msg)
{
    int           srcAddr, destAddr, conflictPktSrcAddr, idx, qBackIdx;
    bool	  collisionLoss;     
    char          buf[MAXLINE];
    double	  macMsgTime;
    set_int	  recvCellSet, noSet;	// set of cells that hear collisions
    set_int::iterator sit;

    MacPkt_t	  macPkt;
    NodeInfo_t    nodeInfo;		// defined in a *msg file
    nodeWiInfo_t  nodeWiInfo;
    recvPkt_t     recvPkt; 

    stat.c[S_MSG_MAC_OVERHEAR]++;

    // 1. get msg payload
    //
    Msg_Mac *msgIn= check_and_cast< Msg_Mac * > (msg);
    if (logLevel <= 2) {
        Util_printMsg(msgIn, modSig, 1, "received ");
    }	
    macPkt = msgIn->getMacPkt();	// get payload from the message

    // 2. initialize recvCellSet
    //
    srcAddr= macPkt.macSrcAddr;
    destAddr= macPkt.macDestAddr;
    recvCellSet.clear();

    if (DB_nodeWiInfo.count(srcAddr) == 0) {
       WARNING ("Warning: %s: MSG_MAC received from unknown srcAddr (= %d)\n",
       	         getFullName(), srcAddr);
       return;
    }       
    if (DB_nodeWiInfo.count(destAddr) == 0) {
       WARNING ("Warning: %s: MSG_MAC received to unknown destAddr (= %d)\n",
       	         getFullName(), destAddr);
       return;
    }       
    recvCellSet= DB_nodeWiInfo[srcAddr].nbSet;

    // 3. fill-in a recvPkt_t struct
    //
    recvPkt.srcAddr= srcAddr;		// mac addresses
    recvPkt.destAddr= destAddr;		// mac addresses
    recvPkt.txStart= msg->getArrivalTime();

    macMsgTime=  (double) (sizeof(Msg_Mac) * 8) / chanRate;
    recvPkt.txEnd=   msg->getArrivalTime() +
                     SimTime (macMsgTime * 1E6, SIMTIME_US);

//    if (logLevel <= 2) {
//        simtime_t  tmp_pktTxTime;
//	tmp_pktTxTime= SimTime (macMsgTime * 1E6, SIMTIME_US);
//	printf("%s\n", modSigSep);
//	printf("%s *** debug: macMsgTime = %g, tmp_pktTxTime= %g \n",
//		   modSig, macMsgTime, tmp_pktTxTime.dbl() );
//	printf("%s\n", modSigSep);
//    }	

    recvPkt.msg= msgIn;          // ptr to received pkt
    DB_recvPkt.push_back(recvPkt);

    // 4. get rid of old pkts in the queue
    //
    while ( (DB_recvPkt.size() > 0) &&
    	    (DB_recvPkt[0].txEnd < recvPkt.txStart) ) {
        DB_recvPkt.pop_front();
    }

    // 5. check collisions between the most recently received pkt
    //    and older pkts. Adjust the recvCellSet accordingly
    //
    collisionLoss= false;
    qBackIdx= DB_recvPkt.size() - 1;
    for (idx= 0; idx < qBackIdx; idx++) { // loop on queued received pkts
        if (DB_recvPkt[idx].txEnd >= recvPkt.txStart) {
	    conflictPktSrcAddr= DB_recvPkt[idx].srcAddr;
	    noSet= DB_nodeWiInfo[conflictPktSrcAddr].nbSet;

            // subtract elements of the noSet
	    for (sit= noSet.begin(); sit != noSet.end(); sit++) {
	    	if (DB_cellNodes[*sit].size() > 0) {
		    collisionLoss= true;
		}    
	        recvCellSet.erase(*sit);
            } // end for		
        } // end if
    } // end for

    if (collisionLoss == true) stat.c[S_MSG_COLLISION]++;

    // 6. relay a copy of recv_msg to each node in each cell in recvCellSet
    //    except the sender's node
    //
    simtime_t    delay= recvPkt.txEnd - recvPkt.txStart;
    msgBroadcast (msg, delay, srcAddr, recvCellSet);
}       

// --------------------
// msgBroadcast:  broadcast copies of msg to all nodes in all cells
//                in the set recvCellSet
//
void WiChan::msgBroadcast (cMessage *msg, simtime_t delay,
                           int srcAddr, set_int recvCellSet)
{
  int      nodeAddr, recvCellIdx;
  set_int::iterator  sit, sit2;

  for (sit= recvCellSet.begin(); sit != recvCellSet.end(); sit++) {
      recvCellIdx= *sit;	 // linear idx of a receiving cell

      // if recvCellIdx is not in the map DB_cellNodes then the cell is empty,
      // just continue
      if (DB_cellNodes.count(recvCellIdx) == 0) continue;

      for (sit2 = DB_cellNodes[recvCellIdx].begin();
      	   sit2 != DB_cellNodes[recvCellIdx].end(); sit2++) {
          nodeAddr= *sit2;
	  if (srcAddr == nodeAddr) continue;
	  sendDelayed (msg->dup(), delay, "ioNode$o",
	               DB_nodeWiInfo[nodeAddr].wiChanGateIdx);
	  stat.c[S_MSG_BROADCAST]++;	// update stats
      }

  } 
}
// --------------------
int WiChan::do_xy2lin (int xIdx, int yIdx) {
  return  (yIdx * xCellCount) + xIdx;
}    

pair_int WiChan::do_lin2xy (int xyCellIdx) {
    return std::make_pair (xyCellIdx % xCellCount, xyCellIdx/xCellCount);
}

// --------------------

double WiChan::getDist (double xi, double yi, double xj, double yj) {
  double dist;
  dist= sqrt( pow( (xj-xi)*xCellSize,2 )  +
              pow( (yj-yi)*yCellSize,2 ) );
  return dist;
}

// -----
bool WiChan::withinAntenaRange (nodeWiInfo_t& nodeWiInfo,
                                double xtest, double ytest) {
  double  dx, dy;
  double  angleRad, angleDeg, midRay, halfWidth, ccwRay, cwRay;

  // 1. translate point (xtest,ytest) as if (xLoc,yLoc) is the origin
  dx= xtest - nodeWiInfo.xLoc;
  dy= ytest - nodeWiInfo.yLoc;

  // 2. find the angle of the vector from origin to (xtest,ytest)
  angleRad= atan2(dy, dx);
  angleDeg= (angleRad * 180) / M_PI;	// M_PI defined in <cmath>

  // 3. find the angles (in degrees) of the CCW and CW rays
  //    around the center of the beam
  midRay    = nodeWiInfo.angleMidRay;
  halfWidth = nodeWiInfo.angleHalfWidth;
  ccwRay    = midRay - halfWidth;
  cwRay     = midRay + halfWidth;

  // 4. convert all angles to be positive (CCW) in the range [0,360)
  angleDeg= Util_scaleAngle (angleDeg);
  ccwRay=   Util_scaleAngle (ccwRay);
  cwRay=    Util_scaleAngle (cwRay);

  // 5. decide if the vector to (xtest,ytest) lies within the CCW and CW
  //    rays
  
  if (halfWidth == 180) return true;	// omnidirectional antena
  else if (ccwRay < cwRay) {
       if ( (angleDeg >= ccwRay) && (angleDeg <= cwRay) ) return true;
  }     
  else if (ccwRay > cwRay) {
       if ( (angleDeg >= ccwRay)  || (angleDeg <= cwRay) ) return true;
  }
  return false;
}


//##newpage
// --------------------
bool  WiChan::isCellReachable_3 (nodeWiInfo_t& nodeWiInfo, int jCellIdx) {
  bool      withinReach;
  int	    i, nodeAddr, xCellIdx_j, yCellIdx_j, reachCount;
  double    xLoc, yLoc, xtest, ytest, txRange, midRay, halfWidth, dist;
  pair_int  pj, pipj;

  reachCount= 0;		// # of sample points found reachable
  pj= do_lin2xy(jCellIdx);
  xCellIdx_j= pj.first;
  yCellIdx_j= pj.second;

  nodeAddr= nodeWiInfo.nodeAddr;
  xLoc= nodeWiInfo.xLoc;
  yLoc= nodeWiInfo.yLoc;
  txRange= nodeWiInfo.txRange;
  midRay=  nodeWiInfo.angleMidRay;
  halfWidth= nodeWiInfo.angleHalfWidth;

  for (i= 0; i < cellReachSampleSize; i++) {
      xtest= uniform (xCellIdx_j * xCellSize , (xCellIdx_j +1) * xCellSize);
      ytest= uniform (yCellIdx_j * yCellSize , (yCellIdx_j +1) * yCellSize);

      withinReach= Util_withinAntenaRange (xLoc, yLoc, txRange, midRay,
                   halfWidth, xtest, ytest, logLevel, modSig);

      // debug information
//      if ((nodeAddr == 4) && (jCellIdx == 2)) {
//          printf ("%s\n", modSigSep);
//          printf ("%s *** debug: nodeAdrr= %d, (jCellIdx= %d, xtest= %g, ytest= %g)\n",
//      	          modSig, nodeAddr, jCellIdx, xtest, ytest);
//          printf ("%s withinReach= %s\n", modSig,
//                  (withinReach == true)? "yes" : "no");
//          printf ("%s\n", modSigSep);
//      }	  
      
      if (withinReach) reachCount++;
  } // end for i

  if (reachCount >= (cellReachThr * cellReachSampleSize) ) return true;
  else return false;
}

// --------------------
void WiChan::update_nodeWiInfo (NodeInfo_t& nodeInfo, int gateIdx) {

  double	xLoc, yLoc, txRange;
  int           nodeAddr, xCellIdx, yCellIdx, xyCellIdx, jCell;
  pair_int	pipj;

  nodeAddr= nodeInfo.srcAddr;	// srcAddr of the received MSH_LOC pkt
  xLoc= nodeInfo.xLoc;  yLoc= nodeInfo.yLoc;
  txRange=   nodeInfo.txRange;
  xCellIdx=  (int) xLoc / xCellSize;	  
  yCellIdx=  (int) yLoc / yCellSize;
  xyCellIdx= do_xy2lin (xCellIdx, yCellIdx);

  // copy information into struct nodeWiInfo
  //
  nodeWiInfo_t  nodeWiInfo;  

  nodeWiInfo.nodeAddr       = nodeAddr;
  nodeWiInfo.xLoc           = xLoc;
  nodeWiInfo.yLoc           = yLoc;
  nodeWiInfo.angleMidRay    = nodeInfo.angleMidRay;
  nodeWiInfo.angleHalfWidth = nodeInfo.angleHalfWidth;
  nodeWiInfo.txRange        = nodeInfo.txRange;

  nodeWiInfo.xCellIdx       = xCellIdx;
  nodeWiInfo.yCellIdx       = yCellIdx;
  nodeWiInfo.xyCellIdx      = xyCellIdx;
  nodeWiInfo.wiChanGateIdx  = gateIdx;

  // find the neighborung cells
  //
  nodeWiInfo.nbSet.clear();
  for (jCell= 0; jCell < xyCellCount; jCell++) {
      if (isCellReachable_3 (nodeWiInfo, jCell)) {
          nodeWiInfo.nbSet.insert(jCell);
      }	  

      //pipj= std::make_pair(xyCellIdx,jCell);
      //if (DB_cellDist[pipj] <= txRange) nodeWiInfo.nbSet.insert(jCell);
  }

  // update the node's current cell
  //
  int  oldCellIdx;
  if (DB_nodeWiInfo.count(nodeAddr) == 1) { // node already exists in the DB?
      oldCellIdx= DB_nodeWiInfo[nodeAddr].xyCellIdx;
      if (oldCellIdx != xyCellIdx) DB_cellNodes[oldCellIdx].erase(nodeAddr);
  }
  DB_cellNodes[xyCellIdx].insert(nodeAddr);   // add nodeAddr to the set of cellNodes

  DB_nodeWiInfo[nodeAddr]= nodeWiInfo;
}

// --------------------
void  WiChan::print_cellNodes (int margin, const char *prefix) {

  int         iCell;
  std::string str;

  printf("%s", modSig); newWS(margin,' ');
  printf("%s", prefix);
  for (iCell= 0; iCell < xyCellCount; iCell++) {
      str= Util_setInt2str (DB_cellNodes[iCell]);
      printf("%s", modSig); newWS(margin,' ');      
      printf ("cell[%d]: %s\n", iCell, str.c_str());
  }
  newWS(1,'\n');
}  

// --------------------
void WiChan::print_recvPkt (int margin, const char *prefix) {
  int         idx;
  recvPkt_t  recvPkt;

  printf("%s", modSig); newWS(margin,' ');
  printf("%s", prefix);
  for (idx= 0; idx < DB_recvPkt.size(); idx++) {
      recvPkt= DB_recvPkt[idx];
      printf("%s", modSig); newWS(margin,' ');      
      printf("[%d]: (srcAddr= %d, destAddr= %d, txStart= %s, txEnd= %s, ...)\n",
      	      idx, recvPkt.srcAddr, recvPkt.destAddr,
	      recvPkt.txStart.str().c_str(), recvPkt.txEnd.str().c_str());
  }
}  
// --------------------
void WiChan::print_nodeWiInfo (int margin, const char *prefix) {

  int                          nodeAddr;
  nodeWiInfo_t                 nodeWiInfo;
  std::string                  str_nbSet;
  map_int2nodeWiInfo::iterator It;

  for (It= DB_nodeWiInfo.begin();   It != DB_nodeWiInfo.end(); It++) {
      nodeAddr=    It->first;
      nodeWiInfo = It->second;

      printf("%s", modSig); newWS(margin,' ');      
      printf("[%d]: (xLoc= %g, yLoc= %g, midRay= %g, halfWidth= %g, txRang= %g\n",
               nodeAddr, nodeWiInfo.xLoc, nodeWiInfo.yLoc,
	       nodeWiInfo.angleMidRay, nodeWiInfo.angleHalfWidth,
	       nodeWiInfo.txRange);

      printf("%s", modSig); newWS(margin,' ');
      printf ("      (xCellIdx= %d, yCellIdx= %d, xyCellIdx= %d\n",
                      nodeWiInfo.xCellIdx, nodeWiInfo.yCellIdx,
		      nodeWiInfo.xyCellIdx);

      printf("%s", modSig); newWS(margin,' ');
      str_nbSet= Util_setInt2str(nodeWiInfo.nbSet);
      printf ("       nbSet= %s) \n", str_nbSet.c_str());
  }
} 
// --------------------
void WiChan::print_wiChan (int margin, const char *prefix) {

  printf("%s", modSig); newWS(margin,' ');
  printf("%s", prefix);

  printf("%s", modSig); newWS(margin,' ');  
  printf("xCellSize= %d, yCellSize= %d, xGridSize= %d, yGridSize= %d \n",
          xCellSize, yCellSize, xGridSize, yGridSize);

  printf("%s", modSig); newWS(margin,' ');  
  printf("xCellCount= %d, yCellCount= %d, xyCellCount= %d \n",
          xCellCount, yCellCount, xyCellCount);

  printf("%s", modSig); newWS(margin,' ');  
  printf("chanRate= %d (bps), maxPktLen= %d (bytes), maxPktTime= %g (sec)\n",
          chanRate, maxPktLen, maxPktTime);

  printf("%s", modSig); newWS(margin,' ');  
  printf("numNodes= %d \n", numNodes);

  newWS (1,'\n'); 
  print_nodeWiInfo(margin, "DB_nodeWiInfo[]:\n");

  newWS (1,'\n'); 
  print_cellNodes(margin, "DB_cellNodes[]:\n");

  newWS (1,'\n');  
  print_recvPkt  (margin, "DB_recvPkt[]:\n");
  newWS(1,'\n');
}  

