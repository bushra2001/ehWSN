// ------------------------------
// Node - file for the EH-WSN project
//
// E. Elmallah: July 2023
// ------------------------------
// Some rv generators: intuniform(0,n); exponential(x);
// ------------------------------

#include "ehWSN.h"	// includes <omentpp.h> and other header files

// ------------------------------
// class: Node

class Node: public cSimpleModule
{
  public:
      // node variables: logLevel (for debugging), module signature
      // string (e.g., "node20   | "), and module signature separator
      //
      int      logLevel;
      char     modSig[MAXWORD], modSigSep[MAXWORD];

      int      nodeAddr, wiChanAddr, sinkAddr, bcastAddr;
      
      double   xLoc, yLoc, angleMidRay, angleHalfWidth, txRange;
      std::deque<int>     destSet;  // set of addresses the node can send to
      map_int2routeEntry  routeTable;

      int     maxPktLen;      // in bytes
      int     chanRate;       // in bps
      double  maxPktTime;     // computed from maxPktLen and chanRate (sec)

      // when time is slotted ...
      double  slotLen;		         // when time is slotted (sec)

      // traffic generation variables
      int     numPktsPerSlot;		 // # of pkts generated each time slot
      int     pktIa;           		 // intera-arival time in units
                                         // of maxPktTime
      int     simPerSlotPktCount;	 // for monitoring how many pkts
      	   				 // generated in current time slot
      
      // NET variables
      int  seqNo;			 // net layer pkts have seqNos
      
      // MAC variables
      int           macBackoffOrder;	 //backoff time in [0,macBackoffOrder-1]
      int           macState;		 //either idle or busy
      deque_macMsg  macTxQueue;		 //mac transmit queue

      // energy variables
      double  eInitial;			 // in mWH: initial energy level
      double  eCurrent;			 // im mWH: current energy level
      double  eStorageCap;		 // in mWH
      double  eTxPerBit;	         // in mWH
      double  eMaxPktConsume;		 // in mWH
      double  eHarvestRate;		 // in mWSecond
      double  eHarvestWastePerSlot;	 //      
      simtime_t	eHarvestLastUpdate;	 // last simtime the harvested energy
      					 // has been updated
      
      // timers parameters
      Msg_TimerService* timerMessages[TIMER_MAX_SIZE];

      int               timerSetupRepeat;  // setup phase repeat times

      STAT stat;            // counters (statistics collected)

      // ----------
      virtual void initialize();
      virtual void handleMessage(cMessage *);
      virtual void finish();

      void   do_msg_timer (cMessage *);
      void   do_msg_mac   (cMessage *msg);
      void   composeSendPkt2RandDest ();
      void   buildRouteTable (std::string& str_staticRouting);

      // node functions
      void nodeVarInit();	   //initialize remaining variables

      // traffic functions
      void trafficVarInit();	   //initialize remaining variables

      // net functions
      void netVarInit();
      
      // mac functions
      void   macVarInit();	   //initialize remaining variables
      void   macTimerBackoffStart ();
      void   macTimerBackoffEnd ();

      // energy functions
      void   eVarInit();	   //initialize remaining variables
      void   eHarvestUpdate();	   //update the amount of harvested energy
      void   eSlotStartUpdate();   //update ops at slot start
      void   eMac2wi_sendMsg (Msg_Mac *msgOut);

      void   print_node (int margin, const char *prefix);

      // timer functions
      void      setTimer (int idx, simtime_t time);
      void      cancelTimer (int idx);
      simtime_t getTimer (int idx);
      void      timerFiredCallback(int timerIdx);
};

//---------- 
Define_Module(Node);	// mandatory!!
//---------- 
//##newpage
//---------- 
void Node::initialize ()
{
  int          i, nx;
  char         buf[MAXLINE], x[MAX_NTOKEN][MAXWORD];
  double       minValidSlotLen;
  std::string  str_destSet, str_staticRouting;

  logLevel=    par("logLevel").intValue();
  nodeAddr=    par("nodeAddr").intValue();
  wiChanAddr=  par("wiChanAddr").intValue();
  sinkAddr=    par("sinkAddr").intValue();
  bcastAddr=   par("bcastAddr").intValue();
  
  xLoc=        par("xLoc").doubleValue();
  yLoc=        par("yLoc").doubleValue();
  angleMidRay= par("angleMidRay").doubleValue();
  angleHalfWidth= par("angleHalfWidth").doubleValue();
  txRange=     par("txRange").doubleValue();
  maxPktLen=   par("maxPktLen").intValue();
  chanRate=    par("chanRate").intValue();
  str_destSet= par("destSet").stdstringValue();

  // when time is slotted ...
  slotLen=         par("slotLen").doubleValue();

  // traffic generation parameters
  numPktsPerSlot=  par("numPktsPerSlot").intValue();
  pktIa=	   par("pktIa").intValue();	//Inter-arrival in units of
  		   				// maxPktTime duration

   // MAC parameters
   macBackoffOrder= par("macBackoffOrder").intValue();

   // energy parameters
   eInitial=     par("eInitial").doubleValue();   	      // in mWH
   eStorageCap=  par("eStorageCap").doubleValue();   // in mWH
   eTxPerBit=    par("eTxPerBit").doubleValue();          // in mWH
   eHarvestRate= par("eHarvestRate").doubleValue();       // in mWSecond   

  // read static routing information						
  str_staticRouting= par("staticRoutingStr").stdstringValue();

  timerSetupRepeat= 1;

  for (i= 0; i < TIMER_MAX_SIZE; i++) timerMessages[i]= NULL;

  //setLogLevel(LOGLEVEL_INFO);
  if (logLevel <= 1) {
      printf("%s:initialize: getFullPath() = %s, getLogLevel()= %d \n",
                  getFullName(), getFullPath().c_str(), getLogLevel());

      if (! getEnvir()->isLoggingEnabled() )
          printf("%s::initiliaze: Warnning logging disabled\n", getFullName());
  }

  // get destSet of node addresses
  //
  strcpy (buf, str_destSet.c_str());
  nx= Util_split (buf, x, ", ");  // split around commas and spaces
  for (i= 0; i < nx; i++) destSet.push_back(atoi(x[i]));

  // initialize remaining node, traffic, MAC, and energy variables
  nodeVarInit();
  netVarInit();
  trafficVarInit();
  macVarInit();
  eVarInit();
  stat.clear();
  
  // print all node information
  if (logLevel <= 2) {
      print_node(0, "initialize(): print_node(): \n");
  } 
  
  setTimer(TIMER_SETUP, 1);	// After 1 sec start phase 1 (of sending
  				// MSG_LOC_UP to the wiChan

  if (logLevel <= 2) {
      printf ("%s finished initialization \n", modSig);
      printf ("%s \n", modSigSep);      
  }      
}
// --------------------

void Node::handleMessage (cMessage* msg)
{
  char         buf[MAXLINE];
  pair_int     pair_sn;	      // a pair of statistic and nodeAddr
  AppPkt_t     appPkt;
  NetPkt_t     netPkt;
  MacPkt_t     macPkt;

  int msgKind= msg->getKind();

  if (msgKind == MSG_TIMER) {
      do_msg_timer(msg);
  }

  else if (msgKind == MSG_LOC_REPLY) {
      if (logLevel <= 2) {
          Msg_Loc *tmp= check_and_cast <Msg_Loc *> (msg);
          Util_printMsg(tmp, modSig, 0, "received ");
      }	  
  }

  else if (msgKind == MSG_MAC) {
      do_msg_mac(msg);
  }
  delete msg;  
} 
// --------------------
void Node::finish() {
  char buf[MAXLINE];

  printf("\n");
  printf ("%s (nodeAddr= %d): finish(): collected statistics\n",
           modSig, nodeAddr);
  stat.print(modSig, 0, "");
}

// --------------------
void Node::do_msg_timer(cMessage *msg) {
  Msg_TimerService *timerMsg = check_and_cast<Msg_TimerService*> (msg);
  int timerIdx= timerMsg -> getTimerIndex();

  if ((timerIdx < 0) || (timerIdx >= TIMER_MAX_SIZE)) return;

  if (timerMessages[timerIdx] != NULL) {
      timerMessages[timerIdx] = NULL;
      timerFiredCallback(timerIdx);
  }
}
// --------------------
void Node::timerFiredCallback(int timer)
{
  if (logLevel <= 2) {
      printf("%s\n", modSigSep);
      printf ("%s timerFiredCallback: timer= %s (time= %g)\n",
	       modSig, str_MSG_TIMER_TYPE[timer], simTime().dbl() );
      printf ("%s\n", modSigSep);	       
  }	       

  if (timer == TIMER_SETUP) {

      // send an update: MSG_LOC_UP (no energy consumed here)
      //
      NodeInfo_t  nodeInfo;
      Msg_Loc *tmp= new Msg_Loc("MSG_LOC_UP", MSG_LOC_UP);

      Util_composePkt (nodeInfo, nodeAddr, wiChanAddr, xLoc, yLoc,
      		       angleMidRay, angleHalfWidth, txRange);
      tmp->setNodeInfo(nodeInfo);		       
      send(tmp, "ioWC$o");	 		       // to wiChan
      stat.c[S_MSG_LOC_SENT]++;

      if ( --timerSetupRepeat > 0) {setTimer (TIMER_SETUP, 1); }
      else {setTimer (TIMER_SLOT, 1); }  // After 1 sec start phase 2:
      	   	     		         // the time slotted phase

  }

  else if (timer == TIMER_SLOT) {
      stat.c[S_SIM_SLOT_COUNT]++;	// increment the "slot count" counter
      simPerSlotPktCount= 0;		// reset the # of pkts transmitted
      			  		// in the current slot

      // Update start-of-slot energy stats, and reset the per-slot
      // wastage amount
      //
      eHarvestUpdate();			   // Update energy stats
      eSlotStartUpdate();		   // Updates done at each slot start

      setTimer (TIMER_SLOT,
      	        SimTime( slotLen * 1E6, SIMTIME_US));
      
      if (numPktsPerSlot <= 0) return;
      
      //Else, wait for one maxPktTime before starting to send data
      setTimer (TIMER_INSLOT_IDELAY,
      	        SimTime(maxPktTime * 1E6, SIMTIME_US));
  }
  else if (timer == TIMER_INSLOT_IDELAY) {
      composeSendPkt2RandDest ();
      simPerSlotPktCount++;
      setTimer (TIMER_INSLOT_RDELAY,
      	        SimTime( ((pktIa+1) * maxPktTime) * 1E6, SIMTIME_US));
  }
  else if (timer == TIMER_INSLOT_RDELAY) {
      if (simPerSlotPktCount >= numPktsPerSlot) return;
      composeSendPkt2RandDest ();
      simPerSlotPktCount++;
      setTimer (TIMER_INSLOT_RDELAY,
      	        SimTime( ((pktIa+1) * maxPktTime) * 1E6, SIMTIME_US));
  }

  else if (timer == TIMER_MAC_BACKOFF_END) {
      macTimerBackoffEnd();		   // transmit and wait for completion
  }
  else if (timer == TIMER_MAC_TX_END) {
      macState= MAC_IDLE;
      macTimerBackoffStart();		// queue or transmit next pkt (if any)
  }      
}

// --------------------
//  process all possible cases if a mac message

void Node::do_msg_mac(cMessage *msgIn) {
  char         buf[MAXLINE];
  pair_int     pair_sn;	      // a pair of statistic and nodeAddr
  AppPkt_t     appPkt;
  NetPkt_t     netPkt;
  MacPkt_t     macPkt;

  Msg_Mac *tmpMac= check_and_cast <Msg_Mac *> (msgIn);
  macPkt= tmpMac->getMacPkt();

  if (macPkt.macDestAddr == bcastAddr) {
      stat.c[S_MSG_MAC_BCAST]++;
      // do nothing further
  }
  else if (macPkt.macDestAddr != nodeAddr) {
      if (macPkt.macSrcAddr != nodeAddr) stat.c[S_MSG_MAC_OVERHEAR]++;
      // drop the msg
  }
  else if (macPkt.macDestAddr == nodeAddr) {
      // check netDestAddr
      //
      stat.c[S_MSG_MAC_RECV]++;
      netPkt= macPkt.frame;

      if (logLevel <= 2) {
          printf ("%s received: time= %s (%g) (nodeAddr= %d)\n", modSig,
	           simTime().str().c_str(), simTime().dbl(), nodeAddr);
          Util_printMsg (tmpMac, modSig, 0, "received ");
	  Util_printPkt (netPkt, modSig, 0, "included: ");
	  printf ("%s\n", modSigSep);
      }
      
      if (netPkt.netDestAddr == nodeAddr) {
          stat.c[S_MSG_NET_RECV]++;

          pair_sn= std::make_pair(S_MSG_NET_RECV, netPkt.netSrcAddr);
	  stat.cWithSrcAddr[pair_sn]++;

	  // process at the application layer
      }
      //
      // else, (netDestAddr != nodeAddr) see if we can help routing
      else {     
          if (routeTable.count(netPkt.netDestAddr) == 1) {
	      // yes, set macDestAddr= nextHop, macSrcAddr= nodeAddr, and
	      // relay the msg.
	      // Note: it is important for WiChan that the macSrcAddr
	      //       is the nodeAddr of the transmitter.
	      //
	      macPkt.macSrcAddr= nodeAddr;
	      macPkt.macDestAddr= routeTable[netPkt.netDestAddr].nextHop;

	      Msg_Mac *msgOut= new Msg_Mac ("MSG_MAC", MSG_MAC);
	      msgOut -> setMacPkt(macPkt);

	      // insert msgOut in the macTxQueue, and start a backoff
	      // process (if mac is idle). msg will be transmitted after
	      // backoff timer expires
	      //
              macTxQueue.push_back(msgOut);
	      macTimerBackoffStart();
          }
	  else { // netDestAddr is not in the routing table; drop msg.
	         // Note: a loop may arise if we set macDestAddr= netDestAddr
		 //       and transmit
	      stat.c[S_MSG_NET_NOROUTE]++;
	  }    
      }	      
  }
}
// --------------------
//  compose and send a data packet to a random destination in destSet[]

void   Node::composeSendPkt2RandDest () {
  int   rand, destAddr, macNextHopAddr;
  char  data[MAXLINE];
  AppPkt_t  appPkt;
  NetPkt_t  netPkt;
  MacPkt_t  macPkt;

  if (destSet.size() == 0) return;	// no destination to send msg to

  Msg_Mac *tmp= new Msg_Mac ("MSG_MAC", MSG_MAC);

  rand=     intuniform(0, destSet.size()-1);
  destAddr= destSet[rand];
  seqNo++;
  memset(data, (char) nodeAddr % 255, sizeof(data));
  Util_composePkt (appPkt, nodeAddr, destAddr, seqNo, data);

  Util_composePkt (netPkt, nodeAddr, destAddr, seqNo, appPkt);

  // important: use routing information
  //
  if (routeTable.count(destAddr) == 1) {
      macNextHopAddr= routeTable[netPkt.netDestAddr].nextHop;
  } else {
      macNextHopAddr= destAddr;
  }      
  Util_composePkt (macPkt, nodeAddr, macNextHopAddr, netPkt);
  
  tmp->setMacPkt(macPkt);

  macTxQueue.push_back(tmp);
  macTimerBackoffStart();	// trigger sending the pkt
  //  eMac2wi_sendMsg(tmp);	// old code: send msg if enough power
}

// ----------
void   Node::buildRouteTable (std::string& str_staticRouting) {
  int   idx, jdx, thisNodeIdx, destAddr, nextHop, nx, ny;
  char  buf[2*MAXLINE], x[MAX_NTOKEN][MAXWORD], y[MAX_NTOKEN][MAXWORD];
  routeEntry_t  routeEntry;

  routeTable.clear();
  strcpy (buf, str_staticRouting.c_str());
  nx= Util_split (buf, x, ")");		// split around character ')'

  for (idx= 0; idx < nx; idx++) { 	//x[idx] is a route
      Util_gsub(x[idx], "( )", "");	//remove characters '(', ')', and ' '

      ny= Util_split(x[idx], y, ",");	//split around ','

      for (jdx= 0; jdx < ny; jdx++) {	//Does node appear in route x[i]?
          if ( nodeAddr == atoi(y[jdx]) ) break;    // if yes, break
      }

      // continue to next path if this node is not on the current path,
      // or it is on the current path but appears in the last position (ny-1)
      //
      if (jdx < (ny-1)) {
          thisNodeIdx= jdx;
	  assert ( nodeAddr == atoi(y[thisNodeIdx]) );
      }
      else continue;

      // process the remaining nodes on the path, if the node appears
      // at position < (ny-1)
      //
      nextHop= atoi( y[thisNodeIdx+1] );
      for (jdx= thisNodeIdx +1; jdx < ny; jdx++) {
          destAddr= atoi(y[jdx]);

	  // Skip updating routeTable[destAddr] if it exists and has a
	  // lower cost
	  if ( (routeTable.count(destAddr) == 1) &&
	       (routeTable[destAddr].cost <= (jdx - thisNodeIdx)) ) continue;

          // Else, update the entry
	  routeEntry.nextHop= nextHop;
	  routeEntry.cost   = jdx - thisNodeIdx;
	  routeTable[destAddr]= routeEntry;
      } // end for jdx
    
  } // end for idx    
    
}
// ----------
// Node functions
void  Node::nodeVarInit() {
  std::string  modSigStr, modSigSepStr;
  
  modSigStr   = Util_formatModSig (getFullName());
  modSigSepStr= modSigStr + "----------";  

  strcpy (modSig, modSigStr.c_str());
  strcpy (modSigSep, modSigSepStr.c_str());
  
  assert ( (angleMidRay >= 0) && (angleMidRay < 360) );
  assert ( (angleHalfWidth > 0) && (angleHalfWidth <= 180) );
}

// ----------
// Traffic functiosn
void  Node::trafficVarInit() {
  double minValidSlotLen;

  maxPktTime=  (double) (8 * maxPktLen)/chanRate;   // in sec  
  simPerSlotPktCount= 0;	 // incremented within each time slot, and
  		      		 // reset at the beginning of a time slot

  // check slotLen
  minValidSlotLen= maxPktTime * (1+ numPktsPerSlot * (1 + pktIa));
  if (minValidSlotLen >= slotLen) {
      FATAL("%s: initialize(): minValidSlotLen (= %g) < slotLen (= %g)\n",
                 getFullName(), minValidSlotLen, slotLen);
  }
}  

// ----------
// NET functions
void  Node::netVarInit() {
  std::string  str_staticRouting;

  seqNo= 0;
  str_staticRouting= par("staticRoutingStr").stdstringValue();
  buildRouteTable (str_staticRouting);
}

// ----------
// MAC functions
void  Node::macVarInit() {
  assert (macBackoffOrder > 0);
  macTxQueue.clear(); macState= MAC_IDLE;
}

// -----
void Node::macTimerBackoffStart () {
  int     backoffSlot;
  double  backoffTime;

  // See if MAC is idle
  if ( (macState == MAC_IDLE) && (macTxQueue.size() > 0) ) {

      macState= MAC_BUSY;
      backoffSlot= intuniform (0,macBackoffOrder-1);

      if (backoffSlot == 0) {
          macTimerBackoffEnd();
      }
      else {
          backoffTime= backoffSlot * maxPktTime;
          setTimer (TIMER_MAC_BACKOFF_END,
                    SimTime ( backoffTime * 1E6, SIMTIME_US) );
          stat.vw[S_MSG_MAC_BOFF_DELAY].newVal(backoffTime);
      }		    
  }
  // else return
}
// -----
void Node::macTimerBackoffEnd() {
  Msg_Mac *msgOut;
  
  // xmit head of the queue & wait for one maxPktTime to finish transmission
  //
  stat.vw[S_MAC_TXQ_BOFF_END_SIZE].newVal(macTxQueue.size());
  msgOut= macTxQueue[0];  macTxQueue.pop_front();
  eMac2wi_sendMsg (msgOut);

  setTimer (TIMER_MAC_TX_END,
   	     SimTime (maxPktTime * 1E6, SIMTIME_US) );
}	     
// ----------
// Energy functions
//
void  Node::eVarInit() {
  if (eInitial > eStorageCap) {
      WARNING("%s: eVarInit(): eInitial (=%g) > eStorageCap (=%g)\n",
               getFullName(), eInitial, eStorageCap);
  }      
  eCurrent= eInitial;
  eMaxPktConsume= 8 * maxPktLen * eTxPerBit;    // in mWH
  eHarvestWastePerSlot= 0;
  eHarvestLastUpdate= simTime();
}

// ----------
void Node::eHarvestUpdate() {
  double     delta, amount;
  simtime_t  diff;

  diff=    simTime() - eHarvestLastUpdate;

  // This sequence is wrong (delta is sometimes negative)
  // delta=   SimTime ( diff.raw() * 1E6, SIMTIME_US).dbl();
  // amount= (delta/ 1E6) * eHarvestRate;	     //eHarvestRate is in mWSec 

  delta= diff.dbl();
  amount= delta * eHarvestRate;    //eHarvestRate is in mWSec
  
  if (logLevel <= 2) {
      printf ("%s eHarvestUpdate: simTime= %g, eHarvestLastUpdate= %g \n",
      	      modSig, simTime().dbl(), eHarvestLastUpdate.dbl());

      printf ("%s diff= %g, delta= %g, amount= %g \n",
               modSig, diff.dbl(), delta, amount);
  }
  stat.vw[S_ENG_HARVEST_SMALL].newVal(amount);
  
  eCurrent += amount;

  if (eCurrent > eStorageCap) {
     eHarvestWastePerSlot += (eCurrent - eStorageCap);
     eCurrent= eStorageCap;
  }     

  eHarvestLastUpdate= simTime();
}
// ----------
// eSlotStartUpdate: operations done at each slot start

void  Node::eSlotStartUpdate() {

  stat.vw[S_ENG_PER_SLOT].newVal(eCurrent);
  stat.vw[S_ENG_WASTE_PER_SLOT].newVal(eHarvestWastePerSlot);
  eHarvestWastePerSlot= 0;
  stat.vw[S_MAC_TXQ_SIZE].newVal(macTxQueue.size());
}

// ----------
// eMac2wi_sendMsg:  A central point responsible for sending a msg
// (if enough energy exists). Called from from the mac layer (macTxQueue)..
//
void  Node::eMac2wi_sendMsg (Msg_Mac *msgOut) {
  char	    buf[MAXLINE];
  pair_int  pair_sn;	      // a pair of statistic and nodeAddr  
  MacPkt_t  macPkt;
  NetPkt_t  netPkt;

  eHarvestUpdate();	    // update harvested energy since last update
  if (eCurrent < eMaxPktConsume) {
      stat.c[S_MSG_ENG_DROP]++; 	// increment dropped msgs
      delete msgOut;
      return;
  }
  // else
  send(msgOut, "ioWC$o");

  macPkt= msgOut->getMacPkt();
  netPkt= macPkt.frame;      

  stat.c[S_MSG_MAC_SENT]++;
  pair_sn= std::make_pair (S_MSG_MAC_SENT, netPkt.netDestAddr);
  stat.cWithDestAddr[pair_sn]++;
  
  eCurrent -= eMaxPktConsume;

  if (logLevel <= 2) {
      memset(buf, 0, sizeof(buf));
      printf("\n");
      printf ("%s sent: time= %s (%g) \n", modSig,
                    simTime().str().c_str(), simTime().dbl());
      Util_printMsg (msgOut, modSig, 0, "sent ");
      Util_printPkt (netPkt, modSig, 0, "included: ");
      printf ("%s\n", modSigSep);
  }
}  

//##newpage
// ----------
void Node::print_node (int margin, const char *prefix) {
  int          i, destAddr, nextHop;
  double       cost;
  std::string  str;
  map_int2routeEntry::iterator  It;

  printf("\n");
  printf("%s", modSig); newWS(margin,' ');
  printf("%s", prefix);

  str= Util_dequeInt2str(destSet);

  printf("%s", modSig); newWS(margin,' ');  
  printf("(nodeAddr= %d): wiChanAddr= %d, xLoc= %g, yLoc= %g \n",
	  nodeAddr, wiChanAddr, xLoc, yLoc);

  printf("%s", modSig); newWS(margin,' ');
  printf("angleMidRay= %g, angleHalfWidth= %g, txRange= %g, destSet= %s\n",
	  angleMidRay, angleHalfWidth, txRange, str.c_str());

  printf("%s", modSig); newWS(margin,' ');
  printf("chanRate= %d (bps), Msg_Mac= %ld (bytes), maxPktLen= %d (bytes)\n",
          chanRate, sizeof(Msg_Mac), maxPktLen);

  printf("%s", modSig); newWS(margin,' ');
  printf("sinkAddr= %d, bcastAddr= %d \n", sinkAddr, bcastAddr);

  printf("%s", modSig); newWS(margin,' ');
  printf("slotLen= %g, numPktsPerSlot= %d, pktIa= %d \n",
          slotLen, numPktsPerSlot, pktIa);

  printf("%s", modSig); newWS(margin,' ');
  printf("eCurrent= %g (mWH), eStorageCap= %g (mWH), eTxPerBit= %g (mWH), \n",
          eCurrent, eStorageCap, eTxPerBit);

  printf("%s", modSig); newWS(margin,' ');
  printf("eMaxPktConsume= %g (mWH) \n", eMaxPktConsume);

  printf("%s", modSig); newWS(margin,' ');
  printf ("Routing Table: \n");

  printf("%s", modSig); newWS(margin,' ');
  for (It= routeTable.begin(); It != routeTable.end(); It++) {
      destAddr= It->first;
      nextHop=  (It->second).nextHop;
      cost=     (It->second).cost;
      printf ("(dest= %d, nextHop= %d, cost= %g), ", destAddr, nextHop, cost);
  }
  printf("\n");
  printf("%s \n", modSigSep);
}

//##newpage
// --------------------
void  Node::setTimer (int timerIdx, simtime_t time)
{
  if ((timerIdx < 0) || (timerIdx >= TIMER_MAX_SIZE))
      FATAL ("TimeService::setTimer(): fatal: timerIdx= %d\n", timerIdx);

  cancelTimer(timerIdx);

//printf("*** debug: Node:setTimer(): cancel timer\n");
  timerMessages[timerIdx]=
      new Msg_TimerService ("Timer_message", MSG_TIMER);
  timerMessages[timerIdx]->setTimerIndex(timerIdx);
  scheduleAt (simTime() + time, timerMessages[timerIdx]);
}    
// --------------------

void Node::cancelTimer (int timerIdx)
{
  if ((timerIdx < 0) || (timerIdx >= TIMER_MAX_SIZE))
      FATAL ("TimeService::cancelTimer(): fatal: timerIdx= %d\n", timerIdx);

//printf("*** debug: Node::cacnelTime(): timerIdx= %d\n", timerIdx);        
  Msg_TimerService* tmp= timerMessages[timerIdx];

  if ( (tmp != NULL) &&  tmp->isScheduled() ) cancelAndDelete (tmp);
  timerMessages[timerIdx] = NULL;
}
// --------------------

simtime_t Node::getTimer (int timerIdx)
{
  if ((timerIdx < 0) || (timerIdx >= TIMER_MAX_SIZE))
      FATAL ("TimeService::getTimer(): fatal: timerIdx= %d\n", timerIdx);

  if (timerMessages[timerIdx] == NULL) return -1;
  else
      return timerMessages[timerIdx]->getArrivalTime();
}
// --------------------
