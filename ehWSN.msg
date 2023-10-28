// ------------------------------
// ehWSN.msg - file of all messages for the EH-WSN project
//
// E. Elmallah: July 2023
// ------------------------------
// Note: This file may contain enumerate structures, C-structs, OMNET++
// 	 packets, and messages (see, e.g., Castalia: MacPacket.msg)
// ------------------------------

message Msg_TimerService {
    int timerIndex;
}

// --------------------
struct NodeInfo_t {
    int   srcAddr;
    int   destAddr;
    double txRange;
    double xLoc;
    double yLoc;
    double angleMidRay;	    // in degrees
    double angleHalfWidth;  // in degrees
}

message Msg_Loc {	    // used in, e.g., MSG_LOC_UP, MSG_LOC_REPLY, etc.
     NodeInfo_t  nodeInfo;
}
// --------------------
struct AppPkt_t {
    int  srcAddr;
    int  destAddr;
    int  seqNo;
    char data[80];	// MAXLINE
}

message Msg_App {
    AppPkt_t   appPkt;
}    
// --------------------
struct NetPkt_t {
    int  netSrcAddr;
    int  netDestAddr;
    int  seqNo;
    AppPkt_t   frame;
}

message Msg_Net {
    NetPkt_t   netPkt;
}    
// --------------------
struct MacPkt_t {
    int  macSrcAddr;
    int  macDestAddr;
    NetPkt_t   frame;
}

message Msg_Mac {
    MacPkt_t   macPkt;
}    
// --------------------
