// ------------------------------
// ehWSN.h - header file for the EH-WSN project
//
// E. Elmallah: July 2023
// ------------------------------

#ifndef EH_WSN_H
#define EH_WSN_H

#include <omnetpp.h>
#include <cstdlib>
#include <cmath>

#include <random>
#include <vector>
#include <deque>
#include <map>
#include <set>
#include <algorithm>
#include "ehWSN_m.h"

// #define  COMPILETIME_LOGLEVEL LOGLEVEL_TRACE  // LOGLEVEL_DETAIL by default

using namespace omnetpp;

// --------------------
void newWS (int x, char c);                   // defined in ehWSN.cc
void WARNING (const char *fmt, ... );
void FATAL (const char *fmt, ... );
// ------------------------------
// some helper funtcions
//

#define MAXLINE	       80      // 80 characters per line
#define MAX_NTOKEN     MAXLINE // number of tokens in a string
#define MAXWORD        32      // characters per token in a string

#define MODULE_SIG_W   7      // text width of module signature string
// --------------------
// basic typedefs
//
typedef std::set<int>                          set_int;
typedef std::deque<int>                        deque_int;
typedef std::pair<int,int>                     pair_int;
typedef std::map< std::pair<int,int>, int>     map_pair2int;
typedef std::map< std::pair<int,int>, double>  map_pair2double;
typedef std::map<int,set_int>                  map_int2set_int;

// ------------------------------
int Util_split (char inStr[], char token[][MAXWORD], const char fs[]);
int Util_gsub (char *t, const char *omitSet, const char *repStr);

void        Util_testWatch (double inAvg);

std::string Util_setInt2str (std::set<int> x);
std::string Util_dequeInt2str(deque_int x);
std::string Util_formatModSig (std::string modName);

double Util_getDist (double xi, double yi, double xj, double yj);
double Util_scaleAngle (double x);
bool Util_withinAntenaRange (double xsrc, double ysrc, double txRange,
      	      	 double	midRay,	double halfWidth, double xtest,	double ytest,
		 int logLevel, const char *modSig);

void Util_composePkt (AppPkt_t& x, int nodeAddr, int destAddr, int seqNo,
		 char data[]);
void Util_composePkt (NetPkt_t& x, int nodeAddr, int netNextHopAddr, int seqNo,
                 AppPkt_t& appPkt);
void Util_composePkt (MacPkt_t& x, int nodeAddr, int macNextHopAddr,
                 NetPkt_t& netPkt);		 

void Util_composePkt (NodeInfo_t& x, int srcAddr, int destAddr,
     		      double xLoc, double yLoc,
		      double angleMidRay, double angleHalfWidth,
		      double txRange);

void Util_printPkt (AppPkt_t& x,
                    const char *modSig, int margin, const char *prefix);
void Util_printPkt (NetPkt_t& x,
                    const char *modSig, int margin, const char *prefix);
void Util_printMsg (Msg_Loc *msg,
                    const char *modSig, int margin, const char *prefix);
void Util_printMsg (Msg_Mac *msg,
                    const char *modSig, int margin, const char *prefix);
// ------------------------------
// class VWATCH: watching a sequence of values (of type double)
//               (may be similar to Boost::accumulator)

class  VWATCH {
public:
    double  lastVal, totalVal, totalVal2, minVal, maxVal;
    int     count;

    VWATCH ();
    ~VWATCH ();
    void   clear();
    void   newVal (double x);
    int    getCount ();
    double getMin ();
    double getMax ();
    double getTotal();
    double getAvg ();
    double getVar ();
    int    info2str(char str[], int size);
};

// ------------------------------
// class STAT: declares one (or more) arrays of accumlators

#define NSTAT	       S_EOL     // number of statistics (counts) collected
enum STAT_TYPE {
    S_MSG_LOC_SENT= 0 ,S_MSG_LOC_RECV,  
    S_MSG_APP_SENT, S_MSG_APP_OVERHEAR, S_MSG_APP_RECV,
    S_MSG_NET_SENT, S_MSG_NET_OVERHEAR, S_MSG_NET_RECV, S_MSG_NET_NOROUTE,
    S_MSG_MAC_SENT, S_MSG_MAC_OVERHEAR, S_MSG_MAC_RECV, S_MSG_MAC_BCAST,
    S_MSG_MAC_BOFF_DELAY,
    S_MSG_COLLISION, S_MSG_BROADCAST,
    S_MSG_ENG_DROP,
    S_SIM_SLOT_COUNT,
    S_MAC_TXQ_SIZE, S_MAC_TXQ_BOFF_END_SIZE,
    S_ENG_PER_SLOT, S_ENG_HARVEST_SMALL, S_ENG_WASTE_PER_SLOT,
    S_EOL
};

static const char *str_STAT_TYPE[]=
    { "S_MSG_LOC_SENT",  "S_MSG_LOC_RECV",
      "S_MSG_APP_SENT",  "S_MSG_APP_OVERHEAR", "S_MSG_APP_RECV",
      "S_MSG_NET_SENT",  "S_MSG_NET_OVERHEAR", "S_MSG_NET_RECV",
      "S_MSG_NET_NOROUTE",
      "S_MSG_MAC_SENT","S_MSG_MAC_OVERHEAR","S_MSG_MAC_RECV","S_MSC_MAC_BCAST",
      "S_MSG_MAC_BOFF_DELAY",
      "S_MSG_COLLISION", "S_MSG_BROADCAST",      
      "S_MSG_ENG_DROP",
      "S_SIM_SLOT_COUNT",
      "S_MAC_TXQ_SIZE", "S_MAC_TXQ_BOFF_END_SIZE",
      "S_ENG_PER_SLOT", "S_ENG_HARVEST_SMALL", "S_ENG_WASTE_PER_SLOT",
      "S_EOL"
    };      

class STAT {
public:
    int           c[NSTAT];	  //each entry counts some stat
    map_pair2int  cWithSrcAddr;   //each entry counts some stat received
    map_pair2int  cWithDestAddr;  //each entry counts some stat received  
    		  		  //from some srcAddr
    VWATCH        vw[NSTAT];	  

  STAT();
  ~STAT();
  void clear();
  void print(const char* modSig, int margin, const char* prefix);
};  
// ------------------------------
//
enum MSG_TYPE {
    MSG_TEST= 0, MSG_TIMER, MSG_LOC_UP, MSG_LOC_REPLY,
    MSG_APP, MSG_NET, MSG_MAC
};
static const char *str_MSG_TYPE[]=
    { "MSG_TEST", "MSG_TIMER", "MSG_LOC_UP", "MSG_LOC_REPLY",
      "MSG_APP", "MSG_NET", "MSG_MAC" };
// ----------

#define TIMER_MAX_SIZE 32    // entries in a timer vector

enum MSG_TIMER_TYPE {
    TIMER_TEST= 0, TIMER_SETUP,
    TIMER_SLOT, TIMER_INSLOT_IDELAY, TIMER_INSLOT_RDELAY,
    TIMER_MAC_BACKOFF_END, TIMER_MAC_TX_END
}; 
static const char *str_MSG_TIMER_TYPE[]=
    { "TIMER_TEST", "TIMER_SETPUP",
      "TIMER_SLOT", "TIMER_INSLOT_IDELAY", "TIMER_INSLOT_RDELAY",
      "TIMER_MAC_BACKOFF_END", "TIMER_MAC_TX_END"
    };
// --------------------
// MAC states
enum MAC_STATE { MAC_IDLE, MAC_BUSY };

static const char *str_MAC_STATE[]= { "MAC_IDLE", "MAC_BUSY" };

// --------------------
// Help structs

struct nodeWiInfo_t {
    int	    nodeAddr;	    
    double  xLoc, yLoc, angleMidRay, angleHalfWidth, txRange;
    int     xCellIdx, yCellIdx, xyCellIdx; //indexes of the cell containing the node
    int     wiChanGateIdx;	     // out gate from WiChan to the node
    set_int nbSet;   	             // set of cell neighbours of the xyCell
};

struct recvPkt_t {
    int        srcAddr, destAddr;
    simtime_t  txStart, txEnd;	  // record start/end of arrival times
    cMessage   *msg;              // pointer to a received pkt
};

struct routeEntry_t {
    int    nextHop;
    double cost;
};    
// --------------------
// more tyedefs

typedef std::map<int, nodeWiInfo_t>  map_int2nodeWiInfo;
typedef std::map<int, routeEntry_t>  map_int2routeEntry;
typedef std::deque< recvPkt_t >      deque_recvPkt;

typedef std::deque< Msg_Mac *>       deque_macMsg;
// --------------------
#endif
