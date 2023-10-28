// ------------------------------
// Util.cc - file for the EH-WSN project
//
// E. Elmallah: July 2023
// ------------------------------

#include "ehWSN.h"	// includes <omentpp.h> and other header files

// --------------------
//  class VWATCH

     VWATCH::VWATCH()  { clear(); }
     VWATCH::~VWATCH() {}

void VWATCH::clear() {
  lastVal= totalVal= totalVal2= 0.0;
  minVal= (double) RAND_MAX; maxVal= -minVal; count= 0;
}

void VWATCH::newVal (double x) {
  lastVal= x; totalVal += x; totalVal2 += x*x;
  minVal= (x < minVal)? x : minVal;
  maxVal= (x > maxVal)? x : maxVal;
  count++;
}

int    VWATCH::getCount () {return count;    }
double VWATCH::getMin ()   {return minVal;   }
double VWATCH::getMax ()   {return maxVal;   }
double VWATCH::getTotal()  {return totalVal; }

double VWATCH::getAvg () {
  if (count == 0) WARNING ("VWATCH::getAvg: warning count=0 \n");
  return (count > 0)? totalVal/count: -999;
}

double VWATCH::getVar () {
  if (count == 0) WARNING ("VWATCH::getVar: warning count=0 \n");
  return (count > 0)? (totalVal2/count)-pow(getAvg(),2):-999;
}

int VWATCH::info2str(char str[], int size) {
  int  len; memset(str,0,size);
  len  = sprintf(str, "(count= %d) ", count);
  len += sprintf(&str[len],"(totalVal=%.5g, minVal=%.5g, maxVal= %.5g) ",
                            totalVal,minVal,maxVal);
  len += sprintf(&str[len],"(avgVal= %.5g, Var= %.5g)",
                             getAvg(), getVar());
  return len;
}
// --------------------
// class STAT

     STAT::STAT()  { clear(); }
     STAT::~STAT() { }
     
void STAT::clear() {
  for (int i= 0; i < NSTAT; i++) { c[i]= 0; vw[i].clear(); }
  cWithSrcAddr.clear();
  cWithDestAddr.clear();  
}

void STAT::print(const char *modSig, int margin, const char *prefix) {
  int        i, len;
  char	     str[2*MAXLINE];
  pair_int   key;
  map_pair2int::iterator  It;

  printf ("%s", modSig); newWS(margin,' ');
  printf("%s", prefix);

  for (i= 0; i < NSTAT; i++) {
      if (c[i] == 0) continue;
      printf("[%s= %d], ", str_STAT_TYPE[i], c[i]);
  }

  printf ("\n%s\n", modSig);
  for (It= cWithSrcAddr.begin(); It != cWithSrcAddr.end(); It++) {
      key= It->first;
      printf ("%s", modSig); newWS(margin,' ');        
      printf ("[(%s, srcAddr= %d)= %d] \n", str_STAT_TYPE[key.first],
      	         key.second, cWithSrcAddr[key]);
  }

  printf ("\n%s\n", modSig);
  for (It= cWithDestAddr.begin(); It != cWithDestAddr.end(); It++) {
      key= It->first;
      printf ("%s", modSig); newWS(margin,' ');        
      printf ("[(%s, destAddr= %d)= %d] \n", str_STAT_TYPE[key.first],
      	         key.second, cWithDestAddr[key]);
  }

  printf ("%s\n", modSig);
  for (i= 0; i < NSTAT; i++) {
      if (vw[i].getCount() == 0) continue;
      printf ("%s", modSig); newWS(margin,' ');
      len= vw[i].info2str(str,sizeof(str));
      printf("vw[%s]: %s\n", str_STAT_TYPE[i], str);
  }
  printf("\n");
}  
// --------------------
// Utility functions

int Util_split (char inStr[], char token[][MAXWORD], const char fs[]) {
  int   i, count;
  char  *tokenp, inStrCopy[MAXLINE];

  count= 0;
  memset (inStrCopy, 0, sizeof(inStrCopy));
  for (i= 0; i < MAX_NTOKEN; i++) memset (token[i], 0, sizeof(token[i]));

  strcpy(inStrCopy, inStr);
  if ( (tokenp= strtok(inStr, fs)) == NULL) return(0);

  strcpy (token[count],tokenp); count++;

  while ( (tokenp= strtok(NULL,fs)) != NULL) {
      strcpy(token[count],tokenp); count++;
  }
  strcpy (inStr, inStrCopy);
  return(count);
}  
// --------------------
// gsub - Replace in string t each occurrence of a character in omitSet with
// 	  the replacement string in "repStr".
//	  If omitSet starts with '^', substitution is done until the first
//	  ordinary character is encountered.
//	  Return the number of substitutions made

int Util_gsub (char *t, const char *omitSet, const char *repStr)
{
    int   i,j, ocSeen, match, nmatch;
    char  outStr[MAXLINE];

    enum { MBEGIN= 1, MALL= 2} task;
    task= (omitSet[0] == '^')? MBEGIN : MALL;
    memset (outStr, 0, sizeof(outStr));

    nmatch= 0; ocSeen= 0;	       // Ordinary character not seen yet
    for (i= 0; i < strlen(t); i++)
    {
	if ( (task == MBEGIN) && (ocSeen == 1)) {
	    outStr[strlen(outStr)]= t[i]; continue;
	}

	match= 0;		// no match found yet for character t[i]
    	for (j= 0; j < strlen(omitSet); j++)
	{
	    if (t[i] == omitSet[j])
	       { match= 1; nmatch++; strcat(outStr, repStr); break; }
	}
	if (match == 0) { ocSeen= 1; outStr[strlen(outStr)]= t[i]; }
    }
    // memset (t, 0, sizeof(t));
    strcpy (t, outStr);
    return(nmatch);
}
//##newpage
// --------------------
void Util_testWatch (double inAvg) {
  int  r;
  char str[2*MAXLINE]; VWATCH w;
  //cRNG  rng;
  
  for (int i=0; i < 20; i++) {
      	   // Note: intuniform apperas to work only when called from a class
	   // derived from cSimpleModule
      //r= intuniform ((int) inAvg-10, (int) inAvg+10);
      w.newVal( (double) r);
      //w.newVal(exponential(inAvg));
  }      
  int len= w.info2str(str, MAXLINE);
  newWS(2,'\n'); printf ("test_watch (strlen= %d): %s\n", len, str);
}
// ------------------------------
std::string Util_setInt2str (set_int x) {
    char               buf[MAXWORD];
    std::string        str= "(";
    set_int::iterator  sit;

    for (sit = x.begin(); sit != x.end(); sit++) {
        if (sit == x.begin()) { sprintf(buf, "%d", *sit);   str += buf; }
	else                  { sprintf(buf, ", %d", *sit); str += buf; }
    }
    str += ")";
    return str;
}
// ------------------------------
std::string Util_dequeInt2str (deque_int x) {
    int	    	       idx;	      
    std::string        str= "(";

    for (idx= 0; idx < x.size(); idx++) {
        if (idx == 0) { str += std::to_string(x[idx]); }
	else          { str += ", " + std::to_string(x[idx]); }
    }
    str += ")";
    return str;
}
// ------------------------------
// format a module's full name to a signature string that
// is printed in the output

std::string  Util_formatModSig (std::string modName) {
  int          i;
  std::string  outStr;

  outStr=
  (modName.length() < MODULE_SIG_W)? modName : modName.substr(0,MODULE_SIG_W-1);

  for (i= modName.length(); i < MODULE_SIG_W; i++) outStr += " ";
  outStr += "|";
  return outStr;
}


// ------------------------------
// Util_getDist: compute Euclidean distance between 2 points in 2D

double Util_getDist (double xi, double yi, double xj, double yj) {
  double dist;
  dist= sqrt( pow( (xj-xi),2 ) + pow( (yj-yi),2 ) );
  return dist;
}

// ----------
// Util_scaleAngle: convert a positive (CCW) or negative (CW) angle
//   of any value to a positive angle in range [0,360)
//

double Util_scaleAngle (double x) {
    x= fmod(x,360); if (x < 0) x += 360; return fabs(x);
}

//##newpage
// ----------
// Util_withinAntenaRange: returns true if point (xtest,ytest) lies
//     within txRange of transmitter at point (xLoc,yLoc) and in the
//     cone defined by the positive angles (in degrees) (midRay,halfWidth).

bool Util_withinAntenaRange (double xLoc, double yLoc, double txRange,
                 double midRay, double halfWidth, double xtest, double ytest,
		 int logLevel, const char* modSig)
{
  bool	  result= false;
  double  dx, dy;
  double  angleRad, angleDeg, ccwRay, cwRay;

  // 1. translate point (xtest,ytest) as if (xLoc,yLoc) is the origin
  dx= xtest - xLoc;
  dy= ytest - yLoc;

  // 2. find the angle of the vector from origin to (xtest,ytest)
  //
  angleRad= atan2(dy, dx);
  angleDeg= (angleRad * 180) / M_PI;    // M_PI defined in <cmath>

  // 3. find the angles (in degrees) of the CCW and CW rays
  //    around the center of the beam
  //
  ccwRay    = midRay - halfWidth;
  cwRay     = midRay + halfWidth;

  // 4. convert all angles to be positive (CCW) in the range [0,360)
  //
  angleDeg= Util_scaleAngle (angleDeg);
  ccwRay=   Util_scaleAngle (ccwRay);
  cwRay=    Util_scaleAngle (cwRay);

  // 5. decide if the vector to (xtest,ytest) lies within the CCW and CW
  //    rays

  if (Util_getDist (xLoc,yLoc,xtest,ytest) <= txRange) {
      if (halfWidth == 180) result= true;    // omnidirectional antena
      else if (ccwRay < cwRay) {
          if ( (angleDeg >= ccwRay) && (angleDeg <= cwRay) ) result= true;
      }
      else if (ccwRay > cwRay) {
          if ( (angleDeg >= ccwRay)  || (angleDeg <= cwRay) ) result= true;
      } 	  
  }

  if (logLevel <= 1) {
      printf("\n");
      printf ("%s Util_withinAntenaRange: (xLoc= %g, yLoc= %g), (xtest= %g, ytest= %g)\n",
	      modSig, xLoc, yLoc, xtest, ytest);
      printf ("%s (midRay= %g, halfWidth= %g, txRange= %g) %s\n",
	        modSig,midRay, halfWidth, txRange,
		(result == true)? "(inside)" : "(outside)" );
   }

   return result;
}

  
  
//##newpage
// ------------------------------
// composePktStructs

void Util_composePkt (AppPkt_t& x, int srcAddr, int destAddr, int seqNo,
     		     char data[])
{
     x.srcAddr= srcAddr;  x.destAddr= destAddr; x.seqNo= seqNo;
     memcpy (&(x.data[0]), data, sizeof(x.data));
     return;
}
// ----------
void Util_composePkt (NetPkt_t& x, int srcAddr, int destAddr, int seqNo,
                      AppPkt_t& frame)
{
     x.netSrcAddr= srcAddr;  x.netDestAddr= destAddr; x.seqNo= seqNo;
     x.frame= frame;
     return;
}
// ----------
void Util_composePkt (MacPkt_t& x, int srcAddr, int destAddr, NetPkt_t& frame)
{
     x.macSrcAddr= srcAddr;  x.macDestAddr= destAddr; x.frame= frame;
     return;
}
// ----------
void Util_composePkt (NodeInfo_t& x, int srcAddr, int destAddr, double xLoc,
     		      double yLoc, double angleMidRay, double angleHalfWidth,
     		      double txRange)
{
     memset ((char *) &x, 0, sizeof(x));
     x.srcAddr= srcAddr;          x.destAddr= destAddr;
     x.xLoc= xLoc;                x.yLoc= yLoc;
     x.angleMidRay= angleMidRay;  x.angleHalfWidth= angleHalfWidth;
     x.txRange= txRange;
     return;
}

// ------------------------------
// composeMsg

// ------------------------------
// printPktStructure

void Util_printPkt (AppPkt_t& x,
                    const char* modSig, int margin, const char *prefix)
{
    printf ("%s", modSig); newWS(margin,' ');
    printf ("%s", prefix);

    printf ("AppPkt_t (srcAddr= %d, destAddr= %d, seqNo= %d)\n",
             x.srcAddr, x.destAddr, x.seqNo);

    printf ("%s", modSig); newWS(margin,' ');	     
    printf ("         (data[0]= %d, data[1]= %d, data[2]= %d, ...(size= %d))\n",
    	   	       x.data[0], x.data[1], x.data[2],
		       (int) sizeof(x.data));
    //newWS(1,'\n');
}

// ------------------------------
void Util_printPkt (NetPkt_t& x,
                    const char* modSig, int margin, const char *prefix)
{
    printf ("%s", modSig); newWS(margin,' ');
    printf("%s", prefix);

    printf ("NetPkt_t (netSrcAddr= %d, netDestAddr= %d, seqNo= %d, ...)\n",
	    x.netSrcAddr, x.netDestAddr, x.seqNo);
    //newWS(1,'\n');
}

// ------------------------------
// printMsg

void Util_printMsg (Msg_Loc *msg,
                    const char* modSig, int margin, const char *prefix)
{
    NodeInfo_t   nodeInfo;        // defined in a *.msg file
    
    nodeInfo= msg->getNodeInfo(); // copy the data part

    printf ("%s", modSig); newWS(margin,' ');
    printf("%s", prefix);

    printf ("Msg_Loc(srcAddr= %d,destAddr= %d, txRange= %g, xLoc= %g,yLoc= %g)\n",
             nodeInfo.srcAddr, nodeInfo.destAddr,
             nodeInfo.txRange, nodeInfo.xLoc, nodeInfo.yLoc);

    printf ("%s", modSig); newWS(margin,' ');
    printf ("(angleMidRay= %g, angleHalfWidth= %g) \n",
             nodeInfo.angleMidRay, nodeInfo.angleHalfWidth);
    //newWS(1,'\n');
}

//----------
void Util_printMsg (Msg_Mac *msg,
                    const char* modSig, int margin, const char *prefix)
{
    MacPkt_t   macPkt;          // defined in a *.msg file
    macPkt= msg->getMacPkt();   // copy the data part

    printf ("%s", modSig); newWS(margin,' ');
    printf("%s", prefix);

    printf ("Msg_Mac (macSrcAddr= %d, macDestAddr= %d, (NetPkt_t frame) )\n",
             macPkt.macSrcAddr, macPkt.macDestAddr);
    //newWS(1,'\n');
}
// ------------------------------
