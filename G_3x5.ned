// ------------------------------
// Network file for EH-WSN project
//
// E. Elmallah: July 2023
// ------------------------------

network G_3x5 {	
    parameters:
        int numNodes= 15;	// a 5x3 grid network with sink at the origin
	int xGridSize= default(50);
	int yGridSize= default(30);

        string staticRoutingStr= "(1,21,22,23) (2,22,23) (3,23,22,21,20) \
	       			  (4,24,23,22,21,20)";

    submodules:
	wiChan:  WiChan; 	// the wireless channel

        // Row 0 of the grid
	sink:   Node { nodeAddr= 111; xLoc= 5; yLoc= 5;  txRange= 100;
		       destSet= "10, 20"; }	
	n15_5:  Node { nodeAddr= 1; xLoc= 15; yLoc= 5; txRange= 15;
		       destSet= "11, 21"; }
	n25_5:  Node { nodeAddr= 2; xLoc= 25; yLoc= 5; txRange= 15;
	               destSet= "12, 22"; }
	n35_5:  Node { nodeAddr= 3; xLoc= 35; yLoc= 5; txRange= 15;
	               destSet= "13, 23"; }
	n45_5:  Node { nodeAddr= 4; xLoc= 45; yLoc= 5; txRange= 15;
	               destSet= "14, 24"; }
	
        // Row 1 of the grid
	n5_10:   Node { nodeAddr= 10; xLoc= 5;  yLoc= 10; txRange= 15; }	
	n15_10:  Node { nodeAddr= 11; xLoc= 15; yLoc= 10; txRange= 15; }
	n25_10:  Node { nodeAddr= 12; xLoc= 25; yLoc= 10; txRange= 15; }
	n35_10:  Node { nodeAddr= 13; xLoc= 35; yLoc= 10; txRange= 15; }
	n45_10:  Node { nodeAddr= 14; xLoc= 45; yLoc= 10; txRange= 15; }
	
        // Row 2 of the grid
	n5_15:   Node { nodeAddr= 20; xLoc= 5;  yLoc= 15; txRange= 15;
		        logLevel= 2;}	
	n15_15:  Node { nodeAddr= 21; xLoc= 15; yLoc= 15; txRange= 15;
		        logLevel= 2;}	
	n25_15:  Node { nodeAddr= 22; xLoc= 25; yLoc= 15; txRange= 15;
		        logLevel= 2;}		
	n35_15:  Node { nodeAddr= 23; xLoc= 35; yLoc= 15; txRange= 15;
		        logLevel= 2;}		
	n45_15:  Node { nodeAddr= 24; xLoc= 45; yLoc= 15; txRange= 15;
		        logLevel= 2;}		

        // node[numNodes]: Node;    // old topology

    connections:
        // for i= 0..numNodes-1 { node[i].ioWC <--> wiChan.ioNode[i]; } //old topology
	// Row 0 connections
        sink.ioWC <--> wiChan.ioNode++;
        n15_5.ioWC <--> wiChan.ioNode++;
	n25_5.ioWC <--> wiChan.ioNode++;
	n35_5.ioWC <--> wiChan.ioNode++;
	n45_5.ioWC <--> wiChan.ioNode++;	

	// Row 1 connections
        n5_10.ioWC  <--> wiChan.ioNode++;	
        n15_10.ioWC <--> wiChan.ioNode++;
	n25_10.ioWC <--> wiChan.ioNode++;
	n35_10.ioWC <--> wiChan.ioNode++;
	n45_10.ioWC <--> wiChan.ioNode++;	

	// Row 2 connections
        n5_15.ioWC  <--> wiChan.ioNode++;	
        n15_15.ioWC <--> wiChan.ioNode++;
	n25_15.ioWC <--> wiChan.ioNode++;
	n35_15.ioWC <--> wiChan.ioNode++;
	n45_15.ioWC <--> wiChan.ioNode++;	
}
// ------------------------------ 
