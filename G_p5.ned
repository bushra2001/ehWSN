// ------------------------------
// Topology file for the EH-WSN project
//
// E. Elmallah: July 2023
// ------------------------------

network G_p5 {		// a path on 5 nodes
    parameters:
        int numNodes= 5;	// a 5x1 grid network with sink at the origin
	int xGridSize= 50;	// meters
	int yGridSize= 10;

	string staticRoutingStr= "(1,2,3,4,5)";
        //string staticRoutingStr= "(1,2,3) (5,4,3)";

    submodules:
	wiChan:  WiChan; 	// the wireless channel

        // Row 0 of the grid
	n5_5:   Node { nodeAddr= 1; xLoc= 5; yLoc= 5;  txRange= 15;
		       destSet= "5"; logLevel= 2;}	
	n15_5:  Node { nodeAddr= 2; xLoc= 15; yLoc= 5; txRange= 15;
		       destSet= ""; logLevel= 2;}
	n25_5:  Node { nodeAddr= 3; xLoc= 25; yLoc= 5; txRange= 15;
	               destSet= ""; logLevel= 2;}
	n35_5:  Node { nodeAddr= 4; xLoc= 35; yLoc= 5; txRange= 15;
	               destSet= ""; logLevel= 2;}
	n45_5:  Node { nodeAddr= 5; xLoc= 45; yLoc= 5; txRange= 15;
	               destSet= "3,4"; logLevel= 2; }
	
        // node[numNodes]: Node;    // old topology

    connections:
        // for i= 0..numNodes-1 { node[i].ioWC <--> wiChan.ioNode[i]; } //old topology
	// Row 0 connections
        n5_5.ioWC <--> wiChan.ioNode++;
        n15_5.ioWC <--> wiChan.ioNode++;
	n25_5.ioWC <--> wiChan.ioNode++;
	n35_5.ioWC <--> wiChan.ioNode++;
	n45_5.ioWC <--> wiChan.ioNode++;	
}
// ------------------------------ 
