# ConCReCT
Mechanism for mitigating a Denial of Service Volumetric attack against IoT based on ContikiOS and validated with Cooja that uses Duty-Cycle control to silence malicious nodes.


ConCReCT
========

Vladimir Borgiani & Emilio R. Tubino

Developed using contiki 2.7 and validated with Cooja simulator

The goal of this code is to allow a sink server to identify a traffic increase (not compatible with regular network traffic) and mute the malicious node.

TEN -> Traffic Expected at node N
TMN -> Traffic Measured at node N

Server
======

	All matrix data can be viewed at Cooja Mote Output using filter word "SISTEMA: ..."
	
	At each action taken against the malicious node (mute), the matrix is printed showing each node status
	
	Max number of nodes is defined at server code as shown below setting L (matrix lines), while C is the number of collumns (attributes like id, number of children, traffic rate and 

		#define L 20		// 20 max number of nodes =  20 children
                    // Use over 20 causes memory issues at tmote Skyso we are limited to 20 nodes at matrix
		#define C 4			// number of collumns to store data about nodes
	
	#define PERIOD 180 		// Time to reset penalty counter back to zero
	
		Node can send max of TEN during interval 3 consecutive times
	
	#define avisoM 3	// Max number of times they can send more than allowed traffic
		If TEN >= TMN penalty is incremented
		If penalty >=3 Server send "Wait" to node and he is muted by 240 seconds period
			
			Important to notice that muted node still carry children nodes traffic
			This is good as child nodes dont need to search for another parent
	
	Right click at server and choose Click button prints the matrix, with nodes status, at Mote output window


Client
======

	Every client can be an attacker
		Right clicking a node and choosing click button changes Hello period from 1/min to 1/sec
	
	Normal hello timing is 1/min, hello attack timing is 1/sec and mute period is 240 sec

	Node in mute state still can forward children messages to server!!! AWESOME!
	
	Print child list every time it sends a message


Filters to apply to mote output in order to better view and troubleshoot issues
===============================================================================

NO: 	  	Information about child nodes
SISTEMA:	All server and client important information
DATA: 		Data exchanged
MATRIZ:   	Server matrix information
