# ConCReCT
Mechanism for mitigating a Denial of Service Volumetric attack against IoT based on ContikiOS and validated with Cooja that uses Duty-Cycle control to silence malicious nodes. There is a new distributed version of the code where every node is responsible for it is child nodes (nrighbors that use him as a parent to get to sink node). 

Centralized: Sink builds a matrix and monitors all nodes (limited by mote memory)
Distributed: Every node builds a matrix and monitors his child nodes, so workload is distributed and memory is lo longer a problem (as long as each node has less than 20 child nodes). Detection and mitigtation are also faster as attacker and detector/mitigator are only 1 hop away from each other.


ConCReCT
========

Vladimir Borgiani & Emilio R. Tubino

Developed using contiki 2.7 and validated with Cooja simulator

The goal of this code is to allow a sink server to identify a traffic increase (not compatible with regular network traffic) and mute the malicious node.

Works better on contiki version named contiki-ids-ids built by Sahid Raza at SVELTE mechanism!!!

TEN -> Traffic Expected at node N
TMN -> Traffic Measured at node N

Sink creates the dodag and clients joint it. Monitors and account when TMN > TEN for more than 3 consecutive times and then send a WAIT message that mutes offensive node. After while mote awake and monitor starts over (penalty counter is cleared). 
