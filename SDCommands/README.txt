// Setup Guide

// config.txt
// Links events to perform actions
// Events 100+ are serialRX events
// Actions 1000+ are serialTX commands

EVENT | ACTION,ACTION,ACTION...
000 | 0;
001 | 1,2,5,6,11,1013,1014,1016,1018;
002 | 1,3,5,6;


// serialRX.txt
// Links received serial commands to events
// PORT is port command received on

EVENT | PORT[COMMAND]
100 | 4[red];
101 | 4[green];
102 | 4[blue];


// serialTX.txt
// Links serial commands to actions

ACTION | PORT[COMMAND]
1000 | 4[MONGOOSE];
1001 | 4[JAGUAR];
1002 | 4[PYTHON];
1003 | 4[MUTE];
