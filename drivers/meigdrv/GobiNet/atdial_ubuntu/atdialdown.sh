#!/bin/bash
NETIF="eth1"
ATPORT="/dev/ttyUSB2"




ifconfig ${NETIF} down
#echo -e "at\r\n" > /dev/ttyUSB2
#echo -e "at\$qcrmcall=0,1,3,2,1\r\n" > /dev/ttyUSB2
./mgattool $ATPORT "at"
./mgattool $ATPORT "at\$qcrmcall=0,1,3"

pkill udhcpc
dibbler-client stop

