#!/bin/bash
NETIF="eth1"
ATPORT="/dev/ttyUSB2"


chmod +x ./mgattool

V6CLI=`which dibbler-client`
V4CLI=`which udhcpc`
if [ -z $V6CLI ]; then
    echo "not dibbler-client"
    echo "should install:sudo apt-get install dibbler-client"
    exit
fi

if [ -z $V4CLI ]; then
    echo "not udhcpc found"
    echo "should install:sudo apt-get install udhcpc"
    exit
fi

while true
do
	QMIDEV=`ls /dev/qcqmi*`
        if [ ! -z $QMIDEV ]; then
		echo "got $QMIDEV"
                break;
        fi
        echo "wait..."
        sleep 0.2
done


ifconfig ${NETIF} down
#echo -e "at\r\n" > /dev/ttyUSB2
#echo -e "at\$qcrmcall=1,1,3,2,1\r\n" > /dev/ttyUSB2
./mgattool $ATPORT at
./mgattool $ATPORT "at\$qcrmcall=1,1,3"

ifconfig ${NETIF} up

udhcpc -i $NETIF
dibbler-client start
dibbler-client run

