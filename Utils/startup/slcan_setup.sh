#! /bin/bash

# ASCII Command vs CAN Bitrate
# s0 --- s1 --- s2 --- s3 --- s4 --- s5 --- s6 --- s7 --- s8
# 10     20     50     100    125    250    500    800    1000  Kbits/s
BAUD=8

# Name of slcan device to attach
DEV=can0


# Detect correct port for device interfaces
TERM_PORT=$(ls -l /dev/serial/by-id/ | grep 1fc9_00a3 | sed 's/.*\///g' | awk '{if(NR==2) print $0}')
CANB_PORT=$(ls -l /dev/serial/by-id/ | grep 1fc9_00a3 | sed 's/.*\///g' | awk '{if(NR==1) print $0}')

# Start or stop the service
while getopts ":ks" opt; do
    case $opt in
        k)
            # Stop the SLCAN service and turn off the can device
            echo "Shutting down can interface..."
            sudo ifconfig $DEV down 2> /dev/null
            sudo pkill slcand

            sudo slcand -c /dev/"$CANB_PORT"
            sudo pkill slcand
            ;;
        s)
            # Open the can device, and start the slcan service.
            echo "Terminal interface on: $TERM_PORT"
            echo "CAN Bus  interface on: $CANB_PORT"

            # Set the mode to slcan
            echo -ne "set can-mode slcan" > /dev/$TERM_PORT

            echo "Attaching slcan device..."

            # Attach to the port with slcand
            sudo slcand -s$BAUD -o /dev/"$CANB_PORT" $DEV

            # Give interface time to come up
            sleep .25

            # Enable the inteface
            sudo ifconfig $DEV up
            sudo ifconfig $DEV txqueuelen 1000

            echo "Interface is set up."
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            exit 1
            ;;
    esac
done
