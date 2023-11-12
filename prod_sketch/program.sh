#!/bin/bash

BINFILE="$1"
PORT="$2"

export CALL="$0"

function usage() {
    echo usage: "${CALL}" download.bin [/dev/serial-port] >&2
    echo Downloads the given binary using the bootloader protocol, talking to a USB-CAN converter on the given serial port. >&2
    exit 1
}


if [ ! -f "${BINFILE}" ] ; then usage ; fi

echo Input file "${BINFILE}"

if [ -z "${PORT}" ] ; then
    # attempts to find a serial port we like
    if [ -c /dev/serial/by-id/usb-STMicroelectronics_GENERIC_F072CBTX_CDC_in_FS_Mode_*-if00 ] ; then
        PORT=/dev/serial/by-id/usb-STMicroelectronics_GENERIC_F072CBTX_CDC_in_FS_Mode_*-if00
    elif [ -c /dev/cu.usbmodem* ] ; then
        PORT=/dev/cu.usbmodem*
    elif [ -c /dev/serial/by-id/usb-OpenMRN_Virtual_COM_Port_* ] ; then
        PORT=/dev/serial/by-id/usb-OpenMRN_Virtual_COM_Port_*
    else
        echo Error: could not find a known USB serial port. Specify the port on the command line. '\n\n' >&2
        usage
    fi
    
fi

echo Serial port: ${PORT}

exec 4<> ${PORT}

function send_packet() {
    echo ${1} >&4
    if [ -z "${QUIET}" ]; then echo ">  ${1}" ; fi
}

function drop_input() {
    while read -u 4 -t 0.1 THROWAWAY ; do
        echo "<i ${THROWAWAY}"
    done
}

function get_packet() {
    read -u 4 -t 2
    RET=$?
    echo "<  ${REPLY}"
    drop_input
    return $RET
}

function send_event() {
    send_packet ":X195B4002N$1;"
}

send_packet ":X19490002N;"
if ! get_packet ; then
    echo Target node not found >&2
    exit 1
fi
if [[ "${REPLY}" =~ ^:X19170...N(.*)\;$ ]] ; then
    echo Target node: 0x${BASH_REMATCH[1]}
else
    echo Unexpected reply from target node.
fi

# enter bootloader
send_event 09000DFF0000000E

sleep 0.35
drop_input

#start bootloading
send_event 09000DFF0000000F

sleep 0.1

COUNT=0

od -t x4 -An -v -w4 --endian=big "${BINFILE}" | while read a ; do
    QUIET=yes send_event 09000DF9$a
    sleep 0.002
    COUNT=$((COUNT + 1))
    if (( $COUNT % 512 == 0 )) ; then
        echo read $(($COUNT * 4 ))
        get_packet
        if [[ "${REPLY}" =~ ^:X195B4...N09000DFF00000011\;$ ]] ; then
            echo Write done
        else
            echo ERROR ">${REPLY}<"
        fi
    fi
done

# complete
send_event 09000DFF00000010

sleep 0.5
drop_input
