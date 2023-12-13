#!/bin/bash

touch /tmp/empty

cat $1 | while read a ; do
  ~/o/openmrn/applications/bootloader_client/targets/linux.x86/bootloader_client -r -t -f /tmp/empty -i localhost -n 0x0501010185$a
done
               
               
