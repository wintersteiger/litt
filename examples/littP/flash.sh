#!/bin/bash

/home/cwinter/openocd/src/openocd -d2 -s /home/cwinter/opentherm -f interface/picoprobe.cfg -f target/rp2040.cfg -s /home/cwinter/openocd/tcl -c "program littP.elf verify reset exit"
