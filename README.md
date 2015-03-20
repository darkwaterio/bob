#Bob Mk1 - "Ides"

This repository contains the hardware and firmware files for Mk1 of the Bob Float (codenamed "Ides").

This version was built for the demonstration at the Big Bang Fair 2015 and is the first deployment of Bob. There are quite a few niggles and updates that are planned for Bob based on the findings from this version, so it's not recommended that you spend much time building this version - however, there will be a lot of cross-over of parts with the next version so you wouldn't need to rebuild everything for Mk2

##Repository Layout

- hardware - contains three directories
  - stl contains the 3D printed part files
  - svg contains the laser cutting files - you may need to alter the outside widths of circles based on the kerf of your cutter
  - pcb contains the eagle files for the power board and the processing board. The processing board is designed for use with the Spark Core. Some tweaks may be needed for the Spark Photon but you *should* be able to use the same board.
- firmware - contains the firmware for the demonstration version of Bob, this accepts commands over the Spark / Bob cloud and performs functions based on that (flashing lights / diving for x seconds) - it doesn't contain depth based functionality (other than reporting) as the tank we used for only a meter deep :)

##Preliminary Bill of Materials

https://docs.google.com/spreadsheets/d/1KixPCNF-eNdthVP5A6QqYoAs1JIhs3zF5F2K2lznduw/edit?usp=sharing

## Build instructions

Will be up soon
