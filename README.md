#Bob the AUV

This repository contains the hardware and firmware files for the Bob Float.

Verisons contained in this repository

##Mk1

This version was built for demonstration at the Big Bang Fair 2015 and is the first deployment of Bob. There are quite a few niggles to be fixed and updates that are planned for Bob based on the findings from this version, so it's not recommended that you spend much time with it - however, there will be a lot of cross-over of parts with the next version so you wouldn't need to rebuild everything for Mk2

Mk1 can be found at https://github.com/darkwaterfoundation/bob/tree/mk1

##Mk2

This version had a few improvements to the code and bottom cap designs to attempt to fix a small leak discovered in the Mk1. This version was shown at Maker Faire UK and the Dublin Maker events

Mk2 can be found at https://github.com/darkwaterfoundation/bob/tree/mk2

##Development

The development branch contains the ongoing designs and work on Mk3 - This is a dramatic redesign and simplifcation. We've moved from using the Spark / Particle core / photon microcontrollers to a dual Arduino Pro Mini system and from Wifi to a longer range wireless communication system.

The Development version can be found at https://github.com/darkwaterfoundation/bob/tree/development

##Repositories Layout

- hardware - contains three directories
  - stl contains the 3D printed parts files
  - svg contains the laser cutting files - you may need to alter the outside widths of circles based on the kerf of your cutter
  - pcb contains the eagle files for the power board and the processing board. The processing board is designed for use with the Spark Core. Some tweaks may be needed for the Spark Photon but you *should* be able to use the same board.
- firmware - contains the firmware for the demonstration version of Bob, this accepts commands over the Spark / Bob cloud and performs functions based on that (flashing lights / diving for x seconds) - it doesn't contain depth based functionality (other than reporting) as the tank we used was only a meter deep :)

##Preliminary Bill of Materials

https://docs.google.com/spreadsheets/d/1KixPCNF-eNdthVP5A6QqYoAs1JIhs3zF5F2K2lznduw/edit?usp=sharing

##Build instructions

Will be up soon

##License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License. To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/ or send a letter to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.

*Some portions of code may be subject to different licensing agreements.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
