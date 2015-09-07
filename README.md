#Bob Mk2 - "Bud"

This repository contains the hardware and firmware files for Mk2 of the Bob Float (codenamed "Bud").

##Repository Layout

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
