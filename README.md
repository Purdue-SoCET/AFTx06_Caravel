# AFTx06_Private

<img src="https://user-images.githubusercontent.com/42724680/121823530-63087e00-cc6b-11eb-951c-e7f11c98dd4a.png" align="center" width="675" height="309">

## Table of Contents

* [Introduction](#Introduction)
* [Implementation/Purpose](#Purpose)
* [Features (Modules/Peripherals)](#Features-modulesperipherals)
* [Building](#Building)
* [Prerequisites](#Prerequisites)
* [Synthesizing](#Synthesizing)
* [Acknowledgements](#Acknowledgements)
* [License](#License)

### Introduction
The "Purdue-SoCET-AFTx06-SkyWater-Project" is the repository for the AFTx06 RISC-V core as designed by the Purdue System-on-Chip Extension Technology (SoCET) team. The AFTx06 is the sixth iteration of the Purdue SoCET team's attempts at creating a core and improves upon our previous design the AFTx05. In comparison with the AFTx05, the AFTx06 has additional features such as an added M-Extension, a Local and External Interrupt Controller, a redesigned Master MUX that now includes Formal Verification, as well as SPI and I2C capabilities. A complete list of the included modules/peripherals is given in the "Features" section.

### Purpose
Our goal is to provide students with an educational hands on experience with an industry quality SoC design flow. Members of the Purdue SoCET team engage with RTL design, physical design, PCB design, chip bringup, verification methods, an array of EDA tools and software development. Through SoCET a student can engage with the entire life cycle of chip design. The AFTx06 thereby represents the combined efforts of the team and its consultants through multiple years of research and design in developing a successful RISC-V core.

### Features (Modules/Peripherals)
List of Current Features (Modules/Peripherals)
<ul> 
<li>APB</li>
<li>GPIO</li>
<li>DEBUGGER</li>
<li>AHB2APB</li>
<li>TIMER</li>
<li>I2C</li>
<li>SRAM-CONTROLLER</li>
<li>MEMORY-BLOCKS</li>
<li>OFFCHIP-SRAM</li>
<li>PLIC</li>
<li>CLINT</li>
<li>CORE-INTERRUPT</li>
<li>SPI</li>
</ul>

## Building

### Prerequisites

<a href="https://github.com/efabless/caravel">Caravel</a>

<a href="https://github.com/The-OpenROAD-Project/OpenLane">OpenLANE</a> 

<a href="http://opencircuitdesign.com/magic/">Magic</a>

<a href="https://github.com/google/skywater-pdk">Google SkyWater PDK</a>


### Synthesizing

At the moment, the files are not synthesizable with Yosys as there exist unsupported structures referenced through one of our own packages that cause an issue within Synthesis, as packages are not supported within Yosys, and prevent the process from continuing. As a result, the design must be first put through Genus for synthesis, continuing with the rest of the design flow process after obtaining a synthesized netlist. 

However, in order to synthesize the design, the user can follow the resulting steps within the command terminal.

*Add Commands Here*

### Layout

*Future Layout Image to be Added*

## Acknowledgements
A working design that was generated through the efforts of current and former Purdue SoCET team members and their consultants over the course of the last several years. 

## License

The SkyWater Open Source PDK is released under the Apache 2.0 license [Apache 2.0 license](<link>). The copyright details (which should also be found at the top of every file) are

> Copyright 2020 SkyWater PDK Authors Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.
