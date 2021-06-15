# AFTx06_Private
- [X] Purpose
- [X] Memory Map
- [X] Add in Summary Sections
- [X] Future Work
- [X] Emphasis on Undergraduate Team
- [X] Wording/Check
- [ ] Add layout image
- [ ] Finish synthesizing commands section
- [ ] Update Block Diagram

* Q for John: Is there off or on chip SRAM? 
* Q for John: Does the Non-symmetric CMOS implementation still exist, as part of the workaround, as it was based off of the 90nm FDSOI available?
* Q for John: Under acknowledgements, do we list Crane, MIT Labs, etc.? 
* Q for John: Keep in Layout for Electromagnetic Structures, Phase locked loop, etc.?

<img src="https://user-images.githubusercontent.com/42724680/121823530-63087e00-cc6b-11eb-951c-e7f11c98dd4a.png" align="center" width="675" height="auto">

## Table of Contents

* [Introduction](#Introduction)
* [Purpose](#Purpose)
* [Features](#Features)
* [Building](#Building)
* [Prerequisites](#Prerequisites)
* [Synthesizing](#Synthesizing)
* [Future Work](#Future-Work)
* [Acknowledgements](#Acknowledgements)
* [License](#License)

## Introduction
The "AFTx06_Private" repository is the collection of files for the AFTx06 RISC-V core that was designed by the Purdue System-on-Chip Extension Technology (**SoCET**) team. As an undergraduate team, our goal is to provide students with an educational hands on experience for system-on-chip (**SoC**) design with industry quality SoC design flows. Within the greater SoCET team, there are six well defined sub-teams: Digital Design, Verification, Software, PCB Design, Physical and Analog Design, and Design Flow. Within the six sub-teams, there are groups of students working on peripheral components or small projects, for example of which, many students develop a project within SoCET for their senior design projects. The results of their experiences are reflected within the design for the AFTx06. Being the sixth iteration of the Purdue SoCET team's SoC design, the AFTx06 improves upon our previous design, the AFTx05. The AFTx06 improves upon the previous iteration through additional features such as an added M-Extension, Local and External Interrupt Controller, redesigned Master MUX that now includes Formal Verification, as well as SPI and I2C capabilities. A complete list of the included modules/peripherals is given in the [Features](#Features) section.

## Purpose

As undergraduates, members of the Purdue SoCET team are able to engage in RTL, Physical, and PCB design; chip bringup; verification methods; and various EDA tools and software suites. This allows a student through SoCET to engage with the entire life cycle of chip design even as an undergraduate with any degree of prior experience. The AFTx06 thereby represents the combined efforts of the team and its consultants through multiple years of research and design in developing a successful RISC-V core that can be used for low power IOT devices, edge node computation, or machine learning applications thanks to the implementation of a SparCE Architecture within our design.

The AFTx06 implements unique features and abilities that are not commonly found on other SoC devices. Arguably the most intruiging feature is the SparCE Machine Learning Architecture, an on-chip machine learning architecture to utilize sparsity in convolution arithmetic, allowing extraneous instructions to be skipped. More details on the SparCE architecture is given in the SparCE Machine Learning Architecture section found under Features, below. The AFTx06 also includes novel and experimental designs such as Non-Symmetric CMOS Implementation of Polymorphic Logic, Layout for Electromigration Test Structures, JTAG interface, Phase-Locked Loop, and Platform-Level Interrupt Controller. Many features are experimental designs put forth by Purdue University professors and implemented by the SoCET team. Further details on these can be found in sections below of the same names.

## Features

<img src="https://user-images.githubusercontent.com/42724680/122116868-bf92a700-cdeb-11eb-8de6-ce6737e1a2ee.png" align="center" width="700" height="auto">

*Figure 1: AFTx06 Block Diagram*

### List of Current Features (Modules/Peripherals):

* 32kHz Ring Oscillator
* APB
* AHB - APB Bridge
* AMBA 3.0 AHB-lite Bus
* CLINT
* CORE-INTERRUPT
* DEBUGGER
* GPIO
* I2C
* MEMORY-BLOCKS
* OFFCHIP-SRAM
* PLIC
* RISC-V Processor (1st AHB Primary Device)
    * RV32I Instruction Set Architecture
    * Pass Through Cache
    * 2 Stage Pipeline 
* SPI
* SRAM-CONTROLLER 
* TIMER
* UART Debugger (2nd AHB Primary Device)

### Memory Map: 

| Address Range                    | Size (Used)                       | Unit                                                                                                                                           |
|----------------------------------|-----------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------|
| 0xFFFF-FFFF ... 0xF000_0000      |                                   | Reserved                                                                                                                                       |
| 0xEFFF_FFFF ... 0xE001_0000      |                                   | Reserved                                                                                                                                       |
| 0xE000_FFFF ... 0xE000_0000      |                                   | Interrupt Controllers Clint - # of Address depends on # of cores (1 core - 20 bytes) PLIC - # of addresses depends on # of external interrupts |
| 0x9000_FFFF ... 0x9000_0000      | 8 Bytes                           | SparCE SASA Table Offset 0x00: Write addr for SASA entries Offset 0x04: Write addr for SASA config                                             |
| 0x8FFF_FFFF ... 0x8005_0000      |                                   | Reserved for other peripherals                                                                                                                 |
| 0x8004_FFFF ... 0x8003_0000      |                                   | I2C                                                                                                                                            |
| 0x8003_FFFF ... 0x8003_0000      |                                   | SPI                                                                                                                                            |
| 0x8002_FFFF ... 0x8002_0000      |                                   | Timer                                                                                                                                          |
| 0x8001_FFFF ... 0x8001_0000      |                                   | PWM                                                                                                                                            |
| 0x8000_FFFF ... 0x8000_0000      |                                   | GPIO                                                                                                                                           |
| 0x7FFF_FFFF ... 0x0020_0000      |                                   | Reserved                                                                                                                                       |
| 32'h001F_FFFF ... 32'h0000_8400 | 2MiB (Initial 33792 bytes unused) | Main Memory for the SoC                                                                                                                        |
| 32'h0000_83FF ... 32'h0000_8000 | 1024 bytes                        | Onchip RAM "TOP RAM"                                                                                                                           |
| 32'h0000_7FFF ... 32'h0000_0200  | 32256 bytes                       | Onchip self test ROM                                                                                                                           |
| 32'h0000_01FF ... 32'h0000_0200  | 480 bytes                         | Reserved                                                                                                                                       |
| 32'h0000_001F ... 32'h0000_0000  | 8 words (16 bytes)                | Address 0x00 = PC to jump to after debugger writes "BOTTOM RAM"                                                                                |

### SparCE Machine Learning Architecture

<img src="https://user-images.githubusercontent.com/42724680/122114707-35e1da00-cde9-11eb-83e0-de6f27f3237f.PNG" align="center" width="600" height="auto">

*Figure 2: SparCE Block Diagram*

SpareCE is an on-chip machine learning architecture that utilizes sparsity in convolution arithmetic to allow extraneous instructions to be skipped. Utilized by our design, the architecture has been designed to improve both the speed and power consumption of this common machine learning calculation. Students Vadim Nikiforov and Chan Weng Yan designed the module as it was described in the paper SparCE: Sparsity Aware General-Purpose Core Extensions to Accelerate Deep Neural Networks [1] with the intent to demonstrate the capabilities of the architecture on an ASIC implementation. 

<img src="https://user-images.githubusercontent.com/42724680/122114770-498d4080-cde9-11eb-83bb-ce5375f77d40.PNG" align="center" width="500" height="auto">

*Figure 3: SparCE Diagram Architecture*

The design architecture of the SparCE module is shown in Fig. 3. The blocks within the dotted boundary illustrate the interaction between functional blocks. Signals outside the boundary are inputs from and outputs to the rest of the pipeline. 

* The Sparsity Register File (**SpRF**) is used to dynamically track which registers in the processor’s register file contain zero values. The SpRF contains one entry corresponding to each register in the register file. 
* The Sparce Value Checker (**SVC**) checks if the value is zero and updates the SpRF correspondingly, when an instruction that writes to a register completes, 
* The Sparsity Aware Skip Address (**SASA**) table is a cache-like block with associative memory structure which stores the information required to skip a region. Each entry contains the PC of the instruction prior to the skippable region, a field which stores the condition for the region to be skippable, and the number of instructions that can be skipped. 
* The Pre-identify and Skip Redundancy Uni (**PSRU**) uses the SASA table to identify and skip redundant instruction regions. For each instruction, we check if its PC contains an entry in the SASA table. An entry in the SASA table indicates that the instruction following the current instruction is the start of a potentially skippable region. In this case, the PSRU checks the SpRF to identify if the registers indicated in the SASA table entry are currently zero. If so, it increments the PC to the end of the redundant instruction sequence, thereby skipping instructions. If not, the pipeline proceeds to execute instructions in program order. 
* The Control Flow Instruction Detector (**CFID**) decodes the instruction in the decode stage rather than waiting for the instruction to be decoded in the execute stage. This allows control flow instructions to have higher precedence than skipping.

### Non-symmetric CMOS Implementation of Polymorphic Logic

As a solution to protect intellectual property from counterfeit and trojan injections, SoCET students Isaiah Grace, John Martinuk, and Brian Graves created a CMOS, non-symmetric implementation of Dr. Appenzeller's polymorphic logic concept that has been used within the AFTx06. A gate was made to function as a NAND gate or a NOR gate depending on the voltages applied to the power rails. A seperate gate was designed to behave as an XOR gate or as a Buffer.

Specifications for the Polymorphic NAND/NOR Gate:
* Power rails of the gate, Vxx and Vyy, operate at 0V and 1.2V
* Input pins, A and B, have a range of 0V - 1.2V
* Output pin X has a range of 0V - 1.2V
* When Vxx is 1.2V and Vyy is 0V, the gate takes inputs A and B and yields a NAND output, X
* When Vxx is 0V and Vyy is 1.2V, the gate takes inputs A and B and yields a NOR output, X

<img src="https://user-images.githubusercontent.com/42724680/122130793-c9250a80-cdfd-11eb-85b1-9539ea0b40d3.PNG" align="center" width="400" height="auto">

*Figure 4: Polymorphic NAND/NOR Schematic*

Specifications for the Polymorphic XOR/BUF Gate:
* Power rails of the gate, Vx and Vy, operate in a range of 0V - 1.2V.
* Input pins, A and B, have a range of 0V - 1.2V.
* Output pin X has a range of 0V - 1.2V. 
* When Vx is at 1.2V and Vy is at 0V, the gate takes inputs A and B and yields an XOR output.
* When Vx is at 0V and Vy is at 1.2V, the gate acts as a buffer.


<img src="https://user-images.githubusercontent.com/42724680/122130800-cc1ffb00-cdfd-11eb-896b-01192c69e8ce.PNG" align="center" width="400" height="auto">

*Figure 5: Polymorphic XOR/BUF Schematic*

These cells were used to create a 32-bit CRC module with a configurable polynomial. This implementation was chosen to showcase the polymorphic cells’ ability to camouflage the CRC polynomial being used. The APB slave interface can also provide inputs, and read outputs, which go to/come from a single NAND/NOR cell or a single XOR/BUF cell to verify the cells’ functionality in a small scale digital design.

### JTAG Interface

JTAG is an extensible serial standard used for board-level IC testing which is used on the AFTx06. Common extensions include support for device programming, memory inspection and software debugging, all of which are found on most commercial microcontrollers. The implementation of JTAG included the JTAG Test Access Port (**TAP**), the mandatory instructions and a subset of optional instructions from the IEEE 1149.1 standard, and a custom extension to interact with the AHB-Lite bus. The JTAG module is comprised of 3 major components: The aforementioned TAP, the AHB Access Point (**AHB AP**) created to allow interfacing with the on-chip AHB bus, and the Clock Domain Crossing (**CDC**) modules designed to transfer data between the JTAG clock domain and the SoC clock domain.

<img src="https://user-images.githubusercontent.com/42724680/122130822-d3df9f80-cdfd-11eb-8ae3-47f1d6ce55e1.PNG" align="center" width="400" height="auto">

*Figure 6: Top Level Diagram of JTAG Interface*

The CDC portion consisted of 2 CDC FIFO modules based off of the Sunburst CDC design [3], and a simple synchronizer for capturing the error flag from the AHB AP. 

The AHB AP uses a 37-bit instruction in order to perform read and write operations on the SoC bus. The instruction format was based off of a JTAG debugger design from Texas Instruments [4]. A normal read/write operation would require 2 such instructions: the first to set the target address, and the second to set the target data and start the bus request. To better accommodate device programming, an optimization allowing an auto-increment after each write was included, which removed the addressing step in consecutive read or write operations. 

### Platform-Level Interrupt Controller

The Platform-Level Interrupt Controller (**PLIC**) is a standard interrupt management protocol for managing timer, software, and external interrupt communications by reading a memory-mapped register. The interrupt controller will be included in AFTx06, the next chip that SoCET will tapeout. It should be noted that there are different priority levels for different user modes (U, S, H, M). Currently, M mode (or machine mode) is the only privilege level being configured because this is a mandatory privilege level for the hardware platform. The PLIC takes in hardware interrupt requests and serves them, along with an interrupt ID, to the processor. The Interrupt Controller is currently built for an address width of 32 bits and stores information in 32-bit registers

<img src="https://user-images.githubusercontent.com/42724680/122130835-d93cea00-cdfd-11eb-82df-ff332b55d8e5.PNG" align="center" width="400" height="auto">

*Figure 7: Top Level Communications Between PLIC, the External Modes, and the Processor*

The Interrupt request registers translate hardware interrupt requests into pulses to be sent to other components in the submodule. Register mask prevents masked interrupts from triggering an interrupt request. The interrupt enable register handles logic controlling which registers are masked; it handles status registers related to masking individual interrupts as well as interrupt masking when the disable low priority interrupts module is enabled. The interrupt pending and priority registers module handle registers controlling interrupt priorities as well as the interrupt pending registers. The interrupt priority registers indicate the priority of each hardware interrupt channel and the interrupt pending registers indicate which registers are in the queue to be serviced. The interrupt priority resolve register handles sending the highest priority interrupt index to the status registers for the CPU to read.

## Building

### Prerequisites

<a href="https://github.com/efabless/caravel">Caravel</a>

<a href="https://github.com/The-OpenROAD-Project/OpenLane">OpenLANE</a> 

<a href="http://opencircuitdesign.com/magic/">Magic</a>

<a href="https://github.com/google/skywater-pdk">Google SkyWater PDK</a>


### Synthesizing

Currently, Yosys cannot synthesize the source code because some of our packages cause issues. As a result, the design must be first put through Genus for synthesis, continuing with the rest of the design flow process after obtaining a synthesized netlist. 

Thus, in order to synthesize the design, the user can follow the resulting steps within the command terminal.

```
make user_project_wrapper
```

### Layout

* ADD LAYOUT IMAGE

## Future Work

In the future, once the team obtains the fabricated AFTx06 we plan on utilizing an SoC Tester, the <a href="https://www.agilent.com/about/cr2003/needs_solutions-93000_soc.html"> Agilent 93000 SOC Series Test System</a>, to test and verify our design under real environmental conditions. Testing will occur locally and be performed by undergraduate members of the Purdue SoCET Team. From testing, we hope to obtain further metrics about our design and analyze how it performs, in order to improve our design for the chip's next iteration. 


## Acknowledgements

A working design was generated through the efforts of current and former Purdue SoCET team members and consultants over the last several years. We would like to thank the Doctors Raghunathan, Appenzeller, and Bermel for their contributions to the SparCE architecture, Polymorphic CMOS, and Electromigration Test Structures, respectively.

## References

[1] S. Sen, S. Jain, S. Venkataramani and A. Raghunathan, "SparCE: Sparsity Aware General-Purpose Core Extensions to Accelerate Deep Neural Networks," in IEEE Transactions on Computers, vol. 68, no. 6, pp. 912-925, 1 June 2019, doi: 10.1109/TC.2018.2879434.

[2] S. Das and J. Appenzeller, "WSe2 field effect transistors with enhanced ambipolar 
characteristics," Applied Physics Letters 103, 103501-1-5 (2013).

[3] C. Cummings, “Simulation and Synthesis Techniques for Asynchronous FIFO Design,” Sunburst Design, Provo, UT, USA, 2002.

[4] JTAG Programmer Overview for Hercules-Based Microcontrollers, Texas Instruments, Dallas, TX, United States, Nov. 2015, Accessed on: September, 4th, 2019. [Online]. 
Available: https://www.ti.com/lit/an/spna230/spna230.pdf

## License

The SkyWater Open Source PDK is released under the Apache 2.0 license [Apache 2.0 license](<link>). The copyright details (which should also be found at the top of every file) are

> Copyright 2020 SkyWater PDK Authors Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.
