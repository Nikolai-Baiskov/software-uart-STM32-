
🧵 Software UART on STM32
This project demonstrates the implementation of a software UART (bit-banged UART) based on the STM32F103C8 microcontroller using the Blue Pill board.

🔧 Technologies Used
Platform: STM32F103C8 (Blue Pill)

Development Environment: IAR Embedded Workbench

Programming Language: C

Communication: Software (bit-banged) UART

🚀 How to Use
Open the project in IAR Embedded Workbench.

Connect the Blue Pill board to your computer.

Build the project and flash the controller.

Used pins: PA2 -TXD, PA3 - RXD

Connect a UART terminal (e.g., YAT) with the following settings:

Baud rate: 19200

Data bits: 8

Stop bits: 1

Parity: None

⚙️ Features
Send and receive data via software UART

Use of timer TIM2 or another

Low load on the microcontroller's resources, operation through timer interrupts

Ability to run 2 independent UARTs on a single timer with minor code modifications

Flexible configuration for baud rate and parity

Use of circular buffers for data transmission and reception

📄 License
This project is distributed under the MIT license. See the LICENSE file for details.

🙌 Acknowledgments
STMicroelectronics for reliable microcontrollers

IAR Systems for the professional IDE

The open-source community for inspiration

📫 Contact
- GitHub: [@Nikolai-Baiskov](https://github.com/Nikolai-Baiskov)
