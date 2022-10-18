
<!-- ABOUT THE PROJECT -->
## Group Project For EEE3097s 

The following project design is for an ARM based digital IP using an STM32F051-microcontroller. This design aims to retrieve, compress and encrypt data from an Inertial Measurement Unit (IMU) sensor. This type of sensor includes an Accelerometer, a Gyroscope, a Magnetometer and a Barometer. This design will be implemented as a buoy installed on an ice 'pancake' in the Southern Ocean to collect data about the ice and wave dynamics.

This data will then be transmitted using the Iridium communication network, which is a global satellite communications network. However, this is extremely costly and therefore the data would need to be compressed to reduce its size. The data is also encrypted for security purposes.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

## Required Hardware
To get started working on this project, the following hardware is required:

| Hardware     | Quantity |
| :----------- | :-----------:|
| ICM-20649 Module | x 1|
| STM32f051 Discovery Board   | x 1|
| FTDI Adaptor   | x 1    |
| Micro-USB   | x 1 |
| Mini-USB   | x 1  |
| Male-to-Female Jumpers   | x 10  |
| Male-to-Male Jumpers   | x 10    |
| Computer to Code   | x 1    |

Note: The quantity of the jumpers stated is far beyond what is actually needed, however, jumpers are a really useful tool for debugging and so having many of them is highly recommended. 

## Required Software
As well as the above mentioned hardware, the following software/tools are needed:

| Hardware     | Quantity |
| :----------- | :-----------:|
| STM32 Cube IDE (Or equivalent such as VS Code) | x 1|
| Serial Port Monitor (Putty on Windows, or SerialTools on Mac) | x 1|

Note: The latest STM software needs to be loaded onto the STM32f051 for this project, this can be done with the STM32 Cube IDE software. 

## Connecting the Hardware
The pins on the WaveShare SenseHAT that need to be connected to the STM32f051 are the following:
| WaveShare HAT Pin     | STM32f051 Pin |
| :-----------: | :-----------:|
| GND | GND |
| 3.3V | 3.3V |
| SDA | PB7|
| SCL | PB6|

The pins on the FTDI adaptor that need to be connected to the STM32f051 are the following:
| FTDI Adaptor    | STM32f051 Pin |
| :-----------: | :-----------:|
| GND | GND |
| 5V | 5V |
| TX | PA10|
| RX | PA9|

