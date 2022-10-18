
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
| STM32f051 Discovery Board   | x 2|
| FTDI Adaptor   | x 1    |
| Micro-USB   | x 1 |
| Mini-USB   | x 1  |
| Male-to-Female Jumpers   | x 10  |
| Male-to-Male Jumpers   | x 10    |
| Computer to Code   | x 1    |



## Required Software
As well as the above mentioned hardware, the following software/tools are needed:

| Hardware     | Quantity |
| :----------- | :-----------:|
|python|-|
| STM32 Cube IDE (Or equivalent such as VS Code) | x 1|
| Serial Port Monitor (Putty on Windows) | x 1|



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

# stm32_hal_icm20948

## Brief

__ICM-20948 is 9-axis IMU sensor.__  
__TAG - `ICM-20948` `AK09916` `SPI` `STM32 HAL`__

|Axis|Not Recommended for New Designs|Recommended for New Designs|
|:---:|:---:|:---:|
|6-axis|`MPU-6050`|`ICM-20602`|
|9-axis|`MPU-9250`|`ICM-20948`| 

## Library Feature
* Read sensor data
* Offset cancellation

## Dev Environment  
- STM32CubeIDE
- STM32 HAL driver
- STM32F411CEU6
- ICM-20948 module ( [SparkFun 9Dof IMU Breakout - ICM-20948 (Qwiic)](https://www.sparkfun.com/products/15335) )

<img src = "https://user-images.githubusercontent.com/48342925/125441402-2af7d878-53d7-4d2f-83df-f304df4c330c.png" width = "25%">
<img src = "https://user-images.githubusercontent.com/48342925/125441479-3370f3fb-8485-4d11-ad3e-1eb6e697f8e0.png" width = "25%">

## STM32CubeMX

![image](https://user-images.githubusercontent.com/48342925/130714344-753f1b21-abe7-412c-b723-0826e43b9203.png)


### SPI
![image](https://user-images.githubusercontent.com/48342925/129033034-ef4d8818-7338-4d90-bcd9-9d66491074bd.png)

#### GPIO
![image](https://user-images.githubusercontent.com/48342925/129033151-7cf98ef2-22e9-44ef-8ddd-77b6ea9abf0b.png)


## Example

### icm20948.h
- SPI1, PA4 (CS Pin)

```c
/* User Configuration */
#define ICM20948_SPI					(&hspi1)

#define ICM20948_SPI_CS_PIN_PORT		GPIOA
#define ICM20948_SPI_CS_PIN_NUMBER		GPIO_PIN_4
```

### main.c
- only contain relative things

```c
#include "icm20948.h"

// variables to store sensor data
axises my_gyro;
axises my_accel;
axises my_mag;

int main(void)
{
    // initialize
    // Modify the inside of init function for personal setting.
    icm20948_init();
    ak09916_init();

    while (1)
    {
        // raw data
        icm20948_gyro_read(&my_gyro);
        icm20948_accel_read(&my_accel);
        ak09916_mag_read(&my_mag);

        // or unit conversion
        icm20948_gyro_read_dps(&my_gyro);
        icm20948_accel_read_g(&my_accel);
        ak09916_mag_read_uT(&my_mag);
    }
}
```
