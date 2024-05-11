# AquaRegulate: Smart Plant Hydration System
![Logo](Images/logo.jpeg)
- AquaRegulate is an automated plant watering system that regulates soil moisture using soil moisture sensors and solenoid valves, ensuring your plants stay perfectly hydrated.

## :ledger: Index
- [What is AquaRegulate about?](#wrench-what-is-aquaregulate-about)
  - [Pre-Requisites](#notebook-pre-requisites)
  - [Purpose](#memo-purpose)
  - [Functionality](#nut_and_bolt-functionality)
  - [Features](#gear-features)
  - [File Structure](#file_folder-file-structure)
- [Details about AquaRegulate](#pen-details-about-aquaregulate)
  - [Wiring Diagram](#wiring-diagram)
  - [Schematics](#schematics)
  - [Flowchart](#flowchart)
  - [Project Pictures](#project-pictures)
- [Bill of Materials](#notebook-bill-of-materials)
- [Gallery](#camera-gallery)
- [Credit/Acknowledgment](#star2-creditacknowledgment)

##  :wrench: What is AquaRegulate about?

### :notebook: Pre-Requisites
- Knowledge of embedded systems and microcontroller programming.
- Familiarity with C for STM32 and ESP32.
- Understanding of UART communication.
- Understanding of ADC.

### :memo: Purpose
The purpose of AquaRegulate is to efficiently water plants or crops without the need for manual intervention. The system can deliver precise amounts of water, reducing wastage and conserving this valuable resource, promote healthy plant growth and development, it saves time for gardeners and farmers, allowing them to focus on other tasks while ensuring plants receive the necessary hydration. By monitoring soil moisture levels and adjusting watering accordingly, the system helps prevent both underwatering, which can stress or kill plants. Finally, the system can result in long-term savings by optimizing water usage and potentially increasing crop yields.

### :nut_and_bolt: Functionality
- Soil moisture sensors to measure the moisture level in the soil.
- Controlling solenoid valves to regulate the water flow to the plants based on the .
- When the soil moisture is above a certain threshold, the system opens the solenoid valves to water the plants.
- Once the moisture level reaches the desired level, the system closes the solenoid valves to stop watering.
- The system continuously monitors the soil moisture level and adjusts the watering accordingly to maintain optimal hydration for the plants.

### :gear: Features
**STM32 Microcontroller with ADC**
- By utilizing DMA for ADC to handle the data, the STM32 microcontrollers acquire ADC samples, making well-suited for the application which gives us high-speed and continuous data acquisition. 
- The ADC converts analog signals from the soil moisture sensor into digital signals that are processed by the microcontroller.

**Transmitting Data to ESP32**
-	The STM32 microcontroller sends the digital sensor data to the ESP32 microcontroller. This data transmission occurs through UART the communication protocol. 
-	The ESP32 microcontroller engages in logic processing to determine the appropriate actions for irrigation control.

**Logic Processing**
-	The ESP32 microcontroller performs the logic logarithm to determine the appropriate action, deciding whether watering is needed based on the received sensor readings.

**Relay Driver**
- The schematic shows a setup where an ESP32 microcontroller is set up to operate five relay (RZ03-1C4-D012) modules allowing for the control of electrical appliances. 
- Each relay is controlled by a TIP120 Darlington transistor acting as a switch, which is managed by the ESP32 using its GPIO pins.
- The relay circuits include 1N4001 diodes to prevent voltage spikes, from loads and LEDs that light up to show relays. 
- The relay coils receive power from a shared +12V line while the ESP32 itself is powered by a +3.3V source. 
- This configuration enables the ESP32s signals to handle power circuits effectively making it an efficient solution, for automating or remotely controlling multiple devices at once.

**Solenoid Activation**
- When the relays are activated, they energize the solenoids, allowing water to flow through the irrigation pipes and reach the plants.

### :file_folder: File Structure
Add a file structure here with the basic details about files, below is an example.

```
.
├── Core
│   ├── Inc
│   │   ├── main.h
│   │   ├── stm32f0xx_hal_conf.h
│   │   └── stm32f0xx_it.h
│   └── Src
│       ├── main.c
│       ├── stm32f0xx_hal_msp.c
│       ├── stm32f0xx_it.c
│       └── system_stm32f0xx.c
├── Drivers
├── Images
│   └── logo.jpeg
├── MDK-ARM
└── Relay
    └── main
        ├── main.c
        └── relay.c
```

## :pen: Details about AquaRegulate 

### Wiring Diagram
![Wiring Diagram](Images/real-diagram.jpg)
![Diagram](Images/diagram.jpg)

### Custom PCB Schematic
**The whole schematic**
![Schematics](Images/schematic.jpg)

**Front of PCB**
![Front](Images/final_schematic_pcb_front.jpg)

**Back of PCB**
![Back](Images/final_schematic_pcb_back.jpg)

**Fill All Zones**
![Fill All](Images/final_schematic_pcb_fill_all_zones.jpg)

**Unfill All Zones**
![Unfill All](Images/final_schematic_pcb_unfill_all_zones.jpg)


### Flowchart
![Flowchart](Images/flowchart.png)

## Project Pictures
### **Plants**
![Plants](Images/plants.jpeg)

### **Circuits and Relay**
![Circuits and Relay](Images/circuits-and-relay.jpeg)

### **Pump**
![Pump](Images/pump.jpeg)

### **Whole Project**
![Whole Project](Images/whole-project.jpeg)

## :notebook: Bill of Materials
| Description                                                          | Part Number                                                                                                                                                                                    | Datahseet                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Qty | Unit Price     | Extended Price           |
| -------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --- | -------------- | ------------------------ |
| 12V Relay driver for controlling high power devices                  | RZ03-1C4-D012                                                                                                                                                                                  | https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=RZ&DocType=DS&DocLang=English                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | 10  | $2.67 | $26.70   |
| Microcontroller compatible with ESP32 for control and communication  | STM32F072RB                                                                                                                                                                                    |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | 1   | $ 12.00  | $12.00   |
| Arduino ESP32 Nano for Wi-Fi connectivity                            | 1965-ESP32-PICO-DEVKITM-2-ND                                                                                                                                                                   | [https://www.amazon.com/Arduino-ABX00092-Bluetooth-MicroPython-Compatible/dp/B0C947C9QS/ref=sr_1_3?crid=ML850PYR893C&dib=eyJ2IjoiMSJ9.tzsRNyA2MvZs7CvxfelOGoXEQU7SOPyjsvEfuYwOpxDaxKoc0rzpw5TDsPvdEuPC2dw7Jppf5QGreHfoasLTJQR-rKREueaoh04m7Yzii7mz3CKoLqH-oWM7WjjW_rNHOcFFXldKf7-1ek3iGOIYQfEba4pjkmdLIylWIFkkuXZjZOb8r1inF82dgfzWGgG-7CIpcvYZ6dQ4vOv1fNyga6y74J5NLA3L_5teXRC7yoY.RTNJyyUG1moiCrID5ooxgfq90pdQJy5171SlEhxfhQY&dib_tag=se&keywords=Arduino+ESP32+Nano+for+Wi-Fi+connectivity&qid=1714179734&sprefix=arduino+esp32+nano+for+wi-fi+connectivity%2Caps%2C202&sr=8-3](https://www.amazon.com/Arduino-ABX00092-Bluetooth-MicroPython-Compatible/dp/B0C947C9QS/ref=sr_1_3?crid=ML850PYR893C&dib=eyJ2IjoiMSJ9.tzsRNyA2MvZs7CvxfelOGoXEQU7SOPyjsvEfuYwOpxDaxKoc0rzpw5TDsPvdEuPC2dw7Jppf5QGreHfoasLTJQR-rKREueaoh04m7Yzii7mz3CKoLqH-oWM7WjjW_rNHOcFFXldKf7-1ek3iGOIYQfEba4pjkmdLIylWIFkkuXZjZOb8r1inF82dgfzWGgG-7CIpcvYZ6dQ4vOv1fNyga6y74J5NLA3L_5teXRC7yoY.RTNJyyUG1moiCrID5ooxgfq90pdQJy5171SlEhxfhQY&dib_tag=se&keywords=Arduino+ESP32+Nano+for+Wi-Fi+connectivity&qid=1714179734&sprefix=arduino+esp32+nano+for+wi-fi+connectivity%2Caps%2C202&sr=8-3)                             | 3   | $   22.00  | $66.00   |
| Capacitive soil moisture sensor                                      | SHT30-DIS-B2.5KS                                                                                                                                                                               | [https://www.amazon.com/Capacitive-Moisture-Corrosion-Resistant-Detection/dp/B07SYBSHGX/ref=sr_1_7?crid=3EOTG9XYEJSBW&dib=eyJ2IjoiMSJ9.Rjq-jful1ChilKba7CvfTbAJPlQA3CakoMGiOCdfsP-9SpWTfxwuN4qxat5qXHiSl4jEdlIQa84zSZrldkowA3qmCAiMXQj7JlyM2wdWZg34nY_RqgmaJn5wa43MUXgFYMa1ACtgnODf0TZBCzdeJ_icAPAHc_9imAGyx3r22hGwUK_j-21_c4BSi6oTOIbSJHK0jetUjsK977ZAyh6eHvI7mBV_2Q6YV7kOB97jNbrDsvaNGe5gTKlWJf71zIwBFuHhUVZSC4O1BKitCM9bzVKtnc99jkU2SKfH93Sd0OU.F22fz6FCw-8l7g1uxyW-uH3pXEoOzB9LxdptjKEG_r0&dib_tag=se&keywords=soil+moisture+sensor&qid=1711233301&sprefix=soil+moistu%2Caps%2C227&sr=8-7](https://www.amazon.com/Capacitive-Moisture-Corrosion-Resistant-Detection/dp/B07SYBSHGX/ref=sr_1_7?crid=3EOTG9XYEJSBW&dib=eyJ2IjoiMSJ9.Rjq-jful1ChilKba7CvfTbAJPlQA3CakoMGiOCdfsP-9SpWTfxwuN4qxat5qXHiSl4jEdlIQa84zSZrldkowA3qmCAiMXQj7JlyM2wdWZg34nY_RqgmaJn5wa43MUXgFYMa1ACtgnODf0TZBCzdeJ_icAPAHc_9imAGyx3r22hGwUK_j-21_c4BSi6oTOIbSJHK0jetUjsK977ZAyh6eHvI7mBV_2Q6YV7kOB97jNbrDsvaNGe5gTKlWJf71zIwBFuHhUVZSC4O1BKitCM9bzVKtnc99jkU2SKfH93Sd0OU.F22fz6FCw-8l7g1uxyW-uH3pXEoOzB9LxdptjKEG_r0&dib_tag=se&keywords=soil+moisture+sensor&qid=1711233301&sprefix=soil+moistu%2Caps%2C227&sr=8-7) | 4   | $ 8.95 | $35.80   |
| 12V solenoid valve for water control                                 | 997                                                                                                                                                                                            | [https://www.adafruit.com/product/997](https://www.adafruit.com/product/997)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | 6   | $6.95 | $41.70   |
| Diode for protecting the transistor from voltage spikes              | 1N4001                                                                                                                                                                                         | [https://cdn-shop.adafruit.com/datasheets/1N4001-D.PDF](https://cdn-shop.adafruit.com/datasheets/1N4001-D.PDF)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | 10  | $1.50 | $15.00   |
| TIP120 Darlington Transistor for controlling the solenoid valve      | TIP120                                                                                                                                                                                         | [https://cdn-shop.adafruit.com/datasheets/TIP120.pdf](https://cdn-shop.adafruit.com/datasheets/TIP120.pdf)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | 10  | $ 2.50 | $25.00   |
| Resistor kit                                                         | [https://www.amazon.com/gp/product/B07P3MFG5D/ref=ppx_yo_dt_b_asin_title_o03_s01?ie=UTF8&psc=1](https://www.amazon.com/gp/product/B07P3MFG5D/ref=ppx_yo_dt_b_asin_title_o03_s01?ie=UTF8&psc=1) |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | 1   | $ 12.99  | $12.99   |
| Diodes kit                                                           | [https://www.amazon.com/gp/product/B07YG8K1R9/ref=ppx_yo_dt_b_asin_title_o03_s00?ie=UTF8&psc=1](https://www.amazon.com/gp/product/B07YG8K1R9/ref=ppx_yo_dt_b_asin_title_o03_s00?ie=UTF8&psc=1) |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | 1   | $9.99 | $9.99 |
| PCB to mount and connect all components                              | Custom PCB                                                                                                                                                                                     |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | 1   | $9.20 | $9.20 |
| Various LEDs for status indicators                                   | LEDs                                                                                                                                                                                           |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |     |                |                          |
| 12V Power adapter to power the system                                | Power Adapter                                                                                                                                                                                  |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |     |                |                          |
| Peristaltic Liquid Pump with Silicone Tubing - 12V DC Power         | Pump                                                                                                                                                                                           |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | 2   | $ 24.95  | $ 49.90  |
| Lowe's 5-Gallon (s) Plastic General Bucket                           | Bucket                                                                                                                                                                                         |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | 1   | $4.98 | $4.98 |
| Proline Series 1/2-in x 1/2-in Threaded Female Adapter Union Fitting | Bulkhead Fitting                                                                                                                                                                               |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | 1   | $ 16.48  | $ 16.48  |
| 1/2-in FIP Brass Quarter Turn Hose Bibb                              | Hose Bibbs                                                                                                                                                                                     |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | 1   | $   11.48  | $ 11.48  |
| Braided Stainless Steel Flexible Faucet Supply Line                  | B4F20UL-2                                                                                                                                                                                      |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | 2   | $6.98 | $ 13.96  |

##  :camera: Gallery
Pictures of your project.


## :star2: Credit/Acknowledgment
- Jeniffer Limondo: u1063824@umail.utah.edu
- Khoi Nguyen: khoi.nguyen@utah.edu
- Seoin Kim: seoin.kim@utah.edu
- Eddie Franco: u1379136@utah.edu
