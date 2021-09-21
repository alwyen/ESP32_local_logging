# esp-idf-w25q64
SPI Flash Memory W25Q64 Access Library for esp-idf.   
I ported from [here](https://github.com/Tamakichi/Arduino-W25Q64).   

# Installation for ESP32

```
git clone https://github.com/nopnop2002/esp-idf-w25q64
cd esp-idf-w25q64
idf.py set-target esp32
idf.py menuconfig
idf.py flash
```

# Installation for ESP32S2

```
git clone https://github.com/nopnop2002/esp-idf-w25q64
cd esp-idf-w25q64
idf.py set-target esp32S2
idf.py menuconfig
idf.py flash
```

# Configuration   
You have to set this config value with menuconfig.   
- CONFIG_MISO_GPIO   
 GPIO number(IOxx) to MISO.
- CONFIG_MOSI_GPIO   
 GPIO number(IOxx) to MOSI.
- CONFIG_SCLK_GPIO   
 GPIO number(IOxx) to SCLK.
- CONFIG_CS_GPIO   
 GPIO number(IOxx) to CS.

![config-main](https://user-images.githubusercontent.com/6020549/108640422-e47daa00-74dc-11eb-8353-242165cde308.jpg)

- Default for ESP32

![config-esp32](https://user-images.githubusercontent.com/6020549/108640424-e5aed700-74dc-11eb-8c42-096b383f27b9.jpg)

- Default for ESP32-S2

![config-esp32s2](https://user-images.githubusercontent.com/6020549/108640426-e5aed700-74dc-11eb-9b28-7d80a1a665ac.jpg)

# API

// Start Flash  
void spi_master_init(W25Q64_t * dev, int GPIO_CS);  

// Get status register1  
esp_err_t W25Q64_readStatusReg1(W25Q64_t * dev, uint8_t * reg1);  

// Get status register2(Winbond only)  
esp_err_t W25Q64_readStatusReg2(W25Q64_t * dev, uint8_t * reg2);  

// Get Unique ID(Winbond only)  
esp_err_t W25Q64_readUniqieID(W25Q64_t * dev, uint8_t * id);  

// Get JEDEC ID(Manufacture, Memory Type,Capacity)  
esp_err_t W25Q64_readManufacturer(W25Q64_t * dev, uint8_t * id);  

// Check busy  
bool W25Q64_IsBusy(W25Q64_t * dev);  

// Set power down mode  
esp_err_t W25Q64_powerDown(W25Q64_t * dev);;  

// Set write enable  
esp_err_t W25Q64_WriteEnable(W25Q64_t * dev);  

// Set write disable  
esp_err_t W25Q64_WriteDisable(W25Q64_t * dev);  

// Read data from memory  
uint16_t W25Q64_read(W25Q64_t * dev, uint32_t addr, uint8_t *buf, uint16_t n);   

// First read data from memory  
uint16_t W25Q64_fastread(W25Q64_t * dev, uint32_t addr, uint8_t *buf, uint16_t n);  

// Erase data by Sector  
bool W25Q64_eraseSector(W25Q64_t * dev, uint16_t sect_no, bool flgwait);  

// Erase data by block(64KB)  
bool W25Q64_erase64Block(W25Q64_t * dev, uint16_t blk_no, bool flgwait);  

// Erase data by Block(32KB)  
bool W25Q64_erase32Block(W25Q64_t * dev, uint16_t blk_no, bool flgwait);  

// Erase all data  
bool W25Q64_eraseAll(W25Q64_t * dev, bool flgwait);  

// Write data to memory  
int16_t W25Q64_pageWrite(W25Q64_t * dev, uint16_t sect_no, uint16_t inaddr, uint8_t* buf, int16_t n);  

# Wireing

|#|W25Q64||ESP32|ESP32S2|
|:-:|:-:|:-:|:-:|:-:|
|1|/CS|--|GPIO5|GPIO34|
|2|MISO|--|GPIO19|GPIO33|
|3|/WP|--|3.3V|3.3V|3.3V|
|4|GND|--|GND|GND|GND|
|5|MOSI|--|GPIO23|GPIO35|
|6|SCK|--|GPIO18|GPIO36|
|7|/HOLD|--|3.3V|3.3V|3.3V|
|8|VCC|--|3.3V|3.3V|3.3V|

__You can change it to any pin using menuconfig.__   
__However, changing to some pins does not work properly.__


# Winbond

|Device|# of Bytes|Address range|# of 4K-Sectors|# of 32K-Blocks|# of 64K-Blocks|JEDEC ID|
|:---|:---|:---|:---|:---|:---|:---|
|W25Q80|1M|0x0FFFFF|256|32|16|EF-40-14|
|W25Q16|2M|0x1FFFFF|512|64|32|EF-40-15|
|W25Q32|4M|0x3FFFFF|1024|128|64|EF-40-16|
|W25Q64|8M|0x7FFFFF|2048|256|128|EF-40-17|
|W25Q128|16M|0xFFFFFF|4096|512|256|EF-40-18|

## W25Q80   
![W25Q80](https://user-images.githubusercontent.com/6020549/81382267-1342b380-9149-11ea-88bc-bc7cd07501a4.jpg)

- Manufacturer:  
 Byte1 : ManufacturerID(0xef=Winbond)  
 Byte2 : MemoryType(0x40=SPI/0x60=QPI)  
 Byte3 : Capacity(2^0x14=2^20=0x100000=1M Byte=8M Bit)  
- First 10Byte : ASCII 0-9  
- Next 32Byte : ASCII A-Z  

## W25Q16   
![W25Q16](https://user-images.githubusercontent.com/6020549/81403592-b6f28a80-916e-11ea-92ef-1bbac7b79e15.jpg)

## W25Q32   
![W25Q32](https://user-images.githubusercontent.com/6020549/81382304-22296600-9149-11ea-8cbe-8da89123539a.jpg)

## W24Q64   
![W25Q64](https://user-images.githubusercontent.com/6020549/81382272-16d63a80-9149-11ea-90b3-aef92642914f.jpg)

## W25Q128   
![W25Q128](https://user-images.githubusercontent.com/6020549/81382327-2c4b6480-9149-11ea-8b34-dcbd6e43aa37.jpg)

---

# MACRONIX   

|Device|# of Bytes|Address range|# of 4K-Sectors|# of 32K-Blocks|# of 64K-Blocks|JEDEC ID|
|:---|:---|:---|:---|:---|:---|:---|
|MX25L3206E|4M|0x3FFFFF|1024|128|64|C2-20-16|
|MX25L6473E|8M|0x7FFFFF|2048|256|128|C2-20-17|
|MX25L12835F|16M|0xFFFFFF|4096|512|256|C2-20-18|

## MX25L3206E   
![MX25L3206E](https://user-images.githubusercontent.com/6020549/84555332-47685000-ad57-11ea-81a1-db88de013da9.jpg)

- Manufacturer:  
 Byte1 : ManufacturerID(0xc2=Macronix)  
 Byte2 : MemoryType(0x20)  
 Byte3 : Capacity(2^0x16=2^22=0x400000=4M Byte=32M Bit)  

## MX25L6473E   
![MX25L6473E](https://user-images.githubusercontent.com/6020549/81383580-32dadb80-914b-11ea-823b-6487b7a7e073.jpg)

## MX25L12835F   
![MX25L12835F](https://user-images.githubusercontent.com/6020549/81383590-353d3580-914b-11ea-913d-ce862c58c36c.jpg)

---

# Note   
I tested these.   
But I couldn't get it working.   
- GD25Q64   
- SST25VF016B   
- SST25VF032B   

# Note   
There is a example to build a FAT file system on External SPI FLSAH Memory is available [here](https://github.com/espressif/esp-idf/tree/master/examples/storage/ext_flash_fatfs).   
The ESP32's onboard FLASH is 4MByte, and you can reserve up to about 3MByte of storage on the onboard FLASH.   
With the large capacity SPI FLSAH Memory, you can add a large amount of storage.   
This repository may no longer be needed.   
