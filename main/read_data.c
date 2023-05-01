#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"

#include "w25q64.h"

#define TAG "W25Q64"

#define PAYLOADSIZE 16

void app_main()
{
	W25Q64_t dev;
	W25Q64_init(&dev);

	// ステータスレジスタ1の取得
	// Get Status Register1
	uint8_t reg1;
	esp_err_t ret;
	ret = W25Q64_readStatusReg1(&dev, &reg1);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "readStatusReg1 fail %d",ret);
		while(1) { vTaskDelay(1); }
	} 
	ESP_LOGI(TAG, "readStatusReg1 : %x", reg1);
	
	// ステータスレジスタ2の取得
	// Get Status Register2
	uint8_t reg2;
	ret = W25Q64_readStatusReg2(&dev, &reg2);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "readStatusReg2 fail %d",ret);
		while(1) { vTaskDelay(1); }
	}
	ESP_LOGI(TAG, "readStatusReg2 : %x", reg2);

	// Unique IDの取得テスト
	// Get Unique ID
	uint8_t uid[8];
	ret = W25Q64_readUniqieID(&dev, uid);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "readUniqieID fail %d",ret);
		while(1) { vTaskDelay(1); }
	}
	ESP_LOGI(TAG, "readUniqieID : %x-%x-%x-%x-%x-%x-%x-%x",
		 uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7]);

	// JEDEC IDの取得テスト
	// Get JEDEC ID
	uint8_t jid[3];
	ret = W25Q64_readManufacturer(&dev, jid);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "readManufacturer fail %d",ret);
		while(1) { vTaskDelay(1); }
	}
	ESP_LOGI(TAG, "readManufacturer : %x-%x-%x",
		 jid[0], jid[1], jid[2]);


	uint8_t rbuf[PAYLOADSIZE];

    // read until first sector is all f's

    // if (access(fname, F_OK) == 0) {
    //     printf("File already exists! Write to a new file or delete the file!\n");
    // } else {

    // }

    // char *fname = "/Users/alexyen/Dropbox/UCSD/Research/Helium/esp-idf-w25q64/data.txt";

    // FILE *fp;
    // fp = fopen(fname, "w");// "w" means that we are going to write on this file
    // // int i = 0;
    // // for(i = 0; i < 10; i++){
    // 	// fprintf(fp, "%d\n", i);
    // fputs("This is Guru99 Tutorial on fputs,", fp);
    // // }
    // fclose(fp);

    int sect_no = 1;
	int inaddr = 0;

	bool done_reading = false;

	uint32_t addr = sect_no;
	addr<<=12;
	addr += inaddr;

	uint16_t count;
	uint32_t modem, time;

	uint8_t data_buf[PAYLOADSIZE];

	printf("Count,Modem,Time\n");

    while(sect_no < 1047){
		while(inaddr < 4095){
			W25Q32_readData(&dev, addr, data_buf, PAYLOADSIZE, &count, &modem, &time);

			if (count == 0xffff && modem == 0xffffffff && time == 0xffffffff){
				done_reading = true;
				break;
			}

			inaddr += PAYLOADSIZE;

			addr = sect_no;
			addr<<=12;
			addr += inaddr;

			// don't think this is necessary but			
			count = 0;
			modem = 0;
			time = 0;

		}
		if (done_reading) break;
		else{
			sect_no += 1;
			inaddr = 0;

			// calculate next addr
			addr = sect_no;
			addr<<=12;
			addr += inaddr;
		}
	}
}