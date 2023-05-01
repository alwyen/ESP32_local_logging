/* The example of ESP-IDF
 *
 * This sample code is in the public domain.
 */

#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"

#include "w25q64.h"

#define TAG "W25Q64"

#define READNUMBYTES 256
#define PAYLOADSIZE 16

//
// Data dump list
// dt(in):Data to dump
// n(in) :Number of bytes of data
//
void dump(uint8_t *dt, int n)
{
	uint16_t clm = 0;
	uint8_t data;
	uint8_t sum;
	uint8_t vsum[16];
	uint8_t total =0;
	uint32_t saddr =0;
	uint32_t eaddr =n-1;

	printf("----------------------------------------------------------\n");
	uint16_t i;
	for (i=0;i<16;i++) vsum[i]=0;  
	uint32_t addr;
	for (addr = saddr; addr <= eaddr; addr++) {
		data = dt[addr];
		if (clm == 0) {
			sum =0;
			printf("%05"PRIx32": ",addr);
		}

		sum+=data;
		vsum[addr % 16]+=data;

		printf("%02x ",data);
		clm++;
		if (clm == 16) {
			printf("|%02x \n",sum);
			clm = 0;
		}
	}
	printf("----------------------------------------------------------\n");
	printf("       ");
	for (i=0; i<16;i++) {
		total+=vsum[i];
		printf("%02x ",vsum[i]);
	}
	printf("|%02x \n\n",total);
}

void ping( TimerHandle_t xTimer )
{
  ESP_LOGI("TIME","tring tring!!!");

}

void app_main()
{
	W25Q64_t dev;
	printf("Init\n");
	printf("Init\n");
	printf("Init\n");
	W25Q64_init(&dev);

	uint32_t start = xTaskGetTickCount();
	printf("Current time (ms?): %u\n", start);

	// ステータスレジスタ1の取得
	// Get Status Register1
	uint8_t reg1;
	esp_err_t ret;
	printf("Read status reg1\n");
	printf("Read status reg1\n");
	printf("Read status reg1\n");
	ret = W25Q64_readStatusReg1(&dev, &reg1);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "readStatusReg1 fail %d",ret);
		while(1) { vTaskDelay(1); }
	} 
	ESP_LOGI(TAG, "readStatusReg1 : %x", reg1);
	
	// ステータスレジスタ2の取得
	// Get Status Register2
	uint8_t reg2;
	printf("read status reg 2\n");
	printf("read status reg 2\n");
	printf("read status reg 2\n");
	ret = W25Q64_readStatusReg2(&dev, &reg2);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "readStatusReg2 fail %d",ret);
		while(1) { vTaskDelay(1); }
	}
	ESP_LOGI(TAG, "readStatusReg2 : %x", reg2);

	// Unique IDの取得テスト
	// Get Unique ID
	uint8_t uid[8];
	printf("read unique ID\n");
	printf("read unique ID\n");
	printf("read unique ID\n");
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
	printf("read manufacturer\n");
	printf("read manufacturer\n");
	printf("read manufacturer\n");
	ret = W25Q64_readManufacturer(&dev, jid);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "readManufacturer fail %d",ret);
		while(1) { vTaskDelay(1); }
	}
	ESP_LOGI(TAG, "readManufacturer : %x-%x-%x",
		 jid[0], jid[1], jid[2]);

	// // erase all on startup; this takes a while
	// W25Q64_eraseAll(&dev, true);

	// // データの読み込み(アドレス0から256バイト取得)
	// // Read 256 byte data from Address=0
	// uint8_t rbuf[READNUMBYTES];    // 取得データ

	// uint32_t modemID = 0x67896789;
	// uint8_t modemID_arr[4];
	// memcpy(modemID_arr, &modemID, 4);

	// printf("byte 0: %u\n", modemID_arr[0]);
	// printf("byte 1: %u\n", modemID_arr[1]);
	// printf("byte 2: %u\n", modemID_arr[2]);
	// printf("byte 3: %u\n", modemID_arr[3]);

	// printf("Test logging init and read for last address written\n");
	// int16_t init_result = W25Q32_initLogging(&dev, modemID_arr);
	// printf("Init result: %d\n", init_result);

	// uint16_t sect_no, inaddr;
	// uint32_t read_modemID;

	// // // read first 6 bytes
	// int16_t lastAddrBytes = 6;
	// memset(rbuf, 0, READNUMBYTES);
	// W25Q64_fastread(&dev, 0, rbuf, READNUMBYTES);
	
	// W25Q32_readLast(rbuf, &sect_no, &inaddr, &read_modemID);
	// printf("Init:\n");
	// printf("Section num: %u\n", sect_no);
	// printf("inaddr: %u\n", inaddr);
	// printf("ModemID: %x\n", read_modemID);
	// printf("\n");

	// int16_t empty_0 = 60000;
	// uint16_t count = 60000;
	// modemID = 0xabcd5678;
	// int empty_1 = 0;
	// uint32_t time = 123456;
	// uint8_t payload_data[PAYLOADSIZE];

	// int i, writePayload_result;
	// for(i = 0; i < 20; i++){
	// 	TagAlongPayload(payload_data, empty_0, count, modemID, empty_1, time);
	// 	writePayload_result = W25Q32_writePayload(&dev, payload_data, PAYLOADSIZE);
	// 	count++;
	// 	time++;
	// }

	// // uint32_t sect_no = 1;
	// // uint32_t inaddr = 0;
	// uint32_t addr = 1;
	// addr<<=12;
	// addr += 0;

	// int len;
	// memset(rbuf, 0, READNUMBYTES);
	// printf("fast read\n");
	// printf("fast read\n");
	// printf("fast read\n");
	// len =  W25Q64_fastread(&dev, addr, rbuf, READNUMBYTES);
	// if (len != READNUMBYTES) {
	// 	ESP_LOGE(TAG, "fastread fail");
	// 	while(1) { vTaskDelay(1); }
	// }
	// ESP_LOGI(TAG, "Fast Read Data: len=%d", len);
	// dump(rbuf, READNUMBYTES);

	// // セクタ単位の削除
	// // Erase data by Sector
	// printf("erase sector\n");
	// printf("erase sector\n");
	// printf("erase sector\n");
	// bool flag = W25Q64_eraseSector(&dev, 0, true);
	// if (flag == false) {
	// 	ESP_LOGE(TAG, "eraseSector fail %d",ret);
	// 	while(1) { vTaskDelay(1); }
	// }
	// memset(rbuf, 0, READNUMBYTES);
	// printf("read\n");
	// printf("read\n");
	// printf("read\n");
	// len =  W25Q64_read(&dev, 0, rbuf, READNUMBYTES);
	// if (len != READNUMBYTES) {
	// 	ESP_LOGE(TAG, "read fail");
	// 	while(1) { vTaskDelay(1); }
	// }
	// ESP_LOGI(TAG, "Read Data: len=%d", len);
	// dump(rbuf, READNUMBYTES);

	// // データ書き込みテスト
	// // Write data to Sector=0 Address=10
	// uint8_t wdata[26];    // 書込みデータ
	// for (int i=0; i<26; i++) {
	// 	wdata[i]='A'+i; // A-Z     
	// }  
	// printf("write\n");
	// printf("write\n");
	// printf("write\n");
	// len =  W25Q64_pageWrite(&dev, 0, 10, wdata, 26);
	// if (len != 26) {
	// 	ESP_LOGE(TAG, "pageWrite fail");
	// 	while(1) { vTaskDelay(1); }
	// }
	// ESP_LOGI(TAG, "Page Write(Sector=0 Address=10) len=%d", len);

	// // 高速データの読み込み(アドレス0から256バイト取得)
	// // First read 256 byte data from Address=0
	// memset(rbuf, 0, READNUMBYTES);
	// printf("fast read\n");
	// printf("fast read\n");
	// printf("fast read\n");
	// len =  W25Q64_fastread(&dev, 0, rbuf, READNUMBYTES);
	// if (len != READNUMBYTES) {
	// 	ESP_LOGE(TAG, "fastread fail");
	// 	while(1) { vTaskDelay(1); }
	// }
	// ESP_LOGI(TAG, "Fast Read Data: len=%d", len);
	// dump(rbuf, READNUMBYTES);

	// // データ書き込みテスト
	// // Write data to Sector=0 Address=0
	// for (int i=0; i < 10;i++) {
	// 	wdata[i]='0'+i; // 0-9     
	// }  
	// printf("write\n");
	// printf("write\n");
	// printf("write\n");
	// len =  W25Q64_pageWrite(&dev, 0, 0, wdata, 10);
	// if (len != 10) {
	// 	ESP_LOGE(TAG, "pageWrite fail");
	// 	while(1) { vTaskDelay(1); }
	// }
	// ESP_LOGI(TAG, "Page Write(Sector=0 Address=0) len=%d", len);

	// // len =  W25Q64_pageWrite(&dev, 0, 16, wdata, );
	// // if (len != 26) {
	// // 	ESP_LOGE(TAG, "pageWrite fail");
	// // 	while(1) { vTaskDelay(1); }
	// // }

	// // 高速データの読み込み(アドレス0から256バイト取得)
	// // First read 256 byte data from Address=0
	// memset(rbuf, 0, READNUMBYTES);
	// printf("fast read\n");
	// printf("fast read\n");
	// printf("fast read\n");
	// len =  W25Q64_fastread(&dev, 0, rbuf, READNUMBYTES);
	// if (len != READNUMBYTES) {
	// 	ESP_LOGE(TAG, "fastread fail");
	// 	while(1) { vTaskDelay(1); }
	// }
	// ESP_LOGI(TAG, "Fast Read Data: len=%d", len);
	// dump(rbuf, READNUMBYTES);

	// ESP_LOGI(TAG, "Success All Test");

	// uint32_t end = xTaskGetTickCount();
	// printf("Current time (ms?): %u\n", end);

	// printf("Time elapsed: %u\n", (end-start));
}

