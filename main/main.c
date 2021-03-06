#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"

#include "lmic.h"

//u1_t NWKSKEY[16] = { 0xa2, 0x25, 0xb5, 0x22, 0x8f, 0xe5, 0xb8, 0x18, 0xb6, 0x19, 0xc1, 0xb0, 0x9b, 0xc8, 0xdb, 0xd2 };
//u1_t APPSKEY[16] = { 0xfa, 0x4c, 0xd4, 0xd2, 0x78, 0x0d, 0x77, 0xc1, 0x03, 0x82, 0xab, 0x77, 0xdb, 0xd8, 0xe7, 0x00 };
//u4_t DEVADDR = 0x07a56a5e;


u1_t NWKSKEY[16] =  {0x19, 0xce, 0x3e, 0x90, 0x4a, 0x53, 0x25, 0x98, 0x27, 0xe1, 0x92, 0xf1, 0x83, 0x59, 0x0c, 0xe0 };
u1_t APPSKEY[16] =  {0x8e, 0x20, 0xb0, 0x64, 0xcf, 0x44, 0x21, 0x9a, 0xd2, 0x9c, 0x81, 0x32, 0x98, 0xbc, 0xd6, 0x42 };
u4_t DEVADDR = 0x079ff83f;

//static const u1_t DEVEUI[8]={ 0xC5, 0x5A, 0x7C, 0x0A, 0x00, 0xF0, 0x07, 0x00 };
//void os_getDevEui (u1_t* buf) { }
//static const u1_t APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//void os_getArtEui (u1_t* buf) { }
//static const u1_t APPKEY[16] = { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10};
//void os_getDevKey (u1_t* buf) {  }

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


static uint8_t mydata[] = "Ciao!";

const unsigned TX_INTERVAL = 60;

void onEvent (ev_t ev) {
    printf("%d", os_getTime());
    printf(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            printf("EV_SCAN_TIMEOUT");
            break;
        case EV_BEACON_FOUND:
            printf("EV_BEACON_FOUND");
            break;
        case EV_BEACON_MISSED:
            printf("EV_BEACON_MISSED");
            break;
        case EV_BEACON_TRACKED:
            printf("EV_BEACON_TRACKED");
            break;
        case EV_JOINING:
            printf("EV_JOINING");
            break;
        case EV_JOINED:
            printf("EV_JOINED");
            break;
        case EV_RFU1:
            printf("EV_RFU1");
            break;
        case EV_JOIN_FAILED:
            printf("EV_JOIN_FAILED");
            break;
        case EV_REJOIN_FAILED:
            printf("EV_REJOIN_FAILED");
            break;
        case EV_TXCOMPLETE:
            printf("EV_TXCOMPLETE (includes waiting for RX windows)");
            if (LMIC.txrxFlags & TXRX_ACK)
              printf("Received ack");
            if (LMIC.dataLen) {
              printf("Received ");
              printf(LMIC.dataLen);
              printf(" bytes of payload");
            }

            if (LMIC.opmode & OP_TXRXPEND) {
                printf("OP_TXRXPEND, not sending");
            } else {
                // Prepare upstream data transmission at the next possible time.
                LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
                printf("Packet queued\n");
            }
            break;
        case EV_LOST_TSYNC:
            printf("EV_LOST_TSYNC");
            break;
        case EV_RESET:
            printf("EV_RESET");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            printf("EV_RXCOMPLETE");
            break;
        case EV_LINK_DEAD:
            printf("EV_LINK_DEAD");
            break;
        case EV_LINK_ALIVE:
            printf("EV_LINK_ALIVE");
            break;
         default:
            printf("Unknown event: %d\n", ev);
            break;
    }
}

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

void os_runloop(void) {

  if (LMIC.opmode & OP_TXRXPEND) {
      printf("OP_TXRXPEND, not sending");
  } else {
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
      printf("Packet queued");
  }
 vTaskDelay(TX_INTERVAL * 1000 / portTICK_PERIOD_MS);

}

void app_main(void)
{
  os_init();

  LMIC_reset();
  printf("LMIC RESET\n");

  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI); // g2-band

  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
//LMIC_setDrTxpow(DR_SF10,14);
  LMIC_setDrTxpow(DR_SF7,14);

  for(int i = 1; i <= 8; i++) LMIC_disableChannel(i);
  //printf("Avvio Join\n");
  //LMIC_startJoining();
  //printf("Join Completo\n");
  	
  xTaskCreate(os_runloop, "os_runloop", 1024 * 2, (void* )0, 10, NULL);
}
