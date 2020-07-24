/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello, world!", that
 * will be processed by The Things Network server.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1, 
*  0.1% in g2). 
 *
 * Change DEVADDR to a unique address! 
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h, default is:
 *   #define CFG_sx1272_radio 1
 * for SX1272 and RFM92, but change to:
 *   #define CFG_sx1276_radio 1
 * for SX1276 and RFM95.
 *

function Decoder(b, port) {
  var lat = (b[0] | b[1]<<8 | b[2]<<16 ) / -10000;
  var lng = (b[3] | b[4]<<8 | b[5]<<16 ) / 10000;
  var alt = b[6];
  var hdop = b[7] / 100;
  if (b[0] == 21 || isNaN(alt)) {
    return {hashbang:'!'};
  }
  return {
    latitude: lat, longitude: lng, altitude: alt, hdop: hdop
  };
}

 *******************************************************************************/

#include <stdio.h>
#include <time.h>
#include <wiringPi.h>
#include <lmic.h>
#include <hal.h>
#include <gps.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

uint8_t coords[8];

int rc;
struct timeval tv;

struct gps_data_t gps_d;

// TTN-MAPPER lorahat01
static const u1_t APPEUI[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const u1_t DEVEUI[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const u1_t APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
//static const u4_t DEVADDR = 0x260411B6; // <-- Change this address for every node!

//////////////////////////////////////////////////
// APPLICATION CALLBACKS
//////////////////////////////////////////////////

// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
    memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
    memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
    memcpy(buf, APPKEY, 16);
}

u4_t cntr=0;
u1_t coords[8];
static osjob_t sendjob;
static osjob_t gpsjob;

void onEvent (ev_t ev) {
    fprintf(stderr, "Ev:%d\n", ev);

    switch(ev) {
      // scheduled data sent (optionally data received)
      // note: this includes the receive window!
      case EV_TXCOMPLETE:
          // use this event to keep track of actual transmissions
          fprintf(stdout, "Event EV_TXCOMPLETE, time: %d\n", millis() / 1000);
          if(LMIC.dataLen) { // data received in rx slot after tx
              //debug_buf(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              fprintf(stdout, "Data Received!\n");
          }
          break;
       default:
          break;
    }
}

static void do_gps_read(osjob_t* j) {
    /* wait for .5 seconds to receive data */
    if (gps_waiting (&gps_d, 500000)) {
        /* read data */
        if ((rc = gps_read(&gps_d)) == -1) {
            printf("error occured reading gps data. code: %d, reason: %s\n", rc, gps_errstr(rc));
        } else {
            /* Display data from the GPS receiver. */
            if ((gps_d.status == STATUS_FIX || gps_d.status == STATUS_DGPS_FIX) &&
                (gps_d.fix.mode == MODE_2D || gps_d.fix.mode == MODE_3D) &&
                !isnan(gps_d.fix.latitude) &&
                !isnan(gps_d.fix.longitude)) {
                    printf("latitude: %f, longitude: %f, altitude: %f, hdop: %f, timestamp: %lf\n", 
				    gps_d.fix.latitude, 
				    gps_d.fix.longitude, 
				    gps_d.fix.altitude, 
				    gps_d.dop.hdop, 
				    gps_d.fix.time);
		int32_t lat = gps_d.fix.latitude * 10000;
                int32_t lon = gps_d.fix.longitude * 10000;

                // Pad 2 int32_t to 6 8uint_t, skipping the last byte (x >> 24)
                coords[0] = lat;
		coords[1] = lat >> 8;
		coords[2] = lat >> 16;
		coords[3] = lon;
		coords[4] = lon >> 8;
		coords[5] = lon >> 16;

		coords[6] = floor(gps_d.fix.altitude);
		coords[7] = gps_d.dop.hdop*100;
		for (int i = 0; i < 8; i++)
		{ 
			printf("%02X ", coords[i]);
		}

		printf("\nPKT latitude: %.1f, longitude: %.1f, altitude: %.1f, hdop: %.1f\n",
				floor(gps_d.fix.latitude*10000), 
				floor(gps_d.fix.longitude*10000), 
				floor(gps_d.fix.altitude), 
				floor(gps_d.dop.hdop*100));
            } else {
                printf("no GPS data available, hdop: %f\n", gps_d.dop.hdop);
		coords[0] = '!';
            }
        }
    } else {
	    printf("GPS Wait timeout\n");
    }

    os_setTimedCallback(j, os_getTime()+sec2osticks(3), do_gps_read);
}

static void do_send(osjob_t* j){
      time_t t=time(NULL);
      fprintf(stdout, "[%x] (%ld) %s\n", hal_ticks(), t, ctime(&t));
      // Show TX channel (channel numbers are local to LMIC)
      // Check if there is not a current TX/RX job running
    if (LMIC.opmode & (1 << 7)) {
      fprintf(stdout, "OP_TXRXPEND, not sending");
    } else {
      // Prepare upstream data transmission at the next possible time.
      if (coords[0] != '!') {
	fprintf(stdout,"Sending GPS data");
        LMIC_setTxData2(2, coords, 8, 0);
      } else {
	fprintf(stdout,"NOT Sending GPS data");
        LMIC_setTxData2(2, coords, 1, 0);
      }
    }
    // Schedule a timed job to run at the given timestamp (absolute system time)
    os_setTimedCallback(j, os_getTime()+sec2osticks(10), do_send);
         
}

void setup() {
  // LMIC init
  wiringPiSetup();

  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // set australian sub band 2
  //LMIC_selectSubBand(1);
  // Set static session parameters. Instead of dynamically establishing a session 
  // by joining the network, precomputed session parameters are be provided.
  //LMIC_setSession (0x1, DEVADDR, (u1_t*)DEVKEY, (u1_t*)ARTKEY);
  LMIC_startJoining();
  // Disable data rate adaptation
  LMIC_setAdrMode(0);
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // Disable beacon tracking
  LMIC_disableTracking ();
  // Stop listening for downstream data (periodical reception)
  LMIC_stopPingable();
  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14);
  // GPS setup 
if ((rc = gps_open("localhost", "2947", &gps_d)) == -1) {
    printf("GPS ERROR code: %d, reason: %s\n", rc, gps_errstr(rc));
}
gps_stream(&gps_d, WATCH_ENABLE | WATCH_JSON, NULL);

}

void loop() {
coords[0] = '!';
// enable gps reading
do_gps_read(&gpsjob);
do_send(&sendjob);

while(1) {
  os_runloop();
//  os_runloop_once();
  }
}


int main() {
  setup();

  while (1) {
    loop();
  }

/* When you are done... */
gps_stream(&gps_d, WATCH_DISABLE, NULL);
gps_close (&gps_d);

  return 0;
}

