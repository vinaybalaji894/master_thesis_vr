/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * Simple user application.
 *
 * Copyright (c) 2016-2017, LEAPS. All rights reserved.
 *
 */

#include "dwm.h"
#include <stdio.h>

/* Thread priority */
#ifndef THREAD_APP_PRIO
#define THREAD_APP_PRIO	20
#endif /* THREAD_APP_PRIO */

/* Thread stack size */
#ifndef THREAD_APP_STACK_SIZE
#define THREAD_APP_STACK_SIZE	(3 * 1024)
#endif /* THREAD_APP_STACK_SIZE */

#define APP_ERR_CHECK(err_code)	\
do {							\
	if ((err_code) != DWM_OK)	\
		printf("err: line(%u) code(%u)", __LINE__, (err_code));\
} while (0)						\

#define MSG_INIT	\
	"\n\n"	\
	"App   :  dwm-simple\n"	\
	"Built :  " __DATE__ " " __TIME__ "\n"	\
	"\n"

/**
 * Event callback
 *
 * @param[in] p_evt  Pointer to event structure
 * @param[in] p_data Pointer to user data
 *
 *
 */
int return_value;
void on_dwm_evt(dwm_evt_t *p_evt, void *p_data)
{
	int i;

	switch (p_evt->header.id) {
	/* New location data */
	case DWM_EVT_NEW_LOC_DATA:
		/* Process the data */

		printf("\nT:%lu ", dwm_systime_us_get());
		if (p_evt->data.loc.p_pos == 0) {
			/* Location engine is disabled */
		} else {
			printf("POS:[%ld,%ld,%ld,%u] ", p_evt->data.loc.p_pos->x,
					p_evt->data.loc.p_pos->y, p_evt->data.loc.p_pos->z,
					p_evt->data.loc.p_pos->qf);
		}

		for (i = 0; i < p_evt->data.loc.anchors.dist.cnt; ++i) {
			printf("DIST%d:", i);

			printf("0x%04X", (unsigned int)(p_evt->data.loc.anchors.dist.addr[i] & 0xffff));
			if (i < p_evt->data.loc.anchors.an_pos.cnt) {
				printf("[%ld,%ld,%ld]",
						p_evt->data.loc.anchors.an_pos.pos[i].x,
						p_evt->data.loc.anchors.an_pos.pos[i].y,
						p_evt->data.loc.anchors.an_pos.pos[i].z);
			}

			printf("=[%lu,%u] ", p_evt->data.loc.anchors.dist.dist[i],
					p_evt->data.loc.anchors.dist.qf[i]);
		}
		printf("\n");
		break;

	default:
		break;
	}

	/* Indicate the application has finished the tasks and can now */
	dwm_sleep();
}

/**
 * Application thread
 *
 * @param[in] data  Pointer to user data
 */

int acc_setup()
{
	uint8_t setbyte[2];
    setbyte[0]=0x20;
    setbyte[1]=0x97;
	uint8_t rcbyte;
	dwm_i2c_write(0x33>>1,&setbyte,2,true);
	dwm_i2c_read(0x33>>1,&rcbyte,1);
	uint8_t reg4[2];
	reg4[0]=0x23;
	reg4[1]=0x18;
	uint8_t rcreg4;
	dwm_i2c_write(0x33>>1,&reg4,2,true);
	dwm_i2c_read(0x33>>1,&rcreg4,1);
	printf("%d\t%d\n",rcbyte,rcreg4);
   // flag=3;
	return 0;
}


int acc_val()
{  //Function to receive the accelerometer data directly from the registers

	// Declaration of the receive buffer
	      uint8_t rvdata[6];
	// Declaration of the transmit buffer. Input parameter the address of the LSB of the x-axis of the
	// accelerometer. Set the MSB to 1 in order to iterate and read multiple addresses in one shot.
	      uint8_t icbyte=0xA8;
    // Master request
		  dwm_i2c_write(0x33>>1,&icbyte,1,true);
    // Slave reply. Reading the LSB and MSB of all axes into the single buffer
		  dwm_i2c_read(0x33>>1,&rvdata,6);
    // Print Statement: for debugging
	      //printf("%d\t%d\t%d\t%d\t%d\t%d\t\n",rvdata[0],rvdata[1],rvdata[2],rvdata[3],rvdata[4],rvdata[5]);
		  //printf("Next iteration\n");

    // Signed 16 - bit integers for the X,Y and Z access
		 // int16_t acc_x;
		  acv.acc_x=rvdata[1];
          acv.acc_x=(acv.acc_x<<8)|(rvdata[0]);

		 // int16_t acc_y;
		  acv.acc_y=rvdata[3];
		  acv.acc_y=(acv.acc_y<<8)|(rvdata[2]);

		  //int16_t acc_z;
		  acv.acc_z=rvdata[5];
		  acv.acc_z=(acv.acc_z<<8)|(rvdata[4]);
     // Print the values for acceleration, for debugging purposes
		  //printf("%d\t%d\t%d\t\n",acv.acc_x,acv.acc_y,acv.acc_z);
		  //printf("Iteration Change\n");

return 0;
}
//ioannis
void accelerometer_setup(void)
{
uint8_t return_value;
uint8_t bytes[2];

// reboot memory
bytes[0] = 0x24;
bytes[1] = 0x80;
return_value = dwm_i2c_write(0x33 >> 1, bytes, 2, false);
APP_ERR_CHECK(return_value);

// power-up - 200Hz = 67
bytes[0] = 0x20;
bytes[1] = 0x67;
return_value = dwm_i2c_write(0x33 >> 1, bytes, 2, false);
APP_ERR_CHECK(return_value);
//
// // INT1 ZYXDA
// bytes[0] = 0x22;
// bytes[1] = 0x10;
// return_value = dwm_i2c_write(0x33 >> 1, bytes, 2, false);
// APP_ERR_CHECK(return_value);

// enable fifo
bytes[0] = 0x24;
bytes[1] = 0x40;
return_value = dwm_i2c_write(0x33 >> 1, bytes, 2, false);
APP_ERR_CHECK(return_value);

//emable bypass mode
bytes[0] = 0x2e;
bytes[1] = 0x00;
return_value = dwm_i2c_write(0x33 >> 1, bytes, 2, false);
APP_ERR_CHECK(return_value);
}

void app_thread_entry(uint32_t data)
{
	dwm_cfg_tag_t cfg_tag;
	dwm_cfg_t cfg;
	uint8_t i2cbyte;
	int rv;
	dwm_pos_t pos;

	/* Initial message */
	printf(MSG_INIT);

	/* Get node configuration */
	APP_ERR_CHECK(dwm_cfg_get(&cfg));

	if ((cfg.mode != DWM_MODE_TAG) ||
		(cfg.accel_en != false) ||
		(cfg.loc_engine_en != true) ||
		(cfg.meas_mode != DWM_MEAS_MODE_TWR) ||
		(cfg.low_power_en != false) ||
		(cfg.common.fw_update_en != false) ||
		(cfg.common.uwb_mode != DWM_UWB_MODE_ACTIVE) ||
		(cfg.common.ble_en != true) ||
		(cfg.common.led_en != true)) {

		/* Configure device as TAG */
		cfg_tag.accel_en = false;
		cfg_tag.loc_engine_en = true;
		cfg_tag.low_power_en = false;
		cfg_tag.meas_mode = DWM_MEAS_MODE_TWR;
		cfg_tag.common.fw_update_en = false;
		cfg_tag.common.uwb_mode = DWM_UWB_MODE_ACTIVE;
		cfg_tag.common.ble_en = true;
		cfg_tag.common.led_en = true;
		APP_ERR_CHECK(dwm_cfg_tag_set(&cfg_tag));

		dwm_reset();
	}

	/* Update rate set to 1 second, stationary update rate set to 5 seconds */
	APP_ERR_CHECK(dwm_upd_rate_set(10, 50));

	/* Register event callback */
	dwm_evt_cb_register(on_dwm_evt, 0);

	/* Test the accelerometer */
	i2cbyte = 0x0f;
	rv = dwm_i2c_write(0x33 >> 1, &i2cbyte, 1, true);

	if (rv == DWM_OK) {
		rv = dwm_i2c_read(0x33 >> 1, &i2cbyte, 1);

		if (rv == DWM_OK) {
			printf("Accelerometer chip ID: %u\n", i2cbyte);
		} else {
			printf("i2c: read failed (%d)\n", rv);
		}
	} else {
		printf("i2c: write failed (%d)\n", rv);
	}
	//acc_setup();
	accelerometer_setup();

    // while (1) {
		/* Thread loop */
		//dwm_pos_get(&pos);
		//printf("x=%ld, y=%ld, z=%ld, qf=%u \n", pos.x, pos.y, pos.z, pos.qf);
		//printf("\t\t time=%lu \n", dwm_systime_us_get());
		//printf("Hello Dickhead\n");
		//acc_val();
		//printf("%lu,%d,%d,%d\n",dwm_systime_us_get(),acv.acc_x,acv.acc_y,acv.acc_z);
		//printf("Accelerometer chip ID: %u\n", i2cbyte);
		//dwm_thread_delay(100);
	//}
//ioannis
	while (1) {
    uint8_t bytes[6];
	// Poll status register for new data on any axis
	bytes[0] = 0x27;
	return_value = dwm_i2c_write(0x33 >> 1, bytes, 1, false);
	APP_ERR_CHECK(return_value);
	return_value = dwm_i2c_read(0x33 >> 1, bytes, 1);
	APP_ERR_CHECK(return_value);

	if (bytes[0] & 0x08) // if xyzda bit is set, get data
	{
	bytes[0] = 0xA7; // 0x27 + 1 MSB for consecutive read
	return_value = dwm_i2c_write(0x33 >> 1, bytes, 1, false);
	APP_ERR_CHECK(return_value);
	return_value = dwm_i2c_read(0x33 >> 1, bytes, 1+6);
	APP_ERR_CHECK(return_value);
	printf("%lu   %5d,%5d,%5d\n", dwm_systime_us_get(), (int16_t)( bytes[1] | (bytes[2]<<8)), (int16_t)( bytes[3] | (bytes[4]<<8)), (int16_t)( bytes[5] | (bytes[6]<<8)));
	// putchar(bytes[0]+0x57);
	// putchar('\n');
	}
	}


}


/**
 * Application entry point. Initialize application thread.
 *
 * @warning ONLY ENABLING OF LOCATION ENGINE OR BLE AND CREATION AND STARTING OF
 * USER THREADS CAN BE DONE IN THIS FUNCTION
 */
void dwm_user_start(void)
{
	uint8_t hndl;
	int rv;

	//dwm_shell_compile();
	dwm_ble_compile();
	dwm_le_compile();
	//dwm_serial_compile();

	/* Create thread */
	rv = dwm_thread_create(THREAD_APP_PRIO, app_thread_entry, (void*)NULL,
			"app", THREAD_APP_STACK_SIZE, &hndl);
	APP_ERR_CHECK(rv);

	/* Start the thread */
	dwm_thread_resume(hndl);
}
