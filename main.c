#define ABP
#define I2C
#define ADS1115

#include <qm_pinmux.h>
#include <qm_i2c.h>
#include <qm_gpio.h>
#include <clk.h>

#include "lmic/lmic.h"
#include "lmic/hal/hal.h"

#include "pb_decode.h"
#include "eslight/protocol.h"

//include hdc1000 libraries:
#include "hdc1000_read.h"

//include power functions:
//#include "clk.h"
#include "power_states.h"
#include "qm_adc.h"
#include "qm_common.h"
#include "qm_comparator.h"
//#include "qm_gpio.h"
#include "qm_interrupt.h"
#include "qm_interrupt_router.h"
#include "qm_isr.h"
//#include "qm_pinmux.h"
#include "qm_pin_functions.h"
#include "qm_rtc.h"

#ifdef I2C

typedef struct {
	uint8_t addr;
	uint8_t subid;
	uint16_t old_val;
} i2c_dev_t;

#define ADS1015_ADDRESS_GND         (0x48)    // 1001 000 (ADDR = GND)
#define ADS1015_ADDRESS_VCC 		(0x49)	  // 1001 001 (ADDR = VCC)
#define ADS1015_ADDRESS_SDA 		(0x50)	  // 1001 010 (ADDR = SDA)
#define ADS1015_ADDRESS_SCL			(0x51)	  // 1001 011 (ADDR = SCL)
#define HDC1000_ADDRESS_00			(0x40) 	  //1000 000
//#define HDC1000_ADDRESS_01			(0x41)    //1000 001
//#define HDC1000_ADDRESS_10			(0x42)    //1000 010
//#define HDC1000_ADDRESS_11			(0x43)    //1000 011

const i2c_dev_t I2C_DEVICES[] = {
	/* Moisture Sensors test: */
	//{ .addr = ADS1015_ADDRESS_GND, .subid = 1, .old_val = 0 }, //Moisture Sensor 1 	- sensor-1
	//{ .addr = ADS1015_ADDRESS_VCC, .subid = 2, .old_val = 0 }  //Moisture Sensor 2 	- sensor-2

	/* Other Sensors test: */
	{ .addr = ADS1015_ADDRESS_GND, .subid = 1, .old_val = 0 }, //Carbono Monoxide 		- sensor-3
	{ .addr = ADS1015_ADDRESS_VCC, .subid = 2, .old_val = 0 }, //Touch Sensor     		- sensor-4
    { .addr = ADS1015_ADDRESS_SDA, .subid = 3, .old_val = 0 }  //Sound Sensor     		- sensor-5

};

static uint8_t currentI2C = 0;

int setupI2C();

#ifdef ADS1115
uint16_t readADS1115(uint8_t addr, uint8_t channel);
#endif
#endif

#ifdef ABP
/*Moisture Sensors test device: */
//static u4_t DEVADDR = 0x03FF0001; // <-- Change this address for every node!
/*Other Sensors test device: */
static u4_t DEVADDR = 0x03FF0002; // <-- Change this address for every node!
static u1_t NWKSKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
static u1_t APPSKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
#else

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t APPEUI[8]={ 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02 };
void os_getArtEui (u1_t* buf) { memcpy(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
static const u1_t DEVEUI[8]={ 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 };
void os_getDevEui (u1_t* buf) { memcpy(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t APPKEY[16] = { 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03 };
void os_getDevKey (u1_t* buf) { memcpy(buf, APPKEY, 16); }
#endif

static uint8_t msg[30];
static size_t msglen;

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 1;

const lmic_pinmap lmic_pins = {
    .nss = QM_PIN_ID_0,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = QM_PIN_ID_15,
    .dio = { QM_PIN_ID_14, LMIC_UNUSED_PIN, QM_PIN_ID_5 }
};

void do_send(osjob_t* j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        // Serial.println(F("OP_TXRXPEND, not sending"));
    	int i=0;
    	i++;
    } else {
    	uint16_t val;
    	//hdc1000_sensor_data_t sensor_info;
    	//int status;
    	/*if(I2C_DEVICES[currentI2C].subid == 3){
    		//read HDC1000 Temperature
    		//hdc1000_sensor_read(&sensor_info, HDC1000_MEASUREMENT_MODE_TEMPERATURE);
    		//val = sensor_info.temperature;
    		//val = 23;
    		status = i2c_init();

    		if(status){
    			//printf("i2c init failed!!!");
    			//val = 1;
    		}
    		else{
    			status = hdc1000_sensor_init(HDC1000_MEASUREMENT_MODE_COMBINED, HDC1000_RESOLUTION_14BIT,
    					HDC1000_BATTERY_STATUS_LOW_INDICATION_DISABLE,
						HDC1000_DO_SOFT_RESET);
    			if(status){
    				//printf("sensor init error!!!");
    				//val = 3;
    			}
    			else{
    				status = sensor_read(&sensor_info, HDC1000_MEASUREMENT_MODE_COMBINED, 1);

    				if(status){
    					//printf("sensor read error!!!");
    					//val = 2;
    				}
    				else{
    					val = sensor_info.temperature;
    				}
    			}
    		}
    	}
    	else if(I2C_DEVICES[currentI2C].subid == 4){
    		//read HDC1000 Humidity
    		//hdc1000_sensor_read(&sensor_info, HDC1000_MEASUREMENT_MODE_HUMIDITY);
    		//val = sensor_info.humidity;
    		//val = 65;
    		status = i2c_init();
    		if(status){
    			//printf("i2c init failed!!!");
    			val = 1;
    		}
    		else{
    			status = hdc1000_sensor_init(HDC1000_MEASUREMENT_MODE_COMBINED, HDC1000_RESOLUTION_14BIT,
    					HDC1000_BATTERY_STATUS_LOW_INDICATION_DISABLE,
						HDC1000_DO_SOFT_RESET);
    			if(status){
    				//printf("sensor init error!!!");
    				val = status;
    			}
    			else{
    				status = sensor_read(&sensor_info, HDC1000_MEASUREMENT_MODE_COMBINED, 1);

    				if(status){
    					//printf("sensor read error!!!");
    					val = 2;
    				}
    				else{
    					val = sensor_info.humidity;
    				}
    			}
    		}
    	}*/
    	//else{
    		val = readADS1115(I2C_DEVICES[currentI2C].addr, 0);
    		//val = 7; 0 --- agua, 100%
    		//100% in water S1: 15600 - 16211	//Moisture Sensor 1 (left sensor)
    		//              S2: 13635 - 16833 	//Moisture Sensor 2 (right sensor)
    		//				S1,S2        >13500 //in water
    		//			 	S1,S2: 8500 - 13500 //muy mojado
    		//				S1,S2: 4600 -  8500 //mojado
    		//				S1,S2: 2500 -  4600 //húmedo
    		//				S1,S2:  500 -  2500	//poco húmedo
    		//				S1,S2: 0-500,>65000 //seco

    		//0% dry		S1: 65535 S2: 65532
    		// delta(max-min): 53247

    		//val = 100 - 100 * (val - min)/(max-min)
    		//LUA SCRIPT Moisture % formula:
    		//float val = 123.07735647 - 100 * val / 53247;
    	//}
    	printf("S%i: %i",I2C_DEVICES[currentI2C].subid, val);//, I2C_DEVICES[currentI2C].old_val);
    	/*switch(I2C_DEVICES[currentI2C].subid){
    	case 1: printf("\tMo1\n\r");
    			break;
    	case 2:	printf("\tMo2\n\r");
				break;
    	case 3:	printf("\tT\n\r");
				break;
    	case 4:	printf("\tH\n\r");
				break;
    	default: break;
    	}*/

    	/*Moisture Sensor Test output: */
    	//if(val<500){printf("\tSeco\n\r");}
    	//else if(val<2500){printf("\tPoco humedo\n\r");}
    	//else if(val<4600){printf("\tHúmedo\n\r");}
    	//else if(val<8500){printf("\tMojado\n\r");}
    	//else if(val<13500){printf("\tMuy mojado\n\r");}
    	//else if(val<65000){printf("\tEn agua\n\r");}
    	//else{printf("\tSin agua\n\r");}


    	currentI2C++;
    	if (currentI2C >= sizeof(I2C_DEVICES) / sizeof(I2C_DEVICES[0])) currentI2C = 0;

    	esl_ValueUpdate vm = esl_ValueUpdate_init_zero;;
    	vm.id = I2C_DEVICES[currentI2C].subid;
    	vm.has_int_val = true;
    	vm.int_val = val;

    	msglen = esl_encode_value_update(msg, sizeof(msg), &vm);
        LMIC_setTxData2(1, msg, msglen, 0);
    }
}


static void rtc_sleep_wakeup()
{

	qm_rtc_config_t rtc_cfg;

	clk_periph_enable(CLK_PERIPH_RTC_REGISTER | CLK_PERIPH_CLK);

	/*
	 * Setup the RTC to get out of sleep mode. Deep sleep will require an
	 * analog comparator interrupt to wake up the system.
	 */
	rtc_cfg.init_val = 0;
	rtc_cfg.alarm_en = 1;
	rtc_cfg.alarm_val = QM_RTC_ALARM_SECOND(CLK_RTC_DIV_1);
	rtc_cfg.callback = NULL;
	rtc_cfg.callback_data = NULL;
	rtc_cfg.prescaler = CLK_RTC_DIV_1;
	qm_rtc_set_config(QM_RTC_0, &rtc_cfg);

	QM_IR_UNMASK_INT(QM_IRQ_RTC_0_INT);

	QM_IRQ_REQUEST(QM_IRQ_RTC_0_INT, qm_rtc_0_isr);

	QM_PUTS("CPU Halt.");
	/* Halt the CPU, RTC alarm will wake. */
	qm_power_cpu_halt();
	QM_PUTS("CPU Halt wakeup.");

	/* Setup wake up isr for RTC. */
	QM_IRQ_REQUEST(QM_IRQ_RTC_0_INT, qm_rtc_0_isr);

	/* Set another alarm one second from now. */
	qm_rtc_set_alarm(QM_RTC_0, QM_RTC[QM_RTC_0]->rtc_ccvr +
				       QM_RTC_ALARM_SECOND(CLK_RTC_DIV_1));
	QM_PUTS("Go to sleep.");
	/* Go to sleep, RTC will wake. */
	qm_power_soc_sleep();
	QM_PUTS("Wake up from sleep.");

	QM_PUTS("Go to deep sleep with RTC.");
	qm_rtc_set_alarm(QM_RTC_0, QM_RTC[QM_RTC_0]->rtc_ccvr +
				       QM_RTC_ALARM_SECOND(CLK_RTC_DIV_1) * 600);
	qm_power_soc_deep_sleep(QM_POWER_WAKE_FROM_RTC);
	// Schedule next transmission
	os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);

}

void onEvent (ev_t ev)
{
    // Serial.print(os_getTime());
    // Serial.print(": ");
    switch(ev) {
    case EV_SCAN_TIMEOUT:
    	// Serial.println(F("EV_SCAN_TIMEOUT"));
		break;
	case EV_BEACON_FOUND:
		// Serial.println(F("EV_BEACON_FOUND"));
		break;
	case EV_BEACON_MISSED:
		// Serial.println(F("EV_BEACON_MISSED"));
		break;
	case EV_BEACON_TRACKED:
		// Serial.println(F("EV_BEACON_TRACKED"));
		break;
	case EV_JOINING:
		// Serial.println(F("EV_JOINING"));
		break;
	case EV_JOINED:
		// Serial.println(F("EV_JOINED"));

		// Disable link check validation (automatically enabled
		// during join, but not supported by TTN at this time).
		LMIC_setLinkCheckMode(0);
		break;
	case EV_RFU1:
		// Serial.println(F("EV_RFU1"));
		break;
	case EV_JOIN_FAILED:
		// Serial.println(F("EV_JOIN_FAILED"));
		break;
	case EV_REJOIN_FAILED:
		// Serial.println(F("EV_REJOIN_FAILED"));
		break;
	case EV_TXCOMPLETE:
		// Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
		if (LMIC.txrxFlags & TXRX_ACK) {
			//   Serial.println(F("Received ack"));
		}

		if (LMIC.dataLen > 0) {
			// Serial.println(F("Received "));
			// Serial.println(LMIC.dataLen);
			// Serial.println(F(" bytes of payload"));
			// LMIC.frame[LMIC.dataBeg]

			esl_ValueUpdate rq = esl_ValueUpdate_init_zero;
			pb_istream_t stream = pb_istream_from_buffer(&LMIC.frame[LMIC.dataBeg], LMIC.dataLen);
			uint8_t type = esl_decode(&stream, &rq);

			switch (type) {
			case esl_MsgType_VALUE_UPDATE:
				if (rq.has_bool_val) {
					if (rq.bool_val) {
						qm_gpio_set_pin(QM_GPIO_0, 24);
					} else {
						qm_gpio_clear_pin(QM_GPIO_0, 24);
					}

					// TODO: Reply relay changed
					// vm.has_bool_val = true;
					// vm.bool_val = rq.bool_val;
				}
				break;
			}
		}
		/* Set up the RTC to wake up the SoC from sleep. */
		rtc_sleep_wakeup();
		break;
	case EV_LOST_TSYNC:
		// Serial.println(F("EV_LOST_TSYNC"));
		break;
	case EV_RESET:
		// Serial.println(F("EV_RESET"));
		break;
	case EV_RXCOMPLETE:
		// data received in ping slot
		// Serial.println(F("EV_RXCOMPLETE"));
		break;
	case EV_LINK_DEAD:
		// Serial.println(F("EV_LINK_DEAD"));
		break;
	case EV_LINK_ALIVE:
		// Serial.println(F("EV_LINK_ALIVE"));
		break;
	 default:
		 // Serial.println(F("Unknown event"));
		break;
    }
}

#ifdef I2C
int setupI2C()
{
	/*  Enable I2C 0 */
	clk_periph_enable(CLK_PERIPH_CLK | CLK_PERIPH_I2C_M0_REGISTER);

	/* set IO pins for SDA and SCL */
	qm_pmux_select(QM_PIN_ID_6, QM_PMUX_FN_2);
	qm_pmux_select(QM_PIN_ID_7, QM_PMUX_FN_2);

	/* Configure I2C */
	qm_i2c_config_t I2Ccfg;
	I2Ccfg.address_mode = QM_I2C_7_BIT;
	I2Ccfg.mode = QM_I2C_MASTER;
	I2Ccfg.speed = QM_I2C_SPEED_STD;

	/* set the configuration through the structure and return if failure */
	if (qm_i2c_set_config(QM_I2C_0, &I2Ccfg) == 0) {
		// Ok
		return true;
	}

	return false;
}

#ifdef ADS1115
#define ADS1115_REG_POINTER_CONVERT

int writeRegister(uint8_t i2cAddress, uint8_t reg, uint16_t value)
{
	qm_i2c_status_t status;
	int rc;

	rc = qm_i2c_master_write(QM_I2C_0, i2cAddress, &reg, sizeof(reg), false, &status);
	if (rc) return rc;

	uint8_t data = (uint8_t)(value>>8);
	rc = qm_i2c_master_write(QM_I2C_0, i2cAddress, &data, sizeof(data), false, &status);
	if (rc) return rc;

	data = (uint8_t)(value & 0xFF);
	rc = qm_i2c_master_write(QM_I2C_0, i2cAddress, &data, sizeof(data), true, &status);
	if (rc) return rc;

	return 0;
}

uint16_t readRegister(uint8_t i2cAddress, uint8_t reg)
{
	qm_i2c_status_t status;
	int rc;

	rc = qm_i2c_master_write(QM_I2C_0, i2cAddress, &reg, sizeof(reg), true, &status);
	if (rc) return rc;

	uint8_t data[2];
	rc = qm_i2c_master_read(QM_I2C_0, i2cAddress, data, sizeof(data), true, &status);
	if (rc) return rc;

	return ((uint16_t)(data[0] << 8)) | data[1];
}

uint16_t readADS1115(uint8_t addr, uint8_t channel)
{
	if (channel > 3)
		return 0;

#define ADS1015_REG_CONFIG_CQUE_NONE    (0x0003)  // Disable the comparator and put ALERT/RDY in high state (default)
#define ADS1015_REG_CONFIG_CLAT_NONLAT  (0x0000)  // Non-latching comparator (default)
#define ADS1015_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default)
#define ADS1015_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default)
#define ADS1015_REG_CONFIG_CMODE_TRAD   (0x0000)  // Traditional comparator with hysteresis (default)
#define ADS1015_REG_CONFIG_DR_1600SPS   (0x0080)  // 1600 samples per second (default)
#define ADS1015_REG_CONFIG_MODE_SINGLE  (0x0100)  // Power-down single-shot mode (default)
#define ADS1015_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3

#define ADS1015_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
#define ADS1015_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
#define ADS1015_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
#define ADS1015_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3
#define ADS1015_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: Set to start a single-conversion

#define ADS1015_REG_POINTER_CONVERT     (0x00)
#define ADS1015_REG_POINTER_CONFIG      (0x01)

#define ADS1115_CONVERSIONDELAY         (8)		  // ms

	uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
			ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
			ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
			ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
			ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
			ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

	  // Set PGA/voltage range
	  config |= ADS1015_REG_CONFIG_PGA_6_144V;

	  // Set single-ended input channel
	  switch (channel) {
	    case (0):
	      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
	      break;
	    case (1):
	      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
	      break;
	    case (2):
	      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
	      break;
	    case (3):
	      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
	      break;
	  }

	  // Set 'start single-conversion' bit
	  config |= ADS1015_REG_CONFIG_OS_SINGLE;

	  // Write config register to the ADC
	  writeRegister(addr, ADS1015_REG_POINTER_CONFIG, config);

	  // Wait for the conversion to complete
	  clk_sys_udelay(ADS1115_CONVERSIONDELAY * 1000);

	  // Read the conversion results
	  // Shift 12-bit results right 4 bits for the ADS1015
	  return readRegister(addr, ADS1015_REG_POINTER_CONVERT);
}
#endif
#endif

int main(void)
{
#ifdef I2C
	setupI2C();
#endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // For ABP
#ifdef ABP
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

    /*
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
	LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    */

    //by default all 72 channels are enabled, so disable all channels which the gateway isn´t listen to (8 to 72)
    for(int channel = 8; channel < 72; channel++){
    	LMIC_disableChannel(channel);
    }
	// Disable link check validation
	LMIC_setLinkCheckMode(0);

	// TTN uses SF9 for its RX2 window.
	LMIC.dn2Dr = DR_SF9;

	// Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
	LMIC_setDrTxpow(DR_SF7, 14);
#endif

    LMIC_setClockError( MAX_CLOCK_ERROR * 1 / 100);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);

    os_runloop();

    return 0;
}
