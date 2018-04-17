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
#include "power_states.h"
#include "qm_adc.h"
#include "qm_common.h"
#include "qm_comparator.h"
#include "qm_interrupt.h"
#include "qm_interrupt_router.h"
#include "qm_isr.h"
#include "qm_pin_functions.h"
#include "qm_rtc.h"

//include Cayenne Low Power Payload libraries
#include "cayenne-lpp/cayenne_lpp.h"
#include "cayenne-lpp/cayenne_lpp.c"

static cayenne_lpp_t lpp;

uint8_t converted_val_uint8_t;		//for digital_input, digital_output, presence
uint16_t converted_val_uint16_t;	//for luminosity
float converted_val_float;			//for analog_input, analog_output, temperature, relative_humidity, accelerometer, barometric_pressure, gyrometer, gps

#define MIN_SLEEP_INTERVAL		10		//Sleep interval in seconds between sensor measurements
#define MAX_SLEEP_INTERVAL		180
#define VALUETHRESHOLD			0.05	//0.01 = 1 %

uint16_t nextSleepInterval = MIN_SLEEP_INTERVAL;

static void rtc_sleep_callback();

#ifdef I2C

typedef struct {
	uint8_t addr;
	uint8_t type;
	uint8_t old_val;
} i2c_dev_t;

#define ADS1015_ADDRESS_GND		(0x48)    // 1001 000 (ADDR = GND)
#define ADS1015_ADDRESS_VCC		(0x49)	  // 1001 001 (ADDR = VCC)
#define ADS1015_ADDRESS_SDA		(0x50)	  // 1001 010 (ADDR = SDA)
#define ADS1015_ADDRESS_SCL		(0x51)	  // 1001 011 (ADDR = SCL)
//#define HDC1000_ADDRESS_00		(0x40) 	  //1000 000
//#define HDC1000_ADDRESS_01		(0x41)    //1000 001
//#define HDC1000_ADDRESS_10		(0x42)    //1000 010
//#define HDC1000_ADDRESS_11		(0x43)    //1000 011

#define CAYENNE_LPP_MOISTURE (105U)
#define CAYENNE_LPP_CARBON_MONOXIDE (106U)
#define CAYENNE_LPP_TOUCH (107U)
#define CAYENNE_LPP_MICROPHONE (108U)

i2c_dev_t I2C_DEVICES[] = {
	/* Moisture Sensors test: */
	{ .addr = ADS1015_ADDRESS_GND, .type = CAYENNE_LPP_MOISTURE, .old_val = 0 }, //Moisture sensor 1 	- sensor-1
	{ .addr = ADS1015_ADDRESS_VCC, .type = CAYENNE_LPP_MOISTURE, .old_val = 0 }  //Moisture sensor 2 	- sensor-2

	/* Other Sensors test: */
	//{ .addr = ADS1015_ADDRESS_GND, .type = CAYENNE_LPP_CARBON_MONOXIDE, .old_val = 0 }, 	//Carbon monoxide 		- sensor-3
	//{ .addr = ADS1015_ADDRESS_VCC, .type = CAYENNE_LPP_TOUCH, .old_val = 0 }, 			//Touch sensor     		- sensor-4
    //{ .addr = ADS1015_ADDRESS_SDA, .type = CAYENNE_LPP_MICROPHONE, .old_val = 0 }, 		//Sound sensor     		- sensor-5

	//{ .addr = ADS1015_ADDRESS_GND, .type = CAYENNE_LPP_TEMPERATURE, .old_val = 0 },
	//{ .addr = ADS1015_ADDRESS_VCC, .type = CAYENNE_LPP_RELATIVE_HUMIDITY, .old_val = 0 },
	//{ .addr = ADS1015_ADDRESS_SDA, .type = CAYENNE_LPP_LUMINOSITY, .old_val = 0 },
	//{ .addr = ADS1015_ADDRESS_VCC, .type = CAYENNE_LPP_GPS, .old_val = 0 }
};

static uint8_t currentI2C = 0;

int setupI2C();

#ifdef ADS1115
uint16_t readADS1115(uint8_t addr, uint8_t channel);
#endif
#endif

#ifdef ABP
/*Moisture Sensors test device: */
static u4_t DEVADDR = 0x03FF0004; // <-- Change this address for every node!
/*Other Sensors test device: */
//static u4_t DEVADDR = 0x03FF0002; // <-- Change this address for every node!

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
//static const u1_t APPKEY[16] = { 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03 };
static const u1_t APPKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
void os_getDevKey (u1_t* buf) { memcpy(buf, APPKEY, 16); }
#endif

//static uint8_t msg[30];
//static uint8_t msg[16];
//static size_t msglen;

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
//const unsigned TX_INTERVAL = 1;

const lmic_pinmap lmic_pins = {
    .nss = QM_PIN_ID_0,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = QM_PIN_ID_15,
    .dio = { QM_PIN_ID_14, LMIC_UNUSED_PIN, QM_PIN_ID_5 }
};

/* function returns 0 if value is inside the range of test_val +/- threshold */
int isInThreshold(int value, int test_val, float threshold){
	if((float)value >= (float)(1-threshold)*(float)test_val && value <= (float)(1+threshold)*(float)test_val ){
		return 0;
	}
	else{
		return 1;
	}
}

static void rtc_sleep_wakeup(uint16_t sleep_interval)
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
	/* after sleep interval schedule next sensor mediation */
	rtc_cfg.callback = rtc_sleep_callback;
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
				       QM_RTC_ALARM_SECOND(CLK_RTC_DIV_1) * sleep_interval);
	qm_power_soc_deep_sleep(QM_POWER_WAKE_FROM_RTC);
}
/*
 * Function to process initial 16 bit sensor value received from ADC into smaller readable format to transmit as Cayenne LPP
 */
void process_value(uint16_t value, uint8_t type, uint8_t channel){
	switch (type){
	//TODO: Define processing for types: CAYENNE_LPP_DIGITAL_INPUT, CAYENNE_LPP_DIGITAL_OUTPUT, CAYENNE_LPP_ANALOG_INPUT
	// CAYENNE_LPP_ANALOG_OUTPUT, CAYENNE_LPP_LUMINOSITY, CAYENNE_LPP_PRESENCE, CAYENNE_LPP_TEMPERATURE, CAYENNE_LPP_RELATIVE_HUMIDITY
	// CAYENNE_LPP_ACCELEROMETER, CAYENNE_LPP_BAROMETRIC_PRESSURE, CAYENNE_LPP_GYROMETER, CAYENNE_LPP_GPS
	case CAYENNE_LPP_MOISTURE:

		if(value > 65000){
			converted_val_uint8_t = 0;
		}
		else if(value > 13500){
			converted_val_uint8_t = 100;
		}
		else{
			converted_val_uint8_t = (uint8_t)(value/135);
		}
		cayenne_lpp_add_moisture(&lpp, channel, converted_val_uint8_t);
		break;

	case CAYENNE_LPP_CARBON_MONOXIDE:

		converted_val_uint8_t = (uint8_t)(value/1000);
		cayenne_lpp_add_carbon_monoxide(&lpp, channel, converted_val_uint8_t);
		break;

	case CAYENNE_LPP_TOUCH:

		converted_val_uint8_t = (uint8_t)(value/1000);
		cayenne_lpp_add_touch(&lpp, channel, converted_val_uint8_t);
		break;

	case CAYENNE_LPP_MICROPHONE:
		converted_val_uint8_t = (uint8_t)(value/1000);
		cayenne_lpp_add_microphone(&lpp, channel, converted_val_uint8_t);
		break;

	default: //TODO: print error message
		break;
	}
}

static void sensor_measurement_tx(osjob_t* j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        // Serial.println(F("OP_TXRXPEND, not sending"));
    	int i=0;
    	i++;
    }
    else {

    	uint16_t val;
    	//float converted_val = 0;
    	cayenne_lpp_reset(&lpp);

    	for(currentI2C = 0; currentI2C < sizeof(I2C_DEVICES); currentI2C++){

    		val = readADS1115(I2C_DEVICES[currentI2C].addr, 0);
    		process_value(val, I2C_DEVICES[currentI2C].type, currentI2C);
    		//cayenne_lpp_add_moisture(&lpp, 1, converted_val);
    	}
    	//more test sensor data:
    	process_value(22.6, CAYENNE_LPP_TEMPERATURE, 1);
    	cayenne_lpp_add_gps(&lpp, 5, -31.73304, -60.52979, 2);
    	//cayenne_lpp_reset(&lpp);
    	//cayenne_lpp_add_temperature(&lpp, 1, 22.5);
    	//cayenne_lpp_add_barometric_pressure(&lpp, 2, 1072.21);
    	//cayenne_lpp_add_relative_humidity(&lpp, 3, 425);
    	//cayenne_lpp_add_luminosity(&lpp, 4, 300);
    	//cayenne_lpp_add_gps(&lpp, 5, -31.73304, -60.52979, 2);
    	//cayenne_lpp_add_digital_input(&lpp, 1, 42);
    	//cayenne_lpp_add_digital_output(&lpp, 1, 123);
    	//cayenne_lpp_add_analog_input(&lpp, 1, 0.01);
    	//cayenne_lpp_add_analog_output(&lpp, 1, 0.05);
    	//cayenne_lpp_add_presence(&lpp, 1, 1);
    	//cayenne_lpp_add_accelerometer(&lpp, 3, 0.5, 0.42, 0.1);
    	//cayenne_lpp_add_gyrometer(&lpp, 4, 0.3, 0.4, 0.5);

    	/* If sensor value changed more than stored old_val +- VALUETRESHOLD
    	 * set next sleep interval to min to get more sensor updates*/
    	/*if(isInThreshold(val, I2C_DEVICES[currentI2C].old_val, VALUETHRESHOLD) != 0 ){
    		nextSleepInterval = MIN_SLEEP_INTERVAL;
    		I2C_DEVICES[currentI2C].old_val = val;
    	}
    	else{
    		//Increase sleep interval to next transmission to save energy
      		if(nextSleepInterval < MAX_SLEEP_INTERVAL){
      			nextSleepInterval += 5;
      		}
      		if(nextSleepInterval >= MAX_SLEEP_INTERVAL) {
      			nextSleepInterval = MAX_SLEEP_INTERVAL;
      		}
    	}
    	*/

    	LMIC_setTxData2(1, lpp.buffer, lpp.cursor, 0);
		//LMIC_setTxData2(1, msg, msglen, 0);

    }
}

void rtc_sleep_callback(){
	qm_power_soc_restore();
	printf("good morning!");
	// Schedule next transmission
	os_setTimedCallback(&sendjob, os_getTime(), sensor_measurement_tx);
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
		/* Set up the RTC to wake up the SoC from sleep and put board to sleep mode. */
		rtc_sleep_wakeup(nextSleepInterval);
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

    //by default all 72 channels are enabled, so disable all channels which the gateway isn´t listen to (8 to 72)
    for(int channel = 8; channel < 72; channel++){
    	LMIC_disableChannel(channel);
    }
	// Disable link check validation
	LMIC_setLinkCheckMode(0);

	// TTN uses SF9 for its RX2 window.
	LMIC.dn2Dr = DR_SF9;

	// Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
	//LMIC_setDrTxpow(DR_SF7, 14);
	LMIC_setDrTxpow(DR_SF7, 14);
#endif

	 //by default all 72 channels are enabled, so disable all channels which the gateway isn´t listen to (8 to 72)
	 //   for(int channel = 8; channel < 72; channel++){
	 //  	LMIC_disableChannel(channel);
	 //   }

    LMIC_setClockError( MAX_CLOCK_ERROR * 1 / 100);

    // Start job (sending automatically starts OTAA too)
    sensor_measurement_tx(&sendjob);

    os_runloop();

    return 0;
}
