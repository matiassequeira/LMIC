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

#ifdef I2C

typedef struct {
	uint8_t addr;
	uint8_t subid;
} i2c_dev_t;

#define ADS1015_ADDRESS_GND         (0x48)    // 1001 000 (ADDR = GND)
#define ADS1015_ADDRESS_VCC 		 0x49

const i2c_dev_t I2C_DEVICES[] = {
    { .addr = ADS1015_ADDRESS_GND, .subid = 1 },
	{ .addr = ADS1015_ADDRESS_VCC, .subid = 2 }
};

static uint8_t currentI2C = 0;

int setupI2C();

#ifdef ADS1115
uint16_t readADS1115(uint8_t addr, uint8_t channel);
#endif
#endif

#ifdef ABP
static u4_t DEVADDR = 0x03FF0001; // <-- Change this address for every node!
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
const unsigned TX_INTERVAL = 10;

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
    	uint16_t val = readADS1115(I2C_DEVICES[currentI2C].addr, 0);
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

		// Schedule next transmission
		os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
	LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

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
