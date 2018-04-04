/*
 * Gateway stress test
 *
 * Description: To test the stability of the LoRa gateway a test scenario
 * with several sensor nodes (AIGSG-PS3) is implemented. Every sensor node
 * transmits continuously messages to the gateway without pausing. When a
 * message transmission is completed a new transmission begins. To establish
 * a maximum message throughput to the gateway without causing collisions on
 * the medium, the first 8 nodes send on a separate channel each. To examine
 * the system behavior additional nodes use the full channel spectrum (0-7)
 * and switch between channels randomly.
 *
 * In total the following test cases are realized:
 * Scenario 1:  1 node on channel 0, 2500 messages with SF7
 * Scenario 2:  2 nodes on channel 0 and 1 respectively, 2500 messages each with SF7
 * Scenario 3:  3 nodes on channel 0, 1 and 2 respectively, 2500 messages each with SF7
 * Scenario 4:  4 nodes on channel 0, 1, 2 and 3 respectively, 2500 messages each with SF7
 * Scenario 5:  5 nodes on channel 0, 1, 2, 3 and 4 respectively, 2500 messages each with SF7
 * Scenario 6:  6 nodes on channel 0, 1, 2, 3, 4 and 5 respectively, 2500 messages each with SF7
 * Scenario 7:  7 nodes on channel 0, 1, 2, 3, 4, 5 and 6 respectively, 2500 messages each with SF7
 * Scenario 8:  8 nodes on channel 0, 1, 2, 3, 4, 5, 6 and 7 respectively, 2500 messages each with SF7
 * Scenario 9:  8 nodes on channel 0, 1, 2, 3, 4, 5, 6 and 7 respectively, 1 extra node using all channels 0-7, 2500 messages each with SF7
 * Scenario 10: 8 nodes on channel 0, 1, 2, 3, 4, 5, 6 and 7 respectively, 2 extra nodes using all channels 0-7,2500 messages each with SF7
 *
 * After that the same setup is repeated with SF10:
 * Scenario 11:  1 node on channel 0, 2500 messages with SF10
 * Scenario 12:  2 nodes on channel 0 and 1 respectively, 2500 messages each with SF10
 * Scenario 13:  3 nodes on channel 0, 1 and 2 respectively, 2500 messages each with SF10
 * Scenario 14:  4 nodes on channel 0, 1, 2 and 3 respectively, 2500 messages each with SF10
 * Scenario 15:  5 nodes on channel 0, 1, 2, 3 and 4 respectively, 2500 messages each with SF10
 * Scenario 16:  6 nodes on channel 0, 1, 2, 3, 4 and 5 respectively, 2500 messages each with SF10
 * Scenario 17:  7 nodes on channel 0, 1, 2, 3, 4, 5 and 6 respectively, 2500 messages each with SF10
 * Scenario 18:  8 nodes on channel 0, 1, 2, 3, 4, 5, 6 and 7 respectively, 2500 messages each with SF10
 * Scenario 19:  8 nodes on channel 0, 1, 2, 3, 4, 5, 6 and 7 respectively, 1 extra node using all channels 0-7, 2500 messages each with SF10
 * Scenario 20: 8 nodes on channel 0, 1, 2, 3, 4, 5, 6 and 7 respectively, 2 extra nodes using all channels 0-7,2500 messages each with SF10
 *
 * Author: Sebastian Scheibe
 * Date: 2018-03-28
 *
 */

#define ABP

#include <qm_pinmux.h>
#include <qm_i2c.h>
#include <qm_gpio.h>
#include <clk.h>

#include "lmic/lmic.h"
#include "lmic/hal/hal.h"

#include "pb_decode.h"
#include "eslight/protocol.h"


/* TEST SZENARIO VARIABLES BEGIN*/

#define MSG_COUNT_SENT 2500 	//defines the max number of messages every node transmits to gateway
#define COUNT_LORA_PLATFORMS 10	//total number of platforms transmitting messages to gateway, used to generate unique messages for each node
#define NODE_ID 9 				/*<--- change NODE_ID for every node 0-9 */
#define SPREADING_FACTOR DR_SF7
/* TEST SZENARIO VARIABLES END*/

#define INIT_MSG_VAL NODE_ID    //generate unique messages for each node

uint8_t currentGPSModuleSubID = 1;
uint16_t currentMsgVal = INIT_MSG_VAL;
uint16_t tx_count = 0;


#ifdef ABP

/* Generate unique device addresses for each node as defined in LDAP*/
static u4_t DEVADDR = 0x03FF0003+NODE_ID;

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

const lmic_pinmap lmic_pins = {
    .nss = QM_PIN_ID_0,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = QM_PIN_ID_15,
    .dio = { QM_PIN_ID_14, LMIC_UNUSED_PIN, QM_PIN_ID_5 }
};

static void sensor_measurement_tx(osjob_t* j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        // Serial.println(F("OP_TXRXPEND, not sending"));
    	int i=0;
    	i++;
    } else {

    	esl_ValueUpdate vm = esl_ValueUpdate_init_zero;
    	vm.id = currentGPSModuleSubID;
    	vm.has_int_val = true;
    	vm.int_val = currentMsgVal;
    	/* Generate unique messages for each node, to be able to identify which
    	 * node's messages got lost during the test. To do so every node
    	 * starts sending its NODE_ID (inital value of currentMsgVal) and then adds
    	 * COUNT_LORA_PLATFORMS for every new message.
    	 * For 10 nodes (COUNT_LORA_PLATFORMS = 10) the following messages are
    	 * generated:
    	 *    node 0's messages: 0, 10, 20, 30, ...
    	 *    node 1's messages: 1, 11, 21, 31, ...
    	 *    node 2's messages: 2, 12, 22, 32, ...
    	 *    node 3's messages: 3, 13, 23, 33, ...
    	 *    node 4's messages: 4, 14, 24, 34, ...
    	 *    node 5's messages: 5, 15, 25, 35, ...
    	 *    node 6's messages: 6, 16, 26, 36, ...
    	 *    node 7's messages: 7, 17, 27, 37, ...
    	 *    node 8's messages: 8, 18, 28, 38, ...
    	 *    node 9's messages: 9, 19, 29, 39, ...
    	 */
    	currentMsgVal+=COUNT_LORA_PLATFORMS;

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
		/*Count the number of sent messages*/
		tx_count++;
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

		// Schedule next transmission and stop when maximum number of messages to be sent is reached
		if(tx_count<MSG_COUNT_SENT){
			os_setTimedCallback(&sendjob, os_getTime(), sensor_measurement_tx);
		}
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

int main(void)
{
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // For ABP
#ifdef ABP
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

    //for first 8 nodes asign to every node a separate tx channel
    if(NODE_ID >= 0 && NODE_ID <= 7){
    	for(int channel = 0; channel < 72; channel++){
    	        	LMIC_disableChannel(channel);
    	       }

    	LMIC_enableChannel(NODE_ID);
    }
    else{
    	//for all other nodes enable channels 0 to 7
    	//by default all 72 channels are enabled, so disable all channels which the gateway isn´t listen to (8 to 72)
    	for(int channel = 8; channel < 72; channel++){
    	    LMIC_disableChannel(channel);
    	}
    }

	// Disable link check validation
	LMIC_setLinkCheckMode(0);

	// TTN uses SF9 for its RX2 window.
	LMIC.dn2Dr = DR_SF9;

	// Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
	LMIC_setDrTxpow(SPREADING_FACTOR, 14);
#endif

    LMIC_setClockError( MAX_CLOCK_ERROR * 1 / 100);
    // Start job (sending automatically starts OTAA too)
    sensor_measurement_tx(&sendjob);
    os_runloop();

    return 0;
}
