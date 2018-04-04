LMIC porting for Quark D2000

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
