#ifndef EPS_PROTO_H
#define EPS_PROTO_H

#include <stdint.h>

#define TEMP_CALIBRATION 7.3
#define INA3221_OFFSET -0.130
#define CONVERSION_TIME_INA3221 8500
#define CONVERSION_TIME_INA260 1100
#define DEPLOYED_BATT_LOW 0
#define DEPLOYED_BATT_HIGH 0
#define DEPLOYED_SOLAR_V 0
#define DEPLOYED_SOLAR_I 0
#define TESTMODE_BATT 0

#define HEATER_OFF_TEMP 50
#define HEATER_ON_TEMP 0
#define BATT_UNDER_VOLTAGE 7
#define BATT_OVER_VOLTAGE 8.4

//GPIO PIN Definitions
#define PCM_IN_EN		70
#define BCR_OUT_EN		71
#define BCR0_EN			00
#define BCR1_EN			00
#define BCR2_EN			00
#define BCR3_EN 		00
#define PDM0_EN			72	//28 valve en
#define PDM1_EN			60	//28trcv en
#define PDM2_EN			67	//12v trcv en
#define PDM3_EN			81	//5V en
#define PDM4_EN			10	//12v rw en
#define PDM5_EN			49	//33 en
#define PDM6_EN 		00
#define PDM7_EN			00
#define PDM8_EN			00
#define PDM9_EN			00
#define PDM10_EN		00
#define PDM11_EN		00
#define HEATER_EN		00
#define TESTMODE_PIN	00
#define PDM0_FAULT		73	//28 valv fault
#define PDM1_FAULT		31	//28trcv fault
#define PDM2_FAULT		66	//12v trcv fault
#define PDM3_FAULT		80	//5V fault
#define PDM4_FAULT 		11	//12v rw fault
#define PDM5_FAULT		48	//33 fault
#define PDM6_FAULT		20	//5v imu fault
#define PDM7_FAULT		00
#define PDM8_FAULT		00
#define PDM9_FAULT		00
#define PDM10_FAULT		00
#define PDM11_FAULT		00

struct iv_int {
  unsigned int iv;
  int16_t i;
  int16_t v;
};

struct PDM_fault {
  unsigned int pin;
  unsigned int fault_count;
}

// Set interrupt service routine for each latching current limiter
int16_t isr_latching_current_limiter();

// Set interrupt service routine for the UART
int16_t isr_UART();

#define HIGH 1
#define LOW 0

#define TEMP_CALIBRATION 7.3
#define INA3221_OFFSET -0.130
#define CONVERSION_TIME_INA3221 8500
#define CONVERSION_TIME_INA260 1100
#define DEPLOYED_BATT_LOW 0
#define DEPLOYED_BATT_HIGH 0
#define DEPLOYED_SOLAR_V 0
#define DEPLOYED_SOLAR_I 0
#define TESTMODE_BATT 0

#define HEATER_OFF_TEMP 50
#define HEATER_ON_TEMP 0
#define BATT_UNDER_VOLTAGE 7
#define BATT_OVER_VOLTAGE 8.4

// GPIO PIN Definitions
#define PCM_IN_EN		00
#define BCR_OUT_EN		00
#define BCR1_EN			00  // Solar Panel 1
#define BCR2_EN			00  // Solar Panel 2
#define BCR3_EN			00  // Solar Panel 3
#define BCR4_EN 		00  // Solar Panel 4

#define PDM1_EN			00	// 28v transceiver enable
#define PDM2_EN			00	// 28v valve enable
#define PDM3_EN			00	// 12v transceiver enable
#define PDM4_EN			00	// 12v valve enable
#define PDM5_EN			00	// 5v RW enable
#define PDM6_EN 		00  // 5v CHREC enable
#define PDM7_EN			00  // 5v IMU enable
#define PDM8_EN			00  // 5v CAM enable
#define PDM9_EN			00  // currently broken
#define PDM10_EN		00  // 3.3v CHREC enable
#define PDM11_EN		00  // 3.3v valves enable
#define PDM12_EN		00	// 3.3v MAISS enable
#define PDM13_EN		00

#define HEATER_EN		00  // Battery heater enable
#define TESTMODE_PIN	00

#define PDM1_FAULT		00	// 28v transceiver fault
#define PDM2_FAULT		00	// 28v valve fault
#define PDM3_FAULT		00	// 12v transceiver fault
#define PDM4_FAULT		00	// 12v transceiver fault
#define PDM5_FAULT 		00	// 5v RW fault
#define PDM6_FAULT		00	// 5v CHREC fault
#define PDM7_FAULT		00	// 5v IMU fault
#define PDM8_FAULT		00  // 5v CAM fault
#define PDM9_FAULT		00  // Currently broken
#define PDM10_FAULT		00  // 3.3v CHREC fault
#define PDM11_FAULT		00  // 3.3v valves fault
#define PDM12_FAULT		00  // 3.3v MAISS fault
#define PDM13_FAULT		00

#define NUM_BCR 4
#define NUM_PDM 13

// GPIO pins
#define BCR_EN BCR1_EN, BCR2_EN, BCR3_EN, BCR4_EN
#define PDM_EN PDM1_EN, PDM2_EN, PDM3_EN, PDM4_EN, PDM5_EN, PDM6_EN, \
                PDM7_EN, PDM8_EN, PDM9_EN, PDM10_EN, PDM11_EN, PDM12_EN, \
                PDM13_EN
#define PDM_FAULT PDM1_FAULT, PDM2_FAULT, PDM3_FAULT, PDM4_FAULT, \
                  PDM5_FAULT, PDM6_FAULT, PDM7_FAULT, PDM8_FAULT, \
                  PDM9_FAULT, PDM10_FAULT, PDM11_FAULT, PDM12_FAULT, \
                  PDM13_FAULT

#endif /* EPS_PROTO_H */
