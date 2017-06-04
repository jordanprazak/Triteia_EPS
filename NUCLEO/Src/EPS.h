//Variable and value definitions 
#ifndef EPS_h

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

struct EPS_Status {

	/* Address bytes */
	uint8_t address_byte_1;
	uint8_t address_byte_2;
	
	/* Command bytes */
	uint8_t command_byte_1;
	uint8_t command_byte_2;
	
	/* System time */
	uint32_t timestamp;
	
	/* Solar panel current values */
	int16_t solar1_i, solar2_i, solar3_i, solar4_i;
	
	/* BCR Output current value */
	int16_t bcr_i;
	
	/* Battery current */
	int16_t batt_i;
	
	/* Rail currents */
	int16_t rail_3_3_i, rail_5_i, rail_12_i, rail_28_i;
	
	/* Solar panel voltage values */
	uint16_t solar1_v, solar2_v, solar3_v, solar4_v;

	/* BCR Output voltage value */
	uint16_t bcr_v;
	
	/* Battery voltage */
	int16_t batt_v;
	
	/* Rail voltages */
	int16_t rail_3_3_v, rail_5_v, rail_12_v, rail_28_v;
	
	/* Battery temperature */
	uint16_t batt_temp;
	
	/* Load fault counters */
	uint8_t ld1, ld2, ld3, ld4, ld5, ld6, ld7, ld8, ld9, ld10, ld11, ld12, ld13;
	
	/* EPS Status byte */
	uint8_t eps_status;
	
	/* Payload status byte */
	uint16_t payload_status;
	
	/* Checksum byte */
	uint8_t checksum;
};
typedef struct EPS_Status EPS_Status_t;

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

#endif
