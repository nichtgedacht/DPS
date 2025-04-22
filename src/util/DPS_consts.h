/**
 * 
 *
 */

#ifndef __DPS_CONSTS_H_
#define __DPS_CONSTS_H_

			//general Constants
#define DPS__PROD_ID					0U
#define DPS__NPROD_ID					1U
#define DPS__STD_SLAVE_ADDRESS 		0x77U
#define DPS__LSB						0x01U
#define DPS__TEMP_STD_MR				2U
#define DPS__TEMP_STD_OSR				3U
#define DPS__PRS_STD_MR				2U
#define DPS__PRS_STD_OSR				3U
#define DPS__OSR_SE 					3U

//we use 0.1 mS units for time calculations, so 10 units are one millisecond
#define DPS__BUSYTIME_SCALING 			10U
// DPS310 has 10 milliseconds of spare time for each synchronous measurement / per second for asynchronous measurements
// this is for error prevention on friday-afternoon-products :D
// you can set it to 0 if you dare, but there is no warranty that it will still work
#define DPS__BUSYTIME_FAILSAFE			0U


#define DPS__MAX_BUSYTIME 				((1000U-DPS__BUSYTIME_FAILSAFE)*DPS__BUSYTIME_SCALING)
#define DPS__NUM_OF_SCAL_FACTS			8

#define DPS__SUCCEEDED 				0
#define DPS__FAIL_UNKNOWN 				-1
#define DPS__FAIL_INIT_FAILED 			-2
#define DPS__FAIL_TOOBUSY 				-3
#define DPS__FAIL_UNFINISHED 			-4

			//product id
#define DPS__REG_INFO_PROD_ID 			DPS__REG_ADR_PROD_ID, \
											DPS__REG_MASK_PROD_ID, \
											DPS__REG_SHIFT_PROD_ID
#define DPS__REG_ADR_PROD_ID 			0x0DU
#define DPS__REG_MASK_PROD_ID 			0x0FU
#define DPS__REG_SHIFT_PROD_ID 		0U

			//revision id
#define DPS__REG_INFO_REV_ID 			DPS__REG_ADR_REV_ID, \
											DPS__REG_MASK_REV_ID, \
											DPS__REG_SHIFT_REV_ID
#define DPS__REG_ADR_REV_ID 			0x0DU
#define DPS__REG_MASK_REV_ID 			0xF0U
#define DPS__REG_SHIFT_REV_ID 			4U

			//operating mode
#define DPS__REG_INFO_OPMODE 			DPS__REG_ADR_OPMODE, \
											DPS__REG_MASK_OPMODE, \
											DPS__REG_SHIFT_OPMODE
#define DPS__REG_ADR_OPMODE 			0x08U
#define DPS__REG_MASK_OPMODE 			0x07U
#define DPS__REG_SHIFT_OPMODE 			0U


			//temperature measure rate
#define DPS__REG_INFO_TEMP_MR 			DPS__REG_ADR_TEMP_MR, \
											DPS__REG_MASK_TEMP_MR, \
											DPS__REG_SHIFT_TEMP_MR
#define DPS__REG_ADR_TEMP_MR 			0x07U
#define DPS__REG_MASK_TEMP_MR 			0x70U
#define DPS__REG_SHIFT_TEMP_MR 		4U

			//temperature oversampling rate
#define DPS__REG_INFO_TEMP_OSR 		DPS__REG_ADR_TEMP_OSR, \
											DPS__REG_MASK_TEMP_OSR, \
											DPS__REG_SHIFT_TEMP_OSR
#define DPS__REG_ADR_TEMP_OSR 			0x07U
#define DPS__REG_MASK_TEMP_OSR 		0x07U
#define DPS__REG_SHIFT_TEMP_OSR 		0U

			//temperature sensor
#define DPS__REG_INFO_TEMP_SENSOR 		DPS__REG_ADR_TEMP_SENSOR, \
											DPS__REG_MASK_TEMP_SENSOR, \
											DPS__REG_SHIFT_TEMP_SENSOR
#define DPS__REG_ADR_TEMP_SENSOR 		0x07U
#define DPS__REG_MASK_TEMP_SENSOR 		0x80U
#define DPS__REG_SHIFT_TEMP_SENSOR 	7U

			//temperature sensor recommendation
#define DPS__REG_INFO_TEMP_SENSORREC 	DPS__REG_ADR_TEMP_SENSORREC, \
											DPS__REG_MASK_TEMP_SENSORREC, \
											DPS__REG_SHIFT_TEMP_SENSORREC
#define DPS__REG_ADR_TEMP_SENSORREC 	0x28U
#define DPS__REG_MASK_TEMP_SENSORREC 	0x80U
#define DPS__REG_SHIFT_TEMP_SENSORREC 	7U

			//temperature shift enable (if temp_osr>3)
#define DPS__REG_INFO_TEMP_SE 			DPS__REG_ADR_TEMP_SE, \
											DPS__REG_MASK_TEMP_SE, \
											DPS__REG_SHIFT_TEMP_SE
#define DPS__REG_ADR_TEMP_SE 			0x09U
#define DPS__REG_MASK_TEMP_SE 			0x08U
#define DPS__REG_SHIFT_TEMP_SE 		3U


			//pressure measure rate
#define DPS__REG_INFO_PRS_MR 			DPS__REG_ADR_PRS_MR, \
											DPS__REG_MASK_PRS_MR, \
											DPS__REG_SHIFT_PRS_MR
#define DPS__REG_ADR_PRS_MR 			0x06U
#define DPS__REG_MASK_PRS_MR 			0x70U
#define DPS__REG_SHIFT_PRS_MR 			4U

			//pressure oversampling rate
#define DPS__REG_INFO_PRS_OSR 			DPS__REG_ADR_PRS_OSR, \
											DPS__REG_MASK_PRS_OSR, \
											DPS__REG_SHIFT_PRS_OSR
#define DPS__REG_ADR_PRS_OSR 			0x06U
#define DPS__REG_MASK_PRS_OSR 			0x07U
#define DPS__REG_SHIFT_PRS_OSR 		0U

			//pressure shift enable (if prs_osr>3)
#define DPS__REG_INFO_PRS_SE 			DPS__REG_ADR_PRS_SE, \
											DPS__REG_MASK_PRS_SE, \
											DPS__REG_SHIFT_PRS_SE
#define DPS__REG_ADR_PRS_SE 			0x09U
#define DPS__REG_MASK_PRS_SE 			0x04U
#define DPS__REG_SHIFT_PRS_SE 			2U


			//temperature ready flag
#define DPS__REG_INFO_TEMP_RDY 		DPS__REG_ADR_TEMP_RDY, \
											DPS__REG_MASK_TEMP_RDY, \
											DPS__REG_SHIFT_TEMP_RDY
#define DPS__REG_ADR_TEMP_RDY 			0x08U
#define DPS__REG_MASK_TEMP_RDY			0x20U
#define DPS__REG_SHIFT_TEMP_RDY 		5U

			//pressure ready flag
#define DPS__REG_INFO_PRS_RDY 			DPS__REG_ADR_PRS_RDY, \
											DPS__REG_MASK_PRS_RDY, \
											DPS__REG_SHIFT_PRS_RDY
#define DPS__REG_ADR_PRS_RDY 			0x08U
#define DPS__REG_MASK_PRS_RDY 			0x10U
#define DPS__REG_SHIFT_PRS_RDY 		4U

			//pressure value
#define DPS__REG_ADR_PRS 				0x00U
#define DPS__REG_LEN_PRS 				3U

			//temperature value
#define DPS__REG_ADR_TEMP 				0x03U
#define DPS__REG_LEN_TEMP 				3U

			//compensation coefficients
#define DPS__REG_ADR_COEF 				0x10U
#define DPS__REG_LEN_COEF 				18


			//FIFO enable
#define DPS__REG_INFO_FIFO_EN 			DPS__REG_ADR_FIFO_EN, \
											DPS__REG_MASK_FIFO_EN, \
											DPS__REG_SHIFT_FIFO_EN
#define DPS__REG_ADR_FIFO_EN 			0x09U
#define DPS__REG_MASK_FIFO_EN 			0x02U
#define DPS__REG_SHIFT_FIFO_EN 		1U

			//FIFO flush
#define DPS__REG_INFO_FIFO_FL 			DPS__REG_ADR_FIFO_EN, \
											DPS__REG_MASK_FIFO_EN, \
											DPS__REG_SHIFT_FIFO_EN
#define DPS__REG_ADR_FIFO_FL 			0x0CU
#define DPS__REG_MASK_FIFO_FL 			0x80U
#define DPS__REG_SHIFT_FIFO_FL 		7U

			//FIFO empty
#define DPS__REG_INFO_FIFO_EMPTY 		DPS__REG_ADR_FIFO_EMPTY, \
											DPS__REG_MASK_FIFO_EMPTY, \
											DPS__REG_SHIFT_FIFO_EMPTY
#define DPS__REG_ADR_FIFO_EMPTY 		0x0BU
#define DPS__REG_MASK_FIFO_EMPTY 		0x01U
#define DPS__REG_SHIFT_FIFO_EMPTY 		0U

			//FIFO full
#define DPS__REG_INFO_FIFO_FULL 		DPS__REG_ADR_FIFO_FULL, \
											DPS__REG_MASK_FIFO_FULL, \
											DPS__REG_SHIFT_FIFO_FULL
#define DPS__REG_ADR_FIFO_FULL 		0x0BU
#define DPS__REG_MASK_FIFO_FULL 		0x02U
#define DPS__REG_SHIFT_FIFO_FULL 		1U


			//INT HL
#define DPS__REG_INFO_INT_HL 			DPS__REG_ADR_INT_HL, \
											DPS__REG_MASK_INT_HL, \
											DPS__REG_SHIFT_INT_HL
#define DPS__REG_ADR_INT_HL 			0x09U
#define DPS__REG_MASK_INT_HL 			0x80U
#define DPS__REG_SHIFT_INT_HL 			7U

			//INT FIFO enable
#define DPS__REG_INFO_INT_EN_FIFO 		DPS__REG_ADR_INT_EN_FIFO, \
											DPS__REG_MASK_INT_EN_FIFO, \
											DPS__REG_SHIFT_INT_EN_FIFO
#define DPS__REG_ADR_INT_EN_FIFO 		0x09U
#define DPS__REG_MASK_INT_EN_FIFO 		0x40U
#define DPS__REG_SHIFT_INT_EN_FIFO 	6U

			//INT TEMP enable
#define DPS__REG_INFO_INT_EN_TEMP 		DPS__REG_ADR_INT_EN_TEMP, \
											DPS__REG_MASK_INT_EN_TEMP, \
											DPS__REG_SHIFT_INT_EN_TEMP
#define DPS__REG_ADR_INT_EN_TEMP 		0x09U
#define DPS__REG_MASK_INT_EN_TEMP 		0x20U
#define DPS__REG_SHIFT_INT_EN_TEMP 	5U

			//INT PRS enable
#define DPS__REG_INFO_INT_EN_PRS 		DPS__REG_ADR_INT_EN_PRS, \
											DPS__REG_MASK_INT_EN_PRS, \
											DPS__REG_SHIFT_INT_EN_PRS
#define DPS__REG_ADR_INT_EN_PRS 		0x09U
#define DPS__REG_MASK_INT_EN_PRS 		0x10U
#define DPS__REG_SHIFT_INT_EN_PRS 		4U

			//INT FIFO flag
#define DPS__REG_INFO_INT_FLAG_FIFO 	DPS__REG_ADR_INT_FLAG_FIFO, \
											DPS__REG_MASK_INT_FLAG_FIFO, \
											DPS__REG_SHIFT_INT_FLAG_FIFO
#define DPS__REG_ADR_INT_FLAG_FIFO 	0x0AU
#define DPS__REG_MASK_INT_FLAG_FIFO 	0x04U
#define DPS__REG_SHIFT_INT_FLAG_FIFO 	2U

			//INT TMP flag
#define DPS__REG_INFO_INT_FLAG_TEMP 	DPS__REG_ADR_INT_FLAG_TEMP, \
											DPS__REG_MASK_INT_FLAG_TEMP, \
											DPS__REG_SHIFT_INT_FLAG_TEMP
#define DPS__REG_ADR_INT_FLAG_TEMP 	0x0AU
#define DPS__REG_MASK_INT_FLAG_TEMP 	0x02U
#define DPS__REG_SHIFT_INT_FLAG_TEMP 	1U

			//INT PRS flag
#define DPS__REG_INFO_INT_FLAG_PRS 	DPS__REG_ADR_INT_FLAG_PRS, \
											DPS__REG_MASK_INT_FLAG_PRS, \
											DPS__REG_SHIFT_INT_FLAG_PRS
#define DPS__REG_ADR_INT_FLAG_PRS 		0x0AU
#define DPS__REG_MASK_INT_FLAG_PRS 	0x01U
#define DPS__REG_SHIFT_INT_FLAG_PRS 	0U



#endif /* DPS_CONSTS_H_ */
