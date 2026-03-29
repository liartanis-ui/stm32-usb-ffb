#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */
/* USER CODE END INCLUDE */

/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */
#define FFB_ID_OFFSET 0x00
#define MAX_EFFECTS 40

// HID Descriptor definitions - Axes
#define HID_USAGE_X		0x30
#define HID_USAGE_Y		0x31
#define HID_USAGE_Z		0x32
#define HID_USAGE_RX	0x33
#define HID_USAGE_RY	0x34
#define HID_USAGE_RZ	0x35
#define HID_USAGE_SL0	0x36
#define HID_USAGE_SL1	0x37
#define HID_USAGE_WHL	0x38
#define HID_USAGE_POV	0x39

// HID Descriptor definitions - FFB Effects
#define HID_USAGE_CONST 0x26    //    Usage ET Constant Force
#define HID_USAGE_RAMP  0x27    //    Usage ET Ramp
#define HID_USAGE_SQUR  0x30    //    Usage ET Square
#define HID_USAGE_SINE  0x31    //    Usage ET Sine
#define HID_USAGE_TRNG  0x32    //    Usage ET Triangle
#define HID_USAGE_STUP  0x33    //    Usage ET Sawtooth Up
#define HID_USAGE_STDN  0x34    //    Usage ET Sawtooth Down
#define HID_USAGE_SPRNG 0x40    //    Usage ET Spring
#define HID_USAGE_DMPR  0x41    //    Usage ET Damper
#define HID_USAGE_INRT  0x42    //    Usage ET Inertia
#define HID_USAGE_FRIC  0x43    //    Usage ET Friction


// HID Descriptor definitions - FFB Report IDs
#define HID_ID_STATE	0x02	// Usage PID State report

#define HID_ID_EFFREP	0x01	// Usage Set Effect Report
#define HID_ID_ENVREP	0x02	// Usage Set Envelope Report
#define HID_ID_CONDREP	0x03	// Usage Set Condition Report
#define HID_ID_PRIDREP	0x04	// Usage Set Periodic Report
#define HID_ID_CONSTREP	0x05	// Usage Set Constant Force Report
#define HID_ID_RAMPREP	0x06	// Usage Set Ramp Force Report
#define HID_ID_CSTMREP	0x07	// Usage Custom Force Data Report
#define HID_ID_SMPLREP	0x08	// Usage Download Force Sample
#define HID_ID_EFOPREP	0x0A	// Usage Effect Operation Report
#define HID_ID_BLKFRREP	0x0B	// Usage PID Block Free Report
#define HID_ID_CTRLREP	0x0C	// Usage PID Device Control
#define HID_ID_GAINREP	0x0D	// Usage Device Gain Report
#define HID_ID_SETCREP	0x0E	// Usage Set Custom Force Report
// Features
#define HID_ID_NEWEFREP	0x11	// Usage Create New Effect Report
#define HID_ID_BLKLDREP	0x12	// Usage Block Load Report
#define HID_ID_POOLREP	0x13	// Usage PID Pool Report

// Control
#define HID_ID_CUSTOMCMD 0xAF   // Custom cmd (old. reserved)
#define HID_ID_HIDCMD	 0xA1   // HID cmd
#define HID_ID_STRCMD	 0xAC   // HID cmd as string. reserved
//#define HID_ID_CUSTOMCMD_IN 0xA2   // Custom cmd in. reserved
//#define HID_ID_CUSTOMCMD_OUT 0xA1   // Custom cmd out. reserved


#define FFB_EFFECT_NONE			0x00
#define FFB_EFFECT_CONSTANT		0x01
#define FFB_EFFECT_RAMP			0x02
#define FFB_EFFECT_SQUARE 		0x03
#define FFB_EFFECT_SINE 		0x04
#define FFB_EFFECT_TRIANGLE		0x05
#define FFB_EFFECT_SAWTOOTHUP	0x06
#define FFB_EFFECT_SAWTOOTHDOWN	0x07
#define FFB_EFFECT_SPRING		0x08
#define FFB_EFFECT_DAMPER		0x09
#define FFB_EFFECT_INERTIA		0x0A
#define FFB_EFFECT_FRICTION		0x0B
#define FFB_EFFECT_CUSTOM		0x0C

#define HID_ACTUATOR_POWER 		0x08
#define HID_SAFETY_SWITCH 		0x04
#define HID_ENABLE_ACTUATORS 	0x02
#define HID_EFFECT_PAUSE		0x01
#define HID_ENABLE_ACTUATORS_MASK 0xFD
#define HID_EFFECT_PLAYING 		0x10

#define HID_DIRECTION_ENABLE 0x04
#define FFB_EFFECT_DURATION_INFINITE 0xffff
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/* USER CODE BEGIN PRIVATE_VARIABLES */
static uint8_t pool_report_buf[6] = {
    0x13,        // Report ID = 0x13 (HID_ID_POOLREP)
    0x00, 0x20,  // RAM Pool Size = 8192 (0x2000)
    0x28,        // Simultaneous Effects Max = 40
    0x01,        // Device Managed Pool = 1
    0x00         // Shared Parameter Blocks = 0, padding
};

// Block Load 缓冲，Create New Effect 时填入分配的 Block Index
static uint8_t block_load_buf[4] = {
    0x12,  // Report ID
    0x01,  // Block Index（动态更新）
    0x01,  // Success
    0x28,  // RAM Pool Available
};
static uint8_t next_block = 1;
/* USER CODE END PRIVATE_VARIABLES */

// 报告描述符
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
		   0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
		    0x09, 0x04,                    // USAGE (Joystick)
		    0xa1, 0x01,                    // COLLECTION (Application)
		    0xa1, 0x00,                    //   COLLECTION (Physical)
		    0x85, 0x01,                    //     REPORT_ID (1)
		    0x05, 0x09,                    //     USAGE_PAGE (Button)
		    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
		    0x29, 0x40,                    //     USAGE_MAXIMUM (Button 64)
		    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
		    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
		    0x95, 0x40,                    //     REPORT_COUNT (64)
		    0x75, 0x01,                    //     REPORT_SIZE (1)
		    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
		    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
		    0x09, HID_USAGE_X,             //     USAGE (X)
		    0x09, HID_USAGE_Y,             //     USAGE (Y)
		    0x09, HID_USAGE_Z,             //     USAGE (Z)
		    0x09, HID_USAGE_RX,            //     USAGE (Rx)
		    0x09, HID_USAGE_RY,            //     USAGE (Ry)
			0x09, HID_USAGE_RZ,            //     USAGE (Rz)
			0x09, HID_USAGE_SL1,           //     USAGE (Dial)
			0x09, HID_USAGE_SL0,           //     USAGE (Slider)
		    0x16, 0x01, 0x80,              //     LOGICAL_MINIMUM (-32767)
		    0x26, 0xff, 0x7f,              //     LOGICAL_MAXIMUM (32767)
		    0x75, 0x10,                    //     REPORT_SIZE (16)
		    0x95, 0x08,                    //     REPORT_COUNT (8)
		    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
		    0xc0,                          //   END_COLLECTION


			// Control reports
			0x06, 0x00, 0xFF,                    // USAGE_PAGE (Vendor)
			0x09, 0x00,                    //   USAGE (Vendor)
			0xA1, 0x01, // Collection (Application)

				0x85,HID_ID_HIDCMD, //    Report ID
				0x09, 0x01,                    //   USAGE (Vendor)
				0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
				0x26, 0x04,	0x00,			   //   Logical Maximum 4
				0x75, 0x08,                    //   REPORT_SIZE (8)
				0x95, 0x01,                    //   REPORT_COUNT (1)
				0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)

				0x09, 0x02,                    //   USAGE (Vendor) class address
				0x75, 0x10,                    //   REPORT_SIZE (16)
				0x95, 0x01,                    //   REPORT_COUNT (1)
				0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)

				0x09, 0x03,                    //   USAGE (Vendor) class instance
				0x75, 0x08,                    //   REPORT_SIZE (8)
				0x95, 0x01,                    //   REPORT_COUNT (1)
				0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)

				0x09, 0x04,                    //   USAGE (Vendor) cmd
				0x75, 0x20,                    //   REPORT_SIZE (32)
				0x95, 0x01,                    //   REPORT_COUNT (1)
				0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)

				0x09, 0x05,                    //   USAGE (Vendor)
				0x75, 0x40,                    //   REPORT_SIZE (64) value
				0x95, 0x01,                    //   REPORT_COUNT (1)
				0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)

				0x09, 0x06,                    //   USAGE (Vendor) address
				0x75, 0x40,                    //   REPORT_SIZE (64)
				0x95, 0x01,                    //   REPORT_COUNT (1)
				0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)

				0x85,HID_ID_HIDCMD, //    Report ID
				0x09, 0x01,                    //   USAGE (Vendor)
				0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
				0x26, 0x04,	0x00,			   //   Logical Maximum 4
				0x75, 0x08,                    //   REPORT_SIZE (8)
				0x95, 0x01,                    //   REPORT_COUNT (1)
				0x81, 0x02,                    //   INPUT (Data,Var,Abs)

				0x09, 0x02,                    //   USAGE (Vendor) class address
				0x75, 0x10,                    //   REPORT_SIZE (16)
				0x95, 0x01,                    //   REPORT_COUNT (1)
				0x81, 0x02,                    //   INPUT (Data,Var,Abs)

				0x09, 0x03,                    //   USAGE (Vendor) class instance
				0x75, 0x08,                    //   REPORT_SIZE (8)
				0x95, 0x01,                    //   REPORT_COUNT (1)
				0x81, 0x02,                    //   INPUT (Data,Var,Abs)

				0x09, 0x04,                    //   USAGE (Vendor) cmd
				0x75, 0x20,                    //   REPORT_SIZE (32)
				0x95, 0x01,                    //   REPORT_COUNT (1)
				0x81, 0x02,                    //   INPUT (Data,Var,Abs)

				0x09, 0x05,                    //   USAGE (Vendor)
				0x75, 0x40,                    //   REPORT_SIZE (64) value
				0x95, 0x01,                    //   REPORT_COUNT (1)
				0x81, 0x02,                    //   INPUT (Data,Var,Abs)

				0x09, 0x06,                    //   USAGE (Vendor) address
				0x75, 0x40,                    //   REPORT_SIZE (64)
				0x95, 0x01,                    //   REPORT_COUNT (1)
				0x81, 0x02,                    //   INPUT (Data,Var,Abs)



		  0xc0,                          //   END_COLLECTION


		// BEGIN PID effects
			0x05,0x0F,        //    Usage Page Physical Interface
			0x09,0x92,        //    Usage PID State report
			0xA1,0x02,        //    Collection Datalink (logical)
			   0x85,HID_ID_STATE+FFB_ID_OFFSET,    //    Report ID 2

//			   0x09,0x22,    //    Usage Effect Block Index
//			   0x15,0x01,    //    Logical Minimum 1
//			   0x25,MAX_EFFECTS,    //    Logical Maximum 28h (40d)
//			   0x35,0x01,    //    Physical Minimum 1
//			   0x45,MAX_EFFECTS,    //    Physical Maximum 28h (40d)
//			   0x75,0x08,    //    Report Size 8
//			   0x95,0x01,    //    Report Count 1
//			   0x81,0x02,    //    Input (Variable)



			   0x09,0x9F,    //    Usage Device is Pause
			   0x09,0xA0,    //    Usage Actuators Enabled
			   0x09,0xA4,    //    Usage Safety Switch
			   0x09,0xA6,    //    Usage Actuator Power

			   0x09,0x94,    //    Usage Effect Playing
			   			   /*
			   			   			   0x15,0x00,    //    Logical Minimum 0
			   			   			   0x25,0x01,    //    Logical Maximum 1
			   			   			   0x35,0x00,    //    Physical Minimum 0
			   			   			   0x45,0x01,    //    Physical Maximum 1
			   			   			   0x75,0x01,    //    Report Size 1
			   			   			   0x95,0x01,    //    Report Count 1
			   			   			   0x81,0x02,    //    Input (Variable)*/ //14

			   0x15,0x00,    //    Logical Minimum 0
			   0x25,0x01,    //    Logical Maximum 1
			   0x35,0x00,    //    Physical Minimum 0
			   0x45,0x01,    //    Physical Maximum 1
			   0x75,0x01,    //    Report Size 1
			   0x95,0x05,    //    Report Count 4
			   0x81,0x02,    //    Input (Variable)
			   0x95,0x03,    //    Report Count 3
			   0x81,0x03,    //    Input (Constant, Variable)
			0xC0    ,    // End Collection

			/*
			Output
			Collection  Datalink:
			Usage Set Effect Report

			ID:1
			Effect Block Index:	8bit

			subcollection Effect Type
			12 effect types, 8bit each

			*/
			0x09,0x21,    //    Usage Set Effect Report
			0xA1,0x02,    //    Collection Datalink (Logical)
			   0x85,HID_ID_EFFREP+FFB_ID_OFFSET,    //    Report ID 1
			   0x09,0x22,    //    Usage Effect Block Index
			   0x15,0x01,    //    Logical Minimum 1
			   0x25,MAX_EFFECTS,    //    Logical Maximum 28h (40d)
			   0x35,0x01,    //    Physical Minimum 1
			   0x45,MAX_EFFECTS,    //    Physical Maximum 28h (40d)
			   0x75,0x08,    //    Report Size 8
			   0x95,0x01,    //    Report Count 1
			   0x91,0x02,    //    Output (Variable)
			   0x09,0x25,    //    Usage Effect Type
			   0xA1,0x02,    //    Collection Datalink
			 0x09, HID_USAGE_CONST,    //    Usage ET Constant Force
			 0x09, HID_USAGE_RAMP,    //    Usage ET Ramp
			 0x09, HID_USAGE_SQUR,    //    Usage ET Square
			 0x09, HID_USAGE_SINE,    //    Usage ET Sine
			 0x09, HID_USAGE_TRNG,    //    Usage ET Triangle
			 0x09, HID_USAGE_STUP,    //    Usage ET Sawtooth Up
			 0x09, HID_USAGE_STDN,    //    Usage ET Sawtooth Down
			 0x09, HID_USAGE_SPRNG,    //    Usage ET Spring
			 0x09, HID_USAGE_DMPR,    //    Usage ET Damper
			 0x09, HID_USAGE_INRT,    //    Usage ET Inertia
			 0x09, HID_USAGE_FRIC,    //    Usage ET Friction
			// 0x09, 0x28,    //    Usage ET Custom Force Data
			      0x25,0x0B,    //    Logical Maximum Bh (11d)
			      0x15,0x01,    //    Logical Minimum 1
			      0x35,0x01,    //    Physical Minimum 1
			      0x45,0x0B,    //    Physical Maximum Bh (11d)
			      0x75,0x08,    //    Report Size 8
			      0x95,0x01,    //    Report Count 1
			      0x91,0x00,    //    Output
			   0xC0    ,          //    End Collection
			   0x09,0x50,         //    Usage Duration
			   0x09,0x54,         //    Usage Trigger Repeat Interval
			   0x09,0x51,         //    Usage Sample Period
			   0x09,0xA7,         //    Usage Start Delay
			   0x15,0x00,         //    Logical Minimum 0
			   0x26,0xFF,0x7F,    //    Logical Maximum 7FFFh (32767d)
			   0x35,0x00,         //    Physical Minimum 0
			   0x46,0xFF,0x7F,    //    Physical Maximum 7FFFh (32767d)
			   0x66,0x03,0x10,    //    Unit 1003h (4099d)
			   0x55,0xFD,         //    Unit Exponent FDh (253d)
			   0x75,0x10,         //    Report Size 10h (16d)
			   0x95,0x04,         //    Report Count 4
			   0x91,0x02,         //    Output (Variable)
			   0x55,0x00,         //    Unit Exponent 0
			   0x66,0x00,0x00,    //    Unit 0
			   0x09,0x52,         //    Usage Gain
			   0x15,0x00,         //    Logical Minimum 0
			   0x26,0xFF,0x00,    //    Logical Maximum FFh (255d) // TODO scaling?
			   0x35,0x00,         //    Physical Minimum 0
			   0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
			   0x75,0x08,         //    Report Size 8
			   0x95,0x01,         //    Report Count 1
			   0x91,0x02,         //    Output (Variable)
			   0x09,0x53,         //    Usage Trigger Button
			   0x15,0x01,         //    Logical Minimum 1
			   0x25,0x08,         //    Logical Maximum 8
			   0x35,0x01,         //    Physical Minimum 1
			   0x45,0x08,         //    Physical Maximum 8
			   0x75,0x08,         //    Report Size 8
			   0x95,0x01,         //    Report Count 1
			   0x91,0x02,         //    Output (Variable)

			   0x09,0x55,       //    Usage Axes Enable TODO multi axis
			   0xA1,0x02,       //    Collection Datalink
			      0x05,0x01,    //    Usage Page Generic Desktop
			      0x09,0x30,    //    Usage X
			      //0x09,0x31,    //    Usage Y
			      0x15,0x00,    //    Logical Minimum 0
			      0x25,0x00,    //    Logical Maximum 0
			      0x75,0x01,    //    Report Size 1
			      0x95,0x01,    //    Report Count 1
			      0x91,0x02,    //    Output (Variable)
			   0xC0     ,    // End Collection
			   0x05,0x0F,    //    Usage Page Physical Interface
			   0x09,0x56,    //    Usage Direction Enable
			   0x95,0x01,    //    Report Count 1
			   0x91,0x02,    //    Output (Variable)
			   0x95,0x06,    //    Report Count 6
			   0x91,0x03,    //    Output (Constant, Variable)

			   0x09,0x57,    //    Usage Direction
			   0xA1,0x02,    //    Collection Datalink
			      0x0B,0x01,0x00,0x0A,0x00,    //    Usage Ordinals: Instance 1
//			      0x0B,0x02,0x00,0x0A,0x00,    //    Usage Ordinals: Instance 2
			      0x66,0x14,0x00,              //    Unit 14h (20d)
//			      0x55,0xFE,                   //    Unit Exponent FEh (254d)
//			      0x15,0x00,                   //    Logical Minimum 0
//			      0x26,0xFF,0x00,              //    Logical Maximum FFh (255d)
				  0x15,0x00,                   //    Logical Minimum 0
				  0x27,0xA0,0x8C,0x00,0x00,    //    Logical Maximum 8CA0h (36000d)
			      0x35,0x00,                   //    Physical Minimum 0
			      0x47,0xA0,0x8C,0x00,0x00,    //    Physical Maximum 8CA0h (36000d)
			      0x66,0x00,0x00,              //    Unit 0
			      0x75,0x10,                   //    Report Size 16
			      0x95,0x01,                   //    Report Count 1
			      0x91,0x02,                   //    Output (Variable)
			      0x55,0x00,                   //    Unit Exponent 0
			      0x66,0x00,0x00,              //    Unit 0
			   0xC0,                           //    End Collection

			   0x05, 0x0F,        //     USAGE_PAGE (Physical Interface)
			   0x09, 0x58,        //     USAGE (Type Specific Block Offset)
			   0xA1, 0x02,        //     COLLECTION (Logical)
			      0x0B, 0x01, 0x00, 0x0A, 0x00, //USAGE (Ordinals:Instance 1
			      //0x0B, 0x02, 0x00, 0x0A, 0x00, //USAGE (Ordinals:Instance 2)
			      0x26, 0xFD, 0x7F, //   LOGICAL_MAXIMUM (32765) ; 32K RAM or ROM max.
			      0x75, 0x10,     //     REPORT_SIZE (16)
			      0x95, 0x01,     //     REPORT_COUNT (1)
			      0x91, 0x02,     //     OUTPUT (Data,Var,Abs)
			   0xC0,              //     END_COLLECTION
			0xC0,                 //     END_COLLECTION

			// Envelope Report Definition
			0x09,0x5A,    //    Usage Set Envelope Report
			0xA1,0x02,    //    Collection Datalink
			   0x85,HID_ID_ENVREP+FFB_ID_OFFSET,         //    Report ID 2
			   0x09,0x22,         //    Usage Effect Block Index
			   0x15,0x01,         //    Logical Minimum 1
			   0x25,MAX_EFFECTS,         //    Logical Maximum 28h (40d)
			   0x35,0x01,         //    Physical Minimum 1
			   0x45,MAX_EFFECTS,         //    Physical Maximum 28h (40d)
			   0x75,0x08,         //    Report Size 8
			   0x95,0x01,         //    Report Count 1
			   0x91,0x02,         //    Output (Variable)
			   0x09,0x5B,         //    Usage Attack Level
			   0x09,0x5D,         //    Usage Fade Level
			 0x16,0x00,0x00,    //    Logical Minimum 0
			 0x26,0xFF,0x7F,    //    Logical Maximum 7FFFh (32767d)
			 0x36,0x00,0x00,    //    Physical Minimum 0
			 0x46,0xFF,0x7F,    //    Physical Maximum 7FFFh (32767d)
			 0x75,0x10,         //    Report Size 16
			 0x95,0x02,         //    Report Count 2
			   0x91,0x02,         //    Output (Variable)
			 0x09, 0x5C,         //    Usage Attack Time
			 0x09, 0x5E,         //    Usage Fade Time
			 0x66, 0x03, 0x10,    //    Unit 1003h (English Linear, Seconds)
			 0x55, 0xFD,         //    Unit Exponent FDh (X10^-3 ==> Milisecond)
			 0x27, 0xFF, 0x7F, 0x00, 0x00,    //    Logical Maximum FFFFFFFFh (4294967295)
			 0x47, 0xFF, 0x7F, 0x00, 0x00,    //    Physical Maximum FFFFFFFFh (4294967295)
			 0x75, 0x20,         //    Report Size 20h (32d)
			 0x91, 0x02,         //    Output (Variable)
			 0x45, 0x00,         //    Physical Maximum 0
			   0x66,0x00,0x00,    //    Unit 0
			   0x55,0x00,         //    Unit Exponent 0
			0xC0     ,            //    End Collection
			0x09,0x5F,    //    Usage Set Condition Report
			0xA1,0x02,    //    Collection Datalink
			   0x85,HID_ID_CONDREP+FFB_ID_OFFSET,    //    Report ID 3
			   0x09,0x22,    //    Usage Effect Block Index
			   0x15,0x01,    //    Logical Minimum 1
			   0x25,MAX_EFFECTS,    //    Logical Maximum 28h (40d)
			   0x35,0x01,    //    Physical Minimum 1
			   0x45,MAX_EFFECTS,    //    Physical Maximum 28h (40d)
			   0x75,0x08,    //    Report Size 8
			   0x95,0x01,    //    Report Count 1
			   0x91,0x02,    //    Output (Variable)
			   0x09,0x23,    //    Usage Parameter Block Offset
			   0x15,0x00,    //    Logical Minimum 0
			   0x25,0x03,    //    Logical Maximum 3
			   0x35,0x00,    //    Physical Minimum 0
			   0x45,0x03,    //    Physical Maximum 3
			   0x75,0x06,    //    Report Size 6
			   0x95,0x01,    //    Report Count 1
			   0x91,0x02,    //    Output (Variable)
			   0x09,0x58,    //    Usage Type Specific Block Off...
			   0xA1,0x02,    //    Collection Datalink
			      0x0B,0x01,0x00,0x0A,0x00,    //    Usage Ordinals: Instance 1
//			      0x0B,0x02,0x00,0x0A,0x00,    //    Usage Ordinals: Instance 2
			      0x75,0x02,                   //    Report Size 2
			      0x95,0x01,                   //    Report Count 1
			      0x91,0x02,                   //    Output (Variable)
			   0xC0     ,         //    End Collection
			   0x16,0x00, 0x80,    //    Logical Minimum 7FFFh (-32767d)
			   0x26,0xff, 0x7f,    //    Logical Maximum 7FFFh (32767d)
			   0x36,0x00, 0x80,    //    Physical Minimum 7FFFh (-32767d)
			   0x46,0xff, 0x7f,    //    Physical Maximum 7FFFh (32767d)

			   0x09,0x60,         //    Usage CP Offset
			   0x75,0x10,         //    Report Size 16
			   0x95,0x01,         //    Report Count 1
			   0x91,0x02,         //    Output (Variable)
			   0x36,0x00, 0x80,    //    Physical Minimum  (-32768)
			   0x46,0xff, 0x7f,    //    Physical Maximum  (32767)
			   0x09,0x61,         //    Usage Positive Coefficient
			   0x09,0x62,         //    Usage Negative Coefficient
			   0x95,0x02,         //    Report Count 2
			   0x91,0x02,         //    Output (Variable)
			 0x16,0x00,0x00,    //    Logical Minimum 0
			 0x26,0xff, 0x7f,	  //    Logical Maximum  (32767)
			 0x36,0x00,0x00,    //    Physical Minimum 0
			   0x46,0xff, 0x7f,    //    Physical Maximum  (32767)
			   0x09,0x63,         //    Usage Positive Saturation
			   0x09,0x64,         //    Usage Negative Saturation
			   0x75,0x10,         //    Report Size 16
			   0x95,0x02,         //    Report Count 2
			   0x91,0x02,         //    Output (Variable)
			   0x09,0x65,         //    Usage Dead Band
			   0x46,0xff, 0x7f,    //    Physical Maximum (32767)
			   0x95,0x01,         //    Report Count 1
			   0x91,0x02,         //    Output (Variable)
			0xC0     ,    //    End Collection
			0x09,0x6E,    //    Usage Set Periodic Report
			0xA1,0x02,    //    Collection Datalink
			   0x85,HID_ID_PRIDREP+FFB_ID_OFFSET,                   //    Report ID 4
			   0x09,0x22,                   //    Usage Effect Block Index
			   0x15,0x01,                   //    Logical Minimum 1
			   0x25,MAX_EFFECTS,                   //    Logical Maximum 28h (40d)
			   0x35,0x01,                   //    Physical Minimum 1
			   0x45,MAX_EFFECTS,                   //    Physical Maximum 28h (40d)
			   0x75,0x08,                   //    Report Size 8
			   0x95,0x01,                   //    Report Count 1
			   0x91,0x02,                   //    Output (Variable)
			   0x09,0x70,                   //    Usage Magnitude
			 0x16,0x00,0x00,              //    Logical Minimum 0
			 0x26,0xff, 0x7f,    //    Logical Maximum 7FFFh (32767d)
			 0x36,0x00,0x00,              //    Physical Minimum 0
			 0x26,0xff, 0x7f,    //    Logical Maximum 7FFFh (32767d)
			   0x75,0x10,                   //    Report Size 16
			   0x95,0x01,                   //    Report Count 1
			   0x91,0x02,                   //    Output (Variable)
			 0x09, 0x6F,                   //    Usage Offset
			 0x16,0x00, 0x80,    //    Logical Minimum 7FFFh (-32767d)
		     0x26,0xff, 0x7f,    //    Logical Maximum 7FFFh (32767d)
		     0x36,0x00, 0x80,    //    Physical Minimum 7FFFh (-32767d)
		     0x46,0xff, 0x7f,    //    Physical Maximum 7FFFh (32767d)
			 0x95, 0x01,                   //    Report Count 1
			 0x75, 0x10,                   //    Report Size 16
			 0x91, 0x02,                   //    Output (Variable)
			 0x09, 0x71,                   //    Usage Phase
			 0x66, 0x14, 0x00,             //    Unit 14h (Eng Rotation, Degrees)
			 0x55, 0xFE,                   //    Unit Exponent FEh (X10^-2)
			 0x15, 0x00,                   //    Logical Minimum 0
			 0x27, 0x9F, 0x8C, 0x00, 0x00, //    Logical Maximum 8C9Fh (35999d)
			 0x35, 0x00,                   //    Physical Minimum 0
			 0x47, 0x9F, 0x8C, 0x00, 0x00, //    Physical Maximum 8C9Fh (35999d)
			 0x75, 0x10,                   //    Report Size 16
			 0x95, 0x01,                   //    Report Count 1
			 0x91, 0x02,                   //    Output (Variable)
			 0x09, 0x72,                   //    Usage Period
			 0x15, 0x01,                   //    Logical Minimum 1
			 0x27, 0xFF, 0x7F, 0x00, 0x00, //    Logical Maximum 7FFFh (32K)
			 0x35, 0x01,                   //    Physical Minimum 1
			 0x47, 0xFF, 0x7F, 0x00, 0x00, //    Physical Maximum 7FFFh (32K)
			 0x66, 0x03, 0x10,             //    Unit 1003h (English Linear, Seconds)
			 0x55, 0xFD,                   //    Unit Exponent FDh (X10^-3 ==> Milisecond)
			 0x75, 0x20,                   //    Report Size 20h (32)
			 0x95, 0x01,                   //    Report Count 1
			 0x91, 0x02,                   //    Output (Variable)
			 0x66, 0x00, 0x00,              //    Unit 0
			   0x55,0x00,                   //    Unit Exponent 0
			0xC0     ,    // End Collection
			0x09,0x73,    //    Usage Set Constant Force Report
			0xA1,0x02,    //    Collection Datalink
			   0x85,HID_ID_CONSTREP+FFB_ID_OFFSET,         //    Report ID 5
			   0x09,0x22,         //    Usage Effect Block Index
			   0x15,0x01,         //    Logical Minimum 1
			   0x25,MAX_EFFECTS,         //    Logical Maximum 28h (40d)
			   0x35,0x01,         //    Physical Minimum 1
			   0x45,MAX_EFFECTS,         //    Physical Maximum 28h (40d)
			   0x75,0x08,         //    Report Size 8
			   0x95,0x01,         //    Report Count 1
			   0x91,0x02,         //    Output (Variable)
			   0x09,0x70,         //    Usage Magnitude
			   0x16,0x00, 0x80,    //    Logical Minimum 7FFFh (-32767d)
			   0x26,0xff, 0x7f,    //    Logical Maximum 7FFFh (32767d)
			   0x36,0x00, 0x80,    //    Physical Minimum 7FFFh (-32767d)
			   0x46,0xff, 0x7f,    //    Physical Maximum 7FFFh (32767d)
			 0x75, 0x10,         //    Report Size 10h (16d)
			   0x95,0x01,         //    Report Count 1
			   0x91,0x02,         //    Output (Variable)
			0xC0     ,    //    End Collection
			0x09,0x74,    //    Usage Set Ramp Force Report
			0xA1,0x02,    //    Collection Datalink
			   0x85,HID_ID_RAMPREP+FFB_ID_OFFSET,         //    Report ID 6
			   0x09,0x22,         //    Usage Effect Block Index
			   0x15,0x01,         //    Logical Minimum 1
			   0x25,MAX_EFFECTS,         //    Logical Maximum 28h (40d)
			   0x35,0x01,         //    Physical Minimum 1
			   0x45,MAX_EFFECTS,         //    Physical Maximum 28h (40d)
			   0x75,0x08,         //    Report Size 8
			   0x95,0x01,         //    Report Count 1
			   0x91,0x02,         //    Output (Variable)
			   0x09,0x75,         //    Usage Ramp Start
			   0x09,0x76,         //    Usage Ramp End
			   0x16,0x00, 0x80,    //    Logical Minimum 7FFFh (-32767d)
			   0x26,0xff, 0x7f,    //    Logical Maximum 7FFFh (32767d)
			   0x36,0x00, 0x80,    //    Physical Minimum 7FFFh (-32767d)
			   0x46,0xff, 0x7f,    //    Physical Maximum 7FFFh (32767d)
			   0x75,0x10,         //    Report Size 16
			   0x95,0x02,         //    Report Count 2
			   0x91,0x02,         //    Output (Variable)
			0xC0     ,    //    End Collection


//			0x09,0x68,    //    Usage Custom Force Data Report
//			0xA1,0x02,    //    Collection Datalink
//			   0x85,HID_ID_CSTMREP+FFB_ID_OFFSET,         //    Report ID 7
//			   0x09,0x22,         //    Usage Effect Block Index
//			   0x15,0x01,         //    Logical Minimum 1
//			   0x25,MAX_EFFECTS,         //    Logical Maximum 28h (40d)
//			   0x35,0x01,         //    Physical Minimum 1
//			   0x45,MAX_EFFECTS,         //    Physical Maximum 28h (40d)
//			   0x75,0x08,         //    Report Size 8
//			   0x95,0x01,         //    Report Count 1
//			   0x91,0x02,         //    Output (Variable)
//			   0x09,0x6C,         //    Usage Custom Force Data Offset
//			   0x15,0x00,         //    Logical Minimum 0
//			   0x26,0x10,0x27,    //    Logical Maximum 2710h (10000d)
//			   0x35,0x00,         //    Physical Minimum 0
//			   0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
//			   0x75,0x10,         //    Report Size 10h (16d)
//			   0x95,0x01,         //    Report Count 1
//			   0x91,0x02,         //    Output (Variable)
//			   0x09,0x69,         //    Usage Custom Force Data
//			   0x15,0x81,         //    Logical Minimum 81h (-127d)
//			   0x25,0x7F,         //    Logical Maximum 7Fh (127d)
//			   0x35,0x00,         //    Physical Minimum 0
//			   0x46,0xFF,0x00,    //    Physical Maximum FFh (255d)
//			   0x75,0x08,         //    Report Size 8
//			   0x95,0x0C,         //    Report Count Ch (12d)
//			   0x92,0x02,0x01,    //       Output (Variable, Buffered)
//			0xC0     ,    //    End Collection
//			0x09,0x66,    //    Usage Download Force Sample
//			0xA1,0x02,    //    Collection Datalink
//			   0x85,HID_ID_SMPLREP+FFB_ID_OFFSET,         //    Report ID 8
//			   0x05,0x01,         //    Usage Page Generic Desktop
//			   0x09,0x30,         //    Usage X
//			   0x09,0x31,         //    Usage Y
//			   0x15,0x81,         //    Logical Minimum 81h (-127d)
//			   0x25,0x7F,         //    Logical Maximum 7Fh (127d)
//			   0x35,0x00,         //    Physical Minimum 0
//			   0x46,0xFF,0x00,    //    Physical Maximum FFh (255d)
//			   0x75,0x08,         //    Report Size 8
//			   0x95,0x02,         //    Report Count 2
//			   0x91,0x02,         //    Output (Variable)
//			0xC0     ,   //    End Collection

			0x05,0x0F,   //    Usage Page Physical Interface
			0x09,0x77,   //    Usage Effect Operation Report
			0xA1,0x02,   //    Collection Datalink
			   0x85,HID_ID_EFOPREP+FFB_ID_OFFSET,    //    Report ID Ah (10d)
			   0x09,0x22,    //    Usage Effect Block Index
			   0x15,0x01,    //    Logical Minimum 1
			   0x25,MAX_EFFECTS,    //    Logical Maximum 28h (40d)
			   0x35,0x01,    //    Physical Minimum 1
			   0x45,MAX_EFFECTS,    //    Physical Maximum 28h (40d)
			   0x75,0x08,    //    Report Size 8
			   0x95,0x01,    //    Report Count 1
			   0x91,0x02,    //    Output (Variable)
			   0x09,0x78,    //    Usage Effect Operation
			   0xA1,0x02,    //    Collection Datalink
			      0x09,0x79,    //    Usage Op Effect Start
			      0x09,0x7A,    //    Usage Op Effect Start Solo
			      0x09,0x7B,    //    Usage Op Effect Stop
			      0x15,0x01,    //    Logical Minimum 1
			      0x25,0x03,    //    Logical Maximum 3
			      0x75,0x08,    //    Report Size 8
			      0x95,0x01,    //    Report Count 1
			      0x91,0x00,    //    Output
			   0xC0     ,         //    End Collection
			   0x09,0x7C,         //    Usage Loop Count
			   0x15,0x00,         //    Logical Minimum 0
			   0x26,0xFF,0x00,    //    Logical Maximum FFh (255d)
			   0x35,0x00,         //    Physical Minimum 0
			   0x46,0xFF,0x00,    //    Physical Maximum FFh (255d)
			   0x91,0x02,         //    Output (Variable)
			0xC0     ,    //    End Collection
			0x09,0x90,    //    Usage PID Block Free Report
			0xA1,0x02,    //    Collection Datalink
			   0x85,HID_ID_BLKFRREP+FFB_ID_OFFSET,    //    Report ID Bh (11d)
			   0x09,0x22,    //    Usage Effect Block Index
			   0x15,0x01,    //    Logical Minimum 1
			   0x25,MAX_EFFECTS,    //    Logical Maximum 28h (40d)
			   0x35,0x01,    //    Physical Minimum 1
			   0x45,MAX_EFFECTS,    //    Physical Maximum 28h (40d)
			   0x75,0x08,    //    Report Size 8
			   0x95,0x01,    //    Report Count 1
			   0x91,0x02,    //    Output (Variable)
			0xC0     ,    //    End Collection

			0x09,0x95,    //    Usage PID Device Control (0x96?)
			0xA1,0x02,    //    Collection Datalink
			0x85,HID_ID_CTRLREP+FFB_ID_OFFSET,    //    Report ID Ch (12d)
			0x09,0x96,    //    Usage PID Device Control (0x96?)
			0xA1,0x02,    //    Collection Datalink

			   0x09,0x97,    //    Usage DC Enable Actuators
			   0x09,0x98,    //    Usage DC Disable Actuators
			   0x09,0x99,    //    Usage DC Stop All Effects
			   0x09,0x9A,    //    Usage DC Device Reset
			   0x09,0x9B,    //    Usage DC Device Pause
			   0x09,0x9C,    //    Usage DC Device Continue



			   0x15,0x01,    //    Logical Minimum 1
			   0x25,0x06,    //    Logical Maximum 6
			   0x75,0x01,    //    Report Size 1
			   0x95,0x08,    //    Report Count 8
			   0x91,0x02,    //    Output

			0xC0     ,    //    End Collection
			0xC0     ,    //    End Collection
			0x09,0x7D,    //    Usage Device Gain Report
			0xA1,0x02,    //    Collection Datalink
			   0x85,HID_ID_GAINREP+FFB_ID_OFFSET,         //    Report ID Dh (13d)
			   0x09,0x7E,         //    Usage Device Gain
			   0x15,0x00,         //    Logical Minimum 0
			   0x26,0xFF,0x00,    //    Logical Maximum FFh (255d)
			   0x35,0x00,         //    Physical Minimum 0
			   0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
			   0x75,0x08,         //    Report Size 8
			   0x95,0x01,         //    Report Count 1
			   0x91,0x02,         //    Output (Variable)
			0xC0     ,            //    End Collection
//			0x09,0x6B,    //    Usage Set Custom Force Report
//			0xA1,0x02,    //    Collection Datalink
//			   0x85,HID_ID_SETCREP+FFB_ID_OFFSET,         //    Report ID Eh (14d)
//			   0x09,0x22,         //    Usage Effect Block Index
//			   0x15,0x01,         //    Logical Minimum 1
//			   0x25,MAX_EFFECTS,         //    Logical Maximum 28h (40d)
//			   0x35,0x01,         //    Physical Minimum 1
//			   0x45,MAX_EFFECTS,         //    Physical Maximum 28h (40d)
//			   0x75,0x08,         //    Report Size 8
//			   0x95,0x01,         //    Report Count 1
//			   0x91,0x02,         //    Output (Variable)
//			   0x09,0x6D,         //    Usage Sample Count
//			   0x15,0x00,         //    Logical Minimum 0
//			   0x26,0xFF,0x00,    //    Logical Maximum FFh (255d)
//			   0x35,0x00,         //    Physical Minimum 0
//			   0x46,0xFF,0x00,    //    Physical Maximum FFh (255d)
//			   0x75,0x08,         //    Report Size 8
//			   0x95,0x01,         //    Report Count 1
//			   0x91,0x02,         //    Output (Variable)
//			   0x09,0x51,         //    Usage Sample Period
//			   0x66,0x03,0x10,    //    Unit 1003h (4099d)
//			   0x55,0xFD,         //    Unit Exponent FDh (253d)
//			   0x15,0x00,         //    Logical Minimum 0
//			   0x26,0xFF,0x7F,    //    Logical Maximum 7FFFh (32767d)
//			   0x35,0x00,         //    Physical Minimum 0
//			   0x46,0xFF,0x7F,    //    Physical Maximum 7FFFh (32767d)
//			   0x75,0x10,         //    Report Size 10h (16d)
//			   0x95,0x01,         //    Report Count 1
//			   0x91,0x02,         //    Output (Variable)
//			   0x55,0x00,         //    Unit Exponent 0
//			   0x66,0x00,0x00,    //    Unit 0
//			0xC0     ,    //    End Collection
			0x09,0xAB,    //    Usage Create New Effect Report
			0xA1,0x02,    //    Collection Datalink
			   0x85,HID_ID_NEWEFREP+FFB_ID_OFFSET,    //    Report ID 1
			   0x09,0x25,    //    Usage Effect Type
			   0xA1,0x02,    //    Collection Datalink
			 0x09, HID_USAGE_CONST,    //    Usage ET Constant Force
			 0x09, HID_USAGE_RAMP,    //    Usage ET Ramp
			 0x09, HID_USAGE_SQUR,    //    Usage ET Square
			 0x09, HID_USAGE_SINE,    //    Usage ET Sine
			 0x09, HID_USAGE_TRNG,    //    Usage ET Triangle
			 0x09, HID_USAGE_STUP,    //    Usage ET Sawtooth Up
			 0x09, HID_USAGE_STDN,    //    Usage ET Sawtooth Down
			 0x09, HID_USAGE_SPRNG,    //    Usage ET Spring
			 0x09, HID_USAGE_DMPR,    //    Usage ET Damper
			 0x09, HID_USAGE_INRT,    //    Usage ET Inertia
			 0x09, HID_USAGE_FRIC,    //    Usage ET Friction
//			 0x09, 0x28,    //    Usage ET Custom Force Data
			   0x25,0x0B,    //    Logical Maximum Ch (11d)
			   0x15,0x01,    //    Logical Minimum 1
			   0x35,0x01,    //    Physical Minimum 1
			   0x45,0x0B,    //    Physical Maximum Ch (11d)
			   0x75,0x08,    //    Report Size 8
			   0x95,0x01,    //    Report Count 1
			   0xB1,0x00,    //    Feature
			0xC0     ,    // End Collection
			0x05,0x01,         //    Usage Page Generic Desktop
			0x09,0x3B,         //    Usage Reserved (Byte count)
			0x15,0x00,         //    Logical Minimum 0
			0x26,0xFF,0x01,    //    Logical Maximum 1FFh (511d)
			0x35,0x00,         //    Physical Minimum 0
			0x46,0xFF,0x01,    //    Physical Maximum 1FFh (511d)
			0x75,0x0A,         //    Report Size Ah (10d)
			0x95,0x01,         //    Report Count 1
			0xB1,0x02,         //    Feature (Variable)
			0x75,0x06,         //    Report Size 6
			0xB1,0x01,         //    Feature (Constant)
			0xC0     ,    //    End Collection
			0x05,0x0F,    //    Usage Page Physical Interface
			0x09,0x89,    //    Usage Block Load Report
			0xA1,0x02,    //    Collection Datalink
			0x85,HID_ID_BLKLDREP+FFB_ID_OFFSET,    //    Report ID 0x12
			0x09,0x22,    //    Usage Effect Block Index
			0x25,MAX_EFFECTS,    //    Logical Maximum 28h (40d)
			0x15,0x01,    //    Logical Minimum 1
			0x35,0x01,    //    Physical Minimum 1
			0x45,MAX_EFFECTS,    //    Physical Maximum 28h (40d)
			0x75,0x08,    //    Report Size 8
			0x95,0x01,    //    Report Count 1
			0xB1,0x02,    //    Feature (Variable)
			0x09,0x8B,    //    Usage Block Load Status
			0xA1,0x02,    //    Collection Datalink
			   0x09,0x8C,    //    Usage Block Load Success
			   0x09,0x8D,    //    Usage Block Load Full
			   0x09,0x8E,    //    Usage Block Load Error
			   0x15,0x01,    //    Logical Minimum 1
			   0x25,0x03,    //    Logical Maximum 3
			   0x35,0x01,    //    Physical Minimum 1
			   0x45,0x03,    //    Physical Maximum 3
			   0x75,0x08,    //    Report Size 8
			   0x95,0x01,    //    Report Count 1
			   0xB1,0x00,    //    Feature
			0xC0     ,                   // End Collection
			0x09,0xAC,                   //    Usage Pool available
			0x15,0x00,                   //    Logical Minimum 0
			0x27,0xFF,0xFF,0x00,0x00,    //    Logical Maximum FFFFh (65535d)
			0x35,0x00,                   //    Physical Minimum 0
			0x47,0xFF,0xFF,0x00,0x00,    //    Physical Maximum FFFFh (65535d)
			0x75,0x10,                   //    Report Size 10h (16d)
			0x95,0x01,                   //    Report Count 1
			0xB1,0x00,                   //    Feature
			0xC0     ,    //    End Collection

			0x09,0x7F,    //    Usage PID Pool Report
			0xA1,0x02,    //    Collection Datalink
				0x85,HID_ID_POOLREP+FFB_ID_OFFSET,                   //    Report ID 0x13
				0x09,0x80,                   //    Usage RAM Pool size
				0x75,0x10,                   //    Report Size 10h (16d)
				0x95,0x01,                   //    Report Count 1
				0x15,0x00,                   //    Logical Minimum 0
				0x35,0x00,                   //    Physical Minimum 0
				0x27,0xFF,0xFF,0x00,0x00,    //    Logical Maximum FFFFh (65535d)
				0x47,0xFF,0xFF,0x00,0x00,    //    Physical Maximum FFFFh (65535d)
				0xB1,0x02,                   //    Feature (Variable)
				0x09,0x83,                   //    Usage Simultaneous Effects Max
				0x26,0xFF,0x00,              //    Logical Maximum FFh (255d)
				0x46,0xFF,0x00,              //    Physical Maximum FFh (255d)
				0x75,0x08,                   //    Report Size 8
				0x95,0x01,                   //    Report Count 1
				0xB1,0x02,                   //    Feature (Variable)
				0x09,0xA9,                   //    Usage Device Managed Pool
				0x09,0xAA,                   //    Usage Shared Parameter Blocks
				0x75,0x01,                   //    Report Size 1
				0x95,0x02,                   //    Report Count 2
				0x15,0x00,                   //    Logical Minimum 0
				0x25,0x01,                   //    Logical Maximum 1
				0x35,0x00,                   //    Physical Minimum 0
				0x45,0x01,                   //    Physical Maximum 1
				0xB1,0x02,                   //    Feature (Variable)
				0x75,0x06,                   //    Report Size 6
				0x95,0x01,                   //    Report Count 1
				0xB1,0x03,                   //    Feature (Constant, Variable)
			0xC0, //    End Collection

  0xC0    /*     END_COLLECTION	             */
};

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

// 函数前向声明
static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state);
static uint8_t* CUSTOM_HID_GetReport_FS(uint8_t report_id, uint16_t *ReportLength); // ← 前向声明

// fops 结构体
USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS =
{
    CUSTOM_HID_ReportDesc_FS,
    CUSTOM_HID_Init_FS,
    CUSTOM_HID_DeInit_FS,
    CUSTOM_HID_OutEvent_FS,
    CUSTOM_HID_GetReport_FS,   // ← 注意有逗号
};

static int8_t CUSTOM_HID_Init_FS(void)
{
    return (USBD_OK);
}

static int8_t CUSTOM_HID_DeInit_FS(void)
{
    return (USBD_OK);
}

static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
    UNUSED(event_idx);  // 不用这两个参数，消除编译警告
    UNUSED(state);

    // 获取接收缓冲区，buf[0] 是 Report ID，后面是数据
    USBD_CUSTOM_HID_HandleTypeDef *hhid =
        (USBD_CUSTOM_HID_HandleTypeDef *)hUsbDeviceFS.pClassData;
    uint8_t *buf = hhid->Report_buf;

    switch (buf[0])  // 根据 Report ID 判断 Windows 发来了什么命令
    {
        case 0x11:  // Create New Effect：Windows 申请创建一个效果槽位
        {
					block_load_buf[1] = next_block;  // 填入分配的槽位
					block_load_buf[2] = 0x01;        // Success
					block_load_buf[3] = 0x28;        // Pool Available
					printf("Create New Effect, assigned block=%d\r\n", next_block);
					if (next_block < 40) next_block++;
					break;  // 不发 IN，等 Windows 来 GET_REPORT
        }
        case 0x01:  // Set Effect：Windows 设置效果的基本参数（类型、时长等）
        {
					printf("Set Effect block=%d type=%d\r\n", buf[1], buf[2]);
					break;
        }
        case 0x03: printf("Set Condition\r\n"); break;  // 弹簧/阻尼参数
        case 0x04: printf("Set Periodic\r\n"); break;   // 正弦波等周期效果参数
        case 0x05: printf("Set Constant\r\n"); break;   // 恒力参数
        case 0x0A: printf("Effect Op block=%d op=%d\r\n", buf[1], buf[2]); break;  // 启动/停止效果
        case 0x0B: printf("Block Free block=%d\r\n", buf[1]); break;   // 释放槽位
        case 0x0C: printf("Device Control %d\r\n", buf[1]); break;     // 使能/禁用执行器
        case 0x0D: printf("Device Gain %d\r\n", buf[1]); break;        // 全局力度
        default: break;
    }

    // 重新挂起接收，准备接收下一条命令
    if (USBD_CUSTOM_HID_ReceivePacket(&hUsbDeviceFS) != (uint8_t)USBD_OK)
        return -1;

    return (USBD_OK);
}

/* USER CODE BEGIN 7 */
static uint8_t* CUSTOM_HID_GetReport_FS(uint8_t report_id, uint16_t *ReportLength)
{
    printf("GetReport id=0x%02X\r\n", report_id);
    if (report_id == HID_ID_POOLREP) {        // 0x13
        *ReportLength = sizeof(pool_report_buf);
        return pool_report_buf;
    } else if (report_id == HID_ID_BLKLDREP) { // 0x12
        *ReportLength = sizeof(block_load_buf);
        return block_load_buf;
    }
    return NULL;
}
/* USER CODE END 7 */

