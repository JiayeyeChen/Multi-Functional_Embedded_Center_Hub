#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include "main.h"
#include "touch_800x480.h"
#include "lcd_rgb.h"
#include "lcd_pwm.h"
#include "usb.h"
#include "tmotor_ak10-9_v2.h"
#include "ak10-9_v2_testing.h"
#include "exoskeleton.h"
#include "adc.h"
#include "bldc_actuators_testing.h"
#include "mrdoor.h"

typedef struct
{
  uint8_t ifPageInitialized;
  void (*Page) (void);
  void (*PageInit) (void);
}PageHandle;

typedef struct
{
  uint16_t x;
  uint16_t y;
  uint16_t xLen;
  uint16_t yLen;
}VirtualButtonPosition;

typedef struct
{
  uint16_t x;
  uint16_t y;
  uint16_t xLen;
  uint16_t yLen;
  uint16_t xSlider;
  uint16_t ySlider;
  uint16_t sliderLen;
  uint16_t sliderWidth;
}VirtualLinearPotentialmeterPosition;

typedef struct
{
  VirtualLinearPotentialmeterPosition pos;
  uint8_t     ifSliderPressed;
  uint8_t     preIfSliderPressed;
  uint32_t    sliderColorPressed;
  uint32_t    sliderColorUnpressed;
  float       maxVal;
  float       minVal;
  uint16_t    preFingerPosX;
  uint16_t    preFingerPosY;
  uint8_t     ifNeedRefresh;
  float*      controlledValue;
}LinearPotentialmeterHandle;

typedef struct
{
  VirtualButtonPosition    pos;
  uint32_t                 colorPressed;
  uint32_t                 colorUnpressed;
  uint8_t                  ifNeedRefresh;
  uint8_t                  ifPressed;
  uint8_t                  preIfPressed;
}ButtonHandle;

typedef struct
{
  uint16_t x;
  uint16_t y;
  uint16_t r;
}VirtualJoystickPosition;

typedef struct
{
  VirtualJoystickPosition pos;
  VirtualJoystickPosition stickPos;
  uint32_t backgroundColor, stickColor;
  uint8_t ifTouched;
  uint16_t fingerDetectR;
  float xAxisPos, yAxisPos;
  float* outputValX;
  float* outputValY;
  float outputValCenter, outputValMagnitude;
  float xDirectionCorrection, yDirectionCorrection;
}JoystickHandle;

typedef struct
{
  PageHandle* curPage;
  PageHandle* prePage;
}UIHandle;

void UI_Init(void);
JoystickHandle Joystick_Create(uint16_t x, uint16_t y, uint16_t r, uint32_t background_color, \
                               uint32_t stick_color, uint16_t stick_r, uint16_t finger_detect_r, \
                               float* output_x, float* output_y, float center_val, float mag_val, float x_direction_correction, float y_direction_correction);
void JoystickUpdate(JoystickHandle* hjoy);
ButtonHandle Button_Create(uint16_t x, uint16_t y, uint16_t xLen, uint16_t yLen, char label[], uint32_t colorUnpressed, uint32_t colorPressed);
LinearPotentialmeterHandle  Potentialmeter_Create(uint16_t x, uint16_t y, uint16_t xLen, \
                                                  uint16_t yLen, uint16_t sliderLen, uint16_t sliderWidth, \
                                                  uint32_t sliderColorUnpressed, uint32_t sliderColorPressed, \
                                                  uint32_t slotColor, float minVal, float maxVal, float startVal, float* ctrVal);
void         PotentialmeterUpdate(LinearPotentialmeterHandle* hpot);
void         ButtonUpdate(ButtonHandle* hbutton);
void         PotentialmeterSliderGoTo(LinearPotentialmeterHandle* hpot, float go_to_val);
uint8_t      ifButtonPressed(ButtonHandle* hbutton);
void         UI(void);
void         UI_Page_Change_To(PageHandle* hpage);
void         UI_Page_LowerLimb_Exoskeleton(void);
void         UI_Page_LowerLimb_Exoskeleton_Init(void);
void         UI_Page_LowerLimb_Exoskeleton_SystemID(void);
void         UI_Page_LowerLimb_Exoskeleton_SystemID_Init(void);
void         UI_Page_LowerLimb_Exoskeleton_GravityCompensation(void);
void         UI_Page_LowerLimb_Exoskeleton_GravityCompensation_Init(void);
void         UI_Page_LowerLimb_Exoskeleton_ParameterAdjusting(void);
void         UI_Page_LowerLimb_Exoskeleton_ParameterAdjusting_Init(void);
void         UI_Page_LowerLimb_Exoskeleton_MuscularTorqueMonitor(void);
void         UI_Page_LowerLimb_Exoskeleton_MuscularTorqueMonitor_Init(void);
void         UI_Page_Home1(void);
void         UI_Page_Home1_Init(void);
void         UI_Page_AK10_9_ManualControlCubeMarsFWServoMode(void);
void         UI_Page_AK10_9_ManualControlCubeMarsFWServoMode_Init(void);
void         UI_Page_AK10_9_ManualControlCubeMarsFWMITMode(void);
void         UI_Page_AK10_9_ManualControlCubeMarsFWMITMode_Init(void);
void         UI_Page_BNO055_Monitor(void);
void         UI_Page_BNO055_Monitor_Init(void);
void         UI_Page_TMotor_Acceleration_Observer_Project(void);
void         UI_Page_TMotor_Acceleration_Observer_Project_Init(void);
void         UI_Page_ADC_Monitor_Init(void);
void         UI_Page_ADC_Monitor(void);
void         UI_Page_CustomizedIMU(void);
void         UI_Page_CustomizedIMU_Init(void);
void 				 UI_Page_BenMoKeJiM15Testing(void);
void 				 UI_Page_BenMoKeJiM15Testing_Init(void);
void 				 UI_Page_LinKongKeJiTesting(void);
void 				 UI_Page_LinKongKeJiTesting_Init(void);
void			   UI_Page_MrDoorTestingTMotor(void);
void			   UI_Page_MrDoorTestingTMotor_Init(void);
void         UI_Page_MrDoorTestingBenMoKeJi(void);
void         UI_Page_MrDoorTestingBenMoKeJi_Init(void);
void			   UI_Page_MrDoorTestingTypeSelection(void);
void			   UI_Page_MrDoorTestingTypeSelection_Init(void);
void         UI_Page_TkCalibration(void);
void         UI_Page_TkCalibration_Init(void);
void         UI_Page_ExoskeletonMotorDurabilityTest(void);
void         UI_Page_ExoskeletonMotorDurabilityTest_Init(void);


/* Lin Kong Ke Ji Testing */
extern uint8_t ifKeepReadingAngle;
////////////////////////////

extern UIHandle hUI;
extern PageHandle UIPage_AK10_9_ManualControlCubeMarsFWServoMode, UIPage_AK10_9_ManualControlCubeMarsFWMITMode;
extern PageHandle UIPage_BenMoKeJiM15_Testing, UIPage_LinKongKeJi_Testing;
#endif
