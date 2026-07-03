#include <lvgl.h>
#include <Preferences.h>
#include <Arduino.h>
#include "driver/ledc.h"

#include <Arduino_GFX_Library.h>

#include "TCA9554.h"

#include <Wire.h>
#include "esp_lcd_touch_axs15231b.h"
#include "ui.h"


/*Defining pins for the board*/
#define LCD_QSPI_CS   12
#define LCD_QSPI_CLK  5
#define LCD_QSPI_D0   1
#define LCD_QSPI_D1   2
#define LCD_QSPI_D2   3
#define LCD_QSPI_D3   4
#define GFX_BL        6// default backlight pin
#define I2C_SDA       8
#define I2C_SCL       7
#define PedalPin      11 //The pedal will be connected to the pin 10 (GPIO11)
#define HandpiecePin  10 //The handpiece will be connected to the pin 12 (GPIO10)


TCA9554 TCA(0x20);

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_QSPI_CS, LCD_QSPI_CLK, LCD_QSPI_D0, LCD_QSPI_D1, LCD_QSPI_D2, LCD_QSPI_D3);
Arduino_GFX *gfx = new Arduino_AXS15231B(bus, -1 /* RST */, 0 /* rotation */, false, 320, 480);

#define DIRECT_RENDER_MODE  1

/*Defining the system variables according to the board we are using*/
uint32_t screenWidth;
uint32_t screenHeight;
uint32_t bufSize;
lv_disp_draw_buf_t draw_buf;
lv_color_t *disp_draw_buf1;
lv_color_t *disp_draw_buf2;
lv_disp_drv_t disp_drv;


/*Defining the variable of the project that will be saved in the system!
These variables will be called and saved in the memory for when the machine starts or closes to 
make it remember the last time it was used*/

Preferences systemValues;
int lastValuePedal =0; //To update the screen only when the values change

int samples[100];   // Circular buffer for the dynamic filter
int sampleIndex = 0;

int SoftFrequencyValue; //Variables that will define the saved values on the buttons
int SoftPowerValue;
int MediumFrequencyValue;
int MediumPowerValue;
int HardFrequencyValue;
int HardPowerValue;
int CustomFrequencyValue;
int CustomPowerValue;
int MaximumSolenoidPWM;
int PedalZero;

uint32_t SolenoidTime; //Values for the sustem configuration
int SolenoidMaximumVoltage;
int SystemInputVoltage;

int LastFrequencyValue; //Values that will be saved for the machine to start where it was left off
int LastPowerValue;

unsigned long previousMillis = 0; //used to call lvgl handler without using delays
unsigned long CurrentTime = 0;
unsigned long runningTotal = 0;
//unsigned long PreviousTime = 0; //not using right now


// -------- GLOBAL STATE --------



//bool solenoidOn = false;
//float sharedPedalValue;
float normalized;
float sensorValue;
float pedalRead;
int previousSensorValue = 0;

volatile int sharedPowerSlider;
volatile int sharedFrequencySlider;
volatile float sharedPedalValue;
volatile bool sharedFrequencyMode; //This will be shared with both cores, one for lvgl and onther for the pedal code
volatile bool sharedPowerMode;
volatile bool uiReady = false;
volatile uint32_t offTime;
volatile unsigned long PreviousTime = 0;


uint16_t PWMValue;    // 0–255 (for analogWrite)


// ---------------- PWM CONFIG ----------------
const int pwmChannel    = 0;
const int pwmResolution = 10;        // 0–1023


uint32_t FixedPeriodMS  = 200;       // Used in Mode 2
uint32_t OnPWMValue     = 700;       // Fixed amplitude for Mode 1


/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, w, h);

  lv_disp_flush_ready(disp_drv);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {

  touch_data_t touch_data;
  bsp_touch_read();

  // int16_t x[1], y[1];
  // uint8_t touched = touch.getPoint(x, y, 1);

  if (bsp_touch_get_coordinates(&touch_data)) {
    data->state = LV_INDEV_STATE_PR;
    /*Set the coordinates*/
    data->point.x = touch_data.coords[0].x;
    data->point.y = touch_data.coords[0].y;
  } 
  else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void setup() {
  createPreferences();
  StartupDataRetrieval();
  ledcAttachChannel(HandpiecePin, 2000, 10, 0); //Handpiece pin with 2000Hz PWM for the mosfet, this guarantees that almost every mosfet will work due to the "low"frequency

  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.begin(115200);
  pinMode (PedalPin, INPUT);

  // Create LVGL task on Core 0
  xTaskCreatePinnedToCore(
      lvglTask,
      "LVGL Task",
      20000,
      NULL,
      1,
      NULL,
      0
  );

  // Create control task on Core 1
  xTaskCreatePinnedToCore(
      controlTask,
      "Control Task",
      10000,
      NULL,
      1,
      NULL,
      1
  );
  


  #ifdef GFX_BL
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
  #endif



  lv_init();

  gfx->begin();
  screenWidth = gfx->width();
  screenHeight = gfx->height();

  bsp_touch_init(&Wire, -1, 0, screenWidth, screenHeight);

  #ifdef DIRECT_RENDER_MODE
  bufSize = screenWidth * screenHeight;
  #else
  bufSize = screenWidth * 80;
  #endif


  disp_draw_buf1 = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
  disp_draw_buf2 = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);

  if (!disp_draw_buf1 || !disp_draw_buf2) 
  {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  } 
  else 
  {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf1, disp_draw_buf2, bufSize);
    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    
    #ifdef DIRECT_RENDER_MODE
    disp_drv.full_refresh = true;
    #endif

    lv_disp_drv_register(&disp_drv);

    /* Initialize the input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);
    

    ui_init();
    //lv_timer_create(MainLoadValues, 20, NULL);
    uiReady = true;
  }
}


void updateBar()
{
    if (!uiReady || ui_PedalBar == NULL) return;

    if (sharedPedalValue != lastValuePedal) {
        lv_bar_set_value(ui_PedalBar, sharedPedalValue, LV_ANIM_OFF);
        lastValuePedal = sharedPedalValue;
    }
}





void loop() {

}

void lvglTask(void *pvParameters)
{
    while (true)
    {
      updateBar();
      lv_timer_handler(); // LVGL engine
      vTaskDelay(pdMS_TO_TICKS(5)); // 10ms refresh
    }
  
}

void controlTask(void *pvParameters)
{
    while (true)
    {
        ReadPedalValue();
        vTaskDelay(1); // Needs to be here because it was glitching
    }
}



void ReadPedalValue(){
  
  pedalRead = analogRead(PedalPin);

  runningTotal -= samples[sampleIndex];   // Remove oldest sample
  samples[sampleIndex] = pedalRead;     // Store new sample
  runningTotal += samples[sampleIndex];   // Add new sample
  

  // Advance circular index
  sampleIndex++;
  if (sampleIndex >= 100) {
      sampleIndex = 0;
  }

  sensorValue = runningTotal / 100;// Compute average of measuremets to avoid peak readings

  sharedPedalValue = round(map(sensorValue, PedalZero, 4095, -40, 100));  // Scale to 0-100


  if (sharedPedalValue>0){
    if (sharedFrequencyMode){
      SpeedPedal();
      updateSolenoid();
    }
    else if (sharedPowerMode){
      PowerPedal();
      updateSolenoid();
    }
  }
  else{
    ledcWrite(HandpiecePin, 0);
  }
  
}




void updateSolenoid()
{

  if(millis() <= PreviousTime + SolenoidTime){
    ledcWrite(HandpiecePin, PWMValue);
    }
  
  else if(millis() < PreviousTime + SolenoidTime + offTime) {
    ledcWrite(HandpiecePin, 0);
    }
  else{
    ledcWrite(HandpiecePin,0);
    PreviousTime=millis();
  } 
   
} 


void SpeedPedal(){ //The pedal choses how fast is the blow the power is fixed

  normalized = sensorValue / 4095.0;

  float frequency = 1+normalized*sharedFrequencySlider;
  uint32_t period = 1000.0 / frequency;

  offTime = period - SolenoidTime;
  if (offTime < 1) offTime = 1;

  // Fixed amplitude (from your variable)
  PWMValue = sharedPowerSlider*MaximumSolenoidPWM/100;
}

void PowerPedal(){ //The pedal choses how powerful is the blow the frequency is fixed
  normalized = sensorValue / 4095.0;

  float frequency = sharedFrequencySlider;
  uint32_t period = 1000.0 / frequency;

  offTime = period - SolenoidTime;
  if (offTime < 1) offTime = 1;

  PWMValue = normalized*sharedPowerSlider*MaximumSolenoidPWM/100;
}

void createPreferences() { //saving the parameters for the machine to work correctly, I tried to name variables in a way that is readable

  systemValues.begin("systemPrefs", false);   // open preferences for writing the code, if it was true it wouuld be read only

  if(!systemValues.isKey("SoftFreq"))       systemValues.putInt("SoftFreq", 0);
  if(!systemValues.isKey("SoftPower"))      systemValues.putInt("SoftPower", 0);

  if(!systemValues.isKey("MedFreq"))        systemValues.putInt("MedFreq", 0);
  if(!systemValues.isKey("MedPower"))       systemValues.putInt("MedPower", 0);

  if(!systemValues.isKey("HardFreq"))       systemValues.putInt("HardFreq", 0);
  if(!systemValues.isKey("HardPower"))      systemValues.putInt("HardPower", 0);

  if(!systemValues.isKey("CustFreq"))       systemValues.putInt("CustFreq", 0);
  if(!systemValues.isKey("CustPower"))      systemValues.putInt("CustPower", 0);

  if(!systemValues.isKey("SolTime"))        systemValues.putInt("SolTime", 0);
  if(!systemValues.isKey("SolMaxVolt"))     systemValues.putInt("SolMaxVolt", 0);

  if(!systemValues.isKey("SysVolt"))        systemValues.putInt("SysVolt", 0);
  if(!systemValues.isKey("HandpiecePWM"))        systemValues.putInt("HandpiecePWM", 0);

  if(!systemValues.isKey("FreqMode"))        systemValues.putBool("FreqMode", 0);
  if(!systemValues.isKey("PowerMode"))        systemValues.putBool("PowerMode", 0);

  if(!systemValues.isKey("SharedFreq"))        systemValues.putInt("SharedFreq", 0);
  if(!systemValues.isKey("SharedPower"))        systemValues.putInt("SharedPower", 0);

  if(!systemValues.isKey("LastFreq"))        systemValues.putInt("LastFreq", 0);
  if(!systemValues.isKey("LastPower"))        systemValues.putInt("LastPower", 0);

  if(!systemValues.isKey("SharedPedalZero"))        systemValues.putInt("SharedPedalZero", 0);

  systemValues.end();
}


void StartupDataRetrieval(){
  systemValues.begin("systemPrefs", true); // read/write
  SoftFrequencyValue      =systemValues.getInt("SoftFreq", 0);
  SoftPowerValue          =systemValues.getInt("SoftPower", 0);
  MediumFrequencyValue    =systemValues.getInt("MedFreq", 0);
  MediumPowerValue        =systemValues.getInt("MedPower", 0);
  HardFrequencyValue      =systemValues.getInt("HardFreq", 0);
  HardPowerValue          =systemValues.getInt("HardPower", 0);
  CustomFrequencyValue    =systemValues.getInt("CustFreq", 0);
  CustomPowerValue        =systemValues.getInt("CustPower", 0);
  SolenoidTime            =systemValues.getInt("SolTime", 0);
  SolenoidMaximumVoltage  =systemValues.getInt("SolMaxVolt", 0);
  SystemInputVoltage      =systemValues.getInt("SysVolt", 0);
  LastFrequencyValue      =systemValues.getInt("LastFreq", 0);
  LastPowerValue          =systemValues.getInt("LastPower", 0);
  MaximumSolenoidPWM      =systemValues.getInt("HandpiecePWM", 0);
  PedalZero               =systemValues.getInt("SharedPedalZero", 0);

  sharedPowerSlider       =systemValues.getInt("SharedPower", 0);
  sharedFrequencySlider   =systemValues.getInt("SharedFreq", 0);

  sharedPowerMode         =systemValues.getBool("PowerMode", 0);
  sharedFrequencyMode     =systemValues.getBool("FreqMode", 0);

  systemValues.end();
}

void SoftSetValue(lv_event_t * e)
{
  static char buffer[10]; // Buffer to hold the string since the label odes not accept int as an argument on the text
  sprintf(buffer, "%d", SoftFrequencyValue);
  _ui_label_set_property(ui_FrequencyLabel, _UI_LABEL_PROPERTY_TEXT, buffer );
  _ui_slider_set_property(ui_FrequencySlider, _UI_SLIDER_PROPERTY_VALUE, SoftFrequencyValue);
 
  sprintf(buffer, "%d", SoftPowerValue);
  _ui_label_set_property(ui_PowerLabel, _UI_LABEL_PROPERTY_TEXT, buffer );
  _ui_slider_set_property(ui_PowerSlider, _UI_SLIDER_PROPERTY_VALUE, SoftPowerValue);
  SaveCurrentValues();
}

void SoftSaveValue(lv_event_t * e)
{
  systemValues.begin("systemPrefs", false);
	systemValues.putInt("SoftFreq", lv_slider_get_value(ui_FrequencySlider));
  systemValues.putInt("SoftPower", lv_slider_get_value(ui_PowerSlider));
  systemValues.end();
  StartupDataRetrieval(); 
}

void MediumSetValue(lv_event_t * e)
{
	static char buffer[10]; // Buffer to hold the string since the label odes not accept int as an argument on the text
  sprintf(buffer, "%d", MediumFrequencyValue);
  _ui_label_set_property(ui_FrequencyLabel, _UI_LABEL_PROPERTY_TEXT, buffer );
  _ui_slider_set_property(ui_FrequencySlider, _UI_SLIDER_PROPERTY_VALUE, MediumFrequencyValue);
 
  sprintf(buffer, "%d", MediumPowerValue);
  _ui_label_set_property(ui_PowerLabel, _UI_LABEL_PROPERTY_TEXT, buffer );
  _ui_slider_set_property(ui_PowerSlider, _UI_SLIDER_PROPERTY_VALUE, MediumPowerValue);
  SaveCurrentValues();
}

void MediumSaveValue(lv_event_t * e)
{
  systemValues.begin("systemPrefs", false);
	systemValues.putInt("MedFreq", lv_slider_get_value(ui_FrequencySlider));
  systemValues.putInt("MedPower", lv_slider_get_value(ui_PowerSlider));
  systemValues.end();
  StartupDataRetrieval();
}

void HardSetValue(lv_event_t * e)
{
	static char buffer[10]; // Buffer to hold the string since the label odes not accept int as an argument on the text
  sprintf(buffer, "%d", HardFrequencyValue);
  _ui_label_set_property(ui_FrequencyLabel, _UI_LABEL_PROPERTY_TEXT, buffer );
  _ui_slider_set_property(ui_FrequencySlider, _UI_SLIDER_PROPERTY_VALUE, HardFrequencyValue);
 
  sprintf(buffer, "%d", HardPowerValue);
  _ui_label_set_property(ui_PowerLabel, _UI_LABEL_PROPERTY_TEXT, buffer );
  _ui_slider_set_property(ui_PowerSlider, _UI_SLIDER_PROPERTY_VALUE, HardPowerValue);
  SaveCurrentValues();
}


void HardSaveValue(lv_event_t * e)
{
  systemValues.begin("systemPrefs", false);
	systemValues.putInt("HardFreq", lv_slider_get_value(ui_FrequencySlider));
  systemValues.putInt("HardPower", lv_slider_get_value(ui_PowerSlider));
  systemValues.end();
  StartupDataRetrieval();
}

void CustomSetValue(lv_event_t * e)
{
	static char buffer[10]; // Buffer to hold the string since the label odes not accept int as an argument on the text
  sprintf(buffer, "%d", CustomFrequencyValue);
  _ui_label_set_property(ui_FrequencyLabel, _UI_LABEL_PROPERTY_TEXT, buffer );
  _ui_slider_set_property(ui_FrequencySlider, _UI_SLIDER_PROPERTY_VALUE, CustomFrequencyValue);
 
  sprintf(buffer, "%d", CustomPowerValue);
  _ui_label_set_property(ui_PowerLabel, _UI_LABEL_PROPERTY_TEXT, buffer );
  _ui_slider_set_property(ui_PowerSlider, _UI_SLIDER_PROPERTY_VALUE, CustomPowerValue);
  SaveCurrentValues();
}

void CustomSaveValue(lv_event_t * e)
{
  systemValues.begin("systemPrefs", false);
	systemValues.putInt("CustFreq", lv_slider_get_value(ui_FrequencySlider));
  systemValues.putInt("CustPower", lv_slider_get_value(ui_PowerSlider));
  systemValues.end();
  StartupDataRetrieval();
}


void SaveSetupValue(lv_event_t * e)
{
  if (!uiReady) return;
  systemValues.begin("systemPrefs", false);
	systemValues.putInt("SysVolt", lv_slider_get_value(ui_InputVoltageSlider));
  systemValues.putInt("SolMaxVolt", lv_slider_get_value(ui_SolenoidVoltageSlider));
  systemValues.putInt("SolTime", lv_slider_get_value(ui_SolenoidPulseSlider));
  if ((1023*(lv_slider_get_value(ui_InputVoltageSlider)/lv_slider_get_value(ui_SolenoidVoltageSlider)))>1023){
    systemValues.putInt("HandpiecePWM", 1023);
  }
  else
  {
    systemValues.putInt("HandpiecePWM", 1023*(lv_slider_get_value(ui_InputVoltageSlider)/lv_slider_get_value(ui_SolenoidVoltageSlider)));
  }
  systemValues.end();
  //delay(5);
  StartupDataRetrieval();
}

void SettingsLoadValues(lv_event_t * e){
  static char buffer[10]; // Buffer to hold the string since the label odes not accept int as an argument on the text
  sprintf(buffer, "%d", SystemInputVoltage);
  _ui_label_set_property(ui_InputVoltageLabel, _UI_LABEL_PROPERTY_TEXT, buffer );
  _ui_slider_set_property(ui_InputVoltageSlider, _UI_SLIDER_PROPERTY_VALUE, SystemInputVoltage);
 
  sprintf(buffer, "%d", SolenoidTime);
  _ui_label_set_property(ui_SolenoidPulseLabel, _UI_LABEL_PROPERTY_TEXT, buffer );
  _ui_slider_set_property(ui_SolenoidPulseSlider, _UI_SLIDER_PROPERTY_VALUE, SolenoidTime);

  sprintf(buffer, "%d", SolenoidMaximumVoltage);
  _ui_label_set_property(ui_SolenoidVoltageLabel, _UI_LABEL_PROPERTY_TEXT, buffer );
  _ui_slider_set_property(ui_SolenoidVoltageSlider, _UI_SLIDER_PROPERTY_VALUE, SolenoidMaximumVoltage);
}


void MainLoadValues(lv_event_t * e){
  if (!uiReady) return;
  if (lv_scr_act() != ui_MainScreen) return;
  if (!lv_obj_is_valid(ui_PowerButton) || 
      !lv_obj_is_valid(ui_FrequencyButton)) return;

  if (sharedPowerMode){
    _ui_state_modify(ui_PowerButton, LV_STATE_CHECKED, _UI_MODIFY_STATE_ADD);
    _ui_state_modify(ui_FrequencyButton, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
  }
  else if(sharedFrequencyMode) {
    _ui_state_modify(ui_FrequencyButton, LV_STATE_CHECKED, _UI_MODIFY_STATE_ADD);
    _ui_state_modify(ui_PowerButton, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
  }

  lv_timer_create([](lv_timer_t * t){
    StartupDataRetrieval();
  }, 10, NULL);

}

void StartupLoadValues(lv_event_t * e){
  if (!uiReady) return;

	static char buffer[10]; // Buffer to hold the string since the label odes not accept int as an argument on the text
  sprintf(buffer, "%d", LastFrequencyValue);
  _ui_label_set_property(ui_FrequencyLabel, _UI_LABEL_PROPERTY_TEXT, buffer );
  _ui_slider_set_property(ui_FrequencySlider, _UI_SLIDER_PROPERTY_VALUE, LastFrequencyValue);
 
  sprintf(buffer, "%d", LastPowerValue);
  _ui_label_set_property(ui_PowerLabel, _UI_LABEL_PROPERTY_TEXT, buffer );
  _ui_slider_set_property(ui_PowerSlider, _UI_SLIDER_PROPERTY_VALUE, LastPowerValue);

}



void SaveCurrentValues()
{
  if (!uiReady) return;

  systemValues.begin("systemPrefs", false);
	systemValues.putInt("LastFreq", lv_slider_get_value(ui_FrequencySlider));
  systemValues.putInt("LastPower", lv_slider_get_value(ui_PowerSlider));
  systemValues.putInt("SharedFreq", lv_slider_get_value(ui_FrequencySlider));
  systemValues.putInt("SharedPower", lv_slider_get_value(ui_PowerSlider));
  systemValues.putBool("PowerMode", lv_obj_has_state(ui_PowerButton, LV_STATE_CHECKED));
  systemValues.putBool("FreqMode", lv_obj_has_state(ui_FrequencyButton, LV_STATE_CHECKED));

  systemValues.end();

  StartupDataRetrieval();
}


void ZeroPedal(){ //This zeroes the pedal to remove residual resistance
  sensorValue = analogRead(PedalPin);
  
  systemValues.begin("systemPrefs", false);
  systemValues.putInt("SharedPedalZero", sensorValue);
  systemValues.end();

  
}

