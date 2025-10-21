#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_now.h>
#include <HttpClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h> 
#include <Preferences.h>

#include <lvgl.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h> 
#include <Adafruit_GFX.h>
#include <ESP32Time.h>
#include <PNGdec.h>

#include "UI/ui.h"
#include "UI/ui_screen.h"
#include "UI/ui_buttons.h"
#include "UI/ui_settings.h"

#include "../lvgl/lvgl.h"

#include "../include/lgfx.h"
#include "../include/defines.h"
#include "../include/debug.h"
#include "../include/structs.h"
#include "../include/settings.h"
#ifdef USE_MULTI_THREAD
  #include "../include/tasks.h"
#endif

#include "touch.h"

static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t disp_draw_buf[800 * 480 / 10];
//static lv_color_t disp_draw_buf;
static lv_disp_drv_t disp_drv;

char macStr[18];
uint8_t bs8_address[] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6};
esp_now_peer_info_t peerInfo;

ESP32Time rtc(0);
char time_str[9];

uint8_t WiFiChannel;
s_espNow espNowPacket;
s_espNowButtons buttons[10];

long switchesLastAlive;
long switchesLastSignal;

Settings *mySettings;

#ifdef USE_MULTI_THREAD
  // Tasks
  #ifdef DISPLAY_AT_CORE1
    TaskHandle_t t_core1_tft;
  #endif

  // Semaphores
  SemaphoreHandle_t semaphoreData;
#endif

long now;  

TwoWire I2Cone = TwoWire(0);
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &I2Cone, OLED_RESET);

SPIClass& spi = SPI;
uint16_t touchCalibration_x0 = 300, touchCalibration_x1 = 3600, touchCalibration_y0 = 300, touchCalibration_y1 = 3600;
uint8_t  touchCalibration_rotate = 1, touchCalibration_invert_x = 2, touchCalibration_invert_y = 0;

Debug *debug;


// Elecrow Display callbacks

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);  

  //lcd.fillScreen(TFT_WHITE);
#if (LV_COLOR_16_SWAP != 0)
 lcd.pushImageDMA(area->x1, area->y1, w, h,(lgfx::rgb565_t*)&color_p->full);
#else
  lcd.pushImageDMA(area->x1, area->y1, w, h,(lgfx::rgb565_t*)&color_p->full);//
#endif

  lv_disp_flush_ready(disp);

}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
      #ifndef MODE_RELEASE
        debug->print(DEBUG_LEVEL_DEBUG2, "Data x :" );
        debug->println(DEBUG_LEVEL_DEBUG2, touch_last_x );
        debug->print(DEBUG_LEVEL_DEBUG2, "Data y :" );
        debug->println(DEBUG_LEVEL_DEBUG2, touch_last_y );
      #endif
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
  delay(15);
}

void initializeUI() {  

  debug->println(DEBUG_LEVEL_INFO, "initialize UI...");  
  ui_init();
  #ifdef USE_MODULE_CONTROLS
    #ifdef USE_MAIN_TAB_VIEW
      ui_init_buttons(ui_MainTabView);
    #else
      ui_init_buttons(ui_Main);
    #endif
  #endif
  #ifdef USE_MODULE_SETTINGS
    ui_init_settings(ui_MainTabView);
  #endif
  ui_init_screen_events();

  /*
  #ifdef ENABLE_STARTUP_LOGO
  lv_disp_load_scr(ui_ScreenLogo);  
  for( int i=0; i < 100; i++ ){
      lv_task_handler();  
      delay(10);
  }
  lv_scr_load_anim(ui_Screen1, LV_SCR_LOAD_ANIM_MOVE_TOP, 500, 0, true);
  #else
  ui_init();
  #endif
  */
  
}
void setupDisplay() {
  debug->println(DEBUG_LEVEL_INFO, "Configuring display...");

  screenWidth = lcd.width();
  screenHeight = lcd.height();

  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * screenHeight / 10);

  /* Initialize the display */
  lv_disp_drv_init(&disp_drv);
  /* Change the following line to your display resolution */
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /* Initialize the (dummy) input device driver */
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif

  lcd.fillScreen(0x000000u);
}

void initializeDisplay() {
  debug->println(DEBUG_LEVEL_INFO, "Initializing display...");
  lcd.begin();
  lcd.fillScreen(0x000000u);
  lcd.setTextSize(2); 
  //lcd.setBrightness(127);
}

void loadSettings() {
  mySettings = new Settings();
  #ifdef CLEAR_SETTINGS
    mySettings->setDefaults();
    mySettings->save();
  #else
    mySettings->load();    
  #endif
  
  #ifdef USE_MODULE_SETTINGS
    #ifdef USE_MODULE_CONTROLS
      if (mySettings->IsWaitRelay()) {
        lv_obj_add_state(ui_WaitRelay, LV_STATE_CHECKED);
      } else {
        lv_obj_add_state(ui_WaitRelay, LV_STATE_DEFAULT);
      }
    #endif
  #endif
}

void startRTC() {
  #ifdef ENABLE_RTC_CLOCK
    debug->println(DEBUG_LEVEL_INFO, "Starting RTC...");
    myRTC.start();
  #endif
}

void initializeDebug() {
  // Initialize Serial and set debug level
  debug = new Debug();
  #if defined(MODE_DEBUG_FULL)
    debug->start(115200, DEBUG_LEVEL_DEBUG2);
  #elif defined(MODE_DEBUG)
    debug->start(115200, DEBUG_LEVEL_DEBUG);
  #elif defined(MODE_RELEASE_INFO)
    debug->start(115200, DEBUG_LEVEL_INFO);
  #elif defined(MODE_RELEASE)
    debug->start(115200, DEBUG_LEVEL_NONE);
  #else
    #error "Select build mode for logs"
  #endif  
}

void initializeTouchScreen() {
  debug->println(DEBUG_LEVEL_INFO, "Initializing touch screen...");
  touch_init();
}

void initializeLVGL() {
  debug->println(DEBUG_LEVEL_INFO, "Initializing LVGL...");
  lv_init();
}

#ifdef USE_MODULE_CONTROLS
void setupButton(int idx, bool enabled, int state, lv_obj_t *obj, uint32_t colorOn, uint32_t colorOff) {

  buttons[idx].enabled = enabled;
  buttons[idx].state = state;
  buttons[idx].needsUpdate = true;
  buttons[idx].obj = obj;
  buttons[idx].colorOn = colorOn;
  buttons[idx].colorOff = colorOff;

}

void setupButtons() {

  debug->println(DEBUG_LEVEL_INFO, "Creating Buttons...");

  setupButton(1, true, STATE_OFF, ui_Panel1, COLOR_BLUE, BUTTON_BACKGROUND);
  setupButton(2, true, STATE_OFF, ui_Panel2, COLOR_BLUE, BUTTON_BACKGROUND);
  setupButton(3, true, STATE_OFF, ui_Panel3, COLOR_BLUE, BUTTON_BACKGROUND);
  setupButton(4, true, STATE_OFF, ui_Panel7, COLOR_RED, BUTTON_BACKGROUND);
  setupButton(5, true, STATE_OFF, ui_Panel4, COLOR_BLUE, BUTTON_BACKGROUND);
  setupButton(6, true, STATE_OFF, ui_Panel5, COLOR_BLUE, BUTTON_BACKGROUND);
  setupButton(7, true, STATE_OFF, ui_Panel6, COLOR_BLUE, BUTTON_BACKGROUND);
  setupButton(8, false, STATE_OFF, nullptr, 0, 0);

  lv_label_set_text(ui_ControlLabel1, "Main LED Bar");
  lv_label_set_text(ui_ControlLabel2, "Roof LED Bar");
  lv_label_set_text(ui_ControlLabel3, "Roof Lights");
  lv_label_set_text(ui_ControlLabel4, "Left Lights");
  lv_label_set_text(ui_ControlLabel5, "Right Lights");
  lv_label_set_text(ui_ControlLabel6, "Rear Lights");
  lv_label_set_text(ui_ControlLabel7, "Emergency Strobe Lights");    
}
#endif

esp_err_t sendToRelay(int btnId, int state) {
  
  debug->println(DEBUG_LEVEL_INFO, "Sending esp_now to Relay");  
  espNowPacket.type = 2; //Buttons
  espNowPacket.id = btnId;
  espNowPacket.value = state;

  if (!mySettings->IsWaitRelay()) {
    #ifdef USE_MULTI_THREAD
      xSemaphoreTake(semaphoreData, portMAX_DELAY);
    #endif
    buttons[btnId].state = state;
    buttons[btnId].needsUpdate = true;
    #ifdef USE_MULTI_THREAD
      xSemaphoreGive(semaphoreData);
    #endif
  }
 
  return esp_now_send(bs8_address, (uint8_t *) &espNowPacket, sizeof(s_espNow));
}

void onWaitRelayPressed(bool pressed) {
  mySettings->setWaitRelay(pressed);
}

#ifdef USE_MODULE_CONTROLS
void btnClick(uint8_t btnId) {

  uint8_t state = buttons[btnId].state;

  if (state == STATE_OFF) {
    state = STATE_ON;
  } else {
    state = STATE_OFF;
  }

  #ifndef MODE_RELEASE
    debug->print(DEBUG_LEVEL_DEBUG, "Request change for id ");
    debug->print(DEBUG_LEVEL_DEBUG, btnId);
    debug->print(DEBUG_LEVEL_DEBUG, ", new state = ");
    debug->println(DEBUG_LEVEL_DEBUG, state);
  #endif

  #ifndef MODE_RELEASE
    esp_err_t result = sendToRelay(btnId, state);
    if (result == ESP_OK) {
      debug->println(DEBUG_LEVEL_DEBUG, " [OK]");
    } else {
      debug->println(DEBUG_LEVEL_DEBUG, " [ERROR]");
      debug->println(DEBUG_LEVEL_ERROR, esp_err_to_name(result));
    }    
  #else
    sendToRelay(btnId, state);
  #endif
}

void onMainBar(lv_event_t * e)
{
	btnClick(1);
}

void onLeftLeds(lv_event_t * e)
{
	btnClick(5);
}

void onRightLeds(lv_event_t * e)
{	
	btnClick(6);
}

void onRearLeds(lv_event_t * e)
{
	btnClick(7);
}

void onRoofBar(lv_event_t * e)
{
	btnClick(2);
}

void onRoofLeds(lv_event_t * e)
{
	btnClick(3);
}

void onStrobeLights(lv_event_t * e)
{
	btnClick(4);	
}
#endif

#ifndef MODE_RELEASE
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  
    debug->print(DEBUG_LEVEL_DEBUG2, "Packet to: ");
    // Copies the sender mac address to a string
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    debug->print(DEBUG_LEVEL_DEBUG2, macStr);
    debug->print(DEBUG_LEVEL_DEBUG2, " send status:\t");
    debug->println(DEBUG_LEVEL_DEBUG2, status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

}
#endif

// Callback when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {

  #ifndef MODE_RELEASE
    debug->print(DEBUG_LEVEL_DEBUG2, "Packet from: ");
    // Copies the sender mac address to a string
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    debug->println(DEBUG_LEVEL_DEBUG2, macStr);
    debug->print(DEBUG_LEVEL_DEBUG2, "Bytes received: ");
    debug->println(DEBUG_LEVEL_DEBUG2, len);
  #endif

  memcpy(&espNowPacket, incomingData, sizeof(espNowPacket));
  int16_t type = espNowPacket.type;
  int16_t id = espNowPacket.id;
  int16_t value = espNowPacket.value;

  now = millis();

  if (type == 2) { // Buttons
    switchesLastSignal = now;
    switchesLastAlive = now;
    #ifdef USE_MULTI_THREAD
      xSemaphoreTake(semaphoreData, portMAX_DELAY);
    #endif
    if (buttons[id].obj != nullptr && buttons[id].state != value) {
      buttons[id].state = value;
      buttons[id].needsUpdate = true;
    } else {
      buttons[id].needsUpdate = false;
    }
    #ifdef USE_MULTI_THREAD
      xSemaphoreGive(semaphoreData);
    #endif
  }
}

void initializeEspNow() {

  debug->println(DEBUG_LEVEL_INFO, "Initializing ESP-NOW...");
 
  WiFi.mode(WIFI_STA);
  
  debug->print(DEBUG_LEVEL_INFO, "MAC Address: ");
  debug->println(DEBUG_LEVEL_INFO, WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    debug->println(DEBUG_LEVEL_ERROR, "Error initializing ESP-NOW");
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  #ifndef MODE_RELEASE
    esp_now_register_send_cb(OnDataSent);
  #endif

  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // register first peer  
  memcpy(peerInfo.peer_addr, bs8_address, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    debug->println(DEBUG_LEVEL_ERROR, "Failed to add BS-8 peer");
  } else {
    debug->println(DEBUG_LEVEL_INFO, "BS-8 client added");
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void createSemaphores() {
#ifdef USE_MULTI_THREAD
  semaphoreData = xSemaphoreCreateMutex();
  xSemaphoreGive(semaphoreData);
#endif
}

void createTasks() {
#ifdef USE_MULTI_THREAD
  debug->println(DEBUG_LEVEL_INFO, "Creating Tasks...");
  #ifdef DISPLAY_AT_CORE1
    debug->println(DEBUG_LEVEL_INFO, "Staring up Display Manager...");

    xTaskCreatePinnedToCore(
      tft_task,       // Task function.
      "TFT_Manager",  // Name of task.
      10000,          // Stack size of task
      NULL,           // Parameter of the task
      0,              // Priority of the task
      &t_core1_tft,   // Task handle to keep track of created task
      1);             // Pin task to core 1

    vTaskSuspend(t_core1_tft);
  #endif

  // other tasks here

  debug->println(DEBUG_LEVEL_INFO, "Setup completed\nStarting tasks...");

  #ifdef DISPLAY_AT_CORE1
    debug->println(DEBUG_LEVEL_INFO, "Starting Display...");
    vTaskResume(t_core1_tft);
    delay(200);
  #endif

#endif
}

void setup() {

  initializeDebug(); 

  debug->println(DEBUG_LEVEL_INFO, "Staring up...");

  initializeDisplay();
  delay(200);

  initializeLVGL();
  initializeTouchScreen();
  setupDisplay();

  createSemaphores();  
  
  initializeEspNow();
  startRTC();  

  initializeUI();
  #ifdef USE_MODULE_CONTROLS
    setupButtons();
  #endif

  loadSettings();

  createTasks();

  debug->println(DEBUG_LEVEL_INFO, "Setup completed");
  debug->println(DEBUG_LEVEL_INFO, "Requesting Buttons status...");

  switchesLastAlive = 0;
  switchesLastSignal = 0;

}

void loop() {

  lv_timer_handler();
  vTaskDelay(DELAY_MAIN_TASK / portTICK_PERIOD_MS);

  now = millis();

  #ifdef USE_MODULE_CONTROLS
    #ifdef USE_MODULE_SETTINGS
      #ifdef SHOW_TOP_BAR
      if (now > (switchesLastSignal + 1000)) {
        lv_obj_set_style_opa(ui_ControlsStatus, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
      } else {
        lv_obj_set_style_opa(ui_ControlsStatus, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
      }
      #endif
    #endif
    // if (now > (switchesLastAlive + 500)) { //every 500ms 
    //   switchesLastAlive = now;     
    //   sendToRelay(0, 0);
    // }
  #endif

  #ifndef DISPLAY_AT_CORE1  
    #ifdef USE_MODULE_CONTROLS
    for (int i=1; i<=BUTTONS_ON_SCREEN; i++) {
      if (buttons[i].needsUpdate) {
        buttons[i].needsUpdate = false;
        if (buttons[i].state == STATE_OFF) {
          lv_obj_set_style_bg_color(buttons[i].obj, lv_color_hex(buttons[i].colorOff), LV_PART_MAIN | LV_STATE_DEFAULT);
        } else {
          lv_obj_set_style_bg_color(buttons[i].obj, lv_color_hex(buttons[i].colorOn), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
      }    
    }
    #endif
  #endif

}
