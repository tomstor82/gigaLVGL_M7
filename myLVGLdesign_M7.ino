#include <Arduino.h>
#include <RPC.h>
#include <Arduino_H7_Video.h>
#include <Arduino_CAN.h>
#include <Arduino_GigaDisplay.h>
#include <Arduino_GigaDisplayTouch.h>
#include "lvgl.h"

Arduino_H7_Video Display(800, 480, GigaDisplayShield);
Arduino_GigaDisplayTouch TouchDetector;

//Create backlight object
GigaDisplayBacklight backlight;

// Define relay pins
#define RELAY1 10   // inverter
#define RELAY2 11   // ceiling heater
#define RELAY3 12   // water heater
#define RELAY4 13   // shower room

// ADDING 4-BIT FONTS WITH ONLY NEEDED CHARACTERS FROM MONTSERRAT 34 & 20 AND FONTAWESOME_SVG_SOLID 20 (https://lvgl.io/tools/fontconverter)
LV_FONT_DECLARE(Montserrat34_0_9_percent);
LV_FONT_DECLARE(Montserrat20_0_9_W_minus);
LV_FONT_DECLARE(FontAwesomeIcons);


//  CANBUS data Identifier List
//  ID 0x03B BYT0+1:INST_VOLT BYT2+3:INST_AMP BYT4+5:ABS_AMP BYT6:SOC **** ABS_AMP from OrionJr errendous ****
//  ID 0x6B2 BYT0+1:LOW_CELL BYT2+3:HIGH_CELL BYT4:HEALTH BYT5+6:CYCLES
//  ID 0x0A9 BYT0:RELAY_STATE BYT1:CCL BYT2:DCL BYT3+4:PACK_AH BYT5+6:AVG_AMP
//  ID 0x0BD BYT0+1:BMS_FAULTS BYT2:HI_TMP BYT3:LO_TMP BYT4:CUSTOM_FLAGS BYT5:BMS_STATUS
//  ID 0x0BE BYT0:HI_CL_ID BYT1:LO_CL_ID BYT2:INT_HEATSINK BYT3+4:MIN_CELL BYT5+6:MAX_CELL

// Temp and rhdity data struct from M4
struct SensorData {
    float temp1;
    float temp2;
    float temp3;
    float temp4;
    float rh1;
    float rh2;
    float rh3;
    float rh4;
    float avg_temp;

    MSGPACK_DEFINE_ARRAY(temp1, temp2, temp3, temp4, rh1, rh2, rh3, rh4, avg_temp);
};

// CanData struct
struct CanData {

    int p = 0;                  // watt calculated by script

    float instU = 0;            // Voltage - multiplied by 10
    float instI = 0;            // Current - multiplied by 10 - negative value indicates charge
    float avgI = 0;             // Average current for clock and sun symbol calculations
    //float absI;             // Absolute Current NOT WORKING CORRECTLY
    float ah = 0;               // Amp hours
    float hC = 0;               // High Cell Voltage in 0,0001V
    float lC = 0;               // Low Cell Voltage in 0,0001V
    float minC = 0;             // Minimum Allowed cell voltage
    float maxC = 0;             // Maximum Allowed cell voltage

    byte soc = 0;               // State of charge - multiplied by 2
    byte hT = 0;                // Highest cell temperature
    byte lT = 0;                // Lowest cell temperature
    byte ry = 0;                // Relay status
    byte dcl = 0;               // Discharge current limit
    byte ccl = 0;               // Charge current limit
    byte h = 0;                 // Health
    byte hCid = 0;              // High Cell ID
    byte lCid = 0;              // Low Cell ID

    uint16_t fu = 0;            // BMS Faults
    uint16_t st = 0;            // BMS Status
    int cc = 0;                 // Total pack cycles (int to avoid overrun issues with uint16_t)

    byte hs = 0;                // Internal Heatsink
    byte cu = 0;                // BMS custom flag currently used for sending charge power enabled and MPO#1 state (used for tripping chg enabled relay)
    
    MSGPACK_DEFINE_ARRAY(instU, instI, soc, hC, lC, h, fu, hT, lT, ah, ry, dcl, ccl, st, cc, avgI, hCid, lCid, p, hs, cu);
};

// Create combined struct with sensor and can data
struct CombinedData {
  SensorData sensorData;
  CanData canData;
};

// Can Message Data struct
struct CanMsgData {
  // receive settings
  uint32_t rxId;
  uint8_t len;
  uint8_t rxBuf[8];

  // send settings
  static const uint8_t CAN_ID = 0x002; // CAN id for the message (constant)
  uint8_t msg_data[3]; // 2 bytes used in BMS for MPO#2 and MPO#1 respectively
  uint8_t msg_cnt;

  bool send_byte1 = false;
  bool send_byte0 = false;
  bool send_byte2 = false;

  // Constructor to initialize non const values
  CanMsgData() : rxId(0), len(0), msg_data{}, msg_cnt(0) {}
};

// Type defined structure for bms status messages allowing it to be passed to function
typedef struct {
    lv_obj_t *parent = NULL;
    lv_obj_t *title_label = NULL;
    lv_obj_t *button = NULL;
    lv_obj_t *status_label[33];
    lv_timer_t *timer = NULL;
    uint8_t y = NULL;
} bms_status_data_t;

// define struct for function user-data
typedef struct { // typedef used to not having to use the struct keyword for declaring struct variable
  lv_obj_t *button;
  lv_obj_t *dcl_label;
  lv_obj_t *label_obj;
  lv_obj_t *msgbox;
  lv_timer_t *timer;
  uint8_t relay_pin = 0;
  uint8_t y_offset = 0;
  unsigned long timeout_ms = 0;
  uint8_t dcl_limit = 0;
  uint8_t set_temp = 20; // should match value of dropdown default index
  bool on = false; // simplifies code by substituting [lv_obj_has_state(data->button, LV_STATE_CHECKED)]
  bool disabled = false; // used by thermostat to prevent dcl_check from enabling disabled buttons
} user_data_t;

typedef struct {
  lv_obj_t *clock_label;
} clock_data_t;

typedef struct {
  lv_obj_t *parent;
  lv_obj_t *label_obj;
  lv_obj_t *msgbox;
  lv_timer_t *timer;
} can_msgbox_data_t;

typedef struct {
  lv_obj_t *soc_arc = NULL;
  lv_obj_t *charge_arc = NULL;
  lv_obj_t *discharge_arc = NULL;
  lv_obj_t *battery_label = NULL;
  lv_obj_t *soc_label = NULL;
  lv_obj_t *volt_label = NULL;
  lv_obj_t *amps_label = NULL;
  lv_obj_t *watt_label = NULL;
  lv_obj_t *ah_label = NULL;
  lv_obj_t *charge_icon = NULL;
  //lv_obj_t *arrow_icon = NULL;
  lv_obj_t *car_battery_icon = NULL;
  lv_obj_t *charge_label = NULL;
  lv_obj_t *load_label = NULL;
} data_display_t;

//Initialise structures
static CanMsgData canMsgData;
static bms_status_data_t bmsStatusData;
static user_data_t userData[4] = {}; // 4 buttons with user_data
static clock_data_t clockData;
static can_msgbox_data_t canMsgBoxData;
static data_display_t dataDisplay;
static CombinedData combinedData;

// Macro short-hands that are free
#define CAN_RX        canMsgData.rxBuf
#define TX_MSG        canMsgData.msg_data
#define TX_CLR_BMS    canMsgData.send_byte0
#define TX_TRIP_PV    canMsgData.send_byte1
#define TX_BALANCING  canMsgData.send_byte2
#define AVG_TEMP      combinedData.sensorData.avg_temp
#define TEMP1         combinedData.sensorData.temp1
#define TEMP2         combinedData.sensorData.temp2
#define TEMP3         combinedData.sensorData.temp3
#define TEMP4         combinedData.sensorData.temp4
#define RH1           combinedData.sensorData.rh1
#define RH2           combinedData.sensorData.rh2
#define RH3           combinedData.sensorData.rh3
#define RH4           combinedData.sensorData.rh4
#define WATTS         combinedData.canData.p
#define VOLT          combinedData.canData.instU
#define AMPS          combinedData.canData.instI
#define AVG_AMPS      combinedData.canData.avgI
#define AH            combinedData.canData.ah
#define HI_CELL_V     combinedData.canData.hC
#define LO_CELL_V     combinedData.canData.lC
#define MAX_CELL_V    combinedData.canData.maxC
#define MIN_CELL_V    combinedData.canData.minC
#define SOC           combinedData.canData.soc
#define HI_TEMP       combinedData.canData.h
#define LO_TEMP       combinedData.canData.lT
#define RELAYS        combinedData.canData.ry
#define DCL           combinedData.canData.dcl
#define CCL           combinedData.canData.ccl
#define HEALTH        combinedData.canData.h
#define HI_CELL_ID    combinedData.canData.hCid
#define LO_CELL_ID    combinedData.canData.lCid
#define BMS_FAULTS    combinedData.canData.fu
#define BMS_STATUS    combinedData.canData.st
#define CYCLES        combinedData.canData.cc
#define HEAT_SINK     combinedData.canData.hs
#define CUSTOM_FLAGS  combinedData.canData.cu

// global variables * 8bits=256 16bits=65536 32bits=4294967296 (millis size) int/float = 4 bytes
int16_t inverter_prestart_p = 0; // must be signed
const uint8_t inverter_standby_p = 75; // takes around 4 minutes to settle down after start
uint8_t pwr_demand = 0;
const uint32_t hot_water_interval_ms = 900000; // 15 min
const uint16_t inverter_startup_delay_ms = 25000; // 25s startup required before comparing current flow for soft start appliances
const uint8_t off_interval_min = 3; // 3 minute off interval reduces standby consumption from 85Wh to around 12,5Wh -84%

static lv_style_t underline;
static lv_style_t medium_font;

static uint8_t brightness = 70;
uint32_t previous_touch_ms = 0;
const uint16_t touch_timeout_ms = 30000; // 30s before screen dimming

// for M4 messages
static String buffer = "";

//******************************************************************************************************
//  BUG#1:  CRASH WHEN DCL HIT 0 BUT INVERTER RELAY OPENED
//  BUG#2:  OCCASIONAL CRASH DUE TO COLORING OF BATTERY ICON ?? SEEMS TO HAPPEN WITH CHANGE FROM CHARGE TO BATTERY OP
//  BUG#3:  BUTTON SHOWING WHEN BALANCING ONLY AFTER SENDING BALANCING ALLOWED SIGNAL
//******************************************************************************************************

// CREATE BUTTONS /// TWO TIMERS CREATED HERE: TEMP UPDATER AND DCL CHECK
void create_button(lv_obj_t *parent, const char *label_text, uint8_t relay_pin, lv_coord_t y_offset, uint8_t dcl_limit, unsigned long timeout_ms, user_data_t *data) {

  // INITIALISE RELAY PINS
  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, LOW); // initialise pin LOW

  // INITIALISE STRUCT DATA
  data->relay_pin = relay_pin;
  data->y_offset = y_offset;
  data->dcl_limit = dcl_limit;
  data->timeout_ms = timeout_ms;

  // CREATE BUTTON
  data->button = lv_btn_create(parent);
    lv_obj_set_pos(data->button, 10, y_offset);
    lv_obj_t *label = lv_label_create(data->button);
    lv_label_set_text(label, label_text);
    lv_obj_center(label);
    lv_obj_add_flag(data->button, LV_OBJ_FLAG_CHECKABLE); // MAKE THE BUTTON STAY PRESSED UNTIL RELEASED

  // ADD DCL TIMER AND LABEL
  data->dcl_label = lv_label_create(lv_obj_get_parent(data->button));
    lv_label_set_long_mode(data->dcl_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_label_set_text(data->dcl_label, "Battery Low Power                    Please Charge                     ");
    lv_obj_set_width(data->dcl_label, 140);
    lv_obj_align_to(data->dcl_label, data->button, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);
    lv_obj_add_flag(data->dcl_label, LV_OBJ_FLAG_HIDDEN); // hide label initially
  
  // CREATE TIMER TO CHECK BMS DCL LIMIT FROM CANBUS
  lv_timer_create(dcl_check, 1000, data);

  // ADD EVENT HANDLER, LABELS AND UPDATE TIMER TO THERMOSTATIC HEATER BUTTONS
  if ( ! timeout_ms ) {
    lv_obj_add_event_cb(data->button, thermostat_event_handler, LV_EVENT_ALL, data);
    data->label_obj = lv_label_create(lv_obj_get_parent(data->button));
      lv_obj_set_width(data->label_obj, 80);
      lv_obj_set_pos(data->label_obj, 160, data->y_offset + 13);
      lv_obj_set_style_text_align(data->label_obj, LV_TEXT_ALIGN_CENTER, 0);

    // CREATE TIMER TO UPDATE TEMPERATURE FROM M4-CORE MANAGED DHT22 SENSORS
    lv_timer_create(update_temp, 10000, data);

    // INITIALISE LABEL TEXT
    lv_label_set_text(data->label_obj, LV_SYMBOL_REFRESH);

    // ADD EVENT HANDLER TO TEMPERATURE INDICATOR
    lv_obj_add_flag(data->label_obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(data->label_obj, sensor_msgbox, LV_EVENT_CLICKED, data);

    // CREATE TEMPERATURE SELECTION DROP DOWN MENU
    create_temperature_dropdown(parent, data);
  }
     
  // ADD EVENT HANDLER FOR HOT WATER AND INVERTER BUTTONS
  else {
    lv_obj_add_event_cb(data->button, hot_water_inverter_event_handler, LV_EVENT_ALL, data);

    // INVERTER LABEL
    if ( relay_pin == RELAY1 ) {
      data->label_obj = lv_label_create(lv_obj_get_parent(data->button));
      lv_obj_set_width(data->label_obj, 120);
      lv_obj_align_to(data->label_obj, data->button, LV_ALIGN_OUT_RIGHT_MID, 80, 0);

      // INITIALISE LABEL TEXT
      lv_label_set_text(data->label_obj, "OFF");
    }
  }
}



// MPPT DELAYER ////////////////////////////////////////////////////////////////////////////////
void mppt_delayer(bool mppt_delay) {
  static uint32_t start_time_ms = 0;

  // THIS FUNCTION IS CONTINIOUSLY CALLED BY SUNRISE_DETECTOR AND WILL SET ARG TO FALSE IF NEEDED

  // IF TRUE ARGUMENT: START TIMER TO ALLOW MPPT TO REMAIN OFF FOR AT LEAST 20s
  if ( mppt_delay && ! start_time_ms ) {
    start_time_ms = millis();
  }
  // IF FALSE AGRUMENT AND TIMER WITHIN PREDETERMINED PERIOD: SET TRUE
  else if ( ! mppt_delay && start_time_ms && (start_time_ms + 20 * 1000) > millis() ) {
    mppt_delay = true;
  }
  // IF TRUE BUT WITH TIMER OR IF FALSE WITH EXPIRED OR WITHOUT TIMER: RESET TIMER FOR NEXT FUNCTION CALL
  else {
    start_time_ms = 0;
  }

  switch ( mppt_delay ) {
    case true:
      TX_MSG[1] = 0x01;
      TX_TRIP_PV = true; // this triggers send function in loop
      Serial.println("DEBUG mppt_delayer called to disable pv charge");
      break;
    case false:
      TX_TRIP_PV = false;
      TX_MSG[1] = 0x00;
   }
}



// SUNRISE DETECTOR ////////////////////////////////////////////////////////////////////////
void sunrise_detector(lv_timer_t *timer) {

  static bool mppt_delay = false;
  static uint32_t time_ms = 0;

  // IS SOLAR CHARGE SIGNAL AVAILABLE THROUGH CUSTOM FLAG AND TIMER NOT RUNNING
  if ( (CUSTOM_FLAGS & 0x0001) == 0x0001 && ! time_ms ) {

    // IN LOW SUN CONDITIONS MPPT DRAINS BATTERY INSTEAD OF CHARGING. IF THIS HAPPENS WHEN INVERTER IS OFF, DISABLE MPPT
    if ( ! userData[3].on && WATTS > 10 ) {
      mppt_delay = true;
    }
    // START TIME AND RETURN FOR ANOTHER ROUND
    else {
      time_ms = millis();
      return;
    }
  }

  // IF SOLAR CHARGE SIGNAL IS LOST
  else if ( ! (CUSTOM_FLAGS & 0x0001) == 0x0001 && time_ms && ! mppt_delay ) {

    // within 10 seconds lets trigger mppt delay as relay flap detected
    if ( time_ms + 10000 > millis() ) {
      mppt_delay = true;
    }

    // more than 10s later e.g. sunset
    else {
      time_ms = 0;
      return;
    }
  }

  // when mppt delay timer has expired - currently after 10 minutes
  if ( mppt_delay && (time_ms + 10 * 60 * 1000) < millis() ) {
    time_ms = 0;
    mppt_delay = false;
  }

  // Send signal to mppt_delayer function
  mppt_delayer(mppt_delay);
}






// CCL AND HIGH CELL VOLTAGE CHECK TIMER ////////////////////////////////////////////////////////////////////////////
void ccl_check(lv_timer_t * timer) {

  if ( CCL == 0 || HI_CELL_V > (MAX_CELL_V - 0.2) ) {
    TX_MSG[1] = 0x01;
    TX_TRIP_PV = true; // trip pv charge relay
    Serial.println("DEBUG ccl_check sending trip pv signal");
  }
  else {
    TX_TRIP_PV = false;
    TX_MSG[1] = 0x00;
  }
}









// TURN OFF BUTTON FUNCTION //////////////////////////////////////////////////////////////////////////////////////
void button_off(user_data_t *data) {
  digitalWrite(data->relay_pin, LOW);
  data->on = false;
  if ( data->timer ) {
    lv_timer_del(data->timer);
    data->timer = NULL; // clear pointer
  }
  Serial.println("DEBUG button off function ran");
}









// DCL AND LOW CELL VOLTAGE CHECK TIMER ////////////////////////////////////////////////////////////////////////////
void dcl_check(lv_timer_t * timer) {
  user_data_t * data = (user_data_t *)timer->user_data;

  // IF BUTTON DISABLED ELSEWHERE SKIP THIS FUNCTION
  if ( data->disabled ) {
    return;
  }

  // IF DCL IS ZERO OR CELL VOLTAGE TOO LOW AND NOT ALREADY DISABLED: LABEL INVERTER ONLY AND TURN OFF AND DISABLE ALL BUTTONS
  else if ( DCL == 0 || LO_CELL_V < (MIN_CELL_V + 0.3) || (RELAYS & 0x01) != 0x01 && ! lv_obj_has_state(userData[3].button, LV_STATE_DISABLED) ) {
    for ( uint8_t i = 0; i < 4; i++ ) {
      if ( userData[i].on ) {
        lv_event_send(userData[i].button, LV_EVENT_RELEASED, NULL);
        // CHECK IF BUTTON IS STILL ON
        if ( userData[i].on ) {
          button_off(&userData[i]);
          Serial.println("DEBUG DCL_CHECK: LV_EVENT_SEND failed to turn off buttons");
        }
      }
      // DISABLE ALL BUTTONS IF NOT ALREADY DONE
      if ( ! lv_obj_has_state(userData[i].button, LV_STATE_DISABLED) ) {
        lv_obj_add_state(userData[i].button, LV_STATE_DISABLED);
      }
    }
    // CLEAR HIDDEN FLAG ON INVERTER TO MAKE IT VISIBLE
    if ( lv_obj_has_flag(userData[3].dcl_label, LV_OBJ_FLAG_HIDDEN) ) {
      lv_obj_clear_flag(userData[3].dcl_label, LV_OBJ_FLAG_HIDDEN);
    }
    // SET INVERTER LABEL TEXT
    if ( lv_label_get_text(userData[3].label_obj) != "OFF" ) {
      lv_label_set_text(userData[3].label_obj, "OFF");
    }
  }
  // INDIVIDUAL BUTTON LIMITS IF NOT ALREADY DISABLED
  else if ( DCL < data->dcl_limit && ! lv_obj_has_state(data->button, LV_STATE_DISABLED) ) {
    // SHOW LABEL
    if ( lv_obj_has_flag(data->dcl_label, LV_OBJ_FLAG_HIDDEN) ) {
      lv_obj_clear_flag(data->dcl_label, LV_OBJ_FLAG_HIDDEN); // clear hidden flag to show
    }
    // TURN OFF BUTTON
    if ( data->on ) {
      lv_event_send(data->button, LV_EVENT_RELEASED, NULL);
      // CHECK IF BUTTON IS STILL ON
      if ( data->on ) {
        Serial.println("DEBUG DCL_CHECK: LV_EVENT_SEND failed to turn off individual button");
        button_off(data);
      }
      // UPDATE INVERTER LABEL
      if ( data->relay_pin == RELAY1 && lv_label_get_text(data->label_obj) != "OFF" ) {
        lv_label_set_text(data->label_obj, "OFF");
      }
    }
    // DISABLE BUTTON - TEST PERFORMED ALREADY
    lv_obj_add_state(data->button, LV_STATE_DISABLED);
  }

  // IF NO DCL LIMIT AND DISCHARGE RELAY CLOSED HIDE FLAG AND ENABLE BUTTONS
  else if ( lv_obj_has_state(data->button, LV_STATE_DISABLED) && (RELAYS & 0x01) ) {
    Serial.println("DEBUG DCL_CHECK: Enabling button");
    if ( ! lv_obj_has_flag(data->dcl_label, LV_OBJ_FLAG_HIDDEN) ) {
      lv_obj_add_flag(data->dcl_label, LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_clear_state(data->button, LV_STATE_DISABLED);
  }
}





// MESSAGE BOX FOR CAN-DATA EVENT HANDLERS AND UPDATE TIMER ////////////////////////////////////
const char* set_can_msgbox_text() {
  static char msgbox_text[512]; // Static buffer to retain the value

  snprintf(msgbox_text, sizeof(msgbox_text),
                 "High Cell          %.2fV           #%d\n"
                 "Low Cell           %.2fV           #%d\n\n"
                 "Charge Limit                      %d A\n"
                 "Discharge Limit                %d A\n\n"
                 "Charge                                    %s\n"
                 "Discharge                               %s\n\n"
                 "Cycles                                     %d\n"
                 "Health                                %d%%\n\n"
                 "BMS Heatsink                   %d\u00B0C\n"
                 "Energy                          %.2f kW",
                 HI_CELL_V, HI_CELL_ID,
                 LO_CELL_V, LO_CELL_ID,
                 CCL,
                 DCL,
                 (CUSTOM_FLAGS & 0x0002) == 0x0002 ? LV_SYMBOL_OK : LV_SYMBOL_CLOSE, // using MPO#1 feedback from BMS which controls both charge FETs
                 (RELAYS & 0x0001) == 0x0001 ? LV_SYMBOL_OK : LV_SYMBOL_CLOSE, // using BMS relay state
                 //(RELAYS & 0x0004) == 0x0004 ? LV_SYMBOL_OK : LV_SYMBOL_CLOSE, // this relay is controlled by charge enabled signal from pv panel
                 CYCLES,
                 HEALTH,
                 HEAT_SINK,
                 (AH * VOLT) / 1000.0);

  return msgbox_text;
}

void can_msgbox_update_timer(lv_timer_t *timer) {
  can_msgbox_data_t *data = (can_msgbox_data_t*)timer->user_data;
  lv_obj_t *label = lv_msgbox_get_text(data->msgbox); // get msgbox text string object excluding title
  lv_label_set_text(label, set_can_msgbox_text()); // Update the text object in the msgBox
}

void can_msgbox(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    can_msgbox_data_t *data = (can_msgbox_data_t*)lv_event_get_user_data(e);
    
    if (code == LV_EVENT_CLICKED) {
        data->msgbox = lv_msgbox_create(data->parent, "     Battery Monitoring Data", set_can_msgbox_text(), NULL, false);
        lv_obj_set_width(data->msgbox, LV_PCT(80)); // Set width to 80% of the screen
        lv_obj_align(data->msgbox, LV_ALIGN_CENTER, 0, 0); // Center the message box on the screen

        // Create a full-screen overlay to detect clicks for closing the message box
        lv_obj_t *overlay = lv_obj_create(lv_scr_act());
        lv_obj_set_size(overlay, LV_HOR_RES, LV_VER_RES);
        lv_obj_set_style_opa(overlay, LV_OPA_TRANSP, 0); // Transparent overlay
        lv_obj_add_event_cb(overlay, close_can_msgbox_event_handler, LV_EVENT_CLICKED, data);

        // Create timer to update data every 10 seconds
        data->timer = lv_timer_create(can_msgbox_update_timer, 10000, data);

        // Pause BMS status data label timer as they show through msgbox
        lv_timer_pause(bmsStatusData.timer);
    }
}

void close_can_msgbox_event_handler(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  can_msgbox_data_t *data = (can_msgbox_data_t*)lv_event_get_user_data(e);

  if ( code == LV_EVENT_CLICKED) {
    lv_timer_del(data->timer); // Delete the timer
    data->timer = NULL;
    lv_msgbox_close(data->msgbox);           // Delete the message box

    // Remove the screen overlay object
    lv_obj_del(lv_event_get_current_target(e));

    // Resume BMS status data label timer
    lv_timer_resume(bmsStatusData.timer);
  }
}





// MESSAGE BOX FOR SENSOR-DATA EVENT HANDLERS AND UPDATE TIMER /////////////////////////////////
const char* set_sensor_msgbox_text() {
  static char msgbox_text[327]; // Static buffer to retain the value

  snprintf(msgbox_text, sizeof(msgbox_text),
                 "Temperature 1:            %.1f°C\nRelative Humidity 1:   %.1f%%\n\n"
                 "Temperature 2:            %.1f°C\nRelative Humidity 2:   %.1f%%\n\n"
                 "Temperature 3:            %.1f°C\nRelative Humidity 3:   %.1f%%\n\n"
                 "Temperature 4:            %.1f°C\nRelative Humidity 4:   %.1f%%",
                 TEMP1, RH1,
                 TEMP2, RH2,
                 TEMP3, RH3,
                 TEMP4, RH4);

  return msgbox_text;
}

void sensor_msgbox_update_timer(lv_timer_t *timer) {
    user_data_t *data = (user_data_t*)timer->user_data;
    lv_obj_t *label = lv_msgbox_get_text(data->msgbox); // get msgBox text string object excluding title
    lv_label_set_text(label, set_sensor_msgbox_text()); // Update the text object in the msgBox
}

void sensor_msgbox(lv_event_t *e) {
    user_data_t *data = (user_data_t*)lv_event_get_user_data(e);
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
        data->msgbox = lv_msgbox_create(lv_obj_get_parent(data->label_obj), "               Sensor Data", set_sensor_msgbox_text(), NULL, false);
        lv_obj_set_width(data->msgbox, LV_PCT(80)); // Set width to 80% of the screen
        lv_obj_align(data->msgbox, LV_ALIGN_CENTER, 0, 0); // Center the message box on the screen

        // Create a full-screen overlay to detect clicks for closing the message box
        lv_obj_t *overlay = lv_obj_create(lv_scr_act());
        lv_obj_set_size(overlay, LV_HOR_RES, LV_VER_RES);
        lv_obj_set_style_opa(overlay, LV_OPA_TRANSP, 0); // Transparent overlay
        lv_obj_add_event_cb(overlay, close_sensor_msgbox_event_handler, LV_EVENT_CLICKED, data);

        // Create timer to update data every 10 seconds
        data->timer = lv_timer_create(sensor_msgbox_update_timer, 10000, data);
    }
}

void close_sensor_msgbox_event_handler(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  user_data_t *data = (user_data_t*)lv_event_get_user_data(e);

  if ( code == LV_EVENT_CLICKED) {
    lv_timer_del(data->timer); // Delete the timer
    data->timer = NULL;
    lv_msgbox_close(data->msgbox);           // Delete the message box

    // Remove the screen overlay object
    lv_obj_del(lv_event_get_current_target(e));
  }
}






// HOT WATER AND INVERTER EVENT HANDLER ////////////////////////////////////////////////////
void hot_water_inverter_event_handler(lv_event_t *e) {
  user_data_t * data = (user_data_t *)lv_event_get_user_data(e);
  lv_event_code_t code = lv_event_get_code(e);
  if(code == LV_EVENT_CLICKED) {

    // Button ON
    if ( lv_obj_has_state(data->button, LV_STATE_CHECKED) ) {

      // Inverter
      if ( data->relay_pin == RELAY1 ) {
        inverter_prestart_p = WATTS;
        // Turn off mppt if pv relat closed but no charge detected to avoid startup issues
        if ( inverter_prestart_p >= 0 && (inverter_standby_p + inverter_prestart_p) > WATTS && (CUSTOM_FLAGS & 0x0002) == 0x0002 ) {
          mppt_delayer(true); // as sunrise_detector calls mppt_delayer every 1s there's no need to call with false
          delay(700); // allowing for mppt to loose power
        }
        lv_label_set_text(data->label_obj, "Inverter ON");
      }

      // Hot water - try to start inverter if it's off
      else if ( userData[3].on == false ) { // ( lv_obj_has_state(userData[3].button, LV_STATE_CHECKED) == false ) {
        lv_event_send(userData[3].button, LV_EVENT_PRESSED, NULL); // Have to include all 3 of these to make it work
        lv_event_send(userData[3].button, LV_EVENT_RELEASED, NULL);
        lv_event_send(userData[3].button, LV_EVENT_CLICKED, NULL);
        // If inverter doesn't start clear hot water button clicked state
        if ( userData[3].on == false ) {
          lv_obj_clear_state(data->button, LV_STATE_CHECKED);
          return; // exit function if inverter doesn't start
        }
        // If started successfully increment pwr demand
        else pwr_demand++;
      }

      // If inverter is ON already, only increment pwr demand
      else pwr_demand++; // only for hot water

      // Send signal to relay
      digitalWrite(data->relay_pin, HIGH);
      data->on = true; // store state

      // Store delay timer for inverter search and for hot water timeout in struct to allow for deletion elsewhere
      if ( data->timer ) {
        lv_timer_del(data->timer);
        data->timer = NULL;
        Serial.println("DEBUG inverter/hot water evt hdlr: deleting timer");
      }
      // CREATE COMBINED TIMER
      data->timer = lv_timer_create(power_check, data->timeout_ms, data);
    }

    // Button OFF
    else {
      button_off(data); // deletes timer, set digital pin LOW and sets on variable to false

      // inverter needs to manipulate the other buttons
      if ( data->relay_pin == RELAY1 ) {
        lv_label_set_text(data->label_obj, "OFF");
        // turn off the other buttons
        for ( uint8_t i = 0; i < 3; i++ ) {
          if ( userData[i].on == true ) {
            lv_event_send(userData[i].button, LV_EVENT_RELEASED, NULL); // release button
            button_off(&userData[i]);
          }
        }
        pwr_demand = 0;
        inverter_prestart_p = 0;
      }

      // hot water
      else {
        pwr_demand ? pwr_demand-- : NULL;
      }
    }
  }
}


// POWER CHECK TIMER ///////////////////////////////////////////////////////////////////////
void power_check(lv_timer_t * timer) {
  user_data_t * data = (user_data_t *)timer->user_data;

  // Inverter
  if ( data->relay_pin == RELAY1 ) {

    // Remain ON if charging when SOC above 50%
    if ( AVG_AMPS < -5 && SOC > 50 ) {
      return;
    }

    // Remain ON if demand variable set
    else if ( pwr_demand ) {
      return;
    }

    // Remain ON if discharge exceeds inverter standby - considering prestart_p and canData.p are signed it should cover most charge/discharge scenarios
    else if ( (inverter_standby_p + inverter_prestart_p) < abs(WATTS) ) {
      return;
    }
  }
  
  // Hot water - reset timer if charging
  else if ( CCL < 10 && AVG_AMPS < 0 ) {
    return;
  }

  // INVERTER OFF INTERVAL START
  else if ( data->relay_pin == RELAY1 && data->on == true ) {
    static uint32_t time_ms = 0;
    static uint8_t minute_count = 0;
    char plural[2] = "s";
    char label[30];

    // INVERTER OFF AND LABEL UPDATER ALGORITHM
    if ( ! time_ms ) {
      time_ms = millis();
      digitalWrite(data->relay_pin, LOW);
    }
    else if ( (time_ms + (1 + minute_count) * 60 * 1000) < millis() && minute_count < off_interval_min ) {
      minute_count++;
      if ( minute_count == 2 ) {
        strcpy(plural, "");
      }
    }
    else if ( (minute_count + 1) == off_interval_min ) {
      minute_count = 0;
      time_ms = 0;
      data->on = false; // to enable inverter startup check
      lv_event_send(data->button, LV_EVENT_CLICKED, NULL);
      return; // to prevent label being written once finished
    }

    snprintf(label, sizeof(label), "OFF - NO LOAD\nON in %d minute%s", (off_interval_min - minute_count), plural);
    lv_label_set_text(data->label_obj, label);
  }

  // Hot water off
  else {
    lv_obj_clear_state(data->button, LV_STATE_CHECKED);
    button_off(data);
  }
}






// THERMOSTAT TIMER ////////////////////////////////////////////////////////////////
void thermostat_timer(lv_timer_t * timer) {
  user_data_t * data = (user_data_t *)timer->user_data;

  static uint32_t thermostat_off_ms = 0;
  bool on = false;

  // Off cycle time checker (2 min set)
  if ( thermostat_off_ms && (thermostat_off_ms + 120000) > millis() ) {
    return;
  }

  // Ceiling heater thermostat ( uses 3 or 1 sensors )
  else if ( data->relay_pin == RELAY2 ) {
    // need to check which sensor is working ( if none the temp updater will disable button )
    if ( AVG_TEMP != 999.0f && AVG_TEMP < data->set_temp ) {
      on = true;
    }
    else if ( TEMP1 != 999.0f && TEMP1 < data->set_temp ) {
      on = true;
    }
    else if ( TEMP2 != 999.0f && TEMP2 < data->set_temp ) {
      on = true;
    }
    else if ( TEMP3 != 999.0f && TEMP4 < data->set_temp ) {
      on = true;
    }

    // Open relays when temperature is higher or equal to selected
    else {
      on = false;
    }
  }

  // Shower heater thermostat
  else {
    if ( TEMP3 < data->set_temp ) {
      on = true;
    }

    // Open relays when temperature is higher or equal to selected
    else {
      on = false;
    }
  }

  // Toggle relay and set timer
  if ( on ) {
    digitalWrite(data->relay_pin, HIGH);
    thermostat_off_ms = 0;
    data->on = true;
  }
  else {
    digitalWrite(data->relay_pin, LOW);
    thermostat_off_ms = millis();
    data->on = false;
  }
}

// THERMOSTAT EVENT HANDLER /////////////////////////////////////////////////////////
void thermostat_event_handler(lv_event_t * e) {
  user_data_t * data = (user_data_t *)lv_event_get_user_data(e);
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_CLICKED) {

    // Button ON
    if ( lv_obj_has_state(data->button, LV_STATE_CHECKED) ) {
      // check if inverter is on
      if ( userData[3].on == false ) {
        lv_event_send(userData[3].button, LV_EVENT_PRESSED, NULL);
        lv_event_send(userData[3].button, LV_EVENT_RELEASED, NULL);
        lv_event_send(userData[3].button, LV_EVENT_CLICKED, NULL);
        // DEBUG if inverter is still off disable change flag
        if ( userData[3].on == false ) {
          lv_obj_clear_state(data->button, LV_STATE_CHECKED);
          return; // exit function if inverter is off
        }
      }
      data->on = true;
      data->timer = lv_timer_create(thermostat_timer, 10000, data); // check temp diff every 10s
      pwr_demand++;
    }

    // Button OFF
    else {
      data->on = false;
      button_off(data);
      pwr_demand ? pwr_demand-- : NULL;
    }
  }
}










// TEMPERATURE DROP DOWN EVENT HANDLER ////////////////////////////////////////////////
void dropdown_event_handler(lv_event_t *e) {
    user_data_t * data = (user_data_t *)lv_event_get_user_data(e);
    lv_obj_t * dd = lv_event_get_target(e);

    // get index of selected dropdown item
    uint8_t id_selected = lv_dropdown_get_selected(dd);

    // link index to selection
    switch (id_selected) {
      case 0:
        data->set_temp = 5;
        break;
      case 1:
        data->set_temp = 18;
        break;
      case 2:
        data->set_temp = 19;
        break;
      case 3:
        data->set_temp = 20;
        break;
      case 4:
        data->set_temp = 21;
        break;
      case 5:
        data->set_temp = 22;
        break;
      default:
        data->set_temp = 23;
    }
}

// CREATE TEMPERATURE SELECTION DROPDOWN MENU ///////////////////////////////////////
void create_temperature_dropdown(lv_obj_t * parent, user_data_t *data) {
  lv_obj_t *dd = lv_dropdown_create(parent);
  lv_dropdown_set_options(dd,
    "5\u00B0C\n18\u00B0C\n19\u00B0C\n20\u00B0C\n21\u00B0C\n22\u00B0C\n23\u00B0C");
    
  // set user data
  lv_dropdown_set_selected(dd, 3); // default index to be displayed. value set_temp in struct
  lv_obj_set_user_data(dd, data);
  lv_obj_add_event_cb(dd, dropdown_event_handler, LV_EVENT_VALUE_CHANGED, data);
  
  // place roller
  lv_obj_set_pos(dd, 235, data->y_offset - 1);
  lv_obj_set_width(dd, 80);
}












// TEMP SENSOR FAULT LABEL MAKER /////////////////////////////////////////////////////////////////
void fault_label_maker(lv_timer_t *timer) {
  user_data_t *data = (user_data_t*)timer->user_data;

  uint8_t faultArr[3];
  uint8_t index = 0;
  char faultMsg[20];

  if (TEMP1 == 999.0f) {
    faultArr[index] = 1;
    index++;
  }
  if (TEMP2 == 999.0f) {
    faultArr[index] = 2;
    index++;
  }
  if (TEMP4 == 999.0f) {
    faultArr[index] = 4;
    index++;
  }

  if (index == 0) {
    strcpy(faultMsg, "");
    return;
  }

  strcpy(faultMsg, "#"); // Start with a #

  for (uint8_t i = 0; i < index; ++i) {
    char buffer[10]; // Buffer to hold the string representation of the number
    if (i > 0) {
      strcat(faultMsg, "+"); // Add + before each element except the first
    }
    snprintf(buffer, sizeof(buffer), "%d", faultArr[i]);
    strcat(faultMsg, buffer);
  }

   // Add whitespace followed by two dashes at the end for single sensor fault
  if ( index = 0 ) {
    strcat(faultMsg, " --");
  }

  // Update the label directly with the warning message
  lv_label_set_text(data->label_obj, faultMsg);
}

// TEMPERATURE UPDATER //////////////////////////////////////////////////////
void update_temp(lv_timer_t *timer) {
  user_data_t *data = (user_data_t *)timer->user_data;

  static lv_timer_t *sensor_fault_timer = NULL;

  char buf[20];
  data->disabled = false; // reset before run - used to prevent dcl_check from enabling disabled buttons
  bool sensor_fault = false;
  bool all_sensors_faulty = false;

  // LIVING ROOM CHECKING EACH SENSOR AND USING SINGLE WORKING SENSOR IF NO AVERAGE TEMPERATURE
  if (data->relay_pin == RELAY2) {
    if (AVG_TEMP != 999.0f) {
      snprintf(buf, sizeof(buf), "%.1f\u00B0C", AVG_TEMP);
    }
    else if (TEMP1 != 999.0f) {
      snprintf(buf, sizeof(buf), "%.1f\u00B0C", TEMP1);
      sensor_fault = true;
    }
    else if (TEMP2 != 999.0f) {
      snprintf(buf, sizeof(buf), "%.1f\u00B0C", TEMP2);
      sensor_fault = true;
    }
    else if (TEMP4 != 999.0f) {
      snprintf(buf, sizeof(buf), "%.1f\u00B0C", TEMP4);
      sensor_fault = true;
    }
    // ALL SENSORS FAULTY
    else {
      snprintf(buf, sizeof(buf), "----");
      all_sensors_faulty = true;
      data->disabled = true;
    }
    // CALL FAULT LABEL MAKER FUNCTION AT INTERVALS 8 OR 5 SEC
    if ( sensor_fault ) {
      sensor_fault_timer = lv_timer_create(fault_label_maker, 8000, data);
      lv_timer_set_repeat_count(sensor_fault_timer, 4); // 8s * 4 = 40s which matching 4 function call intervals
    }
    else if ( all_sensors_faulty ) {
      sensor_fault_timer = lv_timer_create(fault_label_maker, 5000, data);
      lv_timer_set_repeat_count(sensor_fault_timer, 2); // 5s * 2 = 10s which matches 1 function call interval
    }
    else if ( sensor_fault_timer ) {
      lv_timer_del(sensor_fault_timer);
      Serial.println("DEBUG update_temp: Deleting sensor fault timer");
    }
  }

  // CHECK SINGLE SENSOR FOR SHOWER ROOM
  else {
    if (TEMP3 != 999.0f) {
      snprintf(buf, sizeof(buf), "%.1f\u00B0C", TEMP3);
    }
    else {
      snprintf(buf, sizeof(buf), "#3 --");
      data->disabled = true;
    }
  }

  // TURN OFF BUTTON AND ADD DISABLED STATE
  if ( data->disabled && ! lv_obj_has_state(data->button, LV_STATE_DISABLED)) {
    if (data->on == true) {
      lv_event_send(data->button, LV_EVENT_RELEASED, NULL);
    }
    lv_obj_add_state(data->button, LV_STATE_DISABLED);
  }
  // CLEAR DISABLED STATE IF NOT ENFORCED BY DCL_CHECK FUNCTION
  else if ( ! data->disabled && lv_obj_has_state(data->button, LV_STATE_DISABLED) && ! lv_obj_has_state(userData[3].button, LV_STATE_DISABLED) ) {
    lv_obj_clear_state(data->button, LV_STATE_DISABLED);
    Serial.println("DEBUG clearing heater buttons disabled state");
  }

  // Update the label text
  lv_label_set_text(data->label_obj, buf);
}







// CLEAR BMS FLAG CAN MSG EVENT HANDLER ////////////////////////////////////////////////////////////////////
void clear_bms_flag(lv_event_t * e) {
  lv_event_code_t code = lv_event_get_code(e);

  if(code == LV_EVENT_CLICKED) {
    TX_CLR_BMS = true;
    TX_MSG[0] = 0x01;
    canMsgData.msg_cnt = 0; // reset the retry count
  }
  Serial.println("Sending CAN msg to clear BMS flags");
}




// CREATE CLOCK //////////////////////////////////////////////////////////////////////////////
void create_clock_label(lv_obj_t *parent, clock_data_t *data) {

  data->clock_label = lv_label_create(parent);
  lv_obj_align(data->clock_label, LV_ALIGN_TOP_MID, 0, -5); // x=20 perfect if left aligned

  // text update timer
  lv_timer_create(clock_updater, 1000, data);
}

void clock_updater(lv_timer_t *timer) {
  clock_data_t *data = (clock_data_t *)timer->user_data;

  uint16_t h = 0;
  uint8_t m = 0;
  char t[11];
  char c[4] = {"hrs"};
  char state[17];

  // Zero
  if ( AVG_AMPS == 0 ) {
    strcpy(t, "");;
  }

  // Discharge
  else if (AVG_AMPS > 0) {
    h = SOC / (AVG_AMPS/10.0);
    m = (SOC / (AVG_AMPS/10.0) - h) * 60;
    strcpy(state, "Discharged in");
  }

  // Charge
  else if (AVG_AMPS < 0) {
    h = (200 - SOC) / (abs(AVG_AMPS)/10.0);
    m = ((200 - SOC) / (abs(AVG_AMPS)/10.0) - h) * 60;
    strcpy(state, "Fully Charged in");
  }

  // Over-run prevention by showing days
  if (h > 120) {
    uint8_t d = h / 24;
    sprintf(t, "%s %d days", state, d);
  }
  else if (m || h) {
    // Plural adjustment
    if (h == 1) strcpy(c, "hr");

    sprintf(t, "%s %02d:%02d %s", state, h, m, c);
  }

  // Print
  lv_label_set_text(data->clock_label, t);
}





// Function to sign value
int16_t signValue(uint16_t canValue) {
    int16_t signedValue = (canValue > 32767) ? canValue - 65536 : canValue;
    return signedValue;
}

// Sort CAN bus data
void sort_can() {

    if (canMsgData.rxId == 0x3B) {
        VOLT = ((CAN_RX[0] << 8) + CAN_RX[1]) / 10.0;
        AMPS = (signValue((CAN_RX[2] << 8) + CAN_RX[3])) / 10.0; // orion2jr issue: unsigned value despite ticket as signed
        //combinedData.canData.absI = ((CAN_RX[4] << 8) + CAN_RX[5]) / 10.0; // orion2jr issue: set signed and -32767 to fix
        SOC = CAN_RX[6] / 2;
    }
    if (canMsgData.rxId == 0x6B2) {
        LO_CELL_V = ((CAN_RX[0] << 8) + CAN_RX[1]) / 10000.00;
        HI_CELL_V = ((CAN_RX[2] << 8) + CAN_RX[3]) / 10000.00;
        HEALTH = CAN_RX[4];
        CYCLES = (CAN_RX[5] << 8) + CAN_RX[6];
    }
    if (canMsgData.rxId == 0x0A9) {
        RELAYS = CAN_RX[0];
        CCL = CAN_RX[1];
        DCL = CAN_RX[2];
        AH = ((CAN_RX[3] << 8) + CAN_RX[4]) / 10.0;
        AVG_AMPS = (signValue((CAN_RX[5] << 8) + CAN_RX[6])) / 10.0; // orion2jr issue: unsigned value despite ticket as signed
    }
    if (canMsgData.rxId == 0x0BD) {
        BMS_FAULTS = (CAN_RX[0] << 8) + CAN_RX[1];
        HI_TEMP = CAN_RX[2];
        LO_TEMP = CAN_RX[3];
        CUSTOM_FLAGS = CAN_RX[4];
        BMS_STATUS = (CAN_RX[5] << 8) + CAN_RX[6];
    }
    if (canMsgData.rxId == 0x0BE) {
        HI_CELL_ID = CAN_RX[0];
        LO_CELL_ID = CAN_RX[1];
        HEAT_SINK = CAN_RX[2];
        MIN_CELL_V = ((CAN_RX[3] << 8) + CAN_RX[4]) / 10000.00;
        MAX_CELL_V = ((CAN_RX[5] << 8) + CAN_RX[6]) / 10000.00;
    }
    WATTS = AVG_AMPS * VOLT;
}



















// RETRIEVE DATA FROM M4 CORE //////////////////////////////////////////////////////////
void retrieve_M4_data() {
  // Call the RPC function to get sensor data
  combinedData.sensorData = RPC.call("getSensorData").as<SensorData>();
}
















// DISPLAY BRIGHTNESS DIMMING /////////////////////////////////////////////////////////
void dim_display() {
  // check if brightness above 0
  brightness ? brightness-- : NULL;
  // set brightness
  backlight.set(brightness);
}

// SCREEN TOUCH HANDLER FOR DISPLAY DIMMING ///////////////////////////////////////////
void screen_touch(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  if(code == LV_EVENT_CLICKED) {
    previous_touch_ms = millis();
    brightness = 70;
    backlight.set(brightness);
  }
}



















// CREATE STATUS LABELS ////////////////////////////////////////////////////////////
void create_status_label(const char* label_text, bms_status_data_t *data, bool finished = false) {

  static uint8_t i = 0; // static variable to preserve value between function calls

  // if finised argument is true allign button and reset index to be ready for next round of calls
  if ( finished ) {
    // Align button if it is visible
    if ( ! lv_obj_has_flag(data->button, LV_OBJ_FLAG_HIDDEN) ) {
      lv_obj_align_to(data->button, data->status_label[i-1], LV_ALIGN_OUT_BOTTOM_MID, 0, 20); // Align button below last label with index controlled gap (compensated for ++)
    }
    i = 0; // reset index for next cycle
  }

  // create label if it doesn't exist and add passed text to label object
  else {
    if ( ! data->status_label[i] ) {
      data->status_label[i] = lv_label_create(data->parent);
    }
    // add text to label index and allign vertically by index
    lv_label_set_text(data->status_label[i], label_text);
    lv_obj_align_to(data->status_label[i], data->title_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 10 + i * 20); // 10 was 15 before underline added

    // move to next index for next function call
    i++;
  }
}

// REFRESH BMS STATUS DATA ////////////////////////////////////////////////////////////////////
void refresh_bms_status_data(lv_timer_t * timer) {
    bms_status_data_t *data = (bms_status_data_t *)timer->user_data;

    static bool balancing_label_showing = false; // Controlling the flashing feature

    int8_t flag_index = -1; // initialise as -1 as this is for indexing array
    int8_t comparator_index = -1; // controls button visibility for certain messages

    // Clear all status labels initially
    for (uint8_t i = 0; i < (sizeof(data->status_label) / sizeof(data->status_label[0])); i++) {
      if (data->status_label[i]) {
        lv_obj_del(data->status_label[i]); // Properly delete label
        data->status_label[i] = NULL; // Set pointer to NULL
      }
    }
    // CRITICAL CAN TRANSMIT FAILURE
    if ( canMsgData.len == 0 ) { create_status_label("No CAN data from BMS", data); flag_index++; comparator_index++; }

    // BMS flags
    if ((BMS_FAULTS & 0x0100) == 0x0100) { create_status_label("Internal Hardware Fault", data); flag_index++; }
    if ((BMS_FAULTS & 0x0200) == 0x0200) { create_status_label("Internal Cell Comm Fault", data); flag_index++; }
    if ((BMS_FAULTS & 0x0400) == 0x0400) { create_status_label("Weak Cell Fault", data); flag_index++; }
    if ((BMS_FAULTS & 0x0800) == 0x0800) { create_status_label("Low Cell Voltage", data); flag_index++; }
    if ((BMS_FAULTS & 0x1000) == 0x1000) { create_status_label("Open Wire Fault", data); flag_index++; }
    if ((BMS_FAULTS & 0x2000) == 0x2000) { create_status_label("Current Sensor Fault", data); flag_index++; }
    if ((BMS_FAULTS & 0x4000) == 0x4000) { create_status_label("Abnormal SOC Behavior", data); flag_index++; }
    if ((BMS_FAULTS & 0x8000) == 0x8000) { create_status_label("Pack Too Hot Fault", data); flag_index++; }
    if ((BMS_FAULTS & 0x0001) == 0x0001) { create_status_label("Weak Pack Fault", data); flag_index++; }
    if ((BMS_FAULTS & 0x0002) == 0x0002) { create_status_label("External Thermistor Fault", data); flag_index++; }
    if ((BMS_FAULTS & 0x0004) == 0x0004) { create_status_label("Charge Relay Failure", data); flag_index++; }
    if ((BMS_FAULTS & 0x0008) == 0x0008) { create_status_label("Discharge Relay Fault", data); flag_index++; }
    if ((BMS_FAULTS & 0x0010) == 0x0010) { create_status_label("Safety Relay Fault", data); flag_index++; }
    if ((BMS_FAULTS & 0x0020) == 0x0020) { create_status_label("CAN communication Fault", data); flag_index++; }
    if ((BMS_FAULTS & 0x0040) == 0x0040) { create_status_label("Internal Thermistor Fault", data); flag_index++; }
    if ((BMS_FAULTS & 0x0080) == 0x0080) { create_status_label("Internal Logic Fault", data); flag_index++; }

    // Failsafe status
    if ((BMS_STATUS & 0x0001) == 0x0001) { create_status_label("Voltage Failsafe", data); flag_index++; }
    if ((BMS_STATUS & 0x0002) == 0x0002) { create_status_label("Current Failsafe", data); flag_index++; }
    if ((BMS_STATUS & 0x0004) == 0x0004) { create_status_label("Relay Failsafe", data); flag_index++; }
    if ((BMS_STATUS & 0x0010) == 0x0010) { create_status_label("Charge Interlock Failsafe", data); flag_index++; }
    if ((BMS_STATUS & 0x0020) == 0x0020) { create_status_label("Thermistor B-value Table Invalid", data); flag_index++; }
    if ((BMS_STATUS & 0x0040) == 0x0040) { create_status_label("Input Power Supply Failsafe", data); flag_index++; }
    if ((BMS_STATUS & 0x0100) == 0x0100) { create_status_label("Relays Opened under Load Failsafe", data); flag_index++; }
    if ((BMS_STATUS & 0x1000) == 0x1000) { create_status_label("Polarization Model 1 Active", data); flag_index++; }
    if ((BMS_STATUS & 0x2000) == 0x2000) { create_status_label("Polarization Model 2 Active", data); flag_index++; }
    if ((BMS_STATUS & 0x8000) == 0x8000) { create_status_label("Charge Mode Activated over CANBUS", data); flag_index++; }

    // Relay status
    if ((RELAYS & 0x0001) != 0x0001) { create_status_label("Discharge Relay Opened", data); flag_index++; }
    // CONTROLLED BY ARDUINO AND DISABLED IN BMS if ((RELAYS & 0x0002) == 0x0000) { create_status_label("Charge Relay Opened", data); flag_index++; }

    // Custom status messages
    if ( TX_TRIP_PV ) { create_status_label("Charge Disabled by Arduino", data); flag_index++; comparator_index++;}
    if ( ! lv_obj_has_flag(userData[3].dcl_label, LV_OBJ_FLAG_HIDDEN) ) { create_status_label("Discharge Disabled by Arduino", data); flag_index++; comparator_index++;} // If Inverter DCL CHECK triggered

    // Cell balancing check at end ensures higher importance messages appear above
    if ((BMS_STATUS & 0x0008) == 0x0008) {
      if ( balancing_label_showing ) {
        create_status_label("", data); // create blank label
        balancing_label_showing = false;
      }
      else {
        create_status_label("Cell Balancing Active", data);
        balancing_label_showing = true;
      }

      flag_index++;
      comparator_index++;
    }

  // Show title and button
  if ( flag_index > -1 ) {
 
    // SHOW TITLE
    if ( lv_obj_has_flag(data->title_label, LV_OBJ_FLAG_HIDDEN) ) {
      lv_obj_clear_flag(data->title_label, LV_OBJ_FLAG_HIDDEN);
    }
    // SHOW BUTTON
    if ( lv_obj_has_flag(data->button, LV_OBJ_FLAG_HIDDEN) && flag_index != comparator_index ) {
      lv_obj_clear_flag(data->button, LV_OBJ_FLAG_HIDDEN);
    }
    // HIDE BUTTON
    else {
      lv_obj_add_flag(data->button, LV_OBJ_FLAG_HIDDEN);
    }
  }

  // HIDE EVERYTHING IF NO FLAGS AND IF THEY WERE PREVIOUSLY SHOWING
  else {
    if ( ! lv_obj_has_flag(data->title_label, LV_OBJ_FLAG_HIDDEN) ){
       lv_obj_add_flag(data->title_label, LV_OBJ_FLAG_HIDDEN);
    }
    if ( ! lv_obj_has_flag(data->button, LV_OBJ_FLAG_HIDDEN) ) {
       lv_obj_add_flag(data->button, LV_OBJ_FLAG_HIDDEN);
    }
  }

  // run the function with true argument to tell it we are finished checking bms for messages to reset function index
  create_status_label("", data, true);
}

// CREATE BMS STATUS LABELS //////////////////////////////////////////////////////
void create_bms_status_label(lv_obj_t *parent, lv_coord_t y, bms_status_data_t *data) {
  if (data) {
    data->parent = parent;
    data->y = y;

    // Create title label with underline style (hidden initially)
    data->title_label = lv_label_create(parent);
      lv_style_init(&underline);
      lv_style_set_text_decor(&underline, LV_TEXT_DECOR_UNDERLINE); // set underline style
      lv_obj_add_style(data->title_label, &underline, 0);
      lv_label_set_text(data->title_label, "BMS Status Messages");
      lv_obj_align(data->title_label, LV_ALIGN_TOP_MID, 0, y);
      lv_obj_add_flag(data->title_label, LV_OBJ_FLAG_HIDDEN);

    // Initialize button (hidden initially)
    data->button = lv_btn_create(parent);
      lv_obj_t *btn_label = lv_label_create(data->button);
      lv_label_set_text(btn_label, "Clear BMS Flags");
      lv_obj_add_event_cb(data->button, clear_bms_flag, LV_EVENT_CLICKED, NULL);
      lv_obj_add_flag(data->button, LV_OBJ_FLAG_HIDDEN);

    // Refresh status labels every second
    data->timer = lv_timer_create(refresh_bms_status_data, 1000, data); // stored in struct to allow msgbox to pause timer inhibiting labels from showing on msgbox
  }
}






















// DATA SCREEN FLASHING CHARGE SYMBOLS ////////////////////////////////////////////////
void flash_icons(lv_timer_t *timer) {
  data_display_t *data = (data_display_t*)timer->user_data;
  bool flashing_battery = false;

  // FLASHING GREEN WHILST CHARGING
  if ( AVG_AMPS < 0 ) {
    lv_obj_set_style_text_color(data->car_battery_icon, lv_palette_main(LV_PALETTE_GREEN), NULL);
    flashing_battery = true;
  }
  // WHITE ABOVE 20%
  else if ( SOC > 20 ) {
    lv_obj_set_style_text_color(data->car_battery_icon, lv_color_white(), NULL);
  }
  // ORANGE BELOW 20%
  else if ( SOC > 10 ) {
    lv_obj_set_style_text_color(data->car_battery_icon, lv_palette_main(LV_PALETTE_ORANGE), NULL);
  }
  // RED FLASHING BATTERY ICON FROM SOC 10% AND LESS
  else {
    lv_obj_set_style_text_color(data->car_battery_icon, lv_palette_main(LV_PALETTE_RED), NULL);
    flashing_battery = true;
  }

  // MAKE BATTERY ICON VISIBLE IF PREVIOUSLY HIDDEN
  if ( lv_obj_has_flag(data->car_battery_icon, LV_OBJ_FLAG_HIDDEN) ) {
    lv_obj_clear_flag(data->car_battery_icon, LV_OBJ_FLAG_HIDDEN);
  }
  // HIDE IT AGAIN IF MENT TO FLASH
  else if ( flashing_battery ) {
    lv_obj_add_flag(data->car_battery_icon, LV_OBJ_FLAG_HIDDEN);
  }

  // SHOW SUN ICON IF SOLAR DETECTED THROUGH CHARGE ENABLED SIGNAL TRANSMITTED FROM BMS
  if ( (CUSTOM_FLAGS & 0x0001) == 0x0001 ) {
    lv_label_set_text(data->charge_icon, "\uF185"); // sun icon
  }
  // SHOW GRID ICON
  else if ( AVG_AMPS < 0 ) {
    lv_label_set_text(data->charge_icon, "\uF1E6"); // \uF0E7 lightening bolt, \uF1E6 two-pin plug
    TX_MSG[2] = 0x01;
    TX_BALANCING = true;
    return; // no need to continue as this will not be flashing
  }
  // NO CHARGE NO ICON
  else {
    lv_label_set_text(data->charge_icon, "");
    TX_BALANCING = false;
    TX_MSG[2] = 0x00;
    return; // same for this
  }
  
  // FLASH SUN IF THERE IS CHARGE SIGNAL FROM SOLAR BUT NO CHARGING OR MPPT HAS BEEN DISABLED
  if ( AVG_AMPS >= 0 && userData[3].on == false || TX_TRIP_PV ) {
    if ( lv_obj_has_flag(data->charge_icon, LV_OBJ_FLAG_HIDDEN) ) {
      lv_obj_clear_flag(data->charge_icon, LV_OBJ_FLAG_HIDDEN);
    }
    else {
      lv_obj_add_flag(data->charge_icon, LV_OBJ_FLAG_HIDDEN);
    }
  }
}

// DATA SCREEN UPDATER ////////////////////////////////////////////////////////////////
void data_display_updater(lv_timer_t *timer) {
  data_display_t *data = (data_display_t*)timer->user_data;
  char battery[4];
  
  // BATTERY SYMBOL UPDATER
  if ( SOC >= 80 ) {
    strcpy(battery, LV_SYMBOL_BATTERY_FULL);
  }
  else if ( SOC >= 60 ) {
    strcpy(battery, LV_SYMBOL_BATTERY_3);
  }
  else if ( SOC >= 40 ) {
    strcpy(battery, LV_SYMBOL_BATTERY_2);
  }
  else if ( SOC >= 20 ) {
    strcpy(battery, LV_SYMBOL_BATTERY_1);
  }
  else {
    strcpy(battery, LV_SYMBOL_BATTERY_EMPTY);
  }

  // UPDATE SOC ARC VALUES AND CHANGE INDICATOR TO RED COLOUR FROM 10%
  lv_arc_set_value(data->soc_arc, SOC);
  if ( SOC > 10 ) {
    lv_obj_set_style_arc_color(data->soc_arc, lv_palette_main(LV_PALETTE_LIGHT_BLUE), LV_PART_INDICATOR);
  }
  else {
    lv_obj_set_style_arc_color(data->soc_arc, lv_palette_main(LV_PALETTE_RED), LV_PART_INDICATOR);
  }


  // UPDATE CHG & DCH ARC RANGES DYNAMICALLY
  lv_arc_set_range(data->discharge_arc, 0, DCL);
  lv_arc_set_range(data->charge_arc, 0, CCL);

  // UPDATE CHARGE AND DISCHARGE ARCS FROM AVERAGE CURRENT READING
  if ( AVG_AMPS > 0 ) {
    lv_arc_set_value(data->discharge_arc, AVG_AMPS);
    lv_arc_set_value(data->charge_arc, 0);
  }
  else {
    lv_arc_set_value(data->charge_arc, abs(AVG_AMPS));
    lv_arc_set_value(data->discharge_arc, 0);
  }

  // UPDATE LABEL TEXT
  lv_label_set_text_fmt(data->battery_label, "%s Battery", battery);
  lv_label_set_text_fmt(data->soc_label, "%d%%", SOC);
  lv_label_set_text_fmt(data->volt_label, "%.2fV", VOLT);
  lv_label_set_text_fmt(data->amps_label, "%.1fA", AMPS);
  lv_label_set_text_fmt(data->watt_label, "%dW", WATTS);
}

// CREATE DATA DISPLAY //////////////////////////////////////////////////////////////////
void create_data_display(lv_obj_t *parent, data_display_t *data) {

  // CREATE 360 ARC FOR SOC READING
  data->soc_arc = lv_arc_create(parent);
    lv_obj_set_size(data->soc_arc, 200, 200);
    lv_arc_set_rotation(data->soc_arc, 270);
    lv_arc_set_bg_angles(data->soc_arc, 0, 360);
    lv_obj_remove_style(data->soc_arc, NULL, LV_PART_KNOB); // remove arc knob
    lv_obj_set_style_arc_rounded(data->soc_arc, false, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(data->soc_arc, 10, LV_PART_MAIN);
    lv_obj_set_style_arc_width(data->soc_arc, 15, LV_PART_INDICATOR);
    lv_obj_clear_flag(data->soc_arc, LV_OBJ_FLAG_CLICKABLE); // remove clickable feature
    lv_obj_align_to(data->soc_arc, parent, LV_ALIGN_TOP_MID, 0, 30);

  // CREATE CHARGE ARC 
  data->charge_arc = lv_arc_create(parent);
    lv_obj_set_size(data->charge_arc, 300, 300);
    lv_arc_set_rotation(data->charge_arc, 150);
    lv_arc_set_bg_angles(data->charge_arc, 0, 60);
    lv_obj_remove_style(data->charge_arc, NULL, LV_PART_KNOB); // remove arc knob
    lv_obj_set_style_arc_color(data->charge_arc, lv_palette_main(LV_PALETTE_GREEN), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(data->charge_arc, 10, LV_PART_MAIN);
    lv_obj_set_style_arc_width(data->charge_arc, 10, LV_PART_INDICATOR);
    lv_obj_clear_flag(data->charge_arc, LV_OBJ_FLAG_CLICKABLE); // remove clickable feature
    lv_obj_align_to(data->charge_arc, parent, LV_ALIGN_TOP_LEFT, 0, -15);

  // CREATE DISCHARGE
  data->discharge_arc = lv_arc_create(parent);
    lv_obj_set_size(data->discharge_arc, 300, 300);
    lv_arc_set_rotation(data->discharge_arc, 330);
    lv_arc_set_bg_angles(data->discharge_arc, 0, 60);
    lv_arc_set_mode(data->discharge_arc, LV_ARC_MODE_REVERSE);
    lv_obj_remove_style(data->discharge_arc, NULL, LV_PART_KNOB); // remove arc knob
    lv_obj_set_style_arc_color(data->discharge_arc, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(data->discharge_arc, 10, LV_PART_MAIN);
    lv_obj_set_style_arc_width(data->discharge_arc, 10, LV_PART_INDICATOR);
    lv_obj_clear_flag(data->discharge_arc, LV_OBJ_FLAG_CLICKABLE); // remove clickable feature
    lv_obj_align_to(data->discharge_arc, parent, LV_ALIGN_TOP_RIGHT, 0, -15);

  // CREATE LABELS AND ASSOCIATED STYLES
  data->charge_label = lv_label_create(parent);
    lv_label_set_text(data->charge_label, "CHARGE");
    //lv_obj_set_style_text_align(data->charge_label, LV_TEXT_ALIGN_CENTER, NULL);
  
  data->load_label = lv_label_create(parent);
    lv_label_set_text(data->load_label, "LOAD");
    //lv_obj_set_style_text_align(data->load_label, LV_TEXT_ALIGN_CENTER, NULL); // was used when i added % below. but it clutters

  data->battery_label = lv_label_create(parent);

  data->soc_label = lv_label_create(parent);    
    lv_obj_set_style_text_font(data->soc_label, &Montserrat34_0_9_percent, NULL);
    lv_obj_set_style_text_align(data->soc_label, LV_TEXT_ALIGN_CENTER, NULL);
    lv_obj_add_flag(data->soc_label, LV_OBJ_FLAG_CLICKABLE); // make it clickable to open msg_box
    lv_obj_add_event_cb(data->soc_label, can_msgbox, LV_EVENT_CLICKED, &canMsgBoxData); // add event handler to clicks

  data->volt_label = lv_label_create(parent);

  data->amps_label = lv_label_create(parent);
    //lv_obj_set_style_text_align(data->amps_label, LV_TEXT_ALIGN_RIGHT, NULL); // DOESN'T WORK AS INTENDED

  data->watt_label = lv_label_create(parent);
    lv_obj_set_style_text_font(data->watt_label, &Montserrat20_0_9_W_minus, NULL);
    lv_obj_set_style_text_align(data->watt_label, LV_TEXT_ALIGN_CENTER, NULL);

  data->charge_icon = lv_label_create(parent);
    lv_obj_set_style_text_font(data->charge_icon, &FontAwesomeIcons, NULL);
    lv_obj_set_style_text_color(data->charge_icon, lv_palette_main(LV_PALETTE_YELLOW), NULL);

  data->car_battery_icon = lv_label_create(parent);
    lv_obj_set_style_text_font(data->car_battery_icon, &FontAwesomeIcons, NULL);
    lv_label_set_text(data->car_battery_icon, "\uF5DF");

  // ALLIGN LABELS
  lv_obj_align_to(data->charge_label,       data->soc_arc,        LV_ALIGN_OUT_LEFT_BOTTOM,  -5,   8);
  lv_obj_align_to(data->load_label,         data->soc_arc,        LV_ALIGN_OUT_RIGHT_BOTTOM, 12,   8);
  lv_obj_align_to(data->battery_label,      data->soc_arc,        LV_ALIGN_CENTER,          -18, -44);
  lv_obj_align_to(data->soc_label,          data->soc_arc,        LV_ALIGN_CENTER,            0,   0);
  lv_obj_align_to(data->volt_label,         data->soc_arc,        LV_ALIGN_BOTTOM_MID,      -40, -44);
  lv_obj_align_to(data->amps_label,         data->soc_arc,        LV_ALIGN_BOTTOM_MID,       30, -44);
  lv_obj_align_to(data->charge_icon,        data->soc_arc,        LV_ALIGN_OUT_LEFT_MID,     14,   5);
  lv_obj_align_to(data->car_battery_icon,   data->soc_arc,        LV_ALIGN_OUT_RIGHT_MID,    16,   5);
  lv_obj_align_to(data->watt_label,         data->soc_arc,        LV_ALIGN_OUT_BOTTOM_MID,    0,  15);
    
  // CREATE LABEL UPDATE TIMER
  lv_timer_create(data_display_updater, 200, data);

  // FLASHING EFFECT FOR ARROWS AND SUN
  lv_timer_create(flash_icons, 1000, data);
}





// SETUP //////////////////////////////////////////////////////////////////////////////
void setup() {

  lv_init();

  if ( Serial ) Serial.begin(115200); // Initialize Serial Monitor if available
  //while (!Serial); // used to delay boot until serial console ready, in order to read all messages

  // Boot M4 & Initialize RPC protocol
  if ( RPC.begin() ) {
    Serial.println("M7 sending Boot command to M4 Core");
  }
  else {
    Serial.println("M7 Failed to boot M4 Core");
  }

  if ( ! CAN.begin(CanBitRate::BR_500k) ) {
        Serial.println("CAN.begin(...) failed.");
        for (;;) {}
  }
  
  // Initialise display, touch and backlight
  Display.begin();
  TouchDetector.begin();
  backlight.begin();

  // Set display brightness
  backlight.set(brightness);

  // Create two columns on active screen, grid 2x1
  static lv_coord_t col_dsc[] = {370, 370, LV_GRID_TEMPLATE_LAST};
  static lv_coord_t row_dsc[] = {430, 430, LV_GRID_TEMPLATE_LAST};
  lv_obj_t * parent = lv_obj_create(lv_scr_act());
  lv_obj_set_grid_dsc_array(parent, col_dsc, row_dsc);
  lv_obj_set_size(parent, Display.width(), Display.height());
  //lv_obj_set_style_bg_color(parent, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN);
  lv_obj_center(parent);

  // initialise container object
  lv_obj_t *cont;

  // create left column container object
  cont = lv_obj_create(parent);
  lv_obj_add_event_cb(cont, screen_touch, LV_EVENT_ALL, NULL);
  lv_obj_set_grid_cell(cont, LV_GRID_ALIGN_STRETCH, 0, 1,
                        LV_GRID_ALIGN_STRETCH, 0, 1);

  // Create charge/discharge clock
  create_clock_label(cont, &clockData);

  // Display bms messages arg2: y_pos
  create_bms_status_label(cont, 280, &bmsStatusData);

  // Initialise click event for CANdata message box
  canMsgBoxData.parent = cont;

  // check for sunrise by reading BMS charge enable signal from CANbus and sending MPO#1 signal to trip relay if flapping detected
  lv_timer_create(sunrise_detector, 1000, NULL);

  // CCL limit checker for Solar Panel Relay control through CANbus
  lv_timer_create(ccl_check, 1000, NULL);

  // Create data display
  create_data_display(cont, &dataDisplay);

  // create right column container object
  cont = lv_obj_create(parent);
  lv_obj_add_event_cb(cont, screen_touch, LV_EVENT_ALL, NULL);
  lv_obj_set_grid_cell(cont, LV_GRID_ALIGN_STRETCH, 1, 1,
                            LV_GRID_ALIGN_STRETCH, 0, 1);

  // arguments 1:obj  2:label 3:relay_pin 4:y_offset 5:dcl_limit 6:timeout_ms 7:user_data struct

  // Create Button 1 - CEILING HEATER
  create_button(cont, "Ceiling Heater", RELAY2, 20, 70, 0, &userData[0]); // dcl for test max 255 uint8_t

  // Create Button 2 - SHOWER HEATER
  create_button(cont, "Shower Heater",  RELAY4, 120, 10, 0, &userData[1]);

  // Create Button 3 - HOT WATER
  create_button(cont, "Hot Water",      RELAY3, 220, 60, hot_water_interval_ms, &userData[2]);

  // Create Button 4 - INVERTER
  create_button(cont, "Inverter",       RELAY1, 320, 5, inverter_startup_delay_ms, &userData[3]);

}

// LOOP ///////////////////////////////////////////////////////////////////////////////////
void loop() {

  // Handle screen events
  lv_timer_handler();
  lv_task_handler();

  // CANBUS READ AND WRITE
  if (CAN.available()) {
    CanMsg const msg = CAN.read();
    canMsgData.rxId = msg.id;
    canMsgData.len = msg.data_length;
    memcpy(CAN_RX, msg.data, canMsgData.len);
    sort_can();

    // send CAN if commanded
    if ( TX_CLR_BMS || TX_TRIP_PV || TX_BALANCING ) {
      CanMsg send_msg(CanStandardId(canMsgData.CAN_ID), sizeof(TX_MSG), TX_MSG);

      // retry if send failed for byte 0 - clear bms through mpo2
      int const rc = CAN.write(send_msg);
      if ( rc <= 0 && canMsgData.msg_cnt < 3 ) {
        Serial.print("CAN.write(...) failed with error code ");
        Serial.println(rc);
        canMsgData.msg_cnt++;
      }
      // stop sending to mpo2 after 3 retries
      else if ( TX_CLR_BMS ) {
        TX_CLR_BMS = false; // stop sending after 3 tries
        TX_MSG[0] = 0; // clear send data
      }
    }
  }
  if (RPC.available()) {
    // call func to get sensors and can data from M4 core
    retrieve_M4_data();

    // check for messages from M4 core if Serial connection

    if ( Serial ) {
      while (RPC.available()) {
        buffer += (char)RPC.read();  // Fill the buffer with characters
      }
      if (buffer.length() > 0) {
        Serial.println(buffer);
        buffer = ""; // Clear buffer after printing
      }
      else Serial.println("RPC is available but no messages received from M4");
    }
  }

  // if no touch within timeout dim display and brightness above 0
  if ( previous_touch_ms + touch_timeout_ms < millis() && brightness ) {
    dim_display();
  }

  if (Serial) {
    // Average loop lap time of 256 iterations - reflects the lvgl delay at end of loop
    static uint8_t i = 0;
    static uint32_t start_time = millis();
    static bool finished = false;

    // start iterations
    if ( i <  255 && ! finished ) {
      i++;
    }
    // calculate and write result
    else if ( ! finished ) {
      uint8_t avg_lap = (millis() - start_time) / 256;
      char buf[30];
      snprintf(buf, sizeof(buf), "%d ms average loop lap", avg_lap);
      Serial.println(buf);
      finished = true;
    }
    // start again at 30s intervals
    else if ( finished && (start_time + 30000) < millis() ) {
      i = 0;
      start_time = millis();
      finished = false;
    }
  }
  delay(4); // lvgl recommends 5ms delay for display (code takes up 1ms)
}
