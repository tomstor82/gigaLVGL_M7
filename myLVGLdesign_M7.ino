#include <Arduino.h>
#include <RPC.h>
#include <Arduino_H7_Video.h>
#include "lvgl.h"
#include "Arduino_GigaDisplayTouch.h"
#include <Arduino_CAN.h>
#include <Arduino_GigaDisplay.h>

Arduino_H7_Video Display(800, 480, GigaDisplayShield);
Arduino_GigaDisplayTouch TouchDetector;

//Create backlight object
GigaDisplayBacklight backlight;

// Define relay pins
#define RELAY1 10   // inverter
#define RELAY2 11   // ceiling heater
#define RELAY3 12   // water heater
#define RELAY4 13   // shower room

//  CANBUS data Identifier List
//  ID 0x03B BYT0+1:INST_VOLT BYT2+3:INST_AMP BYT4+5:ABS_AMP BYT6:SOC **** ABS_AMP from OrionJr errendous ****
//  ID 0x6B2 BYT0+1:LOW_CELL BYT2+3:HIGH_CELL BYT4:HEALTH BYT5+6:CYCLES
//  ID 0x0A9 BYT0:RELAY_STATE BYT1:CCL BYT2:DCL BYT3+4:PACK_AH BYT5+6:AVG_AMP
//  ID 0x0BD BYT0+1:CUSTOM_FLAGS BYT2:HI_TMP BYT3:LO_TMP BYT4:CUSTOM_FLAGS BYT5:BMS_STATUS
//  ID 0x0BE BYT0:HI_CL_ID BYT1:LO_CL_ID BYT2:INT_HEATSINK BYT3:COUNTER

// Temp and Humidity data struct from M4
struct SensorData {
    float temp1;
    float temp2;
    float temp3;
    float temp4;
    float humi1;
    float humi2;
    float humi3;
    float humi4;
    float avg_temp;

    MSGPACK_DEFINE_ARRAY(temp1, temp2, temp3, temp4, humi1, humi2, humi3, humi4, avg_temp);
};

// CanData struct
struct CanData {

    int p;                  // watt calculated by script

    float instU;            // Voltage - multiplied by 10
    float instI;            // Current - multiplied by 10 - negative value indicates charge
    float avgI;             // Average current for clock and sun symbol calculations
    //float absI;             // Absolute Current NOT WORKING CORRECTLY
    float ah;               // Amp hours
    float hC;               // High Cell Voltage in 0,0001V
    float lC;               // Low Cell Voltage in 0,0001V

    byte soc;               // State of charge - multiplied by 2
    byte hT;                // Highest cell temperature * was int
    byte lT;                // Lowest cell temperature * was int
    byte ry;                // Relay status
    byte dcl;               // Discharge current limit * was unsigned int
    byte ccl;               // Charge current limit * was unsigned int
    byte ct;                // Counter to observe data received
    byte h;                 // Health
    byte hCid;              // High Cell ID
    byte lCid;              // Low Cell ID

    uint16_t fu;            // BMS Faults
    uint16_t st;            // BMS Status
    int cc;                 // Total pack cycles (int to avoid overrun issues with uint16_t)

    byte hs;                // Internal Heatsink
    byte cu;                // BMS custom flag currently used for sending charge power enabled and MPO#1 state (used for tripping chg enabled relay)
    
    MSGPACK_DEFINE_ARRAY(instU, instI, soc, hC, lC, h, fu, hT, lT, ah, ry, dcl, ccl, ct, st, cc, avgI, hCid, lCid, p, hs, cu);
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
  uint8_t msg_data[2]; // 2 bytes used in orion MPO#1 and #2
  uint8_t msg_cnt;

  bool send_mpo1 = false;
  bool send_mpo2 = false;

  // Constructor to initialize non const values
  CanMsgData() : rxId(0), len(0), msg_data{}, msg_cnt(0) {}
  
};

// Type defined structure for bms status messages allowing it to be passed to function
typedef struct {
    lv_obj_t* parent;
    lv_obj_t* title_label;
    lv_obj_t* button;
    lv_obj_t* status_label[28]; // can be expanded beyond the current 28 bms messages
    uint8_t y;
} bms_status_data_t;

// Define an enumeration for the different data types.
typedef enum {
    CAN_DATA_TYPE_INT,
    CAN_DATA_TYPE_FLOAT,
    CAN_DATA_TYPE_DOUBLE_FLOAT, // 2 decimals
    CAN_DATA_TYPE_BYTE
} can_data_type_t;

// define struct for function user-data
typedef struct { // typedef used to not having to use the struct keyword for declaring struct variable
  lv_obj_t* button;
  lv_obj_t* dcl_label;
  lv_obj_t* label_obj;
  lv_obj_t* msgbox;
  lv_timer_t* timer;
  uint8_t relay_pin;
  uint8_t y_offset;
  unsigned long timeout_ms;
  uint8_t dcl_limit;
  uint8_t set_temp = 20; // should match value of dropdown default index
} user_data_t;

typedef struct {
  lv_obj_t* clock_label;
} clock_data_t;

typedef struct {
  lv_obj_t* parent;
  lv_obj_t* label_obj;
  lv_obj_t* msgbox;
  lv_timer_t* timer;
} can_msgbox_data_t;

typedef struct {
  lv_obj_t* label_obj;
  lv_obj_t* label_text;
  const char* label_prefix; // To store label text prefix e.g Voltage
  const char* label_unit; // Label unit e.g V,A or W
  union {
    int* intData;
    float* floatData;
    byte* byteData;
  } canDataProperty;
  can_data_type_t canDataType;
} can_label_t;

//Initialise structures
static CanMsgData canMsgData;
static bms_status_data_t bmsStatusData;
static user_data_t userData[4] = {}; // 4 buttons with user_data
static clock_data_t clockData;
static can_msgbox_data_t canMsgBoxData;
static can_label_t canLabel[6] = {}; // 6 labels so far
static CombinedData combinedData;

// global variables * 8bits=256 16bits=65536 32bits=4294967296 (millis size) int/float = 4 bytes
int16_t inverter_prestart_p = 0; // must be signed
uint8_t inverter_standby_p = 70; // takes around 4 minutes to settle down after start
uint32_t inverter_startup_ms = 0;
uint8_t pwr_demand = 0;
const uint32_t hot_water_interval_ms = 900000; // 15 min
const uint16_t inverter_startup_delay_ms = 25000; // 25s startup required before comparing current flow for soft start appliances
const uint32_t sweep_interval_ms = 180000; // 3 minute sweep interval reduces standby consumption from 85Wh to around 12,5Wh -84%
static lv_style_t style; // cascading style for text labels

static uint8_t brightness = 70;
uint32_t previous_touch_ms = 0;
const uint16_t touch_timeout_ms = 30000; // 30s before screen dimming

// for M4 messages
static String buffer = "";

//**************************************************************************************
//  BUG#1 bms clear fault button wandering down each second
//  BUG#2 crash after a few minutes once inverter is turned OFF
//  BUG#3 inverter doesn't start when mppt has low power. Turn off CHG relay when this is the case
//**************************************************************************************

// CREATE BUTTON INSTANCE
void create_button(lv_obj_t *parent, const char *label_text, uint8_t relay_pin, lv_coord_t y_offset, uint8_t dcl_limit, unsigned long timeout_ms, user_data_t* data) {

  // configure relay pins
  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, LOW); // initialise pin LOW

  // Update struct with received arguments
  data->relay_pin = relay_pin;
  data->y_offset = y_offset;
  data->dcl_limit = dcl_limit;
  data->timeout_ms = timeout_ms;

  // create button object and add to struct to be able to clear later
  data->button = lv_btn_create(parent);

  lv_obj_set_pos(data->button, 10, y_offset);
  lv_obj_t *label = lv_label_create(data->button);
  lv_label_set_text(label, label_text);
  lv_obj_center(label);
  lv_obj_add_flag(data->button, LV_OBJ_FLAG_CHECKABLE);

  // Disable all buttons by DCL limit
  data->dcl_label = lv_label_create(lv_obj_get_parent(data->button));
  lv_label_set_long_mode(data->dcl_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
  lv_obj_set_width(data->dcl_label, 140);
  lv_obj_align_to(data->dcl_label, data->button, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);
  lv_obj_add_flag(data->dcl_label, LV_OBJ_FLAG_HIDDEN); // hide label initially
  lv_timer_create(dcl_check, 100, data); // check every 100ms to allow prompt off if dch relay open

  // create temperature dropdown and dynamic temperature labels for thermostat buttons
  if ( ! timeout_ms ) {
    // create label and update the user data member for access within timer to allow only to update text not object
    data->label_obj = lv_label_create(lv_obj_get_parent(data->button));

    // Set temp label width
    lv_obj_set_width(data->label_obj, 80);
    lv_obj_set_pos(data->label_obj, 160, data->y_offset + 13);

    // Align the text in the center
    lv_obj_set_style_text_align(data->label_obj, LV_TEXT_ALIGN_CENTER, 0);

    // Create timer for updating temperature labels
    lv_timer_create(update_temp, 10000, data);

    // Set initial text
    lv_label_set_text(data->label_obj, LV_SYMBOL_REFRESH);

    // Make label clickable and add event handler for showing sensor message box
    lv_obj_add_flag(data->label_obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(data->label_obj, sensor_msgbox, LV_EVENT_CLICKED, data);

    // Create Drop down for temperature selection
    create_temperature_dropdown(parent, data);
  }
     
  // Hot Water and Inverter Event Handler
  if ( timeout_ms ) { // hot water and inverter
    lv_obj_add_event_cb(data->button, hot_water_inverter_event_handler, LV_EVENT_ALL, data);
  }
  else { // heaters
    lv_obj_add_event_cb(data->button, thermostat_event_handler, LV_EVENT_ALL, data);
  }
  // creating inverter status label
  if ( relay_pin == RELAY1 ) {
    data->label_obj = lv_label_create(lv_obj_get_parent(data->button));
    lv_obj_set_width(data->label_obj, 120);
    lv_obj_align_to(data->label_obj, data->button, LV_ALIGN_OUT_RIGHT_MID, 80, 0);
    // initialise label text
    lv_label_set_text(data->label_obj, "OFF");
  }
}







// SUNRISE CHECK TIMER ////////////////////////////////////////////////////////////////////////
void mppt_delay(lv_timer_t* timer) {

  static uint32_t time_ms = 0;

  //  If CHG ENABLED is received over CAN and then lost within 30s disable solar panels for 30 mins by sending CAN MPO#1 msg
  //  If CHG ENABLED is lost then received and lost again within 30s disable solar panels for 30 mins

  // Initialisation - If pv power is present but no previous timer or can msg sendt
  if ( (combinedData.canData.cu & 0x01) == 0x01 && ! time_ms && ! canMsgData.send_mpo1 ) { //start_time && ! charge_power ) {
    time_ms = millis(); // record time to check for flapping
    return; // no point in continuing as timer will not be met on next conditions
  }

  // Flapp checker - Did charge signal drop?
  else if ( ! (combinedData.canData.cu & 0x01) == 0x01 && time_ms && ! canMsgData.send_mpo1 ) {

    // Is signal drop within 30s it qualifies as flapping - start sending CAN MPO#1 signal to open CHG relay
    if ( time_ms + (30 * 1000) > millis() ) {
      time_ms = millis(); // record time to track when can send started
      canMsgData.msg_data[1] = 0x01;
      canMsgData.send_mpo1 = true; // this triggers send function in loop
    }
    // if no flapping detected as during sunset
    else {
      time_ms = 0; // reset timer to re-enable initialisation check
    }
    return;
  }

  // Finish timer - if 30 minutes has passed stop sending can mpo1 msg to enable solar charging
  else if ( time_ms && (time_ms + (30 * 60 * 1000)) < millis() ) {
    time_ms = 0; // reset timer as we are finished
    canMsgData.send_mpo1 = false; // allow CHG relay to close and solar to start
    canMsgData.msg_data[1] = 0;
  }
}






// CCL CHECK TIMER ////////////////////////////////////////////////////////////////////////////
void ccl_check(lv_timer_t * timer) {

  if ( combinedData.canData.ccl == 0 ) {
    canMsgData.send_mpo1 = true; // trip charge relay
  }
  else canMsgData.send_mpo1 = false;
}





// DCL CHECK TIMER ////////////////////////////////////////////////////////////////////////////
void dcl_check(lv_timer_t * timer) {
  user_data_t * data = (user_data_t *)timer->user_data;

  // disable inverter if discharge relay is tripped
  /*if ( data->relay_pin == RELAY1 && (combinedData.canData.ry & 0x0001) != 0x0001 ) {
    lv_label_set_text(data->dcl_label, "Discharge Relay Tripped by BMS                                      "); // spaces to allow a pause
    lv_obj_clear_flag(data->dcl_label, LV_OBJ_FLAG_HIDDEN); // clear hidden flag to show

    // if inverter is on turn it off
    if ( lv_obj_has_state(data->button, LV_STATE_CHECKED) ) {
      lv_event_send(data->button, LV_EVENT_RELEASED, NULL);
    }
    // disable button
    lv_obj_add_state(data->button, LV_STATE_DISABLED);
  }*/
  // disable buttons if dcl limit below appliance minimum operating dcl
  if ( combinedData.canData.dcl < data->dcl_limit ) {
    lv_label_set_text(data->dcl_label, "Please Charge Battery                                            "); // spaces to allow a pause
    lv_obj_clear_flag(data->dcl_label, LV_OBJ_FLAG_HIDDEN); // clear hidden flag to show

    // if button is on turn it off
    if ( lv_obj_has_state(data->button, LV_STATE_CHECKED) ) {
      lv_event_send(data->button, LV_EVENT_RELEASED, NULL);
    }
    // disable button
    lv_obj_add_state(data->button, LV_STATE_DISABLED);
  }

  // add hidden flag to label if all ok and enable button if previously disabled
  else {
    lv_obj_add_flag(data->dcl_label, LV_OBJ_FLAG_HIDDEN);
    if ( lv_obj_has_state(data->button, LV_STATE_DISABLED) ) {
      lv_obj_clear_state(data->button, LV_STATE_DISABLED);
    }
  }
}





// MESSAGE BOX FOR CAN-DATA EVENT HANDLERS AND UPDATE TIMER ////////////////////////////////////
const char* set_can_msgbox_text() {
  static char msgbox_text[300]; // Static buffer to retain the value

  snprintf(msgbox_text, sizeof(msgbox_text),
                 "High Cell          %.2fV           #%d\n"
                 "Low Cell           %.2fV           #%d\n\n"
                 "Charge Current Limit       %d A\n"
                 "Discharge Current Limit %d A\n\n"
                 "Charge Enabled                %s\n"
                 "Discharge Enabled           %s\n\n"
                 "Battery Pack Cycles          %d\n"
                 "Battery Pack Health         %d%%",
                 combinedData.canData.hC, combinedData.canData.hCid,
                 combinedData.canData.lC, combinedData.canData.lCid,
                 combinedData.canData.ccl,
                 combinedData.canData.dcl,
                 (combinedData.canData.cu & 0x10) == 0x10 ? "NO" : "YES", // check if BMS MPO#1 signal is received
                 combinedData.canData.dcl ? "YES" : "NO", // if DCL = 0 discharge is not enabled
                 combinedData.canData.cc,
                 combinedData.canData.h);
  //Serial.print("Size of msgbox: ");Serial.println(charArr); // set snprintf as "int charArr ="
  //Serial.println(msgbox_text);
  return msgbox_text;
}

void can_msgbox_update_timer(lv_timer_t* timer) {
  can_msgbox_data_t* data = (can_msgbox_data_t*)timer->user_data;
  lv_obj_t* label = lv_msgbox_get_text(data->msgbox); // get msgbox text string object excluding title
  lv_label_set_text(label, set_can_msgbox_text()); // Update the text object in the msgBox
}

void can_msgbox(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    can_msgbox_data_t* data = (can_msgbox_data_t*)lv_event_get_user_data(e);
    
    if (code == LV_EVENT_CLICKED) {
        data->msgbox = lv_msgbox_create(data->parent, "     Battery Monitoring Data", set_can_msgbox_text(), NULL, false);
        lv_obj_set_width(data->msgbox, LV_PCT(80)); // Set width to 80% of the screen
        lv_obj_align(data->msgbox, LV_ALIGN_CENTER, 0, 0); // Center the message box on the screen

        // Create a full-screen overlay to detect clicks for closing the message box
        lv_obj_t* overlay = lv_obj_create(lv_scr_act());
        lv_obj_set_size(overlay, LV_HOR_RES, LV_VER_RES);
        lv_obj_set_style_opa(overlay, LV_OPA_TRANSP, 0); // Transparent overlay
        lv_obj_add_event_cb(overlay, close_can_msgbox_event_handler, LV_EVENT_CLICKED, data);

        // Create timer to update data every 10 seconds
        data->timer = lv_timer_create(can_msgbox_update_timer, 10000, data);
    }
}

void close_can_msgbox_event_handler(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  can_msgbox_data_t* data = (can_msgbox_data_t*)lv_event_get_user_data(e);

  if ( code == LV_EVENT_CLICKED) {
    lv_timer_del(data->timer); // Delete the timer
    lv_msgbox_close(data->msgbox);           // Delete the message box

    // Remove the screen overlay object
    lv_obj_del(lv_event_get_current_target(e));
  }
}





// MESSAGE BOX FOR SENSOR-DATA EVENT HANDLERS AND UPDATE TIMER /////////////////////////////////
const char* set_sensor_msgbox_text() {
  static char msgbox_text[291]; // Static buffer to retain the value

  snprintf(msgbox_text, sizeof(msgbox_text),
                 "Temperature 1:            %.1f°C\nHumidity 1:                   %.1f%%\n\n"
                 "Temperature 2:            %.1f°C\nHumidity 2:                   %.1f%%\n\n"
                 "Temperature 3:            %.1f°C\nHumidity 3:                   %.1f%%\n\n"
                 "Temperature 4:            %.1f°C\nHumidity 4:                   %.1f%%",
                 combinedData.sensorData.temp1, combinedData.sensorData.humi1,
                 combinedData.sensorData.temp2, combinedData.sensorData.humi2,
                 combinedData.sensorData.temp3, combinedData.sensorData.humi3,
                 combinedData.sensorData.temp4, combinedData.sensorData.humi4);

  return msgbox_text;
}

void sensor_msgbox_update_timer(lv_timer_t* timer) {
    user_data_t* data = (user_data_t*)timer->user_data;
    lv_obj_t* label = lv_msgbox_get_text(data->msgbox); // get msgBox text string object excluding title
    lv_label_set_text(label, set_sensor_msgbox_text()); // Update the text object in the msgBox
}

void sensor_msgbox(lv_event_t* e) {
    user_data_t* data = (user_data_t*)lv_event_get_user_data(e);
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
        data->msgbox = lv_msgbox_create(lv_obj_get_parent(data->label_obj), "               Sensor Data", set_sensor_msgbox_text(), NULL, false);
        lv_obj_set_width(data->msgbox, LV_PCT(80)); // Set width to 80% of the screen
        lv_obj_align(data->msgbox, LV_ALIGN_CENTER, 0, 0); // Center the message box on the screen

        // Create a full-screen overlay to detect clicks for closing the message box
        lv_obj_t* overlay = lv_obj_create(lv_scr_act());
        lv_obj_set_size(overlay, LV_HOR_RES, LV_VER_RES);
        lv_obj_set_style_opa(overlay, LV_OPA_TRANSP, 0); // Transparent overlay
        lv_obj_add_event_cb(overlay, close_sensor_msgbox_event_handler, LV_EVENT_CLICKED, data);

        // Create timer to update data every 10 seconds
        data->timer = lv_timer_create(sensor_msgbox_update_timer, 10000, data);
    }
}

void close_sensor_msgbox_event_handler(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  user_data_t* data = (user_data_t*)lv_event_get_user_data(e);

  if ( code == LV_EVENT_CLICKED) {
    lv_timer_del(data->timer); // Delete the timer
    lv_msgbox_close(data->msgbox);           // Delete the message box

    // Remove the screen overlay object
    lv_obj_del(lv_event_get_current_target(e));
  }
}






// HOT WATER AND INVERTER EVENT HANDLER ////////////////////////////////////////////////////
void hot_water_inverter_event_handler(lv_event_t* e) {
  user_data_t * data = (user_data_t *)lv_event_get_user_data(e);
  lv_event_code_t code = lv_event_get_code(e);
  if(code == LV_EVENT_CLICKED) {

    // Button ON
    if ( lv_obj_has_state(data->button, LV_STATE_CHECKED) ) {

      // Inverter
      if ( data->relay_pin == RELAY1 ) { // only for inverter for sweeping
        inverter_prestart_p = combinedData.canData.p;
        lv_label_set_text(data->label_obj, "Inverter ON");
      }

      // Start Inverter if OFF
      else if ( ! lv_obj_has_state(userData[3].button, LV_STATE_CHECKED) ) {
        lv_event_send(userData[3].button, LV_EVENT_PRESSED, NULL); // Have to include all 3 of these to make it work
        lv_event_send(userData[3].button, LV_EVENT_RELEASED, NULL);
        lv_event_send(userData[3].button, LV_EVENT_CLICKED, NULL);
        // DEBUG if inverter is still off disable change flag
        if ( ! lv_obj_has_state(userData[3].button, LV_STATE_CHECKED) ) {
          lv_obj_clear_state(data->button, LV_STATE_CHECKED);
          Serial.println("DEBUG: hot_water_inverter_event_handler - Unable to send start inverter event");
          return; // exit function if inverter is off
        }
      }

      // Hot water demand if ON
      else pwr_demand++; // only for hot water

      digitalWrite(data->relay_pin, HIGH);
      // Create delay timer for inverter sweep and for hot water timout
      data->timer = lv_timer_create(power_check, data->timeout_ms, data); // want to reset timer once excess solar to hot water is activated
    }

    // Button OFF
    else {
      digitalWrite(data->relay_pin, LOW);
      pwr_demand ? pwr_demand-- : NULL;
      if (data->timer) {
        lv_timer_del(data->timer);
      }
      if ( data->relay_pin == RELAY1 ) {
        lv_label_set_text(data->label_obj, "OFF"); // inverter only
        // turn off the other buttons
        for ( uint8_t i = 0; i < 3; i++ ) {
          if ( lv_obj_has_state(userData[i].button, LV_STATE_CHECKED) ) {
            lv_event_send(userData[i].button, LV_EVENT_RELEASED, NULL);
            digitalWrite(userData[i].relay_pin, LOW); // required as timers keep running
          }
        }
        inverter_prestart_p = 0;
      }
    }
  }
}

// INVERTER SWEEP TIMER ///////////////////////////////////////////////////////////////////
void sweep_timer (lv_timer_t* timer) {
  user_data_t* data = (user_data_t *)timer->user_data;

  // stimulate button with click to start inverter again
  lv_event_send(data->button, LV_EVENT_CLICKED, NULL);

  // delete timer so it only runs once as this function is called continuosly by other timer
  lv_timer_del(timer);
}


// POWER CHECK TIMER ///////////////////////////////////////////////////////////////////////
void power_check(lv_timer_t * timer) {
  user_data_t * data = (user_data_t *)timer->user_data;
  bool on = false;
  
  // Inverter
  if ( data->relay_pin == RELAY1 ) {

    // Remain ON if charging when SOC above 50%
    if ( combinedData.canData.avgI < -5 && combinedData.canData.soc > 50 ) {
      on = true;
    }

    // Remain ON if demand variable set
    else if ( pwr_demand ) {
      on = true;
    }

    // Remain ON if charge is more than 20W or discharging exceeds inverter standby
    else if ( (inverter_standby_p + inverter_prestart_p) > (combinedData.canData.p + 20)|| (inverter_standby_p + inverter_prestart_p) < combinedData.canData.p ) {
      on = true;
    }
  }
  
  // Hot water - reset timer if charging
  else if ( combinedData.canData.ccl < 10 && combinedData.canData.avgI < 0 ) {
    on = true;
  }

  if (on) {
    // we start this timer again
    lv_timer_reset(timer);
  }
  // turn off relay
  else {
    digitalWrite(data->relay_pin, LOW);
    // delete timer if relay is turned off as timer is created by click event handler
    lv_timer_del(timer);

    // Inverter sweep timer starting after turning off relay
    if ( data->relay_pin == RELAY1 && lv_obj_has_state(data->button, LV_STATE_CHECKED) ) {
      lv_label_set_text(data->label_obj, "Power Saving\n3 minutes OFF");
      lv_timer_create(sweep_timer, sweep_interval_ms, data);
    }
    else { // clear button flag for hot water
      lv_obj_clear_state(data->button, LV_STATE_CHECKED);
    }
  }
}






// THERMOSTAT TIMER ////////////////////////////////////////////////////////////////
void thermostat_timer(lv_timer_t * timer) {
  user_data_t * data = (user_data_t *)timer->user_data;

  // variable to check if heaters have been on already as I don't want off interval timer on initial ON
  static bool previous_function_calls = false;

  static uint32_t thermostat_off_ms = 0;

  // confirm heater has been on already before timing the off cycle ( prevents flapping every 10s )
  if ( previous_function_calls && thermostat_off_ms + 120000 > millis() ) { // heater OFF for minimum 2 minutes
    // return if off time interval not yet met
    return;
  }

  // ceiling heater thermostat ( uses 3 or 1 sensors )
  if ( data->relay_pin == RELAY2 ) {
    // need to check which sensor is working ( if none is the temp updater will disable button )
    if ( combinedData.sensorData.avg_temp != 999.0f && combinedData.sensorData.avg_temp < data->set_temp ) {
      digitalWrite(data->relay_pin, HIGH);
    }
    else if ( combinedData.sensorData.temp1 != 999.0f && combinedData.sensorData.temp1 < data->set_temp ) {
      digitalWrite(data->relay_pin, HIGH);
    }
    else if ( combinedData.sensorData.temp2 != 999.0f && combinedData.sensorData.temp2 < data->set_temp ) {
      digitalWrite(data->relay_pin, HIGH);
    }
    else if ( combinedData.sensorData.temp3 != 999.0f && combinedData.sensorData.temp4 < data->set_temp ) {
      digitalWrite(data->relay_pin, HIGH);
    }

    // Open relay when temperature is higher or equal to selected
    else {
      digitalWrite(data->relay_pin, LOW);
      thermostat_off_ms = millis();
    }
  }
  // shower heater thermostat
  else {
    // Close relay if temperature is below selected and button has been pressed
    if ( combinedData.sensorData.temp3 < data->set_temp && lv_obj_has_state(data->button, LV_STATE_CHECKED) ) {
      digitalWrite(data->relay_pin, HIGH);
    }
    // Open relay when temperature is higher or equal to selected
    else {
      digitalWrite(data->relay_pin, LOW);
    }
  }
  previous_function_calls = true;
}

// THERMOSTAT EVENT HANDLER /////////////////////////////////////////////////////////
void thermostat_event_handler(lv_event_t * e) {
  user_data_t * data = (user_data_t *)lv_event_get_user_data(e);
  lv_event_code_t code = lv_event_get_code(e);

  //static lv_timer_t* thermostat = NULL;

  if(code == LV_EVENT_CLICKED) {
    if ( lv_obj_has_state(data->button, LV_STATE_CHECKED) ) {
      // check if inverter is on
      if ( ! lv_obj_has_state(userData[3].button, LV_STATE_CHECKED) ) {
        lv_event_send(userData[3].button, LV_EVENT_PRESSED, NULL); // ********************** not working yet **** pressed works not clicked
        lv_event_send(userData[3].button, LV_EVENT_RELEASED, NULL);
        lv_event_send(userData[3].button, LV_EVENT_CLICKED, NULL);
        // DEBUG if inverter is still off disable change flag
        if ( ! lv_obj_has_state(userData[3].button, LV_STATE_CHECKED) ) {
          lv_obj_clear_state(data->button, LV_STATE_CHECKED);
          Serial.println("DEBUG thermostat_event_handler - Unable to send start inverter event");
          return; // exit function if inverter is off
        }
      }
      if ( ! data->timer ) {
        data->timer = lv_timer_create(thermostat_timer, 10000, data); // check temp diff every 10s
      }
      pwr_demand++;
    }
    else {
      digitalWrite(data->relay_pin, LOW);
      // delete timer if it exists
      if ( data->timer ) {
        lv_timer_del(data->timer);
        data->timer = NULL;
      }
      pwr_demand ? pwr_demand-- : NULL;
    }
  }
}










// TEMPERATURE DROP DOWN EVENT HANDLER ////////////////////////////////////////////////
void dropdown_event_handler(lv_event_t *e) {
    user_data_t * data = (user_data_t *)lv_event_get_user_data(e);
    if ( !data ) {
      Serial.println("dropdown_event_handler: no user data");
      return;
    }
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












// TEMP SENSOR FAULT DETECTOR /////////////////////////////////////////////////////////////////
void sensor_fault(lv_timer_t* timer) {
  user_data_t* data = (user_data_t*)timer->user_data;

  uint8_t faultArr[3];
  uint8_t index = 0;
  char faultMsg[20];

  if (combinedData.sensorData.temp1 == 999.0f) {
    faultArr[index] = 1;
    index++;
  }
  if (combinedData.sensorData.temp2 == 999.0f) {
    faultArr[index] = 2;
    index++;
  }
  if (combinedData.sensorData.temp4 == 999.0f) {
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

  // Delete the timer as it is called from within another timer
  lv_timer_del(timer);
}

// TEMPERATURE UPDATER //////////////////////////////////////////////////////
void update_temp(lv_timer_t *timer) {
    user_data_t *data = (user_data_t *)timer->user_data;
    char buf[20];

    // Clear previously set disabled state
    if (lv_obj_has_state(data->button, LV_STATE_DISABLED)) {
        lv_obj_clear_state(data->button, LV_STATE_DISABLED);
    }

    // check sensors for living room. If no avg_temp use single working sensor.
    if (data->relay_pin == RELAY2) {
        if (combinedData.sensorData.avg_temp != 999.0f) {
            snprintf(buf, sizeof(buf), "%.1f\u00B0C", combinedData.sensorData.avg_temp);
        } else if (combinedData.sensorData.temp1 != 999.0f) {
            snprintf(buf, sizeof(buf), "%.1f\u00B0C", combinedData.sensorData.temp1);
            lv_timer_create(sensor_fault, 8000, data);
        } else if (combinedData.sensorData.temp2 != 999.0f) {
            snprintf(buf, sizeof(buf), "%.1f\u00B0C", combinedData.sensorData.temp2);
            lv_timer_create(sensor_fault, 8000, data);
        } else if (combinedData.sensorData.temp4 != 999.0f) {
            snprintf(buf, sizeof(buf), "%.1f\u00B0C", combinedData.sensorData.temp4);
            lv_timer_create(sensor_fault, 8000, data);
        } else {
            snprintf(buf, sizeof(buf), "----");
            lv_timer_create(sensor_fault, 5000, data);

            // Button OFF if no sensor data
            if (lv_obj_has_state(data->button, LV_STATE_CHECKED)) {
                lv_event_send(data->button, LV_EVENT_RELEASED, NULL);
            }

            // Button DISABLED after being turned OFF (no need to test here as state disabled in beginning)
            lv_obj_add_state(data->button, LV_STATE_DISABLED);
        }
    }
    // check single sensor for shower room
    else {
        if (combinedData.sensorData.temp3 != 999.0f) {
            snprintf(buf, sizeof(buf), "%.1f\u00B0C", combinedData.sensorData.temp3);
        } else {
            snprintf(buf, sizeof(buf), "#3 --");
            // Button OFF if no sensor data
            if (lv_obj_has_state(data->button, LV_STATE_CHECKED)) {
                lv_event_send(data->button, LV_EVENT_RELEASED, NULL);
            }
            // Button DISABLED after being turned OFF (no need to test here as state disabled in beginning)
            lv_obj_add_state(data->button, LV_STATE_DISABLED);
        }
    }

    // Update the label text
    lv_label_set_text(data->label_obj, buf);
}







// CLEAR BMS FLAG CAN MSG EVENT HANDLER ////////////////////////////////////////////////////////////////////
void clear_bms_flag(lv_event_t * e) {
  lv_event_code_t code = lv_event_get_code(e);

  if(code == LV_EVENT_CLICKED) {
    canMsgData.send_mpo2 = true;
    canMsgData.msg_data[0] = 0x01;
    canMsgData.msg_cnt = 0; // reset the retry count
  }
}




// CREATE CLOCK //////////////////////////////////////////////////////////////////////////////
void create_clock_label(lv_obj_t* parent, clock_data_t* data) {

  data->clock_label = lv_label_create(parent);
  lv_obj_align(data->clock_label, LV_ALIGN_TOP_MID, 0, -5); // x=20 perfect if left aligned

  // text update timer
  lv_timer_create(clock_updater, 1000, data);
}

void clock_updater(lv_timer_t* timer) {
  clock_data_t *data = (clock_data_t *)timer->user_data;

  uint16_t h = 0;
  uint8_t m = 0;
  char t[11];
  char c[4] = {"hrs"};
  char state[17];

  // Zero
  if ( combinedData.canData.avgI == 0 ) {
    strcpy(t, "");;
  }

  // Discharge
  else if (combinedData.canData.avgI > 0) {
    h = combinedData.canData.soc / (combinedData.canData.avgI/10.0);
    m = (combinedData.canData.soc / (combinedData.canData.avgI/10.0) - h) * 60;
    strcpy(state, "Discharged in");
  }

  // Charge
  else if (combinedData.canData.avgI < 0) {
    h = (200 - combinedData.canData.soc) / (abs(combinedData.canData.avgI)/10.0);
    m = ((200 - combinedData.canData.soc) / (abs(combinedData.canData.avgI)/10.0) - h) * 60;
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







// REFRESH CAN LABEL DATA //////////////////////////////////////////////////////////////////////
void refresh_can_data(lv_timer_t* timer) {
  can_label_t *data = (can_label_t *)timer->user_data;
  char buf[50];

  switch (data->canDataType) {
    case CAN_DATA_TYPE_INT:
      snprintf(buf, sizeof(buf), "%s %d %s", data->label_prefix, *(data->canDataProperty.intData), data->label_unit);
      break;
    case CAN_DATA_TYPE_FLOAT:
      snprintf(buf, sizeof(buf), "%s %.1f %s", data->label_prefix, *(data->canDataProperty.floatData), data->label_unit);
      break;
    case CAN_DATA_TYPE_DOUBLE_FLOAT:
      snprintf(buf, sizeof(buf), "%s %.2f %s", data->label_prefix, *(data->canDataProperty.floatData), data->label_unit);
      break;
    case CAN_DATA_TYPE_BYTE:
      snprintf(buf, sizeof(buf), "%s %d %s", data->label_prefix, *(data->canDataProperty.byteData), data->label_unit);
      break;
    default:
      snprintf(buf, sizeof(buf), "%s %s %s", data->label_prefix, "Unknown", data->label_unit && strlen(data->label_unit) > 0 ? data->label_unit : "");
      break;
  }
  lv_label_set_text(data->label_obj, buf);
}

// CREATE CAN LABELS ////////////////////////////////////////////////////////////////////////////////////////////////////
void create_can_label(lv_obj_t* parent, const char* label_prefix, const char* label_unit, void* canDataProperty, can_data_type_t canDataType, int x_pos, int y_pos, can_label_t* data) {
    if (data) {
        // Update instance with user data
        data->label_obj = lv_label_create(parent);
        data->label_prefix = label_prefix;
        data->label_unit = label_unit;
        data->canDataType = canDataType; // Set the type

        // Assign the appropriate pointer type based on canDataType
        switch (canDataType) {
            case CAN_DATA_TYPE_INT:
                data->canDataProperty.intData = (int*)canDataProperty;
                break;
            case CAN_DATA_TYPE_FLOAT:
                data->canDataProperty.floatData = (float*)canDataProperty;
                break;
            case CAN_DATA_TYPE_DOUBLE_FLOAT:
                data->canDataProperty.floatData = (float*)canDataProperty;
                break;
            case CAN_DATA_TYPE_BYTE:
                data->canDataProperty.byteData = (byte*)canDataProperty;
                break;
            default:
                break;
        }

        lv_obj_set_pos(data->label_obj, x_pos, y_pos);
        // create refresh can data timer
        lv_timer_create(refresh_can_data, 200, data);
    }
    else {
      Serial.println("Error: Unable to allocate memory for CAN label data.");
    }
}

// Function to sign value
int16_t signValue(uint16_t canValue) {
    int16_t signedValue = (canValue > 32767) ? canValue - 65536 : canValue;
    return signedValue;
}

// Sort CAN bus data
void sort_can() {

    if (canMsgData.rxId == 0x3B) {
        combinedData.canData.instU = ((canMsgData.rxBuf[0] << 8) + canMsgData.rxBuf[1]) / 10.0;
        combinedData.canData.instI = (signValue((canMsgData.rxBuf[2] << 8) + canMsgData.rxBuf[3])) / 10.0; // orion2jr issue: unsigned value despite ticket as signed
        //combinedData.canData.absI = ((canMsgData.rxBuf[4] << 8) + canMsgData.rxBuf[5]) / 10.0; // orion2jr issue: set signed and -32767 to fix
        combinedData.canData.soc = canMsgData.rxBuf[6] / 2;
    }
    if (canMsgData.rxId == 0x6B2) {
        combinedData.canData.lC = ((canMsgData.rxBuf[0] << 8) + canMsgData.rxBuf[1]) / 10000.00;
        combinedData.canData.hC = ((canMsgData.rxBuf[2] << 8) + canMsgData.rxBuf[3]) / 10000.00;
        combinedData.canData.h = canMsgData.rxBuf[4];
        combinedData.canData.cc = (canMsgData.rxBuf[5] << 8) + canMsgData.rxBuf[6];
    }
    if (canMsgData.rxId == 0x0A9) {
        combinedData.canData.ry = canMsgData.rxBuf[0];
        combinedData.canData.ccl = canMsgData.rxBuf[1];
        combinedData.canData.dcl = canMsgData.rxBuf[2];
        combinedData.canData.ah = ((canMsgData.rxBuf[3] << 8) + canMsgData.rxBuf[4]) / 10.0;
        combinedData.canData.avgI = (signValue((canMsgData.rxBuf[5] << 8) + canMsgData.rxBuf[6])) / 10.0; // orion2jr issue: unsigned value despite ticket as signed
    }
    if (canMsgData.rxId == 0x0BD) {
        combinedData.canData.fu = (canMsgData.rxBuf[0] << 8) + canMsgData.rxBuf[1];
        combinedData.canData.hT = canMsgData.rxBuf[2];
        combinedData.canData.lT = canMsgData.rxBuf[3];
        combinedData.canData.cu = canMsgData.rxBuf[4];
        combinedData.canData.st = (canMsgData.rxBuf[5] << 8) + canMsgData.rxBuf[6];
    }
    if (canMsgData.rxId == 0x0BE) {
        combinedData.canData.hCid = canMsgData.rxBuf[0];
        combinedData.canData.lCid = canMsgData.rxBuf[1];
        combinedData.canData.hs = canMsgData.rxBuf[2];
        combinedData.canData.ct = canMsgData.rxBuf[3];
    }
    combinedData.canData.p = combinedData.canData.avgI * combinedData.canData.instU;
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
void screen_touch(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if(code == LV_EVENT_CLICKED) {
    previous_touch_ms = millis();
    brightness = 70;
    backlight.set(brightness);
  }
}






// CREATE STATUS LABELS ////////////////////////////////////////////////////////////
void create_status_label(const char* label_text, bms_status_data_t* data, bool finished = false) {

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

    static bool balancing_label_showing = false; // Static to remember between calls

    bool charge_only = false;

    int8_t flag_index = -1; // initialise as -1 as this in array index

    // Clear all status labels initially
    for (uint8_t i = 0; i < (sizeof(data->status_label) / sizeof(data->status_label[0])); i++) {
      if (data->status_label[i]) {
        lv_obj_del(data->status_label[i]); // Properly delete label
        data->status_label[i] = NULL; // Set pointer to NULL
      }
    }

    // BMS flags
    if ((combinedData.canData.fu & 0x0100) == 0x0100) { create_status_label("Internal Hardware Fault", data); flag_index++; }
    if ((combinedData.canData.fu & 0x0200) == 0x0200) { create_status_label("Internal Cell Comm Fault", data); flag_index++; }
    if ((combinedData.canData.fu & 0x0400) == 0x0400) { create_status_label("Weak Cell Fault", data); flag_index++; }
    if ((combinedData.canData.fu & 0x0800) == 0x0800) { create_status_label("Low Cell Voltage", data); flag_index++; }
    if ((combinedData.canData.fu & 0x1000) == 0x1000) { create_status_label("Open Wire Fault", data); flag_index++; }
    if ((combinedData.canData.fu & 0x2000) == 0x2000) { create_status_label("Current Sensor Fault", data); flag_index++; }
    if ((combinedData.canData.fu & 0x4000) == 0x4000) { create_status_label("Abnormal SOC Behavior", data); flag_index++; }
    if ((combinedData.canData.fu & 0x8000) == 0x8000) { create_status_label("Pack Too Hot Fault", data); flag_index++; }
    if ((combinedData.canData.fu & 0x0001) == 0x0001) { create_status_label("Weak Pack Fault", data); flag_index++; }
    if ((combinedData.canData.fu & 0x0002) == 0x0002) { create_status_label("External Thermistor Fault", data); flag_index++; }
    if ((combinedData.canData.fu & 0x0004) == 0x0004) { create_status_label("Charge Relay Failure", data); flag_index++; }
    if ((combinedData.canData.fu & 0x0008) == 0x0008) { create_status_label("Discharge Relay Fault", data); flag_index++; }
    if ((combinedData.canData.fu & 0x0010) == 0x0010) { create_status_label("Safety Relay Fault", data); flag_index++; }
    if ((combinedData.canData.fu & 0x0020) == 0x0020) { create_status_label("CAN communication Fault", data); flag_index++; }
    if ((combinedData.canData.fu & 0x0040) == 0x0040) { create_status_label("Internal Thermistor Fault", data); flag_index++; }
    if ((combinedData.canData.fu & 0x0080) == 0x0080) { create_status_label("Internal Logic Fault", data); flag_index++; }

    // Failsafe status
    if ((combinedData.canData.st & 0x0001) == 0x0001) { create_status_label("Voltage Failsafe", data); flag_index++; }
    if ((combinedData.canData.st & 0x0002) == 0x0002) { create_status_label("Current Failsafe", data); flag_index++; }
    if ((combinedData.canData.st & 0x0004) == 0x0004) { create_status_label("Relay Failsafe", data); flag_index++; }
    if ((combinedData.canData.st & 0x0010) == 0x0010) { create_status_label("Charge Interlock Failsafe", data); flag_index++; }
    if ((combinedData.canData.st & 0x0020) == 0x0020) { create_status_label("Thermistor B-value Table Invalid", data); flag_index++; }
    if ((combinedData.canData.st & 0x0040) == 0x0040) { create_status_label("Input Power Supply Failsafe", data); flag_index++; }
    if ((combinedData.canData.st & 0x0100) == 0x0100) { create_status_label("Relays Opened under Load Failsafe", data); flag_index++; }
    if ((combinedData.canData.st & 0x1000) == 0x1000) { create_status_label("Polarization Model 1 Active", data); flag_index++; }
    if ((combinedData.canData.st & 0x2000) == 0x2000) { create_status_label("Polarization Model 2 Active", data); flag_index++; }
    if ((combinedData.canData.st & 0x8000) == 0x8000) { create_status_label("Charge Mode Activated over CANBUS", data); flag_index++; }

    // Cell balancing check at end ensures higher importance messages appear above
    if ((combinedData.canData.st & 0x0008) == 0x0008) {
      if ( balancing_label_showing ) {
        create_status_label("", data); // create blank label
        balancing_label_showing = false;
      }
      else {
        create_status_label("Cell Balancing Active", data);
        balancing_label_showing = true;
      }

      flag_index++;
      
      // If balancing only button should remain hidden
      if ( flag_index == 0 ) {
        charge_only = true;
      }
    }

  // Show title and button
  if ( flag_index > -1 ) {

    if ( lv_obj_has_flag(data->title_label, LV_OBJ_FLAG_HIDDEN) ) {
      lv_obj_clear_flag(data->title_label, LV_OBJ_FLAG_HIDDEN); // Remove hide flag
    }
    // Show button if more than just balancing flag is present
    if ( lv_obj_has_flag(data->button, LV_OBJ_FLAG_HIDDEN) && ! charge_only ) {
      lv_obj_clear_flag(data->button, LV_OBJ_FLAG_HIDDEN); // Remove hide flag
    }
  }

  // Hide title and button if no flags are present
  else {
    if ( ! lv_obj_has_flag(data->title_label, LV_OBJ_FLAG_HIDDEN) ){
       lv_obj_add_flag(data->title_label, LV_OBJ_FLAG_HIDDEN);
    }
    if ( ! lv_obj_has_flag(data->button, LV_OBJ_FLAG_HIDDEN) ) {
       lv_obj_add_flag(data->button, LV_OBJ_FLAG_HIDDEN);
    }
    // refresh screen to remove scroll bar
    //lv_event_send(lv_obj_get_parent(data->parent), LV_EVENT_REFRESH, NULL); // does nothing for either scr or cont obj
  }

  // run the function with true argument to tell it we are finished checking bms for messages to reset function index
  create_status_label("", data, true);
}

// CREATE BMS STATUS LABELS //////////////////////////////////////////////////////
void create_bms_status_label(lv_obj_t* parent, lv_coord_t y, bms_status_data_t* data) {
    if (data) {
        data->parent = parent;
        data->y = y;

        // Create title label with underline style (hidden initially)
        data->title_label = lv_label_create(parent);
        lv_style_set_text_decor(&style, LV_TEXT_DECOR_UNDERLINE); // set underline style
        lv_obj_add_style(data->title_label, &style, 0); // add style
        lv_label_set_text(data->title_label, "BMS Status Messages");
        lv_obj_align(data->title_label, LV_ALIGN_TOP_MID, 0, y);
        lv_obj_add_flag(data->title_label, LV_OBJ_FLAG_HIDDEN);

        // Initialize button (hidden initially)
        data->button = lv_btn_create(parent);
        lv_obj_t* btn_label = lv_label_create(data->button);
        lv_label_set_text(btn_label, "Clear BMS Flags");
        lv_obj_add_event_cb(data->button, clear_bms_flag, LV_EVENT_CLICKED, NULL);
        lv_obj_add_flag(data->button, LV_OBJ_FLAG_HIDDEN);

        // Refresh status labels every second
        lv_timer_create(refresh_bms_status_data, 1000, data);

    }
    else {
        // Handle memory allocation failure
        Serial.println("Error: Unable to allocate memory for BMS flag data.");
    }
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

  // Initialise the style variable
  lv_style_init(&style);

  // Create two columns on active screen, grid 2x1
  static lv_coord_t col_dsc[] = {370, 370, LV_GRID_TEMPLATE_LAST};
  static lv_coord_t row_dsc[] = {430, 430, LV_GRID_TEMPLATE_LAST};
  lv_obj_t * parent = lv_obj_create(lv_scr_act());
  lv_obj_set_grid_dsc_array(parent, col_dsc, row_dsc);
  lv_obj_set_size(parent, Display.width(), Display.height());
  lv_obj_set_style_bg_color(parent, lv_color_hex(0x03989e), LV_PART_MAIN);
  lv_obj_center(parent);

  // initialise container object
  lv_obj_t * cont;

  // create left column container instance
  cont = lv_obj_create(parent);
  canMsgBoxData.parent = cont;
  lv_obj_add_flag(cont, LV_OBJ_FLAG_CLICKABLE); // add event handling
  lv_obj_add_event_cb(cont, can_msgbox, LV_EVENT_CLICKED, &canMsgBoxData); // add event handler function
  lv_obj_add_event_cb(cont, screen_touch, LV_EVENT_ALL, NULL);
  lv_obj_set_grid_cell(cont, LV_GRID_ALIGN_STRETCH, 0, 1,
                        LV_GRID_ALIGN_STRETCH, 0, 1);

  // Create charge/discharge clock
  create_clock_label(cont, &clockData);

  // check for sunrise by reading BMS charge enable signal from CANbus and sending MPO#1 signal to trip relay if flapping detected
  lv_timer_create(mppt_delay, 1000, NULL);

  // CCL limit checker for Solar Panel Relay control through CANbus
  lv_timer_create(ccl_check, 1000, NULL);

  // Create labels for CAN data
  create_can_label(cont, "SOC", "%", &(combinedData.canData.soc), CAN_DATA_TYPE_BYTE, 20, 20, &canLabel[0]);
  create_can_label(cont, "Current", "A", &(combinedData.canData.instI), CAN_DATA_TYPE_FLOAT, 180, 20, &canLabel[1]);
  create_can_label(cont, "Voltage", "V", &(combinedData.canData.instU), CAN_DATA_TYPE_FLOAT, 20, 50, &canLabel[2]);
  create_can_label(cont, "Power", "W", &(combinedData.canData.p), CAN_DATA_TYPE_INT, 180, 50, &canLabel[3]);
  /*create_can_label(cont, "High Cell ID", "", &(combinedData.canData.hCid), CAN_DATA_TYPE_BYTE, 20, 80, &canLabel[4]);
  create_can_label(cont, "High Cell", "V", &(combinedData.canData.hC), CAN_DATA_TYPE_DOUBLE_FLOAT, 180, 80, &canLabel[5]);
  create_can_label(cont, "Low Cell ID", "", &(combinedData.canData.lCid), CAN_DATA_TYPE_BYTE, 20, 110, &canLabel[6]);
  create_can_label(cont, "Low Cell", "V", &(combinedData.canData.lC), CAN_DATA_TYPE_DOUBLE_FLOAT, 180, 110, &canLabel[7]);
  create_can_label(cont, "Discharge Current Limit          ", "A", &(combinedData.canData.dcl), CAN_DATA_TYPE_BYTE, 20, 160, &canLabel[8]);
  create_can_label(cont, "Charge Current Limit                ", "A", &(combinedData.canData.ccl), CAN_DATA_TYPE_BYTE, 20, 190, &canLabel[9]);
  create_can_label(cont, "Cycles", "", &(combinedData.canData.cc), CAN_DATA_TYPE_INT, 20, 240, &canLabel[10]);
  create_can_label(cont, "Health", "%", &(combinedData.canData.h), CAN_DATA_TYPE_BYTE, 180, 240, &canLabel[11]);*/
  create_can_label(cont, "Capacity", "Ah", &(combinedData.canData.ah), CAN_DATA_TYPE_FLOAT, 20, 80, &canLabel[4]); //270, &canLabel[12]);
  create_can_label(cont, "Internal Heatsink Temperature", "\u00B0C", &(combinedData.canData.hs), CAN_DATA_TYPE_BYTE, 20, 110, &canLabel[5]); //300, &canLabel[13]);
  
  // Display bms messages arg2: y_pos
  create_bms_status_label(cont, 135, &bmsStatusData); // old y_pos 345
  
  // create right column container instance
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
    memcpy(canMsgData.rxBuf, msg.data, canMsgData.len);
    sort_can();

    // send CAN if commanded
    if ( canMsgData.send_mpo1 || canMsgData.send_mpo2 ) {

      CanMsg send_msg(CanStandardId(canMsgData.CAN_ID), sizeof(canMsgData.msg_data), canMsgData.msg_data);

      // retry if send failed (only for mpo2 as mpo1 is timed)
      int const rc = CAN.write(send_msg);
      if ( rc <= 0 && canMsgData.msg_cnt < 3 ) {
        Serial.print("CAN.write(...) failed with error code ");
        Serial.println(rc);
        canMsgData.msg_cnt++;
      }
      // stop sending to mpo2 after 3 retries
      else if ( canMsgData.send_mpo2 ) {
        canMsgData.send_mpo2 = false; // stop sending after 3 tries
        canMsgData.msg_data[0] = 0; // clear send data
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
  delay(4); // lvgl recommends 5ms delay for display and I have measured average performance of 1ms with or without serial
}
