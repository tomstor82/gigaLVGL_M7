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
#define RELAY1 64   // living space
#define RELAY2 62   // shower room
#define RELAY3 60   // water heater
#define RELAY4 58   // inverter

// INVESTIGATE ABS_AMP FROM ORION
//  CANBUS data Identifier List
//  ID 0x03B BYT0+1:INST_VOLT BYT2+3:INST_AMP BYT4+5:ABS_AMP BYT6:SOC **** ABS_AMP from OrionJr errendous ****
//  ID 0x6B2 BYT0+1:LOW_CELL BYT2+3:HIGH_CELL BYT4:HEALTH BYT5+6:CYCLES
//  ID 0x0A9 BYT0:RELAY_STATE BYT1:CCL BYT2:DCL BYT3+4:PACK_AH BYT5+6:AVG_AMP
//  ID 0x0BD BYT0+1:CUSTOM_FLAGS BYT2:HI_TMP BYT3:LO_TMP BYT4:COUNTER BYT5:BMS_STATUS
//  ID 0x0BE BYT0:HI_CL_ID BYT1:LO_CL_ID BYT2:INT_HEATSINK

// CANBUS receive data
uint32_t rxId;
uint8_t len = 0;
uint8_t rxBuf[8];

// CANBUS send data MPI through MPO
static uint8_t const CAN_ID = 0x20; // Must be different from other devices on CANbus
uint8_t const msg_data[] = {0x002, 0x01}; // Length set to 1 byte in BMS, can add to this array if needed
static uint8_t msg_cnt = 0;

// CAN RX INFO DISPLAY DATA
uint8_t pos_x = 100;
uint8_t pos_y = 20;
uint8_t line_gap = 50;

// Define named structs as data types
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
// Define named structs as data types
// CanData struct
struct CanData {

    int p;                  // watt calculated by script

    float instU;             // Voltage - multiplied by 10
    float instI;             // Current - multiplied by 10 - negative value indicates charge
    float avgI;             // Average current for clock and sun symbol calculations
    float absI;             // Absolute Current NOT WORKING CORRECTLY
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
    byte st;                // BMS Status
    byte h;                 // Health
    byte hCid;              // High Cell ID
    byte lCid;              // Low Cell ID

    int fu;                 // BMS faults
    int fs;                 // Fault messages & status from CANBus for displaying wrench icon
    int cc;                 // Total pack cycles

    float kw;               // Active power 0,1kW
    byte hs;                // Internal Heatsink
    
    MSGPACK_DEFINE_ARRAY(instU, instI, soc, hC, lC, h, fu, hT, lT, ah, ry, dcl, ccl, ct, st, cc, fs, avgI, hCid, lCid, p, absI, kw, hs);
};
// Define an enumeration for the different data types.
typedef enum {
    CAN_DATA_TYPE_INT,
    CAN_DATA_TYPE_FLOAT,
    CAN_DATA_TYPE_DOUBLE_FLOAT, // 2 decimals
    CAN_DATA_TYPE_BYTE
} can_data_type_t;

// define struct for function user-data
typedef struct {
  lv_obj_t* container;
  lv_obj_t* my_btn;
  lv_obj_t* label_obj;
  lv_obj_t* label_text;
  uint8_t relay_pin;
  uint8_t y_offset;
  unsigned long timeout_ms;
  uint8_t dcl_limit;
  uint8_t set_temp = 20; // should match value of dropdown default index
  union {
    int* intData;
    float* floatData;
    byte* byteData;
  } canDataProperty;
  can_data_type_t canDataType;
  const char* label_prefix; // To store label text prefix
  const char* label_unit; // Label unit e.g V,A or W
} user_data_t;

struct CombinedData {
  SensorData sensorData;
  CanData canData;
};

// declare global instances
static CombinedData combinedData;

// global variables * 8bits=256 16bits=65536 32bits=4294967296 (millis size)
static lv_obj_t* inv_btn;
static lv_timer_t* thermostat = NULL;

<<<<<<< HEAD
=======
uint8_t brightness = 70;
uint32_t previous_touch_ms;
uint16_t touch_timeout_ms = 20000; // 20s before screen dimming
uint8_t pwr_demand = 0;
//uint32_t previous_sweep_ms;
uint8_t pre_start_avgI;
uint32_t hot_water_interval_ms = 900000; // 15 min
uint16_t inverter_startup_ms = 22000; // 22s startup required before comparing current flow
uint32_t sweep_interval_ms = 180000; // 3 minute sweep interval reduces standby consumption from 75Wh to around 12,5Wh -84%

>>>>>>> dev
//**************************************************************************************//
// NEED TO ADD DTC FLAGS AND SOLAR HOT WATER OVERRIDE, PERHAPS SIMULATE A BUTTON CLICK? 
// ADD RELAY CODE FOR INVERTER RELAY CONTROL (TIMER/CLOCK,SOC,DCL,avgI)                 
//**************************************************************************************//

// CREATE BUTTON INSTANCE
void create_button(lv_obj_t *parent, const char *label_text, uint8_t relay_pin, lv_coord_t y_offset, uint8_t dcl_limit, unsigned long timeout_ms = 0) {
  
  // configure relay pins
  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, LOW); // initialise pin LOW
  
  // Allocate memory for userdata for each button instance
  user_data_t * data = (user_data_t *)malloc(sizeof(user_data_t));
  // Update user data with received arguments
  data->relay_pin = relay_pin;
  data->y_offset = y_offset;
  data->dcl_limit = dcl_limit;
  data->timeout_ms = timeout_ms;

  // create button object and add to struct
  data->my_btn = lv_btn_create(parent); // IMPORTANT TO STORE BUTTON IN USERDATA STRUCT - ELSE CLEARING BUTTONS WON'T WORK
  if ( relay_pin == RELAY4 ) { // adding inverter button to global variable as it needs to be triggered by the other buttons
    inv_btn = data->my_btn;
  }
  lv_obj_set_pos(data->my_btn, 10, y_offset);
  lv_obj_t *label = lv_label_create(data->my_btn);
  lv_label_set_text(label, label_text);
  lv_obj_center(label);
  lv_obj_add_flag(data->my_btn, LV_OBJ_FLAG_CHECKABLE);

  // create temperature dropdown and dynamic temperature labels for thermostat buttons
  if ( ! timeout_ms ) {
    // create label and update the user data member for access within timer to allow only to update text not object
    data->label_obj = lv_label_create(lv_obj_get_parent(data->my_btn));
    create_temperature_dropdown(parent, data);
    // create time updated temperature labels
    lv_timer_create(update_temp, 10000, data);
    lv_obj_set_pos(data->label_obj, 180, y_offset + 13);
  }
  
  // Disable all buttons by DCL limit
  lv_timer_create(dcl_check, 10000, data); // check every 10s
    
  // create event handlers with custom user_data
  if ( timeout_ms ) { // hot water and inverter
    lv_obj_add_event_cb(data->my_btn, timer_event_handler, LV_EVENT_ALL, data);
  }
  else { // heaters
    lv_obj_add_event_cb(data->my_btn, thermostat_event_handler, LV_EVENT_ALL, data);
  }
}

// DCL CHECK TIMER ////////////////////////////////////////////////////////////////////////////
void dcl_check(lv_timer_t * timer) {
  user_data_t * data = (user_data_t *)timer->user_data;
  // create scrolling label and disable button
  if ( combinedData.canData.dcl < data->dcl_limit ) {
    lv_obj_t *label = lv_label_create(lv_obj_get_parent(data->my_btn));
    lv_obj_add_state(data->my_btn, LV_STATE_DISABLED);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_width(label, 150);
    lv_obj_set_pos(label, 10, data->y_offset + 50);
    lv_label_set_text(label, "Please Charge Battery                                            "); // spaces to allow a pause
  }
}

// HOT WATER AND INVERTER EVENT HANDLER ////////////////////////////////////////////////////
void timer_event_handler(lv_event_t * e) {
  user_data_t * data = (user_data_t *)lv_event_get_user_data(e);
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * obj = lv_event_get_target(e);
  if(code == LV_EVENT_CLICKED) {
    LV_UNUSED(obj);
    if ( lv_obj_has_state(data->my_btn, LV_STATE_CHECKED) ) {
      digitalWrite(data->relay_pin, HIGH);
      if ( data->relay_pin == RELAY4 ) { // only for inverter for sweeping
        //previous_sweep_ms = millis();
        pre_start_avgI = combinedData.canData.avgI;
        Serial.print("DEBUG before start avgI: ");
        Serial.println(pre_start_avgI);
      }
      else pwr_demand++; // only for hot water
      // Create timer obj to reset on conditions
      lv_timer_create(switch_off, data->timeout_ms, data); // want to reset timer once excess solar to hot water is activated
    }
    else {
      digitalWrite(data->relay_pin, LOW);
      if ( ! data->relay_pin == RELAY4 ) { // only for hot water
        pwr_demand ? pwr_demand-- : NULL; 
      }
    }
  }
}

// INVERTER SWEEP TIMER ///////////////////////////////////////////////////////////////////
void sweep_timer (lv_timer_t* timer) {
  user_data_t* data = (user_data_t *)timer->user_data; // required to identify button
  // not sure i can call switch_off function or need to create another instance. perhaps best to simulate button click
  lv_event_send(data->my_btn, LV_EVENT_CLICKED, NULL); // SIMULATE BUTTON CLICK TO START TIMER AGAIN
  // delete timer so it only runs once
  lv_timer_del(timer);
}


// SWITCH OFF TIMER ///////////////////////////////////////////////////////////////////////
void switch_off(lv_timer_t * timer) {
  user_data_t * data = (user_data_t *)timer->user_data;
  bool on = false;
  
  // inverter on for 20s to check demand or current flow else off and timer for 3 minutes before restarting this timer (sweep)
  // inverter - reset timer if demand or negative avgI
  if ( data->relay_pin == RELAY4 ) {
    if ( pwr_demand || combinedData.canData.avgI < -5 && combinedData.canData.soc > 50 ) { // keep inverter on above 50% SOC if heaters or hot water demand or charge current
      on = true;
      Serial.println("Inverter ON due Button demand or charge");
    }
    // inverter - reset timer if avgI has risen by more than 2A after inverter start (2A is standby usage)
    else if ( pre_start_avgI + 2 < combinedData.canData.avgI ) {
      on = true;
      Serial.print("Current before start: ");
      Serial.println(pre_start_avgI);
      Serial.print("Current now: ");
      Serial.println(combinedData.canData.avgI);
    }
  }
  // hot water tank - reset timer if excess power generation
  else if ( combinedData.canData.ccl < 10 || combinedData.canData.avgI < 0 ) {
    on = true;
  }

  if (on) {
    // we start this timer again
    Serial.print("ON Timer Reset, demand var: ");
    Serial.println(pwr_demand);
    lv_timer_reset(timer);
  }
  // turn off relay
  else {
    digitalWrite(data->relay_pin, LOW);
    // delete timer if relay is turned off
    lv_timer_del(timer);
    // Inverter sweep timer starting after turning off relay
    if ( data->relay_pin == RELAY4 ) {
      Serial.print("OFF as there's no demand, starting sweep timer. Current avgI: ");
      Serial.println(combinedData.canData.avgI);
      lv_timer_create(sweep_timer, sweep_interval_ms, data);
    }
    else { // clear button flag for hot water
      lv_obj_clear_state(data->my_btn, LV_STATE_CHECKED);
      pwr_demand ? pwr_demand-- : NULL;
    }
  }
}

// THERMOSTAT EVENT HANDLER /////////////////////////////////////////////////////////
void thermostat_event_handler(lv_event_t * e) {
  user_data_t * data = (user_data_t *)lv_event_get_user_data(e);
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * obj = lv_event_get_target(e);
  
  if(code == LV_EVENT_CLICKED) {
    LV_UNUSED(obj);
    if ( lv_obj_has_state(data->my_btn, LV_STATE_CHECKED) ) {
      if ( ! thermostat ) {
        thermostat = lv_timer_create(thermostat_timer, 10000, data); // check temp diff every 10s
      }
      pwr_demand++; // set global var to enable inverter if heater or hot water activated
    }
    else {
      digitalWrite(data->relay_pin, LOW);
      pwr_demand ? pwr_demand-- : NULL;
      if ( thermostat ) {
        lv_timer_del(thermostat);
        thermostat = NULL;
      }
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
  //uint8_t y_offset = data->y_offset;
  lv_dropdown_set_options(dd,
    "5\u00B0C\n18\u00B0C\n19\u00B0C\n20\u00B0C\n21\u00B0C\n22\u00B0C\n23\u00B0C");
    
  // set user data
  lv_dropdown_set_selected(dd, 3); // default index to be displayed. value set_temp in struct
  lv_obj_set_user_data(dd, data);
  lv_obj_add_event_cb(dd, dropdown_event_handler, LV_EVENT_VALUE_CHANGED, data);
  
  // place roller
  lv_obj_set_pos(dd, 255, data->y_offset - 1);
  lv_obj_set_width(dd, 80);
}

// THERMOSTAT TIMER ////////////////////////////////////////////////////////////////
void thermostat_timer(lv_timer_t * timer) {
  user_data_t * data = (user_data_t *)timer->user_data;
  // turn inverter on if off
  if ( ! lv_obj_has_state(inv_btn, LV_STATE_CHECKED) ) {
    lv_event_send(*inv_btn, LV_EVENT_CLICKED, NULL); // send click event to start inverter
    Serial.println("DEBUG Thermostat Timer - trying to send event to start inverter");
  }

  // ceiling heater thermostat
  if ( data->relay_pin == RELAY1 ) {
    // Close relay if temperature is below selected and button has been pressed
    if ( combinedData.sensorData.avg_temp < data->set_temp && lv_obj_has_state(data->my_btn, LV_STATE_CHECKED) ) {
    digitalWrite(data->relay_pin, HIGH);
    }
    // Open relay when temperature is higher or equal to selected
    else {
      digitalWrite(data->relay_pin, LOW);
    }
  }
  // shower heater thermostat
  else {
    // Close relay if temperature is below selected and button has been pressed
    if ( combinedData.sensorData.temp3 < data->set_temp && lv_obj_has_state(data->my_btn, LV_STATE_CHECKED) ) {
      digitalWrite(data->relay_pin, HIGH);
    }
    // Open relay when temperature is higher or equal to selected
    else {
      digitalWrite(data->relay_pin, LOW);
    }
  }
}

// DISPLAY TEMP TIMER ////////////////////////////////////////////////////////////////////////
void update_temp(lv_timer_t *timer) {
  user_data_t * data = (user_data_t *)timer->user_data;
  char buf[10];
  // ceiling heater use avg temp
  if ( data->relay_pin == RELAY1 ) {
    snprintf(buf, sizeof(buf), "%.1f\u00B0C", combinedData.sensorData.avg_temp); // use the global shared instance for temp and humidity
  }
  // shower heater use temp3
  else {
    snprintf(buf, sizeof(buf), "%.1f\u00B0C", combinedData.sensorData.temp3);
  }
  // update label text only and not the label object
  lv_label_set_text(data->label_obj, buf);
}

// CLEAR CAN EVENT HANDLER ////////////////////////////////////////////////////////////////////
void clear_bms_fault(lv_event_t * e) {
  lv_obj_t * obj = lv_event_get_target(e);
  if ( lv_obj_has_state(obj, LV_EVENT_CLICKED) ) {
    msg_cnt = 1;
    Serial.println("Sending CAN message");
  }
}
// REFRESH CAN LABEL DATA //////////////////////////////////////////////////////////////////////
void refresh_can_data(lv_timer_t* timer) {
    user_data_t *data = (user_data_t *)timer->user_data;
    char buf[50];

    switch (data->canDataType) {
        case CAN_DATA_TYPE_INT:
            snprintf(buf, sizeof(buf), "%s %d %s", data->label_prefix, *(data->canDataProperty.intData), data->label_unit);
            break;
        //case CAN_DATA_TYPE_INT_2:
            //snprintf(buf, sizeof(buf), "%s %d %s", data->label_prefix, (*(data->canDataProperty.intData) / 10.0), data->label_unit);
            //break;
        case CAN_DATA_TYPE_FLOAT:
            snprintf(buf, sizeof(buf), "%s %.1f %s", data->label_prefix, *(data->canDataProperty.floatData), data->label_unit); // possible to set 1 or 2 decimals on check?
            break;
        case CAN_DATA_TYPE_DOUBLE_FLOAT:
            snprintf(buf, sizeof(buf), "%s %.2f %s", data->label_prefix, *(data->canDataProperty.floatData), data->label_unit); // possible to set 1 or 2 decimals on check?
            break;
        case CAN_DATA_TYPE_BYTE:
            snprintf(buf, sizeof(buf), "%s %d %s", data->label_prefix, *(data->canDataProperty.byteData), data->label_unit);
            break;
        default:
            snprintf(buf, sizeof(buf), "%s %s %s", data->label_prefix, "Unknown", data->label_unit);
            break;
    }

    lv_label_set_text(data->label_obj, buf);
}

// CREATE CAN LABELS ////////////////////////////////////////////////////////////////////////////////////////////////////
void create_can_label(lv_obj_t* parent, const char* label_prefix, const char* label_unit, void* canDataProperty, can_data_type_t canDataType, int x_pos, int y_pos) {
    // Allocate memory for new user data instance
    user_data_t *data = (user_data_t *)malloc(sizeof(user_data_t));
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
}

// CONVERT UNSIGNED TO SIGNED FUNCTION ////////////////////////////////////////
int16_t signValue(uint16_t canValue) {
  int16_t signedValue = (canValue > 32767) ? canValue - 65536 : canValue;
  return signedValue;
}

// SORT CANBUS DATA ////////////////////////////////////////////////////////////
void sort_can() {

    // I WOULD LIKE TO COMPARE CHECKSUM BUT CPP STD LIBRARY NOT AVAILABLE I BELIEVE
    if (rxId == 0x3B) {
        combinedData.canData.instU = ((rxBuf[0] << 8) + rxBuf[1]) / 10.0;
        combinedData.canData.instI = (signValue((rxBuf[2] << 8) + rxBuf[3])) / 10.0; // orion2jr issue: unsigned value despite ticket as signed
        combinedData.canData.absI = ((rxBuf[4] << 8) + rxBuf[5]) / 10.0; // orion2jr issue: set signed and -32767 to fix
        combinedData.canData.soc = rxBuf[6] / 2;
    }
    if(rxId == 0x6B2) {
        combinedData.canData.lC = ((rxBuf[0] << 8) + rxBuf[1]) / 10000.00;
        combinedData.canData.hC = ((rxBuf[2] << 8) + rxBuf[3]) / 10000.00;
        combinedData.canData.h = rxBuf[4];
        combinedData.canData.cc = (rxBuf[5] << 8) + rxBuf[6];
    }
    if(rxId == 0x0A9) {    
        combinedData.canData.ry = rxBuf[0];
        combinedData.canData.ccl = rxBuf[1];
        combinedData.canData.dcl = rxBuf[2];
        combinedData.canData.ah = ((rxBuf[3] << 8) + rxBuf[4]) / 10.0;
        combinedData.canData.avgI = (signValue((rxBuf[5] << 8) + rxBuf[6])) / 10.0; // orion2jr issue: unsigned value despite ticket as signed
    }
    if(rxId == 0x0BD) {
        combinedData.canData.fu = (rxBuf[0] << 8) + rxBuf[1];
        combinedData.canData.hT = rxBuf[2];
        combinedData.canData.lT = rxBuf[3];
        combinedData.canData.ct = rxBuf[4];
        combinedData.canData.st = rxBuf[5];
    }
    if(rxId == 0x0BE) {
        combinedData.canData.hCid = rxBuf[0];
        combinedData.canData.lCid = rxBuf[1];
        combinedData.canData.hs = rxBuf[2];
        combinedData.canData.kw = ((rxBuf[3] << 8) + rxBuf[4]) / 10.0;
    }
    combinedData.canData.p = combinedData.canData.avgI * combinedData.canData.instU;
}

// RETRIEVE DATA FROM M4 CORE //////////////////////////////////////////////////////////
void retrieve_M4_data() {
    // Call the RPC function to get sensor data
      combinedData.sensorData = RPC.call("getSensorData").as<SensorData>();
      //combinedData.canData = RPC.call("getCanData").as<CanData>(); // until can issue on m4 is solved marked out as it causes crash
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
  lv_obj_t * obj = lv_event_get_target(e);
  if(code == LV_EVENT_CLICKED) {
    LV_UNUSED(obj);
    previous_touch_ms = millis();
    brightness = 70;
    backlight.set(brightness);
  }
}


// VOID SETUP //////////////////////////////////////////////////////////////////////////
void setup() {

  lv_init();

  Serial.begin(115200); // Initialize Serial Monitor
  //while (!Serial);
  // Boot M4 & Initialize RPC protocol
  if ( RPC.begin() ) {
    Serial.println("M7 Booting M4 Core");
  }
  else {
    Serial.println("M7 Failed to boot M4 Core");
  }

  if (!CAN.begin(CanBitRate::BR_500k)) {
        RPC.println("CAN.begin(...) failed.");
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
  lv_obj_set_style_bg_color(parent, lv_color_hex(0x03989e), LV_PART_MAIN);
  lv_obj_center(parent);

  // initialise container object
  lv_obj_t * cont;

  // create left column container instance
  cont = lv_obj_create(parent);
  lv_obj_add_event_cb(cont, screen_touch, LV_EVENT_ALL, NULL);
  lv_obj_set_grid_cell(cont, LV_GRID_ALIGN_STRETCH, 0, 1,
                        LV_GRID_ALIGN_STRETCH, 0, 1);

  // Create labels for CAN data
  create_can_label(cont, "SOC", "%", &(combinedData.canData.soc), CAN_DATA_TYPE_BYTE, 20, 20);
  create_can_label(cont, "Current", "A", &(combinedData.canData.instI), CAN_DATA_TYPE_FLOAT, 180, 20);
  create_can_label(cont, "Voltage", "V", &(combinedData.canData.instU), CAN_DATA_TYPE_FLOAT, 20, 50);
  create_can_label(cont, "Power", "W", &(combinedData.canData.p), CAN_DATA_TYPE_INT, 180, 50);
  create_can_label(cont, "High Cell ID", "", &(combinedData.canData.hCid), CAN_DATA_TYPE_BYTE, 20, 80);
  create_can_label(cont, "High Cell", "V", &(combinedData.canData.hC), CAN_DATA_TYPE_DOUBLE_FLOAT, 180, 80);
  create_can_label(cont, "Low Cell ID", "", &(combinedData.canData.lCid), CAN_DATA_TYPE_BYTE, 20, 110);
  create_can_label(cont, "Low Cell", "V", &(combinedData.canData.lC), CAN_DATA_TYPE_DOUBLE_FLOAT, 180, 110);
  create_can_label(cont, "Discharge Current Limit          ", "A", &(combinedData.canData.dcl), CAN_DATA_TYPE_BYTE, 20, 160);
  create_can_label(cont, "Charge Current Limit                ", "A", &(combinedData.canData.ccl), CAN_DATA_TYPE_BYTE, 20, 190);
  create_can_label(cont, "Cycles", "", &(combinedData.canData.cc), CAN_DATA_TYPE_INT, 20, 240);
  create_can_label(cont, "Health", "%", &(combinedData.canData.h), CAN_DATA_TYPE_BYTE, 180, 240);
  create_can_label(cont, "Capacity", "Ah", &(combinedData.canData.ah), CAN_DATA_TYPE_FLOAT, 20, 270);
  create_can_label(cont, "Internal Heatsink Temperature", "\u00B0C", &(combinedData.canData.hs), CAN_DATA_TYPE_BYTE, 20, 300);
  
  
  // Create button to clear faults
  lv_obj_t* btn1 = lv_btn_create(cont);
  lv_obj_add_event_cb(btn1, clear_bms_fault, LV_EVENT_CLICKED, NULL);
  lv_obj_set_pos(btn1, 110, 600);
  
  lv_obj_t* btn1_label = lv_label_create(btn1);
  lv_label_set_text(btn1_label, "Clear Faults"); 
  lv_obj_center(btn1_label);

  
  // create right column container instance
  cont = lv_obj_create(parent);
  lv_obj_add_event_cb(cont, screen_touch, LV_EVENT_ALL, NULL);
  lv_obj_set_grid_cell(cont, LV_GRID_ALIGN_STRETCH, 1, 1,
                            LV_GRID_ALIGN_STRETCH, 0, 1);

  // Create Button 1 - CEILING HEATER
  // arguments 1:obj  2:label 3:relay_pin 4:y_offset 5:dcl_limit 6:timeout_ms
  create_button(cont, "Ceiling Heater", RELAY1, 20, 70, 0);

  // Create Button 2 - SHOWER HEATER
  create_button(cont, "Shower Heater",  RELAY2, 120, 10, 0);

  // Create Button 3 - HOT WATER
  create_button(cont, "Hot Water",      RELAY3, 300, 60, hot_water_interval_ms); // 15 min timer reset if negative avgI or CCL < 10A

  // Create Button 4 - INVERTER
  create_button(cont, "Inverter",       RELAY4, 400, 1, inverter_startup_ms); // 3 minute sweeps to check if avgI > 2A (stdby is 2A) unless negative avgI or CCL < 10A

}

// LOOP ///////////////////////////////////////////////////////////////////////////////////
void loop() {

  // Handle screen events
  lv_timer_handler();
  lv_task_handler();

  // CANBUS READ AND WRITE
  if (CAN.available()) {
    CanMsg const msg = CAN.read();
    rxId = msg.id;
    len = msg.data_length;
    memcpy(rxBuf, msg.data, len);
    sort_can();

    // send CAN if commanded
    if ( msg_cnt && msg_cnt < 3 ) {
      memcpy((void *)(msg_data + 4), &msg_cnt, sizeof(msg_cnt));
      CanMsg msg(CanStandardId(CAN_ID), sizeof(msg_data), msg_data);

      // retry if send failed
      if ( int const rc = CAN.write(msg); rc <= 0 ) {
        Serial.print("CAN.write(...) failed with error code ");
        Serial.println(rc);
        msg_cnt++;
      }
      else msg_cnt = 0; // sent successfully
    }
  }
  //else { Serial.println("M7 CAN not available"); } // crazy messages despite working
  /*Serial.print("instI: ");
  Serial.println(combinedData.canData.instI);
  Serial.print("Power: ");
  Serial.println(combinedData.canData.p);
  Serial.print("absI: ");
  Serial.println(combinedData.canData.absI);
  Serial.print("avgI: ");
  Serial.println(combinedData.canData.avgI);*/
  

  // write messages from M4 core
  String buffer = "";
  while (RPC.available()) {
    // call func to get sensors and can data from M4 core
    retrieve_M4_data();
    buffer += (char)RPC.read();  // Fill the buffer with characters
  }
  if (buffer.length() > 0) Serial.print(buffer);

  // if no touch within timeout dim display and brightness above 0
  if (previous_touch_ms + touch_timeout_ms < millis() && brightness) {
    dim_display();
  }
    
  delay(5); // calming loop
}
