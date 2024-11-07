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

// Variables for backlight control
int brightness = 100;
lv_timer_t *inactivity_timer = NULL; 
lv_timer_t *dim_timer = NULL;
bool user_interaction = false;

// Define DHT22 pins // OBSOLETE BUT USED FOR CREATING BUTTON REF
#define DHTPIN1 2
#define DHTPIN2 3
#define DHTPIN3 4
#define DHTPIN4 5

// Define relay pins
#define RELAY1 64   // living space
#define RELAY2 62   // shower room
#define RELAY3 60   // water heater

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
uint8_t const msg_data[] = {0x002, 0x01, 0, 0, 0, 0, 0, 0};
static uint8_t msg_cnt = 0;

// CAN RX INFO DISPLAY DATA
int pos_x = 100;
int pos_y = 20;
int line_gap = 50;

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

    float p;                // watt calculated by script

    float rawU;             // Voltage - multiplied by 10
    float rawI;             // Current - multiplied by 10 - negative value indicates charge
    float avgI;             // Average current for clock and sun symbol calculations
    float absI;             // Absolute Current
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

    /*int kw;
    int cap;*/
    
    MSGPACK_DEFINE_ARRAY(rawU, rawI, soc, hC, lC, h, fu, hT, lT, ah, ry, dcl, ccl, ct, st, cc, fs, avgI, hCid, lCid, p, absI);
};
// Define an enumeration for the different data types.
typedef enum {
    CAN_DATA_TYPE_INT,
    CAN_DATA_TYPE_FLOAT,
    CAN_DATA_TYPE_DOUBLE_FLOAT,
    CAN_DATA_TYPE_BYTE
} can_data_type_t;

// define struct for function user-data
typedef struct {
  lv_obj_t *container;
  lv_obj_t *my_btn;
  lv_obj_t *label_obj;
  lv_obj_t* label_text;
  uint8_t relay_pin;
  uint8_t y_offset;
  unsigned long timeout_ms;
  uint8_t dcl_limit;
  uint8_t set_temp = 20; // should match value of dropdown default index
  uint8_t sensor1_pin;
  uint8_t sensor2_pin;
  union {
    int *intData;
    float *floatData;
    byte *byteData;
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

// Variables
unsigned long hot_water_timer = 10000; // duration water heater stays on (ms)


//**************************************************************************************//
// NEED TO ADD DTC FLAGS AND SOLAR HOT WATER OVERRIDE, PERHAPS SIMULATE A BUTTON CLICK? //
//**************************************************************************************//

// CREATE BUTTON INSTANCE
void create_button(lv_obj_t *parent, const char *label_text, uint8_t relay_pin, lv_coord_t y_offset, uint8_t dcl_limit, unsigned long timeout_ms = 0, uint8_t sensor1_pin = 0, uint8_t sensor2_pin = 0) {
  // Allocate memory for userdata for each button crea every switch's new instance
  user_data_t * data = (user_data_t *)malloc(sizeof(user_data_t));
  // Update user data with the instances passed arguments
  data->relay_pin = relay_pin;
  data->y_offset = y_offset;
  data->dcl_limit = dcl_limit;
  data->timeout_ms = timeout_ms;
  data->sensor1_pin = sensor1_pin;
  data->sensor2_pin = sensor2_pin;

  // create button object and add to struct
  data->my_btn = lv_btn_create(parent); // IMPORTANT TO STORE BUTTON IN USERDATA STRUCT - ELSE CLEARING BUTTONS WON'T WORK
  lv_obj_set_pos(data->my_btn, 10, y_offset);
  lv_obj_t *label = lv_label_create(data->my_btn);
  lv_label_set_text(label, label_text);
  lv_obj_center(label);
  lv_obj_add_flag(data->my_btn, LV_OBJ_FLAG_CHECKABLE);

  // create temperature dropdown and dynamic temperature labels for thermostat buttons
  if ( sensor1_pin ) {
    // create label and update the user data member for access within timer to allow only to update text not object
    data->label_obj = lv_label_create(lv_obj_get_parent(data->my_btn));
    create_temperature_dropdown(parent, data);
    // create timed labels
    
    lv_timer_create(display_temp, 10000, data);
    lv_obj_set_pos(data->label_obj, 180, y_offset + 13);
  }
  
  // Disable all buttons by DCL limit
  lv_timer_create(dcl_check, 10000, data); // check every 10s
    
  // configure relay pins
  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, LOW); // initialise pin LOW
  
  // create event handlers with custom user_data
  // CLICKED events for timer and ALL events for thermostat
  if ( timeout_ms ) {
    lv_obj_add_event_cb(data->my_btn, timer_event_handler, LV_EVENT_ALL, data);
  }
  else {
    lv_obj_add_event_cb(data->my_btn, thermostat_event_handler, LV_EVENT_ALL, data);
  }
}

// TIMER FOR DCL CHECK
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

// timer event handler function - HOT WATER ///////////////////////////////////////////////////////////
static void timer_event_handler(lv_event_t * e) {
  user_data_t * data = (user_data_t *)lv_event_get_user_data(e);
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * obj = lv_event_get_target(e);
  //uint8_t relay_pin = data->relay_pin;
    if(code == LV_EVENT_CLICKED) {
    LV_UNUSED(obj);
    if ( lv_obj_has_state(data->my_btn, LV_STATE_CHECKED) ) {
      digitalWrite(data->relay_pin, HIGH);
      // Start timer
      lv_timer_create(switch_off, hot_water_timer, data); // want to disable timer once excess solar to hot water is activated
    }
    else {
      digitalWrite(data->relay_pin, LOW);
    }
  }
}
// SWITCH OFF CALLBACK FUNCTION ///////////////////////////////////////////////////////////////////////
void switch_off(lv_timer_t * timer) {
  user_data_t * data = (user_data_t *)timer->user_data;
  
  // turn off the relay
  digitalWrite(data->relay_pin, LOW);
  
  // clear button flag
  lv_obj_clear_state(data->my_btn, LV_STATE_CHECKED);
}

// thermostat event handler function - HEATERS /////////////////////////////////////////////////////////
void thermostat_event_handler(lv_event_t * e) {
  user_data_t * data = (user_data_t *)lv_event_get_user_data(e);
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * obj = lv_event_get_target(e);
  
  if(code == LV_EVENT_CLICKED) {
    LV_UNUSED(obj);
    if ( lv_obj_has_state(data->my_btn, LV_STATE_CHECKED) ) {
      lv_timer_create(thermostat_timer, 10000, data); // check temp diff every 10s
    }
    else {
      digitalWrite(data->relay_pin, LOW);
      //lv_timer_set_repeat_count(timer, 1);
    }
  }
}

////////////////////////////////////////////////////////// DROP DOWN ///////////////////////////////////////
// event handler for temperature dropdown
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

// THERMOSTAT TIMER AND DISPLAY FUNCTION ///////////////////////////////////////////////////////////////////// HERE WE USE COMBINEDDATA !!!!!!!!!!!!!!!!!!!!!!!!!1
void thermostat_timer(lv_timer_t * timer) {
  user_data_t * data = (user_data_t *)timer->user_data;
  
  if ( data->sensor2_pin ) { // use avg temp
    // Close relay if temperature is below selected and button has been pressed
    if ( combinedData.sensorData.avg_temp < data->set_temp && lv_obj_has_state(data->my_btn, LV_STATE_CHECKED) ) {
    digitalWrite(data->relay_pin, HIGH); 
    }
    // Open relay when temperature is higher or equal to selected
    else {
      digitalWrite(data->relay_pin, LOW);
    }
  }
  else {
    // Close relay if temperature is below selected and button has been pressed
    if ( combinedData.sensorData.temp3 < data->set_temp && lv_obj_has_state(data->my_btn, LV_STATE_CHECKED) ) { // temp3 set as it is easier to code for now **********************************
      digitalWrite(data->relay_pin, HIGH); 
    }
    // Open relay when temperature is higher or equal to selected
    else {
      digitalWrite(data->relay_pin, LOW);
    }
  }  
}

// DISPLAY TEMP FUNCTION ///////////////////////////////////////////////////////////////
void display_temp(lv_timer_t *timer) {
  user_data_t * data = (user_data_t *)timer->user_data;
  char buf[10];
  if ( data->sensor2_pin ) {
    snprintf(buf, sizeof(buf), "%.1f\u00B0C", combinedData.sensorData.avg_temp); // use the global shared instance for temp and humidity
  }
  else {
    snprintf(buf, sizeof(buf), "%.1f\u00B0C", combinedData.sensorData.temp3);
  }
  // update label text only and not the label object
  lv_label_set_text(data->label_obj, buf);
}

// CLEAR CAN EVENT HANDLER ////////////////////////////////////////////////////////////////
void clear_bms_fault(lv_event_t * e) {
    //lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if ( lv_obj_has_state(obj, LV_EVENT_CLICKED) ) {
      //if ( RPC.available() ) {
      //RPC.call("sendCan");
      msg_cnt = 1;
      Serial.println("Sending CAN message");
      //}
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
        lv_timer_create(refresh_can_data, 200, data);
    }
}



// SORT CANBUS MSG
void sort_can() {
  
    if (rxId == 0x3B) {
        combinedData.canData.rawU = ((rxBuf[0] << 8) + rxBuf[1]) / 10.0;
        combinedData.canData.rawI = ((rxBuf[2] << 8) + rxBuf[3]); // BAD DATA 65500 - 0 jumping
        combinedData.canData.absI = ((rxBuf[4] << 8) + rxBuf[5]); // BAD DATA 37250 stuck
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
        combinedData.canData.avgI = ((rxBuf[4] << 8) + rxBuf[5]) / 1000.0; // SHOULD BE DIVIDED BY 10 ACCORDING TO DOCUMENTATION
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
    }
    combinedData.canData.p = abs(combinedData.canData.avgI) * combinedData.canData.rawU;
}

// RETRIEVE DATA FROM M4 CORE
void retrieve_M4_data() {
    // Call the RPC function to get sensor data
      combinedData.sensorData = RPC.call("getSensorData").as<SensorData>();
      //combinedData.canData = RPC.call("getCanData").as<CanData>(); // until can issue on m4 is solved marked out as it causes crash
}

// DIM DISPLAY FUNCTION
void gradualDim(lv_timer_t *timer) {
    if (brightness > 0) {
        backlight.set(brightness--);
    } else {
        lv_timer_del(timer); // Stop the timer once brightness reaches 0
    }
}
// DIM TIMER
void dimDisplay(lv_timer_t *timer) {
    // Start a new timer to gradually dim the display
    dim_timer = lv_timer_create(gradualDim, 100, NULL); // Dim every 100 ms
}
// RESET SCREEN INACTIVITY TIMER
void resetInactivityTimer() {
    if (inactivity_timer) {
        lv_timer_reset(inactivity_timer); // Reset the existing timer
    }
}


// VOID SETUP //////////////////////////////////////////////////////////////////////////
void setup() {

  backlight.begin();
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
  // Set display brightness
  backlight.set(brightness);

  // Set up a callback to reset the inactivity timer on any LVGL event
  /*lv_obj_t *scr = lv_scr_act();
  lv_obj_add_event_cb(scr, [](lv_event_t * e) {
    if (lv_event_get_code(e) == LV_EVENT_PRESSED ||
      lv_event_get_code(e) == LV_EVENT_RELEASED ||
      lv_event_get_code(e) == LV_EVENT_CLICKED) {
      user_interaction = true;
      resetInactivityTimer();
    }
  }, LV_EVENT_ALL, NULL);

  // Start the inactivity timer
  inactivity_timer = lv_timer_create(dimDisplay, 120000, NULL); // 2 min delay*/
  
  // Initialise display and touch
  Display.begin();
  TouchDetector.begin();

  // Create a container with grid 2x1
  static lv_coord_t col_dsc[] = {370, 370, LV_GRID_TEMPLATE_LAST};
  static lv_coord_t row_dsc[] = {430, 430, LV_GRID_TEMPLATE_LAST};
  lv_obj_t * parent = lv_obj_create(lv_scr_act());
  lv_obj_set_grid_dsc_array(parent, col_dsc, row_dsc);
  lv_obj_set_size(parent, Display.width(), Display.height());
  lv_obj_set_style_bg_color(parent, lv_color_hex(0x03989e), LV_PART_MAIN);
  lv_obj_center(parent);

  // initialise container object
  lv_obj_t * cont;

  // create left column
  cont = lv_obj_create(parent);
  lv_obj_set_grid_cell(cont, LV_GRID_ALIGN_STRETCH, 0, 1,
                        LV_GRID_ALIGN_STRETCH, 0, 1);

  // Create labels for CAN data
  create_can_label(cont, "SOC", "%", &(combinedData.canData.soc), CAN_DATA_TYPE_BYTE, 20, 20);
  create_can_label(cont, "Current", "A", &(combinedData.canData.avgI), CAN_DATA_TYPE_FLOAT, 180, 20);
  create_can_label(cont, "Voltage", "V", &(combinedData.canData.rawU), CAN_DATA_TYPE_FLOAT, 20, 50);
  create_can_label(cont, "Power", "W", &(combinedData.canData.p), CAN_DATA_TYPE_FLOAT, 180, 50);
  create_can_label(cont, "High Cell ID", "", &(combinedData.canData.hCid), CAN_DATA_TYPE_BYTE, 20, 80);
  create_can_label(cont, "High Cell", "V", &(combinedData.canData.hC), CAN_DATA_TYPE_DOUBLE_FLOAT, 180, 80);
  create_can_label(cont, "Low Cell ID", "", &(combinedData.canData.lCid), CAN_DATA_TYPE_BYTE, 20, 110);
  create_can_label(cont, "Low Cell", "V", &(combinedData.canData.lC), CAN_DATA_TYPE_DOUBLE_FLOAT, 180, 110);
  create_can_label(cont, "Discharge Current Limit          ", "A", &(combinedData.canData.dcl), CAN_DATA_TYPE_BYTE, 20, 160);
  create_can_label(cont, "Charge Current Limit                ", "A", &(combinedData.canData.ccl), CAN_DATA_TYPE_BYTE, 20, 190);
  create_can_label(cont, "Cycles", "", &(combinedData.canData.cc), CAN_DATA_TYPE_INT, 20, 240);
  create_can_label(cont, "Health", "%", &(combinedData.canData.h), CAN_DATA_TYPE_BYTE, 180, 240);
  create_can_label(cont, "Capacity", "Ah", &(combinedData.canData.ah), CAN_DATA_TYPE_FLOAT, 20, 270);
  
  
  // Create button to clear faults
  lv_obj_t* btn1 = lv_btn_create(cont);
  lv_obj_add_event_cb(btn1, clear_bms_fault, LV_EVENT_CLICKED, NULL);
  lv_obj_set_pos(btn1, 110, 600);
  
  lv_obj_t* btn1_label = lv_label_create(btn1);
  lv_label_set_text(btn1_label, "Clear Faults"); 
  lv_obj_center(btn1_label);

  
  // create right column
  cont = lv_obj_create(parent);
  lv_obj_set_grid_cell(cont, LV_GRID_ALIGN_STRETCH, 1, 1,
                            LV_GRID_ALIGN_STRETCH, 0, 1);

  // Create Switch 1 - args: obj, label, relay_pin, y_offset, dcl_limit, timeout_ms, (sensor1_pin), (sensor2_pin)
  create_button(cont, "Ceiling Heater", RELAY1, 20, 70, 0, DHTPIN1, DHTPIN2);

  // Create Switch 2
  create_button(cont, "Shower Heater", RELAY2, 120, 10, 0, DHTPIN3);

  // Create Switch 3
  create_button(cont, "Hot Water", RELAY3, 300, 60, 10000);
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
  Serial.print("rawI: ");
  Serial.println(combinedData.canData.rawI);
  Serial.print("Power: ");
  Serial.println(combinedData.canData.p);
  Serial.print("absI: ");
  Serial.println(combinedData.canData.absI);
  Serial.print("avgI: ");
  Serial.println(combinedData.canData.avgI);
  

  // write messages from M4 core
  String buffer = "";
  while (RPC.available()) {
    // call func to get sensors and can data from M4 core
    retrieve_M4_data();
    buffer += (char)RPC.read();  // Fill the buffer with characters
  }
  if (buffer.length() > 0) Serial.print(buffer);
  
  delay(5); // calming loop
}
