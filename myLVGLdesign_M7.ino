#include <Arduino.h>
#include <RPC.h>
#include <Arduino_H7_Video.h>
#include "lvgl.h"
#include "Arduino_GigaDisplayTouch.h"

Arduino_H7_Video Display(800, 480, GigaDisplayShield);
Arduino_GigaDisplayTouch TouchDetector;

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
struct CanData {
    unsigned int rawU;
    int rawI;
    byte soc;
    int hC;
    int lC;
    byte h;
    int fu;
    byte tH;
    byte tL;
    float ah;
    byte ry;
    byte dcl;
    byte ccl;
    byte ct;
    byte st;
    int cc;
    int fs;
    int avgI;
    unsigned int p;
    MSGPACK_DEFINE_ARRAY(rawU, rawI, soc, hC, lC, h, fu, tH, tL, ah, ry, dcl, ccl, ct, st, cc, fs, avgI, p);
};

// define struct for function user-data
typedef struct user_data_t {
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
  void *canDataProperty; // Pointer to the specific CanData property
  const char* label_prefix; // To store label text prefix };
  const char* label_unit;
};

struct CombinedData {
  SensorData sensorData;
  CanData canData;
  user_data_t userData;
};

// Declare instance globally, all included struct instances need not be declared
static CombinedData combinedData;

// Variables
unsigned long hot_water_timer = 10000; // duration water heater stays on (ms)


// CREATE SWITCH AND DISABLE ON CONDITIONS //////////////////////////////////////////////
void create_switch(lv_obj_t *parent, const char *label_text, uint8_t relay_pin, lv_coord_t y_offset, uint8_t dcl_limit, unsigned long timeout_ms = 0, uint8_t sensor1_pin = 0, uint8_t sensor2_pin = 0) {
  // Allocate memory for every switch's new instance
  CombinedData * data = (CombinedData *)malloc(sizeof(CombinedData));
  // Update instance with user data
  data->userData.relay_pin = relay_pin;
  data->userData.y_offset = y_offset;
  data->userData.dcl_limit = dcl_limit;
  data->userData.timeout_ms = timeout_ms;
  data->userData.sensor1_pin = sensor1_pin;
  data->userData.sensor2_pin = sensor2_pin;

  // create button object and add to struct
  data->userData.my_btn = lv_btn_create(parent); // IMPORTANT TO STORE BUTTON IN USERDATA STRUCT - ELSE CLEARING BUTTONS WON'T WORK
  lv_obj_set_pos(data->userData.my_btn, 10, y_offset);
  lv_obj_t *label = lv_label_create(data->userData.my_btn);
  lv_label_set_text(label, label_text);
  lv_obj_center(label);
  lv_obj_add_flag(data->userData.my_btn, LV_OBJ_FLAG_CHECKABLE);

  // create temperature dropdown and dynamic temperature labels
  if ( ! timeout_ms ) {
    create_temperature_dropdown(parent, data);
    // create timed labels
    data->userData.label_obj = lv_label_create(lv_obj_get_parent(data->userData.my_btn));
    lv_timer_create(display_temp, 10000, data);
    lv_obj_set_pos(data->userData.label_obj, 180, y_offset + 13);
  }
  
  // lets set label text
  if ( data->canData.dcl < dcl_limit ) {
    lv_obj_add_state(data->userData.my_btn, LV_STATE_DISABLED);
    lv_obj_t *label2low = lv_label_create(parent);
    lv_label_set_long_mode(label2low, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_label_set_text(label2low, "Please Charge Battery                                            "); // spaces to allow a pause
    lv_obj_set_width(label2low, 150);
    lv_obj_set_pos(label2low, 10, y_offset + 50);
  }
  else {
    lv_label_set_text(label, label_text);
  }
  
  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, LOW); // initialise pin LOW
  
  // create event handlers with custom user_data
  // CLICKED events for timer and ALL events for thermostat
  if ( timeout_ms ) {
    lv_obj_add_event_cb(data->userData.my_btn, timer_event_handler, LV_EVENT_ALL, data);
  }
  else {
    lv_obj_add_event_cb(data->userData.my_btn, thermostat_event_handler, LV_EVENT_ALL, data);
  }
}

// timer event handler function - HOT WATER ///////////////////////////////////////////////////////////
static void timer_event_handler(lv_event_t * e) {
  CombinedData * data = (CombinedData *)lv_event_get_user_data(e);
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * obj = lv_event_get_target(e);
  uint8_t relay_pin = data->userData.relay_pin;
    if(code == LV_EVENT_CLICKED) {
    LV_UNUSED(obj);
    if ( lv_obj_has_state(data->userData.my_btn, LV_STATE_CHECKED) ) {
      digitalWrite(relay_pin, HIGH);
      // Start timer
      lv_timer_create(switch_off, hot_water_timer, data); // want to disable timer once excess solar to hot water is activated
    }
    else {
      digitalWrite(relay_pin, LOW);
    }
  }
}
// SWITCH OFF CALLBACK FUNCTION ///////////////////////////////////////////////////////////////////////
void switch_off(lv_timer_t * timer) {
  CombinedData * data = (CombinedData *)timer->user_data;
  
  // turn off the relay
  digitalWrite(data->userData.relay_pin, LOW);
  
  // clear button flag
  lv_obj_clear_state(data->userData.my_btn, LV_STATE_CHECKED);
}

// thermostat event handler function - HEATERS /////////////////////////////////////////////////////////
void thermostat_event_handler(lv_event_t * e) {
  CombinedData * data = (CombinedData *)lv_event_get_user_data(e);
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * obj = lv_event_get_target(e);
  
  if(code == LV_EVENT_CLICKED) {
    LV_UNUSED(obj);
    if ( lv_obj_has_state(data->userData.my_btn, LV_STATE_CHECKED) ) {
      lv_timer_t * timer = lv_timer_create(thermostat_timer, 10000, data); // check temp diff every 10s
    }
    else {
      digitalWrite(data->userData.relay_pin, LOW);
      //lv_timer_set_repeat_count(timer, 1);
    }
  }
}

////////////////////////////////////////////////////////// DROP DOWN ///////////////////////////////////////
// event handler for temperature dropdown
void dropdown_event_handler(lv_event_t *e) {
    CombinedData * data = (CombinedData *)lv_event_get_user_data(e);
    if ( !data && !data->userData.my_btn ) {
      Serial.println("dropdown_event_handler: no user data");
      return;
    }
    lv_obj_t * dd = lv_event_get_target(e);
    // get index of selected dropdown item
    uint8_t id_selected = lv_dropdown_get_selected(dd);
    // link index to selection
    switch (id_selected) {
      case 0:
        data->userData.set_temp = 5;
        break;
      case 1:
        data->userData.set_temp = 18;
        break;
      case 2:
        data->userData.set_temp = 19;
        break;
      case 3:
        data->userData.set_temp = 20;
        break;
      case 4:
        data->userData.set_temp = 21;
        break;
      case 5:
        data->userData.set_temp = 22;
        break;
      default:
        data->userData.set_temp = 23;
    }
}

// CREATE TEMPERATURE SELECTION DROPDOWN MENU ///////////////////////////////////////
void create_temperature_dropdown(lv_obj_t * parent, CombinedData *data) {
  lv_obj_t *dd = lv_dropdown_create(parent);
  uint8_t y_offset = data->userData.y_offset;
  lv_dropdown_set_options(dd,
    "5\u00B0C\n18\u00B0C\n19\u00B0C\n20\u00B0C\n21\u00B0C\n22\u00B0C\n23\u00B0C");
    
  // set user data
  lv_dropdown_set_selected(dd, 3); // default index to be displayed. value set_temp in struct
  lv_obj_set_user_data(dd, data);
  lv_obj_add_event_cb(dd, dropdown_event_handler, LV_EVENT_VALUE_CHANGED, data);
  
  // place roller
  lv_obj_set_pos(dd, 255, y_offset - 1);
  lv_obj_set_width(dd, 80);
}

// THERMOSTAT TIMER AND DISPLAY FUNCTION /////////////////////////////////////////////////////////////////////
void thermostat_timer(lv_timer_t * timer) {
  CombinedData * data = (CombinedData *)timer->user_data;
  
  if ( data->userData.sensor2_pin ) { // use avg temp
    // Close relay if temperature is below selected and button has been pressed
    if ( data->sensorData.avg_temp < data->userData.set_temp && lv_obj_has_state(data->userData.my_btn, LV_STATE_CHECKED) ) {
    digitalWrite(data->userData.relay_pin, HIGH); 
    }
    // Open relay when temperature is higher or equal to selected
    else {
      digitalWrite(data->userData.relay_pin, LOW);
    }
  }
  else {
    // Close relay if temperature is below selected and button has been pressed
    if ( data->sensorData.temp3 < data->userData.set_temp && lv_obj_has_state(data->userData.my_btn, LV_STATE_CHECKED) ) { // temp3 set as it is easier to code for now
      digitalWrite(data->userData.relay_pin, HIGH); 
    }
    // Open relay when temperature is higher or equal to selected
    else {
      digitalWrite(data->userData.relay_pin, LOW);
    }
  }  
}

// DISPLAY TEMP FUNCTION ///////////////////////////////////////////////////////////////
void display_temp(lv_timer_t *timer) {
  CombinedData * data = (CombinedData *)timer->user_data;
  char buf[10];
  if ( data->userData.sensor2_pin ) {
    snprintf(buf, sizeof(buf), "%.1f\u00B0C", combinedData.sensorData.avg_temp); // use the global shared instance for temp and humidity
  }
  else {
    snprintf(buf, sizeof(buf), "%.1f\u00B0C", combinedData.sensorData.temp3);
  }
  lv_label_set_text(data->userData.label_obj, buf);
}

// CLEAR CAN EVENT HANDLER ////////////////////////////////////////////////////////////////
void clear_bms_fault(lv_event_t * e) {
    //lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if ( lv_obj_has_state(obj, LV_EVENT_CLICKED) ) {
      if ( RPC.available() ) {
      RPC.call("sendCan");
      Serial.println("M7 Sending CAN message through M4");
      }
    }
  }
// REFRESH CAN LABEL DATA //////////////////////////////////////////////////////////////////////
void refresh_can_data(lv_timer_t* timer) {
    CombinedData *data = (CombinedData *)timer->user_data;
    char buf[50];
    
    // Determine the type of property and format accordingly
    snprintf(buf, sizeof(buf), "%s %d %s", data->userData.label_prefix, *(int*)(data->userData.canDataProperty),data->userData.label_unit);

    lv_label_set_text(data->userData.label_obj, buf);
}

void create_can_label(lv_obj_t* parent, const char* label_prefix, const char* label_unit, void* canDataProperty, int x_pos, int y_pos) {
    // Allocate memory for new user data instance
    CombinedData * data = (CombinedData *)malloc(sizeof(CombinedData));
    if (data) {
        // Update instance with user data
        data->userData.label_obj = lv_label_create(parent);
        data->userData.canDataProperty = canDataProperty;
        data->userData.label_prefix = label_prefix;
        data->userData.label_unit = label_unit;

        lv_obj_set_pos(data->userData.label_obj, x_pos, y_pos);
        lv_timer_create(refresh_can_data, 200, data);
    }
}

  
// RETRIEVE DATA FROM M4 CORE
void retrieve_M4_data() {
    // Call the RPC function to get sensor data
      combinedData.sensorData = RPC.call("getSensorData").as<SensorData>();
      //combinedData.canData = RPC.call("getCanData").as<CanData>(); // until can issue on m4 is solved marked out as it causes crash
}

// VOID SETUP //////////////////////////////////////////////////////////////////////////
void setup() {
  
  lv_init();

  Serial.begin(115200); // Initialize Serial Monitor
  while (!Serial);
  // Boot M4 & Initialize RPC protocol
  if ( RPC.begin() ) {
    Serial.println("M7 Booting M4 Core");
  }
  else {
    Serial.println("M7 Failed to boot M4 Core");
  }
  
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
  create_can_label(cont, "SOC", "\u0025", &combinedData.canData.soc, 20, 20);
  create_can_label(cont, "Amperage", "A", &combinedData.canData.rawI, 200, 20);
  create_can_label(cont, "Voltage", "V", &combinedData.canData.rawU, 20, 50);
  create_can_label(cont, "Power", "W", &combinedData.canData.p, 200, 50);
    
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
  create_switch(cont, "Ceiling Heater", RELAY1, 20, 70, 0, DHTPIN1, DHTPIN2);

  // Create Switch 2
  create_switch(cont, "Shower Heater", RELAY2, 120, 10, 0, DHTPIN3);

  // Create Switch 3
  create_switch(cont, "Hot Water", RELAY3, 300, 60, 10000);
}

// LOOP ///////////////////////////////////////////////////////////////////////////////////
void loop() {

  // Handle screen events
  lv_timer_handler();
  lv_task_handler();

  // write messages from M4 core
  String buffer = "";
  while (RPC.available()) {
    // call func to get sensors and can data from M4 core
    retrieve_M4_data();
    buffer += (char)RPC.read();  // Fill the buffer with characters
  }
  if (buffer.length() > 0) {
    Serial.print(buffer);

  //  Serial.print("M7 SOC: ");
//    Serial.println(combinedData.canData.soc);
    //Serial.print("M7 Current: ");
    //Serial.println(combinedData.canData.rawI);
    Serial.print("M7 Temperature and Humidity: ");
    Serial.println(combinedData.sensorData.temp3);
    Serial.println(combinedData.sensorData.humi3);
  }
  delay(5); // calming loop
}
