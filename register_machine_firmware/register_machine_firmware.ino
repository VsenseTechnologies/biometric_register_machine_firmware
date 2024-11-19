#include<Arduino.h>
#include<Vsense_Finger_Print.h>
#include<ArduinoJson.h>
#include <arduino_base64.hpp>
#include<SPIFFS.h>

//serial communication baud rate in bits/sec
#define SERIAL_COMMUNICATION_BAUD_RATE 115200
//finger print sensor communication baud rate in bits/sec
#define FINGER_PRINT_SENSOR_COMMUNICATION_BAUD_RATE 57600
//fingerprint template size in bytes
#define FINGERPRINT_TEMPLATE_SIZE 512
//base64 encoded string length
#define BASE64_ENCODED_STRING_SIZE 685

//intializing the hardware serial communication
HardwareSerial serial_port(2);

//initializing the finger print sensor by specifying UART2
Adafruit_Fingerprint finger=Adafruit_Fingerprint(&serial_port); 

//initializing the json document by specifying the memory in bytes
StaticJsonDocument<512> json_doc;

//creating the finger print reading handler
TaskHandle_t finger_print_reading_task_handler;

const uint8_t buzzer_pin=4;
const uint8_t green_led1_pin=14;
const uint8_t green_led2_pin=27;
const uint8_t red_led_pin=26;


//finger print sensor status
uint8_t sensor_status;

//json response buffer
String json_response;

//buffer to store finger print template
uint8_t template_buffer[FINGERPRINT_TEMPLATE_SIZE];

//buffer to store base64 encoded string
char base64_encoded_string[BASE64_ENCODED_STRING_SIZE];

void finger_print_sensor_not_found_indicater() {
  digitalWrite(buzzer_pin,1);
  delay(1000);
  digitalWrite(buzzer_pin,0);
  delay(1000);
}

void invalid_finger_indicater() {
  digitalWrite(buzzer_pin,1);
  delay(200);
  digitalWrite(buzzer_pin,0);
  delay(15);
  digitalWrite(buzzer_pin,1);
  delay(200);
  digitalWrite(buzzer_pin,0);
  delay(20);
  digitalWrite(buzzer_pin,1);
  delay(200);
  digitalWrite(buzzer_pin,0);
  delay(15);
}

void power_up_indicater() {
  digitalWrite(green_led1_pin,1);
}

void power_down_indicater() {
  digitalWrite(green_led1_pin,0);
}

bool decode_input_json(String &input_message) {
  //clearing the prev json buffer
  json_doc.clear();
  //decoding the json message
  DeserializationError error=deserializeJson(json_doc,input_message);
  //checking for the json decode error
  if(error) {
    return false;
  }

  return true;
}

uint8_t get_input_control_status() {
  return json_doc["control_status"];
}

uint8_t take_finger_print() {
  //taking raw image from the finger print sensor
  int sensor_status=finger.getImage();

  //matching the sensor status
  switch(sensor_status) {
    case FINGERPRINT_OK:
      return 1;
    case FINGERPRINT_NOFINGER:
      return 0;
    case FINGERPRINT_PACKETRECIEVEERR:
      return 0;
    case FINGERPRINT_IMAGEFAIL:
      invalid_finger_indicater();
      return 2;
    default:
      invalid_finger_indicater();
      return 2;
  }

}

uint8_t create_character_file(uint8_t buffer_id){
  if(finger.image2Tz(buffer_id) == FINGERPRINT_OK) {
    return 0;
  }else{
    invalid_finger_indicater();
    return 1;
  }
}

uint8_t create_model() {
  int sensor_status=finger.createModel();
  if(sensor_status == FINGERPRINT_OK) {
    return 0;
  }else if(sensor_status == FINGERPRINT_ENROLLMISMATCH){
    return 1;
  }else{
    return 2;
  }
}

uint8_t transfer_template(){
  if(finger.getModel() == FINGERPRINT_OK) {
    return 0;
  }else{
    return 1;
  }
}

uint8_t store_template_to_buffer(int size,uint8_t *buffer) {
  if(finger.get_template_buffer(size,buffer) == FINGERPRINT_OK) {
    return 0;
  }else{
    return 1;
  }
}


void base64_encoder(int input_buffer_length,uint8_t *input,char *output) {
  base64::encode(input,input_buffer_length,output);
}

String base64_to_json_encoder(int base64_buffer_length,char *base64_buffer) {
  //setting the base64 buffer 
  String base64_string="";
  //setting the json buffer
  String json_message;
  
  for(int i=0;i<base64_buffer_length;i++) {
    base64_string+=base64_buffer[i];
  }

  //clearing the json_buffer
  json_doc.clear();

  json_doc["error_status"]="0";
  json_doc["message_type"]="3";
  json_doc["fingerprint_data"]=base64_string;

  //encoding to json format
  serializeJson(json_doc,json_message);

  return json_message;
}

void clear_serial_buffer() {
  while(Serial.available() > 0) {
    Serial.read();
  }
}

void finger_print_input_handler(uint8_t buffer_id) {

  while(1) {
    sensor_status=take_finger_print();

    if(sensor_status == 1) {
      break;
    }

    if(sensor_status == 2) {
      json_response="{\"error_status\":\"1\",\"error_type\":\"1\"}#";
      Serial.println(json_response);
      return;
    }

  }

  sensor_status=create_character_file(buffer_id);

  if(sensor_status == 0 ){
    //clearing the prev json buffer
    json_doc.clear();
    json_doc["error_status"]="0";
    json_doc["message_type"]= buffer_id == 1 ? "0" : "1";
    serializeJson(json_doc,json_response);
    json_response+="#";
    Serial.println(json_response);
    return;
  }

  if(sensor_status == 1) {
    json_response="{\"error_status\":\"1\",\"error_type\":\"2\"}#";
    Serial.println(json_response);
    return;
  }

}

void download_finger_print_data_to_controller_buffer() {
  sensor_status = create_model();

  if(sensor_status == 1) {
    json_response="{\"error_status\":\"1\",\"error_type\":\"3\"}#";
    Serial.println(json_response);
    return;
  }

  if(sensor_status == 2) {
    json_response="{\"error_status\":\"1\",\"error_type\":\"4\"}#";
    Serial.println(json_response);
    return;
  }

  sensor_status=transfer_template();

  if(sensor_status == 1) {
    json_response="{\"error_status\":\"1\",\"error_type\":\"5\"}#";
    Serial.println(json_response);
    return;
  }

  sensor_status=store_template_to_buffer(FINGERPRINT_TEMPLATE_SIZE,template_buffer);

  if(sensor_status == 1) {
    json_response="{\"error_status\":\"1\",\"error_type\":\"6\"}#";
    Serial.println(json_response);
    return;
  } 

  json_response="{\"error_status\":\"0\",\"message_type\":\"2\"}#";
  Serial.println(json_response);
  return;

}

void get_finger_print_data() {
  base64_encoder(BASE64_ENCODED_STRING_SIZE,template_buffer,base64_encoded_string);

  json_response=base64_to_json_encoder(BASE64_ENCODED_STRING_SIZE,base64_encoded_string);

  json_response+="#";

  Serial.println(json_response);

  return;
}

void setup() {
  //setting the gpio pins
  pinMode(buzzer_pin,OUTPUT);
  pinMode(green_led1_pin,OUTPUT);
  pinMode(green_led2_pin,OUTPUT);
  pinMode(red_led_pin,OUTPUT);

  //indicating the power up
  power_up_indicater();

  //setting the serial communication baudrate
  Serial.begin(SERIAL_COMMUNICATION_BAUD_RATE);
  while(!Serial);
  //setting the finger print sensor baudrate
  finger.begin(FINGER_PRINT_SENSOR_COMMUNICATION_BAUD_RATE);

  //verifying the finger print sensor connection
  if(!finger.verifyPassword()) {
    while(1) {
      finger_print_sensor_not_found_indicater();
    }
  }

}

void loop() {

  if(Serial.available() > 0) {
    String serial_json_message=Serial.readString();

    bool is_json_decode_successfull=decode_input_json(serial_json_message);

    if(is_json_decode_successfull){
      uint8_t control_status=get_input_control_status();

      switch(control_status) {
        case 0:
          finger_print_input_handler(1);
          clear_serial_buffer();
          break;
        case 1:
          finger_print_input_handler(2);
          clear_serial_buffer();
          break;
        case 2:
          download_finger_print_data_to_controller_buffer();
          clear_serial_buffer();
          break;
        case 3:
          get_finger_print_data();
          clear_serial_buffer();
          break;
      }

    }else{
      json_response="{\"error_status\":\"1\",\"error_type\":\"0\"}#";
      Serial.println(json_response);
    }

  }

}
