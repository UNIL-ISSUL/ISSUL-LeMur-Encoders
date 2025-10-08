/**
 * @file teleplot.cpp
 * @brief Implementation of Teleplot communication functions
 * 
 * This file implements functions to send data to Teleplot visualization tool
 * in the correct format. Supports single values and multiple values as arrays.
 */

#include "teleplot.h"

void teleplot_print(String text, int data, uint32_t timestamp) {
  Serial.print(">");
  Serial.print(text);
  Serial.print(":");
  Serial.print(timestamp);
  Serial.print(":");
  Serial.println(data);
}

void teleplot_print(String text, float data, uint32_t timestamp) {
  Serial.print(">");
  Serial.print(text);
  Serial.print(":");
  Serial.print(timestamp);
  Serial.print(":");
  Serial.println(data);
}

void teleplot_print(String text, const float data[], int count, uint32_t timestamp) {
  Serial.print(">");
  Serial.print(text);
  Serial.print(":");
  for(int i = 0; i < count; i++) {
    Serial.print(timestamp);
    Serial.print(":");
    Serial.print(data[i]);
    if(i < count - 1) {
      Serial.print(";");
    }
  }
  Serial.println();
}

void teleplot_print(String text, const int data[], int count, uint32_t timestamp) {
  Serial.print(">");
  Serial.print(text);
  Serial.print(":");
  for(int i = 0; i < count; i++) {
    Serial.print(timestamp);
    Serial.print(":");
    Serial.print(data[i]);
    if(i < count - 1) {
      Serial.print(";");
    }
  }
  Serial.println();
}

void teleplot_examples() {
  uint32_t now = millis();
  
  // Example 1: Simple array declaration and call
  // Output: >sensors:1234567:12.5;1234567:34.7;1234567:56.2;1234567:78.9
  float sensor_data[] = {12.5, 34.7, 56.2, 78.9};
  teleplot_print("sensors", sensor_data, 4, now);
  
  // Example 2: Using template version (automatic size detection)
  // Output: >temp:1234567:25.3;1234567:26.1;1234567:24.8
  float temperatures[] = {25.3, 26.1, 24.8};
  teleplot_print("temp", temperatures, now); // No need to specify count
  
  // Example 3: Integer arrays
  // Output: >digital:1234567:1;1234567:0;1234567:1;1234567:1;1234567:0
  int digital_values[] = {1, 0, 1, 1, 0};
  teleplot_print("digital", digital_values, 5, now);
  
  // Example 4: Your requested syntax example
  // Output: >toto:1234567:10.5;1234567:20.3;1234567:30.7
  float var1 = 10.5, var2 = 20.3, var3 = 30.7;
  float toto_data[] = {var1, var2, var3};
  teleplot_print("toto", toto_data, 3, now);
}