#include "teleplot.h"

// Format: >name:timestamp:value§unit|flags
void teleplot_print(const String &name, float value, uint32_t timestamp, const String &unit, const String &flags) {
  Serial.print(">");
  Serial.print(name);
  Serial.print(":");
  if (timestamp > 0) {
    Serial.print(timestamp);
    Serial.print(":");
  }
  Serial.print(value);
  if (unit.length() > 0) {
    Serial.print("§");
    Serial.print(unit);
  }
  if (flags.length() > 0) {
    Serial.print("|");
    Serial.print(flags);
  }
  Serial.println();
}

void teleplot_print(const String &name, int32_t value, uint32_t timestamp, const String &unit, const String &flags) {
  Serial.print(">");
  Serial.print(name);
  Serial.print(":");
  if (timestamp > 0) {
    Serial.print(timestamp);
    Serial.print(":");
  }
  Serial.print(value);
  if (unit.length() > 0) {
    Serial.print("§");
    Serial.print(unit);
  }
  if (flags.length() > 0) {
    Serial.print("|");
    Serial.print(flags);
  }
  Serial.println();
}

void teleplot_print_group(const String &group, const String &name, float value, uint32_t timestamp, const String &unit) {
  String fullName = group + "/" + name;
  teleplot_print(fullName, value, timestamp, unit);
}

void teleplot_print_group(const String &group, const String &name, int32_t value, uint32_t timestamp, const String &unit) {
  String fullName = group + "/" + name;
  teleplot_print(fullName, value, timestamp, unit);
}

void teleplot_print_text(const String &name, const String &text_value, uint32_t timestamp, const String &group) {
  Serial.print(">");
  if (group.length() > 0) {
    Serial.print(group);
    Serial.print("/");
  }
  Serial.print(name);
  Serial.print(":");
  if (timestamp > 0) {
    Serial.print(timestamp);
    Serial.print(":");
  }
  Serial.print(text_value);
  Serial.println("|t");
}

void teleplot_print_xy(const String &name, float x, float y) {
  Serial.print(">");
  Serial.print(name);
  Serial.print(":");
  Serial.print(x);
  Serial.print(":");
  Serial.print(y);
  Serial.println("|xy");
}

void teleplot_print_array(const String &group, const float data[], size_t count, uint32_t timestamp, const String &unit) {
  for (size_t i = 0; i < count; i++) {
    teleplot_print_group(group, String(i), data[i], timestamp, unit);
  }
}

void teleplot_print_array(const String &group, const int32_t data[], size_t count, uint32_t timestamp, const String &unit) {
  for (size_t i = 0; i < count; i++) {
    teleplot_print_group(group, String(i), data[i], timestamp, unit);
  }
}

void teleplot_print_batch(const String &name, const float data[], const uint32_t timestamps[], size_t count, const String &unit) {
  if (count == 0) return;
  Serial.print(">");
  Serial.print(name);
  Serial.print(":");
  for (size_t i = 0; i < count; i++) {
    Serial.print(timestamps[i]);
    Serial.print(":");
    Serial.print(data[i]);
    if (i < count - 1) {
      Serial.print(";");
    }
  }
  if (unit.length() > 0) {
    Serial.print("§");
    Serial.print(unit);
  }
  Serial.println();
}
