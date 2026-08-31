/**
 * @file teleplot.h
 * @brief Teleplot communication functions for data visualization
 * 
 * Supports:
 * - Single float / int values with optional unit and flags
 * - Grouped curves on the same chart (group/name)
 * - Text & state annotations (|t)
 * - 2D XY plots (|xy)
 * - Array of values at instant t (emitted as group/index)
 * - Time-series historical batch (t0:v0;t1:v1;...)
 */

#ifndef TELEPLOT_H
#define TELEPLOT_H

#include <Arduino.h>

// Single value plotting
void teleplot_print(const String &name, float value, uint32_t timestamp = 0, const String &unit = "", const String &flags = "", int digits = 4);
void teleplot_print(const String &name, int32_t value, uint32_t timestamp = 0, const String &unit = "", const String &flags = "");

// Grouped plotting: superimposes curves on the same chart under "group"
void teleplot_print_group(const String &group, const String &name, float value, uint32_t timestamp = 0, const String &unit = "", int digits = 4);
void teleplot_print_group(const String &group, const String &name, int32_t value, uint32_t timestamp = 0, const String &unit = "");

// Text & State plotting (renders as text indicator / log in Teleplot using |t)
void teleplot_print_text(const String &name, const String &text_value, uint32_t timestamp = 0, const String &group = "");

// 2D XY Plot (|xy)
void teleplot_print_xy(const String &name, float x, float y);

// Array of values at instant t (emitted as group/0, group/1, ...)
void teleplot_print_array(const String &group, const float data[], size_t count, uint32_t timestamp = 0, const String &unit = "");
void teleplot_print_array(const String &group, const int32_t data[], size_t count, uint32_t timestamp = 0, const String &unit = "");

template<typename T, size_t N>
void teleplot_print_array(const String &group, const T (&data)[N], uint32_t timestamp = 0, const String &unit = "") {
  teleplot_print_array(group, data, N, timestamp, unit);
}

// Time-series batch (t0:v0;t1:v1;...)
void teleplot_print_batch(const String &name, const float data[], const uint32_t timestamps[], size_t count, const String &unit = "", int digits = 4);

#endif // TELEPLOT_H
