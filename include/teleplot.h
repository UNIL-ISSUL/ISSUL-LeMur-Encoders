/**
 * @file teleplot.h
 * @brief Teleplot communication functions for data visualization
 * 
 * This file contains functions to send data to Teleplot visualization tool
 * in the correct format. Supports single values and multiple values as arrays.
 * 
 * Teleplot format: >name:timestamp:value1;timestamp:value2;timestamp:value3
 */

#ifndef TELEPLOT_H
#define TELEPLOT_H

#include <Arduino.h>

/**
 * @brief Send a single integer value to Teleplot
 * @param text The variable name/label for the plot
 * @param data The integer value to send
 * @param timestamp The timestamp in milliseconds
 */
void teleplot_print(String text, int data, uint32_t timestamp);

/**
 * @brief Send a single float value to Teleplot
 * @param text The variable name/label for the plot
 * @param data The float value to send
 * @param timestamp The timestamp in milliseconds
 */
void teleplot_print(String text, float data, uint32_t timestamp);

/**
 * @brief Send multiple float values to Teleplot as an array
 * @param text The variable name/label for the plot
 * @param data Array of float values to send
 * @param count Number of elements in the array
 * @param timestamp The timestamp in milliseconds
 */
void teleplot_print(String text, const float data[], int count, uint32_t timestamp);

/**
 * @brief Send multiple integer values to Teleplot as an array
 * @param text The variable name/label for the plot
 * @param data Array of integer values to send
 * @param count Number of elements in the array
 * @param timestamp The timestamp in milliseconds
 */
void teleplot_print(String text, const int data[], int count, uint32_t timestamp);

/**
 * @brief Template function for automatic array size detection
 * @tparam T Data type (int, float, etc.)
 * @tparam N Array size (automatically detected)
 * @param text The variable name/label for the plot
 * @param data Array of values to send
 * @param timestamp The timestamp in milliseconds
 */
template<typename T, int N>
void teleplot_print(String text, const T (&data)[N], uint32_t timestamp) {
  teleplot_print(text, data, N, timestamp);
}

/**
 * @brief Example function showing different ways to use teleplot_print with arrays
 * Output format: >name:timestamp:value1;timestamp:value2;timestamp:value3
 */
void teleplot_examples();

#endif // TELEPLOT_H