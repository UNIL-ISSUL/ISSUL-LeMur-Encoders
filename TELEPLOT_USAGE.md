# Teleplot Module Documentation

## Overview
The teleplot module provides functions to send data to the Teleplot visualization tool in the correct format. It supports both single values and multiple values as arrays.

## Files
- `include/teleplot.h` - Header file with function declarations
- `src/teleplot.cpp` - Implementation file with function definitions

## Usage

### Include the Header
```cpp
#include "teleplot.h"
```

### Function Signatures

#### Single Values
```cpp
// Send single integer
teleplot_print("speed", 150, millis());

// Send single float  
teleplot_print("temperature", 25.6, millis());
```

#### Multiple Values (Arrays)
```cpp
// Float array with explicit count
float data[] = {1.5, 2.3, 4.7};
teleplot_print("sensors", data, 3, millis());

// Integer array with explicit count
int values[] = {10, 20, 30, 40};
teleplot_print("digital", values, 4, millis());

// Template version (automatic size detection)
float temps[] = {25.1, 26.3, 24.8};
teleplot_print("temperatures", temps, millis()); // No need to specify count
```

### Your Requested Syntax
```cpp
uint32_t now = millis();
float var1 = 10.5, var2 = 20.3, var3 = 30.7;
float toto_data[] = {var1, var2, var3};
teleplot_print("toto", toto_data, 3, now);
```

## Output Format

### Single Value
```
>speed:1234567:150
```

### Multiple Values
```  
>sensors:1234567:1.5;1234567:2.3;1234567:4.7
```

## Examples
Call `teleplot_examples()` to see various usage patterns demonstrated.

## Integration
The module is automatically included in your main project. Just include the header file and use the functions as shown above.

## Benefits
- ✅ Clean, organized code structure
- ✅ Reusable across projects
- ✅ Proper Teleplot format compliance
- ✅ Type safety and automatic array sizing
- ✅ Comprehensive documentation and examples