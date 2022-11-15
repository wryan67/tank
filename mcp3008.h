#pragma once
#include <vector>

#define MCP3008_SINGLE   8
#define MCP3008_DIFF     0
#define MCP3008_CHANNELS 8
using namespace std;

struct MCP3008DataType {
    float volts;
    float movingAverage;
    vector<float> window;
};

