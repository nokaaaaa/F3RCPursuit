#pragma once
#include <mbed.h>
#include "parameters.hpp"

class DT35 {
    public:
        DT35(PinName analog_in_pin);
        
        float getObsDistance();

    private:
       AnalogIn analog_in;//メンバ変数


};