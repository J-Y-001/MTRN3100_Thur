#pragma once

#include <Arduino.h>

#include "math.h"

namespace mtrn3100 {


class Turning {
public:
    Turning() : {

    }

    float turn() {

    }

    void chaining(String command) {
        for (int i = 0; i < command.length(); i++) {
            if (command[i] == 'l') {
                
            }
            if (command[i] == 'r') {

            }
            if (command[i] == 'f') {

            }
        }
    }

private:

};

}  // namespace mtrn3100
