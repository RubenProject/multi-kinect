#pragma once

#include "pch.h"


struct InputState
{
    std::array<bool, GLFW_KEY_LAST> keysdown;
    std::array<bool, GLFW_KEY_LAST> keyspress;
    void reset() {
        for (int i = 0; i < GLFW_KEY_LAST; ++i) {
            keyspress[i] = false;
        }
    };
};


extern InputState *inputstate;
