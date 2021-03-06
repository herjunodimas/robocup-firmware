#include <gtest/gtest.h>

#include "../drivers/rotarySelector/RotarySelector.hpp"
#include "Mbed.hpp"

// A very simple test that checks that the RotarySelector calculates the value
// appropriately given a set of binary inputs.
TEST(RotarySelector, canRead) {
    RotarySelector<DigitalIn> selector({1, 1, 0, 1});
    EXPECT_EQ(11, selector.read());
}
