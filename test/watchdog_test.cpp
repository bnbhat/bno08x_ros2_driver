#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "bno08x_driver/watchdog.hpp"
#include <thread>

class WatchdogTest : public ::testing::Test {
protected:
    void SetUp() override {
        callback_called_ = false;
    }

    bool callback_called_;
    static void callback_function(bool* called) {
        *called = true;
    }
};

TEST_F(WatchdogTest, DefaultConstructor) {
    Watchdog watchdog;
    EXPECT_EQ(watchdog.get_timeout(), std::chrono::milliseconds(1000));
    EXPECT_EQ(watchdog.get_check_interval(), std::chrono::milliseconds(500));
}

TEST_F(WatchdogTest, CustomCheckInterval) {
    const auto check_interval = std::chrono::milliseconds(200);
    Watchdog watchdog(std::chrono::milliseconds(1000), 
                     std::chrono::milliseconds(check_interval));
    EXPECT_EQ(watchdog.get_check_interval(), check_interval);
}

TEST_F(WatchdogTest, SetCheckInterval) {
    Watchdog watchdog;
    const auto new_interval = std::chrono::milliseconds(300);
    watchdog.set_check_interval(new_interval);
    EXPECT_EQ(watchdog.get_check_interval(), new_interval);
}

TEST_F(WatchdogTest, CallbackExecutionWithCustomInterval) {
    Watchdog watchdog(
        std::chrono::milliseconds(100),  // timeout
        std::chrono::milliseconds(50),   // check_interval
        [this]() { callback_function(&callback_called_); }
    );
    
    watchdog.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    EXPECT_TRUE(callback_called_);
}

TEST_F(WatchdogTest, ResetPreventsCallback) {
    Watchdog watchdog(
        std::chrono::milliseconds(100),  // timeout
        std::chrono::milliseconds(50),   // check_interval
        [this]() { callback_function(&callback_called_); }
    );
    
    watchdog.start();
    for(int i = 0; i < 5; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
        watchdog.reset();
    }
    EXPECT_FALSE(callback_called_);
}

// ... existing tests ...

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}