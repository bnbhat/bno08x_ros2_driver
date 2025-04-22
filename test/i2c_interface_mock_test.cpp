#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "bno08x_ros2_driver/i2c_interface.hpp"

class MockI2CInterface : public I2CInterface {
public:
    MockI2CInterface(const std::string& i2c_bus_file, uint8_t i2c_addr)
        : I2CInterface(i2c_bus_file, i2c_addr) {}

    MOCK_METHOD(int, open, (), (override));
    MOCK_METHOD(void, close, (), (override));
    MOCK_METHOD(int, read, (uint8_t *pBuffer, unsigned len, uint32_t *t_us), (override));
    MOCK_METHOD(int, write, (uint8_t *pBuffer, unsigned len), (override));
};

TEST(I2CInterfaceTest, OpenSuccess) {
    MockI2CInterface mock("/dev/i2c-1", 0x28);

    EXPECT_CALL(mock, open())
        .Times(1)
        .WillOnce(::testing::Return(0));

    EXPECT_EQ(mock.open(), 0);
}

TEST(I2CInterfaceTest, ReadSuccess) {
    MockI2CInterface mock("/dev/i2c-1", 0x28);

    uint8_t buffer[10];
    uint32_t timestamp = 0;

    EXPECT_CALL(mock, read(buffer, 10, &timestamp))
        .Times(1)
        .WillOnce(::testing::Return(10));

    EXPECT_EQ(mock.read(buffer, 10, &timestamp), 10);
}

TEST(I2CInterfaceTest, WriteSuccess) {
    MockI2CInterface mock("/dev/i2c-1", 0x28);

    uint8_t buffer[10] = {1, 2, 3, 4, 5};

    EXPECT_CALL(mock, write(buffer, 5))
        .Times(1)
        .WillOnce(::testing::Return(5));

    EXPECT_EQ(mock.write(buffer, 5), 5);
}
