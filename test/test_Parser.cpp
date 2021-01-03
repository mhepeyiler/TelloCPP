#include <gtest/gtest.h>
#include <Parser.hpp>

Parser p;

TEST(Orientation, PositiveNos)
{
    ASSERT_EQ(0, std::get<0>(p.getOrien()));
    ASSERT_EQ(0, std::get<1>(p.getOrien()));
    ASSERT_EQ(0, std::get<2>(p.getOrien()));
}

TEST(Velocity, PositiveNos)
{
    ASSERT_EQ(0, std::get<0>(p.getVel()));
    ASSERT_EQ(0, std::get<1>(p.getVel()));
    ASSERT_EQ(0, std::get<2>(p.getVel()));
}

TEST(Temperature, PositiveNos)
{
    ASSERT_EQ(69, std::get<0>(p.getMinMaxTemp()));
    ASSERT_EQ(69, std::get<1>(p.getMinMaxTemp()));
}

TEST(TimeOfFlight, PositiveNos)
{
    ASSERT_EQ(80, p.getTOF());
}

TEST(Height, PositiveNos)
{
    ASSERT_EQ(70, p.getHeight());
}

TEST(Battery, PositiveNos)
{
    ASSERT_EQ(80, p.getBattery());
}

TEST(Barometer, PositiveNos)
{
    ASSERT_EQ(64.45, p.getBar());
}

TEST(Time, PositiveNos)
{
    ASSERT_EQ(4, p.getMotorOnTime());
}

TEST(Acceleration, PositiveNos)
{
    ASSERT_EQ(-5.00, std::get<0>(p.getVel()));
    ASSERT_EQ(-3.00, std::get<1>(p.getVel()));
    ASSERT_EQ(-1009.00, std::get<2>(p.getVel()));
    
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    std::cout << "Test";
    std::string s{R"(pitch:0;roll:0;yaw:0;
                    vgx:0;vgy:0;vgz:0;
                    templ:69;temph:72;
                    tof:80;h:70;bat:80;baro:64.45;time:4;
                    agx:-5.00;agy:-3.00;agz:-1009.00;)"};
    p(s);
    return RUN_ALL_TESTS();
}