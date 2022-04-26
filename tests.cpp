//#include <iostream>
//#include "header.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "AutoBrake.h"

struct MockServiceBus : IServiseBus {
	MOCK_METHOD1(publish, void(const BrakeCommand&));
	MOCK_METHOD1(subscribe, void(SpeedUpdateCallback));
	MOCK_METHOD1(subscribe, void(CarDetectedCallback));
	MOCK_METHOD1(subscribe, void(SpeedLimitCallback));
};

using ::testing::_;
using ::testing::A;
using ::testing::Field;
using ::testing::DoubleEq;
using ::testing::NiceMock;
using ::testing::StrictMock;
using ::testing::Invoke;

struct NiceAutoBrakeTest : ::testing::Test {
	NiceMock<MockServiceBus> bus{};
	AutoBrake auto_brake{ bus };
};

struct StrictAutoBrakeTest : ::testing::Test {
	StrictAutoBrakeTest() {
		EXPECT_CALL(bus, subscribe(A<CarDetectedCallback>()))
		.Times(1)
		.WillOnce(Invoke([this](const auto & x) {
			car_detected_callback_ = x;
		}));
		EXPECT_CALL(bus, subscribe(A<SpeedUpdateCallback>()))
		.Times(1)
		.WillOnce(Invoke([this](const auto & x) {
			speed_update_callback_ = x;
		}));
		EXPECT_CALL(bus, subscribe(A<SpeedLimitCallback>()))
		.Times(1)
		.WillOnce(Invoke([this](const auto & x) {
			speed_limit_callback_ = x;
		}));
	}
	SpeedUpdateCallback speed_update_callback_{};
	CarDetectedCallback car_detected_callback_{};
	SpeedLimitCallback speed_limit_callback_{};
	StrictMock<MockServiceBus> bus{};
};


TEST_F(NiceAutoBrakeTest, InitialSpeedIsZero) {
	EXPECT_EQ(auto_brake.get_speed_mps(), 0);
}

TEST_F(NiceAutoBrakeTest, InitialSensivityIsFive) {
	EXPECT_EQ(auto_brake.get_collision_threshold_s(), 5L);
}

TEST_F(NiceAutoBrakeTest, SensivityGreaterThenOne) {
	EXPECT_THROW(auto_brake.set_collision_threshold_s(0.5), 
				std::invalid_argument);
}

TEST_F(NiceAutoBrakeTest, LastSpeedLimitIsThirtyNine) {
	EXPECT_EQ(auto_brake.get_last_speed_limit_mps(), 39L);
}

TEST_F(StrictAutoBrakeTest, SpeedIsSave) {
	AutoBrake auto_brake{ bus };
	speed_update_callback_(SpeedUpdate{ 39L });
	EXPECT_EQ(auto_brake.get_speed_mps(), 39L);
	speed_update_callback_(SpeedUpdate{ 20L });
	EXPECT_EQ(auto_brake.get_speed_mps(), 20L);
	speed_update_callback_(SpeedUpdate{ 0L });
	EXPECT_EQ(auto_brake.get_speed_mps(), 0L);
}

TEST_F(StrictAutoBrakeTest, SpeedLimitIsSave) {
	AutoBrake auto_brake{ bus };
	speed_limit_callback_(SpeedLimitDetected{ 40L });
	EXPECT_EQ(auto_brake.get_last_speed_limit_mps(), 40L);
	speed_limit_callback_(SpeedLimitDetected{ 35L });
	EXPECT_EQ(auto_brake.get_last_speed_limit_mps(), 35L);
	speed_limit_callback_(SpeedLimitDetected{ 20L });
	EXPECT_EQ(auto_brake.get_last_speed_limit_mps(), 20L);
}

TEST_F(StrictAutoBrakeTest, CollisionAlert) {
	EXPECT_CALL(bus, publish(
				Field(&BrakeCommand::time_to_collision_s,
				DoubleEq(0L))
			)
	).Times(1);
	EXPECT_CALL(bus, publish(
				Field(&BrakeCommand::time_to_collision_s,
				DoubleEq(1L))
			)
	).Times(1);
	AutoBrake auto_brake{ bus };
	auto_brake.set_collision_threshold_s(10L);
	speed_update_callback_(SpeedUpdate{ 100L });
	car_detected_callback_(CarDetected{ 100L, 0L });
}

TEST_F(StrictAutoBrakeTest, CollisionNoAlert) {
	EXPECT_CALL(bus, publish(
		Field(&BrakeCommand::time_to_collision_s,
			DoubleEq(0L))
	)
	).Times(1);
	AutoBrake auto_brake{ bus };
	auto_brake.set_collision_threshold_s(2L);
	speed_update_callback_(SpeedUpdate{ 100L });
	car_detected_callback_(CarDetected{ 1000L, 50L });
}

TEST_F(StrictAutoBrakeTest, SpeedLimitAlert) {
	EXPECT_CALL(bus, publish(
		Field(&BrakeCommand::time_to_collision_s,
			DoubleEq(0L))
					)
	).Times(1);

	AutoBrake auto_brake{ bus };
	speed_limit_callback_(SpeedLimitDetected{ 35L });
	speed_update_callback_(SpeedUpdate{ 40L });
}

TEST_F(StrictAutoBrakeTest, SpeedLimitNoAlert) {
	EXPECT_CALL(bus, publish(A<const BrakeCommand&>()))
	.Times(0);

	AutoBrake auto_brake{ bus };
	speed_update_callback_(SpeedUpdate{ 34L });
	speed_limit_callback_(SpeedLimitDetected{ 35L });
}

TEST_F(StrictAutoBrakeTest, SpeedLimitOneAlert) {
	EXPECT_CALL(bus, publish(
		Field(&BrakeCommand::time_to_collision_s,
			DoubleEq(0L))
					)
	).Times(1);

	AutoBrake auto_brake{ bus };
	speed_limit_callback_(SpeedLimitDetected{ 35L });
	speed_update_callback_(SpeedUpdate{ 30L });
	speed_limit_callback_(SpeedLimitDetected{ 25L });
}
