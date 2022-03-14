#pragma once
#include <functional>


struct SpeedUpdate {
	double velocity_mps;
};

struct CarDetected {
	double distance_m;
	double velocity_mps;
};

struct BrakeCommand {
	double time_to_collision_s;
};

struct SpeedLimitDetected {
	uint16_t speed_mps;
};


using SpeedUpdateCallback = std::function<void(const SpeedUpdate&)>;
using CarDetectedCallback = std::function<void(const CarDetected&)>;
using SpeedLimitCallback = std::function<void(const SpeedLimitDetected&)>;

struct IServiseBus {
	virtual ~IServiseBus() = default;
	virtual void publish(const BrakeCommand&) = 0;
	virtual void subscribe(SpeedUpdateCallback) = 0;
	virtual void subscribe(CarDetectedCallback) = 0;
	virtual void subscribe(SpeedLimitCallback) = 0;
};


struct AutoBrake {
public:
	AutoBrake(IServiseBus& bus)
		: collision_threshold_s_{ 5 }
		, speed_mps_{}
		, last_speed_limit_{ 39 }
	{
		bus.subscribe([this, &bus](const SpeedUpdate& update) {
			if (update.velocity_mps > static_cast<double>(last_speed_limit_)) {
				bus.publish(BrakeCommand{ 0 });
			}
			/*speed_mps_ = (update.velocity_mps > static_cast<double>(last_speed_limit_))
				? last_speed_limit_ : update.velocity_mps;*/
			speed_mps_ = update.velocity_mps;
		});

		bus.subscribe([this, &bus](const CarDetected& cd) {
			const auto relative_speed_mps = speed_mps_ - cd.velocity_mps;
			const auto time_to_collision = cd.distance_m / relative_speed_mps;
			if (time_to_collision > 0 && time_to_collision <= collision_threshold_s_) {
				bus.publish(BrakeCommand{ time_to_collision });
			}
		});

		bus.subscribe([this, &bus](const SpeedLimitDetected& sl) {
			last_speed_limit_ = sl.speed_mps;
			if (speed_mps_ > static_cast<double>(last_speed_limit_)) {
				bus.publish(BrakeCommand{ 0 });
			}
		});
	}

	void set_collision_threshold_s(const double x) {
		if (x < 1LL) {
			throw std::invalid_argument{
				"Collision less then 1."
			};
		}
		collision_threshold_s_ = x;
	}

	double get_collision_threshold_s() const {
		return collision_threshold_s_;
	}

	double get_speed_mps() const {
		return speed_mps_;
	}

	uint16_t get_last_speed_limit_mps() const {
		return last_speed_limit_;
	}
private:
	double collision_threshold_s_;
	double speed_mps_;
	uint16_t last_speed_limit_;
};

