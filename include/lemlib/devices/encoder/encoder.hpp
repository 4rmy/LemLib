#pragma once

#include "pros/rotation.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"

namespace lemlib {
class Encoder {
    public:
        enum class POLL_FREQUENCY {
            V5_MOTOR = 10,
            V5_ROTATION = 10,
            V5_OPTICAL = 10
        };
        /**
         * @brief Construct a new Encoder
         *
         */
        Encoder(const int pollRate) : pollRate(pollRate) {}

        /**
         * @brief Get the angle rotated by the encoder, in radians
         *
         * @return float angle rotated by the encoder, in radians
         */
        virtual float getAngle() = 0;

        /**
         * @brief Get the angle rotated by the encoder since the last time it was checked, in radians
         *
         * @param update whether to update the last angle measured by the encoder. True by default
         * @return float angle rotated by the encoder, in radians
         */
        float getAngleDelta(bool update = true);

        /**
         * @brief Reset the encoder
         *
         * @return true encoder calibration failed
         * @return false encoder calibration succeeded
         */
        virtual bool reset() = 0;

        int getPollRate() const;
    protected:
        float lastAngle = 0;
        const int pollRate;
};
} // namespace lemlib