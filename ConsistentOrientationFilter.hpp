// The MIT License (MIT)
// Copyright 2024 Liu Chuangye @ chuangyeliu0206@gmail.com
#ifndef FILTER_CONSISTENTORIENTATIONFILTER_HPP_
#define FILTER_CONSISTENTORIENTATIONFILTER_HPP_

#include "ErrorStateKalmanFilter.hpp"
#include "EmbeddedLie.hpp"
namespace Filter
{
    namespace ConsistentOrientation
    {
        using namespace EmbeddedLie;

        /// @brief error vector [dq]
        using ErrorVec = Matrix<double, 3, 1>;

        using MeasurementVec = Vector3d;

        /// @brief input vector [dt, ang_vel]
        using ControlVec = Matrix<double, 4, 1>;

        using NominalState_t = Quaterniond;

        /// @brief
        class ConsistentOrientationFilter : public ErrorStateKalmanFilter<NominalState_t, ErrorVec, MeasurementVec, ControlVec>
        {
        protected:
            NominalState_t updateNominalState(const NominalState_t &state, const ControlVec &u) override
            {
                NominalState_t nxt_state;
                Vector3d dtheta = (u.tail<3>()) * u(0);
                nxt_state = state * quat_Exp(dtheta);
                nxt_state.normalize();
                return nxt_state;
            }

            SystemJacobian updateSysJacobian(const NominalState_t &state, const ErrorVec &x, const ControlVec &u) override
            {
                return Matrix3d::Identity();
            }

            ControlJacobian updateControlJacobian(const NominalState_t &state, const ErrorVec &x, const ControlVec &u) override
            {
                return state.toRotationMatrix() * u(0);
            }

            SystemJacobian jacobianOfReset(const ErrorVec &x) override
            {
                SystemJacobian J = Matrix3d::Identity();
                J = Matrix3d::Identity() - 0.5 * skew(Vector3d(x.head<3>()));
                return J;
            }

            NominalState_t correctNominalState(const NominalState_t &state, const ErrorVec &x) override
            {
                NominalState_t corrected_state;
                corrected_state = state * quat_Exp(Vector3d(x.head<3>()));
                corrected_state.normalize();
                return corrected_state;
            }

            MeasurementJacobian updateMeasJacobian(const NominalState_t &state, const ErrorVec &x) override
            {
                MeasurementJacobian H = Matrix3d::Zero();
                Vector3d gravity_inG = Vector3d(0, 0, 1);
                // std::cout << "state " << state.w() << " " << state.x() << " " << state.y() << " " << state.z() << std::endl;
                // std::cout << "rotation " << state.toRotationMatrix().transpose()(0, 0) << " " << state.toRotationMatrix().transpose()(1, 1) << " " << state.toRotationMatrix().transpose()(2, 2) << std::endl;
                // std::cout << "skew " << skew(gravity_inG)(0,1) << " " << state.x() << " " << state.y() << " " << state.z() << std::endl;
                H = state.toRotationMatrix().transpose() * skew(gravity_inG);
                return H;
            }

        private:
            float frequency = 200.0;

        public:
            /// @brief set IMU noise and random walk
            /// @param noise unit: rad/s/sqrt(Hz)
            /// @param random_walk  unit: rad/s^2/sqrt(hr)
            /// @note recommend calibrating the IMU noise and random walk using Kalibr Allan deviation calibration
            void setImuParam(const Vector3d &noise, const double frequency = 200.0)
            {
                this->frequency = frequency;
                noiseCov controlNoiseCov = Matrix3d::Zero();
                controlNoiseCov = noise.asDiagonal() * (double)(sqrt(frequency));
                ErrorStateKalmanFilter::setSystemNoiseCovariance(noiseCov::Zero());
                ErrorStateKalmanFilter::setControlNoiseCovariance(controlNoiseCov);
            }

            /// @brief set measurement noise
            /// @param noise
            void setMeasurementParam(const Vector3d &noise)
            {
                ErrorStateKalmanFilter::setMeasurementNoiseCovariance(noise.asDiagonal());
            }

            void setInitialState(const Quaterniond &q)
            {
                NominalState_t state;
                state = q;
                ErrorStateKalmanFilter::setInitialState(state);
            }
        };
    }; // namespace ConsistentOrientation
}; // namesapce Filter

#endif