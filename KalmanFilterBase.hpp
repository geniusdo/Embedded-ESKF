// The MIT License (MIT)
// Copyright 2024 Liu Chuangye @ chuangyeliu0206@gmail.com
// Copyright (c) 2015 Markus Herb

#ifndef FILTER_KALMANFILTERBASE_HPP_
#define FILTER_KALMANFILTERBASE_HPP_

#include <Eigen/Dense>

namespace Filter
{
    using namespace Eigen;

    template <typename T, int N>
    using Covariance = Matrix<T, N, N>;

    /**
     * @brief Abstract base class for all Kalman Filters
     *
     * @param StateType The vector-type of the system state
     */
    template <class StateType>
    class KalmanFilterBase
    {
    public:
        static_assert(/*StateType::RowsAtCompileTime == Dynamic ||*/ StateType::RowsAtCompileTime > 0,
                      "State vector must contain at least 1 element" /* or be dynamic */);
        static_assert(StateType::ColsAtCompileTime == 1, "State type must be a column vector");

        //! Numeric scalar type
        typedef typename StateType::Scalar T;

        //! Type of the state vector
        typedef StateType State;

        //! Dimension of the state vector
        static constexpr int Dim = StateType::RowsAtCompileTime;

    protected:
        //! Estimated state
        State x;

        //! Covariance of the estimated state
        Covariance<T, Dim> cov;

    public:
        /// @brief get the state
        /// @return state
        const State &getState() const
        {
            return x;
        }

        /// @brief get the covariance of the state
        /// @return covariance of the state
        const Covariance<T, Dim> &getCovariance() const
        {
            return cov;
        }

        /// @brief
        /// @param initialState
        void init(const State &initialState)
        {
            x = initialState;
        }

    protected:
        /**
         * @brief Protected constructor
         */
        KalmanFilterBase()
        {
        }
    };
}

#endif
