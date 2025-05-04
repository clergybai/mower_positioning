#ifndef SRC_SYSTEMMODEL_HPP
#define SRC_SYSTEMMODEL_HPP

#include <kalman/LinearizedSystemModel.hpp>

namespace mower {
    namespace positioning {

/**
 * @brief System state vector-type for a 3DOF planar robot
 *
 * This is a system state for a very simple planar robot that
 * is characterized by its (x,y)-Position and angular orientation.
 *
 * @param T Numeric scalar type
 */
        template<typename T>
        class State : public Kalman::Vector<T, 5> {
        public:
            KALMAN_VECTOR(State, T,
            5)

            //! X-position
            static constexpr size_t X = 0;
            //! Y-Position
            static constexpr size_t Y = 1;
            //! Orientation
            static constexpr size_t THETA = 2;
            //! Speed in x dir
            static constexpr size_t VX = 3;
            //! Speed angular
            static constexpr size_t VR = 4;


            T x_pos() const { return (*this)[X]; }

            T y_pos() const { return (*this)[Y]; }

            T theta() const { return (*this)[THETA]; }
            T vx() const { return (*this)[VX]; }
            T vr() const { return (*this)[VR]; }

            T &x_pos() { return (*this)[X]; }

            T &y_pos() { return (*this)[Y]; }

            T &theta() { return (*this)[THETA]; }
            T &vx() { return (*this)[VX]; }
            T &vr() { return (*this)[VR]; }

        };

/**
 * @brief System control-input vector-type for a 3DOF planar robot
 *
 * This is the system control-input of a very simple planar robot that
 * can control the velocity in its current direction as well as the
 * change in direction.
 *
 * @param T Numeric scalar type
 */
        template<typename T>
        class Control : public Kalman::Vector<T, 2> {
        public:
            KALMAN_VECTOR(Control, T,
            2)

            //! Velocity
            static constexpr size_t V = 0;
            //! Angular Rate (Orientation-change)
            static constexpr size_t STEERING_ANGLE = 1;

            T v() const { return (*this)[V]; }

            T steering_angle() const { return (*this)[STEERING_ANGLE]; }

            T &v() { return (*this)[V]; }

            T& steering_angle() { return (*this)[STEERING_ANGLE]; }
        };

/**
 * @brief System model for a simple planar 3DOF robot
 *
 * This is the system model defining how our robot moves from one
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
        template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
        class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase> {
        public:
            //! State type shortcut definition
            typedef mower::positioning::State <T> S;

            //! Control type shortcut definition
            typedef mower::positioning::Control <T> C;

            SystemModel() : dt(0.0), wheelbase(1.0), vehicle_type("differential") {
                Kalman::Covariance<State<T>> c;
                c.setIdentity();
                c *= 0.001;
                this->setCovariance(c);
            }

            void setDt(double dt) {
                this->dt = dt;
            }

            void setWheelbase(double wheelbase) { this->wheelbase = wheelbase; }

            void setVehicleType(const std::string& type) { vehicle_type = type; }

            /**
             * @brief Definition of (non-linear) state transition function
             *
             * This function defines how the system state is propagated through time,
             * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to
             * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
             * the system control input \f$u\f$.
             *
             * @param [in] x The system state in current time-step
             * @param [in] u The control vector input
             * @returns The (predicted) system state in the next time-step
             */
            S f(const S &x, const C &u) const {
                //! Predicted state vector after transition
                S x_;
                T v = u.v();
                T theta = x.theta();
                T vr;

                if (vehicle_type == "differential") {
                    vr = u.steering_angle(); // For differential, steering_angle is dtheta
                } else if (vehicle_type == "ackermann") {
                    T steering_angle = u.steering_angle();
                    vr = (std::abs(steering_angle) > 1e-3) ? (v * std::tan(steering_angle) / wheelbase) : 0.0;
                } else {
                    vr = 0.0; // Unknown type
                }

                // New orientation given by old orientation plus orientation change
                auto newOrientation = theta + vr * dt;
                // Re-scale orientation to [-pi/2 to +pi/2]

                x_.theta() = newOrientation;

                // New x-position given by old x-position plus change in x-direction
                // Change in x-direction is given by the cosine of the (new) orientation
                // times the velocity
                x_.x() = x.x() + std::cos(newOrientation) * v * dt;
                x_.y() = x.y() + std::sin(newOrientation) * v * dt;

                x_.vx() = x.vx();
                x_.vr() = x.vr();
//                x_.vx() = u.v();
//                x_.vr() = u.dtheta();


                // Return transitioned state vector
                return x_;
            }

        protected:
            double dt = 0;
            double wheelbase; // Added: Wheelbase for Ackermann
            std::string vehicle_type; // Added: Vehicle type

            /**
             * @brief Update jacobian matrices for the system state transition function using current state
             *
             * This will re-compute the (state-dependent) elements of the jacobian matrices
             * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
             * current state \f$x\f$.
             *
             * @note This is only needed when implementing a LinearizedSystemModel,
             *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
             *       When using a fully non-linear filter such as the UnscentedKalmanFilter
             *       or its square-root form then this is not needed.
             *
             * @param x The current system state around which to linearize
             * @param u The current system control input
             */
            void updateJacobians(const S &x, const C &u) {
                // F = df/dx (Jacobian of state transition w.r.t. the state)
                this->F.setZero();
                T v = u.v();
                T theta = x.theta();
                T vr;
                if (vehicle_type == "differential") {
                    vr = u.steering_angle();
                } else if (vehicle_type == "ackermann") {
                    T steering_angle = u.steering_angle();
                    vr = (std::abs(steering_angle) > 1e-3) ? (v * std::tan(steering_angle) / wheelbase) : 0.0;
                } else {
                    vr = 0.0;
                }

                // partial derivative of x.x() w.r.t. x.x()
                this->F(S::X, S::X) = 1;
                // partial derivative of x.x() w.r.t. x.theta()
                this->F(S::X, S::THETA) = -std::sin(theta + vr * dt) * v * dt;

                // partial derivative of x.y() w.r.t. x.y()
                this->F(S::Y, S::Y) = 1;
                // partial derivative of x.y() w.r.t. x.theta()
                this->F(S::Y, S::THETA) = std::cos(theta + vr * dt) * v * dt;

                // partial derivative of x.theta() w.r.t. x.theta()
                this->F(S::THETA, S::THETA) = 1;

                // partial derivative of x.theta() w.r.t. x.theta()
                this->F(S::VX, S::VX) = 1;
                // partial derivative of x.theta() w.r.t. x.theta()
                this->F(S::VR, S::VR) = 1;


                // W = df/dw (Jacobian of state transition w.r.t. the noise)
                this->W.setIdentity();
                // TODO: more sophisticated noise modelling
                //       i.e. The noise affects the the direction in which we move as
                //       well as the velocity (i.e. the distance we move)
            }
        };

    } // namespace Robot
}

#endif //SRC_SYSTEMMODEL_HPP
