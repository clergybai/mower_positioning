#include "mower_positioning_core.h"


const mower::positioning::StateT &mower::positioning::mower_positioning_core::predict(double vx, double steering_angle, double dt) {
    sys.setDt(dt);
    u.v() = vx;
    u.steering_angle() = steering_angle;
    return ekf.predict(sys, u);
}

const mower::positioning::StateT &mower::positioning::mower_positioning_core::updatePosition(double x, double y, double covariance) {
    pos_m.x_pos() = x;
    pos_m.y_pos() = y;

    Kalman::Covariance<PositionMeasurementT> c;
    c.setIdentity();
    c *= covariance;

    pm.setCovariance(c);

    return ekf.update(pm, pos_m);
}

const mower::positioning::StateT &
mower::positioning::mower_positioning_core::updateOrientation(double theta, double covariance) {
    orient_m.theta() = theta;
    Kalman::Covariance<OrientationMeasurementT> c;
    c.setIdentity();
    c *= covariance;

    om.setCovariance(c);

    return ekf.update(om, orient_m);
}

const mower::positioning::StateT &
mower::positioning::mower_positioning_core::updateOrientation2(double vx, double vy, double covariance) {
    orient_m2.vx() = vx;
    orient_m2.vy() = vy;

    Kalman::Covariance<OrientationMeasurementT2> c;
    c.setIdentity();
    c *= covariance;

    om2.setCovariance(c);

    return ekf.update(om2, orient_m2);
}
const mower::positioning::StateT &
mower::positioning::mower_positioning_core::updateSpeed(double vx, double vr, double covariance) {
    speed_m.vx() = vx;
    speed_m.vr() = vr;
//
//    Kalman::Covariance<SpeedMeasurementT> c;
//    c.setIdentity();
//    c *= covariance;
//
//    sm.setCovariance(c);

    return ekf.update(sm, speed_m);
}

const mower::positioning::StateT &mower::positioning::mower_positioning_core::getState() {
    return ekf.getState();
}

const Kalman::Covariance<mower::positioning::StateT> &mower::positioning::mower_positioning_core::getCovariance() {
    return ekf.getCovariance();
}

mower::positioning::mower_positioning_core::mower_positioning_core() {
//    Kalman::Covariance<StateT> c;
//    c.setIdentity();
//    c *= 0.001;
//    sys.setCovariance(c);
    setState(0,0,0,0,0);
}

void mower::positioning::mower_positioning_core::setState(double px, double py, double theta, double vx, double vr) {
    StateT x;
    x.setZero();
    x.x() = px;
    x.y() = py;
    x.theta() = theta;
    x.vx() = vx;
    x.vr() = vr;
    this->ekf.init(x);
    Kalman::Covariance<StateT> c;
    c.setIdentity();
    this->ekf.setCovariance(c);
}

void mower::positioning::mower_positioning_core::setAntennaOffset(double offset_x, double offset_y) {
    pm.antenna_offset_x = om2.antenna_offset_x = offset_x;
    pm.antenna_offset_y = om2.antenna_offset_y = offset_y;
}

void mower::positioning::mower_positioning_core::setWheelbase(double wheelbase) {
    sys.setWheelbase(wheelbase);
}

void mower::positioning::mower_positioning_core::setVehicleType(const std::string& vehicle_type) {
    sys.setVehicleType(vehicle_type); // Added
}

