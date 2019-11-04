package com.irurueta.navigation.inertial;

import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

import java.util.Objects;

/**
 * Base class for body kinematics resolved either along ECI or ECEF frames.
 */
@SuppressWarnings("WeakerAccess")
public abstract class ECIorECEFKinematics<T extends ECIorECEFKinematics> {
    /**
     * Specific force of body frame with respect ECI or ECEF frame resolved along body-frame x-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     */
    double mFx;

    /**
     * Specific force of body frame with respect ECI or ECEF frame resolved along body-frame y-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     */
    double mFy;

    /**
     * Specific force of body frame with respect ECI or ECEF frame resolved along body-frame z-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     */
    double mFz;

    /**
     * Angular rate of body frame with respect ECI or ECEF frame, resolved about body-frame x-axis, averaged
     * over time interval and expressed in radians per second (rad/s).
     */
    double mAngularRateX;

    /**
     * Angular rate of body frame with respect ECI or ECEF frame, resolved about body-frame y-axis, averaged
     * over time interval and expressed in radians per second (rad/s).
     */
    double mAngularRateY;

    /**
     * Angular rate of body frame with respect ECI or ECEF frame, resolved about body-frame z-axis, averaged
     * over time interval and expressed in radians per second (rad/s).
     */
    double mAngularRateZ;

    /**
     * Constructor.
     */
    public ECIorECEFKinematics() {
    }

    /**
     * Constructor.
     *
     * @param fx Specific force of body frame with respect ECI or ECEF frame resolved along body-frame x-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fy Specific force of body frame with respect ECI or ECEF frame resolved along body-frame y-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fz Specific force of body frame with respect ECI or ECEF frame resolved along body-frame z-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     */
    public ECIorECEFKinematics(final double fx, final double fy, final double fz) {
        setSpecificForceCoordinates(fx, fy, fz);
    }

    /**
     * Constructor.
     *
     * @param fx           Specific force of body frame with respect ECI or ECEF frame resolved along body-frame x-axis,
     *                     averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fy           Specific force of body frame with respect ECI or ECEF frame resolved along body-frame y-axis,
     *                     averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fz           Specific force of body frame with respect ECI or ECEF frame resolved along body-frame z-axis,
     *                     averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param angularRateX Angular rate of body frame with respect ECI or ECEF frame, resolved about body-frame
     *                     x-axis, averaged over time interval and expressed in radians per second (rad/s).
     * @param angularRateY Angular rate of body frame with respect ECI or ECEF frame, resolved about body-frame
     *                     y-axis, averaged over time interval and expressed in radians per second (rad/s).
     * @param angularRateZ Angular rate of body frame with respect ECI or ECEF frame, resolved about body-frame
     *                     z-axis, averaged over time interval and expressed in radians per second (rad/s).
     */
    public ECIorECEFKinematics(final double fx, final double fy, final double fz,
                               final double angularRateX, final double angularRateY,
                               final double angularRateZ) {
        setSpecificForceCoordinates(fx, fy, fz);
        setAngularRateCoordinates(angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Constructor.
     *
     * @param specificForceX Specific force of body frame with respect ECI or ECEF frame resolved along body-frame
     *                       x-axis, averaged over time interval.
     * @param specificForceY Specific force of body frame with respect ECI or ECEF frame resolved along body-frame
     *                       y-axis, averaged over time interval.
     * @param specificForceZ Specific force of body frame with respect ECI or ECEF frame resolved along body-frame
     *                       z-axis, averaged over time interval.
     */
    public ECIorECEFKinematics(final Acceleration specificForceX,
                               final Acceleration specificForceY,
                               final Acceleration specificForceZ) {
        setSpecificForceCoordinates(specificForceX, specificForceY, specificForceZ);
    }

    /**
     * Constructor.
     *
     * @param angularSpeedX Angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame
     *                      x-axis, averaged over time interval.
     * @param angularSpeedY Angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame
     *                      y-axis, averaged over time interval.
     * @param angularSpeedZ Angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame
     *                      z-axis, averaged over time interval.
     */
    public ECIorECEFKinematics(final AngularSpeed angularSpeedX,
                               final AngularSpeed angularSpeedY,
                               final AngularSpeed angularSpeedZ) {
        setAngularSpeedCoordinates(angularSpeedX, angularSpeedY, angularSpeedZ);
    }

    /**
     * Constructor.
     *
     * @param specificForceX Specific force of body frame with respect ECI or ECEF frame resolved along body-frame
     *                       x-axis, averaged over time interval.
     * @param specificForceY Specific force of body frame with respect ECI or ECEF frame resolved along body-frame
     *                       y-axis, averaged over time interval.
     * @param specificForceZ Specific force of body frame with respect ECI or ECEF frame resolved along body-frame
     *                       z-axis, averaged over time interval.
     * @param angularSpeedX Angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame
     *                      x-axis, averaged over time interval.
     * @param angularSpeedY Angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame
     *                      y-axis, averaged over time interval.
     * @param angularSpeedZ Angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame
     *                      z-axis, averaged over time interval.
     */
    public ECIorECEFKinematics(final Acceleration specificForceX,
                          final Acceleration specificForceY,
                          final Acceleration specificForceZ,
                          final AngularSpeed angularSpeedX,
                          final AngularSpeed angularSpeedY,
                          final AngularSpeed angularSpeedZ) {
        setSpecificForceCoordinates(specificForceX, specificForceY, specificForceZ);
        setAngularSpeedCoordinates(angularSpeedX, angularSpeedY, angularSpeedZ);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public ECIorECEFKinematics(final T input) {
        copyFrom(input);
    }

    /**
     * Gets specific force of body frame with respect ECI or ECEF frame resolved along body-frame x-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     *
     * @return specific force of body frame with respect ECI or ECEF frame resolved along body-frame x-axis.
     */
    public double getFx() {
        return mFx;
    }

    /**
     * Sets specific force of body frame with respect ECI or ECEF frame resolved along body-frame x-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     *
     * @param fx specific force of body frame with respect ECI or ECEF frame resolved along body-frame x-axis.
     */
    public void setFx(final double fx) {
        mFx = fx;
    }

    /**
     * Gets specific force of body frame with respect ECI or ECEF frame resolved along body-frame y-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     *
     * @return specific force of body frame with respect ECI or ECEF frame resolved along body-frame y-axis.
     */
    public double getFy() {
        return mFy;
    }

    /**
     * Sets specific force of body frame with respect ECI or ECEF frame resolved along body-frame y-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     *
     * @param fy specific force of body frame with respect ECI or ECEF frame resolved along body-frame y-axis.
     */
    public void setFy(final double fy) {
        mFy = fy;
    }

    /**
     * Gets specific force of body frame with respect ECI or ECEF frame resolved along body-frame z-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     *
     * @return specific force of body frame with respect ECI or ECEF frame resolved along body-frame z-axis.
     */
    public double getFz() {
        return mFz;
    }

    /**
     * Sets specific force of body frame with respect ECI or ECEF frame resolved along body-frame z-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     *
     * @param fz specific force of body frame with respect ECI or ECEF frame resolved along body-frame z-axis.
     */
    public void setFz(final double fz) {
        mFz = fz;
    }

    /**
     * Sets specific force coordinates of body frame with respect ECI or ECEF frame resolved along body-frame
     * axes, averaged over time interval and expressed in meters per squared second (m/s^2).
     *
     * @param fx Specific force of body frame with respect ECI or ECEF frame resolved along body-frame x-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fy Specific force of body frame with respect ECI or ECEF frame resolved along body-frame y-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fz Specific force of body frame with respect ECI or ECEF frame resolved along body-frame z-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     */
    public void setSpecificForceCoordinates(final double fx, final double fy, final double fz) {
        mFx = fx;
        mFy = fy;
        mFz = fz;
    }

    /**
     * Gets specific force of body frame with respect ECI or ECEF frame resolved along body-frame x-axis, averaged
     * over time interval.
     *
     * @param result instance where specific force of body frame with respect ECI or ECEF frame resolved along
     *               body-frame x-axis will be stored.
     */
    public void getSpecificForceX(final Acceleration result) {
        result.setValue(mFx);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets specific force of body frame with respect ECI or ECEF frame resolved along body-frame x-axis, averaged
     * over time interval.
     *
     * @return a new instance with specific force of body frame with respect ECI or ECEF frame resolved along
     * body-frame x-axis.
     */
    public Acceleration getSpecificForceX() {
        return new Acceleration(mFx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets specific force of body frame with respect ECI or ECEF frame resolved along body-frame x-axis, averaged
     * over time interval.
     *
     * @param specificForceX specific force of body frame with respect ECI or ECEF frame resolved along
     *                       body-frame x-axis that will be set.
     */
    public void setSpecificForceX(final Acceleration specificForceX) {
        mFx = AccelerationConverter.convert(specificForceX.getValue().doubleValue(),
                specificForceX.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets specific force of body frame with respect ECI or ECEF frame resolved along body-frame y-axis, averaged
     * over time interval.
     *
     * @param result instance where specific force of body frame with respect ECI or ECEF frame resolved along
     *               body-frame y-axis will be stored.
     */
    public void getSpecificForceY(final Acceleration result) {
        result.setValue(mFy);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets specific force of body frame with respect ECI or ECEF frame resolved along body-frame y-axis, averaged
     * over time interval.
     *
     * @return a new instance with specific force of body frame with respect ECI or ECEF frame resolved along
     * body-frame y-axis.
     */
    public Acceleration getSpecificForceY() {
        return new Acceleration(mFy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets specific force of body frame with respect ECI or ECEF frame resolved along body frame y-axis, averaged
     * over time interval.
     *
     * @param specificForceY specific force of body frame with respect ECI or ECEF frame resolved along
     *                       body-frame y-axis that will be set.
     */
    public void setSpecificForceY(final Acceleration specificForceY) {
        mFy = AccelerationConverter.convert(specificForceY.getValue().doubleValue(),
                specificForceY.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets specific force of body frame with respect ECI or ECEF frame resolved along body-frame z-axis, averaged
     * over time interval.
     *
     * @param result instance where specific force of body frame with respect ECI or ECEF frame resolved along
     *               body-frame z-axis will be stored.
     */
    public void getSpecificForceZ(final Acceleration result) {
        result.setValue(mFz);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets specific force of body frame with respect ECI or ECEF frame resolved along body-frame z-axis, averaged
     * over time interval.
     *
     * @return a new instance with specific force of body frame with respect ECI or ECEF frame resolved along
     * body-frame z-axis.
     */
    public Acceleration getSpecificForceZ() {
        return new Acceleration(mFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets specific force of body frame with respect ECI or ECEF frame resolved along body-frame z-axis, averaged
     * over time interval.
     *
     * @param specificForceZ specific force of body frame with respect ECI or ECEF frame resolved along
     *                       body-frame z-axis that will be set.
     */
    public void setSpecificForceZ(final Acceleration specificForceZ) {
        mFz = AccelerationConverter.convert(specificForceZ.getValue().doubleValue(),
                specificForceZ.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets specific force coordinates of body frame with respect ECI or ECEF frame resolved along body-frame axes,
     * averaged over time interval.
     *
     * @param specificForceX Specific force of body frame with respect ECI or ECEF frame resolved along body-frame
     *                       x-axis, averaged over time interval.
     * @param specificForceY Specific force of body frame with respect ECI or ECEF frame resolved along body-frame
     *                       y-axis, averaged over time interval.
     * @param specificForceZ Specific force of body frame with respect ECI or ECEF frame resolved along body-frame
     *                       z-axis, averaged over time interval.
     */
    public void setSpecificForceCoordinates(final Acceleration specificForceX,
                                            final Acceleration specificForceY,
                                            final Acceleration specificForceZ) {
        setSpecificForceX(specificForceX);
        setSpecificForceY(specificForceY);
        setSpecificForceZ(specificForceZ);
    }

    /**
     * Gets angular rate of body frame with respect ECI or ECEF frame, resolved about body-frame x-axis, averaged
     * over time interval and expressed in radians per second (rad/s).
     *
     * @return angular rate of body frame with respect ECI or ECEF frame resolved about body-frame x-axis.
     */
    public double getAngularRateX() {
        return mAngularRateX;
    }

    /**
     * Sets angular rate of body frame with respect ECI or ECEF frame, resolved about body-frame x-axis,
     * over time interval and expressed in radians per second (rad/s).
     *
     * @param angularRateX angular rate of body frame with respect ECI or ECEF frame resolved
     *                     about body-frame x-axis.
     */
    public void setAngularRateX(final double angularRateX) {
        mAngularRateX = angularRateX;
    }

    /**
     * Gets angular rate of body frame with respect ECI or ECEF frame, resolved about body-frame y-axis, averaged
     * over time interval and expressed in radians per second (rad/s).
     *
     * @return angular rate of body frame with respect ECI or ECEF frame resolved about body-frame y-axis.
     */
    public double getAngularRateY() {
        return mAngularRateY;
    }

    /**
     * Sets angular rate of body frame with respect ECI or ECEF frame, resolved about body-frame y-axis, averaged
     * over time interval and expressed in radians per second (rad/s).
     *
     * @param angularRateY angular rate of body frame with respect ECI or ECEF frame resolved
     *                     about body-frame y-axis.
     */
    public void setAngularRateY(final double angularRateY) {
        mAngularRateY = angularRateY;
    }

    /**
     * Gets angular rate of body frame with respect ECI or ECEF frame, resolved about body-frame z-axis, averaged
     * over time interval and expressed in radians per second (rad/s).
     *
     * @return angular rate of body frame with respect ECI or ECEF frame resolved about body-frame z-axis.
     */
    public double getAngularRateZ() {
        return mAngularRateZ;
    }

    /**
     * Sets angular rate of body frame with respect ECI or ECEF frame, resolved about body-frame z-axis, averaged
     * over time interval and expressed in radians per second (rad/s).
     *
     * @param angularRateZ angular rate of body frame with respect ECI or ECEF frame resolved
     *                     about body-frame z-axis.
     */
    public void setAngularRateZ(final double angularRateZ) {
        mAngularRateZ = angularRateZ;
    }

    /**
     * Sets angular rate coordinates of body frame with respect ECI or ECEF frame, resolved about body-frame axes,
     * averaged over time interval and expressed in radians per second (rad/s).
     *
     * @param angularRateX Angular rate of body frame with respect ECI or ECEF frame, resolved about body-frame
     *                     x-axis, averaged over time interval and expressed in radians per second (rad/s).
     * @param angularRateY Angular rate of body frame with respect ECI or ECEF frame, resolved about body-frame
     *                     y-axis, averaged over time interval and expressed in radians per second (rad/s).
     * @param angularRateZ Angular rate of body frame with respect ECI or ECEF frame, resolved about body-frame
     *                     z-axis, averaged over time interval and expressed in radians per second (rad/s).
     */
    public void setAngularRateCoordinates(final double angularRateX,
                                          final double angularRateY, final double angularRateZ) {
        mAngularRateX = angularRateX;
        mAngularRateY = angularRateY;
        mAngularRateZ = angularRateZ;
    }

    /**
     * Gets angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame x-axis, averaged
     * over time interval.
     *
     * @param result instance where angular speed of body frame with respect ECI or ECEF frame resolved about
     *               body-frame x-axis will be stored.
     */
    public void getAngularSpeedX(final AngularSpeed result) {
        result.setValue(mAngularRateX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame x-axis, averaged
     * over time interval.
     *
     * @return a new instance of angular speed of body frame with respect ECI or ECEF frame resolved about
     * body-frame X-axis.
     */
    public AngularSpeed getAngularSpeedX() {
        return new AngularSpeed(mAngularRateX, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame x-axis, averaged
     * over time interval.
     *
     * @param angularSpeedX angular speed of body frame with respect ECI or ECEF frame resolved about body-frame
     *                      x-axis that will be set.
     */
    public void setAngularSpeedX(final AngularSpeed angularSpeedX) {
        mAngularRateX = AngularSpeedConverter.convert(angularSpeedX.getValue().doubleValue(),
                angularSpeedX.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame y-axis, averaged
     * over time interval.
     *
     * @param result instance where angular speed of body frame with respect ECI or ECEF frame resolved about
     *               body-frame y-axis will be stored.
     */
    public void getAngularSpeedY(final AngularSpeed result) {
        result.setValue(mAngularRateY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame y-axis, averaged
     * over time interval.
     *
     * @return a new instance of angular speed of body frame with respect ECI or ECEF frame resolved about
     * body-frame y-axis.
     */
    public AngularSpeed getAngularSpeedY() {
        return new AngularSpeed(mAngularRateY, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame y-axis, averaged
     * over time interval.
     *
     * @param angularSpeedY angular speed of body frame with respect ECI or ECEF frame resolved about body-frame
     *                      y-axis that will be set.
     */
    public void setAngularSpeedY(final AngularSpeed angularSpeedY) {
        mAngularRateY = AngularSpeedConverter.convert(angularSpeedY.getValue().doubleValue(),
                angularSpeedY.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame z-axis, averaged
     * over time interval.
     *
     * @param result instance where angular speed of body frame with respect ECI or ECEF frame resolved about
     *               body-frame z-axis will be stored.
     */
    public void getAngularSpeedZ(final AngularSpeed result) {
        result.setValue(mAngularRateZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame z-axis, averaged
     * over time interval.
     *
     * @return a new instance of angular speed of body frame with respect ECI or ECEF frame resolved about
     * body-frame z-axis.
     */
    public AngularSpeed getAngularSpeedZ() {
        return new AngularSpeed(mAngularRateZ, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame y-axis, averaged
     * over time interval.
     *
     * @param angularSpeedZ angular speed of body frame with respect ECI or ECEF frame resolved about body-frame
     *                      z-axis, that will be set.
     */
    public void setAngularSpeedZ(final AngularSpeed angularSpeedZ) {
        mAngularRateZ = AngularSpeedConverter.convert(angularSpeedZ.getValue().doubleValue(),
                angularSpeedZ.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets angular speed coordinates of body frame with respect ECI or ECEF frame, resolved about body-frame axes,
     * averaged over time interval.
     *
     * @param angularSpeedX Angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame
     *                      x-axis, averaged over time interval.
     * @param angularSpeedY Angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame
     *                      y-axis, averaged over time interval.
     * @param angularSpeedZ Angular speed of body frame with respect ECI or ECEF frame, resolved about body-frame
     *                      z-axis, averaged over time interval.
     */
    public void setAngularSpeedCoordinates(final AngularSpeed angularSpeedX,
                                           final AngularSpeed angularSpeedY,
                                           final AngularSpeed angularSpeedZ) {
        setAngularSpeedX(angularSpeedX);
        setAngularSpeedY(angularSpeedY);
        setAngularSpeedZ(angularSpeedZ);
    }

    /**
     * Gets norm of specific force expressed in meters per squared second (m/s^2).
     *
     * @return norm of specific force.
     */
    public double getSpecificForceNorm() {
        return Math.sqrt(mFx * mFx + mFy * mFy + mFz * mFz);
    }

    /**
     * Gets norm of specific force.
     *
     * @param result instance where result will be stored.
     */
    public void getSpecificForceNormAsAcceleration(final Acceleration result) {
        result.setValue(getSpecificForceNorm());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets norm of specific force.
     *
     * @return a new acceleration instance containing norm of specific force.
     */
    public Acceleration getSpecificForceNormAsAcceleration() {
        return new Acceleration(getSpecificForceNorm(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets norm of angular rate expressed in radians per second (rad/s).
     *
     * @return norm of angular rate.
     */
    public double getAngularRateNorm() {
        return Math.sqrt(mAngularRateX * mAngularRateX
                + mAngularRateY * mAngularRateY + mAngularRateZ * mAngularRateZ);
    }

    /**
     * Gets norm of angular rate.
     *
     * @param result instance where norm of angular rate will be stored.
     */
    public void getAngularSpeedNorm(final AngularSpeed result) {
        result.setValue(getAngularRateNorm());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets norm of angular rate.
     *
     * @return norm of angular rate.
     */
    public AngularSpeed getAngularSpeedNorm() {
        return new AngularSpeed(getAngularRateNorm(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final T output) {
        output.mFx = mFx;
        output.mFy = mFy;
        output.mFz = mFz;
        output.mAngularRateX = mAngularRateX;
        output.mAngularRateY = mAngularRateY;
        output.mAngularRateZ = mAngularRateZ;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final T input) {
        mFx = input.mFx;
        mFy = input.mFy;
        mFz = input.mFz;
        mAngularRateX = input.mAngularRateX;
        mAngularRateY = input.mAngularRateY;
        mAngularRateZ = input.mAngularRateZ;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mFx, mFy, mFz, mAngularRateX, mAngularRateY, mAngularRateZ);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final T other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum allowed difference between specific force and angular
     *                  rate coordinates.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(final T other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mFx - other.mFx) <= threshold
                && Math.abs(mFy - other.mFy) <= threshold
                && Math.abs(mFz - other.mFz) <= threshold
                && Math.abs(mAngularRateX - other.mAngularRateX) <= threshold
                && Math.abs(mAngularRateY - other.mAngularRateY) <= threshold
                && Math.abs(mAngularRateZ - other.mAngularRateZ) <= threshold;
    }
}