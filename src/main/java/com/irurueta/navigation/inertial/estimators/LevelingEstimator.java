/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.navigation.inertial.estimators;

import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleConverter;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

/**
 * Leveling is the process of attitude initialization of a body.
 * When the INS is stationary, self-alignment can be used to initialize
 * the roll and pitch with all but the poorest inertial sensors.
 * However, accurate self-alignment of the heading requires
 * aviation-grade gyros or better. Heading is often initialized using
 * a magnetic compass.
 * When the INS is initialized in motion, another navigation system
 * must provide an attitude reference.
 * This class is based on Paul D. Groves. Principles of GNSS Inertial
 * and multi-sensor integrated navigation systemd. 2nd ed. p. 196.
 *
 * Because this implementation neglects effects of Earth rotation on sensed
 * specific force, and also neglects the north component of gravity in a local
 * navigation frame (which is not zero because Earth is not fully spherical),
 * the expected results should be accurate up to about 1e-3 radians
 * (about 0.05 degrees).
 */
public class LevelingEstimator {

    /**
     * Private constructor to prevent instantiation.
     */
    private LevelingEstimator() { }

    /**
     * Gets roll angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param fy y-coordinate of measured body specific force
     *           expressed in meters per squared second (m/s^2).
     * @param fz z-coordinate of measured body specific force
     *           expressed in meters per squared second (m/s^2).
     * @return roll angle expressed in radians.
     */
    public static double getRoll(final double fy, final double fz) {
        return Math.atan2(-fy, -fz);
    }

    /**
     * Gets pitch angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param fx x-coordinate of measured body specific force
     *           expressed in meters per squared second (m/s^2).
     * @param fy y-coordinate of measured body specific force
     *           expressed in meters per squared second (m/s^2).
     * @param fz z-coordinate of measured body specific force
     *           expressed in meters per squared second (m/s^2).
     * @return pitch angle expressed in radians.
     */
    public static double getPitch(
            final double fx, final double fy, final double fz) {
        final double fy2 = fy * fy;
        final double fz2 = fz * fz;

        return Math.atan(fx / Math.sqrt(fy2 + fz2));
    }

    /**
     * Gets yaw angle of body attitude expressed in radians.
     * This method requires previously computed roll and pitch angles
     * along with gyroscope measurements.
     * This method can only be used for high-accuracy gyroscopes,
     * otherwise, yaw angle must be measured using a magnetometer.
     *
     * @param roll         previously computed roll angle expressed in radians.
     * @param pitch        previously computed pitch angle expressed in radians.
     * @param angularRateX x-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateY y-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateZ z-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @return yaw angle expressed in radians.
     */
    public static double getYaw(
            final double roll, final double pitch,
            final double angularRateX, final double angularRateY,
            final double angularRateZ) {

        final double sinRoll = Math.sin(roll);
        final double cosRoll = Math.cos(roll);

        final double sinPitch = Math.sin(pitch);
        final double cosPitch = Math.cos(pitch);

        final double sinYaw = -angularRateY * cosRoll
                + angularRateZ * sinRoll;
        final double cosYaw = angularRateX * cosPitch
                + (angularRateY * sinRoll + angularRateZ * cosRoll)
                * sinPitch;

        return Math.atan2(sinYaw, cosYaw);
    }

    /**
     * Gets yaw angle of body attitude expressed in radians.
     * This method can only be used for high-accuracy gyroscopes,
     * otherwise, yaw angle must be measured using a magnetometer.
     *
     * @param fx           x-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           y-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           z-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX x-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateY y-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateZ z-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @return yaw angle expressed in radians.
     */
    public static double getYaw(
            final double fx, final double fy, final double fz,
            final double angularRateX, final double angularRateY,
            final double angularRateZ) {
        final double roll = getRoll(fy, fz);
        final double pitch = getPitch(fx, fy, fz);
        return getYaw(roll, pitch, angularRateX,
                angularRateY, angularRateZ);
    }

    /**
     * Gets roll angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param kinematics body kinematics containing measured
     *                   body specific force.
     * @return roll angle expressed in radians.
     */
    public static double getRoll(final BodyKinematics kinematics) {
        return getRoll(kinematics.getFy(), kinematics.getFz());
    }

    /**
     * Gets pitch angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param kinematics body kinematics containing measured
     *                   body specific force.
     * @return pitch angle expressed in radians.
     */
    public static double getPitch(final BodyKinematics kinematics) {
        return getPitch(kinematics.getFx(), kinematics.getFy(),
                kinematics.getFz());
    }

    /**
     * Gets yaw angle of body attitude expressed in radians.
     * This method can only be used for high-accuracy gyroscopes.
     * otherwise, yaw angle must be measured using a magnetometer.
     *
     * @param kinematics body kinematics containing measured
     *                   body specific force ang angular rates.
     * @return yaw angle expressed in radians.
     */
    public static double getYaw(final BodyKinematics kinematics) {
        return getYaw(kinematics.getFx(), kinematics.getFy(),
                kinematics.getFz(), kinematics.getAngularRateX(),
                kinematics.getAngularRateY(), kinematics.getAngularRateZ());
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     *
     * @param fx           x-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           y-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           z-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX x-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateY y-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateZ z-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param result       instance where attitude will be stored.
     */
    public static void getAttitude(
            final double fx, final double fy, final double fz,
            final double angularRateX, final double angularRateY,
            final double angularRateZ, final CoordinateTransformation result) {
        result.setSourceType(FrameType.LOCAL_NAVIGATION_FRAME);
        result.setDestinationType(FrameType.BODY_FRAME);

        final double roll = getRoll(fy, fz);
        final double pitch = getPitch(fx, fy, fz);
        final double yaw = getYaw(roll, pitch,
                angularRateX, angularRateY, angularRateZ);
        result.setEulerAngles(roll, pitch, yaw);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     *
     * @param kinematics body kinematics containing measured
     *                   body specific force and angular rate.
     * @param result     instance where attitude will be stored.
     */
    public static void getAttitude(
            final BodyKinematics kinematics,
            final CoordinateTransformation result) {
        getAttitude(kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), result);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     *
     * @param fx           x-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           y-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           z-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX x-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateY y-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateZ z-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @return a coordinate transformation containing body attitude.
     */
    public static CoordinateTransformation getAttitude(
            final double fx, final double fy, final double fz,
            final double angularRateX, final double angularRateY,
            final double angularRateZ) {

        final double roll = getRoll(fy, fz);
        final double pitch = getPitch(fx, fy, fz);
        final double yaw = getYaw(roll, pitch,
                angularRateX, angularRateY, angularRateZ);

        return new CoordinateTransformation(roll, pitch, yaw,
                FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     *
     * @param kinematics body kinematics containing measured body
     *                   specific force and angular rate.
     * @return a coordinate transformation containing body attitude.
     */
    public static CoordinateTransformation getAttitude(
            final BodyKinematics kinematics) {
        return getAttitude(kinematics.getFx(), kinematics.getFy(),
                kinematics.getFz(), kinematics.getAngularRateX(),
                kinematics.getAngularRateY(), kinematics.getAngularRateZ());
    }

    /**
     * Gets roll angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param fy y-coordinate of measured body specific force.
     * @param fz z-coordinate of measured body specific force.
     * @return roll angle expressed in radians.
     */
    public static double getRoll(
            final Acceleration fy, final Acceleration fz) {
        return getRoll(convertAcceleration(fy), convertAcceleration(fz));
    }

    /**
     * Gets pitch angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param fx x-coordinate of measured body specific force.
     * @param fy y-coordinate of measured body specific force.
     * @param fz z-coordinate of measured body specific force.
     * @return pitch angle expressed in radians.
     */
    public static double getPitch(
            final Acceleration fx, final Acceleration fy,
            final Acceleration fz) {
        return getPitch(convertAcceleration(fx), convertAcceleration(fy),
                convertAcceleration(fz));
    }

    /**
     * Gets yaw angle of body attitude expressed in radians.
     * This method requires previously computed roll and pitch angles
     * along with gyroscope measurements.
     * This method can only be used for high-accuracy gyroscopes,
     * otherwise, yaw angle must be measured using a magnetometer.
     *
     * @param roll         previously computed roll angle.
     * @param pitch        previously computed pitch angle.
     * @param angularRateX x-coordinate of body angular rate.
     * @param angularRateY y-coordinate of body angular rate.
     * @param angularRateZ z-coordinate of body angular rate.
     * @return yaw angle expressed in radians.
     */
    public static double getYaw(
            final Angle roll, final Angle pitch, final AngularSpeed angularRateX,
            final AngularSpeed angularRateY, final AngularSpeed angularRateZ) {
        return getYaw(convertAngle(roll), convertAngle(pitch),
                convertAngularSpeed(angularRateX),
                convertAngularSpeed(angularRateY),
                convertAngularSpeed(angularRateZ));
    }

    /**
     * Gets yaw angle of body attitude expressed in radians.
     * This method can only be used for high-accuracy gyroscopes,
     * otherwise, yaw angle must be measured using a magnetometer.
     *
     * @param fx           x-coordinate of measured body specific force.
     * @param fy           y-coordinate of measured body specific force.
     * @param fz           z-coordinate of measured body specific force.
     * @param angularRateX x-coordinate of body angular rate.
     * @param angularRateY y-coordinate of body angular rate.
     * @param angularRateZ z-coordinate of body angular rate.
     * @return yaw angle expressed in radians.
     */
    public static double getYaw(
            final Acceleration fx, final Acceleration fy, final Acceleration fz,
            final AngularSpeed angularRateX, final AngularSpeed angularRateY,
            final AngularSpeed angularRateZ) {
        return getYaw(convertAcceleration(fx), convertAcceleration(fy),
                convertAcceleration(fz), convertAngularSpeed(angularRateX),
                convertAngularSpeed(angularRateY),
                convertAngularSpeed(angularRateZ));
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     *
     * @param fx           x-coordinate of measured body specific force.
     * @param fy           y-coordinate of measured body specific force.
     * @param fz           z-coordinate of measured body specific force.
     * @param angularRateX x-coordinate of body angular rate.
     * @param angularRateY y-coordinate of body angular rate.
     * @param angularRateZ z-coordinate of body angular rate.
     * @param result       instance where attitude will be stored.
     */
    public static void getAttitude(
            final Acceleration fx, final Acceleration fy, final Acceleration fz,
            final AngularSpeed angularRateX, final AngularSpeed angularRateY,
            final AngularSpeed angularRateZ,
            final CoordinateTransformation result) {
        getAttitude(convertAcceleration(fx), convertAcceleration(fy),
                convertAcceleration(fz), convertAngularSpeed(angularRateX),
                convertAngularSpeed(angularRateY),
                convertAngularSpeed(angularRateZ), result);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     *
     * @param fx           x-coordinate of measured body specific force.
     * @param fy           y-coordinate of measured body specific force.
     * @param fz           z-coordinate of measured body specific force.
     * @param angularRateX x-coordinate of body angular rate.
     * @param angularRateY y-coordinate of body angular rate.
     * @param angularRateZ z-coordinate of body angular rate.
     * @return a coordinate transformation containing body attitude.
     */
    public static CoordinateTransformation getAttitude(
            final Acceleration fx, final Acceleration fy, final Acceleration fz,
            final AngularSpeed angularRateX, final AngularSpeed angularRateY,
            final AngularSpeed angularRateZ) {
        return getAttitude(convertAcceleration(fx), convertAcceleration(fy),
                convertAcceleration(fz), convertAngularSpeed(angularRateX),
                convertAngularSpeed(angularRateY),
                convertAngularSpeed(angularRateZ));
    }

    /**
     * Gets roll angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param fy y-coordinate of measured body specific force
     *           expressed in meters per squared second (m/s^2).
     * @param fz z-coordinate of measured body specific force
     *           expressed in meters per squared second (m/s^2).
     * @param result instance where roll angle will be stored.
     */
    public static void getRollAsAngle(
            final double fy, final double fz, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getRoll(fy, fz));
    }

    /**
     * Gets roll angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param fy y-coordinate of measured body specific force
     *           expressed in meters per squared second (m/s^2).
     * @param fz z-coordinate of measured body specific force
     *           expressed in meters per squared second (m/s^2).
     * @return roll angle.
     */
    public static Angle getRollAsAngle(final double fy, final double fz) {
        return new Angle(getRoll(fy, fz), AngleUnit.RADIANS);
    }

    /**
     * Gets pitch angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param fx x-coordinate of measured body specific force
     *           expressed in meters per squared second (m/s^2).
     * @param fy y-coordinate of measured body specific force
     *           expressed in meters per squared second (m/s^2).
     * @param fz z-coordinate of measured body specific force
     *           expressed in meters per squared second (m/s^2).
     * @param result instance where pitch angle will be stored.
     */
    public static void getPitchAsAngle(
            final double fx, final double fy, final double fz,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getPitch(fx, fy, fz));
    }

    /**
     * Gets pitch angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param fx x-coordinate of measured body specific force
     *           expressed in meters per squared second (m/s^2).
     * @param fy y-coordinate of measured body specific force
     *           expressed in meters per squared second (m/s^2).
     * @param fz z-coordinate of measured body specific force
     *           expressed in meters per squared second (m/s^2).
     * @return pitch angle.
     */
    public static Angle getPitchAsAngle(
            final double fx, final double fy, final double fz) {
        return new Angle(getPitch(fx, fy, fz), AngleUnit.RADIANS);
    }

    /**
     * Gets yaw angle of body attitude expressed in radians.
     * This method requires previously computed roll and pitch angles
     * along with gyroscope measurements.
     * This method can only be used for high-accuracy gyroscopes,
     * otherwise, yaw angle must be measured using a magnetometer.
     *
     * @param roll         previously computed roll angle expressed in radians.
     * @param pitch        previously computed pitch angle expressed in radians.
     * @param angularRateX x-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateY y-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateZ z-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param result instance where yaw angle will be stored.
     */
    public static void getYawAsAngle(
            final double roll, final double pitch, final double angularRateX,
            final double angularRateY, final double angularRateZ,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getYaw(roll, pitch, angularRateX, angularRateY,
                angularRateZ));
    }

    /**
     * Gets yaw angle of body attitude expressed in radians.
     * This method requires previously computed roll and pitch angles
     * along with gyroscope measurements.
     * This method can only be used for high-accuracy gyroscopes,
     * otherwise, yaw angle must be measured using a magnetometer.
     *
     * @param roll         previously computed roll angle expressed in radians.
     * @param pitch        previously computed pitch angle expressed in radians.
     * @param angularRateX x-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateY y-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateZ z-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @return yaw angle.
     */
    public static Angle getYawAsAngle(
            final double roll, final double pitch, final double angularRateX,
            final double angularRateY, final double angularRateZ) {
        return new Angle(getYaw(roll, pitch, angularRateX, angularRateY,
                angularRateZ), AngleUnit.RADIANS);
    }

    /**
     * Gets yaw angle of body attitude expressed in radians.
     * This method can only be used for high-accuracy gyroscopes,
     * otherwise, yaw angle must be measured using a magnetometer.
     *
     * @param fx           x-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           y-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           z-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX x-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateY y-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateZ z-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param result instance where yaw angle will be stored.
     */
    public static void getYawAsAngle(
            final double fx, final double fy, final double fz,
            final double angularRateX, final double angularRateY,
            final double angularRateZ, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getYaw(fx, fy, fz, angularRateX, angularRateY,
                angularRateZ));
    }

    /**
     * Gets yaw angle of body attitude expressed in radians.
     * This method can only be used for high-accuracy gyroscopes,
     * otherwise, yaw angle must be measured using a magnetometer.
     *
     * @param fx           x-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           y-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           z-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX x-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateY y-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateZ z-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @return yaw angle.
     */
    public static Angle getYawAsAngle(
            final double fx, final double fy, final double fz,
            final double angularRateX, final double angularRateY,
            final double angularRateZ) {
        return new Angle(getYaw(fx, fy, fz, angularRateX, angularRateY,
                angularRateZ), AngleUnit.RADIANS);
    }

    /**
     * Gets roll angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param kinematics body kinematics containing measured
     *                   body specific force.
     * @param result instance where roll angle will be stored.
     */
    public static void getRollAsAngle(
            final BodyKinematics kinematics, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getRoll(kinematics));
    }

    /**
     * Gets roll angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param kinematics body kinematics containing measured
     *                   body specific force.
     * @return roll angle.
     */
    public static Angle getRollAsAngle(final BodyKinematics kinematics) {
        return new Angle(getRoll(kinematics), AngleUnit.RADIANS);
    }

    /**
     * Gets pitch angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param kinematics body kinematics containing measured
     *                   body specific force.
     * @param result instance where pitch angle will be stored.
     */
    public static void getPitchAsAngle(
            final BodyKinematics kinematics, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getPitch(kinematics));
    }

    /**
     * Gets pitch angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param kinematics body kinematics containing measured
     *                   body specific force.
     * @return pitch angle.
     */
    public static Angle getPitchAsAngle(final BodyKinematics kinematics) {
        return new Angle(getPitch(kinematics), AngleUnit.RADIANS);
    }

    /**
     * Gets yaw angle of body attitude expressed in radians.
     * This method can only be used for high-accuracy gyroscopes.
     * otherwise, yaw angle must be measured using a magnetometer.
     *
     * @param kinematics body kinematics containing measured
     *                   body specific force ang angular rates.
     * @param result instance where yaw angle will be stored.
     */
    public static void getYawAsAngle(
            final BodyKinematics kinematics, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getYaw(kinematics));
    }

    /**
     * Gets yaw angle of body attitude expressed in radians.
     * This method can only be used for high-accuracy gyroscopes.
     * otherwise, yaw angle must be measured using a magnetometer.
     *
     * @param kinematics body kinematics containing measured
     *                   body specific force ang angular rates.
     * @return yaw angle.
     */
    public static Angle getYawAsAngle(final BodyKinematics kinematics) {
        return new Angle(getYaw(kinematics), AngleUnit.RADIANS);
    }

    /**
     * Gets roll angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param fy y-coordinate of measured body specific force.
     * @param fz z-coordinate of measured body specific force.
     * @param result instance where roll angle will be stored.
     */
    public static void getRollAsAngle(
            final Acceleration fy, final Acceleration fz, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getRoll(fy, fz));
    }

    /**
     * Gets roll angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param fy y-coordinate of measured body specific force.
     * @param fz z-coordinate of measured body specific force.
     * @return roll angle.
     */
    public static Angle getRollAsAngle(
            final Acceleration fy, final Acceleration fz) {
        return new Angle(getRoll(fy, fz), AngleUnit.RADIANS);
    }

    /**
     * Gets pitch angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param fx x-coordinate of measured body specific force.
     * @param fy y-coordinate of measured body specific force.
     * @param fz z-coordinate of measured body specific force.
     * @param result instance where pitch angle will be stored.
     */
    public static void getPitchAsAngle(
            final Acceleration fx, final Acceleration fy, final Acceleration fz,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getPitch(fx, fy, fz));
    }

    /**
     * Gets pitch angle of body attitude expressed in radians.
     * This is based on expression (5.101) of Paul D. Groves.
     * Principles of GNSS Inertial and multi-sensor integrated
     * navigation systemd. 2nd ed.
     *
     * @param fx x-coordinate of measured body specific force.
     * @param fy y-coordinate of measured body specific force.
     * @param fz z-coordinate of measured body specific force.
     * @return pitch angle.
     */
    public static Angle getPitchAsAngle(
            final Acceleration fx, final Acceleration fy, final Acceleration fz) {
        return new Angle(getPitch(fx, fy, fz), AngleUnit.RADIANS);
    }

    /**
     * Gets yaw angle of body attitude expressed in radians.
     * This method requires previously computed roll and pitch angles
     * along with gyroscope measurements.
     * This method can only be used for high-accuracy gyroscopes,
     * otherwise, yaw angle must be measured using a magnetometer.
     *
     * @param roll         previously computed roll angle.
     * @param pitch        previously computed pitch angle.
     * @param angularRateX x-coordinate of body angular rate.
     * @param angularRateY y-coordinate of body angular rate.
     * @param angularRateZ z-coordinate of body angular rate.
     * @param result instance where yaw angle will be stored.
     */
    public static void getYawAsAngle(
            final Angle roll, final Angle pitch, final AngularSpeed angularRateX,
            final AngularSpeed angularRateY, final AngularSpeed angularRateZ,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getYaw(roll, pitch, angularRateX, angularRateY,
                angularRateZ));
    }

    /**
     * Gets yaw angle of body attitude expressed in radians.
     * This method requires previously computed roll and pitch angles
     * along with gyroscope measurements.
     * This method can only be used for high-accuracy gyroscopes,
     * otherwise, yaw angle must be measured using a magnetometer.
     *
     * @param roll         previously computed roll angle.
     * @param pitch        previously computed pitch angle.
     * @param angularRateX x-coordinate of body angular rate.
     * @param angularRateY y-coordinate of body angular rate.
     * @param angularRateZ z-coordinate of body angular rate.
     * @return yaw angle.
     */
    public static Angle getYawAsAngle(
            final Angle roll, final Angle pitch, final AngularSpeed angularRateX,
            final AngularSpeed angularRateY, final AngularSpeed angularRateZ) {
        return new Angle(getYaw(roll, pitch, angularRateX, angularRateY,
                angularRateZ), AngleUnit.RADIANS);
    }

    /**
     * Gets yaw angle of body attitude expressed in radians.
     * This method can only be used for high-accuracy gyroscopes,
     * otherwise, yaw angle must be measured using a magnetometer.
     *
     * @param fx           x-coordinate of measured body specific force.
     * @param fy           y-coordinate of measured body specific force.
     * @param fz           z-coordinate of measured body specific force.
     * @param angularRateX x-coordinate of body angular rate.
     * @param angularRateY y-coordinate of body angular rate.
     * @param angularRateZ z-coordinate of body angular rate.
     * @param result instance where yaw angle will be stored.
     */
    public static void getYawAsAngle(
            final Acceleration fx, final Acceleration fy, final Acceleration fz,
            final AngularSpeed angularRateX, final AngularSpeed angularRateY,
            final AngularSpeed angularRateZ, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getYaw(fx, fy, fz, angularRateX, angularRateY,
                angularRateZ));
    }

    /**
     * Gets yaw angle of body attitude expressed in radians.
     * This method can only be used for high-accuracy gyroscopes,
     * otherwise, yaw angle must be measured using a magnetometer.
     *
     * @param fx           x-coordinate of measured body specific force.
     * @param fy           y-coordinate of measured body specific force.
     * @param fz           z-coordinate of measured body specific force.
     * @param angularRateX x-coordinate of body angular rate.
     * @param angularRateY y-coordinate of body angular rate.
     * @param angularRateZ z-coordinate of body angular rate.
     * @return yaw angle.
     */
    public static Angle getYawAsAngle(
            final Acceleration fx, final Acceleration fy, final Acceleration fz,
            final AngularSpeed angularRateX, final AngularSpeed angularRateY,
            final AngularSpeed angularRateZ) {
        return new Angle(getYaw(fx, fy, fz, angularRateX, angularRateY,
                angularRateZ), AngleUnit.RADIANS);
    }

    /**
     * Converts an instance of acceleration to meters per squared second (m/s^2).
     *
     * @param acceleration acceleration instance to be converted.
     * @return converted acceleration value.
     */
    private static double convertAcceleration(final Acceleration acceleration) {
        return AccelerationConverter.convert(
                acceleration.getValue().doubleValue(),
                acceleration.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Converts an instance of angular speed to radians per second (rad/s).
     *
     * @param angularSpeed angular speed instance to be converted.
     * @return converted angular speed value.
     */
    private static double convertAngularSpeed(final AngularSpeed angularSpeed) {
        return AngularSpeedConverter.convert(
                angularSpeed.getValue().doubleValue(),
                angularSpeed.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Converts an instance of an angle to radians (rad).
     *
     * @param angle angle to be converted.
     * @return converted angle value.
     */
    private static double convertAngle(final Angle angle) {
        return AngleConverter.convert(angle.getValue().doubleValue(),
                angle.getUnit(), AngleUnit.RADIANS);
    }
}
