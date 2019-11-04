/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.frames;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleConverter;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceConverter;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedConverter;
import com.irurueta.units.SpeedUnit;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains position, velocity and coordinates transformation matrix expressed in NED frame.
 * Position is expressed as latitude, longitude and height.
 * Velocity of body frame is expressed with respect ECEF frame and resolved along north, east and down axes,
 * as defined in {@link FrameType#LOCAL_NAVIGATION_FRAME}.
 */
@SuppressWarnings("WeakerAccess")
public class NEDFrame implements Frame, Serializable, Cloneable {

    /**
     * Number of coordinates representing velocity.
     */
    public static final int NUM_VELOCITY_COORDINATES = 3;

    /**
     * Latitude expressed in radians.
     */
    private double mLatitude;

    /**
     * Longitude expressed in radians.
     */
    private double mLongitude;

    /**
     * Height expressed in meters.
     */
    private double mHeight;

    /**
     * Coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along North axis.
     */
    private double mVn;

    /**
     * Coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along East axis.
     */
    private double mVe;

    /**
     * Coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along Down axis.
     */
    private double mVd;

    /**
     * Body to NED coordinate transformation matrix.
     */
    private CoordinateTransformation mC;

    /**
     * Constructor.
     * Initializes position and velocity coordinates to zero and the coordinate transformation matrix to the
     * identity.
     */
    public NEDFrame() {
        mC = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
    }

    /**
     * Constructor.
     *
     * @param c Body to NED (Local Navigation frame) coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public NEDFrame(final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     */
    public NEDFrame(final double latitude, final double longitude, final double height) {
        this();
        setPosition(latitude, longitude, height);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians to be set.
     * @param longitude longitude expressed in radians to be set.
     * @param height    height to be set.
     */
    public NEDFrame(final double latitude, final double longitude, final Distance height) {
        this();
        setPosition(latitude, longitude, height);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude to be set.
     * @param longitude longitude to be set.
     * @param height    height expressed in meters to be set.
     */
    public NEDFrame(final Angle latitude, final Angle longitude, final double height) {
        this();
        setPosition(latitude, longitude, height);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude to be set.
     * @param longitude longitude to be set.
     * @param height    height to be set.
     */
    public NEDFrame(final Angle latitude, final Angle longitude, final Distance height) {
        this();
        setPosition(latitude, longitude, height);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param vn        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along North axis.
     * @param ve        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along East axis.
     * @param vd        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along Down axis.
     */
    public NEDFrame(final double latitude, final double longitude, final double height,
                    final double vn, final double ve, final double vd) {
        this(latitude, longitude, height);
        setVelocityCoordinates(vn, ve, vd);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians to be set.
     * @param longitude longitude expressed in radians to be set.
     * @param height    height to be set.
     * @param vn        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along North axis.
     * @param ve        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along East axis.
     * @param vd        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along Down axis.
     */
    public NEDFrame(final double latitude, final double longitude, final Distance height,
                    final double vn, final double ve, final double vd) {
        this(latitude, longitude, height);
        setVelocityCoordinates(vn, ve, vd);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude to be set.
     * @param longitude longitude to be set.
     * @param height    height expressed in meters to be set.
     * @param vn        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along North axis.
     * @param ve        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along East axis.
     * @param vd        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along Down axis.
     */
    public NEDFrame(final Angle latitude, final Angle longitude, final double height,
                    final double vn, final double ve, final double vd) {
        this(latitude, longitude, height);
        setVelocityCoordinates(vn, ve, vd);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude to be set.
     * @param longitude longitude to be set.
     * @param height    height to be set.
     * @param vn        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along North axis.
     * @param ve        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along East axis.
     * @param vd        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along Down axis.
     */
    public NEDFrame(final Angle latitude, final Angle longitude, final Distance height,
                    final double vn, final double ve, final double vd) {
        this(latitude, longitude, height);
        setVelocityCoordinates(vn, ve, vd);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param speedN    coordinate of velocity of body frame with respect ECEF frame and resolved along North axis.
     * @param speedE    coordinate of velocity of body frame with respect ECEF frame and resolved along East axis.
     * @param speedD    coordinate of velocity of body frame with respect ECEF frame and resolved along Down axis.
     */
    public NEDFrame(final double latitude, final double longitude, final double height,
                    final Speed speedN, final Speed speedE, final Speed speedD) {
        this(latitude, longitude, height);
        setSpeedCoordinates(speedN, speedE, speedD);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height to be set.
     * @param speedN    coordinate of velocity of body frame with respect ECEF frame and resolved along North axis.
     * @param speedE    coordinate of velocity of body frame with respect ECEF frame and resolved along East axis.
     * @param speedD    coordinate of velocity of body frame with respect ECEF frame and resolved along Down axis.
     */
    public NEDFrame(final double latitude, final double longitude, final Distance height,
                    final Speed speedN, final Speed speedE, final Speed speedD) {
        this(latitude, longitude, height);
        setSpeedCoordinates(speedN, speedE, speedD);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude to be set.
     * @param longitude longitude to be set.
     * @param height    height expressed in meters to be set
     * @param speedN    coordinate of velocity of body frame with respect ECEF frame and resolved along North axis.
     * @param speedE    coordinate of velocity of body frame with respect ECEF frame and resolved along East axis.
     * @param speedD    coordinate of velocity of body frame with respect ECEF frame and resolved along Down axis.
     */
    public NEDFrame(final Angle latitude, final Angle longitude, final double height,
                    final Speed speedN, final Speed speedE, final Speed speedD) {
        this(latitude, longitude, height);
        setSpeedCoordinates(speedN, speedE, speedD);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude to be set.
     * @param longitude longitude to be set.
     * @param height    height to be set.
     * @param speedN    coordinate of velocity of body frame with respect ECEF frame and resolved along North axis.
     * @param speedE    coordinate of velocity of body frame with respect ECEF frame and resolved along East axis.
     * @param speedD    coordinate of velocity of body frame with respect ECEF frame and resolved along Down axis.
     */
    public NEDFrame(final Angle latitude, final Angle longitude, final Distance height,
                    final Speed speedN, final Speed speedE, final Speed speedD) {
        this(latitude, longitude, height);
        setSpeedCoordinates(speedN, speedE, speedD);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param c         Body to NED (Local Navigation frame) coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public NEDFrame(final double latitude, final double longitude, final double height,
                    final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        this(latitude, longitude, height);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude to be set.
     * @param longitude longitude to be set.
     * @param height    height expressed in meters to be set.
     * @param c         Body to NED (Local Navigation frame) coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public NEDFrame(final Angle latitude, final Angle longitude, final double height,
                    final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        this(latitude, longitude, height);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude to be set.
     * @param longitude longitude to be set.
     * @param height    height to be set.
     * @param c         Body to NED (Local Navigation frame) coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public NEDFrame(final Angle latitude, final Angle longitude, final Distance height,
                    final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        this(latitude, longitude, height);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param vn        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along North axis.
     * @param ve        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along East axis.
     * @param vd        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along Down axis.
     * @param c         Body to NED (Local Navigation frame) coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public NEDFrame(final double latitude, final double longitude, final double height,
                    final double vn, final double ve, final double vd,
                    final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        this(latitude, longitude, height, vn, ve, vd);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians to be set.
     * @param longitude longitude expressed in radians to be set.
     * @param height    height to be set.
     * @param vn        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along North axis.
     * @param ve        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along East axis.
     * @param vd        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along Down axis.
     * @param c         Body to NED (Local Navigation frame) coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public NEDFrame(final double latitude, final double longitude, final Distance height,
                    final double vn, final double ve, final double vd,
                    final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        this(latitude, longitude, height, vn, ve, vd);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude to be set.
     * @param longitude longitude to be set.
     * @param height    height expressed in meters to be set.
     * @param vn        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along North axis.
     * @param ve        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along East axis.
     * @param vd        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along Down axis.
     * @param c         Body to NED (Local Navigation frame) coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public NEDFrame(final Angle latitude, final Angle longitude, final double height,
                    final double vn, final double ve, final double vd,
                    final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        this(latitude, longitude, height, vn, ve, vd);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude to be set.
     * @param longitude longitude to be set.
     * @param height    height to be set.
     * @param vn        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along North axis.
     * @param ve        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along East axis.
     * @param vd        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame
     *                  and resolved along Down axis.
     * @param c         Body to NED (Local Navigation frame) coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public NEDFrame(final Angle latitude, final Angle longitude, final Distance height,
                    final double vn, final double ve, final double vd,
                    final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        this(latitude, longitude, height, vn, ve, vd);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param speedN    coordinate of velocity of body frame with respect ECEF frame and resolved along North axis.
     * @param speedE    coordinate of velocity of body frame with respect ECEF frame and resolved along East axis.
     * @param speedD    coordinate of velocity of body frame with respect ECEF frame and resolved along Down axis.
     * @param c         Body to NED (Local Navigation frame) coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public NEDFrame(final double latitude, final double longitude, final double height,
                    final Speed speedN, final Speed speedE, final Speed speedD,
                    final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        this(latitude, longitude, height, speedN, speedE, speedD);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height to be set.
     * @param speedN    coordinate of velocity of body frame with respect ECEF frame and resolved along North axis.
     * @param speedE    coordinate of velocity of body frame with respect ECEF frame and resolved along East axis.
     * @param speedD    coordinate of velocity of body frame with respect ECEF frame and resolved along Down axis.
     * @param c         Body to NED (Local Navigation frame) coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public NEDFrame(final double latitude, final double longitude, final Distance height,
                    final Speed speedN, final Speed speedE, final Speed speedD,
                    final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        this(latitude, longitude, height, speedN, speedE, speedD);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude to be set.
     * @param longitude longitude to be set.
     * @param height    height expressed in meters to be set
     * @param speedN    coordinate of velocity of body frame with respect ECEF frame and resolved along North axis.
     * @param speedE    coordinate of velocity of body frame with respect ECEF frame and resolved along East axis.
     * @param speedD    coordinate of velocity of body frame with respect ECEF frame and resolved along Down axis.
     * @param c         Body to NED (Local Navigation frame) coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public NEDFrame(final Angle latitude, final Angle longitude, final double height,
                    final Speed speedN, final Speed speedE, final Speed speedD,
                    final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        this(latitude, longitude, height, speedN, speedE, speedD);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude to be set.
     * @param longitude longitude to be set.
     * @param height    height to be set.
     * @param speedN    coordinate of velocity of body frame with respect ECEF frame and resolved along North axis.
     * @param speedE    coordinate of velocity of body frame with respect ECEF frame and resolved along East axis.
     * @param speedD    coordinate of velocity of body frame with respect ECEF frame and resolved along Down axis.
     * @param c         Body to NED (Local Navigation frame) coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public NEDFrame(final Angle latitude, final Angle longitude, final Distance height,
                    final Speed speedN, final Speed speedE, final Speed speedD,
                    final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        this(latitude, longitude, height, speedN, speedE, speedD);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param input NED frame to copy data from.
     */
    public NEDFrame(final NEDFrame input) {
        this();
        copyFrom(input);
    }

    /**
     * Gets latitude expressed in radians.
     *
     * @return latitude expressed in radians.
     */
    public double getLatitude() {
        return mLatitude;
    }

    /**
     * Sets latitude expressed in radians.
     *
     * @param latitude latitude expressed in radians to be set.
     */
    public void setLatitude(final double latitude) {
        mLatitude = latitude;
    }

    /**
     * Gets longitude expressed in radians.
     *
     * @return longitude expressed in radians.
     */
    public double getLongitude() {
        return mLongitude;
    }

    /**
     * Sets longitude expressed in radians.
     *
     * @param longitude longitude expressed in radians to be set.
     */
    public void setLongitude(final double longitude) {
        mLongitude = longitude;
    }

    /**
     * Gets height expressed in meters.
     *
     * @return height expressed in meters.
     */
    public double getHeight() {
        return mHeight;
    }

    /**
     * Sets height expressed in meters.
     *
     * @param height height expressed in meters to be set.
     */
    public void setHeight(double height) {
        mHeight = height;
    }

    /**
     * Sets body position.
     *
     * @param latitude  latitude expressed in radians to be set.
     * @param longitude longitude expressed in radians to be set.
     * @param height    height expressed in meters to be set.
     */
    public void setPosition(final double latitude, final double longitude, final double height) {
        mLatitude = latitude;
        mLongitude = longitude;
        mHeight = height;
    }

    /**
     * Gets latitude.
     *
     * @param result instance where latitude will be stored.
     */
    public void getLatitudeAngle(final Angle result) {
        result.setValue(mLatitude);
        result.setUnit(AngleUnit.RADIANS);
    }

    /**
     * Gets latitude.
     *
     * @return latitude.
     */
    public Angle getLatitudeAngle() {
        return new Angle(mLatitude, AngleUnit.RADIANS);
    }

    /**
     * Sets latitude.
     *
     * @param latitudeAngle latitude to be set.
     */
    public void setLatitudeAngle(final Angle latitudeAngle) {
        mLatitude = AngleConverter.convert(latitudeAngle.getValue().doubleValue(),
                latitudeAngle.getUnit(), AngleUnit.RADIANS);
    }

    /**
     * Gets longitude.
     *
     * @param result instance where longitude will be stored.
     */
    public void getLongitudeAngle(final Angle result) {
        result.setValue(mLongitude);
        result.setUnit(AngleUnit.RADIANS);
    }

    /**
     * Gets longitude.
     *
     * @return longitude.
     */
    public Angle getLongitudeAngle() {
        return new Angle(mLongitude, AngleUnit.RADIANS);
    }

    /**
     * Sets longitude.
     *
     * @param longitudeAngle longitude to be set.
     */
    public void setLongitudeAngle(final Angle longitudeAngle) {
        mLongitude = AngleConverter.convert(longitudeAngle.getValue().doubleValue(),
                longitudeAngle.getUnit(), AngleUnit.RADIANS);
    }

    /**
     * Gets height.
     *
     * @param result instance where height will be stored.
     */
    public void getHeightDistance(final Distance result) {
        result.setValue(mHeight);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets height.
     *
     * @return height.
     */
    public Distance getHeightDistance() {
        return new Distance(mHeight, DistanceUnit.METER);
    }

    /**
     * Sets height.
     *
     * @param heightDistance height to be set.
     */
    public void setHeightDistance(final Distance heightDistance) {
        mHeight = DistanceConverter.convert(heightDistance.getValue().doubleValue(),
                heightDistance.getUnit(), DistanceUnit.METER);
    }

    /**
     * Sets body position.
     *
     * @param latitude  latitude expressed in radians to be set.
     * @param longitude longitude expressed in radians to be set.
     * @param height    height to be set.
     */
    public void setPosition(final double latitude, final double longitude, final Distance height) {
        mLatitude = latitude;
        mLongitude = longitude;
        setHeightDistance(height);
    }

    /**
     * Sets body position.
     *
     * @param latitude  latitude to be set.
     * @param longitude longitude to be set.
     * @param height    height expressed in meters to be set.
     */
    public void setPosition(final Angle latitude, final Angle longitude, final double height) {
        setLatitudeAngle(latitude);
        setLongitudeAngle(longitude);
        mHeight = height;
    }

    /**
     * Sets body position.
     *
     * @param latitude  latitude to be set.
     * @param longitude longitude to be set.
     * @param height    height to be set.
     */
    public void setPosition(final Angle latitude, final Angle longitude, final Distance height) {
        setLatitudeAngle(latitude);
        setLongitudeAngle(longitude);
        setHeightDistance(height);
    }

    /**
     * Gets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along North axis.
     *
     * @return North velocity coordinate value.
     */
    public double getVn() {
        return mVn;
    }

    /**
     * Sets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along North axis.
     *
     * @param vn North velocity coordinate value.
     */
    public void setVn(final double vn) {
        mVn = vn;
    }

    /**
     * Gets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along East axis.
     *
     * @return East velocity coordinate value.
     */
    public double getVe() {
        return mVe;
    }

    /**
     * Sets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along East axis.
     *
     * @param ve East velocity coordinate value.
     */
    public void setVe(final double ve) {
        mVe = ve;
    }

    /**
     * Gets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along Down axis.
     *
     * @return Down velocity coordinate value.
     */
    public double getVd() {
        return mVd;
    }

    /**
     * Sets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along Down axis.
     *
     * @param vd Down velocity coordinate value.
     */
    public void setVd(final double vd) {
        mVd = vd;
    }

    /**
     * Sets velocity coordinates of body frame expressed in meters per second (m/s) resolved along North, East, Down
     * axes.
     *
     * @param vn North velocity coordinate value.
     * @param ve East velocity coordinate value.
     * @param vd Down velocity coordinate value.
     */
    public void setVelocityCoordinates(final double vn, final double ve, final double vd) {
        mVn = vn;
        mVe = ve;
        mVd = vd;
    }

    /**
     * Gets norm of velocity expressed in meters per second (m/s), which represents
     * the speed of the body.
     *
     * @return norm of velocity expressed in meters per second (m/s).
     */
    public double getVelocityNorm() {
        return Math.sqrt(mVn * mVn + mVe * mVe + mVd * mVd);
    }

    /**
     * Gets norm of velocity, which represents the speed of the body.
     *
     * @param result velocity norm.
     */
    public void getVelocityNormAsSpeed(final Speed result) {
        result.setValue(getVelocityNorm());
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets norm of velocity, which represents the speed of the body.
     *
     * @return velocity norm.
     */
    public Speed getVelocityNormAsSpeed() {
        return new Speed(getVelocityNorm(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along North axis.
     *
     * @param result instance where North velocity coordinate will be stored.
     */
    public void getSpeedN(final Speed result) {
        result.setValue(mVn);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along North axis.
     *
     * @return North velocity coordinate.
     */
    public Speed getSpeedN() {
        return new Speed(mVn, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along North axis.
     *
     * @param speedN North velocity coordinate to be set.
     */
    public void setSpeedN(final Speed speedN) {
        mVn = SpeedConverter.convert(speedN.getValue().doubleValue(),
                speedN.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along East axis.
     *
     * @param result instance where East velocity coordinate will be stored.
     */
    public void getSpeedE(final Speed result) {
        result.setValue(mVe);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along East axis.
     *
     * @return East velocity coordinate.
     */
    public Speed getSpeedE() {
        return new Speed(mVe, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along East axis.
     *
     * @param speedE East velocity coordinate to be set.
     */
    public void setSpeedE(final Speed speedE) {
        mVe = SpeedConverter.convert(speedE.getValue().doubleValue(),
                speedE.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along Down axis.
     *
     * @param result instance where Down velocity coordinate will be stored.
     */
    public void getSpeedD(final Speed result) {
        result.setValue(mVd);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along Down axis.
     *
     * @return Down velocity coordinate.
     */
    public Speed getSpeedD() {
        return new Speed(mVd, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along Down axis.
     *
     * @param speedD Down velocity coordinate to be set.
     */
    public void setSpeedD(final Speed speedD) {
        mVd = SpeedConverter.convert(speedD.getValue().doubleValue(),
                speedD.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets velocity coordinates of body frame resolved along North, East, Down
     * axes.
     *
     * @param speedN North velocity coordinate.
     * @param speedE East velocity coordinate.
     * @param speedD Down velocity coordinate.
     */
    public void setSpeedCoordinates(final Speed speedN, final Speed speedE,
                                    final Speed speedD) {
        setSpeedN(speedN);
        setSpeedE(speedE);
        setSpeedD(speedD);
    }

    /**
     * Gets coordinate transformation matrix.
     *
     * @return coordinate transformation matrix.
     */
    @Override
    public CoordinateTransformation getCoordinateTransformation() {
        CoordinateTransformation result = new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);
        getCoordinateTransformation(result);
        return result;
    }

    /**
     * Gets coordinate transformation matrix.
     *
     * @param result instance where coordinate transformation matrix will be copied to.
     */
    @Override
    public void getCoordinateTransformation(final CoordinateTransformation result) {
        mC.copyTo(result);
    }

    /**
     * Gets coordinate transformation matrix.
     * This is equivalent to calling getCoordinateTransformation().getMatrix(), but more efficient
     *
     * @return coordinate transformation matrix.
     */
    @Override
    public Matrix getCoordinateTransformationMatrix() {
        Matrix result;
        try {
            result = new Matrix(CoordinateTransformation.ROWS, CoordinateTransformation.COLS);
            getCoordinateTransformationMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets coordinate transformation matrix.
     * This is equivalent to calling getCoordinateTransformation().getMatrix(), but more efficient
     *
     * @param result instance where coordinate transformation matrix will be copied to.
     */
    @Override
    public void getCoordinateTransformationMatrix(final Matrix result) {
        mC.mMatrix.copyTo(result);
    }

    /**
     * Sets coordinate transformation matrix.
     * Provided value must be a body to NED transformation matrix.
     *
     * @param c coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    @Override
    public void setCoordinateTransformation(final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        if (!isValidCoordinateTransformation(c)) {
            throw new InvalidSourceAndDestinationFrameTypeException();
        }

        mC = c;
    }

    /**
     * Checks whether provided coordinate transformation matrix is valid or not.
     * Only body to NED transformation matrices are considered to be valid.
     *
     * @param c coordinate transformation matrix to be checked.
     * @return true if provided value is valid, false otherwise.
     */
    public static boolean isValidCoordinateTransformation(final CoordinateTransformation c) {
        return c.getSourceType() == FrameType.BODY_FRAME &&
                c.getDestinationType() == FrameType.LOCAL_NAVIGATION_FRAME;
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final NEDFrame output) {
        output.mLatitude = mLatitude;
        output.mLongitude = mLongitude;
        output.mHeight = mHeight;

        output.mVn = mVn;
        output.mVe = mVe;
        output.mVd = mVd;

        mC.copyTo(output.mC);
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final NEDFrame input) {
        mLatitude = input.mLatitude;
        mLongitude = input.mLongitude;
        mHeight = input.mHeight;

        mVn = input.mVn;
        mVe = input.mVe;
        mVd = input.mVd;

        mC.copyFrom(input.mC);
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mLatitude, mLongitude, mHeight, mVn, mVe, mVd, mC);
    }

    /**
     * Checks if provided object is an ECEFFrame having exactly the same contents as
     * this instance.
     *
     * @param obj Object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (obj == null) {
            return false;
        }
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof NEDFrame)) {
            return false;
        }

        final NEDFrame other = (NEDFrame) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final NEDFrame other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum difference allowed between position, velocity and coordinate transformation matrix.
     * @return true if both instances are considered to be equal (up to provided threshold), false otherwise.
     */
    public boolean equals(final NEDFrame other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mLatitude - other.mLatitude) <= threshold
                && Math.abs(mLongitude - other.mLongitude) <= threshold
                && Math.abs(mHeight - other.mHeight) <= threshold
                && Math.abs(mVn - other.mVn) <= threshold
                && Math.abs(mVe - other.mVe) <= threshold
                && Math.abs(mVd - other.mVd) <= threshold
                && mC.equals(other.mC, threshold);
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     */
    @SuppressWarnings({"MethodDoesntCallSuperMethod", "CloneDoesntDeclareCloneNotSupportedException"})
    @Override
    protected Object clone() {
        return new NEDFrame(this);
    }
}
