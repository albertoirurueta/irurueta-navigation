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
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Rotation3D;
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
public class NEDFrame implements Frame, Serializable, Cloneable {

    /**
     * Number of coordinates representing velocity.
     */
    public static final int NUM_VELOCITY_COORDINATES = 3;

    /**
     * Latitude expressed in radians.
     */
    private double latitude;

    /**
     * Longitude expressed in radians.
     */
    private double longitude;

    /**
     * Height expressed in meters.
     */
    private double height;

    /**
     * Coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along North axis.
     */
    private double vn;

    /**
     * Coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along East axis.
     */
    private double ve;

    /**
     * Coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along Down axis.
     */
    private double vd;

    /**
     * Body to NED coordinate transformation matrix.
     */
    private CoordinateTransformation c;

    /**
     * Constructor.
     * Initializes position and velocity coordinates to zero and the coordinate transformation matrix to the
     * identity.
     */
    public NEDFrame() {
        c = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
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
     * @param position curvilinear position containing latitude, longitude and height.
     */
    public NEDFrame(final NEDPosition position) {
        this();
        setPosition(position);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param vn        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along North axis.
     * @param ve        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along East axis.
     * @param vd        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along Down axis.
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
     * @param vn        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along North axis.
     * @param ve        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along East axis.
     * @param vd        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along Down axis.
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
     * @param vn        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along North axis.
     * @param ve        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along East axis.
     * @param vd        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along Down axis.
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
     * @param position curvilinear position to be set containing latitude, longitude and height.
     * @param velocity velocity of body frame resolved along North, East, Down axes.
     */
    public NEDFrame(final NEDPosition position, final NEDVelocity velocity) {
        this(position);
        setVelocity(velocity);
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
     * @param position curvilinear position containing latitude, longitude and height.
     * @param c        Body to NED (Local Navigation frame) coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public NEDFrame(final NEDPosition position, final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param vn        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along North axis.
     * @param ve        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along East axis.
     * @param vd        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along Down axis.
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
     * @param vn        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along North axis.
     * @param ve        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along East axis.
     * @param vd        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along Down axis.
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
     * @param vn        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along North axis.
     * @param ve        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along East axis.
     * @param vd        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along Down axis.
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
     * @param vn        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along North axis.
     * @param ve        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along East axis.
     * @param vd        coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF
     *                  frame and resolved along Down axis.
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
     * @param position curvilinear position to be set containing latitude, longitude and height.
     * @param velocity velocity of body frame resolved along North, East, Down axes.
     * @param c        Body to NED (Local Navigation frame) coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public NEDFrame(final NEDPosition position, final NEDVelocity velocity,
                    final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        this(position, velocity);
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
        return latitude;
    }

    /**
     * Sets latitude expressed in radians.
     *
     * @param latitude latitude expressed in radians to be set.
     */
    public void setLatitude(final double latitude) {
        this.latitude = latitude;
    }

    /**
     * Gets longitude expressed in radians.
     *
     * @return longitude expressed in radians.
     */
    public double getLongitude() {
        return longitude;
    }

    /**
     * Sets longitude expressed in radians.
     *
     * @param longitude longitude expressed in radians to be set.
     */
    public void setLongitude(final double longitude) {
        this.longitude = longitude;
    }

    /**
     * Gets height expressed in meters.
     *
     * @return height expressed in meters.
     */
    public double getHeight() {
        return height;
    }

    /**
     * Sets height expressed in meters.
     *
     * @param height height expressed in meters to be set.
     */
    public void setHeight(final double height) {
        this.height = height;
    }

    /**
     * Sets body position.
     *
     * @param latitude  latitude expressed in radians to be set.
     * @param longitude longitude expressed in radians to be set.
     * @param height    height expressed in meters to be set.
     */
    public void setPosition(final double latitude, final double longitude, final double height) {
        this.latitude = latitude;
        this.longitude = longitude;
        this.height = height;
    }

    /**
     * Gets latitude.
     *
     * @param result instance where latitude will be stored.
     */
    public void getLatitudeAngle(final Angle result) {
        result.setValue(latitude);
        result.setUnit(AngleUnit.RADIANS);
    }

    /**
     * Gets latitude.
     *
     * @return latitude.
     */
    public Angle getLatitudeAngle() {
        return new Angle(latitude, AngleUnit.RADIANS);
    }

    /**
     * Sets latitude.
     *
     * @param latitudeAngle latitude to be set.
     */
    public void setLatitudeAngle(final Angle latitudeAngle) {
        latitude = AngleConverter.convert(latitudeAngle.getValue().doubleValue(), latitudeAngle.getUnit(),
                AngleUnit.RADIANS);
    }

    /**
     * Gets longitude.
     *
     * @param result instance where longitude will be stored.
     */
    public void getLongitudeAngle(final Angle result) {
        result.setValue(longitude);
        result.setUnit(AngleUnit.RADIANS);
    }

    /**
     * Gets longitude.
     *
     * @return longitude.
     */
    public Angle getLongitudeAngle() {
        return new Angle(longitude, AngleUnit.RADIANS);
    }

    /**
     * Sets longitude.
     *
     * @param longitudeAngle longitude to be set.
     */
    public void setLongitudeAngle(final Angle longitudeAngle) {
        longitude = AngleConverter.convert(longitudeAngle.getValue().doubleValue(), longitudeAngle.getUnit(),
                AngleUnit.RADIANS);
    }

    /**
     * Gets height.
     *
     * @param result instance where height will be stored.
     */
    public void getHeightDistance(final Distance result) {
        result.setValue(height);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets height.
     *
     * @return height.
     */
    public Distance getHeightDistance() {
        return new Distance(height, DistanceUnit.METER);
    }

    /**
     * Sets height.
     *
     * @param heightDistance height to be set.
     */
    public void setHeightDistance(final Distance heightDistance) {
        height = DistanceConverter.convert(heightDistance.getValue().doubleValue(), heightDistance.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Sets body position.
     *
     * @param latitude  latitude expressed in radians to be set.
     * @param longitude longitude expressed in radians to be set.
     * @param height    height to be set.
     */
    public void setPosition(final double latitude, final double longitude, final Distance height) {
        this.latitude = latitude;
        this.longitude = longitude;
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
        this.height = height;
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
     * Gets curvilinear position, expressed in terms of latitude, longitude and height.
     *
     * @param result instance where curvilinear coordinates will be stored.
     */
    public void getPosition(final NEDPosition result) {
        result.setCoordinates(latitude, longitude, height);
    }

    /**
     * Gets curvilinear position, expressed in terms of latitude, longitude and height.
     *
     * @return curvilinear coordinates.
     */
    public NEDPosition getPosition() {
        return new NEDPosition(latitude, longitude, height);
    }

    /**
     * Sets curvilinear position, expressed in terms of latitude, longitude and height.
     *
     * @param position curvilinear position to be set.
     */
    public void setPosition(final NEDPosition position) {
        latitude = position.getLatitude();
        longitude = position.getLongitude();
        height = position.getHeight();
    }

    /**
     * Gets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along North axis.
     *
     * @return North velocity coordinate value.
     */
    public double getVn() {
        return vn;
    }

    /**
     * Sets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along North axis.
     *
     * @param vn North velocity coordinate value.
     */
    public void setVn(final double vn) {
        this.vn = vn;
    }

    /**
     * Gets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along East axis.
     *
     * @return East velocity coordinate value.
     */
    public double getVe() {
        return ve;
    }

    /**
     * Sets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along East axis.
     *
     * @param ve East velocity coordinate value.
     */
    public void setVe(final double ve) {
        this.ve = ve;
    }

    /**
     * Gets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along Down axis.
     *
     * @return Down velocity coordinate value.
     */
    public double getVd() {
        return vd;
    }

    /**
     * Sets coordinate of velocity of body frame expressed in meters per second (m/s) with respect ECEF frame and
     * resolved along Down axis.
     *
     * @param vd Down velocity coordinate value.
     */
    public void setVd(final double vd) {
        this.vd = vd;
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
        this.vn = vn;
        this.ve = ve;
        this.vd = vd;
    }

    /**
     * Gets norm of velocity expressed in meters per second (m/s), which represents
     * the speed of the body.
     *
     * @return norm of velocity expressed in meters per second (m/s).
     */
    public double getVelocityNorm() {
        return Math.sqrt(vn * vn + ve * ve + vd * vd);
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
        result.setValue(vn);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along North axis.
     *
     * @return North velocity coordinate.
     */
    public Speed getSpeedN() {
        return new Speed(vn, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along North axis.
     *
     * @param speedN North velocity coordinate to be set.
     */
    public void setSpeedN(final Speed speedN) {
        vn = SpeedConverter.convert(speedN.getValue().doubleValue(), speedN.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along East axis.
     *
     * @param result instance where East velocity coordinate will be stored.
     */
    public void getSpeedE(final Speed result) {
        result.setValue(ve);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along East axis.
     *
     * @return East velocity coordinate.
     */
    public Speed getSpeedE() {
        return new Speed(ve, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along East axis.
     *
     * @param speedE East velocity coordinate to be set.
     */
    public void setSpeedE(final Speed speedE) {
        ve = SpeedConverter.convert(speedE.getValue().doubleValue(), speedE.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along Down axis.
     *
     * @param result instance where Down velocity coordinate will be stored.
     */
    public void getSpeedD(final Speed result) {
        result.setValue(vd);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along Down axis.
     *
     * @return Down velocity coordinate.
     */
    public Speed getSpeedD() {
        return new Speed(vd, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets coordinate of velocity of body frame with respect ECEF frame and
     * resolved along Down axis.
     *
     * @param speedD Down velocity coordinate to be set.
     */
    public void setSpeedD(final Speed speedD) {
        vd = SpeedConverter.convert(speedD.getValue().doubleValue(), speedD.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets velocity coordinates of body frame resolved along North, East, Down
     * axes.
     *
     * @param speedN North velocity coordinate.
     * @param speedE East velocity coordinate.
     * @param speedD Down velocity coordinate.
     */
    public void setSpeedCoordinates(final Speed speedN, final Speed speedE, final Speed speedD) {
        setSpeedN(speedN);
        setSpeedE(speedE);
        setSpeedD(speedD);
    }

    /**
     * Gets velocity coordinates of body frame resolved along North, East, Down axes.
     *
     * @param result instance where velocity values will be stored.
     */
    public void getVelocity(final NEDVelocity result) {
        result.setCoordinates(vn, ve, vd);
    }

    /**
     * Gets velocity coordinates of body frame resolved along North, East, Down axes.
     *
     * @return velocity coordinates.
     */
    public NEDVelocity getVelocity() {
        return new NEDVelocity(vn, ve, vd);
    }

    /**
     * Sets velocity coordinates of body frame resolved along North, East, Down axes.
     *
     * @param velocity velocity to be set.
     */
    public void setVelocity(final NEDVelocity velocity) {
        vn = velocity.getVn();
        ve = velocity.getVe();
        vd = velocity.getVd();
    }

    /**
     * Gets coordinate transformation matrix.
     *
     * @return coordinate transformation matrix.
     */
    @Override
    public CoordinateTransformation getCoordinateTransformation() {
        final var result = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
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
        c.copyTo(result);
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
        c.matrix.copyTo(result);
    }

    /**
     * Sets coordinate transformation matrix keeping current source and destination {@link FrameType}.
     * This is more efficient than getting a copy of coordinate transformation calling to
     * {@link #getCoordinateTransformation()}, setting coordinate matrix into copied coordinate transformation and
     * then setting the coordinate transformation calling
     * {@link #setCoordinateTransformation(CoordinateTransformation)}.
     *
     * @param matrix    a 3x3 coordinate transformation matrix to be set.
     * @param threshold threshold to validate rotation matrix.
     * @throws InvalidRotationMatrixException if provided matrix is not a valid rotation matrix (3x3 and orthonormal).
     * @throws IllegalArgumentException       if provided threshold is negative.
     */
    @Override
    public void setCoordinateTransformationMatrix(final Matrix matrix, final double threshold)
            throws InvalidRotationMatrixException {
        c.setMatrix(matrix,threshold);
    }

    /**
     * Sts coordinate transformation matrix keeping current source and destination {@link FrameType}.
     * This is more efficient than getting a copy of coordinate transformation calling to
     * {@link #getCoordinateTransformation()}, setting coordinate matrix into copied coordinate transformation and
     * then setting the coordinate transformation calling
     * {@link #setCoordinateTransformation(CoordinateTransformation)}.
     *
     * @param matrix a 3x3 coordinate transformation matrix to be set.
     * @throws InvalidRotationMatrixException if provided matrix is not a valid rotation matrix (3x3 and orthonormal).
     */
    @Override
    public void setCoordinateTransformationMatrix(final Matrix matrix) throws InvalidRotationMatrixException {
        c.setMatrix(matrix);
    }

    /**
     * Gets coordinate transformation as a new 3D rotation instance.
     * This is equivalent to calling getCoordinateTransformation().asRotation(), but more efficient.
     *
     * @return new coordinate transformation as a 3D rotation.
     * @throws InvalidRotationMatrixException if internal matrix cannot be converted to a 3D rotation.
     */
    @Override
    public Rotation3D getCoordinateTransformationRotation() throws InvalidRotationMatrixException {
        return c.asRotation();
    }

    /**
     * Gets coordinate transformation as a 3D rotation.
     * This is equivalent to calling getCoordinateTransformation().asRotation(), but more efficient.
     *
     * @param result instance where coordinate transformation 3D rotation will be copied to.
     * @throws InvalidRotationMatrixException if internal matrix cannot be converted to a 3D rotation.
     */
    @Override
    public void getCoordinateTransformationRotation(final Rotation3D result) throws InvalidRotationMatrixException {
        c.asRotation(result);
    }

    /**
     * Sets coordinate transformation from 3D rotation and keeping current source and destination {@link FrameType}.
     * This is more efficient than getting a copy of coordinate transformation calling to
     * {@link #getCoordinateTransformation()}, setting rotation into copied coordinate transformation and
     * then setting the coordinate transformation calling
     * {@link #setCoordinateTransformation(CoordinateTransformation)}.
     *
     * @param rotation set rotation into current coordinate rotation.
     */
    @Override
    public void setCoordinateTransformationRotation(final Rotation3D rotation) {
        c.fromRotation(rotation);
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

        this.c = c;
    }

    /**
     * Checks whether provided coordinate transformation matrix is valid or not.
     * Only body to NED transformation matrices are considered to be valid.
     *
     * @param c coordinate transformation matrix to be checked.
     * @return true if provided value is valid, false otherwise.
     */
    public static boolean isValidCoordinateTransformation(final CoordinateTransformation c) {
        return c.getSourceType() == FrameType.BODY_FRAME
                && c.getDestinationType() == FrameType.LOCAL_NAVIGATION_FRAME;
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final NEDFrame output) {
        output.latitude = latitude;
        output.longitude = longitude;
        output.height = height;

        output.vn = vn;
        output.ve = ve;
        output.vd = vd;

        c.copyTo(output.c);
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final NEDFrame input) {
        latitude = input.latitude;
        longitude = input.longitude;
        height = input.height;

        vn = input.vn;
        ve = input.ve;
        vd = input.vd;

        c.copyFrom(input.c);
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(latitude, longitude, height, vn, ve, vd, c);
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

        return Math.abs(latitude - other.latitude) <= threshold
                && Math.abs(longitude - other.longitude) <= threshold
                && Math.abs(height - other.height) <= threshold
                && Math.abs(vn - other.vn) <= threshold
                && Math.abs(ve - other.ve) <= threshold
                && Math.abs(vd - other.vd) <= threshold
                && c.equals(other.c, threshold);
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final var result = (NEDFrame)super.clone();
        copyTo(result);
        return result;
    }
}
