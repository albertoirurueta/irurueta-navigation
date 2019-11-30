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

import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.units.Distance;
import com.irurueta.units.Speed;

/**
 * Contains position, velocity and coordinates transformation matrix expressed in ECEF frame.
 * Position and velocity of this frame is expressed along ECEF axes as described here:
 * {@link FrameType#EARTH_CENTERED_EARTH_FIXED_FRAME}.
 */
@SuppressWarnings("WeakerAccess")
public class ECEFFrame extends ECIorECEFFrame<ECEFFrame> {

    /**
     * Constructor.
     * Initializes position and velocity coordinates to zero and the coordinate transformation matrix to the
     * identity.
     */
    public ECEFFrame() {
        mC = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
    }

    /**
     * Constructor.
     *
     * @param c Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        super(c);
    }

    /**
     * Constructor.
     *
     * @param x cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     * @param y cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     * @param z cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     */
    public ECEFFrame(final double x, final double y, final double z) {
        this();
        setCoordinates(x, y, z);
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in meters (m) and resolved along ECEF-frame axes.
     */
    public ECEFFrame(final Point3D position) {
        this();
        setPosition(position);
    }

    /**
     * Constructor.
     *
     * @param positionX cartesian x coordinate of body position to be set resolved along ECEF-frame axes.
     * @param positionY cartesian y coordinate of body position to be set resolved along ECEF-frame axes.
     * @param positionZ cartesian z coordinate of body position to be set resolved along ECEF-frame axes.
     */
    public ECEFFrame(final Distance positionX, final Distance positionY,
                     final Distance positionZ) {
        this();
        setPositionCoordinates(positionX, positionY, positionZ);
    }

    /**
     * Constructor.
     *
     * @param position cartesian position.
     */
    public ECEFFrame(final ECEFPosition position) {
        this();
        setPosition(position);
    }

    /**
     * Constructor.
     *
     * @param x  cartesian x coordinate of body position expressed in meters (m) and resolved along ECEF-frame axes.
     * @param y  cartesian y coordinate of body position expressed in meters (m) and resolved along ECEF-frame axes.
     * @param z  cartesian z coordinate of body position expressed in meters (m) and resolved along ECEF-frame axes.
     * @param vx x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *           ECEF-frame axes.
     * @param vy y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *           ECEF-frame axes.
     * @param vz z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *           ECEF-frame axes.
     */
    public ECEFFrame(final double x, final double y, final double z,
                     final double vx, final double vy, final double vz) {
        this(x, y, z);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param position cartesian position.
     * @param vx       x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     * @param vy       y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     * @param vz       z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     */
    public ECEFFrame(final ECEFPosition position,
                     final double vx, final double vy, final double vz) {
        this(position);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param position body position expresed in meters (m) and resolved along ECEF-frame axes.
     * @param vx       x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     * @param vy       y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     * @param vz       z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     */
    public ECEFFrame(final Point3D position,
                     final double vx, final double vy, final double vz) {
        this(position);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in meters (m) and resolved along ECEF-frame axes.
     * @param speedX   x coordinate of velocity to be set resolved along ECEF-frame axes.
     * @param speedY   y coordinate of velocity to be set resolved along ECEF-frame axes.
     * @param speedZ   z coordinate of velocity to be set resolved along ECEF-frame axes.
     */
    public ECEFFrame(final Point3D position,
                     final Speed speedX, final Speed speedY, final Speed speedZ) {
        this(position);
        setSpeedCoordinates(speedX, speedY, speedZ);
    }

    /**
     * Constructor.
     *
     * @param position cartesian position.
     * @param speedX   x coordinate of velocity to be set resolved along ECEF-frame axes.
     * @param speedY   y coordinate of velocity to be set resolved along ECEF-frame axes.
     * @param speedZ   z coordinate of velocity to be set resolved along ECEF-frame axes.
     */
    public ECEFFrame(final ECEFPosition position,
                     final Speed speedX, final Speed speedY, final Speed speedZ) {
        this(position);
        setSpeedCoordinates(speedX, speedY, speedZ);
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in meters (m) and resolved along ECEF-frame axes.
     * @param velocity velocity to be set resolved along ECEF-frame axes.
     */
    public ECEFFrame(final Point3D position,
                     final ECEFVelocity velocity) {
        this(position);
        setVelocity(velocity);
    }

    /**
     * Constructor.
     *
     * @param x      cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     *               along ECEF-frame axes.
     * @param y      cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     *               along ECEF-frame axes.
     * @param z      cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     *               along ECEF-frame axes.
     * @param speedX x coordinate of velocity to be set resolved along ECEF-frame axes.
     * @param speedY y coordinate of velocity to be set resolved along ECEF-frame axes.
     * @param speedZ z coordinate of velocity to be set resolved along ECEF-frame axes.
     */
    public ECEFFrame(final double x, final double y, final double z,
                     final Speed speedX, final Speed speedY, final Speed speedZ) {
        this(x, y, z);
        setSpeedCoordinates(speedX, speedY, speedZ);
    }

    /**
     * Constructor.
     *
     * @param x        cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     *                 along ECEF-frame axes.
     * @param y        cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     *                 along ECEF-frame axes.
     * @param z        cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     *                 along ECEF-frame axes.
     * @param velocity velocity to be set resolved along ECEF-frame axes.
     */
    public ECEFFrame(final double x, final double y, final double z,
                     final ECEFVelocity velocity) {
        this(x, y, z);
        setVelocity(velocity);
    }

    /**
     * Constructor.
     *
     * @param positionX cartesian x coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param positionY cartesian y coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param positionZ cartesian z coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param vx        x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                  ECEF-frame axes.
     * @param vy        y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                  ECEF-frame axes.
     * @param vz        z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                  ECEF-frame axes.
     */
    public ECEFFrame(final Distance positionX, final Distance positionY,
                     final Distance positionZ, final double vx,
                     final double vy, final double vz) {
        this(positionX, positionY, positionZ);
        setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Constructor.
     *
     * @param positionX cartesian x coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param positionY cartesian y coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param positionZ cartesian z coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param speedX    x coordinate of velocity to be set, resolved along ECEF-frame axes.
     * @param speedY    y coordinate of velocity to be set, resolved along ECEF-frame axes.
     * @param speedZ    z coordinate of velocity to be set, resolved along ECEF-frame axes.
     */
    public ECEFFrame(final Distance positionX, final Distance positionY,
                     final Distance positionZ, final Speed speedX,
                     final Speed speedY, final Speed speedZ) {
        this(positionX, positionY, positionZ);
        setSpeedCoordinates(speedX, speedY, speedZ);
    }

    /**
     * Constructor.
     *
     * @param positionX cartesian x coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param positionY cartesian y coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param positionZ cartesian z coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param velocity  velocity to be set resolved along ECEF-frame axes.
     */
    public ECEFFrame(final Distance positionX, final Distance positionY,
                     final Distance positionZ, final ECEFVelocity velocity) {
        this(positionX, positionY, positionZ);
        setVelocity(velocity);
    }

    /**
     * Constructor.
     *
     * @param position cartesian position.
     * @param velocity velocity to be set resolved along ECEF-frame axes.
     */
    public ECEFFrame(final ECEFPosition position,
                     final ECEFVelocity velocity) {
        this(position);
        setVelocity(velocity);
    }

    /**
     * Constructor.
     *
     * @param x cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     * @param y cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     * @param z cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *          ECEF-frame axes.
     * @param c Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final double x, final double y, final double z, final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(x, y, z);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param position cartesian position.
     * @param c        Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final ECEFPosition position, final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in meters (m) and resolved along ECEF-frame axes.
     * @param c        Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final Point3D position,
                     final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {

        this(position);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param positionX cartesian x coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param positionY cartesian y coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param positionZ cartesian z coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param c         Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final Distance positionX, final Distance positionY,
                     final Distance positionZ,
                     final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {

        this(positionX, positionY, positionZ);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param x  cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *           ECEF-frame axes.
     * @param y  cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *           ECEF-frame axes.
     * @param z  cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *           ECEF-frame axes.
     * @param vx x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *           ECEF-frame axes.
     * @param vy y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *           ECEF-frame axes.
     * @param vz z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *           ECEF-frame axes.
     * @param c  Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final double x, final double y, final double z,
                     final double vx, final double vy, final double vz,
                     final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(x, y, z, vx, vy, vz);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param position cartesian position.
     * @param vx       x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     * @param vy       y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     * @param vz       z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     * @param c        Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final ECEFPosition position,
                     final double vx, final double vy, final double vz,
                     final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, vx, vy, vz);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param x        cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *                 ECEF-frame axes.
     * @param y        cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *                 ECEF-frame axes.
     * @param z        cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved along
     *                 ECEF-frame axes.
     * @param velocity velocity to be set resolved along ECEF-frame axes.
     * @param c        Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final double x, final double y, final double z,
                     final ECEFVelocity velocity,
                     final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(x, y, z, velocity);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param position cartesian position.
     * @param velocity velocity to be set resolved along ECEF-frame axes.
     * @param c        Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final ECEFPosition position, final ECEFVelocity velocity,
                     final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, velocity);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in meters (m) and resolved along ECEF-frame axes.
     * @param vx       x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     * @param vy       y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     * @param vz       z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                 ECEF-frame axes.
     * @param c        Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final Point3D position,
                     final double vx, final double vy, final double vz,
                     final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, vx, vy, vz);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in meters (m) and resolved along ECEF-frame axes.
     * @param velocity velocity to be set resolved along ECEF-frame axes.
     * @param c        Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final Point3D position,
                     final ECEFVelocity velocity,
                     final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, velocity);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in meters (m) and resolved along ECEF-frame axes.
     * @param speedX   x coordinate of velocity to be set, resolved along ECEF-frame axes.
     * @param speedY   y coordinate of velocity to be set, resolved along ECEF-frame axes.
     * @param speedZ   z coordinate of velocity to be set, resolved along ECEF-frame axes.
     * @param c        Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final Point3D position,
                     final Speed speedX, final Speed speedY, final Speed speedZ,
                     final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, speedX, speedY, speedZ);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param position cartesian position.
     * @param speedX   x coordinate of velocity to be set, resolved along ECEF-frame axes.
     * @param speedY   y coordinate of velocity to be set, resolved along ECEF-frame axes.
     * @param speedZ   z coordinate of velocity to be set, resolved along ECEF-frame axes.
     * @param c        Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final ECEFPosition position,
                     final Speed speedX, final Speed speedY, final Speed speedZ,
                     final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(position, speedX, speedY, speedZ);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param x      cartesian x coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     *               along ECEF-frame axes.
     * @param y      cartesian y coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     *               along ECEF-frame axes.
     * @param z      cartesian z coordinate of body position expressed in meters (m) with respect ECEF frame, resolved
     *               along ECEF-frame axes.
     * @param speedX x coordinate of velocity to be set, resolved along ECEF-frame axes.
     * @param speedY y coordinate of velocity to be set, resolved along ECEF-frame axes.
     * @param speedZ z coordinate of velocity to be set, resolved along ECEF-frame axes.
     * @param c      Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final double x, final double y, final double z,
                     final Speed speedX, final Speed speedY, final Speed speedZ,
                     final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(x, y, z, speedX, speedY, speedZ);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param positionX cartesian x coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param positionY cartesian y coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param positionZ cartesian z coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param vx        x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                  ECEF-frame axes.
     * @param vy        y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                  ECEF-frame axes.
     * @param vz        z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved along
     *                  ECEF-frame axes.
     * @param c         Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final Distance positionX, final Distance positionY,
                     final Distance positionZ, final double vx,
                     final double vy, final double vz,
                     final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(positionX, positionY, positionZ, vx, vy, vz);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param positionX cartesian x coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param positionY cartesian y coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param positionZ cartesian z coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param velocity  velocity to be set resolved along ECEF-frame axes.
     * @param c         Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final Distance positionX, final Distance positionY,
                     final Distance positionZ, final ECEFVelocity velocity,
                     final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(positionX, positionY, positionZ, velocity);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param positionX cartesian x coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param positionY cartesian y coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param positionZ cartesian z coordinate of body position to be set, resolved along ECEF-frame axes.
     * @param speedX    x coordinate of velocity to be set, resolved along ECEF-frame axes.
     * @param speedY    y coordinate of velocity to be set, resolved along ECEF-frame axes.
     * @param speedZ    z coordinate of velocity to be set, resolved along ECEF-frame axes.
     * @param c         Body to ECEF coordinate transformation matrix to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types are invalid.
     */
    public ECEFFrame(final Distance positionX, final Distance positionY,
                     final Distance positionZ, final Speed speedX,
                     final Speed speedY, final Speed speedZ,
                     final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(positionX, positionY, positionZ, speedX, speedY, speedZ);
        setCoordinateTransformation(c);
    }

    /**
     * Constructor.
     *
     * @param input ECEF frame to copy data from.
     */
    public ECEFFrame(final ECEFFrame input) {
        this();
        copyFrom(input);
    }

    /**
     * Gets cartesian position.
     *
     * @param result instance where cartesian position will be stored.
     */
    public void getECEFPosition(final ECEFPosition result) {
        result.setCoordinates(mX, mY, mZ);
    }

    /**
     * Gets cartesian position.
     *
     * @return cartesian position.
     */
    public ECEFPosition getECEFPosition() {
        return new ECEFPosition(mX, mY, mZ);
    }

    /**
     * Sets cartesian position.
     *
     * @param position cartesian position to be set.
     */
    public void setPosition(final ECEFPosition position) {
        mX = position.getX();
        mY = position.getY();
        mZ = position.getZ();
    }

    /**
     * Gets cartesian velocity.
     *
     * @param result instance where cartesian velocity will be stored.
     */
    public void getECEFVelocity(final ECEFVelocity result) {
        result.setCoordinates(mVx, mVy, mVz);
    }

    /**
     * Gets cartesian velocity.
     *
     * @return cartesian velocity.
     */
    public ECEFVelocity getECEFVelocity() {
        return new ECEFVelocity(mVx, mVy, mVz);
    }

    /**
     * Sets cartesian velocity.
     *
     * @param velocity cartesian velocity to be set.
     */
    public void setVelocity(final ECEFVelocity velocity) {
        mVx = velocity.getVx();
        mVy = velocity.getVy();
        mVz = velocity.getVz();
    }

    /**
     * Gets coordinate transformation matrix.
     *
     * @return coordinate transformation matrix.
     */
    @Override
    public CoordinateTransformation getCoordinateTransformation() {
        final CoordinateTransformation result = new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
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
     * Sets coordinate transformation matrix.
     * Provided value must be a body to ECEF transformation matrix.
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
     * Checks whether provided coordinate transformation is valid or not.
     * Only body to ECEF transformation matrices are considered to be valid.
     *
     * @param c coordinate transformation matrix to be checked.
     * @return true if provided value is valid, false otherwise.
     */
    public static boolean isValidCoordinateTransformation(final CoordinateTransformation c) {
        return c.getSourceType() == FrameType.BODY_FRAME &&
                c.getDestinationType() == FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME;
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
        if (!(obj instanceof ECEFFrame)) {
            return false;
        }

        final ECEFFrame other = (ECEFFrame) obj;
        return equals(other);
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     */
    @SuppressWarnings({"MethodDoesntCallSuperMethod", "CloneDoesntDeclareCloneNotSupportedException"})
    @Override
    protected Object clone() {
        return new ECEFFrame(this);
    }
}
