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

import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.RankDeficientMatrixException;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.SerializationHelper;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class CoordinateTransformationTest {

    private static final double THRESHOLD = 1e-6;
    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;
    private static final double LATITUDE_DEGREES = 41.3825;
    private static final double LONGITUDE_DEGREES = 2.176944;
    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double ABSOLUTE_ERROR = 1e-6;

    @Test
    void testConstants() {
        assertEquals(3, CoordinateTransformation.ROWS);
        assertEquals(3, CoordinateTransformation.COLS);
        assertEquals(1e-11, CoordinateTransformation.DEFAULT_THRESHOLD, 0.0);
        assertEquals(CoordinateTransformation.EARTH_ROTATION_RATE, Constants.EARTH_ROTATION_RATE, 0.0);
    }

    @Test
    void testConstructor() throws WrongSizeException, InvalidRotationMatrixException {

        // test constructor with source and destination
        var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        // check
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new CoordinateTransformation(null,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));
        assertThrows(NullPointerException.class, () -> new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                null));

        // test constructor with matrix, source, destination and threshold
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        c = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, THRESHOLD);

        // check
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c.getDestinationType());
        assertEquals(m, c.getMatrix());

        // Force InvalidRotationMatrixException
        assertThrows(InvalidRotationMatrixException.class, () -> new CoordinateTransformation(
                new Matrix(3, 3), FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME, THRESHOLD));
        assertThrows(InvalidRotationMatrixException.class, () -> new CoordinateTransformation(
                new Matrix(1, 3), FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME,
                THRESHOLD));
        assertThrows(InvalidRotationMatrixException.class, () -> new CoordinateTransformation(
                new Matrix(3, 1), FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME,
                THRESHOLD));

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new CoordinateTransformation(m, null,
                FrameType.LOCAL_NAVIGATION_FRAME, THRESHOLD));
        assertThrows(NullPointerException.class, () -> new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                null, THRESHOLD));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new CoordinateTransformation(m,
                FrameType.LOCAL_NAVIGATION_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, -1.0));


        // test constructor with matrix, source and destination
        c = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        // check
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c.getDestinationType());
        assertEquals(m, c.getMatrix());

        // Force InvalidRotationMatrixException
        assertThrows(InvalidRotationMatrixException.class, () -> new CoordinateTransformation(
                new Matrix(3, 3), FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME));
        assertThrows(InvalidRotationMatrixException.class, () -> new CoordinateTransformation(
                new Matrix(1, 3), FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME));
        assertThrows(InvalidRotationMatrixException.class, () -> new CoordinateTransformation(
                new Matrix(3, 1), FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME));

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new CoordinateTransformation(m, null,
                FrameType.LOCAL_NAVIGATION_FRAME));
        assertThrows(NullPointerException.class, () -> new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                null));

        // test constructor from euler angles
        c = new CoordinateTransformation(roll, pitch, yaw, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        // check
        assertEquals(roll, c.getRollEulerAngle(), ABSOLUTE_ERROR);
        assertEquals(pitch, c.getPitchEulerAngle(), ABSOLUTE_ERROR);
        assertEquals(yaw, c.getYawEulerAngle(), ABSOLUTE_ERROR);
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c.getDestinationType());


        // test constructor from euler angles
        final var rollAngle = new Angle(roll, AngleUnit.RADIANS);
        final var pitchAngle = new Angle(pitch, AngleUnit.RADIANS);
        final var yawAngle = new Angle(yaw, AngleUnit.RADIANS);

        c = new CoordinateTransformation(rollAngle, pitchAngle, yawAngle, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        // check
        assertEquals(roll, c.getRollEulerAngle(), ABSOLUTE_ERROR);
        assertEquals(pitch, c.getPitchEulerAngle(), ABSOLUTE_ERROR);
        assertEquals(yaw, c.getYawEulerAngle(), ABSOLUTE_ERROR);
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c.getDestinationType());
        assertTrue(c.getRollEulerAngleMeasurement().equals(rollAngle, THRESHOLD));
        assertTrue(c.getPitchEulerAngleMeasurement().equals(pitchAngle, THRESHOLD));
        assertTrue(c.getYawEulerAngleMeasurement().equals(yawAngle, THRESHOLD));

        // test constructor from 3D rotation
        c = new CoordinateTransformation(q, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        // check
        assertEquals(q, c.asRotation());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c.getDestinationType());

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new CoordinateTransformation(q, null,
                FrameType.LOCAL_NAVIGATION_FRAME));
        assertThrows(NullPointerException.class, () -> new CoordinateTransformation(q, FrameType.LOCAL_NAVIGATION_FRAME,
                null));

        // test constructor from another value
        c = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var c2 = new CoordinateTransformation(c);

        // check
        assertEquals(c.getSourceType(), c2.getSourceType());
        assertEquals(c.getDestinationType(), c2.getDestinationType());
        assertEquals(c.getMatrix(), c2.getMatrix());
    }

    @Test
    void testGetMatrix() throws InvalidRotationMatrixException, WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        var m2 = c.getMatrix();

        var m3 = new Matrix(1, 1);
        c.getMatrix(m3);

        assertEquals(m, m2);
        assertNotSame(m, m2);

        assertEquals(m, m3);
        assertNotSame(m, m3);
    }

    @Test
    void testSetMatrixWithThreshold() throws WrongSizeException, InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();

        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        // check default value
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // set new value
        c.setMatrix(m, THRESHOLD);

        // check
        assertEquals(m, c.getMatrix());
        assertNotSame(m, c.getMatrix());

        // Force InvalidRotationMatrixException
        assertThrows(InvalidRotationMatrixException.class, () -> c.setMatrix(new Matrix(3, 3),
                THRESHOLD));
        assertThrows(InvalidRotationMatrixException.class, () -> c.setMatrix(new Matrix(1, 3),
                THRESHOLD));
        assertThrows(InvalidRotationMatrixException.class, () -> c.setMatrix(new Matrix(3, 1),
                THRESHOLD));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> c.setMatrix(m, -1.0));
    }

    @Test
    void testSetMatrixWithoutThreshold() throws WrongSizeException, InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();

        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        // check default value
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // set new value
        c.setMatrix(m);

        // check
        assertEquals(c.getMatrix(), m);
        assertNotSame(c.getMatrix(), m);

        // Force InvalidRotationMatrixException
        assertThrows(InvalidRotationMatrixException.class, () -> c.setMatrix(new Matrix(3, 3)));
        assertThrows(InvalidRotationMatrixException.class, () -> c.setMatrix(new Matrix(1, 3)));
        assertThrows(InvalidRotationMatrixException.class, () -> c.setMatrix(new Matrix(3, 1)));
    }

    @Test
    void testIsValidMatrixWithThreshold() throws WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();

        assertTrue(CoordinateTransformation.isValidMatrix(m, THRESHOLD));

        assertFalse(CoordinateTransformation.isValidMatrix(new Matrix(3, 3), THRESHOLD));
        assertFalse(CoordinateTransformation.isValidMatrix(new Matrix(1, 3), THRESHOLD));
        assertFalse(CoordinateTransformation.isValidMatrix(new Matrix(3, 1), THRESHOLD));

        // test with Nan matrix
        m.initialize(Double.NaN);
        assertFalse(CoordinateTransformation.isValidMatrix(m, THRESHOLD));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> CoordinateTransformation.isValidMatrix(m, -1.0));
    }

    @Test
    void testIsValidMatrixWithoutThreshold() throws WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();

        assertTrue(CoordinateTransformation.isValidMatrix(m));

        assertFalse(CoordinateTransformation.isValidMatrix(new Matrix(3, 3)));
        assertFalse(CoordinateTransformation.isValidMatrix(new Matrix(1, 3)));
        assertFalse(CoordinateTransformation.isValidMatrix(new Matrix(3, 1)));

        // test with Nan matrix
        m.initialize(Double.NaN);
        assertFalse(CoordinateTransformation.isValidMatrix(m));
    }

    @Test
    void testGetSetEulerAngles() {
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        c.setEulerAngles(roll1, pitch1, yaw1);

        final var roll2 = c.getRollEulerAngle();
        final var pitch2 = c.getPitchEulerAngle();
        final var yaw2 = c.getYawEulerAngle();

        assertEquals(roll1, roll2, ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetEulerAnglesMeasurements() {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rollAngle1 = new Angle(roll, AngleUnit.RADIANS);
        final var pitchAngle1 = new Angle(pitch, AngleUnit.RADIANS);
        final var yawAngle1 = new Angle(yaw, AngleUnit.RADIANS);

        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        c.setEulerAngles(rollAngle1, pitchAngle1, yawAngle1);

        final var rollAngle2 = new Angle(0.0, AngleUnit.DEGREES);
        c.getRollEulerAngleMeasurement(rollAngle2);
        final var rollAngle3 = c.getRollEulerAngleMeasurement();

        final var pitchAngle2 = new Angle(0.0, AngleUnit.DEGREES);
        c.getPitchEulerAngleMeasurement(pitchAngle2);
        final var pitchAngle3 = c.getPitchEulerAngleMeasurement();

        final var yawAngle2 = new Angle(0.0, AngleUnit.DEGREES);
        c.getYawEulerAngleMeasurement(yawAngle2);
        final var yawAngle3 = c.getYawEulerAngleMeasurement();

        assertTrue(rollAngle1.equals(rollAngle2, THRESHOLD));
        assertTrue(rollAngle1.equals(rollAngle3, THRESHOLD));

        assertTrue(pitchAngle1.equals(pitchAngle2, THRESHOLD));
        assertTrue(pitchAngle1.equals(pitchAngle3, THRESHOLD));

        assertTrue(yawAngle1.equals(yawAngle2, THRESHOLD));
        assertTrue(yawAngle1.equals(yawAngle3, THRESHOLD));
    }

    @Test
    void testGetSetSourceType() {

        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        // check initial value
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getSourceType());

        // set new value
        c.setSourceType(FrameType.BODY_FRAME);

        // check
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> c.setSourceType(null));
    }

    @Test
    void testGetSetDestinationType() {

        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        // check initial value
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c.getDestinationType());

        // set new value
        c.setDestinationType(FrameType.BODY_FRAME);

        // check
        assertEquals(FrameType.BODY_FRAME, c.getDestinationType());

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> c.setDestinationType(null));
    }

    @Test
    void testAsRotation() throws InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var rotation1 = c.asRotation();
        final var rotation2 = new Quaternion();
        c.asRotation(rotation2);

        assertEquals(m, rotation1.asInhomogeneousMatrix());
        assertTrue(m.equals(rotation2.asInhomogeneousMatrix(), THRESHOLD));
    }

    @Test
    void testFromRotation() throws InvalidRotationMatrixException {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotation1 = new Quaternion(roll, pitch, yaw);

        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        c.fromRotation(rotation1);
        final var rotation2 = c.asRotation();
        assertEquals(rotation2, rotation1);
        assertEquals(rotation1.asInhomogeneousMatrix(), c.getMatrix());
    }

    @Test
    void testCopyTo() throws InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c1 = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var c2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

        c1.copyTo(c2);

        // check
        assertEquals(c1.getSourceType(), c2.getSourceType());
        assertEquals(c1.getDestinationType(), c2.getDestinationType());
        assertEquals(c1.getMatrix(), c2.getMatrix());
    }

    @Test
    void testCopyFrom() throws InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c1 = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var c2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

        c2.copyFrom(c1);

        // check
        assertEquals(c1.getSourceType(), c2.getSourceType());
        assertEquals(c1.getDestinationType(), c2.getDestinationType());
        assertEquals(c1.getMatrix(), c2.getMatrix());
    }

    @Test
    void testHashCode() throws InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c1 = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var c2 = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var c3 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

        assertEquals(c1.hashCode(), c2.hashCode());
        assertNotEquals(c1.hashCode(), c3.hashCode());
    }

    @Test
    void testEquals() throws InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c1 = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var c2 = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var c3 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(c1.equals((Object) c1));
        //noinspection EqualsWithItself
        assertTrue(c1.equals(c1));
        assertTrue(c1.equals(c2));
        assertFalse(c1.equals(c3));
        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertFalse(c1.equals((Object) null));
        assertFalse(c1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertNotEquals(new Object(), c1);
    }

    @Test
    void testEqualsWithThreshold() throws InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c1 = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var c2 = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var c3 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

        assertTrue(c1.equals(c1, THRESHOLD));
        assertTrue(c1.equals(c2, THRESHOLD));
        assertFalse(c1.equals(c3, THRESHOLD));
        assertFalse(c1.equals(null, THRESHOLD));
    }

    @Test
    void testInverse() throws InvalidRotationMatrixException, WrongSizeException, RankDeficientMatrixException,
            DecomposerException {

        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c1 = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var c2 = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var c3 = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        // check that initially they are equal
        assertEquals(c1, c2);
        assertEquals(c1, c3);

        c1.inverse(c2);
        c3.inverse();

        assertEquals(c2, c3);
        assertNotEquals(c1, c2);

        final var m2 = c2.getMatrix();

        assertEquals(m2, m.transposeAndReturnNew());
        assertTrue(m2.equals(Utils.inverse(m), THRESHOLD));
    }

    @Test
    void testInverseAndReturnNew() throws InvalidRotationMatrixException, WrongSizeException,
            RankDeficientMatrixException, DecomposerException {

        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var invC = c.inverseAndReturnNew();

        assertNotEquals(c, invC);

        final var invM = invC.getMatrix();

        assertEquals(invM, m.transposeAndReturnNew());
        assertTrue(invM.equals(Utils.inverse(m), THRESHOLD));
    }

    @Test
    void testEcefToNedMatrix() throws WrongSizeException {
        final var cosLat = Math.cos(Math.toRadians(LATITUDE_DEGREES));
        final var sinLat = Math.sin(Math.toRadians(LATITUDE_DEGREES));
        final var cosLong = Math.cos(Math.toRadians(LONGITUDE_DEGREES));
        final var sinLong = Math.sin(Math.toRadians(LONGITUDE_DEGREES));

        final var cen = new Matrix(3, 3);
        cen.setElementAt(0, 0, -sinLat * cosLong);
        cen.setElementAt(1, 0, -sinLong);
        cen.setElementAt(2, 0, -cosLat * cosLong);

        cen.setElementAt(0, 1, -sinLat * sinLong);
        cen.setElementAt(1, 1, cosLong);
        cen.setElementAt(2, 1, -cosLat * sinLong);

        cen.setElementAt(0, 2, cosLat);
        cen.setElementAt(1, 2, 0.0);
        cen.setElementAt(2, 2, -sinLat);

        final var cen1 = new Matrix(1, 1);
        final var cen2 = new Matrix(3, 3);
        CoordinateTransformation.ecefToNedMatrix(Math.toRadians(LATITUDE_DEGREES),
                Math.toRadians(LONGITUDE_DEGREES), cen1);
        CoordinateTransformation.ecefToNedMatrix(Math.toRadians(LATITUDE_DEGREES),
                Math.toRadians(LONGITUDE_DEGREES), cen2);
        final var cen3 = CoordinateTransformation.ecefToNedMatrix(
                Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES));

        assertEquals(cen1, cen2);
        assertEquals(cen1, cen3);
        assertEquals(cen1, cen);

        final var latitude = new Angle(LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var longitude = new Angle(LONGITUDE_DEGREES, AngleUnit.DEGREES);

        final var cen4 = new Matrix(3, 3);
        CoordinateTransformation.ecefToNedMatrix(latitude, longitude, cen4);
        final var cen5 = CoordinateTransformation.ecefToNedMatrix(latitude, longitude);

        assertEquals(cen1, cen4);
        assertEquals(cen1, cen5);
    }

    @Test
    void testEcefToNedCoordinateTransformationMatrix() {

        final var c1 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        CoordinateTransformation.ecefToNedCoordinateTransformationMatrix(Math.toRadians(LATITUDE_DEGREES),
                Math.toRadians(LONGITUDE_DEGREES), c1);
        final var c2 = CoordinateTransformation.ecefToNedCoordinateTransformationMatrix(
                Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES));

        final var cen = CoordinateTransformation.ecefToNedMatrix(
                Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES));

        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c1.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c1.getDestinationType());
        assertEquals(c1.getMatrix(), cen);

        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c2.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c2.getDestinationType());
        assertEquals(c2.getMatrix(), cen);

        final var latitude = new Angle(LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var longitude = new Angle(LONGITUDE_DEGREES, AngleUnit.DEGREES);

        final var c3 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        CoordinateTransformation.ecefToNedCoordinateTransformationMatrix(latitude, longitude, c3);
        final var c4 = CoordinateTransformation.ecefToNedCoordinateTransformationMatrix(latitude, longitude);

        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c3.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c3.getDestinationType());
        assertEquals(c3.getMatrix(), cen);

        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c4.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c4.getDestinationType());
        assertEquals(c4.getMatrix(), cen);
    }

    @Test
    void testNedToEcefMatrix() throws WrongSizeException {
        final var cen = CoordinateTransformation.ecefToNedMatrix(
                Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES));
        final var cne = cen.transposeAndReturnNew();

        final var cne1 = new Matrix(1, 1);
        final var cne2 = new Matrix(3, 3);
        CoordinateTransformation.nedToEcefMatrix(
                Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES), cne1);
        CoordinateTransformation.nedToEcefMatrix(
                Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES), cne2);
        final var cne3 = CoordinateTransformation.nedToEcefMatrix(
                Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES));

        assertEquals(cne1, cne2);
        assertEquals(cne1, cne3);
        assertEquals(cne1, cne);

        final var latitude = new Angle(LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var longitude = new Angle(LONGITUDE_DEGREES, AngleUnit.DEGREES);

        final var cne4 = new Matrix(3, 3);
        CoordinateTransformation.nedToEcefMatrix(latitude, longitude, cne4);
        final var cne5 = CoordinateTransformation.nedToEcefMatrix(latitude, longitude);

        assertEquals(cne1, cne4);
        assertEquals(cne1, cne5);
    }

    @Test
    void testNedToEcefCoordinateTransformationMatrix() {
        final var c1 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        CoordinateTransformation.nedToEcefCoordinateTransformationMatrix(
                Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES), c1);
        final var c2 = CoordinateTransformation.nedToEcefCoordinateTransformationMatrix(
                Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES));

        final var cne = CoordinateTransformation.nedToEcefMatrix(
                Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES));

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c1.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c1.getDestinationType());
        assertEquals(c1.getMatrix(), cne);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c2.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c2.getDestinationType());
        assertEquals(c2.getMatrix(), cne);

        final var latitude = new Angle(LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var longitude = new Angle(LONGITUDE_DEGREES, AngleUnit.DEGREES);

        final var c3 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        CoordinateTransformation.nedToEcefCoordinateTransformationMatrix(latitude, longitude, c3);
        final var c4 = CoordinateTransformation.nedToEcefCoordinateTransformationMatrix(latitude, longitude);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c3.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c3.getDestinationType());
        assertEquals(c3.getMatrix(), cne);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c4.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c4.getDestinationType());
        assertEquals(c4.getMatrix(), cne);
    }

    @Test
    void testEcefToEciMatrixFromTimeInterval() throws WrongSizeException {
        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var cei1 = new Matrix(3, 3);
        CoordinateTransformation.ecefToEciMatrixFromTimeInterval(timeInterval, cei1);
        final var cei2 = CoordinateTransformation.ecefToEciMatrixFromTimeInterval(timeInterval);
        final var cei3 = new Matrix(3, 3);
        CoordinateTransformation.ecefToEciMatrixFromTimeInterval(TIME_INTERVAL_SECONDS, cei3);
        final var cei4 = CoordinateTransformation.ecefToEciMatrixFromTimeInterval(TIME_INTERVAL_SECONDS);

        assertEquals(cei1, cei2);
        assertEquals(cei1, cei3);
        assertEquals(cei1, cei4);

        final var alpha = CoordinateTransformation.EARTH_ROTATION_RATE * TIME_INTERVAL_SECONDS;
        final var cei = Matrix.identity(3, 3);
        cei.setElementAtIndex(0, Math.cos(alpha));
        cei.setElementAtIndex(1, Math.sin(alpha));
        cei.setElementAtIndex(3, -Math.sin(alpha));
        cei.setElementAtIndex(4, Math.cos(alpha));
        assertEquals(cei1, cei);

        // test when zero time interval is equal to the identity
        final var cei5 = CoordinateTransformation.ecefToEciMatrixFromTimeInterval(0.0);
        assertEquals(cei5, Matrix.identity(3, 3));
    }

    @Test
    void testEcefToEciMatrixFromAngle() throws WrongSizeException {
        final var alpha = CoordinateTransformation.EARTH_ROTATION_RATE * TIME_INTERVAL_SECONDS;
        final var angle = new Angle(alpha, AngleUnit.RADIANS);
        final var cei1 = new Matrix(3, 3);
        CoordinateTransformation.ecefToEciMatrixFromAngle(angle, cei1);
        final var cei2 = CoordinateTransformation.ecefToEciMatrixFromAngle(angle);
        final var cei3 = new Matrix(3, 3);
        CoordinateTransformation.ecefToEciMatrixFromAngle(alpha, cei3);
        final var cei4 = CoordinateTransformation.ecefToEciMatrixFromAngle(alpha);

        assertEquals(cei1, cei2);
        assertEquals(cei1, cei3);
        assertEquals(cei1, cei4);

        final var cei5 = CoordinateTransformation.ecefToEciMatrixFromTimeInterval(TIME_INTERVAL_SECONDS);

        assertEquals(cei1, cei5);

        final var cei = Matrix.identity(3, 3);
        cei.setElementAtIndex(0, Math.cos(alpha));
        cei.setElementAtIndex(1, Math.sin(alpha));
        cei.setElementAtIndex(3, -Math.sin(alpha));
        cei.setElementAtIndex(4, Math.cos(alpha));
        assertEquals(cei1, cei);

        // test when zero angle is equal to the identity
        final var cei6 = CoordinateTransformation.ecefToEciMatrixFromAngle(0.0);
        assertEquals(cei6, Matrix.identity(3, 3));
    }

    @Test
    void testEcefToEciCoordinateTransformationMatrixFromTimeInterval() {
        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var c1 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        CoordinateTransformation.ecefToEciCoordinateTransformationMatrixFromTimeInterval(timeInterval, c1);
        final var c2 = CoordinateTransformation.ecefToEciCoordinateTransformationMatrixFromTimeInterval(timeInterval);
        final var c3 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        CoordinateTransformation.ecefToEciCoordinateTransformationMatrixFromTimeInterval(TIME_INTERVAL_SECONDS, c3);
        final var c4 = CoordinateTransformation.ecefToEciCoordinateTransformationMatrixFromTimeInterval(
                TIME_INTERVAL_SECONDS);

        final var cei = CoordinateTransformation.ecefToEciMatrixFromTimeInterval(TIME_INTERVAL_SECONDS);

        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c1.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c1.getDestinationType());
        assertEquals(c1.getMatrix(), cei);

        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c2.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c2.getDestinationType());
        assertEquals(c2.getMatrix(), cei);

        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c3.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c3.getDestinationType());
        assertEquals(c3.getMatrix(), cei);

        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c4.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c4.getDestinationType());
        assertEquals(c4.getMatrix(), cei);
    }

    @Test
    void testEcefToEciCoordinateTransformationMatrixFromAngle() {
        final var alpha = CoordinateTransformation.EARTH_ROTATION_RATE * TIME_INTERVAL_SECONDS;
        final var angle = new Angle(alpha, AngleUnit.RADIANS);

        final var c1 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        CoordinateTransformation.ecefToEciCoordinateTransformationMatrixFromAngle(angle, c1);
        final var c2 = CoordinateTransformation.ecefToEciCoordinateTransformationMatrixFromAngle(angle);
        final var c3 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        CoordinateTransformation.ecefToEciCoordinateTransformationMatrixFromAngle(alpha, c3);
        final var c4 = CoordinateTransformation.ecefToEciCoordinateTransformationMatrixFromAngle(alpha);

        final var cei = CoordinateTransformation.ecefToEciMatrixFromAngle(alpha);

        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c1.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c1.getDestinationType());
        assertEquals(c1.getMatrix(), cei);

        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c2.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c2.getDestinationType());
        assertEquals(c2.getMatrix(), cei);

        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c3.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c3.getDestinationType());
        assertEquals(c3.getMatrix(), cei);

        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c4.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c4.getDestinationType());
        assertEquals(c4.getMatrix(), cei);
    }

    @Test
    void testEciToEcefMatrixFromTimeInterval() throws WrongSizeException, RankDeficientMatrixException,
            DecomposerException {

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var cie1 = new Matrix(3, 3);
        CoordinateTransformation.eciToEcefMatrixFromTimeInterval(timeInterval, cie1);
        final var cie2 = CoordinateTransformation.eciToEcefMatrixFromTimeInterval(timeInterval);
        final var cie3 = new Matrix(3, 3);
        CoordinateTransformation.eciToEcefMatrixFromTimeInterval(TIME_INTERVAL_SECONDS, cie3);
        final var cie4 = CoordinateTransformation.eciToEcefMatrixFromTimeInterval(TIME_INTERVAL_SECONDS);

        assertEquals(cie1, cie2);
        assertEquals(cie1, cie3);
        assertEquals(cie1, cie4);

        final var cei = CoordinateTransformation.ecefToEciMatrixFromTimeInterval(TIME_INTERVAL_SECONDS);

        assertTrue(cie1.equals(Utils.inverse(cei), THRESHOLD));

        // test when zero time interval is equal to the identity
        final var cie5 = CoordinateTransformation.eciToEcefMatrixFromTimeInterval(0.0);
        assertEquals(cie5, Matrix.identity(3, 3));
    }

    @Test
    void testEciToEcefMatrixFromAngle() throws WrongSizeException {
        final var alpha = CoordinateTransformation.EARTH_ROTATION_RATE * TIME_INTERVAL_SECONDS;
        final var angle = new Angle(alpha, AngleUnit.RADIANS);
        final var cie1 = new Matrix(3, 3);
        CoordinateTransformation.eciToEcefMatrixFromAngle(angle, cie1);
        final var cie2 = CoordinateTransformation.eciToEcefMatrixFromAngle(angle);
        final var cie3 = new Matrix(3, 3);
        CoordinateTransformation.eciToEcefMatrixFromAngle(alpha, cie3);
        final var cie4 = CoordinateTransformation.eciToEcefMatrixFromAngle(alpha);

        assertEquals(cie1, cie2);
        assertEquals(cie1, cie3);
        assertEquals(cie1, cie4);

        final var cie5 = CoordinateTransformation.eciToEcefMatrixFromTimeInterval(TIME_INTERVAL_SECONDS);

        assertEquals(cie1, cie5);

        // test when zero angle is equal to the identity
        final var cie6 = CoordinateTransformation.eciToEcefMatrixFromAngle(0.0);
        assertEquals(cie6, Matrix.identity(3, 3));
    }

    @Test
    void testEciToEcefCoordinateTransformationMatrixFromTimeInterval() {
        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var c1 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        CoordinateTransformation.eciToEcefCoordinateTransformationMatrixFromTimeInterval(timeInterval, c1);
        final var c2 = CoordinateTransformation.eciToEcefCoordinateTransformationMatrixFromTimeInterval(timeInterval);
        final var c3 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        CoordinateTransformation.eciToEcefCoordinateTransformationMatrixFromTimeInterval(TIME_INTERVAL_SECONDS, c3);
        final var c4 = CoordinateTransformation.eciToEcefCoordinateTransformationMatrixFromInterval(
                TIME_INTERVAL_SECONDS);

        final var cie = CoordinateTransformation.eciToEcefMatrixFromTimeInterval(TIME_INTERVAL_SECONDS);

        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c1.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c1.getDestinationType());
        assertEquals(c1.getMatrix(), cie);

        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c2.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c2.getDestinationType());
        assertEquals(c2.getMatrix(), cie);

        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c3.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c3.getDestinationType());
        assertEquals(c3.getMatrix(), cie);

        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c4.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c4.getDestinationType());
        assertEquals(c4.getMatrix(), cie);
    }

    @Test
    void testEciToEcefCoordinateTransformationMatrixFromAngle() {
        final var alpha = CoordinateTransformation.EARTH_ROTATION_RATE * TIME_INTERVAL_SECONDS;
        final var angle = new Angle(alpha, AngleUnit.RADIANS);

        final var c1 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        CoordinateTransformation.eciToEcefCoordinateTransformationMatrixFromAngle(angle, c1);
        final var c2 = CoordinateTransformation.eciToEcefCoordinateTransformationMatrixFromAngle(angle);
        final var c3 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        CoordinateTransformation.eciToEcefCoordinateTransformationMatrixFromAngle(alpha, c3);
        final var c4 = CoordinateTransformation.eciToEcefCoordinateTransformationMatrixFromAngle(alpha);

        final var cie = CoordinateTransformation.eciToEcefMatrixFromAngle(alpha);

        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c1.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c1.getDestinationType());
        assertEquals(c1.getMatrix(), cie);

        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c2.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c2.getDestinationType());
        assertEquals(c2.getMatrix(), cie);

        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c3.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c3.getDestinationType());
        assertEquals(c3.getMatrix(), cie);

        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c4.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c4.getDestinationType());
        assertEquals(c4.getMatrix(), cie);
    }

    @Test
    void testClone() throws InvalidRotationMatrixException, CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();

        final var c1 = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var c2 = c1.clone();

        assertEquals(c1, c2);
    }

    @Test
    void testSerializeAndDeserialize() throws InvalidRotationMatrixException, IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c1 = new CoordinateTransformation(m, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, THRESHOLD);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(c1);
        final var c2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(c1, c2);
        assertNotSame(c1, c2);
    }
}
