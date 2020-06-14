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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class BodyKinematicsSequenceTest {

    private static final double MIN_SPECIFIC_FORCE = -9.81;
    private static final double MAX_SPECIFIC_FORCE = 9.81;

    private static final double MIN_ANGULAR_RATE_VALUE = -1.0;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    private static final double MIN_TIMESTAMP_VALUE = -10.0;
    private static final double MAX_TIMESTAMP_VALUE = 10.0;

    @Test
    public void testConstructor1() {
        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence<>();

        assertFalse(sequence.getSortedItems(null));
        assertNull(sequence.getSortedItems());
        assertEquals(sequence.getItemsCount(), 0);
    }

    @Test
    public void testConstructor2() {
        final List<StandardDeviationTimedBodyKinematics> items = createItems();

        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence<>(items);

        final List<StandardDeviationTimedBodyKinematics> sorted1 = new ArrayList<>();
        assertTrue(sequence.getSortedItems(sorted1));
        final List<StandardDeviationTimedBodyKinematics> sorted2 = sequence
                .getSortedItems();

        // check
        assertEquals(sorted1, sorted2);

        assertEquals(sorted1.size(), 2);
        assertTrue(sorted1.get(0).getTimestampSeconds()
                < sorted1.get(1).getTimestampSeconds());
        assertEquals(sequence.getItemsCount(), 2);
    }

    @Test
    public void testConstructor3() throws InvalidSourceAndDestinationFrameTypeException {
        final CoordinateTransformation cbn = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence<>(cbn);

        // check
        assertSame(sequence.getStartBodyAttitude(), cbn);

        // Force InvalidSourceAndDestinationFrameTypeException
        sequence = null;
        try {
            sequence = new BodyKinematicsSequence<>(
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.LOCAL_NAVIGATION_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
        }
        assertNull(sequence);
    }

    @Test
    public void testConstructor4() throws InvalidSourceAndDestinationFrameTypeException {
        final List<StandardDeviationTimedBodyKinematics> items = createItems();
        final CoordinateTransformation cbn = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence<>(items, cbn);

        final List<StandardDeviationTimedBodyKinematics> sorted1 = new ArrayList<>();
        assertTrue(sequence.getSortedItems(sorted1));
        final List<StandardDeviationTimedBodyKinematics> sorted2 = sequence
                .getSortedItems();

        // check
        assertEquals(sorted1, sorted2);

        assertEquals(sorted1.size(), 2);
        assertTrue(sorted1.get(0).getTimestampSeconds()
                < sorted1.get(1).getTimestampSeconds());
        assertEquals(sequence.getItemsCount(), 2);
        assertSame(sequence.getStartBodyAttitude(), cbn);

        // Force InvalidSourceAndDestinationFrameTypeException
        sequence = null;
        try {
            sequence = new BodyKinematicsSequence<>(items,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.LOCAL_NAVIGATION_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
        }
        assertNull(sequence);
    }

    @Test
    public void testSetItems() {
        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence<>();

        assertFalse(sequence.getSortedItems(null));
        assertNull(sequence.getSortedItems());

        // set items
        final List<StandardDeviationTimedBodyKinematics> items = createItems();
        sequence.setItems(items);

        // check
        final List<StandardDeviationTimedBodyKinematics> sorted = sequence
                .getSortedItems();

        assertEquals(sorted.size(), 2);
        assertTrue(sorted.get(0).getTimestampSeconds()
                < sorted.get(1).getTimestampSeconds());
        assertEquals(sequence.getItemsCount(), 2);
    }

    @Test
    public void testIsValidBodyToEcefCoordinateTransformationMatrix() {
        final CoordinateTransformation cbn1 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final CoordinateTransformation cbn2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        assertFalse(BodyKinematicsSequence.isValidBodyToEcefCoordinateTransformationMatrix(cbn1));
        assertTrue(BodyKinematicsSequence.isValidBodyToEcefCoordinateTransformationMatrix(cbn2));
    }

    @Test
    public void testGetSetStartBodyAttitude() throws InvalidSourceAndDestinationFrameTypeException {
        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence<>();

        // check initial value
        assertNull(sequence.getStartBodyAttitude());

        // set new value
        final CoordinateTransformation cbn = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        sequence.setStartBodyAttitude(cbn);

        // check
        assertSame(sequence.getStartBodyAttitude(), cbn);

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            sequence.setStartBodyAttitude(
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.LOCAL_NAVIGATION_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
        }
    }

    private List<StandardDeviationTimedBodyKinematics> createItems() {
        final BodyKinematics kinematics1 = createBodyKinematics();
        final BodyKinematics kinematics2 = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final double specificForceStandardDeviation1 =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double specificForceStandardDeviation2 =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);

        final double angularRateStandardDeviation1 =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final double angularRateStandardDeviation2 =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics1 =
                new StandardDeviationTimedBodyKinematics(kinematics1, timestampSeconds1,
                        specificForceStandardDeviation1, angularRateStandardDeviation1);
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics2 =
                new StandardDeviationTimedBodyKinematics(kinematics2, timestampSeconds2,
                        specificForceStandardDeviation2, angularRateStandardDeviation2);

        return Arrays.asList(
                standardDeviationTimedBodyKinematics1, standardDeviationTimedBodyKinematics2);
    }

    private BodyKinematics createBodyKinematics() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final double angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final double angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final double angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);

        return new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }
}
