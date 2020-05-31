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
