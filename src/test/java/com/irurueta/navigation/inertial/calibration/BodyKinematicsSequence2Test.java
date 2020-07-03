package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class BodyKinematicsSequence2Test {
    private static final double MIN_SPECIFIC_FORCE = -9.81;
    private static final double MAX_SPECIFIC_FORCE = 9.81;

    private static final double MIN_ANGULAR_RATE_VALUE = -1.0;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    private static final double MIN_TIMESTAMP_VALUE = -10.0;
    private static final double MAX_TIMESTAMP_VALUE = 10.0;

    @Test
    public void testConstructor1() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        // check default values
        assertFalse(sequence.getSortedItems(null));
        assertNull(sequence.getSortedItems());
        assertEquals(sequence.getItemsCount(), 0);
        assertEquals(sequence.getBeforeMeanFx(), 0.0, 0.0);
        assertEquals(sequence.getBeforeMeanFy(), 0.0, 0.0);
        assertEquals(sequence.getBeforeMeanFz(), 0.0, 0.0);
        assertEquals(sequence.getAfterMeanFx(), 0.0, 0.0);
        assertEquals(sequence.getAfterMeanFy(), 0.0, 0.0);
        assertEquals(sequence.getAfterMeanFz(), 0.0, 0.0);

        final Acceleration accelerationX1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(accelerationX1);
        final Acceleration accelerationX2 = sequence
                .getBeforeMeanSpecificForceX();

        assertEquals(accelerationX1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationX1, accelerationX2);

        final Acceleration accelerationY1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceY(accelerationY1);
        final Acceleration accelerationY2 = sequence
                .getBeforeMeanSpecificForceY();

        assertEquals(accelerationY1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationY1, accelerationY2);

        final Acceleration accelerationZ1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceZ(accelerationZ1);
        final Acceleration accelerationZ2 = sequence
                .getBeforeMeanSpecificForceZ();

        assertEquals(accelerationZ1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationZ1, accelerationZ2);

        final Acceleration accelerationX3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceX(accelerationX3);
        final Acceleration accelerationX4 = sequence
                .getAfterMeanSpecificForceX();

        assertEquals(accelerationX3.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationX3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationX3, accelerationX4);

        final Acceleration accelerationY3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceY(accelerationY3);
        final Acceleration accelerationY4 = sequence
                .getAfterMeanSpecificForceY();

        assertEquals(accelerationY3.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationY3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationY3, accelerationY4);

        final Acceleration accelerationZ3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceZ(accelerationZ3);
        final Acceleration accelerationZ4 = sequence
                .getAfterMeanSpecificForceZ();

        assertEquals(accelerationZ3.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationZ3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationZ3, accelerationZ4);
    }

    @Test
    public void testConstructor2() {
        final List<StandardDeviationTimedBodyKinematics> items = createItems();

        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>(items);

        // check default values
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

        assertEquals(sequence.getBeforeMeanFx(), 0.0, 0.0);
        assertEquals(sequence.getBeforeMeanFy(), 0.0, 0.0);
        assertEquals(sequence.getBeforeMeanFz(), 0.0, 0.0);
        assertEquals(sequence.getAfterMeanFx(), 0.0, 0.0);
        assertEquals(sequence.getAfterMeanFy(), 0.0, 0.0);
        assertEquals(sequence.getAfterMeanFz(), 0.0, 0.0);

        final Acceleration accelerationX1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(accelerationX1);
        final Acceleration accelerationX2 = sequence
                .getBeforeMeanSpecificForceX();

        assertEquals(accelerationX1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationX1, accelerationX2);

        final Acceleration accelerationY1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceY(accelerationY1);
        final Acceleration accelerationY2 = sequence
                .getBeforeMeanSpecificForceY();

        assertEquals(accelerationY1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationY1, accelerationY2);

        final Acceleration accelerationZ1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceZ(accelerationZ1);
        final Acceleration accelerationZ2 = sequence
                .getBeforeMeanSpecificForceZ();

        assertEquals(accelerationZ1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationZ1, accelerationZ2);

        final Acceleration accelerationX3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceX(accelerationX3);
        final Acceleration accelerationX4 = sequence
                .getAfterMeanSpecificForceX();

        assertEquals(accelerationX3.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationX3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationX3, accelerationX4);

        final Acceleration accelerationY3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceY(accelerationY3);
        final Acceleration accelerationY4 = sequence
                .getAfterMeanSpecificForceY();

        assertEquals(accelerationY3.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationY3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationY3, accelerationY4);

        final Acceleration accelerationZ3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceZ(accelerationZ3);
        final Acceleration accelerationZ4 = sequence
                .getAfterMeanSpecificForceZ();

        assertEquals(accelerationZ3.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationZ3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationZ3, accelerationZ4);
    }

    @Test
    public void testConstructor3() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>(
                        beforeMeanFx, beforeMeanFy, beforeMeanFz,
                        afterMeanFx, afterMeanFy, afterMeanFz);

        // check default values
        assertFalse(sequence.getSortedItems(null));
        assertNull(sequence.getSortedItems());
        assertEquals(sequence.getItemsCount(), 0);
        assertEquals(sequence.getBeforeMeanFx(), beforeMeanFx, 0.0);
        assertEquals(sequence.getBeforeMeanFy(), beforeMeanFy, 0.0);
        assertEquals(sequence.getBeforeMeanFz(), beforeMeanFz, 0.0);
        assertEquals(sequence.getAfterMeanFx(), afterMeanFx, 0.0);
        assertEquals(sequence.getAfterMeanFy(), afterMeanFy, 0.0);
        assertEquals(sequence.getAfterMeanFz(), afterMeanFz, 0.0);

        final Acceleration accelerationX1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(accelerationX1);
        final Acceleration accelerationX2 = sequence
                .getBeforeMeanSpecificForceX();

        assertEquals(accelerationX1.getValue().doubleValue(),
                beforeMeanFx, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationX1, accelerationX2);

        final Acceleration accelerationY1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceY(accelerationY1);
        final Acceleration accelerationY2 = sequence
                .getBeforeMeanSpecificForceY();

        assertEquals(accelerationY1.getValue().doubleValue(),
                beforeMeanFy, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationY1, accelerationY2);

        final Acceleration accelerationZ1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceZ(accelerationZ1);
        final Acceleration accelerationZ2 = sequence
                .getBeforeMeanSpecificForceZ();

        assertEquals(accelerationZ1.getValue().doubleValue(),
                beforeMeanFz, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationZ1, accelerationZ2);

        final Acceleration accelerationX3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceX(accelerationX3);
        final Acceleration accelerationX4 = sequence
                .getAfterMeanSpecificForceX();

        assertEquals(accelerationX3.getValue().doubleValue(),
                afterMeanFx, 0.0);
        assertEquals(accelerationX3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationX3, accelerationX4);

        final Acceleration accelerationY3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceY(accelerationY3);
        final Acceleration accelerationY4 = sequence
                .getAfterMeanSpecificForceY();

        assertEquals(accelerationY3.getValue().doubleValue(),
                afterMeanFy, 0.0);
        assertEquals(accelerationY3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationY3, accelerationY4);

        final Acceleration accelerationZ3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceZ(accelerationZ3);
        final Acceleration accelerationZ4 = sequence
                .getAfterMeanSpecificForceZ();

        assertEquals(accelerationZ3.getValue().doubleValue(),
                afterMeanFz, 0.0);
        assertEquals(accelerationZ3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationZ3, accelerationZ4);
    }

    @Test
    public void testConstructor4() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        final Acceleration beforeMeanSpecificForceX =
                new Acceleration(beforeMeanFx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration beforeMeanSpecificForceY =
                new Acceleration(beforeMeanFy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration beforeMeanSpecificForceZ =
                new Acceleration(beforeMeanFz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration afterMeanSpecificForceX =
                new Acceleration(afterMeanFx,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration afterMeanSpecificForceY =
                new Acceleration(afterMeanFy,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration afterMeanSpecificForceZ =
                new Acceleration(afterMeanFz,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>(
                        beforeMeanSpecificForceX,
                        beforeMeanSpecificForceY,
                        beforeMeanSpecificForceZ,
                        afterMeanSpecificForceX,
                        afterMeanSpecificForceY,
                        afterMeanSpecificForceZ);

        // check default values
        assertFalse(sequence.getSortedItems(null));
        assertNull(sequence.getSortedItems());
        assertEquals(sequence.getItemsCount(), 0);
        assertEquals(sequence.getBeforeMeanFx(), beforeMeanFx, 0.0);
        assertEquals(sequence.getBeforeMeanFy(), beforeMeanFy, 0.0);
        assertEquals(sequence.getBeforeMeanFz(), beforeMeanFz, 0.0);
        assertEquals(sequence.getAfterMeanFx(), afterMeanFx, 0.0);
        assertEquals(sequence.getAfterMeanFy(), afterMeanFy, 0.0);
        assertEquals(sequence.getAfterMeanFz(), afterMeanFz, 0.0);

        final Acceleration accelerationX1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(accelerationX1);
        final Acceleration accelerationX2 = sequence
                .getBeforeMeanSpecificForceX();

        assertEquals(accelerationX1.getValue().doubleValue(),
                beforeMeanFx, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationX1, accelerationX2);

        final Acceleration accelerationY1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceY(accelerationY1);
        final Acceleration accelerationY2 = sequence
                .getBeforeMeanSpecificForceY();

        assertEquals(accelerationY1.getValue().doubleValue(),
                beforeMeanFy, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationY1, accelerationY2);

        final Acceleration accelerationZ1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceZ(accelerationZ1);
        final Acceleration accelerationZ2 = sequence
                .getBeforeMeanSpecificForceZ();

        assertEquals(accelerationZ1.getValue().doubleValue(),
                beforeMeanFz, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationZ1, accelerationZ2);

        final Acceleration accelerationX3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceX(accelerationX3);
        final Acceleration accelerationX4 = sequence
                .getAfterMeanSpecificForceX();

        assertEquals(accelerationX3.getValue().doubleValue(),
                afterMeanFx, 0.0);
        assertEquals(accelerationX3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationX3, accelerationX4);

        final Acceleration accelerationY3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceY(accelerationY3);
        final Acceleration accelerationY4 = sequence
                .getAfterMeanSpecificForceY();

        assertEquals(accelerationY3.getValue().doubleValue(),
                afterMeanFy, 0.0);
        assertEquals(accelerationY3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationY3, accelerationY4);

        final Acceleration accelerationZ3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceZ(accelerationZ3);
        final Acceleration accelerationZ4 = sequence
                .getAfterMeanSpecificForceZ();

        assertEquals(accelerationZ3.getValue().doubleValue(),
                afterMeanFz, 0.0);
        assertEquals(accelerationZ3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationZ3, accelerationZ4);
    }

    @Test
    public void testConstructor5() {
        final List<StandardDeviationTimedBodyKinematics> items = createItems();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>(items,
                        beforeMeanFx, beforeMeanFy, beforeMeanFz,
                        afterMeanFx, afterMeanFy, afterMeanFz);

        // check default values
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

        assertEquals(sequence.getBeforeMeanFx(), beforeMeanFx, 0.0);
        assertEquals(sequence.getBeforeMeanFy(), beforeMeanFy, 0.0);
        assertEquals(sequence.getBeforeMeanFz(), beforeMeanFz, 0.0);
        assertEquals(sequence.getAfterMeanFx(), afterMeanFx, 0.0);
        assertEquals(sequence.getAfterMeanFy(), afterMeanFy, 0.0);
        assertEquals(sequence.getAfterMeanFz(), afterMeanFz, 0.0);

        final Acceleration accelerationX1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(accelerationX1);
        final Acceleration accelerationX2 = sequence
                .getBeforeMeanSpecificForceX();

        assertEquals(accelerationX1.getValue().doubleValue(),
                beforeMeanFx, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationX1, accelerationX2);

        final Acceleration accelerationY1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceY(accelerationY1);
        final Acceleration accelerationY2 = sequence
                .getBeforeMeanSpecificForceY();

        assertEquals(accelerationY1.getValue().doubleValue(),
                beforeMeanFy, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationY1, accelerationY2);

        final Acceleration accelerationZ1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceZ(accelerationZ1);
        final Acceleration accelerationZ2 = sequence
                .getBeforeMeanSpecificForceZ();

        assertEquals(accelerationZ1.getValue().doubleValue(),
                beforeMeanFz, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationZ1, accelerationZ2);

        final Acceleration accelerationX3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceX(accelerationX3);
        final Acceleration accelerationX4 = sequence
                .getAfterMeanSpecificForceX();

        assertEquals(accelerationX3.getValue().doubleValue(),
                afterMeanFx, 0.0);
        assertEquals(accelerationX3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationX3, accelerationX4);

        final Acceleration accelerationY3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceY(accelerationY3);
        final Acceleration accelerationY4 = sequence
                .getAfterMeanSpecificForceY();

        assertEquals(accelerationY3.getValue().doubleValue(),
                afterMeanFy, 0.0);
        assertEquals(accelerationY3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationY3, accelerationY4);

        final Acceleration accelerationZ3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceZ(accelerationZ3);
        final Acceleration accelerationZ4 = sequence
                .getAfterMeanSpecificForceZ();

        assertEquals(accelerationZ3.getValue().doubleValue(),
                afterMeanFz, 0.0);
        assertEquals(accelerationZ3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationZ3, accelerationZ4);
    }

    @Test
    public void testConstructor6() {
        final List<StandardDeviationTimedBodyKinematics> items = createItems();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        final Acceleration beforeMeanSpecificForceX =
                new Acceleration(beforeMeanFx,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration beforeMeanSpecificForceY =
                new Acceleration(beforeMeanFy,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration beforeMeanSpecificForceZ =
                new Acceleration(beforeMeanFz,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration afterMeanSpecificForceX =
                new Acceleration(afterMeanFx,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration afterMeanSpecificForceY =
                new Acceleration(afterMeanFy,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration afterMeanSpecificForceZ =
                new Acceleration(afterMeanFz,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>(items,
                        beforeMeanSpecificForceX,
                        beforeMeanSpecificForceY,
                        beforeMeanSpecificForceZ,
                        afterMeanSpecificForceX,
                        afterMeanSpecificForceY,
                        afterMeanSpecificForceZ);

        // check default values
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

        assertEquals(sequence.getBeforeMeanFx(), beforeMeanFx, 0.0);
        assertEquals(sequence.getBeforeMeanFy(), beforeMeanFy, 0.0);
        assertEquals(sequence.getBeforeMeanFz(), beforeMeanFz, 0.0);
        assertEquals(sequence.getAfterMeanFx(), afterMeanFx, 0.0);
        assertEquals(sequence.getAfterMeanFy(), afterMeanFy, 0.0);
        assertEquals(sequence.getAfterMeanFz(), afterMeanFz, 0.0);

        final Acceleration accelerationX1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(accelerationX1);
        final Acceleration accelerationX2 = sequence
                .getBeforeMeanSpecificForceX();

        assertEquals(accelerationX1.getValue().doubleValue(),
                beforeMeanFx, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationX1, accelerationX2);

        final Acceleration accelerationY1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceY(accelerationY1);
        final Acceleration accelerationY2 = sequence
                .getBeforeMeanSpecificForceY();

        assertEquals(accelerationY1.getValue().doubleValue(),
                beforeMeanFy, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationY1, accelerationY2);

        final Acceleration accelerationZ1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceZ(accelerationZ1);
        final Acceleration accelerationZ2 = sequence
                .getBeforeMeanSpecificForceZ();

        assertEquals(accelerationZ1.getValue().doubleValue(),
                beforeMeanFz, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationZ1, accelerationZ2);

        final Acceleration accelerationX3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceX(accelerationX3);
        final Acceleration accelerationX4 = sequence
                .getAfterMeanSpecificForceX();

        assertEquals(accelerationX3.getValue().doubleValue(),
                afterMeanFx, 0.0);
        assertEquals(accelerationX3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationX3, accelerationX4);

        final Acceleration accelerationY3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceY(accelerationY3);
        final Acceleration accelerationY4 = sequence
                .getAfterMeanSpecificForceY();

        assertEquals(accelerationY3.getValue().doubleValue(),
                afterMeanFy, 0.0);
        assertEquals(accelerationY3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationY3, accelerationY4);

        final Acceleration accelerationZ3 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceZ(accelerationZ3);
        final Acceleration accelerationZ4 = sequence
                .getAfterMeanSpecificForceZ();

        assertEquals(accelerationZ3.getValue().doubleValue(),
                afterMeanFz, 0.0);
        assertEquals(accelerationZ3.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(accelerationZ3, accelerationZ4);
    }

    @Test
    public void testConstructor7() {
        final List<StandardDeviationTimedBodyKinematics> items = createItems();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence1 =
                new BodyKinematicsSequence2<>(items,
                        beforeMeanFx, beforeMeanFy, beforeMeanFz,
                        afterMeanFx, afterMeanFy, afterMeanFz);

        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence2 =
                new BodyKinematicsSequence2<>(sequence1);


        // check default values
        assertEquals(sequence1.getSortedItems(), sequence2.getSortedItems());
        assertEquals(sequence2.getBeforeMeanFx(), beforeMeanFx, 0.0);
        assertEquals(sequence2.getBeforeMeanFy(), beforeMeanFy, 0.0);
        assertEquals(sequence2.getBeforeMeanFz(), beforeMeanFz, 0.0);
        assertEquals(sequence2.getAfterMeanFx(), afterMeanFx, 0.0);
        assertEquals(sequence2.getAfterMeanFy(), afterMeanFy, 0.0);
        assertEquals(sequence2.getAfterMeanFz(), afterMeanFz, 0.0);

        assertEquals(sequence1, sequence2);
    }

    @Test
    public void testSetItems() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        assertFalse(sequence.getSortedItems(null));
        assertNull(sequence.getSortedItems());

        // set items
        final List<StandardDeviationTimedBodyKinematics> items = createItems();
        sequence.setItems(items);

        // check
        final List<StandardDeviationTimedBodyKinematics> sorted1 = sequence
                .getSortedItems();
        final List<StandardDeviationTimedBodyKinematics> sorted2 =
                new ArrayList<>();
        assertTrue(sequence.getSortedItems(sorted2));

        assertEquals(sorted1.size(), 2);
        assertTrue(sorted1.get(0).getTimestampSeconds()
                < sorted1.get(1).getTimestampSeconds());
        assertEquals(sorted1, sorted2);
        assertEquals(sequence.getItemsCount(), 2);
    }

    @Test
    public void testGetSetBeforeMeanFx() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        // check default value
        assertEquals(sequence.getBeforeMeanFx(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double beforeMeanFx = randomizer.nextDouble(
                MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        sequence.setBeforeMeanFx(beforeMeanFx);

        // check
        assertEquals(sequence.getBeforeMeanFx(), beforeMeanFx, 0.0);
    }

    @Test
    public void testGetSetBeforeMeanFy() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        // check default value
        assertEquals(sequence.getBeforeMeanFy(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double beforeMeanFy = randomizer.nextDouble(
                MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        sequence.setBeforeMeanFy(beforeMeanFy);

        // check
        assertEquals(sequence.getBeforeMeanFy(), beforeMeanFy, 0.0);
    }

    @Test
    public void testGetSetBeforeMeanFz() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        // check default value
        assertEquals(sequence.getBeforeMeanFz(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double beforeMeanFz = randomizer.nextDouble(
                MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        sequence.setBeforeMeanFz(beforeMeanFz);

        // check
        assertEquals(sequence.getBeforeMeanFz(), beforeMeanFz, 0.0);
    }

    @Test
    public void testSetBeforeMeanSpecificForceCoordinates1() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        // check default value
        assertEquals(sequence.getBeforeMeanFx(), 0.0, 0.0);
        assertEquals(sequence.getBeforeMeanFy(), 0.0, 0.0);
        assertEquals(sequence.getBeforeMeanFz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double beforeMeanFx = randomizer.nextDouble(
                MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFy = randomizer.nextDouble(
                MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFz = randomizer.nextDouble(
                MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        sequence.setBeforeMeanSpecificForceCoordinates(
                beforeMeanFx, beforeMeanFy, beforeMeanFz);

        // check
        assertEquals(sequence.getBeforeMeanFx(), beforeMeanFx, 0.0);
        assertEquals(sequence.getBeforeMeanFy(), beforeMeanFy, 0.0);
        assertEquals(sequence.getBeforeMeanFz(), beforeMeanFz, 0.0);
    }

    @Test
    public void testGetSetBeforeMeanSpecificForceX() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        // check default value
        final Acceleration fx1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(fx1);
        final Acceleration fx2 = sequence.getBeforeMeanSpecificForceX();

        assertEquals(fx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(fx1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(fx1, fx2);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double beforeMeanFx = randomizer.nextDouble(
                MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final Acceleration fx3 = new Acceleration(beforeMeanFx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        sequence.setBeforeMeanSpecificForceX(fx3);

        // check
        final Acceleration fx4 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(fx4);
        final Acceleration fx5 = sequence.getBeforeMeanSpecificForceX();

        assertEquals(fx3, fx4);
        assertEquals(fx3, fx5);
    }

    @Test
    public void testGetSetBeforeMeanSpecificForceY() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        // check default value
        final Acceleration fy1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceY(fy1);
        final Acceleration fy2 = sequence.getBeforeMeanSpecificForceY();

        assertEquals(fy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(fy1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(fy1, fy2);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double meanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final Acceleration fy3 = new Acceleration(meanFy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        sequence.setBeforeMeanSpecificForceY(fy3);

        // check
        final Acceleration fy4 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceY(fy4);
        final Acceleration fy5 = sequence.getBeforeMeanSpecificForceY();

        assertEquals(fy3, fy4);
        assertEquals(fy3, fy5);
    }

    @Test
    public void testGetSetBeforeMeanSpecificForceZ() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        // check default value
        final Acceleration fz1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceZ(fz1);
        final Acceleration fz2 = sequence.getBeforeMeanSpecificForceZ();

        assertEquals(fz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(fz1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(fz1, fz2);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double meanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final Acceleration fz3 = new Acceleration(meanFz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        sequence.setBeforeMeanSpecificForceZ(fz3);

        // check
        final Acceleration fz4 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceZ(fz4);
        final Acceleration fz5 = sequence.getBeforeMeanSpecificForceZ();

        assertEquals(fz3, fz4);
        assertEquals(fz3, fz5);
    }

    @Test
    public void testSetBeforeMeanSpecificForceCoordinates2() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        // check default value
        assertEquals(sequence.getBeforeMeanFx(), 0.0, 0.0);
        assertEquals(sequence.getBeforeMeanFy(), 0.0, 0.0);
        assertEquals(sequence.getBeforeMeanFz(), 0.0, 0.0);
        assertEquals(sequence.getAfterMeanFx(), 0.0, 0.0);
        assertEquals(sequence.getAfterMeanFy(), 0.0, 0.0);
        assertEquals(sequence.getAfterMeanFz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        final Acceleration beforeMeanSpecificForceX =
                new Acceleration(beforeMeanFx,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration beforeMeanSpecificForceY =
                new Acceleration(beforeMeanFy,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration beforeMeanSpecificForceZ =
                new Acceleration(beforeMeanFz,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);

        sequence.setBeforeMeanSpecificForceCoordinates(
                beforeMeanSpecificForceX,
                beforeMeanSpecificForceY,
                beforeMeanSpecificForceZ);

        // check
        assertEquals(sequence.getBeforeMeanFx(), beforeMeanFx, 0.0);
        assertEquals(sequence.getBeforeMeanFy(), beforeMeanFy, 0.0);
        assertEquals(sequence.getBeforeMeanFz(), beforeMeanFz, 0.0);
    }

    @Test
    public void testGetSetAfterMeanFx() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        // check default value
        assertEquals(sequence.getAfterMeanFx(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double afterMeanFx = randomizer.nextDouble(
                MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        sequence.setAfterMeanFx(afterMeanFx);

        // check
        assertEquals(sequence.getAfterMeanFx(), afterMeanFx, 0.0);
    }

    @Test
    public void testGetSetAfterMeanFy() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        // check default value
        assertEquals(sequence.getAfterMeanFy(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double afterMeanFy = randomizer.nextDouble(
                MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        sequence.setAfterMeanFy(afterMeanFy);

        // check
        assertEquals(sequence.getAfterMeanFy(), afterMeanFy, 0.0);
    }

    @Test
    public void testGetSetAfterMeanFz() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        // check default value
        assertEquals(sequence.getAfterMeanFz(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double afterMeanFz = randomizer.nextDouble(
                MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        sequence.setAfterMeanFz(afterMeanFz);

        // check
        assertEquals(sequence.getAfterMeanFz(), afterMeanFz, 0.0);
    }

    @Test
    public void testSetAfterMeanSpecificForceCoordinates1() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        // check default value
        assertEquals(sequence.getAfterMeanFx(), 0.0, 0.0);
        assertEquals(sequence.getAfterMeanFy(), 0.0, 0.0);
        assertEquals(sequence.getAfterMeanFz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double afterMeanFx = randomizer.nextDouble(
                MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFy = randomizer.nextDouble(
                MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFz = randomizer.nextDouble(
                MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        sequence.setAfterMeanSpecificForceCoordinates(
                afterMeanFx, afterMeanFy, afterMeanFz);

        // check
        assertEquals(sequence.getAfterMeanFx(), afterMeanFx, 0.0);
        assertEquals(sequence.getAfterMeanFy(), afterMeanFy, 0.0);
        assertEquals(sequence.getAfterMeanFz(), afterMeanFz, 0.0);
    }

    @Test
    public void testGetSetAfterMeanSpecificForceX() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        // check default value
        final Acceleration fx1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceX(fx1);
        final Acceleration fx2 = sequence.getAfterMeanSpecificForceX();

        assertEquals(fx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(fx1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(fx1, fx2);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double afterMeanFx = randomizer.nextDouble(
                MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final Acceleration fx3 = new Acceleration(afterMeanFx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        sequence.setBeforeMeanSpecificForceX(fx3);

        // check
        final Acceleration fx4 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(fx4);
        final Acceleration fx5 = sequence.getBeforeMeanSpecificForceX();

        assertEquals(fx3, fx4);
        assertEquals(fx3, fx5);
    }

    @Test
    public void testGetSetAfterMeanSpecificForceY() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        // check default value
        final Acceleration fy1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceY(fy1);
        final Acceleration fy2 = sequence.getAfterMeanSpecificForceY();

        assertEquals(fy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(fy1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(fy1, fy2);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double meanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final Acceleration fy3 = new Acceleration(meanFy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        sequence.setAfterMeanSpecificForceY(fy3);

        // check
        final Acceleration fy4 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceY(fy4);
        final Acceleration fy5 = sequence.getAfterMeanSpecificForceY();

        assertEquals(fy3, fy4);
        assertEquals(fy3, fy5);
    }

    @Test
    public void testGetSetAfterMeanSpecificForceZ() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        // check default value
        final Acceleration fz1 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceZ(fz1);
        final Acceleration fz2 = sequence.getAfterMeanSpecificForceZ();

        assertEquals(fz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(fz1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(fz1, fz2);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double meanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final Acceleration fz3 = new Acceleration(meanFz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        sequence.setAfterMeanSpecificForceZ(fz3);

        // check
        final Acceleration fz4 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceZ(fz4);
        final Acceleration fz5 = sequence.getAfterMeanSpecificForceZ();

        assertEquals(fz3, fz4);
        assertEquals(fz3, fz5);
    }

    @Test
    public void testSetAfterMeanSpecificForceCoordinates2() {
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence2<>();

        // check default value
        assertEquals(sequence.getBeforeMeanFx(), 0.0, 0.0);
        assertEquals(sequence.getBeforeMeanFy(), 0.0, 0.0);
        assertEquals(sequence.getBeforeMeanFz(), 0.0, 0.0);
        assertEquals(sequence.getAfterMeanFx(), 0.0, 0.0);
        assertEquals(sequence.getAfterMeanFy(), 0.0, 0.0);
        assertEquals(sequence.getAfterMeanFz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        final Acceleration afterMeanSpecificForceX =
                new Acceleration(afterMeanFx,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration afterMeanSpecificForceY =
                new Acceleration(afterMeanFy,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration afterMeanSpecificForceZ =
                new Acceleration(afterMeanFz,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);

        sequence.setAfterMeanSpecificForceCoordinates(
                afterMeanSpecificForceX,
                afterMeanSpecificForceY,
                afterMeanSpecificForceZ);

        // check
        assertEquals(sequence.getAfterMeanFx(), afterMeanFx, 0.0);
        assertEquals(sequence.getAfterMeanFy(), afterMeanFy, 0.0);
        assertEquals(sequence.getAfterMeanFz(), afterMeanFz, 0.0);
    }

    @Test
    public void testCopyFrom1() {
        final List<StandardDeviationTimedBodyKinematics> items = createItems();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence1 =
                new BodyKinematicsSequence2<>(items,
                        beforeMeanFx, beforeMeanFy, beforeMeanFz,
                        afterMeanFx, afterMeanFy, afterMeanFz);

        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence2 =
                new BodyKinematicsSequence2<>();

        sequence2.copyFrom(sequence1);

        assertEquals(sequence1, sequence2);

        assertEquals(sequence2.getSortedItems().size(), items.size());
        assertEquals(sequence2.getBeforeMeanFx(), beforeMeanFx, 0.0);
        assertEquals(sequence2.getBeforeMeanFy(), beforeMeanFy, 0.0);
        assertEquals(sequence2.getBeforeMeanFz(), beforeMeanFz, 0.0);
        assertEquals(sequence2.getAfterMeanFx(), afterMeanFx, 0.0);
        assertEquals(sequence2.getAfterMeanFy(), afterMeanFy, 0.0);
        assertEquals(sequence2.getAfterMeanFz(), afterMeanFz, 0.0);
    }

    @Test
    public void testCopyFrom2() {
        final List<StandardDeviationTimedBodyKinematics> items = createItems();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence1 =
                new BodyKinematicsSequence2<>(items,
                        beforeMeanFx, beforeMeanFy, beforeMeanFz,
                        afterMeanFx, afterMeanFy, afterMeanFz);

        assertNotNull(sequence1.getSortedItems());

        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence2 =
                new BodyKinematicsSequence2<>();

        sequence2.copyFrom(sequence1);

        assertEquals(sequence1, sequence2);

        assertEquals(sequence2.getSortedItems().size(), items.size());
        assertEquals(sequence2.getBeforeMeanFx(), beforeMeanFx, 0.0);
        assertEquals(sequence2.getBeforeMeanFy(), beforeMeanFy, 0.0);
        assertEquals(sequence2.getBeforeMeanFz(), beforeMeanFz, 0.0);
        assertEquals(sequence2.getAfterMeanFx(), afterMeanFx, 0.0);
        assertEquals(sequence2.getAfterMeanFy(), afterMeanFy, 0.0);
        assertEquals(sequence2.getAfterMeanFz(), afterMeanFz, 0.0);
    }

    @Test
    public void testCopyFrom3() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence1 =
                new BodyKinematicsSequence2<>(
                        beforeMeanFx, beforeMeanFy, beforeMeanFz,
                        afterMeanFx, afterMeanFy, afterMeanFz);

        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence2 =
                new BodyKinematicsSequence2<>();

        sequence2.copyFrom(sequence1);

        assertEquals(sequence1, sequence2);

        assertNull(sequence2.getSortedItems());
        assertEquals(sequence2.getBeforeMeanFx(), beforeMeanFx, 0.0);
        assertEquals(sequence2.getBeforeMeanFy(), beforeMeanFy, 0.0);
        assertEquals(sequence2.getBeforeMeanFz(), beforeMeanFz, 0.0);
        assertEquals(sequence2.getAfterMeanFx(), afterMeanFx, 0.0);
        assertEquals(sequence2.getAfterMeanFy(), afterMeanFy, 0.0);
        assertEquals(sequence2.getAfterMeanFz(), afterMeanFz, 0.0);
    }

    @Test
    public void testCopyTo() {
        final List<StandardDeviationTimedBodyKinematics> items = createItems();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence1 =
                new BodyKinematicsSequence2<>(items,
                        beforeMeanFx, beforeMeanFy, beforeMeanFz,
                        afterMeanFx, afterMeanFy, afterMeanFz);

        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence2 =
                new BodyKinematicsSequence2<>();

        sequence1.copyTo(sequence2);

        assertEquals(sequence1, sequence2);

        assertEquals(sequence2.getSortedItems().size(), items.size());
        assertEquals(sequence2.getBeforeMeanFx(), beforeMeanFx, 0.0);
        assertEquals(sequence2.getBeforeMeanFy(), beforeMeanFy, 0.0);
        assertEquals(sequence2.getBeforeMeanFz(), beforeMeanFz, 0.0);
        assertEquals(sequence2.getAfterMeanFx(), afterMeanFx, 0.0);
        assertEquals(sequence2.getAfterMeanFy(), afterMeanFy, 0.0);
        assertEquals(sequence2.getAfterMeanFz(), afterMeanFz, 0.0);
    }

    @Test
    public void testEqualsAndHashCode() {
        final List<StandardDeviationTimedBodyKinematics> items = createItems();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence1 =
                new BodyKinematicsSequence2<>(items,
                        beforeMeanFx, beforeMeanFy, beforeMeanFz,
                        afterMeanFx, afterMeanFy, afterMeanFz);
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence2 =
                new BodyKinematicsSequence2<>(sequence1);
        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence3 =
                new BodyKinematicsSequence2<>();

        assertEquals(sequence1, sequence2);
        assertNotEquals(sequence1, sequence3);

        assertEquals(sequence1.hashCode(), sequence2.hashCode());
        assertNotEquals(sequence1.hashCode(), sequence3.hashCode());
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final List<StandardDeviationTimedBodyKinematics> items = createItems();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);
        final double afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE,
                MAX_SPECIFIC_FORCE);

        final BodyKinematicsSequence2<StandardDeviationTimedBodyKinematics> sequence1 =
                new BodyKinematicsSequence2<>(items,
                        beforeMeanFx, beforeMeanFy, beforeMeanFz,
                        afterMeanFx, afterMeanFy, afterMeanFz);
        final Object sequence2 = sequence1.clone();

        assertEquals(sequence1, sequence2);
    }

    private static List<StandardDeviationTimedBodyKinematics> createItems() {
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

    private static BodyKinematics createBodyKinematics() {
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
