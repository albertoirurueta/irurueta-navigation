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
package com.irurueta.navigation.lateration;

import com.irurueta.geometry.Circle;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;
import static org.mockito.Mockito.mock;

public class InhomogeneousLinearLeastSquaresLateration2DSolverTest implements LaterationSolverListener<Point2D> {

    private static final int MIN_CIRCLES = 3;
    private static final int MAX_CIRCLES = 10;

    private static final double MIN_RANDOM_VALUE = -50.0;
    private static final double MAX_RANDOM_VALUE = 50.0;

    private static final double MIN_DISTANCE_ERROR = -1e-2;
    private static final double MAX_DISTANCE_ERROR = 1e-2;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-1;

    private static final int TIMES = 50;

    private int solveStart;
    private int solveEnd;

    @Test
    public void testConstructor() {
        // empty constructor
        InhomogeneousLinearLeastSquaresLateration2DSolver solver =
                new InhomogeneousLinearLeastSquaresLateration2DSolver();

        // check correctness
        assertNull(solver.getListener());
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNull(solver.getCircles());
        assertEquals(LaterationSolverType.INHOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // constructor with positions and distances
        final Point2D[] positions = new Point2D[3];
        positions[0] = new InhomogeneousPoint2D();
        positions[1] = new InhomogeneousPoint2D();
        positions[2] = new InhomogeneousPoint2D();
        final double[] distances = new double[3];
        solver = new InhomogeneousLinearLeastSquaresLateration2DSolver(positions, distances);

        // check correctness
        assertNull(solver.getListener());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.INHOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        final double[] wrong = new double[4];
        final Point2D[] shortPositions = new Point2D[1];
        final double[] shortDistances = new double[1];
        solver = null;
        try {
            solver = new InhomogeneousLinearLeastSquaresLateration2DSolver(null, distances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new InhomogeneousLinearLeastSquaresLateration2DSolver(positions, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new InhomogeneousLinearLeastSquaresLateration2DSolver(positions, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new InhomogeneousLinearLeastSquaresLateration2DSolver(shortPositions, shortDistances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with listener
        // noinspection unchecked
        final LaterationSolverListener<Point2D> listener = mock(LaterationSolverListener.class);
        solver = new InhomogeneousLinearLeastSquaresLateration2DSolver(listener);

        // check correctness
        assertSame(listener, solver.getListener());
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNull(solver.getCircles());
        assertEquals(LaterationSolverType.INHOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // constructor with positions, distances and listener
        solver = new InhomogeneousLinearLeastSquaresLateration2DSolver(positions, distances, listener);

        // check correctness
        assertSame(listener, solver.getListener());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.INHOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new InhomogeneousLinearLeastSquaresLateration2DSolver(null, distances, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new InhomogeneousLinearLeastSquaresLateration2DSolver(positions, null, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new InhomogeneousLinearLeastSquaresLateration2DSolver(positions, wrong, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new InhomogeneousLinearLeastSquaresLateration2DSolver(shortPositions, shortDistances,
                    listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with circles
        final Circle[] circles = new Circle[3];
        circles[0] = new Circle(positions[0], distances[0]);
        circles[1] = new Circle(positions[1], distances[1]);
        circles[2] = new Circle(positions[2], distances[2]);
        solver = new InhomogeneousLinearLeastSquaresLateration2DSolver(circles);

        // check correctness
        assertNull(solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.INHOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        final Circle[] shortCircles = new Circle[1];

        solver = null;
        try {
            solver = new InhomogeneousLinearLeastSquaresLateration2DSolver((Circle[]) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new InhomogeneousLinearLeastSquaresLateration2DSolver(shortCircles);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with circles and listener
        solver = new InhomogeneousLinearLeastSquaresLateration2DSolver(circles, listener);

        // check correctness
        assertSame(listener, solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.INHOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new InhomogeneousLinearLeastSquaresLateration2DSolver(null, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new InhomogeneousLinearLeastSquaresLateration2DSolver(shortCircles, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);
    }

    @Test
    public void testGetSetCircles() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final InhomogeneousLinearLeastSquaresLateration2DSolver solver =
                new InhomogeneousLinearLeastSquaresLateration2DSolver();

        // initial value
        assertNull(solver.getCircles());

        // set new value
        final Point2D[] positions = new Point2D[3];
        positions[0] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        positions[1] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        positions[2] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        final double[] distances = new double[3];
        distances[0] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[1] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[2] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);

        final Circle[] circles = new Circle[3];
        circles[0] = new Circle(positions[0], distances[0]);
        circles[1] = new Circle(positions[1], distances[1]);
        circles[2] = new Circle(positions[2], distances[2]);

        solver.setCircles(circles);

        // check
        final Circle[] circles2 = solver.getCircles();
        for (int i = 0; i < 3; i++) {
            assertSame(circles[i].getCenter(), circles2[i].getCenter());
            assertEquals(circles[i].getRadius(), circles2[i].getRadius(), 0.0);
        }

        // force IllegalArgumentException
        try {
            solver.setCircles(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setCircles(new Circle[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final InhomogeneousLinearLeastSquaresLateration2DSolver solver =
                new InhomogeneousLinearLeastSquaresLateration2DSolver();

        // initial value
        assertNull(solver.getListener());

        // set new value
        // noinspection unchecked
        final LaterationSolverListener<Point2D> listener = mock(LaterationSolverListener.class);
        solver.setListener(listener);

        // check
        assertSame(listener, solver.getListener());
    }

    @Test
    public void testGetSetPositionsAndDistances() throws LockedException {
        final InhomogeneousLinearLeastSquaresLateration2DSolver solver =
                new InhomogeneousLinearLeastSquaresLateration2DSolver();

        // initial value
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());

        // set new values
        final Point2D[] positions = new Point2D[3];
        positions[0] = new InhomogeneousPoint2D();
        positions[1] = new InhomogeneousPoint2D();
        positions[2] = new InhomogeneousPoint2D();
        final double[] distances = new double[3];

        solver.setPositionsAndDistances(positions, distances);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());

        // Force IllegalArgumentException
        try {
            solver.setPositionsAndDistances(null, distances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setPositionsAndDistances(positions, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setPositionsAndDistances(positions, new double[4]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setPositionsAndDistances(new Point2D[1], new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSolveNoError() throws LaterationException, NotReadyException, LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numCircles = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D center;
            double radius;
            final Circle[] circles = new Circle[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);
                circles[i] = new Circle(center, radius);
            }

            final InhomogeneousLinearLeastSquaresLateration2DSolver solver =
                    new InhomogeneousLinearLeastSquaresLateration2DSolver(circles, this);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            final Point2D estimatedPosition = solver.getEstimatedPosition();
            if (estimatedPosition.distanceTo(position) > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // Force NotReadyException
        final InhomogeneousLinearLeastSquaresLateration2DSolver solver =
                new InhomogeneousLinearLeastSquaresLateration2DSolver();
        try {
            solver.solve();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }

        // Force LaterationException
        final Circle[] circles = new Circle[3];
        InhomogeneousPoint2D center;
        double radius;
        for (int i = 0; i < 3; i++) {
            center = new InhomogeneousPoint2D(0.0, 0.0);
            radius = LaterationSolver.EPSILON;
            circles[i] = new Circle(center, radius);
        }
        solver.setCircles(circles);
        try {
            solver.solve();
            fail("LaterationException expected but not thrown");
        } catch (final LaterationException ignore) {
        }
    }

    @Test
    public void testSolveWithError() throws LaterationException, NotReadyException, LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0, numInvalid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numPoints = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D point;
            double distance, error;
            final Point2D[] positions = new Point2D[numPoints];
            final double[] distances = new double[numPoints];
            for (int i = 0; i < numPoints; i++) {
                point = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                distance = point.distanceTo(position);
                error = randomizer.nextDouble(MIN_DISTANCE_ERROR, MAX_DISTANCE_ERROR);
                positions[i] = point;
                distances[i] = distance + error;
            }

            final InhomogeneousLinearLeastSquaresLateration2DSolver solver =
                    new InhomogeneousLinearLeastSquaresLateration2DSolver(positions, distances, this);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            final Point2D estimatedPosition = solver.getEstimatedPosition();
            distance = estimatedPosition.distanceTo(position);
            if (distance >= LARGE_ABSOLUTE_ERROR) {
                numInvalid++;
            } else {
                numValid++;
                assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
            }
        }

        assertTrue(numValid > numInvalid);
    }

    @Test
    public void testSolve3CirclesNoError() throws LaterationException, NotReadyException, LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numCircles = MIN_CIRCLES;

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D center;
            double radius;
            final Circle[] circles = new Circle[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);
                circles[i] = new Circle(center, radius);
            }

            final InhomogeneousLinearLeastSquaresLateration2DSolver solver =
                    new InhomogeneousLinearLeastSquaresLateration2DSolver(circles, this);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            final Point2D estimatedPosition = solver.getEstimatedPosition();
            if (estimatedPosition.distanceTo(position) > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // Force NotReadyException
        final InhomogeneousLinearLeastSquaresLateration2DSolver solver =
                new InhomogeneousLinearLeastSquaresLateration2DSolver();
        try {
            solver.solve();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }

        // Force LaterationException
        final Circle[] circles = new Circle[3];
        InhomogeneousPoint2D center;
        double radius;
        for (int i = 0; i < 3; i++) {
            center = new InhomogeneousPoint2D(0.0, 0.0);
            radius = LaterationSolver.EPSILON;
            circles[i] = new Circle(center, radius);
        }
        solver.setCircles(circles);
        try {
            solver.solve();
            fail("LaterationException expected but not thrown");
        } catch (final LaterationException ignore) {
        }
    }

    @Override
    public void onSolveStart(final LaterationSolver<Point2D> solver) {
        solveStart++;
        checkLocked((InhomogeneousLinearLeastSquaresLateration2DSolver) solver);
    }

    @Override
    public void onSolveEnd(final LaterationSolver<Point2D> solver) {
        solveEnd++;
        checkLocked((InhomogeneousLinearLeastSquaresLateration2DSolver) solver);
    }

    private void reset() {
        solveStart = solveEnd = 0;
    }

    private void checkLocked(final InhomogeneousLinearLeastSquaresLateration2DSolver solver) {
        try {
            solver.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setPositionsAndDistances(null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setCircles(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.solve();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception ignore) {
            fail("LockedException expected but not thrown");
        }
    }
}
