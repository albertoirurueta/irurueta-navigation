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

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Sphere;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;
import static org.mockito.Mockito.mock;

public class HomogeneousLinearLeastSquaresLateration3DSolverTest implements LaterationSolverListener<Point3D> {

    private static final int MIN_SPHERES = 4;
    private static final int MAX_SPHERES = 10;

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
        HomogeneousLinearLeastSquaresLateration3DSolver solver =
                new HomogeneousLinearLeastSquaresLateration3DSolver();

        // check correctness
        assertNull(solver.getListener());
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(3, solver.getNumberOfDimensions());
        assertNull(solver.getSpheres());
        assertEquals(LaterationSolverType.HOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());

        // constructor with positions and distances
        final Point3D[] positions = new Point3D[4];
        positions[0] = new InhomogeneousPoint3D();
        positions[1] = new InhomogeneousPoint3D();
        positions[2] = new InhomogeneousPoint3D();
        positions[3] = new InhomogeneousPoint3D();
        final double[] distances = new double[4];
        solver = new HomogeneousLinearLeastSquaresLateration3DSolver(positions, distances);

        // check correctness
        assertNull(solver.getListener());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(3, solver.getNumberOfDimensions());
        assertNotNull(solver.getSpheres());
        assertEquals(LaterationSolverType.HOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        final double[] wrong = new double[5];
        final Point3D[] shortPositions = new Point3D[1];
        final double[] shortDistances = new double[1];
        solver = null;
        try {
            solver = new HomogeneousLinearLeastSquaresLateration3DSolver(null, distances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new HomogeneousLinearLeastSquaresLateration3DSolver(positions, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new HomogeneousLinearLeastSquaresLateration3DSolver(positions, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new HomogeneousLinearLeastSquaresLateration3DSolver(shortPositions, shortDistances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);

        // constructor with listener
        // noinspection unchecked
        final LaterationSolverListener<Point3D> listener = mock(LaterationSolverListener.class);
        solver = new HomogeneousLinearLeastSquaresLateration3DSolver(listener);

        // check correctness
        assertSame(listener, solver.getListener());
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(3, solver.getNumberOfDimensions());
        assertNull(solver.getSpheres());
        assertEquals(LaterationSolverType.HOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());

        // constructor with positions, distances and listener
        solver = new HomogeneousLinearLeastSquaresLateration3DSolver(positions, distances, listener);

        // check correctness
        assertSame(listener, solver.getListener());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(3, solver.getNumberOfDimensions());
        assertNotNull(solver.getSpheres());
        assertEquals(LaterationSolverType.HOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new HomogeneousLinearLeastSquaresLateration3DSolver(null, distances, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new HomogeneousLinearLeastSquaresLateration3DSolver(positions, null, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new HomogeneousLinearLeastSquaresLateration3DSolver(positions, wrong, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new HomogeneousLinearLeastSquaresLateration3DSolver(shortPositions, shortDistances,
                    listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);

        // constructor with spheres
        final Sphere[] spheres = new Sphere[4];
        spheres[0] = new Sphere(positions[0], distances[0]);
        spheres[1] = new Sphere(positions[1], distances[1]);
        spheres[2] = new Sphere(positions[1], distances[1]);
        spheres[3] = new Sphere(positions[1], distances[1]);
        solver = new HomogeneousLinearLeastSquaresLateration3DSolver(spheres);

        // check correctness
        assertNull(solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(3, solver.getNumberOfDimensions());
        assertNotNull(solver.getSpheres());
        assertEquals(LaterationSolverType.HOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        final Sphere[] shortSpheres = new Sphere[1];

        solver = null;
        try {
            solver = new HomogeneousLinearLeastSquaresLateration3DSolver((Sphere[]) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new HomogeneousLinearLeastSquaresLateration3DSolver(shortSpheres);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);

        // constructor with spheres and listener
        solver = new HomogeneousLinearLeastSquaresLateration3DSolver(spheres, listener);

        // check correctness
        assertSame(listener, solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(3, solver.getNumberOfDimensions());
        assertNotNull(solver.getSpheres());
        assertEquals(LaterationSolverType.HOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new HomogeneousLinearLeastSquaresLateration3DSolver(null, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new HomogeneousLinearLeastSquaresLateration3DSolver(shortSpheres, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);
    }

    @Test
    public void testGetSetSpheres() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final HomogeneousLinearLeastSquaresLateration3DSolver solver =
                new HomogeneousLinearLeastSquaresLateration3DSolver();

        // initial value
        assertNull(solver.getSpheres());

        // set new value
        final Point3D[] positions = new Point3D[4];
        positions[0] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        positions[1] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        positions[2] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        positions[3] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        final double[] distances = new double[4];
        distances[0] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[1] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[2] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[3] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);

        final Sphere[] spheres = new Sphere[4];
        spheres[0] = new Sphere(positions[0], distances[0]);
        spheres[1] = new Sphere(positions[1], distances[1]);
        spheres[2] = new Sphere(positions[2], distances[2]);
        spheres[3] = new Sphere(positions[3], distances[3]);

        solver.setSpheres(spheres);

        // check
        final Sphere[] spheres2 = solver.getSpheres();
        for (int i = 0; i < 4; i++) {
            assertSame(spheres[i].getCenter(), spheres2[i].getCenter());
            assertEquals(spheres[i].getRadius(), spheres2[i].getRadius(), 0.0);
        }

        // force IllegalArgumentException
        try {
            solver.setSpheres(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setSpheres(new Sphere[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final HomogeneousLinearLeastSquaresLateration3DSolver solver =
                new HomogeneousLinearLeastSquaresLateration3DSolver();

        // initial value
        assertNull(solver.getListener());

        // set new value
        // noinspection unchecked
        final LaterationSolverListener<Point3D> listener = mock(LaterationSolverListener.class);
        solver.setListener(listener);

        // check
        assertSame(listener, solver.getListener());
    }

    @Test
    public void testGetSetPositionsAndDistances() throws LockedException {
        final HomogeneousLinearLeastSquaresLateration3DSolver solver =
                new HomogeneousLinearLeastSquaresLateration3DSolver();

        // initial value
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());

        // set new values
        final Point3D[] positions = new Point3D[4];
        positions[0] = new InhomogeneousPoint3D();
        positions[1] = new InhomogeneousPoint3D();
        positions[2] = new InhomogeneousPoint3D();
        positions[3] = new InhomogeneousPoint3D();
        final double[] distances = new double[4];

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
            solver.setPositionsAndDistances(positions, new double[3]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setPositionsAndDistances(new Point3D[1], new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSolveNoError() throws LaterationException, NotReadyException, LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius;
            final Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);
                spheres[i] = new Sphere(center, radius);
            }

            final HomogeneousLinearLeastSquaresLateration3DSolver solver =
                    new HomogeneousLinearLeastSquaresLateration3DSolver(spheres, this);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            final Point3D estimatedPosition = solver.getEstimatedPosition();
            if (estimatedPosition.distanceTo(position) > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // Force NotReadyException
        final HomogeneousLinearLeastSquaresLateration3DSolver solver =
                new HomogeneousLinearLeastSquaresLateration3DSolver();
        try {
            solver.solve();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) {
        }

        // Force LaterationException
        final Sphere[] circles = new Sphere[4];
        InhomogeneousPoint3D center;
        double radius;
        for (int i = 0; i < 4; i++) {
            center = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            radius = LaterationSolver.EPSILON;
            circles[i] = new Sphere(center, radius);
        }
        solver.setSpheres(circles);
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
            final int numPoints = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D point;
            double distance, error;
            final Point3D[] positions = new Point3D[numPoints];
            final double[] distances = new double[numPoints];
            for (int i = 0; i < numPoints; i++) {
                point = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                distance = point.distanceTo(position);
                error = randomizer.nextDouble(MIN_DISTANCE_ERROR, MAX_DISTANCE_ERROR);
                positions[i] = point;
                distances[i] = distance + error;
            }

            final HomogeneousLinearLeastSquaresLateration3DSolver solver =
                    new HomogeneousLinearLeastSquaresLateration3DSolver(positions, distances, this);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            final Point3D estimatedPosition = solver.getEstimatedPosition();
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
    public void testSolve4SpheresNoError() throws LaterationException, NotReadyException, LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numSpheres = MIN_SPHERES;

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius;
            final Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);
                spheres[i] = new Sphere(center, radius);
            }

            final HomogeneousLinearLeastSquaresLateration3DSolver solver =
                    new HomogeneousLinearLeastSquaresLateration3DSolver(spheres, this);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            final Point3D estimatedPosition = solver.getEstimatedPosition();
            if (estimatedPosition.distanceTo(position) > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // Force NotReadyException
        final HomogeneousLinearLeastSquaresLateration3DSolver solver =
                new HomogeneousLinearLeastSquaresLateration3DSolver();
        try {
            solver.solve();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }

        // Force LaterationException
        final Sphere[] circles = new Sphere[4];
        InhomogeneousPoint3D center;
        double radius;
        for (int i = 0; i < 4; i++) {
            center = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            radius = LaterationSolver.EPSILON;
            circles[i] = new Sphere(center, radius);
        }
        solver.setSpheres(circles);
        try {
            solver.solve();
            fail("LaterationException expected but not thrown");
        } catch (final LaterationException ignore) {
        }
    }

    @Override
    public void onSolveStart(final LaterationSolver<Point3D> solver) {
        solveStart++;
        checkLocked((HomogeneousLinearLeastSquaresLateration3DSolver) solver);
    }

    @Override
    public void onSolveEnd(final LaterationSolver<Point3D> solver) {
        solveEnd++;
        checkLocked((HomogeneousLinearLeastSquaresLateration3DSolver) solver);
    }

    private void reset() {
        solveStart = solveEnd = 0;
    }

    private void checkLocked(final HomogeneousLinearLeastSquaresLateration3DSolver solver) {
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
            solver.setSpheres(null);
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
