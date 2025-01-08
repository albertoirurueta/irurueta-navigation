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
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.mock;

@ExtendWith(MockitoExtension.class)
class InhomogeneousLinearLeastSquaresLateration3DSolverTest implements LaterationSolverListener<Point3D> {

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
    void testConstructor() {
        // empty constructor
        var solver = new InhomogeneousLinearLeastSquaresLateration3DSolver();

        // check correctness
        assertNull(solver.getListener());
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(3, solver.getNumberOfDimensions());
        assertNull(solver.getSpheres());
        assertEquals(LaterationSolverType.INHOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());

        // constructor with positions and distances
        final var positions = new Point3D[4];
        positions[0] = new InhomogeneousPoint3D();
        positions[1] = new InhomogeneousPoint3D();
        positions[2] = new InhomogeneousPoint3D();
        positions[3] = new InhomogeneousPoint3D();
        final var distances = new double[4];
        solver = new InhomogeneousLinearLeastSquaresLateration3DSolver(positions, distances);

        // check correctness
        assertNull(solver.getListener());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(3, solver.getNumberOfDimensions());
        assertNotNull(solver.getSpheres());
        assertEquals(LaterationSolverType.INHOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        final var wrong = new double[5];
        final var shortPositions = new Point3D[1];
        final var shortDistances = new double[1];
        assertThrows(IllegalArgumentException.class,
                () -> new InhomogeneousLinearLeastSquaresLateration3DSolver(null, distances));
        assertThrows(IllegalArgumentException.class,
                () -> new InhomogeneousLinearLeastSquaresLateration3DSolver(positions, null));
        assertThrows(IllegalArgumentException.class,
                () -> new InhomogeneousLinearLeastSquaresLateration3DSolver(positions, wrong));
        assertThrows(IllegalArgumentException.class,
                () -> new InhomogeneousLinearLeastSquaresLateration3DSolver(shortPositions, shortDistances));

        // constructor with listener
        // noinspection unchecked
        final LaterationSolverListener<Point3D> listener = mock(LaterationSolverListener.class);
        solver = new InhomogeneousLinearLeastSquaresLateration3DSolver(listener);

        // check correctness
        assertSame(listener, solver.getListener());
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(3, solver.getNumberOfDimensions());
        assertNull(solver.getSpheres());
        assertEquals(LaterationSolverType.INHOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());

        // constructor with positions, distances and listener
        solver = new InhomogeneousLinearLeastSquaresLateration3DSolver(positions, distances, listener);

        // check correctness
        assertSame(listener, solver.getListener());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(3, solver.getNumberOfDimensions());
        assertNotNull(solver.getSpheres());
        assertEquals(LaterationSolverType.INHOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new InhomogeneousLinearLeastSquaresLateration3DSolver(null, distances, listener));
        assertThrows(IllegalArgumentException.class,
                () -> new InhomogeneousLinearLeastSquaresLateration3DSolver(positions, null, listener));
        assertThrows(IllegalArgumentException.class,
                () -> new InhomogeneousLinearLeastSquaresLateration3DSolver(positions, wrong, listener));
        assertThrows(IllegalArgumentException.class,
                () -> new InhomogeneousLinearLeastSquaresLateration3DSolver(shortPositions, shortDistances, listener));

        // constructor with spheres
        final var spheres = new Sphere[4];
        spheres[0] = new Sphere(positions[0], distances[0]);
        spheres[1] = new Sphere(positions[1], distances[1]);
        spheres[2] = new Sphere(positions[1], distances[1]);
        spheres[3] = new Sphere(positions[1], distances[1]);
        solver = new InhomogeneousLinearLeastSquaresLateration3DSolver(spheres);

        // check correctness
        assertNull(solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(3, solver.getNumberOfDimensions());
        assertNotNull(solver.getSpheres());
        assertEquals(LaterationSolverType.INHOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        final var shortSpheres = new Sphere[1];
        assertThrows(IllegalArgumentException.class,
                () -> new InhomogeneousLinearLeastSquaresLateration3DSolver((Sphere[]) null));
        assertThrows(IllegalArgumentException.class,
                () -> new InhomogeneousLinearLeastSquaresLateration3DSolver(shortSpheres));

        // constructor with spheres and listener
        solver = new InhomogeneousLinearLeastSquaresLateration3DSolver(spheres, listener);

        // check correctness
        assertSame(listener, solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(3, solver.getNumberOfDimensions());
        assertNotNull(solver.getSpheres());
        assertEquals(LaterationSolverType.INHOMOGENEOUS_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new InhomogeneousLinearLeastSquaresLateration3DSolver(null, listener));
        assertThrows(IllegalArgumentException.class,
                () -> new InhomogeneousLinearLeastSquaresLateration3DSolver(shortSpheres, listener));
    }

    @Test
    void testGetSetSpheres() throws LockedException {
        final var randomizer = new UniformRandomizer();

        final var solver = new InhomogeneousLinearLeastSquaresLateration3DSolver();

        // initial value
        assertNull(solver.getSpheres());

        // set new value
        final var positions = new Point3D[4];
        positions[0] = new InhomogeneousPoint3D(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());
        positions[1] = new InhomogeneousPoint3D(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());
        positions[2] = new InhomogeneousPoint3D(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());
        positions[3] = new InhomogeneousPoint3D(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());
        final var distances = new double[4];
        distances[0] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[1] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[2] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[3] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);

        final var spheres = new Sphere[4];
        spheres[0] = new Sphere(positions[0], distances[0]);
        spheres[1] = new Sphere(positions[1], distances[1]);
        spheres[2] = new Sphere(positions[2], distances[2]);
        spheres[3] = new Sphere(positions[3], distances[3]);

        solver.setSpheres(spheres);

        // check
        final var spheres2 = solver.getSpheres();
        for (var i = 0; i < 4; i++) {
            assertSame(spheres[i].getCenter(), spheres2[i].getCenter());
            assertEquals(spheres[i].getRadius(), spheres2[i].getRadius(), 0.0);
        }

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> solver.setSpheres(null));
        assertThrows(IllegalArgumentException.class, () -> solver.setSpheres(new Sphere[1]));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var solver = new InhomogeneousLinearLeastSquaresLateration3DSolver();

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
    void testGetSetPositionsAndDistances() throws LockedException {
        final var solver = new InhomogeneousLinearLeastSquaresLateration3DSolver();

        // initial value
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());

        // set new values
        final var positions = new Point3D[4];
        positions[0] = new InhomogeneousPoint3D();
        positions[1] = new InhomogeneousPoint3D();
        positions[2] = new InhomogeneousPoint3D();
        positions[3] = new InhomogeneousPoint3D();
        final var distances = new double[4];

        solver.setPositionsAndDistances(positions, distances);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> solver.setPositionsAndDistances(null, distances));
        assertThrows(IllegalArgumentException.class, () -> solver.setPositionsAndDistances(positions, null));
        assertThrows(IllegalArgumentException.class, () -> solver.setPositionsAndDistances(positions, new double[3]));
        assertThrows(IllegalArgumentException.class, () -> solver.setPositionsAndDistances(new Point3D[1],
                new double[1]));
    }

    @Test
    void testSolveNoError() throws LaterationException, NotReadyException, LockedException {
        final var randomizer = new UniformRandomizer();

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var spheres = new Sphere[numSpheres];
            for (var i = 0; i < numSpheres; i++) {
                final var center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                final var radius = center.distanceTo(position);
                spheres[i] = new Sphere(center, radius);
            }

            final var solver = new InhomogeneousLinearLeastSquaresLateration3DSolver(spheres, this);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            final var estimatedPosition = solver.getEstimatedPosition();
            if (estimatedPosition.distanceTo(position) > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // Force NotReadyException
        final var solver = new InhomogeneousLinearLeastSquaresLateration3DSolver();
        assertThrows(NotReadyException.class, solver::solve);

        // Force LaterationException
        final var circles = new Sphere[4];
        for (var i = 0; i < 4; i++) {
            final var center = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var radius = LaterationSolver.EPSILON;
            circles[i] = new Sphere(center, radius);
        }
        solver.setSpheres(circles);
        assertThrows(LaterationException.class, solver::solve);
    }

    @Test
    void testSolveWithError() throws LaterationException, NotReadyException, LockedException {
        final var randomizer = new UniformRandomizer();

        var numValid = 0;
        var numInvalid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numPoints = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var positions = new Point3D[numPoints];
            final var distances = new double[numPoints];
            for (var i = 0; i < numPoints; i++) {
                final var point = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                final var distance = point.distanceTo(position);
                final var error = randomizer.nextDouble(MIN_DISTANCE_ERROR, MAX_DISTANCE_ERROR);
                positions[i] = point;
                distances[i] = distance + error;
            }

            final var solver = new InhomogeneousLinearLeastSquaresLateration3DSolver(positions, distances,
                    this);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            final var estimatedPosition = solver.getEstimatedPosition();
            final var distance = estimatedPosition.distanceTo(position);
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
    void testSolve4SpheresNoError() throws LaterationException, NotReadyException, LockedException {
        final var randomizer = new UniformRandomizer();

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSpheres = MIN_SPHERES;

            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var spheres = new Sphere[numSpheres];
            for (var i = 0; i < numSpheres; i++) {
                final var center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                final var radius = center.distanceTo(position);
                spheres[i] = new Sphere(center, radius);
            }

            final var solver = new InhomogeneousLinearLeastSquaresLateration3DSolver(spheres, this);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            final var estimatedPosition = solver.getEstimatedPosition();
            if (estimatedPosition.distanceTo(position) > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // Force NotReadyException
        final var solver = new InhomogeneousLinearLeastSquaresLateration3DSolver();
        assertThrows(NotReadyException.class, solver::solve);

        // Force LaterationException
        final var circles = new Sphere[4];
        for (var i = 0; i < 4; i++) {
            final var center = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var radius = LaterationSolver.EPSILON;
            circles[i] = new Sphere(center, radius);
        }
        solver.setSpheres(circles);
        assertThrows(LaterationException.class, solver::solve);
    }

    @Override
    public void onSolveStart(final LaterationSolver<Point3D> solver) {
        solveStart++;
        checkLocked((InhomogeneousLinearLeastSquaresLateration3DSolver) solver);
    }

    @Override
    public void onSolveEnd(final LaterationSolver<Point3D> solver) {
        solveEnd++;
        checkLocked((InhomogeneousLinearLeastSquaresLateration3DSolver) solver);
    }

    private void reset() {
        solveStart = solveEnd = 0;
    }

    private static void checkLocked(final InhomogeneousLinearLeastSquaresLateration3DSolver solver) {
        assertThrows(LockedException.class, () -> solver.setListener(null));
        assertThrows(LockedException.class, () -> solver.setPositionsAndDistances(null, null));
        assertThrows(LockedException.class, () -> solver.setSpheres(null));
        assertThrows(LockedException.class, solver::solve);
    }
}
