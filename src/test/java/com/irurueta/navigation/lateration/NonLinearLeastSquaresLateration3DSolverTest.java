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

public class NonLinearLeastSquaresLateration3DSolverTest implements LaterationSolverListener<Point3D> {

    private static final int MIN_SPHERES = 4;
    private static final int MAX_SPHERES = 10;

    private static final double MIN_RANDOM_VALUE = -50.0;
    private static final double MAX_RANDOM_VALUE = 50.0;

    private static final double MIN_POSITION_ERROR = 1e-2;
    private static final double MAX_POSITION_ERROR = 1.0;

    private static final double MIN_DISTANCE_ERROR = -1e-3;
    private static final double MAX_DISTANCE_ERROR = 1e-3;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-1;

    private static final int TIMES = 50;

    private int solveStart;
    private int solveEnd;

    @Test
    public void testConstructor() {
        // empty constructor
        NonLinearLeastSquaresLateration3DSolver solver = new NonLinearLeastSquaresLateration3DSolver();

        // check correctness
        assertNull(solver.getListener());
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);


        // constructor with positions and distances
        final Point3D[] positions = new Point3D[4];
        positions[0] = new InhomogeneousPoint3D();
        positions[1] = new InhomogeneousPoint3D();
        positions[2] = new InhomogeneousPoint3D();
        positions[3] = new InhomogeneousPoint3D();
        final double[] distances = new double[4];
        solver = new NonLinearLeastSquaresLateration3DSolver(positions, distances);

        // check correctness
        assertNull(solver.getListener());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);

        // Force IllegalArgumentException
        final double[] wrong = new double[3];
        final Point3D[] shortPositions = new Point3D[1];
        final double[] shortDistances = new double[1];
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver((Point3D[]) null, distances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(shortPositions, shortDistances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with initial position
        final InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D();
        solver = new NonLinearLeastSquaresLateration3DSolver(initialPosition);

        // check correctness
        assertNull(solver.getListener());
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);


        // constructor with positions, distances and initial position
        solver = new NonLinearLeastSquaresLateration3DSolver(positions, distances, initialPosition);

        // check correctness
        assertNull(solver.getListener());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNotNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver((Point3D[]) null, distances, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, wrong, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(shortPositions, shortDistances, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with listener
        // noinspection unchecked
        final LaterationSolverListener<Point3D> listener = mock(LaterationSolverListener.class);
        solver = new NonLinearLeastSquaresLateration3DSolver(listener);

        // check correctness
        assertSame(solver.getListener(), listener);
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);


        // constructor with positions, distances and listener
        solver = new NonLinearLeastSquaresLateration3DSolver(positions, distances, listener);

        // check correctness
        assertSame(solver.getListener(), listener);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver((Point3D[]) null, distances, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, null, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, wrong, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(shortPositions, shortDistances, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with initial position and listener
        solver = new NonLinearLeastSquaresLateration3DSolver(initialPosition, listener);

        // check correctness
        assertSame(solver.getListener(), listener);
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);


        // constructor with positions, distances, initial position and listener
        solver = new NonLinearLeastSquaresLateration3DSolver(positions, distances, initialPosition, listener);

        // check correctness
        assertSame(solver.getListener(), listener);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNotNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver((Point3D[]) null, distances, initialPosition,
                    listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, null, initialPosition,
                    listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, wrong, initialPosition, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(shortPositions, shortDistances, initialPosition,
                    listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with spheres
        final Sphere[] spheres = new Sphere[4];
        spheres[0] = new Sphere(positions[0], distances[0]);
        spheres[1] = new Sphere(positions[1], distances[1]);
        spheres[2] = new Sphere(positions[2], distances[2]);
        spheres[3] = new Sphere(positions[3], distances[3]);
        solver = new NonLinearLeastSquaresLateration3DSolver(spheres);

        // check correctness
        assertNull(solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);

        // Force IllegalArgumentException
        final Sphere[] shortSpheres = new Sphere[1];

        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver((Sphere[]) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(shortSpheres);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with spheres and initial position
        solver = new NonLinearLeastSquaresLateration3DSolver(spheres, initialPosition);

        // check correctness
        assertNull(solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNotNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(shortSpheres, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with spheres and listener
        solver = new NonLinearLeastSquaresLateration3DSolver(spheres, listener);

        // check correctness
        assertSame(solver.getListener(), listener);
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver((Sphere[]) null, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(shortSpheres, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with circles, initial position and listener
        solver = new NonLinearLeastSquaresLateration3DSolver(spheres, initialPosition, listener);

        // check correctness
        assertSame(solver.getListener(), listener);
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNotNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(null, initialPosition, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(shortSpheres, initialPosition, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with positions, distances and standard deviations
        final double[] standardDeviations = new double[4];
        solver = new NonLinearLeastSquaresLateration3DSolver(positions, distances,
                standardDeviations);

        // check correctness
        assertNull(solver.getListener());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(null, distances,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, null,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, distances,
                    (double[]) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, wrong,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, distances,
                    wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(shortPositions, shortDistances,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with positions, distances, standard deviations and initial position
        solver = new NonLinearLeastSquaresLateration3DSolver(positions, distances,
                standardDeviations, initialPosition);

        // check correctness
        assertNull(solver.getListener());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNotNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(null, distances,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, null,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, distances,
                    null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, wrong,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, distances,
                    wrong, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(shortPositions, shortDistances,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // check positions, distances, standard deviations and listener
        solver = new NonLinearLeastSquaresLateration3DSolver(positions, distances,
                standardDeviations, this);

        // check correctness
        assertSame(solver.getListener(), this);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(null, distances,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, null,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, distances,
                    (double[]) null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, wrong,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, distances,
                    wrong, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(shortPositions, shortDistances,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // check positions, distances, standard deviations, initial position and listener
        solver = new NonLinearLeastSquaresLateration3DSolver(positions, distances,
                standardDeviations, initialPosition, this);

        // check correctness
        assertSame(solver.getListener(), this);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNotNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(null, distances,
                    standardDeviations, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, null,
                    standardDeviations, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, distances,
                    null, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, wrong,
                    standardDeviations, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(positions, distances,
                    wrong, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(shortPositions, shortDistances,
                    standardDeviations, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with spheres and standard deviations
        solver = new NonLinearLeastSquaresLateration3DSolver(spheres,
                standardDeviations);

        // check correctness
        assertNull(solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver((Sphere[]) null,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(shortSpheres,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with spheres, standard deviations and initial position
        solver = new NonLinearLeastSquaresLateration3DSolver(spheres,
                standardDeviations, initialPosition);

        // check correctness
        assertNull(solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNotNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver((Sphere[]) null,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(shortSpheres,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with spheres, standard deviations and listener
        solver = new NonLinearLeastSquaresLateration3DSolver(spheres,
                standardDeviations, this);

        // check correctness
        assertSame(solver.getListener(), this);
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver((Sphere[]) null,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(shortSpheres,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with spheres, standard deviations, initial position and listener
        solver = new NonLinearLeastSquaresLateration3DSolver(spheres,
                standardDeviations, initialPosition, this);

        // check correctness
        assertSame(solver.getListener(), this);
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNotNull(solver.getSpheres());
        assertEquals(solver.getType(), LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver((Sphere[]) null,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration3DSolver(shortSpheres,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);
    }

    @Test
    public void testGetSetSpheres() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final NonLinearLeastSquaresLateration3DSolver solver = new NonLinearLeastSquaresLateration3DSolver();

        // initial value
        assertNull(solver.getSpheres());

        // set new value
        final Point3D[] positions = new Point3D[4];
        positions[0] = new InhomogeneousPoint3D(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());
        positions[1] = new InhomogeneousPoint3D(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());
        positions[2] = new InhomogeneousPoint3D(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());
        positions[3] = new InhomogeneousPoint3D(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());

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
        assertSame(spheres[0].getCenter(), spheres2[0].getCenter());
        assertSame(spheres[1].getCenter(), spheres2[1].getCenter());
        assertSame(spheres[2].getCenter(), spheres2[2].getCenter());
        assertSame(spheres[3].getCenter(), spheres2[3].getCenter());
        assertEquals(spheres[0].getRadius(), spheres2[0].getRadius(), 0.0);
        assertEquals(spheres[1].getRadius(), spheres2[1].getRadius(), 0.0);
        assertEquals(spheres[2].getRadius(), spheres2[2].getRadius(), 0.0);
        assertEquals(spheres[3].getRadius(), spheres2[3].getRadius(), 0.0);

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
    public void testGetSetSpheresAndStandardDeviations() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final NonLinearLeastSquaresLateration3DSolver solver = new NonLinearLeastSquaresLateration3DSolver();

        // initial value
        assertNull(solver.getSpheres());
        assertNull(solver.getDistanceStandardDeviations());

        // set new value
        final Point3D[] positions = new Point3D[4];
        positions[0] = new InhomogeneousPoint3D(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());
        positions[1] = new InhomogeneousPoint3D(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());
        positions[2] = new InhomogeneousPoint3D(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());
        positions[3] = new InhomogeneousPoint3D(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());

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

        final double[] standardDeviations = new double[4];

        solver.setSpheresAndStandardDeviations(spheres, standardDeviations);

        // check
        final Sphere[] spheres2 = solver.getSpheres();
        assertSame(spheres[0].getCenter(), spheres2[0].getCenter());
        assertSame(spheres[1].getCenter(), spheres2[1].getCenter());
        assertSame(spheres[2].getCenter(), spheres2[2].getCenter());
        assertSame(spheres[3].getCenter(), spheres2[3].getCenter());
        assertEquals(spheres[0].getRadius(), spheres2[0].getRadius(), 0.0);
        assertEquals(spheres[1].getRadius(), spheres2[1].getRadius(), 0.0);
        assertEquals(spheres[2].getRadius(), spheres2[2].getRadius(), 0.0);
        assertEquals(spheres[3].getRadius(), spheres2[3].getRadius(), 0.0);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);

        // force IllegalArgumentException
        try {
            solver.setSpheresAndStandardDeviations(null, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setSpheresAndStandardDeviations(new Sphere[1], standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setSpheresAndStandardDeviations(spheres, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setSpheresAndStandardDeviations(spheres, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final NonLinearLeastSquaresLateration3DSolver solver = new NonLinearLeastSquaresLateration3DSolver();

        // initial value
        assertNull(solver.getListener());

        // set new value
        // noinspection unchecked
        final LaterationSolverListener<Point3D> listener = mock(LaterationSolverListener.class);
        solver.setListener(listener);

        // check
        assertSame(solver.getListener(), listener);
    }

    @Test
    public void testGetSetPositionsAndDistances() throws LockedException {
        final NonLinearLeastSquaresLateration3DSolver solver = new NonLinearLeastSquaresLateration3DSolver();

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
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
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
    public void testGetSetPositionsDistancesAndStandardDeviations() throws LockedException {
        final NonLinearLeastSquaresLateration3DSolver solver = new NonLinearLeastSquaresLateration3DSolver();

        // initial value
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertNull(solver.getDistanceStandardDeviations());
        assertFalse(solver.isReady());

        // set new values
        final Point3D[] positions = new Point3D[4];
        positions[0] = new InhomogeneousPoint3D();
        positions[1] = new InhomogeneousPoint3D();
        positions[2] = new InhomogeneousPoint3D();
        positions[3] = new InhomogeneousPoint3D();
        final double[] distances = new double[4];
        final double[] standardDeviations = new double[4];

        solver.setPositionsDistancesAndStandardDeviations(positions, distances,
                standardDeviations);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);

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
    public void testGetSetInitialPosition() throws LockedException {
        final NonLinearLeastSquaresLateration3DSolver solver = new NonLinearLeastSquaresLateration3DSolver();

        // check initial value
        assertNull(solver.getInitialPosition());

        // set new value
        final InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D();
        solver.setInitialPosition(initialPosition);

        // check
        assertSame(solver.getInitialPosition(), initialPosition);
    }

    @Test
    public void testSolveNoInitialPositionAndNoError()
            throws LaterationException, NotReadyException, LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0, numInvalid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numCircles = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius;
            final Sphere[] spheres = new Sphere[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);
                spheres[i] = new Sphere(center, radius);
            }

            final NonLinearLeastSquaresLateration3DSolver solver = new NonLinearLeastSquaresLateration3DSolver(
                    spheres, this);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            final Point3D estimatedPosition = solver.getEstimatedPosition();
            final double distance = estimatedPosition.distanceTo(position);
            if (distance >= ABSOLUTE_ERROR) {
                numInvalid++;
            } else {
                numValid++;
                assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            }
        }

        assertTrue(numValid > numInvalid);

        // Force NotReadyException
        final NonLinearLeastSquaresLateration3DSolver solver = new NonLinearLeastSquaresLateration3DSolver();
        try {
            solver.solve();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }

        // Force LaterationException
        final Sphere[] spheres = new Sphere[4];
        InhomogeneousPoint3D center;
        double radius;
        for (int i = 0; i < 4; i++) {
            center = new InhomogeneousPoint3D(Double.NaN, Double.NaN, Double.NaN);
            radius = LaterationSolver.EPSILON;
            spheres[i] = new Sphere(center, radius);
        }
        solver.setSpheres(spheres);
        try {
            solver.solve();
            fail("LaterationException expected but not thrown");
        } catch (final LaterationException ignore) {
        }
    }

    @Test
    public void testSolveWithInitialPositionAndNoError()
            throws LaterationException, NotReadyException, LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // when an initial solution close to the real solution is provided, the algorithm
        // always converges to the true solution
        for (int t = 0; t < TIMES; t++) {
            final int numCircles = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                    position.getInhomX() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR),
                    position.getInhomY() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR),
                    position.getInhomZ() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR));
            InhomogeneousPoint3D center;
            double radius;
            final Sphere[] spheres = new Sphere[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);
                spheres[i] = new Sphere(center, radius);
            }

            final NonLinearLeastSquaresLateration3DSolver solver = new NonLinearLeastSquaresLateration3DSolver(
                    spheres, initialPosition, this);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            final Point3D estimatedPosition = solver.getEstimatedPosition();
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testSolveNoInitialPositionAndError()
            throws LaterationException, NotReadyException, LockedException {
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

            final NonLinearLeastSquaresLateration3DSolver solver = new NonLinearLeastSquaresLateration3DSolver(
                    positions, distances, this);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            Point3D estimatedPosition = solver.getEstimatedPosition();
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
    public void testSolveWithInitialPositionAndError()
            throws LaterationException, NotReadyException, LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // when an initial solution close to the real solution is provided, the algorithm
        // always converges to the true solution
        for (int t = 0; t < TIMES; t++) {
            final int numPoints = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                    position.getInhomX() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR),
                    position.getInhomY() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR),
                    position.getInhomZ() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR));
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

            final NonLinearLeastSquaresLateration3DSolver solver = new NonLinearLeastSquaresLateration3DSolver(
                    positions, distances, initialPosition, this);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            final Point3D estimatedPosition = solver.getEstimatedPosition();
            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testSolveNoInitialPositionAndNoError4Spheres()
            throws LaterationException, NotReadyException, LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0, numInvalid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numCircles = MIN_SPHERES;

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius;
            final Sphere[] spheres = new Sphere[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);
                spheres[i] = new Sphere(center, radius);
            }

            final NonLinearLeastSquaresLateration3DSolver solver = new NonLinearLeastSquaresLateration3DSolver(
                    spheres, this);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            final Point3D estimatedPosition = solver.getEstimatedPosition();
            final double distance = estimatedPosition.distanceTo(position);
            if (distance >= ABSOLUTE_ERROR) {
                numInvalid++;
            } else {
                numValid++;
                assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            }
        }

        assertTrue(numValid > numInvalid);

        // Force NotReadyException
        final NonLinearLeastSquaresLateration3DSolver solver = new NonLinearLeastSquaresLateration3DSolver();
        try {
            solver.solve();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }

        // Force LaterationException
        final Sphere[] spheres = new Sphere[4];
        InhomogeneousPoint3D center;
        double radius;
        for (int i = 0; i < 4; i++) {
            center = new InhomogeneousPoint3D(Double.NaN, Double.NaN, Double.NaN);
            radius = LaterationSolver.EPSILON;
            spheres[i] = new Sphere(center, radius);
        }
        solver.setSpheres(spheres);
        try {
            solver.solve();
            fail("LaterationException expected but not thrown");
        } catch (final LaterationException ignore) {
        }
    }

    @Override
    public void onSolveStart(final LaterationSolver<Point3D> solver) {
        solveStart++;
        checkLocked((NonLinearLeastSquaresLateration3DSolver) solver);
    }

    @Override
    public void onSolveEnd(final LaterationSolver<Point3D> solver) {
        solveEnd++;
        checkLocked((NonLinearLeastSquaresLateration3DSolver) solver);
    }

    private void reset() {
        solveStart = solveEnd = 0;
    }

    private void checkLocked(final NonLinearLeastSquaresLateration3DSolver solver) {
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
            solver.setPositionsDistancesAndStandardDeviations(
                    null, null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setInitialPosition(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setSpheres(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setSpheresAndStandardDeviations(
                    null, null);
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
