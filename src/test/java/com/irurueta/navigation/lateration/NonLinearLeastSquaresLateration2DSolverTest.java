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

public class NonLinearLeastSquaresLateration2DSolverTest implements LaterationSolverListener<Point2D> {

    private static final int MIN_CIRCLES = 3;
    private static final int MAX_CIRCLES = 10;

    private static final double MIN_RANDOM_VALUE = -50.0;
    private static final double MAX_RANDOM_VALUE = 50.0;

    private static final double MIN_POSITION_ERROR = 1e-2;
    private static final double MAX_POSITION_ERROR = 1.0;

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
        NonLinearLeastSquaresLateration2DSolver solver = new NonLinearLeastSquaresLateration2DSolver();

        // check correctness
        assertNull(solver.getListener());
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertNull(solver.getInitialPosition());
        assertNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());


        // constructor with positions and distances
        final Point2D[] positions = new Point2D[3];
        positions[0] = new InhomogeneousPoint2D();
        positions[1] = new InhomogeneousPoint2D();
        positions[2] = new InhomogeneousPoint2D();
        final double[] distances = new double[3];
        solver = new NonLinearLeastSquaresLateration2DSolver(positions, distances);

        // check correctness
        assertNull(solver.getListener());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        final double[] wrong = new double[4];
        final Point2D[] shortPositions = new Point2D[1];
        final double[] shortDistances = new double[1];
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver((Point2D[]) null, distances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(shortPositions, shortDistances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);

        // constructor with initial position
        final InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D();
        solver = new NonLinearLeastSquaresLateration2DSolver(initialPosition);

        // check correctness
        assertNull(solver.getListener());
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertSame(initialPosition, solver.getInitialPosition());
        assertNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());


        // constructor with positions, distances and initial position
        solver = new NonLinearLeastSquaresLateration2DSolver(positions, distances, initialPosition);

        // check correctness
        assertNull(solver.getListener());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertSame(initialPosition, solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver((Point2D[]) null, distances, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, wrong, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(shortPositions, shortDistances,
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with listener
        // noinspection unchecked
        final LaterationSolverListener<Point2D> listener = mock(LaterationSolverListener.class);
        solver = new NonLinearLeastSquaresLateration2DSolver(listener);

        // check correctness
        assertSame(listener, solver.getListener());
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertNull(solver.getInitialPosition());
        assertNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // constructor with positions, distances and listener
        solver = new NonLinearLeastSquaresLateration2DSolver(positions, distances, listener);

        // check correctness
        assertSame(listener, solver.getListener());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver((Point2D[]) null, distances, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, null, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, wrong, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(shortPositions, shortDistances, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with initial position and listener
        solver = new NonLinearLeastSquaresLateration2DSolver(initialPosition, listener);

        // check correctness
        assertSame(listener, solver.getListener());
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertSame(initialPosition, solver.getInitialPosition());
        assertNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // constructor with positions, distances, initial position and listener
        solver = new NonLinearLeastSquaresLateration2DSolver(positions, distances, initialPosition, listener);

        // check correctness
        assertSame(listener, solver.getListener());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertSame(initialPosition, solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver((Point2D[]) null, distances, initialPosition,
                    listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, null, initialPosition,
                    listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, wrong, initialPosition, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(shortPositions, shortDistances, initialPosition,
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
        solver = new NonLinearLeastSquaresLateration2DSolver(circles);

        // check correctness
        assertNull(solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        final Circle[] shortCircles = new Circle[1];

        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver((Circle[]) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(shortCircles);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with circles and initial position
        solver = new NonLinearLeastSquaresLateration2DSolver(circles, initialPosition);

        // check correctness
        assertNull(solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertSame(initialPosition, solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(shortCircles, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with circles and listener
        solver = new NonLinearLeastSquaresLateration2DSolver(circles, listener);

        // check correctness
        assertSame(listener, solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver((Circle[]) null, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(shortCircles, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with circles, initial position and listener
        solver = new NonLinearLeastSquaresLateration2DSolver(circles, initialPosition, listener);

        // check correctness
        assertSame(listener, solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(null, initialPosition, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(shortCircles, initialPosition, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with positions, distances and standard deviations
        final double[] standardDeviations = new double[3];
        solver = new NonLinearLeastSquaresLateration2DSolver(positions, distances, standardDeviations);

        // check correctness
        assertNull(solver.getListener());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(null, distances, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, null, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, distances, (double[]) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, wrong, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, distances, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(shortPositions, shortDistances,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with positions, distances, standard deviations and initial position
        solver = new NonLinearLeastSquaresLateration2DSolver(positions, distances,
                standardDeviations, initialPosition);

        // check correctness
        assertNull(solver.getListener());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertSame(initialPosition, solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(null, distances,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, null,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, distances,
                    null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, wrong,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, distances, wrong, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(shortPositions, shortDistances,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // check positions, distances, standard deviations and listener
        solver = new NonLinearLeastSquaresLateration2DSolver(positions, distances,
                standardDeviations, this);

        // check correctness
        assertSame(this, solver.getListener());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(null, distances,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, null,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, distances,
                    (double[]) null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, wrong,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, distances, wrong, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(shortPositions, shortDistances,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // check positions, distances, standard deviations, initial position and listener
        solver = new NonLinearLeastSquaresLateration2DSolver(positions, distances,
                standardDeviations, initialPosition, this);

        // check correctness
        assertSame(this, solver.getListener());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertSame(initialPosition, solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(null, distances,
                    standardDeviations, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, null,
                    standardDeviations, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, distances,
                    null, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, wrong,
                    standardDeviations, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(positions, distances,
                    wrong, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(shortPositions, shortDistances,
                    standardDeviations, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with circles and standard deviations
        solver = new NonLinearLeastSquaresLateration2DSolver(circles, standardDeviations);

        // check correctness
        assertNull(solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver((Circle[]) null, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(shortCircles, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with circles, standard deviations and initial position
        solver = new NonLinearLeastSquaresLateration2DSolver(circles, standardDeviations, initialPosition);

        // check correctness
        assertNull(solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertSame(initialPosition, solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver((Circle[]) null,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(shortCircles,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with circles, standard deviations and listener
        solver = new NonLinearLeastSquaresLateration2DSolver(circles, standardDeviations, this);

        // check correctness
        assertSame(this, solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver((Circle[]) null,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(shortCircles,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with circles, standard deviations, initial position and listener
        solver = new NonLinearLeastSquaresLateration2DSolver(circles,
                standardDeviations, initialPosition, this);

        // check correctness
        assertSame(this, solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(2, solver.getNumberOfDimensions());
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(0.0, solver.getChiSq(), 0.0);
        assertSame(initialPosition, solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(LaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER, solver.getType());
        assertEquals(3, solver.getMinRequiredPositionsAndDistances());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver((Circle[]) null,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new NonLinearLeastSquaresLateration2DSolver(shortCircles,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);
    }

    @Test
    public void testGetSetCircles() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final NonLinearLeastSquaresLateration2DSolver solver = new NonLinearLeastSquaresLateration2DSolver();

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
        assertSame(circles[0].getCenter(), circles2[0].getCenter());
        assertSame(circles[1].getCenter(), circles2[1].getCenter());
        assertSame(circles[2].getCenter(), circles2[2].getCenter());
        assertEquals(circles[0].getRadius(), circles2[0].getRadius(), 0.0);
        assertEquals(circles[1].getRadius(), circles2[1].getRadius(), 0.0);
        assertEquals(circles[2].getRadius(), circles2[2].getRadius(), 0.0);

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
    public void testGetSetCirclesAndStandardDeviations() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final NonLinearLeastSquaresLateration2DSolver solver = new NonLinearLeastSquaresLateration2DSolver();

        // initial value
        assertNull(solver.getCircles());
        assertNull(solver.getDistanceStandardDeviations());

        // set new values
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

        final double[] standardDeviations = new double[3];

        solver.setCirclesAndStandardDeviations(circles, standardDeviations);

        // check
        final Circle[] circles2 = solver.getCircles();
        assertSame(circles[0].getCenter(), circles2[0].getCenter());
        assertSame(circles[1].getCenter(), circles2[1].getCenter());
        assertSame(circles[2].getCenter(), circles2[2].getCenter());
        assertEquals(circles[0].getRadius(), circles2[0].getRadius(), 0.0);
        assertEquals(circles[1].getRadius(), circles2[1].getRadius(), 0.0);
        assertEquals(circles[2].getRadius(), circles2[2].getRadius(), 0.0);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);

        // force IllegalArgumentException
        try {
            solver.setCirclesAndStandardDeviations(null, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setCirclesAndStandardDeviations(new Circle[1], standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setCirclesAndStandardDeviations(circles, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setCirclesAndStandardDeviations(circles, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final NonLinearLeastSquaresLateration2DSolver solver = new NonLinearLeastSquaresLateration2DSolver();

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
        final NonLinearLeastSquaresLateration2DSolver solver = new NonLinearLeastSquaresLateration2DSolver();

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
    public void testGetSetPositionsDistancesAndStandardDeviations() throws LockedException {
        final NonLinearLeastSquaresLateration2DSolver solver = new NonLinearLeastSquaresLateration2DSolver();

        // initial value
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertNull(solver.getDistanceStandardDeviations());
        assertFalse(solver.isReady());

        // set new values
        final Point2D[] positions = new Point2D[3];
        positions[0] = new InhomogeneousPoint2D();
        positions[1] = new InhomogeneousPoint2D();
        positions[2] = new InhomogeneousPoint2D();
        final double[] distances = new double[3];
        final double[] standardDeviations = new double[3];

        solver.setPositionsDistancesAndStandardDeviations(positions, distances, standardDeviations);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver.isReady());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());

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
    public void testGetSetInitialPosition() throws LockedException {
        final NonLinearLeastSquaresLateration2DSolver solver = new NonLinearLeastSquaresLateration2DSolver();

        // check initial value
        assertNull(solver.getInitialPosition());

        // set new value
        final InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D();
        solver.setInitialPosition(initialPosition);

        // check
        assertSame(initialPosition, solver.getInitialPosition());
    }

    @Test
    public void testSolveNoInitialPositionAndNoError()
            throws LaterationException, NotReadyException, LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0, numInvalid = 0;
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

            final NonLinearLeastSquaresLateration2DSolver solver = new NonLinearLeastSquaresLateration2DSolver(
                    circles, this);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            final Point2D estimatedPosition = solver.getEstimatedPosition();
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
        final NonLinearLeastSquaresLateration2DSolver solver = new NonLinearLeastSquaresLateration2DSolver();
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
            center = new InhomogeneousPoint2D(Double.NaN, Double.NaN);
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
    public void testSolveWithInitialPositionAndNoError()
            throws LaterationException, NotReadyException, LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // when an initial solution close to the real solution is provided, the algorithm
        // always converges to the true solution
        for (int t = 0; t < TIMES; t++) {
            final int numCircles = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D(
                    position.getInhomX() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR),
                    position.getInhomY() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR));
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

            final NonLinearLeastSquaresLateration2DSolver solver = new NonLinearLeastSquaresLateration2DSolver(
                    circles, initialPosition, this);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            final Point2D estimatedPosition = solver.getEstimatedPosition();
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testSolveNoInitialPositionAndError()
            throws LaterationException, NotReadyException, LockedException {
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

            final NonLinearLeastSquaresLateration2DSolver solver = new NonLinearLeastSquaresLateration2DSolver(
                    positions, distances, this);

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
    public void testSolveWithInitialPositionAndError()
            throws LaterationException, NotReadyException, LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // when an initial solution close to the real solution is provided, the algorithm
        // always converges to the true solution
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numPoints = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            final InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D(
                    position.getInhomX() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR),
                    position.getInhomY() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR));
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

            final NonLinearLeastSquaresLateration2DSolver solver = new NonLinearLeastSquaresLateration2DSolver(
                    positions, distances, initialPosition, this);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            final Point2D estimatedPosition = solver.getEstimatedPosition();
            if (!position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveNoInitialPositionAndNoError3Circles()
            throws LaterationException, NotReadyException, LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0, numInvalid = 0;
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

            final NonLinearLeastSquaresLateration2DSolver solver = new NonLinearLeastSquaresLateration2DSolver(
                    circles, this);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            final Point2D estimatedPosition = solver.getEstimatedPosition();
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
        final NonLinearLeastSquaresLateration2DSolver solver = new NonLinearLeastSquaresLateration2DSolver();
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
            center = new InhomogeneousPoint2D(Double.NaN, Double.NaN);
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
        checkLocked((NonLinearLeastSquaresLateration2DSolver) solver);
    }

    @Override
    public void onSolveEnd(final LaterationSolver<Point2D> solver) {
        solveEnd++;
        checkLocked((NonLinearLeastSquaresLateration2DSolver) solver);
    }

    private void reset() {
        solveStart = solveEnd = 0;
    }

    private void checkLocked(final NonLinearLeastSquaresLateration2DSolver solver) {
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
            solver.setCircles(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setCirclesAndStandardDeviations(
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
