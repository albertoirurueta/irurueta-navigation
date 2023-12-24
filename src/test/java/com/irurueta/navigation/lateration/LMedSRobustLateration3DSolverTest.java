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
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class LMedSRobustLateration3DSolverTest implements RobustLaterationSolverListener<Point3D> {

    private static final int MIN_SPHERES = 100;
    private static final int MAX_SPHERES = 500;

    private static final double MIN_RANDOM_VALUE = -50.0;
    private static final double MAX_RANDOM_VALUE = 50.0;

    private static final double MIN_DISTANCE_ERROR = -1e-2;
    private static final double MAX_DISTANCE_ERROR = 1e-2;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final int TIMES = 50;

    private static final int PERCENTAGE_OUTLIERS = 20;

    private static final double STD_OUTLIER_ERROR = 10.0;

    private int solveStart;
    private int solveEnd;
    private int solveNextIteration;
    private int solveProgressChange;

    @Test
    public void testConstructor() {
        // empty constructor
        LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check correctness
        assertEquals(LMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, solver.getMethod());
        assertEquals(3, solver.getNumberOfDimensions());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());
        assertEquals(4, solver.getPreliminarySubsetSize());
        assertNull(solver.getSpheres());
        assertNull(solver.getListener());
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, solver.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, solver.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, solver.getMaxIterations());
        assertNull(solver.getInliersData());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, solver.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, solver.isCovarianceKept());
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertNull(solver.getDistanceStandardDeviations());
        assertFalse(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // constructor with listener
        solver = new LMedSRobustLateration3DSolver(this);

        // check correctness
        assertEquals(LMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, solver.getMethod());
        assertEquals(3, solver.getNumberOfDimensions());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());
        assertEquals(4, solver.getPreliminarySubsetSize());
        assertNull(solver.getSpheres());
        assertSame(this, solver.getListener());
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, solver.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, solver.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, solver.getMaxIterations());
        assertNull(solver.getInliersData());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, solver.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, solver.isCovarianceKept());
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertNull(solver.getDistanceStandardDeviations());
        assertFalse(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // constructor with positions and distances
        final Point3D[] positions = new Point3D[4];
        positions[0] = new InhomogeneousPoint3D();
        positions[1] = new InhomogeneousPoint3D();
        positions[2] = new InhomogeneousPoint3D();
        positions[3] = new InhomogeneousPoint3D();
        final double[] distances = new double[4];
        solver = new LMedSRobustLateration3DSolver(positions, distances);

        // check correctness
        assertEquals(LMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, solver.getMethod());
        assertEquals(3, solver.getNumberOfDimensions());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());
        assertEquals(4, solver.getPreliminarySubsetSize());
        assertNotNull(solver.getSpheres());
        assertNull(solver.getListener());
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, solver.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, solver.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, solver.getMaxIterations());
        assertNull(solver.getInliersData());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, solver.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, solver.isCovarianceKept());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertNull(solver.getDistanceStandardDeviations());
        assertTrue(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        final double[] wrong = new double[5];
        final Point3D[] shortPositions = new Point3D[1];
        final double[] shortDistances = new double[1];
        solver = null;
        try {
            solver = new LMedSRobustLateration3DSolver((Point3D[]) null, distances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(positions, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(positions, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(shortPositions, shortDistances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with positions, distances and standard deviations
        final double[] standardDeviations = new double[4];
        solver = new LMedSRobustLateration3DSolver(positions, distances, standardDeviations);

        // check correctness
        assertEquals(LMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, solver.getMethod());
        assertEquals(3, solver.getNumberOfDimensions());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());
        assertEquals(4, solver.getPreliminarySubsetSize());
        assertNotNull(solver.getSpheres());
        assertNull(solver.getListener());
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, solver.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, solver.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, solver.getMaxIterations());
        assertNull(solver.getInliersData());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, solver.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, solver.isCovarianceKept());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        solver = null;
        try {
            solver = new LMedSRobustLateration3DSolver(null, distances, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(positions, null, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(positions, distances, (double[]) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(positions, wrong, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(positions, distances, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(shortPositions, shortDistances, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with positions, distances, standard deviations and listener
        solver = new LMedSRobustLateration3DSolver(positions, distances, standardDeviations, this);

        // check correctness
        assertEquals(LMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, solver.getMethod());
        assertEquals(3, solver.getNumberOfDimensions());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());
        assertEquals(4, solver.getPreliminarySubsetSize());
        assertNotNull(solver.getSpheres());
        assertSame(this, solver.getListener());
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, solver.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, solver.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, solver.getMaxIterations());
        assertNull(solver.getInliersData());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, solver.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, solver.isCovarianceKept());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        solver = null;
        try {
            solver = new LMedSRobustLateration3DSolver(null, distances,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(positions, null,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(positions, distances,
                    null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(positions, wrong,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(positions, distances,
                    wrong, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(shortPositions,
                    shortDistances, standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with positions, distances and listener
        solver = new LMedSRobustLateration3DSolver(positions, distances, this);

        // check correctness
        assertEquals(LMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, solver.getMethod());
        assertEquals(3, solver.getNumberOfDimensions());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());
        assertEquals(4, solver.getPreliminarySubsetSize());
        assertNotNull(solver.getSpheres());
        assertSame(this, solver.getListener());
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, solver.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, solver.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, solver.getMaxIterations());
        assertNull(solver.getInliersData());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, solver.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, solver.isCovarianceKept());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertNull(solver.getDistanceStandardDeviations());
        assertTrue(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        solver = null;
        try {
            solver = new LMedSRobustLateration3DSolver((Point3D[]) null, distances, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(positions, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(positions, wrong, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(shortPositions, shortDistances, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with circles
        final Sphere[] spheres = new Sphere[4];
        spheres[0] = new Sphere(positions[0], distances[0]);
        spheres[1] = new Sphere(positions[1], distances[1]);
        spheres[2] = new Sphere(positions[2], distances[2]);
        spheres[3] = new Sphere(positions[3], distances[3]);
        solver = new LMedSRobustLateration3DSolver(spheres);

        // check correctness
        assertEquals(LMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, solver.getMethod());
        assertEquals(3, solver.getNumberOfDimensions());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());
        assertEquals(4, solver.getPreliminarySubsetSize());
        assertNotNull(solver.getSpheres());
        assertNull(solver.getListener());
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, solver.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, solver.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, solver.getMaxIterations());
        assertNull(solver.getInliersData());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, solver.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, solver.isCovarianceKept());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertNull(solver.getDistanceStandardDeviations());
        assertTrue(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        final Sphere[] shortSpheres = new Sphere[1];

        solver = null;
        try {
            solver = new LMedSRobustLateration3DSolver((Sphere[]) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(shortSpheres);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);

        // constructor with circles and standard deviations
        solver = new LMedSRobustLateration3DSolver(spheres, standardDeviations);

        // check correctness
        assertEquals(LMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, solver.getMethod());
        assertEquals(3, solver.getNumberOfDimensions());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());
        assertEquals(4, solver.getPreliminarySubsetSize());
        assertNotNull(solver.getSpheres());
        assertNull(solver.getListener());
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, solver.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, solver.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, solver.getMaxIterations());
        assertNull(solver.getInliersData());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, solver.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, solver.isCovarianceKept());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        solver = null;
        try {
            solver = new LMedSRobustLateration3DSolver((Sphere[]) null, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(spheres, (double[]) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(shortSpheres, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(spheres, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with circles and listener
        solver = new LMedSRobustLateration3DSolver(spheres, this);

        // check correctness
        assertEquals(LMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, solver.getMethod());
        assertEquals(3, solver.getNumberOfDimensions());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());
        assertEquals(4, solver.getPreliminarySubsetSize());
        assertNotNull(solver.getSpheres());
        assertSame(this, solver.getListener());
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, solver.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, solver.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, solver.getMaxIterations());
        assertNull(solver.getInliersData());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, solver.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, solver.isCovarianceKept());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertNull(solver.getDistanceStandardDeviations());
        assertTrue(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        solver = null;
        try {
            solver = new LMedSRobustLateration3DSolver(null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(shortSpheres, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // constructor with circles, standard deviation and listener
        solver = new LMedSRobustLateration3DSolver(spheres, standardDeviations, this);

        // check correctness
        assertEquals(LMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, solver.getMethod());
        assertEquals(3, solver.getNumberOfDimensions());
        assertEquals(4, solver.getMinRequiredPositionsAndDistances());
        assertEquals(4, solver.getPreliminarySubsetSize());
        assertNotNull(solver.getSpheres());
        assertSame(this, solver.getListener());
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, solver.getProgressDelta(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, solver.getConfidence(), 0.0);
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, solver.getMaxIterations());
        assertNull(solver.getInliersData());
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, solver.isResultRefined());
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, solver.isCovarianceKept());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        solver = null;
        try {
            solver = new LMedSRobustLateration3DSolver((Sphere[]) null, standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(spheres, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(shortSpheres, standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new LMedSRobustLateration3DSolver(spheres, wrong, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        final LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check initial value
        assertEquals(LMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);

        // set new value
        solver.setStopThreshold(1.0);

        // check
        assertEquals(1.0, solver.getStopThreshold(), 0.0);

        // force IllegalArgumentException
        try {
            solver.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetSpheres() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check initial value
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
    public void testGetSetCirclesAndStandardDeviations() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check initial value
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
        final double[] standardDeviations = new double[4];
        standardDeviations[0] = randomizer.nextDouble();
        standardDeviations[1] = randomizer.nextDouble();
        standardDeviations[2] = randomizer.nextDouble();
        standardDeviations[3] = randomizer.nextDouble();

        final Sphere[] spheres = new Sphere[4];
        spheres[0] = new Sphere(positions[0], distances[0]);
        spheres[1] = new Sphere(positions[1], distances[1]);
        spheres[2] = new Sphere(positions[2], distances[2]);
        spheres[3] = new Sphere(positions[3], distances[3]);
        solver.setSpheresAndStandardDeviations(spheres, standardDeviations);

        // check
        final Sphere[] spheres2 = solver.getSpheres();
        for (int i = 0; i < 4; i++) {
            assertSame(spheres[i].getCenter(), spheres2[i].getCenter());
            assertEquals(spheres[i].getRadius(), spheres2[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);

        // force IllegalArgumentException
        try {
            solver.setSpheresAndStandardDeviations(null, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setSpheresAndStandardDeviations(spheres, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setSpheresAndStandardDeviations(new Sphere[1], standardDeviations);
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
    public void testGetSetPreliminarySubsetSize() throws LockedException {
        final LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check initial value
        assertEquals(4, solver.getPreliminarySubsetSize());

        // set new value
        solver.setPreliminarySubsetSize(5);

        // check
        assertEquals(5, solver.getPreliminarySubsetSize());

        // force IllegalArgumentException
        try {
            solver.setPreliminarySubsetSize(3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check default value
        assertNull(solver.getListener());

        // set new value
        solver.setListener(this);

        // check
        assertSame(this, solver.getListener());
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        final LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check default value
        assertNull(solver.getInitialPosition());

        // set new value
        final Point3D p = Point3D.create();
        solver.setInitialPosition(p);

        // check
        assertSame(p, solver.getInitialPosition());
    }

    @Test
    public void testIsSetLinearSolverUsed() throws LockedException {
        final LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check default value
        assertTrue(solver.isLinearSolverUsed());

        // set new value
        solver.setLinearSolverUsed(false);

        // check
        assertFalse(solver.isLinearSolverUsed());
    }

    @Test
    public void testIsSetHomogeneousLinearSolverUsed() throws LockedException {
        final LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check default value
        assertFalse(solver.isHomogeneousLinearSolverUsed());

        // set new value
        solver.setHomogeneousLinearSolverUsed(true);

        // check
        assertTrue(solver.isHomogeneousLinearSolverUsed());
    }

    @Test
    public void testIsSetPreliminarySolutionRefined() throws LockedException {
        final LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check default value
        assertTrue(solver.isPreliminarySolutionRefined());

        // set new value
        solver.setPreliminarySolutionRefined(false);

        // check
        assertFalse(solver.isPreliminarySolutionRefined());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check default value
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, solver.getProgressDelta(), 0.0);

        // set new value
        solver.setProgressDelta(0.5f);

        // check
        assertEquals(0.5f, solver.getProgressDelta(), 0.0);

        // force IllegalArgumentException
        try {
            solver.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetConfidence() throws LockedException {
        final LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check default value
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, solver.getConfidence(), 0.0);

        // set new value
        solver.setConfidence(0.8);

        // check
        assertEquals(0.8, solver.getConfidence(), 0.0);

        // force IllegalArgumentException
        try {
            solver.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMaxIterations() throws LockedException {
        final LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check default value
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, solver.getMaxIterations());

        // set new value
        solver.setMaxIterations(10);

        // check
        assertEquals(10, solver.getMaxIterations());

        // force IllegalArgumentException
        try {
            solver.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        final LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check default value
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, solver.isResultRefined());

        // set new value
        solver.setResultRefined(!RobustLaterationSolver.DEFAULT_REFINE_RESULT);

        // check
        assertEquals(!RobustLaterationSolver.DEFAULT_REFINE_RESULT, solver.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check default value
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, solver.isCovarianceKept());

        // set new value
        solver.setCovarianceKept(!RobustLaterationSolver.DEFAULT_REFINE_RESULT);

        // check
        assertEquals(!RobustLaterationSolver.DEFAULT_REFINE_RESULT, solver.isCovarianceKept());
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check default value
        assertNull(solver.getQualityScores());

        // set new value
        solver.setQualityScores(new double[3]);

        // check
        assertNull(solver.getQualityScores());
    }

    @Test
    public void testGetSetPositionsAndDistances() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check default value
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());

        // set new values
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

        solver.setPositionsAndDistances(positions, distances);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());

        // force IllegalArgumentException
        final double[] wrong = new double[5];
        final Point3D[] shortPositions = new Point3D[1];
        final double[] shortDistances = new double[1];
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
            solver.setPositionsAndDistances(positions, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setPositionsAndDistances(shortPositions, shortDistances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetPositionsDistancesAndStandardDeviations() throws LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver();

        // check default value
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());

        // set new values
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
        final double[] standardDeviations = new double[4];
        standardDeviations[0] = randomizer.nextDouble();
        standardDeviations[1] = randomizer.nextDouble();
        standardDeviations[2] = randomizer.nextDouble();
        standardDeviations[3] = randomizer.nextDouble();

        solver.setPositionsDistancesAndStandardDeviations(positions, distances, standardDeviations);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());

        // force IllegalArgumentException
        final double[] wrong = new double[5];
        final Point3D[] shortPositions = new Point3D[1];
        final double[] shortDistances = new double[1];
        final double[] shortStandardDeviations = new double[1];
        try {
            solver.setPositionsDistancesAndStandardDeviations(null, distances, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setPositionsDistancesAndStandardDeviations(positions, null, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setPositionsDistancesAndStandardDeviations(positions, distances, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setPositionsDistancesAndStandardDeviations(positions, wrong, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setPositionsDistancesAndStandardDeviations(positions, distances, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setPositionsDistancesAndStandardDeviations(shortPositions, shortDistances, shortStandardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSolveNoInlierErrorNoRefinement() throws Exception {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            final Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver(spheres, this);
            solver.setResultRefined(false);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertEquals(0, solveNextIteration);
            assertEquals(0, solveProgressChange);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            final Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(1, solveStart);
            assertEquals(1, solveEnd);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new LMedSRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (final NotReadyException ignore) {
            }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveNoInlierErrorWithRefinement() throws Exception {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            final Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            LMedSRobustLateration3DSolver solver =
                    new LMedSRobustLateration3DSolver(spheres, this);
            solver.setResultRefined(true);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertEquals(0, solveNextIteration);
            assertEquals(0, solveProgressChange);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            final Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(1, solveStart);
            assertEquals(1, solveEnd);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new LMedSRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (final NotReadyException ignore) {
            }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveWithInlierErrorWithRefinement() throws Exception {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            final Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                error += randomizer.nextDouble(MIN_DISTANCE_ERROR, MAX_DISTANCE_ERROR);
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver(spheres, this);
            solver.setResultRefined(true);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertEquals(0, solveNextIteration);
            assertEquals(0, solveProgressChange);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            final Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(1, solveStart);
            assertEquals(1, solveEnd);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new LMedSRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (final NotReadyException ignore) {
            }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveWithInlierErrorWithRefinementAndStandardDeviations() throws Exception {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            final Sphere[] spheres = new Sphere[numSpheres];
            final double[] standardDeviations = new double[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                    standardDeviations[i] = STD_OUTLIER_ERROR;
                } else {
                    // inlier
                    error = 0.0;
                    standardDeviations[i] = 0.0;
                }
                // add variance of uniform distribution containing inlier error
                standardDeviations[i] += Math.pow(MAX_DISTANCE_ERROR - MIN_DISTANCE_ERROR, 2.0) / 12.0;
                standardDeviations[i] = Math.sqrt(standardDeviations[i]);
                // add inlier error
                error += randomizer.nextDouble(MIN_DISTANCE_ERROR, MAX_DISTANCE_ERROR);
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver(spheres, this);
            solver.setResultRefined(true);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertEquals(0, solveNextIteration);
            assertEquals(0, solveProgressChange);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            final Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(1, solveStart);
            assertEquals(1, solveEnd);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new LMedSRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (final NotReadyException ignore) {
            }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveNoPreliminaryLinearSolver() throws Exception {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            final Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver(spheres, this);
            solver.setLinearSolverUsed(false);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertEquals(0, solveNextIteration);
            assertEquals(0, solveProgressChange);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            final Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(1, solveStart);
            assertEquals(1, solveEnd);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new LMedSRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (final NotReadyException ignore) {
            }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveInhomogeneousPreliminaryLinearSolver() throws Exception {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            final Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver(spheres, this);
            solver.setLinearSolverUsed(true);
            solver.setHomogeneousLinearSolverUsed(false);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertEquals(0, solveNextIteration);
            assertEquals(0, solveProgressChange);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            final Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(1, solveStart);
            assertEquals(1, solveEnd);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new LMedSRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (final NotReadyException ignore) {
            }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveHomogeneousPreliminaryLinearSolver() throws Exception {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            final Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver(spheres, this);
            solver.setLinearSolverUsed(true);
            solver.setHomogeneousLinearSolverUsed(true);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertEquals(0, solveNextIteration);
            assertEquals(0, solveProgressChange);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            final Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(1, solveStart);
            assertEquals(1, solveEnd);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new LMedSRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (final NotReadyException ignore) {
            }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveRefinePreliminarySolutions() throws Exception {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            final Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver(spheres, this);
            solver.setLinearSolverUsed(true);
            solver.setPreliminarySolutionRefined(true);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertEquals(0, solveNextIteration);
            assertEquals(0, solveProgressChange);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            final Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(1, solveStart);
            assertEquals(1, solveEnd);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new LMedSRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (final NotReadyException ignore) {
            }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolvePreliminarySolutionsNotRefined() throws Exception {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            final Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver(spheres, this);
            solver.setLinearSolverUsed(true);
            solver.setPreliminarySolutionRefined(false);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertEquals(0, solveNextIteration);
            assertEquals(0, solveProgressChange);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            final Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(1, solveStart);
            assertEquals(1, solveEnd);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new LMedSRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (final NotReadyException ignore) {
            }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveNoPreliminaryLinearSolverAndNoPreliminarySolutionsRefinement()
            throws Exception {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            final Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver(spheres, this);
            solver.setLinearSolverUsed(false);
            solver.setPreliminarySolutionRefined(false);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertEquals(0, solveNextIteration);
            assertEquals(0, solveProgressChange);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            final Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(1, solveStart);
            assertEquals(1, solveEnd);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new LMedSRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (final NotReadyException ignore) {
            }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveNoPreliminaryLinearSolverWithInitialPosition() throws Exception {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            final Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver(spheres, this);
            solver.setLinearSolverUsed(false);
            solver.setInitialPosition(position);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertEquals(0, solveNextIteration);
            assertEquals(0, solveProgressChange);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            final Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(1, solveStart);
            assertEquals(1, solveEnd);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new LMedSRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (final NotReadyException ignore) {
            }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveLargerPreliminarySubsetSize() throws Exception {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            final Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            LMedSRobustLateration3DSolver solver = new LMedSRobustLateration3DSolver(spheres, this);
            solver.setResultRefined(false);
            solver.setPreliminarySubsetSize(5);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertEquals(0, solveNextIteration);
            assertEquals(0, solveProgressChange);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            final Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(1, solveStart);
            assertEquals(1, solveEnd);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new LMedSRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (final NotReadyException ignore) {
            }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onSolveStart(final RobustLaterationSolver<Point3D> solver) {
        solveStart++;
        checkLocked((LMedSRobustLateration3DSolver) solver);
    }

    @Override
    public void onSolveEnd(final RobustLaterationSolver<Point3D> solver) {
        solveEnd++;
        checkLocked((LMedSRobustLateration3DSolver) solver);
    }

    @Override
    public void onSolveNextIteration(final RobustLaterationSolver<Point3D> solver, final int iteration) {
        solveNextIteration++;
        checkLocked((LMedSRobustLateration3DSolver) solver);
    }

    @Override
    public void onSolveProgressChange(final RobustLaterationSolver<Point3D> solver, final float progress) {
        solveProgressChange++;
        checkLocked((LMedSRobustLateration3DSolver) solver);
    }

    private void reset() {
        solveStart = solveEnd = solveNextIteration = solveProgressChange = 0;
    }

    private void checkLocked(final LMedSRobustLateration3DSolver solver) {
        try {
            solver.setPreliminarySubsetSize(4);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setInitialPosition(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setLinearSolverUsed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setHomogeneousLinearSolverUsed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setPreliminarySolutionRefined(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setMaxIterations(5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setResultRefined(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setCovarianceKept(false);
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
            solver.setSpheres(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setSpheresAndStandardDeviations(null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setStopThreshold(0.5);
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
