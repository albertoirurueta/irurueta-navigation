/*
 * Copyright (C) 2018 Alberto Irurueta Carro (alberto@irurueta.com)
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
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class PROMedSRobustLateration3DSolverTest implements RobustLaterationSolverListener<Point3D> {

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
    void testConstructor() {
        // empty constructor
        var solver = new PROMedSRobustLateration3DSolver();

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        solver = new PROMedSRobustLateration3DSolver(this);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        final var positions = new Point3D[4];
        positions[0] = new InhomogeneousPoint3D();
        positions[1] = new InhomogeneousPoint3D();
        positions[2] = new InhomogeneousPoint3D();
        positions[3] = new InhomogeneousPoint3D();
        final var distances = new double[4];
        solver = new PROMedSRobustLateration3DSolver(positions, distances);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertFalse(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        final var wrong = new double[5];
        final var shortPositions = new Point3D[1];
        final var shortDistances = new double[1];
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver((Point3D[]) null,
                distances));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(positions,
                null));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(positions, wrong));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(shortPositions,
                shortDistances));

        // constructor with positions, distances and standard deviations
        final var standardDeviations = new double[4];
        solver = new PROMedSRobustLateration3DSolver(positions, distances, standardDeviations);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertFalse(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(null, distances,
                standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(positions, null,
                standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(positions, distances,
                (double[]) null));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(positions, wrong,
                standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(positions, distances,
                wrong));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(shortPositions,
                shortDistances, standardDeviations));

        // constructor with positions, distances, standard deviations and listener
        solver = new PROMedSRobustLateration3DSolver(positions, distances, standardDeviations, this);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertFalse(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(null, distances,
                standardDeviations, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(positions, null,
                standardDeviations, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(positions, distances,
                null, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(positions, wrong,
                standardDeviations, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(positions, distances,
                wrong, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(shortPositions,
                shortDistances, standardDeviations, this));

        // constructor with positions, distances and listener
        solver = new PROMedSRobustLateration3DSolver(positions, distances, this);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertFalse(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver((Point3D[]) null,
                distances, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(positions, null,
                this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(positions, wrong,
                this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(shortPositions,
                shortDistances, this));

        // constructor with spheres
        final var spheres = new Sphere[4];
        spheres[0] = new Sphere(positions[0], distances[0]);
        spheres[1] = new Sphere(positions[1], distances[1]);
        spheres[2] = new Sphere(positions[2], distances[2]);
        spheres[3] = new Sphere(positions[3], distances[3]);
        solver = new PROMedSRobustLateration3DSolver(spheres);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertFalse(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        final var shortSpheres = new Sphere[1];
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver((Sphere[]) null));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(shortSpheres));

        // constructor with spheres and standard deviations
        solver = new PROMedSRobustLateration3DSolver(spheres, standardDeviations);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertFalse(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver((Sphere[]) null,
                standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(spheres,
                (double[]) null));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(shortSpheres,
                standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(spheres, wrong));

        // constructor with spheres and listener
        solver = new PROMedSRobustLateration3DSolver(spheres, this);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertFalse(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver((Sphere[]) null,
                this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(shortSpheres,
                this));

        // constructor with spheres, standard deviation and listener
        solver = new PROMedSRobustLateration3DSolver(spheres, standardDeviations, this);

        // check correctness
        assertEquals(PROMedSRobustLateration2DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertFalse(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver((Sphere[]) null,
                standardDeviations, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(spheres,
                null, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(shortSpheres,
                standardDeviations, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(spheres, wrong,
                this));

        // constructor with quality scores
        final var qualityscores = new double[4];
        solver = new PROMedSRobustLateration3DSolver(qualityscores);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertSame(qualityscores, solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver((double[]) null));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(new double[2]));

        // constructor with quality scores and listener
        solver = new PROMedSRobustLateration3DSolver(qualityscores, this);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertSame(qualityscores, solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver((double[]) null,
                this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(new double[2],
                this));

        // constructor with quality scores, positions and distances
        solver = new PROMedSRobustLateration3DSolver(qualityscores, positions, distances);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertSame(qualityscores, solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        final var shortQualityScores = new double[1];
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(null,
                positions, distances));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                (Point3D[]) null, distances));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores, positions,
                null));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores, positions,
                wrong));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(shortQualityScores,
                positions, distances));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                shortPositions, shortDistances));

        // constructor with quality scores, positions and distances
        solver = new PROMedSRobustLateration3DSolver(qualityscores, positions, distances, standardDeviations);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertSame(qualityscores, solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(null,
                positions, distances, standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                null, distances, standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                positions, null, standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                positions, distances, (double[]) null));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores, positions,
                wrong, standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores, positions,
                distances, wrong));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(shortQualityScores,
                positions, distances, standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                shortPositions, shortDistances, standardDeviations));

        // constructor with quality scores, positions, distances,
        // standard deviations and listener
        solver = new PROMedSRobustLateration3DSolver(qualityscores, positions, distances, standardDeviations,
                this);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertSame(qualityscores, solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(null,
                positions, distances, standardDeviations, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                null, distances, standardDeviations, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                positions, null, standardDeviations, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                positions, distances, null, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                positions, wrong, standardDeviations, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                positions, distances, wrong, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(shortQualityScores,
                positions, distances, standardDeviations, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                shortPositions, shortDistances, standardDeviations, this));

        // constructor with quality scores, positions, distances and listener
        solver = new PROMedSRobustLateration3DSolver(qualityscores, positions, distances, this);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertSame(qualityscores, solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(null,
                positions, distances, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                (Point3D[]) null, distances, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores, positions,
                null, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores, positions,
                wrong, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(shortQualityScores,
                positions, distances, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                shortPositions, shortDistances, this));

        // constructor with quality scores and spheres
        solver = new PROMedSRobustLateration3DSolver(qualityscores, spheres);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertSame(qualityscores, solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(null,
                spheres));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                (Sphere[]) null));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(shortQualityScores,
                spheres));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                shortSpheres));

        // constructor with quality scores, spheres and standard deviations
        solver = new PROMedSRobustLateration3DSolver(qualityscores, spheres, standardDeviations);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertSame(qualityscores, solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(null,
                spheres, standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                (Sphere[]) null, standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores, spheres,
                (double[]) null));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(shortQualityScores,
                spheres, standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                shortSpheres, standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores, spheres,
                wrong));

        // constructor with quality scores, spheres and listener
        solver = new PROMedSRobustLateration3DSolver(qualityscores, spheres, this);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertSame(qualityscores, solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(null,
                spheres, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                null, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(shortQualityScores,
                spheres, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                shortSpheres, this));

        // constructor with quality scores, spheres, standard deviations and listener
        solver = new PROMedSRobustLateration3DSolver(qualityscores, spheres, standardDeviations, this);

        // check correctness
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, solver.getMethod());
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
        assertSame(qualityscores, solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(null,
                spheres, standardDeviations, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                (Sphere[]) null, standardDeviations, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores, spheres,
                null, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(shortQualityScores,
                spheres, standardDeviations, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                shortSpheres, standardDeviations, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustLateration3DSolver(qualityscores,
                spheres, wrong, this));
    }

    @Test
    void testGetSetStopThreshold() throws LockedException {
        final var solver = new PROMedSRobustLateration3DSolver();

        // check initial value
        assertEquals(PROMedSRobustLateration3DSolver.DEFAULT_STOP_THRESHOLD, solver.getStopThreshold(), 0.0);

        // set new value
        solver.setStopThreshold(1.0);

        // check
        assertEquals(1.0, solver.getStopThreshold(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> solver.setStopThreshold(0.0));
    }

    @Test
    void testGetSetSpheres() throws LockedException {
        final var randomizer = new UniformRandomizer();

        final var solver = new PROMedSRobustLateration3DSolver();

        // check initial value
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
    void testGetSetSpheresAndStandardDeviations() throws LockedException {
        final var randomizer = new UniformRandomizer();

        final var solver = new PROMedSRobustLateration3DSolver();

        // check initial value
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
        final var standardDeviations = new double[4];
        standardDeviations[0] = randomizer.nextDouble();
        standardDeviations[1] = randomizer.nextDouble();
        standardDeviations[2] = randomizer.nextDouble();
        standardDeviations[3] = randomizer.nextDouble();

        final var spheres = new Sphere[4];
        spheres[0] = new Sphere(positions[0], distances[0]);
        spheres[1] = new Sphere(positions[1], distances[1]);
        spheres[2] = new Sphere(positions[2], distances[2]);
        spheres[3] = new Sphere(positions[3], distances[3]);
        solver.setSpheresAndStandardDeviations(spheres, standardDeviations);

        // check
        final var spheres2 = solver.getSpheres();
        for (var i = 0; i < 4; i++) {
            assertSame(spheres[i].getCenter(), spheres2[i].getCenter());
            assertEquals(spheres[i].getRadius(), spheres2[i].getRadius(), 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> solver.setSpheresAndStandardDeviations(null,
                standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> solver.setSpheresAndStandardDeviations(spheres,
                null));
        assertThrows(IllegalArgumentException.class, () -> solver.setSpheresAndStandardDeviations(new Sphere[1],
                standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> solver.setSpheresAndStandardDeviations(spheres,
                new double[1]));
    }

    @Test
    void testGetSetPreliminarySubsetSize() throws LockedException {
        final var solver = new PROMedSRobustLateration3DSolver();

        // check initial value
        assertEquals(4, solver.getPreliminarySubsetSize());

        // set new value
        solver.setPreliminarySubsetSize(5);

        // check
        assertEquals(5, solver.getPreliminarySubsetSize());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> solver.setPreliminarySubsetSize(3));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var solver = new PROMedSRobustLateration3DSolver();

        // check default value
        assertNull(solver.getListener());

        // set new value
        solver.setListener(this);

        // check
        assertSame(this, solver.getListener());
    }

    @Test
    void testGetSetInitialPosition() throws LockedException {
        final var solver = new PROMedSRobustLateration3DSolver();

        // check default value
        assertNull(solver.getInitialPosition());

        // set new value
        final var p = Point3D.create();
        solver.setInitialPosition(p);

        // check
        assertSame(p, solver.getInitialPosition());
    }

    @Test
    void testIsSetLinearSolverUsed() throws LockedException {
        final var solver = new PROMedSRobustLateration3DSolver();

        // check default value
        assertTrue(solver.isLinearSolverUsed());

        // set new value
        solver.setLinearSolverUsed(false);

        // check
        assertFalse(solver.isLinearSolverUsed());
    }

    @Test
    void testIsSetHomogeneousLinearSolverUsed() throws LockedException {
        final var solver = new PROMedSRobustLateration3DSolver();

        // check default value
        assertFalse(solver.isHomogeneousLinearSolverUsed());

        // set new value
        solver.setHomogeneousLinearSolverUsed(true);

        // check
        assertTrue(solver.isHomogeneousLinearSolverUsed());
    }

    @Test
    void testIsSetPreliminarySolutionRefined() throws LockedException {
        final var solver = new PROMedSRobustLateration3DSolver();

        // check default value
        assertTrue(solver.isPreliminarySolutionRefined());

        // set new value
        solver.setPreliminarySolutionRefined(false);

        // check
        assertFalse(solver.isPreliminarySolutionRefined());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var solver = new PROMedSRobustLateration3DSolver();

        // check default value
        assertEquals(RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, solver.getProgressDelta(), 0.0);

        // set new value
        solver.setProgressDelta(0.5f);

        // check
        assertEquals(0.5f, solver.getProgressDelta(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> solver.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> solver.setProgressDelta(2.0f));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var solver = new PROMedSRobustLateration3DSolver();

        // check default value
        assertEquals(RobustLaterationSolver.DEFAULT_CONFIDENCE, solver.getConfidence(), 0.0);

        // set new value
        solver.setConfidence(0.8);

        // check
        assertEquals(0.8, solver.getConfidence(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> solver.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> solver.setConfidence(2.0));
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var solver = new PROMedSRobustLateration3DSolver();

        // check default value
        assertEquals(RobustLaterationSolver.DEFAULT_MAX_ITERATIONS, solver.getMaxIterations());

        // set new value
        solver.setMaxIterations(10);

        // check
        assertEquals(10, solver.getMaxIterations());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> solver.setMaxIterations(0));
    }

    @Test
    void testIsSetResultRefined() throws LockedException {
        final var solver = new PROMedSRobustLateration3DSolver();

        // check default value
        assertEquals(RobustLaterationSolver.DEFAULT_REFINE_RESULT, solver.isResultRefined());

        // set new value
        solver.setResultRefined(!RobustLaterationSolver.DEFAULT_REFINE_RESULT);

        // check
        assertEquals(!RobustLaterationSolver.DEFAULT_REFINE_RESULT, solver.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var solver = new PROMedSRobustLateration3DSolver();

        // check default value
        assertEquals(RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE, solver.isCovarianceKept());

        // set new value
        solver.setCovarianceKept(!RobustLaterationSolver.DEFAULT_REFINE_RESULT);

        // check
        assertEquals(!RobustLaterationSolver.DEFAULT_REFINE_RESULT, solver.isCovarianceKept());
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var solver = new PROMedSRobustLateration3DSolver();

        // check default value
        assertNull(solver.getQualityScores());

        // set new value
        final var qualityScores = new double[4];
        solver.setQualityScores(qualityScores);

        // check
        assertSame(qualityScores, solver.getQualityScores());
    }

    @Test
    void testGetSetPositionsAndDistances() throws LockedException {
        final var randomizer = new UniformRandomizer();

        final var solver = new PROMedSRobustLateration3DSolver();

        // check default value
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());

        // set new values
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

        solver.setPositionsAndDistances(positions, distances);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);

        // force IllegalArgumentException
        final var wrong = new double[5];
        final var shortPositions = new Point3D[1];
        final var shortDistances = new double[1];
        assertThrows(IllegalArgumentException.class, () -> solver.setPositionsAndDistances(null, distances));
        assertThrows(IllegalArgumentException.class, () -> solver.setPositionsAndDistances(positions, null));
        assertThrows(IllegalArgumentException.class, () -> solver.setPositionsAndDistances(positions, wrong));
        assertThrows(IllegalArgumentException.class, () -> solver.setPositionsAndDistances(shortPositions,
                shortDistances));
    }

    @Test
    void testGetSetPositionsDistancesAndStandardDeviations() throws LockedException {
        final var randomizer = new UniformRandomizer();

        final var solver = new PROMedSRobustLateration3DSolver();

        // check default value
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());

        // set new values
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
        final var standardDeviations = new double[4];
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
        final var wrong = new double[5];
        final var shortPositions = new Point3D[1];
        final var shortDistances = new double[1];
        final var shortStandardDeviations = new double[1];
        assertThrows(IllegalArgumentException.class, () -> solver.setPositionsDistancesAndStandardDeviations(
                null, distances, standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> solver.setPositionsDistancesAndStandardDeviations(positions,
                null, standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> solver.setPositionsDistancesAndStandardDeviations(positions,
                distances, null));
        assertThrows(IllegalArgumentException.class, () -> solver.setPositionsDistancesAndStandardDeviations(positions,
                wrong, standardDeviations));
        assertThrows(IllegalArgumentException.class, () -> solver.setPositionsDistancesAndStandardDeviations(positions,
                distances, wrong));
        assertThrows(IllegalArgumentException.class, () -> solver.setPositionsDistancesAndStandardDeviations(
                shortPositions, shortDistances, shortStandardDeviations));
    }

    @Test
    void testSolveNoInlierErrorNoRefinement() throws Exception {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var spheres = new Sphere[numSpheres];
            final var qualityScores = new double[numSpheres];
            for (var i = 0; i < numSpheres; i++) {
                final var center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                var radius = center.distanceTo(position);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            PROMedSRobustLateration3DSolver solver = new PROMedSRobustLateration3DSolver(qualityScores,
                    spheres, this);
            solver.setResultRefined(false);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertEquals(0, solveNextIteration);
            assertEquals(0, solveProgressChange);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            final var estimatedPosition = solver.solve();

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
            solver = new PROMedSRobustLateration3DSolver();
            assertThrows(NotReadyException.class, solver::solve);

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSolveNoInlierErrorWithRefinement() throws Exception {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var spheres = new Sphere[numSpheres];
            final var qualityScores = new double[numSpheres];
            for (var i = 0; i < numSpheres; i++) {
                final var center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                var radius = center.distanceTo(position);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            var solver = new PROMedSRobustLateration3DSolver(qualityScores, spheres, this);
            solver.setResultRefined(true);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertEquals(0, solveNextIteration);
            assertEquals(0, solveProgressChange);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            final var estimatedPosition = solver.solve();

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
            solver = new PROMedSRobustLateration3DSolver();
            assertThrows(NotReadyException.class, solver::solve);

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSolveWithInlierErrorWithRefinement() throws Exception {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var spheres = new Sphere[numSpheres];
            final var qualityScores = new double[numSpheres];
            for (var i = 0; i < numSpheres; i++) {
                final var center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                var radius = center.distanceTo(position);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                error += randomizer.nextDouble(MIN_DISTANCE_ERROR, MAX_DISTANCE_ERROR);
                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            var solver = new PROMedSRobustLateration3DSolver(qualityScores, spheres, this);
            solver.setResultRefined(true);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertEquals(0, solveNextIteration);
            assertEquals(0, solveProgressChange);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            final var estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            if (solver.getInliersData() != null) {
                assertNotNull(solver.getInliersData());
                assertNotNull(solver.getInliersData().getInliers());
                assertNotNull(solver.getInliersData().getResiduals());
            }

            assertEquals(1, solveStart);
            assertEquals(1, solveEnd);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new PROMedSRobustLateration3DSolver();
            assertThrows(NotReadyException.class, solver::solve);

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSolveWithInlierErrorWithRefinementAndStandardDeviations() throws Exception {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var spheres = new Sphere[numSpheres];
            final var standardDeviations = new double[numSpheres];
            final var qualityScores = new double[numSpheres];
            for (var i = 0; i < numSpheres; i++) {
                final var center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                var radius = center.distanceTo(position);

                double error;
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
                qualityScores[i] = 1.0 / (1.0 + Math.abs(error));
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            var solver = new PROMedSRobustLateration3DSolver(qualityScores, spheres, this);
            solver.setResultRefined(true);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertEquals(0, solveNextIteration);
            assertEquals(0, solveProgressChange);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            final var estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            if (solver.getInliersData() != null) {
                assertNotNull(solver.getInliersData());
                assertNotNull(solver.getInliersData().getInliers());
                assertNotNull(solver.getInliersData().getResiduals());
            }

            assertEquals(1, solveStart);
            assertEquals(1, solveEnd);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new PROMedSRobustLateration3DSolver();
            assertThrows(NotReadyException.class, solver::solve);

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSolveNoPreliminaryLinearSolver() throws Exception {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var spheres = new Sphere[numSpheres];
            final var qualityScores = new double[numSpheres];
            for (var i = 0; i < numSpheres; i++) {
                final var center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                var radius = center.distanceTo(position);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            var solver = new PROMedSRobustLateration3DSolver(qualityScores, spheres, this);
            solver.setLinearSolverUsed(false);

            reset();
            assertEquals(0, solveStart);
            assertEquals(0, solveEnd);
            assertEquals(0, solveNextIteration);
            assertEquals(0, solveProgressChange);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            final var estimatedPosition = solver.solve();

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
            solver = new PROMedSRobustLateration3DSolver();
            assertThrows(NotReadyException.class, solver::solve);

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSolveInhomogeneousPreliminaryLinearSolver() throws Exception {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var spheres = new Sphere[numSpheres];
            final var qualityScores = new double[numSpheres];
            for (var i = 0; i < numSpheres; i++) {
                final var center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                var radius = center.distanceTo(position);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            var solver = new PROMedSRobustLateration3DSolver(qualityScores, spheres, this);
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

            final var estimatedPosition = solver.solve();

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
            solver = new PROMedSRobustLateration3DSolver();
            assertThrows(NotReadyException.class, solver::solve);

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSolveHomogeneousPreliminaryLinearSolver() throws Exception {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var spheres = new Sphere[numSpheres];
            final var qualityScores = new double[numSpheres];
            for (var i = 0; i < numSpheres; i++) {
                final var center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                var radius = center.distanceTo(position);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            var solver = new PROMedSRobustLateration3DSolver(qualityScores, spheres, this);
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

            final var estimatedPosition = solver.solve();

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
            solver = new PROMedSRobustLateration3DSolver();
            assertThrows(NotReadyException.class, solver::solve);

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSolveRefinePreliminarySolutions() throws Exception {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var spheres = new Sphere[numSpheres];
            final var qualityScores = new double[numSpheres];
            for (var i = 0; i < numSpheres; i++) {
                final var center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                var radius = center.distanceTo(position);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            var solver = new PROMedSRobustLateration3DSolver(qualityScores, spheres, this);
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

            final var estimatedPosition = solver.solve();

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
            solver = new PROMedSRobustLateration3DSolver();
            assertThrows(NotReadyException.class, solver::solve);

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSolvePreliminarySolutionsNotRefined() throws Exception {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var spheres = new Sphere[numSpheres];
            final var qualityScores = new double[numSpheres];
            for (var i = 0; i < numSpheres; i++) {
                final var center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                var radius = center.distanceTo(position);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            var solver = new PROMedSRobustLateration3DSolver(qualityScores, spheres, this);
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

            final var estimatedPosition = solver.solve();

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
            solver = new PROMedSRobustLateration3DSolver();
            assertThrows(NotReadyException.class, solver::solve);

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSolveNoPreliminaryLinearSolverAndNoPreliminarySolutionsRefinement() throws Exception {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var spheres = new Sphere[numSpheres];
            final var qualityScores = new double[numSpheres];
            for (var i = 0; i < numSpheres; i++) {
                final var center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                var radius = center.distanceTo(position);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            var solver = new PROMedSRobustLateration3DSolver(qualityScores, spheres, this);
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

            final var estimatedPosition = solver.solve();

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
            solver = new PROMedSRobustLateration3DSolver();
            assertThrows(NotReadyException.class, solver::solve);

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSolveNoPreliminaryLinearSolverWithInitialPosition() throws Exception {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var spheres = new Sphere[numSpheres];
            final var qualityScores = new double[numSpheres];
            for (var i = 0; i < numSpheres; i++) {
                final var center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                var radius = center.distanceTo(position);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            var solver = new PROMedSRobustLateration3DSolver(qualityScores, spheres, this);
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

            final var estimatedPosition = solver.solve();

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
            solver = new PROMedSRobustLateration3DSolver();
            assertThrows(NotReadyException.class, solver::solve);

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSolveLargerPreliminarySubsetSize() throws Exception {
        final var randomizer = new UniformRandomizer();
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_OUTLIER_ERROR);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            final var position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var spheres = new Sphere[numSpheres];
            final var qualityScores = new double[numSpheres];
            for (var i = 0; i < numSpheres; i++) {
                final var center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                var radius = center.distanceTo(position);

                double error;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);
                radius = Math.max(RobustLaterationSolver.EPSILON, radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            var solver = new PROMedSRobustLateration3DSolver(qualityScores, spheres, this);
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

            final var estimatedPosition = solver.solve();

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
            solver = new PROMedSRobustLateration3DSolver();
            assertThrows(NotReadyException.class, solver::solve);

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onSolveStart(final RobustLaterationSolver<Point3D> solver) {
        solveStart++;
        checkLocked((PROMedSRobustLateration3DSolver) solver);
    }

    @Override
    public void onSolveEnd(final RobustLaterationSolver<Point3D> solver) {
        solveEnd++;
        checkLocked((PROMedSRobustLateration3DSolver) solver);
    }

    @Override
    public void onSolveNextIteration(final RobustLaterationSolver<Point3D> solver, final int iteration) {
        solveNextIteration++;
        checkLocked((PROMedSRobustLateration3DSolver) solver);
    }

    @Override
    public void onSolveProgressChange(final RobustLaterationSolver<Point3D> solver, final float progress) {
        solveProgressChange++;
        checkLocked((PROMedSRobustLateration3DSolver) solver);
    }

    private void reset() {
        solveStart = solveEnd = solveNextIteration = solveProgressChange = 0;
    }

    private static void checkLocked(final PROMedSRobustLateration3DSolver solver) {
        assertThrows(LockedException.class, () -> solver.setPreliminarySubsetSize(4));
        assertThrows(LockedException.class, () -> solver.setListener(null));
        assertThrows(LockedException.class, () -> solver.setInitialPosition(null));
        assertThrows(LockedException.class, () -> solver.setLinearSolverUsed(true));
        assertThrows(LockedException.class, () -> solver.setHomogeneousLinearSolverUsed(true));
        assertThrows(LockedException.class, () -> solver.setPreliminarySolutionRefined(true));
        assertThrows(LockedException.class, () -> solver.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> solver.setConfidence(0.5));
        assertThrows(LockedException.class, () -> solver.setMaxIterations(5));
        assertThrows(LockedException.class, () -> solver.setResultRefined(false));
        assertThrows(LockedException.class, () -> solver.setCovarianceKept(false));
        assertThrows(LockedException.class, () -> solver.setPositionsAndDistances(null, null));
        assertThrows(LockedException.class, () -> solver.setPositionsDistancesAndStandardDeviations(
                null, null, null));
        assertThrows(LockedException.class, () -> solver.setSpheres(null));
        assertThrows(LockedException.class, () -> solver.setSpheresAndStandardDeviations(null,
                null));
        assertThrows(LockedException.class, () -> solver.setStopThreshold(0.5));
        assertThrows(LockedException.class, () -> solver.setQualityScores(null));
        assertThrows(LockedException.class, solver::solve);
    }
}

