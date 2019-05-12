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
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

@SuppressWarnings("Duplicates")
public class MSACRobustLateration3DSolverTest implements
        RobustLaterationSolverListener<Point3D> {

    private static final int MIN_SPHERES = 100;
    private static final int MAX_SPHERES = 500;

    private static final double MIN_RANDOM_VALUE = -50.0;
    private static final double MAX_RANDOM_VALUE = 50.0;

    private static final double MIN_DISTANCE_ERROR = -1e-2;
    private static final double MAX_DISTANCE_ERROR = 1e-2;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final int TIMES = 100;

    private static final int PERCENTAGE_OUTLIERS = 20;

    private static final double STD_OUTLIER_ERROR = 10.0;

    private int solveStart;
    private int solveEnd;
    private int solveNextIteration;
    private int solveProgressChange;

    public MSACRobustLateration3DSolverTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        // empty constructor
        MSACRobustLateration3DSolver solver = new MSACRobustLateration3DSolver();

        // check correctness
        assertEquals(solver.getThreshold(),
                MSACRobustLateration3DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);
        assertEquals(solver.getPreliminarySubsetSize(), 4);
        assertNull(solver.getSpheres());
        assertNull(solver.getListener());
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(solver.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(solver.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(solver.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertNull(solver.getInliersData());
        assertEquals(solver.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(solver.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertNull(solver.getDistanceStandardDeviations());
        assertFalse(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // constructor with listener
        solver = new MSACRobustLateration3DSolver(this);

        // check correctness
        assertEquals(solver.getThreshold(),
                MSACRobustLateration3DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);
        assertEquals(solver.getPreliminarySubsetSize(), 4);
        assertNull(solver.getSpheres());
        assertSame(solver.getListener(), this);
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(solver.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(solver.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(solver.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertNull(solver.getInliersData());
        assertEquals(solver.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(solver.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertNull(solver.getDistanceStandardDeviations());
        assertFalse(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());


        // constructor with positions and distances
        Point3D[] positions = new Point3D[4];
        positions[0] = new InhomogeneousPoint3D();
        positions[1] = new InhomogeneousPoint3D();
        positions[2] = new InhomogeneousPoint3D();
        positions[3] = new InhomogeneousPoint3D();
        double[] distances = new double[4];
        solver = new MSACRobustLateration3DSolver(positions, distances);

        // check correctness
        assertEquals(solver.getThreshold(),
                MSACRobustLateration3DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);
        assertEquals(solver.getPreliminarySubsetSize(), 4);
        assertNotNull(solver.getSpheres());
        assertNull(solver.getListener());
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(solver.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(solver.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(solver.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertNull(solver.getInliersData());
        assertEquals(solver.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(solver.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertNull(solver.getDistanceStandardDeviations());
        assertTrue(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        double[] wrong = new double[5];
        Point3D[] shortPositions = new Point3D[1];
        double[] shortDistances = new double[1];
        solver = null;
        try {
            solver = new MSACRobustLateration3DSolver((Point3D[])null, distances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(positions, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(positions, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(shortPositions, shortDistances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        // constructor with positions, distances and standard deviations
        double[] standardDeviations = new double[4];
        solver = new MSACRobustLateration3DSolver(positions, distances,
                standardDeviations);

        // check correctness
        assertEquals(solver.getThreshold(),
                MSACRobustLateration3DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);
        assertEquals(solver.getPreliminarySubsetSize(), 4);
        assertNotNull(solver.getSpheres());
        assertNull(solver.getListener());
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(solver.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(solver.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(solver.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertNull(solver.getInliersData());
        assertEquals(solver.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(solver.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        solver = null;
        try {
            solver = new MSACRobustLateration3DSolver(null, distances,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(positions, null,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(positions, distances,
                    (double[]) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(positions, wrong,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(positions, distances,
                    wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(shortPositions,
                    shortDistances, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        // constructor with positions, distances, standard deviations and listener
        solver = new MSACRobustLateration3DSolver(positions, distances,
                standardDeviations, this);

        // check correctness
        assertEquals(solver.getThreshold(),
                MSACRobustLateration3DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);
        assertEquals(solver.getPreliminarySubsetSize(), 4);
        assertNotNull(solver.getSpheres());
        assertSame(solver.getListener(), this);
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(solver.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(solver.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(solver.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertNull(solver.getInliersData());
        assertEquals(solver.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(solver.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        solver = null;
        try {
            solver = new MSACRobustLateration3DSolver(null, distances,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(positions, null,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(positions, distances,
                    null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(positions, wrong,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(positions, distances,
                    wrong, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(shortPositions,
                    shortDistances, standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        // constructor with positions, distances and listener
        solver = new MSACRobustLateration3DSolver(positions, distances,
                this);

        // check correctness
        assertEquals(solver.getThreshold(),
                MSACRobustLateration3DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);
        assertEquals(solver.getPreliminarySubsetSize(), 4);
        assertNotNull(solver.getSpheres());
        assertSame(solver.getListener(), this);
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(solver.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(solver.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(solver.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertNull(solver.getInliersData());
        assertEquals(solver.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(solver.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertNull(solver.getDistanceStandardDeviations());
        assertTrue(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        solver = null;
        try {
            solver = new MSACRobustLateration3DSolver((Point3D[])null, distances,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(positions, null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(positions, wrong,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(shortPositions,
                    shortDistances, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        // constructor with circles
        Sphere[] spheres = new Sphere[4];
        spheres[0] = new Sphere(positions[0], distances[0]);
        spheres[1] = new Sphere(positions[1], distances[1]);
        spheres[2] = new Sphere(positions[2], distances[2]);
        spheres[3] = new Sphere(positions[3], distances[3]);
        solver = new MSACRobustLateration3DSolver(spheres);

        // check correctness
        assertEquals(solver.getThreshold(),
                MSACRobustLateration3DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);
        assertEquals(solver.getPreliminarySubsetSize(), 4);
        assertNotNull(solver.getSpheres());
        assertNull(solver.getListener());
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(solver.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(solver.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(solver.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertNull(solver.getInliersData());
        assertEquals(solver.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(solver.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertNull(solver.getDistanceStandardDeviations());
        assertTrue(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        Sphere[] shortSpheres = new Sphere[1];

        solver = null;
        try {
            solver = new MSACRobustLateration3DSolver((Sphere[])null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(shortSpheres);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        // constructor with circles and standard deviations
        solver = new MSACRobustLateration3DSolver(spheres,
                standardDeviations);

        // check correctness
        assertEquals(solver.getThreshold(),
                MSACRobustLateration3DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);
        assertEquals(solver.getPreliminarySubsetSize(), 4);
        assertNotNull(solver.getSpheres());
        assertNull(solver.getListener());
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(solver.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(solver.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(solver.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertNull(solver.getInliersData());
        assertEquals(solver.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(solver.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertSame(solver.getDistanceStandardDeviations(),
                standardDeviations);
        assertTrue(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        solver = null;
        try {
            solver = new MSACRobustLateration3DSolver((Sphere[])null,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(spheres,
                    (double[]) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(shortSpheres,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(spheres, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        // constructor with circles and listener
        solver = new MSACRobustLateration3DSolver(spheres, this);

        // check correctness
        assertEquals(solver.getThreshold(),
                MSACRobustLateration3DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);
        assertEquals(solver.getPreliminarySubsetSize(), 4);
        assertNotNull(solver.getSpheres());
        assertSame(solver.getListener(), this);
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(solver.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(solver.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(solver.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertNull(solver.getInliersData());
        assertEquals(solver.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(solver.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
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
            solver = new MSACRobustLateration3DSolver(null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(shortSpheres,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        // constructor with circles, standard deviation and listener
        solver = new MSACRobustLateration3DSolver(spheres,
                standardDeviations, this);

        // check correctness
        assertEquals(solver.getThreshold(),
                MSACRobustLateration3DSolver.DEFAULT_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 4);
        assertEquals(solver.getPreliminarySubsetSize(), 4);
        assertNotNull(solver.getSpheres());
        assertSame(solver.getListener(), this);
        assertNull(solver.getInitialPosition());
        assertTrue(solver.isLinearSolverUsed());
        assertFalse(solver.isHomogeneousLinearSolverUsed());
        assertTrue(solver.isPreliminarySolutionRefined());
        assertFalse(solver.isLocked());
        assertEquals(solver.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(solver.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(solver.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);
        assertNull(solver.getInliersData());
        assertEquals(solver.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);
        assertEquals(solver.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertSame(solver.getDistanceStandardDeviations(),
                standardDeviations);
        assertTrue(solver.isReady());
        assertNull(solver.getQualityScores());
        assertNull(solver.getCovariance());
        assertNull(solver.getEstimatedPosition());

        // force IllegalArgumentException
        solver = null;
        try {
            solver = new MSACRobustLateration3DSolver((Sphere[])null,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(spheres,
                    null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(shortSpheres,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new MSACRobustLateration3DSolver(spheres, wrong,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        MSACRobustLateration3DSolver solver =
                new MSACRobustLateration3DSolver();

        // check initial value
        assertEquals(solver.getThreshold(),
                MSACRobustLateration3DSolver.DEFAULT_THRESHOLD, 0.0);

        // set new value
        solver.setThreshold(1.0);

        // check
        assertEquals(solver.getThreshold(), 1.0, 0.0);

        // force IllegalArgumentException
        try {
            solver.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetCircles() throws LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        MSACRobustLateration3DSolver solver =
                new MSACRobustLateration3DSolver();

        // check initial value
        assertNull(solver.getSpheres());

        // set new value
        Point3D[] positions = new Point3D[4];
        positions[0] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        positions[1] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        positions[2] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        positions[3] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        double[] distances = new double[4];
        distances[0] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[1] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[2] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[3] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);

        Sphere[] spheres = new Sphere[4];
        spheres[0] = new Sphere(positions[0], distances[0]);
        spheres[1] = new Sphere(positions[1], distances[1]);
        spheres[2] = new Sphere(positions[2], distances[2]);
        spheres[3] = new Sphere(positions[3], distances[3]);
        solver.setSpheres(spheres);

        // check
        Sphere[] spheres2 = solver.getSpheres();
        for (int i = 0; i < 4; i++) {
            assertSame(spheres[i].getCenter(), spheres2[i].getCenter());
            assertEquals(spheres[i].getRadius(), spheres2[i].getRadius(), 0.0);
        }

        // force IllegalArgumentException
        try {
            solver.setSpheres(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setSpheres(new Sphere[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetCirclesAndStandardDeviations() throws LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        MSACRobustLateration3DSolver solver =
                new MSACRobustLateration3DSolver();

        // check initial value
        assertNull(solver.getSpheres());

        // set new value
        Point3D[] positions = new Point3D[4];
        positions[0] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        positions[1] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        positions[2] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        positions[3] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        double[] distances = new double[4];
        distances[0] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[1] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[2] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[3] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        double[] standardDeviations = new double[4];
        standardDeviations[0] = randomizer.nextDouble();
        standardDeviations[1] = randomizer.nextDouble();
        standardDeviations[2] = randomizer.nextDouble();
        standardDeviations[3] = randomizer.nextDouble();

        Sphere[] spheres = new Sphere[4];
        spheres[0] = new Sphere(positions[0], distances[0]);
        spheres[1] = new Sphere(positions[1], distances[1]);
        spheres[2] = new Sphere(positions[2], distances[2]);
        spheres[3] = new Sphere(positions[3], distances[3]);
        solver.setSpheresAndStandardDeviations(spheres, standardDeviations);

        // check
        Sphere[] spheres2 = solver.getSpheres();
        for (int i = 0; i < 4; i++) {
            assertSame(spheres[i].getCenter(), spheres2[i].getCenter());
            assertEquals(spheres[i].getRadius(), spheres2[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(),
                standardDeviations);

        // force IllegalArgumentException
        try {
            solver.setSpheresAndStandardDeviations(null,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setSpheresAndStandardDeviations(spheres,
                    null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setSpheresAndStandardDeviations(new Sphere[1],
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setSpheresAndStandardDeviations(spheres,
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetPreliminarySubsetSize() throws LockedException {
        MSACRobustLateration3DSolver solver =
                new MSACRobustLateration3DSolver();

        // check initial value
        assertEquals(solver.getPreliminarySubsetSize(), 4);

        // set new value
        solver.setPreliminarySubsetSize(5);

        // check
        assertEquals(solver.getPreliminarySubsetSize(), 5);

        // force IllegalArgumentException
        try {
            solver.setPreliminarySubsetSize(3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        MSACRobustLateration3DSolver solver =
                new MSACRobustLateration3DSolver();

        // check default value
        assertNull(solver.getListener());

        // set new value
        solver.setListener(this);

        // check
        assertSame(solver.getListener(), this);
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        MSACRobustLateration3DSolver solver =
                new MSACRobustLateration3DSolver();

        // check default value
        assertNull(solver.getInitialPosition());

        // set new value
        Point3D p = Point3D.create();
        solver.setInitialPosition(p);

        // check
        assertSame(solver.getInitialPosition(), p);
    }

    @Test
    public void testIsSetLinearSolverUsed() throws LockedException {
        MSACRobustLateration3DSolver solver =
                new MSACRobustLateration3DSolver();

        // check default value
        assertTrue(solver.isLinearSolverUsed());

        // set new value
        solver.setLinearSolverUsed(false);

        // check
        assertFalse(solver.isLinearSolverUsed());
    }

    @Test
    public void testIsSetHomogeneousLinearSolverUsed() throws LockedException {
        MSACRobustLateration3DSolver solver =
                new MSACRobustLateration3DSolver();

        // check default value
        assertFalse(solver.isHomogeneousLinearSolverUsed());

        // set new value
        solver.setHomogeneousLinearSolverUsed(true);

        // check
        assertTrue(solver.isHomogeneousLinearSolverUsed());
    }

    @Test
    public void testIsSetPreliminarySolutionRefined() throws LockedException {
        MSACRobustLateration3DSolver solver =
                new MSACRobustLateration3DSolver();

        // check default value
        assertTrue(solver.isPreliminarySolutionRefined());

        // set new value
        solver.setPreliminarySolutionRefined(false);

        // check
        assertFalse(solver.isPreliminarySolutionRefined());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        MSACRobustLateration3DSolver solver =
                new MSACRobustLateration3DSolver();

        // check default value
        assertEquals(solver.getProgressDelta(),
                RobustLaterationSolver.DEFAULT_PROGRESS_DELTA, 0.0);

        // set new value
        solver.setProgressDelta(0.5f);

        // check
        assertEquals(solver.getProgressDelta(), 0.5f, 0.0);

        // force IllegalArgumentException
        try {
            solver.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetConfidence() throws LockedException {
        MSACRobustLateration3DSolver solver =
                new MSACRobustLateration3DSolver();

        // check default value
        assertEquals(solver.getConfidence(),
                RobustLaterationSolver.DEFAULT_CONFIDENCE, 0.0);

        // set new value
        solver.setConfidence(0.8);

        // check
        assertEquals(solver.getConfidence(), 0.8, 0.0);

        // force IllegalArgumentException
        try {
            solver.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetMaxIterations() throws LockedException {
        MSACRobustLateration3DSolver solver =
                new MSACRobustLateration3DSolver();

        // check default value
        assertEquals(solver.getMaxIterations(),
                RobustLaterationSolver.DEFAULT_MAX_ITERATIONS);

        // set new value
        solver.setMaxIterations(10);

        // check
        assertEquals(solver.getMaxIterations(), 10);

        // force IllegalArgumentException
        try {
            solver.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        MSACRobustLateration3DSolver solver =
                new MSACRobustLateration3DSolver();

        // check default value
        assertEquals(solver.isResultRefined(),
                RobustLaterationSolver.DEFAULT_REFINE_RESULT);

        // set new value
        solver.setResultRefined(
                !RobustLaterationSolver.DEFAULT_REFINE_RESULT);

        // check
        assertEquals(solver.isResultRefined(),
                !RobustLaterationSolver.DEFAULT_REFINE_RESULT);
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        MSACRobustLateration3DSolver solver =
                new MSACRobustLateration3DSolver();

        // check default value
        assertEquals(solver.isCovarianceKept(),
                RobustLaterationSolver.DEFAULT_KEEP_COVARIANCE);

        // set new value
        solver.setCovarianceKept(
                !RobustLaterationSolver.DEFAULT_REFINE_RESULT);

        // check
        assertEquals(solver.isCovarianceKept(),
                !RobustLaterationSolver.DEFAULT_REFINE_RESULT);
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        MSACRobustLateration3DSolver solver =
                new MSACRobustLateration3DSolver();

        // check default value
        assertNull(solver.getQualityScores());

        // set new value
        solver.setQualityScores(new double[3]);

        // check
        assertNull(solver.getQualityScores());
    }

    @Test
    public void testGetSetPositionsAndDistances() throws LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        MSACRobustLateration3DSolver solver =
                new MSACRobustLateration3DSolver();

        // check default value
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());

        // set new values
        Point3D[] positions = new Point3D[4];
        positions[0] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        positions[1] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        positions[2] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        positions[3] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        double[] distances = new double[4];
        distances[0] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[1] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[2] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[3] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);

        solver.setPositionsAndDistances(positions, distances);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);

        // force IllegalArgumentException
        double[] wrong = new double[5];
        Point3D[] shortPositions = new Point3D[1];
        double[] shortDistances = new double[1];
        try {
            solver.setPositionsAndDistances(null, distances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setPositionsAndDistances(positions, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setPositionsAndDistances(positions, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setPositionsAndDistances(shortPositions, shortDistances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetPositionsDistancesAndStandardDeviations() throws LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        MSACRobustLateration3DSolver solver =
                new MSACRobustLateration3DSolver();

        // check default value
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());

        // set new values
        Point3D[] positions = new Point3D[4];
        positions[0] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        positions[1] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        positions[2] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        positions[3] = new InhomogeneousPoint3D(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        double[] distances = new double[4];
        distances[0] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[1] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[2] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[3] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        double[] standardDeviations = new double[4];
        standardDeviations[0] = randomizer.nextDouble();
        standardDeviations[1] = randomizer.nextDouble();
        standardDeviations[2] = randomizer.nextDouble();
        standardDeviations[3] = randomizer.nextDouble();

        solver.setPositionsDistancesAndStandardDeviations(
                positions, distances, standardDeviations);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(),
                standardDeviations);

        // force IllegalArgumentException
        double[] wrong = new double[5];
        Point3D[] shortPositions = new Point3D[1];
        double[] shortDistances = new double[1];
        double[] shortStandardDeviations = new double[1];
        try {
            solver.setPositionsDistancesAndStandardDeviations(
                    null, distances, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setPositionsDistancesAndStandardDeviations(
                    positions, null, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setPositionsDistancesAndStandardDeviations(
                    positions, distances, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setPositionsDistancesAndStandardDeviations(
                    positions, wrong, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setPositionsDistancesAndStandardDeviations(
                    positions, distances, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setPositionsDistancesAndStandardDeviations(
                    shortPositions, shortDistances, shortStandardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testSolveNoInlierErrorNoRefinement() throws Exception {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON,
                        radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            MSACRobustLateration3DSolver solver =
                    new MSACRobustLateration3DSolver(spheres, this);
            solver.setResultRefined(false);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new MSACRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (NotReadyException ignore) { }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveNoInlierErrorWithRefinement() throws Exception {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON,
                        radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            MSACRobustLateration3DSolver solver =
                    new MSACRobustLateration3DSolver(spheres, this);
            solver.setResultRefined(true);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new MSACRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (NotReadyException ignore) { }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveWithInlierErrorWithRefinement() throws Exception {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                error += randomizer.nextDouble(MIN_DISTANCE_ERROR, MAX_DISTANCE_ERROR);
                radius = Math.max(RobustLaterationSolver.EPSILON,
                        radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            MSACRobustLateration3DSolver solver =
                    new MSACRobustLateration3DSolver(spheres, this);
            solver.setResultRefined(true);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new MSACRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (NotReadyException ignore) { }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveWithInlierErrorWithRefinementAndStandardDeviatons()
            throws Exception {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            Sphere[] spheres = new Sphere[numSpheres];
            double[] standardDeviations = new double[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
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
                radius = Math.max(RobustLaterationSolver.EPSILON,
                        radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            MSACRobustLateration3DSolver solver =
                    new MSACRobustLateration3DSolver(spheres, this);
            solver.setResultRefined(true);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new MSACRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (NotReadyException ignore) { }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveNoPreliminaryLinearSolver() throws Exception {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON,
                        radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            MSACRobustLateration3DSolver solver =
                    new MSACRobustLateration3DSolver(spheres, this);
            solver.setLinearSolverUsed(false);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new MSACRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (NotReadyException ignore) { }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveInhomogeneousPreliminaryLinearSolver() throws Exception {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON,
                        radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            MSACRobustLateration3DSolver solver =
                    new MSACRobustLateration3DSolver(spheres, this);
            solver.setLinearSolverUsed(true);
            solver.setHomogeneousLinearSolverUsed(false);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new MSACRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (NotReadyException ignore) { }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveHomogeneousPreliminaryLinearSolver() throws Exception {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON,
                        radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            MSACRobustLateration3DSolver solver =
                    new MSACRobustLateration3DSolver(spheres, this);
            solver.setLinearSolverUsed(true);
            solver.setHomogeneousLinearSolverUsed(true);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new MSACRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (NotReadyException ignore) { }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveRefinePreliminarySolutions() throws Exception {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON,
                        radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            MSACRobustLateration3DSolver solver =
                    new MSACRobustLateration3DSolver(spheres, this);
            solver.setLinearSolverUsed(true);
            solver.setPreliminarySolutionRefined(true);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new MSACRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (NotReadyException ignore) { }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolvePreliminarySolutionsNotRefined() throws Exception {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON,
                        radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            MSACRobustLateration3DSolver solver =
                    new MSACRobustLateration3DSolver(spheres, this);
            solver.setLinearSolverUsed(true);
            solver.setPreliminarySolutionRefined(false);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new MSACRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (NotReadyException ignore) { }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveNoPreliminaryLinearSolverAndNoPreliminarySolutionsRefinement()
            throws Exception {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON,
                        radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            MSACRobustLateration3DSolver solver =
                    new MSACRobustLateration3DSolver(spheres, this);
            solver.setLinearSolverUsed(false);
            solver.setPreliminarySolutionRefined(false);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new MSACRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (NotReadyException ignore) { }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveNoPreliminaryLinearSolverWithInitialPosition() throws Exception {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON,
                        radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            MSACRobustLateration3DSolver solver =
                    new MSACRobustLateration3DSolver(spheres, this);
            solver.setLinearSolverUsed(false);
            solver.setInitialPosition(position);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new MSACRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (NotReadyException ignore) { }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSolveLargerPreliminarySubsetSize() throws Exception {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_OUTLIER_ERROR);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numSpheres = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius, error;
            Sphere[] spheres = new Sphere[numSpheres];
            for (int i = 0; i < numSpheres; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);

                if(randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    error = errorRandomizer.nextDouble();
                } else {
                    // inlier
                    error = 0.0;
                }
                radius = Math.max(RobustLaterationSolver.EPSILON,
                        radius + error);
                spheres[i] = new Sphere(center, radius);
            }

            MSACRobustLateration3DSolver solver =
                    new MSACRobustLateration3DSolver(spheres, this);
            solver.setResultRefined(false);
            solver.setPreliminarySubsetSize(5);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point3D estimatedPosition = solver.solve();

            // check
            if (!position.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNull(solver.getCovariance());
            assertNotNull(solver.getInliersData());
            assertNotNull(solver.getInliersData().getInliers());
            assertNotNull(solver.getInliersData().getResiduals());

            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);
            assertTrue(solveNextIteration > 0);
            assertTrue(solveProgressChange >= 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());

            // force NotReadyException
            solver = new MSACRobustLateration3DSolver();

            try {
                solver.solve();
                fail("LockedException expected but not thrown");
            } catch (NotReadyException ignore) { }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onSolveStart(RobustLaterationSolver<Point3D> solver) {
        solveStart++;
        checkLocked((MSACRobustLateration3DSolver)solver);
    }

    @Override
    public void onSolveEnd(RobustLaterationSolver<Point3D> solver) {
        solveEnd++;
        checkLocked((MSACRobustLateration3DSolver)solver);
    }

    @Override
    public void onSolveNextIteration(RobustLaterationSolver<Point3D> solver, int iteration) {
        solveNextIteration++;
        checkLocked((MSACRobustLateration3DSolver)solver);
    }

    @Override
    public void onSolveProgressChange(RobustLaterationSolver<Point3D> solver, float progress) {
        solveProgressChange++;
        checkLocked((MSACRobustLateration3DSolver)solver);
    }

    private void reset() {
        solveStart = solveEnd = solveNextIteration =
                solveProgressChange = 0;
    }

    private void checkLocked(MSACRobustLateration3DSolver solver) {
        try {
            solver.setPreliminarySubsetSize(3);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setInitialPosition(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setLinearSolverUsed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setHomogeneousLinearSolverUsed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setPreliminarySolutionRefined(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setMaxIterations(5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setResultRefined(false);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setCovarianceKept(false);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setPositionsAndDistances(null, null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setPositionsDistancesAndStandardDeviations(
                    null, null, null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }

        try {
            solver.setSpheres(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setSpheresAndStandardDeviations(null, null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.solve();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception ignore) {
            fail("LockedException expected but not thrown");
        }
    }
}
