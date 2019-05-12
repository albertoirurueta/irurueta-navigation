package com.irurueta.navigation.lateration;

import static org.junit.Assert.*;

import com.irurueta.geometry.Circle;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

@SuppressWarnings("Duplicates")
public class LMedSRobustLateration2DSolverTest implements
        RobustLaterationSolverListener<Point2D> {

    private static final int MIN_CIRCLES = 100;
    private static final int MAX_CIRCLES = 500;

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

    public LMedSRobustLateration2DSolverTest() { }

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
        LMedSRobustLateration2DSolver solver = new LMedSRobustLateration2DSolver();

        // check correctness
        assertEquals(solver.getStopThreshold(),
                LMedSRobustLateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 3);
        assertEquals(solver.getPreliminarySubsetSize(), 3);
        assertNull(solver.getCircles());
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
        solver = new LMedSRobustLateration2DSolver(this);

        // check correctness
        assertEquals(solver.getStopThreshold(),
                LMedSRobustLateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 3);
        assertEquals(solver.getPreliminarySubsetSize(), 3);
        assertNull(solver.getCircles());
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
        Point2D[] positions = new Point2D[3];
        positions[0] = new InhomogeneousPoint2D();
        positions[1] = new InhomogeneousPoint2D();
        positions[2] = new InhomogeneousPoint2D();
        double[] distances = new double[3];
        solver = new LMedSRobustLateration2DSolver(positions, distances);

        // check correctness
        assertEquals(solver.getStopThreshold(),
                LMedSRobustLateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 3);
        assertEquals(solver.getPreliminarySubsetSize(), 3);
        assertNotNull(solver.getCircles());
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
        double[] wrong = new double[4];
        Point2D[] shortPositions = new Point2D[1];
        double[] shortDistances = new double[1];
        solver = null;
        try {
            solver = new LMedSRobustLateration2DSolver((Point2D[])null, distances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(positions, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(positions, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(shortPositions, shortDistances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        // constructor with positions, distances and standard deviations
        double[] standardDeviations = new double[3];
        solver = new LMedSRobustLateration2DSolver(positions, distances,
                standardDeviations);

        // check correctness
        assertEquals(solver.getStopThreshold(),
                LMedSRobustLateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 3);
        assertEquals(solver.getPreliminarySubsetSize(), 3);
        assertNotNull(solver.getCircles());
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
            solver = new LMedSRobustLateration2DSolver(null, distances,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(positions, null,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(positions, distances,
                    (double[]) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(positions, wrong,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(positions, distances,
                    wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(shortPositions,
                    shortDistances, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        // constructor with positions, distances, standard deviations and listener
        solver = new LMedSRobustLateration2DSolver(positions, distances,
                standardDeviations, this);

        // check correctness
        assertEquals(solver.getStopThreshold(),
                LMedSRobustLateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 3);
        assertEquals(solver.getPreliminarySubsetSize(), 3);
        assertNotNull(solver.getCircles());
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
            solver = new LMedSRobustLateration2DSolver(null, distances,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(positions, null,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(positions, distances,
                    null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(positions, wrong,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(positions, distances,
                    wrong, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(shortPositions,
                    shortDistances, standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        // constructor with positions, distances and listener
        solver = new LMedSRobustLateration2DSolver(positions, distances,
                this);

        // check correctness
        assertEquals(solver.getStopThreshold(),
                LMedSRobustLateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 3);
        assertEquals(solver.getPreliminarySubsetSize(), 3);
        assertNotNull(solver.getCircles());
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
            solver = new LMedSRobustLateration2DSolver((Point2D[])null, distances,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(positions, null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(positions, wrong,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(shortPositions,
                    shortDistances, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        // constructor with circles
        Circle[] circles = new Circle[3];
        circles[0] = new Circle(positions[0], distances[0]);
        circles[1] = new Circle(positions[1], distances[1]);
        circles[2] = new Circle(positions[2], distances[2]);
        solver = new LMedSRobustLateration2DSolver(circles);

        // check correctness
        assertEquals(solver.getStopThreshold(),
                LMedSRobustLateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 3);
        assertEquals(solver.getPreliminarySubsetSize(), 3);
        assertNotNull(solver.getCircles());
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
        Circle[] shortCircles = new Circle[1];

        solver = null;
        try {
            solver = new LMedSRobustLateration2DSolver((Circle[])null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(shortCircles);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        // constructor with circles and standard deviations
        solver = new LMedSRobustLateration2DSolver(circles,
                standardDeviations);

        // check correctness
        assertEquals(solver.getStopThreshold(),
                LMedSRobustLateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 3);
        assertEquals(solver.getPreliminarySubsetSize(), 3);
        assertNotNull(solver.getCircles());
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
            solver = new LMedSRobustLateration2DSolver((Circle[])null,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(circles,
                    (double[]) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(shortCircles,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(circles, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        // constructor with circles and listener
        solver = new LMedSRobustLateration2DSolver(circles, this);

        // check correctness
        assertEquals(solver.getStopThreshold(),
                LMedSRobustLateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 3);
        assertEquals(solver.getPreliminarySubsetSize(), 3);
        assertNotNull(solver.getCircles());
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
            solver = new LMedSRobustLateration2DSolver(null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(shortCircles,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        // constructor with circles, standard deviation and listener
        solver = new LMedSRobustLateration2DSolver(circles,
                standardDeviations, this);

        // check correctness
        assertEquals(solver.getStopThreshold(),
                LMedSRobustLateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(solver.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertEquals(solver.getMinRequiredPositionsAndDistances(), 3);
        assertEquals(solver.getPreliminarySubsetSize(), 3);
        assertNotNull(solver.getCircles());
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
            solver = new LMedSRobustLateration2DSolver((Circle[])null,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(circles,
                    null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(shortCircles,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new LMedSRobustLateration2DSolver(circles, wrong,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        LMedSRobustLateration2DSolver solver =
                new LMedSRobustLateration2DSolver();

        // check initial value
        assertEquals(solver.getStopThreshold(),
                LMedSRobustLateration2DSolver.DEFAULT_STOP_THRESHOLD, 0.0);

        // set new value
        solver.setStopThreshold(1.0);

        // check
        assertEquals(solver.getStopThreshold(), 1.0, 0.0);

        // force IllegalArgumentException
        try {
            solver.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetCircles() throws LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        LMedSRobustLateration2DSolver solver =
                new LMedSRobustLateration2DSolver();

        // check initial value
        assertNull(solver.getCircles());

        // set new value
        Point2D[] positions = new Point2D[3];
        positions[0] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        positions[1] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        positions[2] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        double[] distances = new double[3];
        distances[0] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[1] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[2] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);

        Circle[] circles = new Circle[3];
        circles[0] = new Circle(positions[0], distances[0]);
        circles[1] = new Circle(positions[1], distances[1]);
        circles[2] = new Circle(positions[2], distances[2]);
        solver.setCircles(circles);

        // check
        Circle[] circles2 = solver.getCircles();
        for (int i = 0; i < 3; i++) {
            assertSame(circles[i].getCenter(), circles2[i].getCenter());
            assertEquals(circles[i].getRadius(), circles2[i].getRadius(), 0.0);
        }

        // force IllegalArgumentException
        try {
            solver.setCircles(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setCircles(new Circle[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetCirclesAndStandardDeviations() throws LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        LMedSRobustLateration2DSolver solver =
                new LMedSRobustLateration2DSolver();

        // check initial value
        assertNull(solver.getCircles());

        // set new value
        Point2D[] positions = new Point2D[3];
        positions[0] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        positions[1] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        positions[2] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        double[] distances = new double[3];
        distances[0] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[1] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[2] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        double[] standardDeviations = new double[3];
        standardDeviations[0] = randomizer.nextDouble();
        standardDeviations[1] = randomizer.nextDouble();
        standardDeviations[2] = randomizer.nextDouble();

        Circle[] circles = new Circle[3];
        circles[0] = new Circle(positions[0], distances[0]);
        circles[1] = new Circle(positions[1], distances[1]);
        circles[2] = new Circle(positions[2], distances[2]);
        solver.setCirclesAndStandardDeviations(circles, standardDeviations);

        // check
        Circle[] circles2 = solver.getCircles();
        for (int i = 0; i < 3; i++) {
            assertSame(circles[i].getCenter(), circles2[i].getCenter());
            assertEquals(circles[i].getRadius(), circles2[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(),
                standardDeviations);

        // force IllegalArgumentException
        try {
            solver.setCirclesAndStandardDeviations(null,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setCirclesAndStandardDeviations(circles,
                    null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setCirclesAndStandardDeviations(new Circle[1],
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setCirclesAndStandardDeviations(circles,
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetPreliminarySubsetSize() throws LockedException {
        LMedSRobustLateration2DSolver solver =
                new LMedSRobustLateration2DSolver();

        // check initial value
        assertEquals(solver.getPreliminarySubsetSize(), 3);

        // set new value
        solver.setPreliminarySubsetSize(4);

        // check
        assertEquals(solver.getPreliminarySubsetSize(), 4);

        // force IllegalArgumentException
        try {
            solver.setPreliminarySubsetSize(2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        LMedSRobustLateration2DSolver solver =
                new LMedSRobustLateration2DSolver();

        // check default value
        assertNull(solver.getListener());

        // set new value
        solver.setListener(this);

        // check
        assertSame(solver.getListener(), this);
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        LMedSRobustLateration2DSolver solver =
                new LMedSRobustLateration2DSolver();

        // check default value
        assertNull(solver.getInitialPosition());

        // set new value
        Point2D p = Point2D.create();
        solver.setInitialPosition(p);

        // check
        assertSame(solver.getInitialPosition(), p);
    }

    @Test
    public void testIsSetLinearSolverUsed() throws LockedException {
        LMedSRobustLateration2DSolver solver =
                new LMedSRobustLateration2DSolver();

        // check default value
        assertTrue(solver.isLinearSolverUsed());

        // set new value
        solver.setLinearSolverUsed(false);

        // check
        assertFalse(solver.isLinearSolverUsed());
    }

    @Test
    public void testIsSetHomogeneousLinearSolverUsed() throws LockedException {
        LMedSRobustLateration2DSolver solver =
                new LMedSRobustLateration2DSolver();

        // check default value
        assertFalse(solver.isHomogeneousLinearSolverUsed());

        // set new value
        solver.setHomogeneousLinearSolverUsed(true);

        // check
        assertTrue(solver.isHomogeneousLinearSolverUsed());
    }

    @Test
    public void testIsSetPreliminarySolutionRefined() throws LockedException {
        LMedSRobustLateration2DSolver solver =
                new LMedSRobustLateration2DSolver();

        // check default value
        assertTrue(solver.isPreliminarySolutionRefined());

        // set new value
        solver.setPreliminarySolutionRefined(false);

        // check
        assertFalse(solver.isPreliminarySolutionRefined());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        LMedSRobustLateration2DSolver solver =
                new LMedSRobustLateration2DSolver();

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
        LMedSRobustLateration2DSolver solver =
                new LMedSRobustLateration2DSolver();

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
        LMedSRobustLateration2DSolver solver =
                new LMedSRobustLateration2DSolver();

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
        LMedSRobustLateration2DSolver solver =
                new LMedSRobustLateration2DSolver();

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
        LMedSRobustLateration2DSolver solver =
                new LMedSRobustLateration2DSolver();

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
        LMedSRobustLateration2DSolver solver =
                new LMedSRobustLateration2DSolver();

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

        LMedSRobustLateration2DSolver solver =
                new LMedSRobustLateration2DSolver();

        // check default value
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());

        // set new values
        Point2D[] positions = new Point2D[3];
        positions[0] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        positions[1] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        positions[2] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        double[] distances = new double[3];
        distances[0] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[1] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[2] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);

        solver.setPositionsAndDistances(positions, distances);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);

        // force IllegalArgumentException
        double[] wrong = new double[4];
        Point2D[] shortPositions = new Point2D[1];
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

        LMedSRobustLateration2DSolver solver =
                new LMedSRobustLateration2DSolver();

        // check default value
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());

        // set new values
        Point2D[] positions = new Point2D[3];
        positions[0] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        positions[1] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        positions[2] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        double[] distances = new double[3];
        distances[0] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[1] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[2] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        double[] standardDeviations = new double[3];
        standardDeviations[0] = randomizer.nextDouble();
        standardDeviations[1] = randomizer.nextDouble();
        standardDeviations[2] = randomizer.nextDouble();

        solver.setPositionsDistancesAndStandardDeviations(
                positions, distances, standardDeviations);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(),
                standardDeviations);

        // force IllegalArgumentException
        double[] wrong = new double[4];
        Point2D[] shortPositions = new Point2D[1];
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
            int numCircles = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D center;
            double radius, error;
            Circle[] circles = new Circle[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint2D(
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
                circles[i] = new Circle(center, radius);
            }

            LMedSRobustLateration2DSolver solver =
                    new LMedSRobustLateration2DSolver(circles, this);
            solver.setResultRefined(false);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point2D estimatedPosition = solver.solve();

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
            solver = new LMedSRobustLateration2DSolver();

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
            int numCircles = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D center;
            double radius, error;
            Circle[] circles = new Circle[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint2D(
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
                circles[i] = new Circle(center, radius);
            }

            LMedSRobustLateration2DSolver solver =
                    new LMedSRobustLateration2DSolver(circles, this);
            solver.setResultRefined(true);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point2D estimatedPosition = solver.solve();

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
            solver = new LMedSRobustLateration2DSolver();

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
            int numCircles = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D center;
            double radius, error;
            Circle[] circles = new Circle[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint2D(
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
                circles[i] = new Circle(center, radius);
            }

            LMedSRobustLateration2DSolver solver =
                    new LMedSRobustLateration2DSolver(circles, this);
            solver.setResultRefined(true);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point2D estimatedPosition = solver.solve();

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
            solver = new LMedSRobustLateration2DSolver();

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
            int numCircles = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D center;
            double radius, error;
            Circle[] circles = new Circle[numCircles];
            double[] standardDeviations = new double[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint2D(
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
                circles[i] = new Circle(center, radius);
            }

            LMedSRobustLateration2DSolver solver =
                    new LMedSRobustLateration2DSolver(circles, this);
            solver.setResultRefined(true);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point2D estimatedPosition = solver.solve();

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
            solver = new LMedSRobustLateration2DSolver();

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
            int numCircles = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D center;
            double radius, error;
            Circle[] circles = new Circle[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint2D(
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
                circles[i] = new Circle(center, radius);
            }

            LMedSRobustLateration2DSolver solver =
                    new LMedSRobustLateration2DSolver(circles, this);
            solver.setLinearSolverUsed(false);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point2D estimatedPosition = solver.solve();

            //check
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
            solver = new LMedSRobustLateration2DSolver();

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
            int numCircles = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D center;
            double radius, error;
            Circle[] circles = new Circle[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint2D(
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
                circles[i] = new Circle(center, radius);
            }

            LMedSRobustLateration2DSolver solver =
                    new LMedSRobustLateration2DSolver(circles, this);
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

            Point2D estimatedPosition = solver.solve();

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
            solver = new LMedSRobustLateration2DSolver();

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
            int numCircles = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D center;
            double radius, error;
            Circle[] circles = new Circle[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint2D(
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
                circles[i] = new Circle(center, radius);
            }

            LMedSRobustLateration2DSolver solver =
                    new LMedSRobustLateration2DSolver(circles, this);
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

            Point2D estimatedPosition = solver.solve();

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
            solver = new LMedSRobustLateration2DSolver();

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
            int numCircles = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D center;
            double radius, error;
            Circle[] circles = new Circle[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint2D(
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
                circles[i] = new Circle(center, radius);
            }

            LMedSRobustLateration2DSolver solver =
                    new LMedSRobustLateration2DSolver(circles, this);
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

            Point2D estimatedPosition = solver.solve();

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
            solver = new LMedSRobustLateration2DSolver();

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
            int numCircles = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D center;
            double radius, error;
            Circle[] circles = new Circle[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint2D(
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
                circles[i] = new Circle(center, radius);
            }

            LMedSRobustLateration2DSolver solver =
                    new LMedSRobustLateration2DSolver(circles, this);
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

            Point2D estimatedPosition = solver.solve();

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
            solver = new LMedSRobustLateration2DSolver();

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
            int numCircles = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D center;
            double radius, error;
            Circle[] circles = new Circle[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint2D(
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
                circles[i] = new Circle(center, radius);
            }

            LMedSRobustLateration2DSolver solver =
                    new LMedSRobustLateration2DSolver(circles, this);
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

            Point2D estimatedPosition = solver.solve();

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
            solver = new LMedSRobustLateration2DSolver();

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
            int numCircles = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D center;
            double radius, error;
            Circle[] circles = new Circle[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint2D(
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
                circles[i] = new Circle(center, radius);
            }

            LMedSRobustLateration2DSolver solver =
                    new LMedSRobustLateration2DSolver(circles, this);
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

            Point2D estimatedPosition = solver.solve();

            //check
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
            solver = new LMedSRobustLateration2DSolver();

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
            int numCircles = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D center;
            double radius, error;
            Circle[] circles = new Circle[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint2D(
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
                circles[i] = new Circle(center, radius);
            }

            LMedSRobustLateration2DSolver solver =
                    new LMedSRobustLateration2DSolver(circles, this);
            solver.setResultRefined(false);
            solver.setPreliminarySubsetSize(4);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertEquals(solveNextIteration, 0);
            assertEquals(solveProgressChange, 0);
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertNull(solver.getEstimatedPosition());

            Point2D estimatedPosition = solver.solve();

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
            solver = new LMedSRobustLateration2DSolver();

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
    public void onSolveStart(RobustLaterationSolver<Point2D> solver) {
        solveStart++;
        checkLocked((LMedSRobustLateration2DSolver)solver);
    }

    @Override
    public void onSolveEnd(RobustLaterationSolver<Point2D> solver) {
        solveEnd++;
        checkLocked((LMedSRobustLateration2DSolver)solver);
    }

    @Override
    public void onSolveNextIteration(RobustLaterationSolver<Point2D> solver, int iteration) {
        solveNextIteration++;
        checkLocked((LMedSRobustLateration2DSolver)solver);
    }

    @Override
    public void onSolveProgressChange(RobustLaterationSolver<Point2D> solver, float progress) {
        solveProgressChange++;
        checkLocked((LMedSRobustLateration2DSolver)solver);
    }

    private void reset() {
        solveStart = solveEnd = solveNextIteration =
                solveProgressChange = 0;
    }

    private void checkLocked(LMedSRobustLateration2DSolver solver) {
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
            solver.setCircles(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setCirclesAndStandardDeviations(null, null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setStopThreshold(0.5);
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
