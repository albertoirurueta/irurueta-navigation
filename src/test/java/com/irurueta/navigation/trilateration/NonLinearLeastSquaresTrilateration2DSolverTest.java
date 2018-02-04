package com.irurueta.navigation.trilateration;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.irurueta.geometry.Circle;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

public class NonLinearLeastSquaresTrilateration2DSolverTest implements TrilaterationSolverListener<Point2D> {

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

    public NonLinearLeastSquaresTrilateration2DSolverTest() { }

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
        //empty constructor
        NonLinearLeastSquaresTrilateration2DSolver solver = new NonLinearLeastSquaresTrilateration2DSolver();

        //check correctness
        assertNull(solver.getListener());
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);


        //constructor with positions and distances
        Point2D[] positions = new Point2D[2];
        positions[0] = new InhomogeneousPoint2D();
        positions[1] = new InhomogeneousPoint2D();
        double[] distances = new double[2];
        solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, distances);

        //check correctness
        assertNull(solver.getListener());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);

        //Force IllegalArgumentException
        double[] wrong = new double[3];
        Point2D[] shortPositions = new Point2D[1];
        double[] shortDistances = new double[1];
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver((Point2D[])null, distances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(shortPositions, shortDistances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with initial position
        InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D();
        solver = new NonLinearLeastSquaresTrilateration2DSolver(initialPosition);

        //check correctness
        assertNull(solver.getListener());
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);


        //constructor with positions, distances and initial position
        solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, distances, initialPosition);

        //check correctness
        assertNull(solver.getListener());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNotNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver((Point2D[])null, distances, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, wrong, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(shortPositions, shortDistances, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with listener
        //noinspection unchecked
        TrilaterationSolverListener<Point2D> listener = mock(TrilaterationSolverListener.class);
        solver = new NonLinearLeastSquaresTrilateration2DSolver(listener);

        //check correctness
        assertSame(solver.getListener(), listener);
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);


        //constructor with positions, distances and listener
        solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, distances, listener);

        //check correctness
        assertSame(solver.getListener(), listener);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver((Point2D[])null, distances, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, null, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, wrong, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(shortPositions, shortDistances, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with initial position and listener
        solver = new NonLinearLeastSquaresTrilateration2DSolver(initialPosition, listener);

        //check correctness
        assertSame(solver.getListener(), listener);
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);


        //constructor with positions, distances, initial position and listener
        solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, distances, initialPosition, listener);

        //check correctness
        assertSame(solver.getListener(), listener);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNotNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver((Point2D[])null, distances, initialPosition,
                    listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, null, initialPosition,
                    listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, wrong, initialPosition, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(shortPositions, shortDistances, initialPosition,
                    listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with circles
        Circle[] circles = new Circle[2];
        circles[0] = new Circle(positions[0], distances[0]);
        circles[1] = new Circle(positions[1], distances[1]);
        solver = new NonLinearLeastSquaresTrilateration2DSolver(circles);

        //check correctness
        assertNull(solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);

        //Force IllegalArgumentException
        Circle[] shortCircles = new Circle[1];

        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver((Circle[]) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(shortCircles);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with circles and initial position
        solver = new NonLinearLeastSquaresTrilateration2DSolver(circles, initialPosition);

        //check correctness
        assertNull(solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNotNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(shortCircles, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with circles and listener
        solver = new NonLinearLeastSquaresTrilateration2DSolver(circles, listener);

        //check correctness
        assertSame(solver.getListener(), listener);
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver((Circle[]) null, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(shortCircles, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with circles, initial position and listener
        solver = new NonLinearLeastSquaresTrilateration2DSolver(circles, initialPosition, listener);

        //check correctness
        assertSame(solver.getListener(), listener);
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNotNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(null, initialPosition, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(shortCircles, initialPosition, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with positions, distances and stadard deviations
        double[] standardDeviations = new double[2];
        solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, distances,
                standardDeviations);

        //check correctness
        assertNull(solver.getListener());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(null, distances,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, null,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, distances,
                    (double[])null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, wrong,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, distances,
                    wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(shortPositions, shortDistances,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with positions, distances, standard deviations and initial position
        solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, distances,
                standardDeviations, initialPosition);

        //check correctness
        assertNull(solver.getListener());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNotNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(null, distances,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, null,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, distances,
                    null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, wrong,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, distances,
                    wrong, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(shortPositions, shortDistances,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //check positions, distances, standard deviations and listener
        solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, distances,
                standardDeviations, this);

        //check correctness
        assertSame(solver.getListener(), this);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(null, distances,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, null,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, distances,
                    (double[])null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, wrong,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, distances,
                    wrong, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(shortPositions, shortDistances,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //check positions, distances, standard deviations, initial position and listener
        solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, distances,
                standardDeviations, initialPosition, this);

        //check correctness
        assertSame(solver.getListener(), this);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNotNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(null, distances,
                    standardDeviations, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, null,
                    standardDeviations, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, distances,
                    null, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, wrong,
                    standardDeviations, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(positions, distances,
                    wrong, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(shortPositions, shortDistances,
                    standardDeviations, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with circles and standard deviations
        solver = new NonLinearLeastSquaresTrilateration2DSolver(circles,
                standardDeviations);

        //check correctness
        assertNull(solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver((Circle[]) null,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(shortCircles,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with circles, standard deviations and initial position
        solver = new NonLinearLeastSquaresTrilateration2DSolver(circles,
                standardDeviations, initialPosition);

        //check correctness
        assertNull(solver.getListener());
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNotNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver((Circle[]) null,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(shortCircles,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with circles, standard deviations and listener
        solver = new NonLinearLeastSquaresTrilateration2DSolver(circles,
                standardDeviations, this);

        //check correctness
        assertSame(solver.getListener(), this);
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertNull(solver.getInitialPosition());
        assertNotNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver((Circle[]) null,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(shortCircles,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with circles, standard deviations, initial position and listener
        solver = new NonLinearLeastSquaresTrilateration2DSolver(circles,
                standardDeviations, initialPosition, this);

        //check correctness
        assertSame(solver.getListener(), this);
        assertNotNull(solver.getPositions());
        assertNotNull(solver.getDistances());
        assertTrue(solver.isReady());
        assertNull(solver.getEstimatedPositionCoordinates());
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNotNull(solver.getDistanceStandardDeviations());
        assertNull(solver.getCovariance());
        assertEquals(solver.getChiSq(), 0.0, 0.0);
        assertSame(solver.getInitialPosition(), initialPosition);
        assertNotNull(solver.getCircles());
        assertEquals(solver.getType(), TrilaterationSolverType.NON_LINEAR_TRILATERATION_SOLVER);

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver((Circle[]) null,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration2DSolver(shortCircles,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);
    }

    @Test
    public void testGetSetCircles() throws LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        NonLinearLeastSquaresTrilateration2DSolver solver = new NonLinearLeastSquaresTrilateration2DSolver();

        //initial value
        assertNull(solver.getCircles());

        //set new value
        Point2D[] positions = new Point2D[2];
        positions[0] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        positions[1] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        double[] distances = new double[2];
        distances[0] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[1] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);

        Circle[] circles = new Circle[2];
        circles[0] = new Circle(positions[0], distances[0]);
        circles[1] = new Circle(positions[1], distances[1]);

        solver.setCircles(circles);

        //check
        Circle[] circles2 = solver.getCircles();
        assertSame(circles[0].getCenter(), circles2[0].getCenter());
        assertSame(circles[1].getCenter(), circles2[1].getCenter());
        assertEquals(circles[0].getRadius(), circles2[0].getRadius(), 0.0);
        assertEquals(circles[1].getRadius(), circles2[1].getRadius(), 0.0);

        //force IllegalArgumentException
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

        NonLinearLeastSquaresTrilateration2DSolver solver = new NonLinearLeastSquaresTrilateration2DSolver();

        //initial value
        assertNull(solver.getCircles());
        assertNull(solver.getDistanceStandardDeviations());

        //set new values
        Point2D[] positions = new Point2D[2];
        positions[0] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        positions[1] = new InhomogeneousPoint2D(randomizer.nextDouble(), randomizer.nextDouble());
        double[] distances = new double[2];
        distances[0] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);
        distances[1] = randomizer.nextDouble(1.0, MAX_RANDOM_VALUE);

        Circle[] circles = new Circle[2];
        circles[0] = new Circle(positions[0], distances[0]);
        circles[1] = new Circle(positions[1], distances[1]);

        double[] standardDeviations = new double[2];

        solver.setCirclesAndStandardDeviations(circles, standardDeviations);

        //check
        Circle[] circles2 = solver.getCircles();
        assertSame(circles[0].getCenter(), circles2[0].getCenter());
        assertSame(circles[1].getCenter(), circles2[1].getCenter());
        assertEquals(circles[0].getRadius(), circles2[0].getRadius(), 0.0);
        assertEquals(circles[1].getRadius(), circles2[1].getRadius(), 0.0);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);

        //force IllegalArgumentException
        try {
            solver.setCirclesAndStandardDeviations(null, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setCirclesAndStandardDeviations(new Circle[1], standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setCirclesAndStandardDeviations(circles, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setCirclesAndStandardDeviations(circles, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        NonLinearLeastSquaresTrilateration2DSolver solver = new NonLinearLeastSquaresTrilateration2DSolver();

        //initial value
        assertNull(solver.getListener());

        //set new value
        //noinspection unchecked
        TrilaterationSolverListener<Point2D> listener = mock(TrilaterationSolverListener.class);
        solver.setListener(listener);

        //check
        assertSame(solver.getListener(), listener);
    }

    @Test
    public void testGetSetPositionsAndDistances() throws LockedException {
        NonLinearLeastSquaresTrilateration2DSolver solver = new NonLinearLeastSquaresTrilateration2DSolver();

        //initial value
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());

        //set new values
        Point2D[] positions = new Point2D[2];
        positions[0] = new InhomogeneousPoint2D();
        positions[1] = new InhomogeneousPoint2D();
        double[] distances = new double[2];

        solver.setPositionsAndDistances(positions, distances);

        //check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());

        //Force IllegalArgumentException
        try {
            solver.setPositionsAndDistances(null, distances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setPositionsAndDistances(positions, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setPositionsAndDistances(positions, new double[3]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setPositionsAndDistances(new Point2D[1], new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetPositionsDistancesAndStandardDeviations() throws LockedException {
        NonLinearLeastSquaresTrilateration2DSolver solver = new NonLinearLeastSquaresTrilateration2DSolver();

        //initial value
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertNull(solver.getDistanceStandardDeviations());
        assertFalse(solver.isReady());

        //set new values
        Point2D[] positions = new Point2D[2];
        positions[0] = new InhomogeneousPoint2D();
        positions[1] = new InhomogeneousPoint2D();
        double[] distances = new double[2];
        double[] standardDeviations = new double[2];

        solver.setPositionsDistancesAndStandardDeviations(positions, distances,
                standardDeviations);

        //check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);

        //Force IllegalArgumentException
        try {
            solver.setPositionsAndDistances(null, distances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setPositionsAndDistances(positions, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setPositionsAndDistances(positions, new double[3]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setPositionsAndDistances(new Point2D[1], new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        NonLinearLeastSquaresTrilateration2DSolver solver = new NonLinearLeastSquaresTrilateration2DSolver();

        //check initial value
        assertNull(solver.getInitialPosition());

        //set new value
        InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D();
        solver.setInitialPosition(initialPosition);

        //check
        assertSame(solver.getInitialPosition(), initialPosition);
    }

    @Test
    public void testSolveNoInitialPositionAndNoError()
            throws TrilaterationException, NotReadyException, LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0, numInvalid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numCircles = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D center;
            double radius;
            Circle[] circles = new Circle[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);
                circles[i] = new Circle(center, radius);
            }

            NonLinearLeastSquaresTrilateration2DSolver solver = new NonLinearLeastSquaresTrilateration2DSolver(
                    circles, this);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            Point2D estimatedPosition = solver.getEstimatedPosition();
            double distance = estimatedPosition.distanceTo(position);
            if (distance >= ABSOLUTE_ERROR) {
                numInvalid++;
            } else {
                numValid++;
                assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            }
        }

        assertTrue(numValid > numInvalid);

        //Force NotReadyException
        NonLinearLeastSquaresTrilateration2DSolver solver = new NonLinearLeastSquaresTrilateration2DSolver();
        try {
            solver.solve();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }

        //Force TrilaterationException
        Circle[] circles = new Circle[2];
        InhomogeneousPoint2D center;
        double radius;
        for (int i = 0; i < 2; i++) {
            center = new InhomogeneousPoint2D(Double.NaN, Double.NaN);
            radius = TrilaterationSolver.EPSILON;
            circles[i] = new Circle(center, radius);
        }
        solver.setCircles(circles);
        try {
            solver.solve();
            fail("TrilaterationException expected but not thrown");
        } catch (TrilaterationException ignore) { }
    }

    @Test
    public void testSolveWithInitialPositionAndNoError()
            throws TrilaterationException, NotReadyException, LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        //when an initial solution close to the real solution is provided, the algorithm
        //always converges to the true solution
        for (int t = 0; t < TIMES; t++) {
            int numCircles = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D(
                    position.getInhomX() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR),
                    position.getInhomY() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR));
            InhomogeneousPoint2D center;
            double radius;
            Circle[] circles = new Circle[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);
                circles[i] = new Circle(center, radius);
            }

            NonLinearLeastSquaresTrilateration2DSolver solver = new NonLinearLeastSquaresTrilateration2DSolver(
                    circles, initialPosition, this);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            Point2D estimatedPosition = solver.getEstimatedPosition();
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testSolveNoInitialPositionAndError()
            throws TrilaterationException, NotReadyException, LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0, numInvalid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numPoints = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D point;
            double distance, error;
            Point2D[] positions = new Point2D[numPoints];
            double[] distances = new double[numPoints];
            for (int i = 0; i < numPoints; i++) {
                point = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                distance = point.distanceTo(position);
                error = randomizer.nextDouble(MIN_DISTANCE_ERROR, MAX_DISTANCE_ERROR);
                positions[i] = point;
                distances[i] = distance + error;
            }

            NonLinearLeastSquaresTrilateration2DSolver solver = new NonLinearLeastSquaresTrilateration2DSolver(
                    positions, distances, this);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            Point2D estimatedPosition = solver.getEstimatedPosition();
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
            throws TrilaterationException, NotReadyException, LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        //when an initial solution close to the real solution is provided, the algorithm
        //always converges to the true solution
        for (int t = 0; t < TIMES; t++) {
            int numPoints = randomizer.nextInt(MIN_CIRCLES, MAX_CIRCLES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D(
                    position.getInhomX() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR),
                    position.getInhomY() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR));
            InhomogeneousPoint2D point;
            double distance, error;
            Point2D[] positions = new Point2D[numPoints];
            double[] distances = new double[numPoints];
            for (int i = 0; i < numPoints; i++) {
                point = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                distance = point.distanceTo(position);
                error = randomizer.nextDouble(MIN_DISTANCE_ERROR, MAX_DISTANCE_ERROR);
                positions[i] = point;
                distances[i] = distance + error;
            }

            NonLinearLeastSquaresTrilateration2DSolver solver = new NonLinearLeastSquaresTrilateration2DSolver(
                    positions, distances, initialPosition, this);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            Point2D estimatedPosition = solver.getEstimatedPosition();
            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
        }
    }

    @Override
    public void onSolveStart(TrilaterationSolver<Point2D> solver) {
        solveStart++;
        checkLocked((NonLinearLeastSquaresTrilateration2DSolver)solver);
    }

    @Override
    public void onSolveEnd(TrilaterationSolver<Point2D> solver) {
        solveEnd++;
        checkLocked((NonLinearLeastSquaresTrilateration2DSolver)solver);
    }

    private void reset() {
        solveStart = solveEnd = 0;
    }

    private void checkLocked(NonLinearLeastSquaresTrilateration2DSolver solver) {
        try {
            solver.setListener(null);
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
            solver.setInitialPosition(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setCircles(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setCirclesAndStandardDeviations(
                    null, null);
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
