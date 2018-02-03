package com.irurueta.navigation.trilateration;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.irurueta.geometry.Sphere;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

public class NonLinearLeastSquaresTrilateration3DSolverTest implements TrilaterationSolverListener<Point3D> {

    private static final int MIN_SPHERES = 4;
    private static final int MAX_SPHERES = 10;

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

    public NonLinearLeastSquaresTrilateration3DSolverTest() { }

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
        NonLinearLeastSquaresTrilateration3DSolver solver = new NonLinearLeastSquaresTrilateration3DSolver();

        //check correctness
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


        //constructor with positions and distances
        Point3D[] positions = new Point3D[2];
        positions[0] = new InhomogeneousPoint3D();
        positions[1] = new InhomogeneousPoint3D();
        double[] distances = new double[2];
        solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, distances);

        //check correctness
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

        //Force IllegalArgumentException
        double[] wrong = new double[3];
        Point3D[] shortPositions = new Point3D[1];
        double[] shortDistances = new double[1];
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver((Point3D[])null, distances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(shortPositions, shortDistances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with initial position
        InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D();
        solver = new NonLinearLeastSquaresTrilateration3DSolver(initialPosition);

        //check correctness
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


        //constructor with positions, distances and initial position
        solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, distances, initialPosition);

        //check correctness
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

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver((Point3D[])null, distances, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, wrong, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(shortPositions, shortDistances, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with listener
        //noinspection unchecked
        TrilaterationSolverListener<Point3D> listener = mock(TrilaterationSolverListener.class);
        solver = new NonLinearLeastSquaresTrilateration3DSolver(listener);

        //check correctness
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


        //constructor with positions, distances and listener
        solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, distances, listener);

        //check correctness
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

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver((Point3D[])null, distances, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, null, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, wrong, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(shortPositions, shortDistances, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with initial position and listener
        solver = new NonLinearLeastSquaresTrilateration3DSolver(initialPosition, listener);

        //check correctness
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


        //constructor with positions, distances, initial position and listener
        solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, distances, initialPosition, listener);

        //check correctness
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

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver((Point3D[])null, distances, initialPosition,
                    listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, null, initialPosition,
                    listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, wrong, initialPosition, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(shortPositions, shortDistances, initialPosition,
                    listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with spheres
        Sphere[] spheres = new Sphere[2];
        spheres[0] = new Sphere(positions[0], distances[0]);
        spheres[1] = new Sphere(positions[1], distances[1]);
        solver = new NonLinearLeastSquaresTrilateration3DSolver(spheres);

        //check correctness
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

        //Force IllegalArgumentException
        Sphere[] shortSpheres = new Sphere[1];

        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver((Sphere[]) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(shortSpheres);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with spheres and initial position
        solver = new NonLinearLeastSquaresTrilateration3DSolver(spheres, initialPosition);

        //check correctness
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

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(shortSpheres, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with spheres and listener
        solver = new NonLinearLeastSquaresTrilateration3DSolver(spheres, listener);

        //check correctness
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

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver((Sphere[]) null, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(shortSpheres, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with circles, initial position and listener
        solver = new NonLinearLeastSquaresTrilateration3DSolver(spheres, initialPosition, listener);

        //check correctness
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

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(null, initialPosition, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(shortSpheres, initialPosition, listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with positions, distances and standard deviations
        double[] standardDeviations = new double[2];
        solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, distances,
                standardDeviations);

        //check correctness
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

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(null, distances,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, null,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, distances,
                    (double[])null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, wrong,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, distances,
                    wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(shortPositions, shortDistances,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with positions, distances, standard deviations and initial position
        solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, distances,
                standardDeviations, initialPosition);

        //check correctness
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

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(null, distances,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, null,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, distances,
                    null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, wrong,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, distances,
                    wrong, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(shortPositions, shortDistances,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //check positions, distances, standard deviations and listener
        solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, distances,
                standardDeviations, this);

        //check correctness
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

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(null, distances,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, null,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, distances,
                    (double[])null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, wrong,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, distances,
                    wrong, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(shortPositions, shortDistances,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //check positions, distances, standard deviations, initial position and listener
        solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, distances,
                standardDeviations, initialPosition, this);

        //check correctness
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

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(null, distances,
                    standardDeviations, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, null,
                    standardDeviations, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, distances,
                    null, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, wrong,
                    standardDeviations, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(positions, distances,
                    wrong, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(shortPositions, shortDistances,
                    standardDeviations, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with spheres and standard deviations
        solver = new NonLinearLeastSquaresTrilateration3DSolver(spheres,
                standardDeviations);

        //check correctness
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

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver((Sphere[]) null,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(shortSpheres,
                    standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with spheres, standard deviations and initial position
        solver = new NonLinearLeastSquaresTrilateration3DSolver(spheres,
                standardDeviations, initialPosition);

        //check correctness
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

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver((Sphere[]) null,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(shortSpheres,
                    standardDeviations, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with spheres, standard deviations and listener
        solver = new NonLinearLeastSquaresTrilateration3DSolver(spheres,
                standardDeviations, this);

        //check correctness
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

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver((Sphere[]) null,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(shortSpheres,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //constructor with spheres, standard deviations, initial position and listener
        solver = new NonLinearLeastSquaresTrilateration3DSolver(spheres,
                standardDeviations, initialPosition, this);

        //check correctness
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

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver((Sphere[]) null,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new NonLinearLeastSquaresTrilateration3DSolver(shortSpheres,
                    standardDeviations, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);
    }

    @Test
    public void testGetSetSpheres() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        NonLinearLeastSquaresTrilateration3DSolver solver = new NonLinearLeastSquaresTrilateration3DSolver();

        //initial value
        assertNull(solver.getSpheres());

        //set new value
        Point3D[] positions = new Point3D[2];
        positions[0] = new InhomogeneousPoint3D(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());
        positions[1] = new InhomogeneousPoint3D(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());
        double[] distances = new double[2];
        distances[0] = randomizer.nextDouble();
        distances[1] = randomizer.nextDouble();

        Sphere[] spheres = new Sphere[2];
        spheres[0] = new Sphere(positions[0], distances[0]);
        spheres[1] = new Sphere(positions[1], distances[1]);

        solver.setSpheres(spheres);

        //check
        Sphere[] spheres2 = solver.getSpheres();
        assertSame(spheres[0].getCenter(), spheres2[0].getCenter());
        assertSame(spheres[1].getCenter(), spheres2[1].getCenter());
        assertEquals(spheres[0].getRadius(), spheres2[0].getRadius(), 0.0);
        assertEquals(spheres[1].getRadius(), spheres2[1].getRadius(), 0.0);

        //force IllegalArgumentException
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
    public void testGetSetSpheresAndStandardDeviations() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        NonLinearLeastSquaresTrilateration3DSolver solver = new NonLinearLeastSquaresTrilateration3DSolver();

        //initial value
        assertNull(solver.getSpheres());
        assertNull(solver.getDistanceStandardDeviations());

        //set new value
        Point3D[] positions = new Point3D[2];
        positions[0] = new InhomogeneousPoint3D(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());
        positions[1] = new InhomogeneousPoint3D(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());
        double[] distances = new double[2];
        distances[0] = randomizer.nextDouble();
        distances[1] = randomizer.nextDouble();

        Sphere[] spheres = new Sphere[2];
        spheres[0] = new Sphere(positions[0], distances[0]);
        spheres[1] = new Sphere(positions[1], distances[1]);

        double[] standardDeviations = new double[2];

        solver.setSpheresAndStandardDeviations(spheres, standardDeviations);

        //check
        Sphere[] spheres2 = solver.getSpheres();
        assertSame(spheres[0].getCenter(), spheres2[0].getCenter());
        assertSame(spheres[1].getCenter(), spheres2[1].getCenter());
        assertEquals(spheres[0].getRadius(), spheres2[0].getRadius(), 0.0);
        assertEquals(spheres[1].getRadius(), spheres2[1].getRadius(), 0.0);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);

        //force IllegalArgumentException
        try {
            solver.setSpheresAndStandardDeviations(null, standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setSpheresAndStandardDeviations(new Sphere[1], standardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setSpheresAndStandardDeviations(spheres, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setSpheresAndStandardDeviations(spheres, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetListener() {
        NonLinearLeastSquaresTrilateration3DSolver solver = new NonLinearLeastSquaresTrilateration3DSolver();

        //initial value
        assertNull(solver.getListener());

        //set new value
        //noinspection unchecked
        TrilaterationSolverListener<Point3D> listener = mock(TrilaterationSolverListener.class);
        solver.setListener(listener);

        //check
        assertSame(solver.getListener(), listener);
    }

    @Test
    public void testGetSetPositionsAndDistances() {
        NonLinearLeastSquaresTrilateration3DSolver solver = new NonLinearLeastSquaresTrilateration3DSolver();

        //initial value
        assertNull(solver.getPositions());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());

        //set new values
        Point3D[] positions = new Point3D[2];
        positions[0] = new InhomogeneousPoint3D();
        positions[1] = new InhomogeneousPoint3D();
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
            solver.setPositionsAndDistances(new Point3D[1], new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testSolveNoInitialPositionAndNoError()
            throws TrilaterationException, NotReadyException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0, numInvalid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numCircles = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D center;
            double radius;
            Sphere[] spheres = new Sphere[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);
                spheres[i] = new Sphere(center, radius);
            }

            NonLinearLeastSquaresTrilateration3DSolver solver = new NonLinearLeastSquaresTrilateration3DSolver(
                    spheres, this);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            Point3D estimatedPosition = solver.getEstimatedPosition();
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
        NonLinearLeastSquaresTrilateration3DSolver solver = new NonLinearLeastSquaresTrilateration3DSolver();
        try {
            solver.solve();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }

        //Force TrilaterationException
        Sphere[] spheres = new Sphere[2];
        InhomogeneousPoint3D center;
        double radius;
        for (int i = 0; i < 2; i++) {
            center = new InhomogeneousPoint3D(Double.NaN, Double.NaN, Double.NaN);
            radius = TrilaterationSolver.EPSILON;
            spheres[i] = new Sphere(center, radius);
        }
        solver.setSpheres(spheres);
        try {
            solver.solve();
            fail("TrilaterationException expected but not thrown");
        } catch (TrilaterationException ignore) { }
    }

    @Test
    public void testSolveWithInitialPositionAndNoError()
            throws TrilaterationException, NotReadyException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        //when an initial solution close to the real solution is provided, the algorithm
        //always converges to the true solution
        for (int t = 0; t < TIMES; t++) {
            int numCircles = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                    position.getInhomX() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR),
                    position.getInhomY() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR),
                    position.getInhomZ() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR));
            InhomogeneousPoint3D center;
            double radius;
            Sphere[] spheres = new Sphere[numCircles];
            for (int i = 0; i < numCircles; i++) {
                center = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
                radius = center.distanceTo(position);
                spheres[i] = new Sphere(center, radius);
            }

            NonLinearLeastSquaresTrilateration3DSolver solver = new NonLinearLeastSquaresTrilateration3DSolver(
                    spheres, initialPosition, this);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            Point3D estimatedPosition = solver.getEstimatedPosition();
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testSolveNoInitialPositionAndError()
            throws TrilaterationException, NotReadyException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0, numInvalid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numPoints = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D point;
            double distance, error;
            Point3D[] positions = new Point3D[numPoints];
            double[] distances = new double[numPoints];
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

            NonLinearLeastSquaresTrilateration3DSolver solver = new NonLinearLeastSquaresTrilateration3DSolver(
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
            throws TrilaterationException, NotReadyException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        //when an initial solution close to the real solution is provided, the algorithm
        //always converges to the true solution
        for (int t = 0; t < TIMES; t++) {
            int numPoints = randomizer.nextInt(MIN_SPHERES, MAX_SPHERES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                    position.getInhomX() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR),
                    position.getInhomY() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR),
                    position.getInhomZ() + randomizer.nextDouble(
                            MIN_POSITION_ERROR, MAX_POSITION_ERROR));
            InhomogeneousPoint3D point;
            double distance, error;
            Point3D[] positions = new Point3D[numPoints];
            double[] distances = new double[numPoints];
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

            NonLinearLeastSquaresTrilateration3DSolver solver = new NonLinearLeastSquaresTrilateration3DSolver(
                    positions, distances, initialPosition, this);

            reset();
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);
            assertTrue(solver.isReady());
            assertNull(solver.getEstimatedPosition());
            assertNull(solver.getEstimatedPositionCoordinates());

            solver.solve();

            Point3D estimatedPosition = solver.getEstimatedPosition();
            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
        }
    }

    @Override
    public void onSolveStart(TrilaterationSolver<Point3D> solver) {
        solveStart++;
    }

    @Override
    public void onSolveEnd(TrilaterationSolver<Point3D> solver) {
        solveEnd++;
    }

    private void reset() {
        solveStart = solveEnd = 0;
    }
}
