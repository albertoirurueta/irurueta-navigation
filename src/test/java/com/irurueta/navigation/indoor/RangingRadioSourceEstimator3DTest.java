package com.irurueta.navigation.indoor;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.geometry.Accuracy3D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.text.MessageFormat;
import java.text.NumberFormat;
import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

@SuppressWarnings("Duplicates")
public class RangingRadioSourceEstimator3DTest implements
        RangingRadioSourceEstimatorListener<WifiAccessPoint, Point3D> {

    private static final Logger LOGGER = Logger.getLogger(
            RangingRadioSourceEstimator3DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; //(Hz)
    private static final double TRANSMITTED_POWER_DBM = -50.0;

    private static final int MIN_READINGS = 50;
    private static final int MAX_READINGS = 100;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double ERROR_STD = 0.2;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_POSITION_ERROR = 0.5;

    private static final int TIMES = 50;

    private int estimateStart;
    private int estimateEnd;

    public RangingRadioSourceEstimator3DTest() { }

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
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        //test empty constructor
        RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RangingRadioSourceEstimator3D<>();

        //check default values
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        //test constructor with readings
        List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated3D<>(accessPoint, 0.0,
                    position));
        }

        estimator = new RangingRadioSourceEstimator3D<>(readings);

        //check default values
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RangingRadioSourceEstimator3D<>(
                    (List<RangingReadingLocated3D<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RangingRadioSourceEstimator3D<>(
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with listener
        estimator = new RangingRadioSourceEstimator3D<>(this);

        //check default values
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());


        //test constructor with readings and listener
        estimator = new RangingRadioSourceEstimator3D<>(readings, this);

        //check default values
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RangingRadioSourceEstimator3D<>(
                    (List<RangingReadingLocated3D<WifiAccessPoint>>)null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RangingRadioSourceEstimator3D<>(
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>(),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial position
        InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator = new RangingRadioSourceEstimator3D<>(initialPosition);

        //check default values
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertNull(estimator.getListener());
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());


        //test constructor with readings and initial position
        estimator = new RangingRadioSourceEstimator3D<>(readings, initialPosition);

        //check default values
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertNull(estimator.getListener());
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RangingRadioSourceEstimator3D<>(null,
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RangingRadioSourceEstimator3D<>(
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>(),
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with initial position and listener
        estimator = new RangingRadioSourceEstimator3D<>(initialPosition, this);

        //check default values
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getReadings());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());


        //test constructor with readings, initial position and listener
        estimator = new RangingRadioSourceEstimator3D<>(readings, initialPosition,
                this);

        //check default values
        assertEquals(estimator.getMinReadings(), 4);
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedRadioSource());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RangingRadioSourceEstimator3D<>(null,
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RangingRadioSourceEstimator3D<>(
                    new ArrayList<RangingReadingLocated3D<WifiAccessPoint>>(),
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RangingRadioSourceEstimator3D<>();

        //check default value
        assertNull(estimator.getInitialPosition());

        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        estimator.setInitialPosition(initialPosition);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
    }

    @Test
    public void testAreValidReadings() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated3D<>(accessPoint,
                    0.0, position));
        }

        RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RangingRadioSourceEstimator3D<>();

        assertTrue(estimator.areValidReadings(readings));

        assertFalse(estimator.areValidReadings(null));
        assertFalse(estimator.areValidReadings(
                new ArrayList<RangingReadingLocated<WifiAccessPoint, Point3D>>()));
    }

    @Test
    public void testGetSetReadings() throws LockedException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 5; i++) {
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            readings.add(new RangingReadingLocated3D<>(accessPoint,
                    0.0, position));
        }

        RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RangingRadioSourceEstimator3D<>();

        //initial value
        assertNull(estimator.getReadings());
        assertFalse(estimator.isReady());

        //set new value
        estimator.setReadings(readings);

        //check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator.isReady());

        //force IllegalArgumentException
        try {
            estimator.setReadings(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RangingRadioSourceEstimator3D<>();

        //check default value
        assertNull(estimator.getListener());

        //set new value
        estimator.setListener(this);

        //check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testEstimateWithoutInitialPositionAndWithoutError()
            throws LockedException, NotReadyException, AlgebraException,
            RadioSourceEstimationException {

        int numValidPosition = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance,
                        readingsPositions[i]));
            }

            RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (IndoorException e) {
                continue;
            }

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        //force NotReadyException
        RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RangingRadioSourceEstimator3D<>();

        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithInitialPositionAndWithoutError()
            throws LockedException, NotReadyException, AlgebraException,
            RadioSourceEstimationException {

        int numValidPosition = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance,
                        readingsPositions[i]));
            }

            RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, accessPointPosition,
                            this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (IndoorException e) {
                continue;
            }

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        //force NotReadyException
        RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                new RangingRadioSourceEstimator3D<>(new InhomogeneousPoint3D());

        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithoutInitialPositionAndDistanceErrors()
            throws LockedException, NotReadyException, AlgebraException {

        int numValidPosition = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(accessPointPosition);
                double error = Math.abs(errorRandomizer.nextDouble());

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i]));
            }

            RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (IndoorException e) {
                continue;
            }

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= LARGE_POSITION_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        LARGE_POSITION_ERROR));
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));
    }

    @Test
    public void testEstimateWithInitialPositionAndDistanceErrors()
            throws LockedException, NotReadyException, AlgebraException {

        int numValidPosition = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(accessPointPosition);
                double error = Math.abs(errorRandomizer.nextDouble());

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i]));
            }

            RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, accessPointPosition,
                            this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (IndoorException e) {
                continue;
            }

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= LARGE_POSITION_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        LARGE_POSITION_ERROR));
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));
    }

    @Test
    public void testEstimateWithInitialPositionWithError()
            throws LockedException, NotReadyException, AlgebraException {

        int numValidPosition = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated3D<>(accessPoint, distance,
                        readingsPositions[i]));
            }

            InhomogeneousPoint3D initialPosition =
                    new InhomogeneousPoint3D(
                            accessPointPosition.getInhomX() + randomizer.nextDouble(MIN_POS, MAX_POS),
                            accessPointPosition.getInhomY() + randomizer.nextDouble(MIN_POS, MAX_POS),
                            accessPointPosition.getInhomZ() + randomizer.nextDouble(MIN_POS, MAX_POS));
            RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, initialPosition,
                            this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (IndoorException e) {
                continue;
            }

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));
    }

    @Test
    public void testEstimateWithInitialPositionWithPositionAndDistanceErrors()
            throws LockedException, NotReadyException, AlgebraException {

        int numValidPosition = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);

            int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(accessPointPosition);
                double error = Math.abs(errorRandomizer.nextDouble());

                readings.add(new RangingReadingLocated3D<>(accessPoint,
                        distance + error, readingsPositions[i]));
            }

            InhomogeneousPoint3D initialPosition =
                    new InhomogeneousPoint3D(
                            accessPointPosition.getInhomX() + randomizer.nextDouble(MIN_POS, MAX_POS),
                            accessPointPosition.getInhomY() + randomizer.nextDouble(MIN_POS, MAX_POS),
                            accessPointPosition.getInhomZ() + randomizer.nextDouble(MIN_POS, MAX_POS));
            RangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                    new RangingRadioSourceEstimator3D<>(readings, initialPosition,
                            this);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            try {
                estimator.estimate();
            } catch (IndoorException e) {
                continue;
            }

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            WifiAccessPointLocated3D estimatedAccessPoint =
                    (WifiAccessPointLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedAccessPoint.getBssid(), "bssid");
            assertEquals(estimatedAccessPoint.getFrequency(), FREQUENCY, 0.0);
            assertNull(estimatedAccessPoint.getSsid());
            assertEquals(estimatedAccessPoint.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedAccessPoint.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= LARGE_POSITION_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        LARGE_POSITION_ERROR));
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
        }

        assertTrue(numValidPosition > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));
    }

    @Test
    public void testEstimateBeacon() throws LockedException, NotReadyException,
            AlgebraException, RadioSourceEstimationException {

        int numValidPosition = 0;
        double avgPositionError = 0.0, avgValidPositionError = 0.0,
                avgInvalidPositionError = 0.0;
        double avgPositionStd = 0.0, avgValidPositionStd = 0.0,
                avgInvalidPositionStd = 0.0, avgPositionStdConfidence = 0.0;
        double avgPositionAccuracy = 0.0, avgValidPositionAccuracy = 0.0,
                avgInvalidPositionAccuracy = 0.0, avgPositionAccuracyConfidence = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            InhomogeneousPoint3D accessPointPosition =
                    new InhomogeneousPoint3D(
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS),
                            randomizer.nextDouble(MIN_POS, MAX_POS));

            BeaconIdentifier identifier = BeaconIdentifier.fromUuid(UUID.randomUUID());
            Beacon beacon = new Beacon(Collections.singletonList(identifier),
                    TRANSMITTED_POWER_DBM, FREQUENCY);

            int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);
            Point3D[] readingsPositions = new Point3D[numReadings];
            List<RangingReadingLocated3D<Beacon>> readings = new ArrayList<>();
            for (int i = 0; i < numReadings; i++) {
                readingsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double distance = readingsPositions[i].distanceTo(accessPointPosition);

                readings.add(new RangingReadingLocated3D<>(beacon, distance,
                        readingsPositions[i]));
            }

            RangingRadioSourceEstimator3D<Beacon> estimator =
                    new RangingRadioSourceEstimator3D<>(readings);

            reset();
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());

            try {
                estimator.estimate();
            } catch (IndoorException e) {
                continue;
            }

            //check
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertNotNull(estimator.getEstimatedCovariance());
            assertNotNull(estimator.getEstimatedPositionCovariance());

            BeaconLocated3D estimatedBeacon =
                    (BeaconLocated3D) estimator.getEstimatedRadioSource();

            assertEquals(estimatedBeacon.getIdentifiers(), beacon.getIdentifiers());
            assertEquals(estimatedBeacon.getFrequency(), beacon.getFrequency(), 0.0);
            assertEquals(estimatedBeacon.getPosition(),
                    estimator.getEstimatedPosition());
            assertEquals(estimatedBeacon.getPositionCovariance(),
                    estimator.getEstimatedPositionCovariance());

            Accuracy3D accuracyStd = new Accuracy3D(
                    estimator.getEstimatedPositionCovariance());
            accuracyStd.setStandardDeviationFactor(1.0);

            Accuracy3D accuracy = new Accuracy3D(estimator.getEstimatedPositionCovariance());
            accuracy.setConfidence(0.99);

            double positionStd = accuracyStd.getAverageAccuracy();
            double positionStdConfidence = accuracyStd.getConfidence();
            double positionAccuracy = accuracy.getAverageAccuracy();
            double positionAccuracyConfidence = accuracy.getConfidence();

            double positionDistance = estimator.getEstimatedPosition().
                    distanceTo(accessPointPosition);
            if (positionDistance <= ABSOLUTE_ERROR) {
                assertTrue(estimator.getEstimatedPosition().equals(accessPointPosition,
                        ABSOLUTE_ERROR));
                numValidPosition++;

                avgValidPositionError += positionDistance;
                avgValidPositionStd += positionStd;
                avgValidPositionAccuracy += positionAccuracy;
            } else {
                avgInvalidPositionError += positionDistance;
                avgInvalidPositionStd += positionStd;
                avgInvalidPositionAccuracy += positionAccuracy;
            }

            avgPositionError += positionDistance;
            avgPositionStd += positionStd;
            avgPositionStdConfidence += positionStdConfidence;
            avgPositionAccuracy += positionAccuracy;
            avgPositionAccuracyConfidence += positionAccuracyConfidence;

            assertArrayEquals(estimator.getEstimatedPosition().asArray(),
                    estimator.getEstimatedPositionCoordinates(), 0.0);
        }

        assertTrue(numValidPosition > 0);

        avgValidPositionError /= numValidPosition;
        avgInvalidPositionError /= (TIMES - numValidPosition);
        avgPositionError /= TIMES;

        avgValidPositionStd /= numValidPosition;
        avgInvalidPositionStd /= (TIMES - numValidPosition);
        avgPositionStd /= TIMES;
        avgPositionStdConfidence /= TIMES;

        avgValidPositionAccuracy /= numValidPosition;
        avgInvalidPositionAccuracy /= (TIMES - numValidPosition);
        avgPositionAccuracy /= TIMES;
        avgPositionAccuracyConfidence /= TIMES;

        LOGGER.log(Level.INFO, "Percentage valid position: {0} %",
                (double)numValidPosition / (double)TIMES * 100.0);

        LOGGER.log(Level.INFO, "Avg. valid position error: {0} meters",
                avgValidPositionError);
        LOGGER.log(Level.INFO, "Avg. invalid position error: {0} meters",
                avgInvalidPositionError);
        LOGGER.log(Level.INFO, "Avg. position error: {0} meters",
                avgPositionError);

        NumberFormat format = NumberFormat.getPercentInstance();
        String formattedConfidence = format.format(avgPositionStdConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position standard deviation {0} meters ({1} confidence)",
                avgValidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position standard deviation {0} meters ({1} confidence)",
                avgInvalidPositionStd, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position standard deviation {0} meters ({1} confidence)",
                avgPositionStd, formattedConfidence));

        formattedConfidence = format.format(avgPositionAccuracyConfidence);
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Valid position accuracy {0} meters ({1} confidence)",
                avgValidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Invalid position accuracy {0} meters ({1} confidence)",
                avgInvalidPositionAccuracy, formattedConfidence));
        LOGGER.log(Level.INFO, MessageFormat.format(
                "Position accuracy {0} meters ({1} confidence)",
                avgPositionAccuracy, formattedConfidence));

        //force NotReadyException
        RangingRadioSourceEstimator3D<Beacon> estimator =
                new RangingRadioSourceEstimator3D<>();

        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Override
    public void onEstimateStart(RangingRadioSourceEstimator<WifiAccessPoint,
            Point3D> estimator) {
        estimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(RangingRadioSourceEstimator<WifiAccessPoint,
            Point3D> estimator) {
        estimateEnd++;
        checkLocked(estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    private void checkLocked(RangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
        try {
            estimator.setInitialPosition(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setReadings(null);
            fail("LockedExeption expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
    }
}
