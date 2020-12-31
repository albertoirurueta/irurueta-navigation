package com.irurueta.navigation.inertial.calibration;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.ECEFGravity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownFrameAccelerometerLinearLeastSquaresCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownGravityNormAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerAndGyroscopeMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerAndGyroscopeMeasurementsGeneratorListener;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.generators.GyroscopeMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.gyroscope.EasyGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.KnownFrameGyroscopeLinearLeastSquaresCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionIntegrator;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class IMUErrorsCreatorTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;
    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;
    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double MIN_DELTA_POS_METERS = -1e-3;
    private static final double MAX_DELTA_POS_METERS = 1e-3;
    private static final double MIN_DELTA_ANGLE_DEGREES = -2.0;
    private static final double MAX_DELTA_ANGLE_DEGREES = 2.0;

    private static final int TIMES = 100;

    private static final double ABSOLUTE_ERROR = 5e-4;

    private static final double SMALL_ABSOLUTE_ERROR = 1e-6;

    private static final double SMALL_ROOT_PSD = 1e-15;


    @Test
    public void testConstructor1() {
        final IMUErrorsCreator creator = new IMUErrorsCreator();

        // check default values
        assertNull(creator.getAccelerometerCalibrationSource());
        assertNull(creator.getGyroscopeCalibrationSource());
        assertNull(creator.getAccelerometerNoiseRootPsdSource());
        assertNull(creator.getGyroscopeNoiseRootPsdSource());
        assertEquals(0.0, creator.getAccelerometerQuantizationLevel(), 0.0);
        assertEquals(0.0, creator.getGyroQuantizationLevel(), 0.0);
        assertFalse(creator.isReady());
    }

    @Test
    public void testConstructor2() {
        final KnownFrameAccelerometerLinearLeastSquaresCalibrator accelerometerCalibrator =
                new KnownFrameAccelerometerLinearLeastSquaresCalibrator();
        final KnownFrameGyroscopeLinearLeastSquaresCalibrator gyroscopeCalibrator =
                new KnownFrameGyroscopeLinearLeastSquaresCalibrator();
        final AccelerometerMeasurementsGenerator accelerometerGenerator =
                new AccelerometerMeasurementsGenerator();
        final GyroscopeMeasurementsGenerator gyroscopeGenerator =
                new GyroscopeMeasurementsGenerator();

        final IMUErrorsCreator creator = new IMUErrorsCreator(
                accelerometerCalibrator, gyroscopeCalibrator,
                accelerometerGenerator, gyroscopeGenerator);

        // check default values
        assertSame(accelerometerCalibrator, creator.getAccelerometerCalibrationSource());
        assertSame(gyroscopeCalibrator, creator.getGyroscopeCalibrationSource());
        assertSame(accelerometerGenerator, creator.getAccelerometerNoiseRootPsdSource());
        assertSame(gyroscopeGenerator, creator.getGyroscopeNoiseRootPsdSource());
        assertEquals(0.0, creator.getAccelerometerQuantizationLevel(), 0.0);
        assertEquals(0.0, creator.getGyroQuantizationLevel(), 0.0);
        assertFalse(creator.isReady());
    }

    @Test
    public void testConstructor3() {
        final KnownFrameAccelerometerLinearLeastSquaresCalibrator accelerometerCalibrator =
                new KnownFrameAccelerometerLinearLeastSquaresCalibrator();
        final KnownFrameGyroscopeLinearLeastSquaresCalibrator gyroscopeCalibrator =
                new KnownFrameGyroscopeLinearLeastSquaresCalibrator();
        final AccelerometerMeasurementsGenerator accelerometerGenerator =
                new AccelerometerMeasurementsGenerator();
        final GyroscopeMeasurementsGenerator gyroscopeGenerator =
                new GyroscopeMeasurementsGenerator();

        final Random random = new Random();
        final double accelerometerQuantizationLevel = random.nextDouble();
        final double gyroscopeQuantizationLevel = random.nextDouble();

        final IMUErrorsCreator creator = new IMUErrorsCreator(
                accelerometerCalibrator, gyroscopeCalibrator,
                accelerometerGenerator, gyroscopeGenerator,
                accelerometerQuantizationLevel,
                gyroscopeQuantizationLevel);

        // check default values
        assertSame(accelerometerCalibrator, creator.getAccelerometerCalibrationSource());
        assertSame(gyroscopeCalibrator, creator.getGyroscopeCalibrationSource());
        assertSame(accelerometerGenerator, creator.getAccelerometerNoiseRootPsdSource());
        assertSame(gyroscopeGenerator, creator.getGyroscopeNoiseRootPsdSource());
        assertEquals(accelerometerQuantizationLevel,
                creator.getAccelerometerQuantizationLevel(), 0.0);
        assertEquals(gyroscopeQuantizationLevel,
                creator.getGyroQuantizationLevel(), 0.0);
        assertFalse(creator.isReady());
    }

    @Test
    public void testGetSetAccelerometerCalibrationSource() {
        final IMUErrorsCreator creator = new IMUErrorsCreator();

        // check default value
        assertNull(creator.getAccelerometerCalibrationSource());

        // set new value
        final KnownFrameAccelerometerLinearLeastSquaresCalibrator accelerometerCalibrator =
                new KnownFrameAccelerometerLinearLeastSquaresCalibrator();

        creator.setAccelerometerCalibrationSource(accelerometerCalibrator);

        // check
        assertSame(accelerometerCalibrator,
                creator.getAccelerometerCalibrationSource());
    }

    @Test
    public void testGetSetGyroscopeCalibrationSource() {
        final IMUErrorsCreator creator = new IMUErrorsCreator();

        // check default value
        assertNull(creator.getGyroscopeCalibrationSource());

        // set new value
        final KnownFrameGyroscopeLinearLeastSquaresCalibrator gyroscopeCalibrator =
                new KnownFrameGyroscopeLinearLeastSquaresCalibrator();

        creator.setGyroscopeCalibrationSource(gyroscopeCalibrator);

        // check
        assertSame(gyroscopeCalibrator,
                creator.getGyroscopeCalibrationSource());
    }

    @Test
    public void testGetSetAccelerometerNoiseRootPsdSource() {
        final IMUErrorsCreator creator = new IMUErrorsCreator();

        // check default value
        assertNull(creator.getAccelerometerNoiseRootPsdSource());

        // set new value
        final AccelerometerMeasurementsGenerator accelerometerGenerator =
                new AccelerometerMeasurementsGenerator();

        creator.setAccelerometerNoiseRootPsdSource(accelerometerGenerator);

        // check
        assertSame(accelerometerGenerator,
                creator.getAccelerometerNoiseRootPsdSource());
    }

    @Test
    public void testGetSetGyroscopeNoiseRootPsdSource() {
        final IMUErrorsCreator creator = new IMUErrorsCreator();

        // check default value
        assertNull(creator.getGyroscopeNoiseRootPsdSource());

        // set new value
        final GyroscopeMeasurementsGenerator gyroscopeGenerator =
                new GyroscopeMeasurementsGenerator();

        creator.sstGyroscopeNoiseRootPsdSource(gyroscopeGenerator);

        // check
        assertSame(gyroscopeGenerator, creator.getGyroscopeNoiseRootPsdSource());
    }

    @Test
    public void testGetSetAccelerometerQuantizationLevel() {
        final IMUErrorsCreator creator = new IMUErrorsCreator();

        // check default value
        assertEquals(0.0, creator.getAccelerometerQuantizationLevel(),
                0.0);

        // set new value
        final Random random = new Random();
        final double accelerometerQuantizationLevel = random.nextDouble();

        creator.setAccelerometerQuantizationLevel(accelerometerQuantizationLevel);

        // check
        assertEquals(accelerometerQuantizationLevel,
                creator.getAccelerometerQuantizationLevel(), 0.0);
    }

    @Test
    public void testGetSetGyroQuantizationLevel() {
        final IMUErrorsCreator creator = new IMUErrorsCreator();

        // check default value
        assertEquals(0.0, creator.getGyroQuantizationLevel(), 0.0);

        // set new value
        final Random random = new Random();
        final double gyroscopeQuantizationLevel = random.nextDouble();

        creator.setGyroQuantizationLevel(gyroscopeQuantizationLevel);

        // check
        assertEquals(gyroscopeQuantizationLevel,
                creator.getGyroQuantizationLevel(), 0.0);
    }

    @Test
    public void testCreate() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            InvalidRotationMatrixException, NotReadyException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, SMALL_ROOT_PSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double roll = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation nedC = new CoordinateTransformation(
                    roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                            ecefFrame, ecefFrame);

            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> gyroscopeMeasurements =
                    new ArrayList<>();
            final List<StandardDeviationBodyKinematics> accelerometerMeasurements =
                    new ArrayList<>();

            final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                    new AccelerometerAndGyroscopeMeasurementsGenerator(
                            new AccelerometerAndGyroscopeMeasurementsGeneratorListener() {
                                @Override
                                public void onInitializationStarted(
                                        final AccelerometerAndGyroscopeMeasurementsGenerator generator) {

                                }

                                @Override
                                public void onInitializationCompleted(
                                        final AccelerometerAndGyroscopeMeasurementsGenerator generator,
                                        final double accelerometerBaseNoiseLevel) {

                                }

                                @Override
                                public void onError(
                                        final AccelerometerAndGyroscopeMeasurementsGenerator generator,
                                        final TriadStaticIntervalDetector.ErrorReason reason) {

                                }

                                @Override
                                public void onStaticIntervalDetected(
                                        final AccelerometerAndGyroscopeMeasurementsGenerator generator) {

                                }

                                @Override
                                public void onDynamicIntervalDetected(
                                        final AccelerometerAndGyroscopeMeasurementsGenerator generator) {

                                }

                                @Override
                                public void onStaticIntervalSkipped(
                                        final AccelerometerAndGyroscopeMeasurementsGenerator generator) {

                                }

                                @Override
                                public void onDynamicIntervalSkipped(
                                        final AccelerometerAndGyroscopeMeasurementsGenerator generator) {

                                }

                                @Override
                                public void onGeneratedAccelerometerMeasurement(
                                        final AccelerometerAndGyroscopeMeasurementsGenerator generator,
                                        final StandardDeviationBodyKinematics measurement) {
                                    accelerometerMeasurements.add(measurement);
                                }

                                @Override
                                public void onGeneratedGyroscopeMeasurement(
                                        final AccelerometerAndGyroscopeMeasurementsGenerator generator,
                                        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> measurement) {
                                    gyroscopeMeasurements.add(measurement);
                                }

                                @Override
                                public void onReset(
                                        final AccelerometerAndGyroscopeMeasurementsGenerator generator) {

                                }
                            });

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector
                    .DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                    errors, random, 0);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final int n = Math.max(numSequences + 1, numMeasurements);

            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            int start = initialStaticSamples;
            for (int i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics,
                        errors, random, start);
                start += staticPeriodLength;

                // generate dynamic samples
                generateDynamicSamples(generator, dynamicPeriodLength,
                        trueKinematics, randomizer, ecefFrame, nedFrame,
                        errors, random, start, false);
                start += dynamicPeriodLength;
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator gyroCalibrator =
                    new EasyGyroscopeCalibrator(gyroscopeMeasurements,
                            true, false,
                            initialBg, initialMg, initialGg, ba, ma);

            try {
                gyroCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final double[] estimatedBg = gyroCalibrator.getEstimatedBiases();
            final Matrix estimatedMg = gyroCalibrator.getEstimatedMg();
            final Matrix estimatedGg = gyroCalibrator.getEstimatedGg();

            final ECEFGravity gravity = ECEFGravityEstimator
                    .estimateGravityAndReturnNew(ecefFrame);

            KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(
                            gravity.getNorm(), accelerometerMeasurements,
                            true);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final double[] estimatedBa = accelerometerCalibrator.getEstimatedBiases();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final double estimatedAccelerometerNoiseRootPsd =
                    generator.getAccelerometerBaseNoiseLevelRootPsd();
            final double estimatedGyroNoiseRootPsd =
                    generator.getGyroscopeBaseNoiseLevelRootPsd();

            assertNotNull(estimatedBa);
            assertNotNull(estimatedMa);
            assertNotNull(estimatedBg);
            assertNotNull(estimatedMg);
            assertNotNull(estimatedGg);

            final double accelerometerQuantizationLevel = random.nextDouble();
            final double gyroscopeQuantizationLevel = random.nextDouble();

            final IMUErrorsCreator creator = new IMUErrorsCreator(
                    accelerometerCalibrator, gyroCalibrator,
                    generator, generator, accelerometerQuantizationLevel,
                    gyroscopeQuantizationLevel);

            final IMUErrors estimatedErrors = creator.create();

            assertNotNull(estimatedErrors);
            assertArrayEquals(estimatedBa,
                    estimatedErrors.getAccelerometerBiases(), 0.0);
            assertArrayEquals(estimatedBg,
                    estimatedErrors.getGyroBiases(), 0.0);
            assertEquals(estimatedMa,
                    estimatedErrors.getAccelerometerScaleFactorAndCrossCouplingErrors());
            assertEquals(estimatedMg,
                    estimatedErrors.getGyroScaleFactorAndCrossCouplingErrors());
            assertEquals(estimatedGg,
                    estimatedErrors.getGyroGDependentBiases());
            assertEquals(estimatedAccelerometerNoiseRootPsd,
                    estimatedErrors.getAccelerometerNoiseRootPSD(), 0.0);
            assertEquals(estimatedGyroNoiseRootPsd,
                    estimatedErrors.getGyroNoiseRootPSD(), 0.0);
            assertEquals(accelerometerQuantizationLevel,
                    estimatedErrors.getAccelerometerQuantizationLevel(), 0.0);
            assertEquals(gyroscopeQuantizationLevel,
                    estimatedErrors.getGyroQuantizationLevel(), 0.0);

            if (!bg.equals(gyroCalibrator.getEstimatedBiasesAsMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(gyroCalibrator.getEstimatedBiasesAsMatrix(), ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, 0.0));

            if (!ba.equals(accelerometerCalibrator.getEstimatedBiasesAsMatrix(), SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(accelerometerCalibrator.getEstimatedBiasesAsMatrix(), SMALL_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, SMALL_ABSOLUTE_ERROR));

            assertTrue(estimatedAccelerometerNoiseRootPsd > 0.0);
            assertTrue(estimatedGyroNoiseRootPsd > 0.0);

            final double accelerometerNoiseLevel = generator.getAccelerometerBaseNoiseLevel();
            final double gyroNoiseLevel = generator.getGyroscopeBaseNoiseLevel();
            final double sqrtTimeInterval = Math.sqrt(generator.getTimeInterval());
            assertEquals(accelerometerNoiseLevel * sqrtTimeInterval,
                    estimatedAccelerometerNoiseRootPsd,
                    SMALL_ABSOLUTE_ERROR);
            assertEquals(gyroNoiseLevel * sqrtTimeInterval,
                    estimatedGyroNoiseRootPsd,
                    SMALL_ABSOLUTE_ERROR);

            assertEquals(SMALL_ROOT_PSD, estimatedAccelerometerNoiseRootPsd,
                    SMALL_ABSOLUTE_ERROR);
            assertEquals(0.0, estimatedGyroNoiseRootPsd,
                    SMALL_ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // Force NotReadyException
        final IMUErrorsCreator creator = new IMUErrorsCreator();
        try {
            creator.create();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    private Matrix generateBa() {
        return Matrix.newFromArray(new double[]{
                900 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                -1300 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                800 * MICRO_G_TO_METERS_PER_SECOND_SQUARED});
    }

    private Matrix generateBg() {
        return Matrix.newFromArray(new double[]{
                -9 * DEG_TO_RAD / 3600.0,
                13 * DEG_TO_RAD / 3600.0,
                -8 * DEG_TO_RAD / 3600.0});
    }

    private Matrix generateMaCommonAxis() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                0.0, -600e-6, 250e-6,
                0.0, 0.0, 450e-6
        }, false);

        return result;
    }

    private Matrix generateMg() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                0.0, -300e-6, -150e-6,
                0.0, 0.0, -350e-6
        }, false);

        return result;
    }

    private double getAccelNoiseRootPSD() {
        return 100.0 * MICRO_G_TO_METERS_PER_SECOND_SQUARED;
    }

    private double getGyroNoiseRootPSD() {
        return 0.01 * DEG_TO_RAD / 60.0;
    }

    private void generateStaticSamples(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator,
            final int numSamples,
            final BodyKinematics trueKinematics,
            final IMUErrors errors,
            final Random random,
            final int startSample)
            throws LockedException {

        final TimedBodyKinematics timedMeasuredKinematics = new TimedBodyKinematics();
        final BodyKinematics measuredKinematics = new BodyKinematics();
        for (int i = 0, j = startSample; i < numSamples; i++, j++) {

            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);

            timedMeasuredKinematics.setKinematics(measuredKinematics);
            timedMeasuredKinematics.setTimestampSeconds(
                    j * TIME_INTERVAL_SECONDS);

            assertTrue(generator.process(timedMeasuredKinematics));
        }
    }

    @SuppressWarnings("SameParameterValue")
    private void generateDynamicSamples(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator,
            final int numSamples,
            final BodyKinematics trueKinematics,
            final UniformRandomizer randomizer,
            final ECEFFrame ecefFrame,
            final NEDFrame nedFrame,
            final IMUErrors errors,
            final Random random,
            final int startSample,
            final boolean changePosition)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException, InvalidRotationMatrixException {

        final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
        final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
        final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

        final double deltaX = changePosition ?
                randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final double deltaY = changePosition ?
                randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final double deltaZ = changePosition ?
                randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;

        final double deltaRoll = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaPitch = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaYaw = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final double ecefX = ecefFrame.getX();
        final double ecefY = ecefFrame.getY();
        final double ecefZ = ecefFrame.getZ();

        final CoordinateTransformation nedC = nedFrame.getCoordinateTransformation();

        final double roll = nedC.getRollEulerAngle();
        final double pitch = nedC.getPitchEulerAngle();
        final double yaw = nedC.getYawEulerAngle();

        final Quaternion beforeQ = new Quaternion();
        nedC.asRotation(beforeQ);

        NEDFrame oldNedFrame = new NEDFrame(nedFrame);
        NEDFrame newNedFrame = new NEDFrame();
        ECEFFrame oldEcefFrame = new ECEFFrame(ecefFrame);
        ECEFFrame newEcefFrame = new ECEFFrame();

        double oldEcefX = ecefX - deltaX;
        double oldEcefY = ecefY - deltaY;
        double oldEcefZ = ecefZ - deltaZ;
        double oldRoll = roll - deltaRoll;
        double oldPitch = pitch - deltaPitch;
        double oldYaw = yaw - deltaYaw;

        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                new BodyKinematicsSequence<>();
        final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList =
                new ArrayList<>();

        final TimedBodyKinematics timedMeasuredKinematics = new TimedBodyKinematics();
        final BodyKinematics measuredKinematics = new BodyKinematics();

        for (int i = 0, j = startSample; i < numSamples; i++, j++) {
            final double progress = (double) i / (double) numSamples;

            final double newRoll = oldRoll + interpolate(deltaRoll, progress);
            final double newPitch = oldPitch + interpolate(deltaPitch, progress);
            final double newYaw = oldYaw + interpolate(deltaYaw, progress);
            final CoordinateTransformation newNedC =
                    new CoordinateTransformation(
                            newRoll, newPitch, newYaw,
                            FrameType.BODY_FRAME,
                            FrameType.LOCAL_NAVIGATION_FRAME);
            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + interpolate(deltaX, progress);
            final double newEcefY = oldEcefY + interpolate(deltaY, progress);
            final double newEcefZ = oldEcefZ + interpolate(deltaZ, progress);

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

            // update true kinematics using new position and rotation
            ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS,
                    newEcefFrame, oldEcefFrame, trueKinematics);

            // add error to true kinematics
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);

            timedMeasuredKinematics.setKinematics(measuredKinematics);
            timedMeasuredKinematics.setTimestampSeconds(timestampSeconds);

            assertTrue(generator.process(timedMeasuredKinematics));

            final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                    new StandardDeviationTimedBodyKinematics(
                            new BodyKinematics(trueKinematics), timestampSeconds,
                            specificForceStandardDeviation,
                            angularRateStandardDeviation);
            trueTimedKinematicsList.add(trueTimedKinematics);

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = newEcefX;
            oldEcefY = newEcefY;
            oldEcefZ = newEcefZ;
        }

        trueSequence.setItems(trueTimedKinematicsList);

        final Quaternion afterQ = new Quaternion();
        QuaternionIntegrator.integrateGyroSequence(
                trueSequence, beforeQ, afterQ);

        final CoordinateTransformation newNedC =
                new CoordinateTransformation(
                        afterQ.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);
        newNedFrame.setCoordinateTransformation(newNedC);

        NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);


        // update current ECEF and NED frames
        ecefFrame.copyFrom(newEcefFrame);
        nedFrame.copyFrom(newNedFrame);

        // after dynamic sequence finishes, update true kinematics for a
        // static sequence at current frame
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS,
                newEcefFrame, newEcefFrame, trueKinematics);
    }

    // This is required to simulate a smooth transition of values during
    // dynamic period, to avoid a sudden rotation or translation and simulate
    // a more natural behaviour.
    private double interpolate(final double value, final double progress) {
        return -2.0 * (Math.abs(progress - 0.5) - 0.5) * value;
    }

}
