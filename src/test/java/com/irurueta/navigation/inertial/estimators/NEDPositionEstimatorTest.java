package com.irurueta.navigation.inertial.estimators;

import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class NEDPositionEstimatorTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-7;
    private static final int TIMES = 100;

    @Test
    public void testEstimate() {

        final NEDPositionEstimator estimator = new NEDPositionEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double oldLatitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double oldLongitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final double oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);

        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);

        final NEDPosition result1 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final NEDPosition result2 = new NEDPosition();
        estimator.estimate(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result2);

        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Angle oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);

        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);

        final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        final NEDPosition result3 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD, result3);

        final NEDPosition result4 = new NEDPosition();
        estimator.estimate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD, result4);

        final NEDPosition result5 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, vn, ve, vd, result5);

        final NEDPosition result6 = new NEDPosition();
        estimator.estimate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, vn, ve, vd, result6);

        final NEDPosition result7 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                oldHeight, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD, result7);

        final NEDPosition result8 = new NEDPosition();
        estimator.estimate(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD, result8);

        final NEDPosition oldPosition = new NEDPosition(
                oldLatitude, oldLongitude, oldHeight);
        final NEDPosition result9 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldPosition, oldVn, oldVe, oldVd,
                vn, ve, vd, result9);

        final NEDPosition result10 = new NEDPosition();
        estimator.estimate(timeInterval, oldPosition, oldVn, oldVe, oldVd,
                vn, ve, vd, result10);

        final NEDPosition result11 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldPosition,
                oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD,
                result11);

        final NEDPosition result12 = new NEDPosition();
        estimator.estimate(timeInterval, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD, result12);

        final NEDVelocity oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final NEDVelocity velocity = new NEDVelocity(vn, ve, vd);
        final NEDPosition result13 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity, result13);

        final NEDPosition result14 = new NEDPosition();
        estimator.estimate(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity, result14);

        final NEDPosition result15 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, velocity, result15);

        final NEDPosition result16 = new NEDPosition();
        estimator.estimate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, velocity, result16);

        final NEDPosition result17 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldPosition, oldVelocity, velocity,
                result17);

        final NEDPosition result18 = new NEDPosition();
        estimator.estimate(timeInterval, oldPosition, oldVelocity, velocity, result18);

        // check
        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
        assertEquals(result1, result5);
        assertEquals(result1, result6);
        assertEquals(result1, result7);
        assertEquals(result1, result8);
        assertEquals(result1, result9);
        assertEquals(result1, result10);
        assertEquals(result1, result11);
        assertEquals(result1, result12);
        assertEquals(result1, result13);
        assertEquals(result1, result14);
        assertEquals(result1, result15);
        assertEquals(result1, result16);
        assertEquals(result1, result17);
        assertEquals(result1, result18);
    }

    @Test
    public void testEstimateAndReturnNew() {
        final NEDPositionEstimator estimator = new NEDPositionEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double oldLatitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double oldLongitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final double oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);

        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);

        final NEDPosition result1 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final NEDPosition result2 = estimator.estimateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd);

        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Angle oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);

        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);

        final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        final NEDPosition result3 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD);

        final NEDPosition result4 = estimator.estimateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD);

        final NEDPosition result5 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, vn, ve, vd);

        final NEDPosition result6 = estimator.estimateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, vn, ve, vd);

        final NEDPosition result7 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                oldHeight, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD);

        final NEDPosition result8 = estimator.estimateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD);

        final NEDPosition oldPosition = new NEDPosition(
                oldLatitude, oldLongitude, oldHeight);
        final NEDPosition result9 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldVn, oldVe, oldVd,
                vn, ve, vd);

        final NEDPosition result10 = estimator.estimateAndReturnNew(
                timeInterval, oldPosition, oldVn, oldVe, oldVd,
                vn, ve, vd);

        final NEDPosition result11 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition,
                oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD);

        final NEDPosition result12 = estimator.estimateAndReturnNew(
                timeInterval, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD);

        final NEDVelocity oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final NEDVelocity velocity = new NEDVelocity(vn, ve, vd);
        final NEDPosition result13 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity);

        final NEDPosition result14 = estimator.estimateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity);

        final NEDPosition result15 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, velocity);

        final NEDPosition result16 = estimator.estimateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, velocity);

        final NEDPosition result17 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldVelocity, velocity);

        final NEDPosition result18 = estimator.estimateAndReturnNew(
                timeInterval, oldPosition, oldVelocity, velocity);

        // check
        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
        assertEquals(result1, result5);
        assertEquals(result1, result6);
        assertEquals(result1, result7);
        assertEquals(result1, result8);
        assertEquals(result1, result9);
        assertEquals(result1, result10);
        assertEquals(result1, result11);
        assertEquals(result1, result12);
        assertEquals(result1, result13);
        assertEquals(result1, result14);
        assertEquals(result1, result15);
        assertEquals(result1, result16);
        assertEquals(result1, result17);
        assertEquals(result1, result18);
    }

    @Test
    public void testEstimatePosition() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double oldLatitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double oldLongitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final double oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);

        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);

        final NEDPosition result1 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final NEDPosition result2 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result2);

        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Angle oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);

        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);

        final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        final NEDPosition result3 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD, result3);

        final NEDPosition result4 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD, result4);

        final NEDPosition result5 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, vn, ve, vd, result5);

        final NEDPosition result6 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, vn, ve, vd, result6);

        final NEDPosition result7 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                oldHeight, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD, result7);

        final NEDPosition result8 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD, result8);

        final NEDPosition oldPosition = new NEDPosition(
                oldLatitude, oldLongitude, oldHeight);
        final NEDPosition result9 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                TIME_INTERVAL_SECONDS, oldPosition, oldVn, oldVe, oldVd,
                vn, ve, vd, result9);

        final NEDPosition result10 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                timeInterval, oldPosition, oldVn, oldVe, oldVd,
                vn, ve, vd, result10);

        final NEDPosition result11 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                TIME_INTERVAL_SECONDS, oldPosition,
                oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD,
                result11);

        final NEDPosition result12 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                timeInterval, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD, result12);

        final NEDVelocity oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final NEDVelocity velocity = new NEDVelocity(vn, ve, vd);
        final NEDPosition result13 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity, result13);

        final NEDPosition result14 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity, result14);

        final NEDPosition result15 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, velocity, result15);

        final NEDPosition result16 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, velocity, result16);

        final NEDPosition result17 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                TIME_INTERVAL_SECONDS, oldPosition, oldVelocity, velocity,
                result17);

        final NEDPosition result18 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                timeInterval, oldPosition, oldVelocity, velocity, result18);

        // check
        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
        assertEquals(result1, result5);
        assertEquals(result1, result6);
        assertEquals(result1, result7);
        assertEquals(result1, result8);
        assertEquals(result1, result9);
        assertEquals(result1, result10);
        assertEquals(result1, result11);
        assertEquals(result1, result12);
        assertEquals(result1, result13);
        assertEquals(result1, result14);
        assertEquals(result1, result15);
        assertEquals(result1, result16);
        assertEquals(result1, result17);
        assertEquals(result1, result18);
    }

    @Test
    public void testEstimatePositionAndReturnNew() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double oldLatitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double oldLongitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final double oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);

        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);

        final NEDPosition result1 = NEDPositionEstimator.estimatePositionAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final NEDPosition result2 = NEDPositionEstimator.estimatePositionAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd);

        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Angle oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);

        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);

        final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        final NEDPosition result3 = NEDPositionEstimator.estimatePositionAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD);

        final NEDPosition result4 = NEDPositionEstimator.estimatePositionAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD);

        final NEDPosition result5 = NEDPositionEstimator.estimatePositionAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, vn, ve, vd);

        final NEDPosition result6 = NEDPositionEstimator.estimatePositionAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, vn, ve, vd);

        final NEDPosition result7 = NEDPositionEstimator.estimatePositionAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                oldHeight, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD);

        final NEDPosition result8 = NEDPositionEstimator.estimatePositionAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD);

        final NEDPosition oldPosition = new NEDPosition(
                oldLatitude, oldLongitude, oldHeight);
        final NEDPosition result9 = NEDPositionEstimator.estimatePositionAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldVn, oldVe, oldVd,
                vn, ve, vd);

        final NEDPosition result10 = NEDPositionEstimator.estimatePositionAndReturnNew(
                timeInterval, oldPosition, oldVn, oldVe, oldVd,
                vn, ve, vd);

        final NEDPosition result11 = NEDPositionEstimator.estimatePositionAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition,
                oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD);

        final NEDPosition result12 = NEDPositionEstimator.estimatePositionAndReturnNew(
                timeInterval, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD);

        final NEDVelocity oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final NEDVelocity velocity = new NEDVelocity(vn, ve, vd);
        final NEDPosition result13 = NEDPositionEstimator.estimatePositionAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity);

        final NEDPosition result14 = NEDPositionEstimator.estimatePositionAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity);

        final NEDPosition result15 = NEDPositionEstimator.estimatePositionAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, velocity);

        final NEDPosition result16 = NEDPositionEstimator.estimatePositionAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, velocity);

        final NEDPosition result17 = NEDPositionEstimator.estimatePositionAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldVelocity, velocity);

        final NEDPosition result18 = NEDPositionEstimator.estimatePositionAndReturnNew(
                timeInterval, oldPosition, oldVelocity, velocity);

        // check
        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
        assertEquals(result1, result5);
        assertEquals(result1, result6);
        assertEquals(result1, result7);
        assertEquals(result1, result8);
        assertEquals(result1, result9);
        assertEquals(result1, result10);
        assertEquals(result1, result11);
        assertEquals(result1, result12);
        assertEquals(result1, result13);
        assertEquals(result1, result14);
        assertEquals(result1, result15);
        assertEquals(result1, result16);
        assertEquals(result1, result17);
        assertEquals(result1, result18);
    }

    @Test(expected = IllegalArgumentException.class)
    public void testEstimateWithNegativeTimeIntervalThrowsIllegalArgumentException() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double oldLatitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double oldLongitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final double oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);

        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                MAX_VELOCITY_VALUE);

        final NEDPosition result = new NEDPosition();
        NEDPositionEstimator.estimatePosition(
                -TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
    }

    @Test
    public void testEstimateWithZeroTimeInterval() {
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double oldLatitude = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double oldLongitude = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final double oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                    MAX_VELOCITY_VALUE);
            final double oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                    MAX_VELOCITY_VALUE);
            final double oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                    MAX_VELOCITY_VALUE);

            final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                    MAX_VELOCITY_VALUE);
            final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                    MAX_VELOCITY_VALUE);
            final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                    MAX_VELOCITY_VALUE);

            final NEDPosition result = new NEDPosition();
            NEDPositionEstimator.estimatePosition(
                    0.0, oldLatitude, oldLongitude, oldHeight,
                    oldVn, oldVe, oldVd, vn, ve, vd, result);

            assertEquals(result.getLatitude(), oldLatitude, ABSOLUTE_ERROR);
            assertEquals(result.getLongitude(), oldLongitude, ABSOLUTE_ERROR);
            assertEquals(result.getHeight(), oldHeight, ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testEstimateWithZeroVelocity() {
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double oldLatitude = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double oldLongitude = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final double oldVn = 0.0;
            final double oldVe = 0.0;
            final double oldVd = 0.0;

            final double vn = 0.0;
            final double ve = 0.0;
            final double vd = 0.0;

            final NEDPosition result = new NEDPosition();
            NEDPositionEstimator.estimatePosition(
                    TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                    oldVn, oldVe, oldVd, vn, ve, vd, result);

            assertEquals(result.getLatitude(), oldLatitude, ABSOLUTE_ERROR);
            assertEquals(result.getLongitude(), oldLongitude, ABSOLUTE_ERROR);
            assertEquals(result.getHeight(), oldHeight, ABSOLUTE_ERROR);
        }

    }

    @Test
    public void testCompareWithVelocityEstimator() {
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double oldLatitude = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double oldLongitude = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final NEDPosition oldPosition = new NEDPosition(oldLatitude, oldLongitude,
                    oldHeight);

            final double oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                    MAX_VELOCITY_VALUE);
            final double oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                    MAX_VELOCITY_VALUE);
            final double oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                    MAX_VELOCITY_VALUE);

            final NEDVelocity oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);

            final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                    MAX_VELOCITY_VALUE);
            final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                    MAX_VELOCITY_VALUE);
            final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE,
                    MAX_VELOCITY_VALUE);

            final NEDVelocity velocity = new NEDVelocity(vn, ve, vd);

            final NEDPosition positionResult = new NEDPosition();
            NEDPositionEstimator.estimatePosition(
                    TIME_INTERVAL_SECONDS, oldPosition,
                    oldVelocity, velocity, positionResult);

            final NEDVelocity velocityResult = new NEDVelocity();
            NEDVelocityEstimator.estimateVelocity(TIME_INTERVAL_SECONDS, oldPosition,
                    oldVelocity, positionResult, velocityResult);

            assertEquals(velocityResult.getVn(), vn, LARGE_ABSOLUTE_ERROR);
            assertEquals(velocityResult.getVe(), ve, LARGE_ABSOLUTE_ERROR);
            assertEquals(velocityResult.getVd(), vd, LARGE_ABSOLUTE_ERROR);
            assertTrue(velocityResult.equals(velocity, LARGE_ABSOLUTE_ERROR));

            final NEDPosition positionResult2 = NEDPositionEstimator
                    .estimatePositionAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition,
                            oldVelocity, velocityResult);

            assertTrue(positionResult2.equals(positionResult, ABSOLUTE_ERROR));
        }
    }
}
