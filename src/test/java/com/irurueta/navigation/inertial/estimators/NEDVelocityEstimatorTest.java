package com.irurueta.navigation.inertial.estimators;

import com.irurueta.navigation.frames.NEDFrame;
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

public class NEDVelocityEstimatorTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final int TIMES = 100;

    @Test
    public void testEstimate() {

        final NEDVelocityEstimator estimator = new NEDVelocityEstimator();

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

        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDVelocity result1 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final NEDVelocity result2 = new NEDVelocity();
        estimator.estimate(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result2);

        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Angle oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);
        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);
        final NEDVelocity result3 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                latitudeAngle, longitudeAngle, heightDistance, result3);

        final NEDVelocity result4 = new NEDVelocity();
        estimator.estimate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                latitudeAngle, longitudeAngle, heightDistance, result4);

        final NEDVelocity result5 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, latitudeAngle, longitudeAngle,
                heightDistance, result5);

        final NEDVelocity result6 = new NEDVelocity();
        estimator.estimate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, latitudeAngle,
                longitudeAngle, heightDistance, result6);

        final NEDVelocity result7 = new NEDVelocity();
        final NEDVelocity oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                oldHeight, oldVelocity, latitude, longitude, height, result7);

        final NEDVelocity result8 = new NEDVelocity();
        estimator.estimate(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, latitude, longitude, height, result8);

        final NEDVelocity result9 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, latitudeAngle, longitudeAngle,
                heightDistance, result9);

        final NEDVelocity result10 = new NEDVelocity();
        estimator.estimate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, latitudeAngle, longitudeAngle,
                heightDistance, result10);

        final NEDFrame oldFrame = new NEDFrame(oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd);
        final NEDVelocity result11 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldFrame, latitude, longitude,
                height, result11);

        final NEDVelocity result12 = new NEDVelocity();
        estimator.estimate(timeInterval, oldFrame, latitude, longitude, height,
                result12);

        final NEDVelocity result13 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldFrame, latitudeAngle,
                longitudeAngle, heightDistance, result13);

        final NEDVelocity result14 = new NEDVelocity();
        estimator.estimate(timeInterval, oldFrame, latitudeAngle, longitudeAngle,
                heightDistance, result14);

        final NEDPosition oldPosition = new NEDPosition(
                oldLatitude, oldLongitude, oldHeight);
        final NEDPosition position = new NEDPosition(latitude, longitude, height);
        final NEDVelocity result15 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldPosition, oldVn, oldVe, oldVd,
                position, result15);

        final NEDVelocity result16 = new NEDVelocity();
        estimator.estimate(timeInterval, oldPosition, oldVn, oldVe, oldVd,
                position, result16);

        final NEDVelocity result17 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldPosition, oldSpeedN,
                oldSpeedE, oldSpeedD, position, result17);

        final NEDVelocity result18 = new NEDVelocity();
        estimator.estimate(timeInterval, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD,
                position, result18);

        final NEDVelocity result19 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldPosition, oldVelocity,
                position, result19);

        final NEDVelocity result20 = new NEDVelocity();
        estimator.estimate(timeInterval, oldPosition, oldVelocity, position,
                result20);

        final NEDVelocity result21 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldFrame, position,
                result21);

        final NEDVelocity result22 = new NEDVelocity();
        estimator.estimate(timeInterval, oldFrame, position, result22);

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
        assertEquals(result1, result19);
        assertEquals(result1, result20);
        assertEquals(result1, result21);
        assertEquals(result1, result22);
    }

    @Test
    public void testEstimateAndReturnNew() {

        final NEDVelocityEstimator estimator = new NEDVelocityEstimator();

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

        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDVelocity result1 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final NEDVelocity result2 = estimator.estimateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height);

        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Angle oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);
        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);
        final NEDVelocity result3 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                latitudeAngle, longitudeAngle, heightDistance);

        final NEDVelocity result4 = estimator.estimateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                latitudeAngle, longitudeAngle, heightDistance);

        final NEDVelocity result5 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, latitudeAngle, longitudeAngle,
                heightDistance);

        final NEDVelocity result6 = estimator.estimateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, latitudeAngle,
                longitudeAngle, heightDistance);

        final NEDVelocity oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final NEDVelocity result7 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                oldHeight, oldVelocity, latitude, longitude, height);

        final NEDVelocity result8 = estimator.estimateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, latitude, longitude, height);

        final NEDVelocity result9 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, latitudeAngle, longitudeAngle,
                heightDistance);

        final NEDVelocity result10 = estimator.estimateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, latitudeAngle, longitudeAngle,
                heightDistance);

        final NEDFrame oldFrame = new NEDFrame(oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd);
        final NEDVelocity result11 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, latitude, longitude,
                height);

        final NEDVelocity result12 = estimator.estimateAndReturnNew(
                timeInterval, oldFrame, latitude, longitude, height);

        final NEDVelocity result13 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, latitudeAngle,
                longitudeAngle, heightDistance);

        final NEDVelocity result14 = estimator.estimateAndReturnNew(
                timeInterval, oldFrame, latitudeAngle, longitudeAngle,
                heightDistance);

        final NEDPosition oldPosition = new NEDPosition(
                oldLatitude, oldLongitude, oldHeight);
        final NEDPosition position = new NEDPosition(latitude, longitude, height);
        final NEDVelocity result15 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldVn, oldVe, oldVd,
                position);

        final NEDVelocity result16 = estimator.estimateAndReturnNew(
                timeInterval, oldPosition, oldVn, oldVe, oldVd,
                position);

        final NEDVelocity result17 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldSpeedN,
                oldSpeedE, oldSpeedD, position);

        final NEDVelocity result18 = estimator.estimateAndReturnNew(
                timeInterval, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD,
                position);

        final NEDVelocity result19 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldVelocity,
                position);

        final NEDVelocity result20 = estimator.estimateAndReturnNew(
                timeInterval, oldPosition, oldVelocity, position);

        final NEDVelocity result21 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, position);

        final NEDVelocity result22 = estimator.estimateAndReturnNew(
                timeInterval, oldFrame, position);

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
        assertEquals(result1, result19);
        assertEquals(result1, result20);
        assertEquals(result1, result21);
        assertEquals(result1, result22);
    }

    @Test
    public void testEstimateVelocity() {

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

        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDVelocity result1 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final NEDVelocity result2 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result2);

        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Angle oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);
        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);
        final NEDVelocity result3 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                latitudeAngle, longitudeAngle, heightDistance, result3);

        final NEDVelocity result4 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                latitudeAngle, longitudeAngle, heightDistance, result4);

        final NEDVelocity result5 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, latitudeAngle, longitudeAngle,
                heightDistance, result5);

        final NEDVelocity result6 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, latitudeAngle,
                longitudeAngle, heightDistance, result6);

        final NEDVelocity result7 = new NEDVelocity();
        final NEDVelocity oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        NEDVelocityEstimator.estimateVelocity(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                oldHeight, oldVelocity, latitude, longitude, height, result7);

        final NEDVelocity result8 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, latitude, longitude, height, result8);

        final NEDVelocity result9 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, latitudeAngle, longitudeAngle,
                heightDistance, result9);

        final NEDVelocity result10 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, latitudeAngle, longitudeAngle,
                heightDistance, result10);

        final NEDFrame oldFrame = new NEDFrame(oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd);
        final NEDVelocity result11 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                TIME_INTERVAL_SECONDS, oldFrame, latitude, longitude,
                height, result11);

        final NEDVelocity result12 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                timeInterval, oldFrame, latitude, longitude, height,
                result12);

        final NEDVelocity result13 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                TIME_INTERVAL_SECONDS, oldFrame, latitudeAngle,
                longitudeAngle, heightDistance, result13);

        final NEDVelocity result14 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                timeInterval, oldFrame, latitudeAngle, longitudeAngle,
                heightDistance, result14);

        final NEDPosition oldPosition = new NEDPosition(
                oldLatitude, oldLongitude, oldHeight);
        final NEDPosition position = new NEDPosition(latitude, longitude, height);
        final NEDVelocity result15 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                TIME_INTERVAL_SECONDS, oldPosition, oldVn, oldVe, oldVd,
                position, result15);

        final NEDVelocity result16 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                timeInterval, oldPosition, oldVn, oldVe, oldVd,
                position, result16);

        final NEDVelocity result17 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                TIME_INTERVAL_SECONDS, oldPosition, oldSpeedN,
                oldSpeedE, oldSpeedD, position, result17);

        final NEDVelocity result18 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                timeInterval, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD,
                position, result18);

        final NEDVelocity result19 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                TIME_INTERVAL_SECONDS, oldPosition, oldVelocity,
                position, result19);

        final NEDVelocity result20 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                timeInterval, oldPosition, oldVelocity, position,
                result20);

        final NEDVelocity result21 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                TIME_INTERVAL_SECONDS, oldFrame, position,
                result21);

        final NEDVelocity result22 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(
                timeInterval, oldFrame, position, result22);

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
        assertEquals(result1, result19);
        assertEquals(result1, result20);
        assertEquals(result1, result21);
        assertEquals(result1, result22);
    }

    @Test
    public void testEstimateVelocityAndReturnNew() {

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

        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDVelocity result1 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final NEDVelocity result2 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height);

        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Angle oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);
        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);
        final NEDVelocity result3 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                latitudeAngle, longitudeAngle, heightDistance);

        final NEDVelocity result4 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                latitudeAngle, longitudeAngle, heightDistance);

        final NEDVelocity result5 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, latitudeAngle, longitudeAngle,
                heightDistance);

        final NEDVelocity result6 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, latitudeAngle,
                longitudeAngle, heightDistance);

        final NEDVelocity oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final NEDVelocity result7 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                oldHeight, oldVelocity, latitude, longitude, height);

        final NEDVelocity result8 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, latitude, longitude, height);

        final NEDVelocity result9 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, latitudeAngle, longitudeAngle,
                heightDistance);

        final NEDVelocity result10 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, latitudeAngle, longitudeAngle,
                heightDistance);

        final NEDFrame oldFrame = new NEDFrame(oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd);
        final NEDVelocity result11 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, latitude, longitude,
                height);

        final NEDVelocity result12 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                timeInterval, oldFrame, latitude, longitude, height);

        final NEDVelocity result13 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, latitudeAngle,
                longitudeAngle, heightDistance);

        final NEDVelocity result14 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                timeInterval, oldFrame, latitudeAngle, longitudeAngle,
                heightDistance);

        final NEDPosition oldPosition = new NEDPosition(
                oldLatitude, oldLongitude, oldHeight);
        final NEDPosition position = new NEDPosition(latitude, longitude, height);
        final NEDVelocity result15 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldVn, oldVe, oldVd,
                position);

        final NEDVelocity result16 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                timeInterval, oldPosition, oldVn, oldVe, oldVd,
                position);

        final NEDVelocity result17 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldSpeedN,
                oldSpeedE, oldSpeedD, position);

        final NEDVelocity result18 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                timeInterval, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD,
                position);

        final NEDVelocity result19 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldVelocity,
                position);

        final NEDVelocity result20 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                timeInterval, oldPosition, oldVelocity, position);

        final NEDVelocity result21 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, position);

        final NEDVelocity result22 = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                timeInterval, oldFrame, position);

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
        assertEquals(result1, result19);
        assertEquals(result1, result20);
        assertEquals(result1, result21);
        assertEquals(result1, result22);
    }

    @Test(expected = IllegalArgumentException.class)
    public void testEstimateWhenZeroTimeIntervalThrowsException() {
        final NEDVelocityEstimator estimator = new NEDVelocityEstimator();

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

        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDVelocity result1 = new NEDVelocity();
        estimator.estimate(0.0, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result1);
    }

    @Test
    public void testWhenNoPositionChange() {

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

            final NEDVelocity result = NEDVelocityEstimator.estimateVelocityAndReturnNew(
                    TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                    oldVn, oldVe, oldVd, oldLatitude, oldLongitude, oldHeight);

            assertEquals(result.getVn(), -oldVn, ABSOLUTE_ERROR);
            assertEquals(result.getVe(), -oldVe, ABSOLUTE_ERROR);
            assertEquals(result.getVd(), -oldVd, ABSOLUTE_ERROR);
        }
    }
}
