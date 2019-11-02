package com.irurueta.navigation.inertial.estimators;

import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.GravityNED;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class GravityNEDEstimatorTest {
    private static final double LATITUDE_DEGREES = 41.3825;
    private static final double LONGITUDE_DEGREES = 2.176944;
    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double GRAVITY = 9.81;
    private static final double ABSOLUTE_ERROR = 0.03;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final int TIMES = 100;

    @Test
    public void testConstants() {
        assertEquals(GravityNEDEstimator.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
        assertEquals(GravityNEDEstimator.EARTH_POLAR_RADIUS_WGS84,
                Constants.EARTH_POLAR_RADIUS_WGS84, 0.0);
        assertEquals(GravityNEDEstimator.EARTH_ECCENTRICITY,
                Constants.EARTH_ECCENTRICITY, 0.0);
        assertEquals(GravityNEDEstimator.EARTH_FLATTENING_WGS84,
                Constants.EARTH_FLATTENING_WGS84, 0.0);
        assertEquals(GravityNEDEstimator.EARTH_GRAVITATIONAL_CONSTANT,
                Constants.EARTH_GRAVITATIONAL_CONSTANT, 0.0);
        assertEquals(GravityNEDEstimator.EARTH_ROTATION_RATE,
                Constants.EARTH_ROTATION_RATE, 0.0);
    }

    @Test
    public void testEstimateWithCoordinates() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double latitude = Math.toRadians(LATITUDE_DEGREES);
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final GravityNEDEstimator estimator = new GravityNEDEstimator();
        final GravityNED gravity = new GravityNED();
        estimator.estimate(latitude, height, gravity);

        final double g = Math.sqrt(Math.pow(gravity.getGn(), 2.0)
                + Math.pow(gravity.getGe(), 2.0)
                + Math.pow(gravity.getGd(), 2.0));

        assertEquals(g, GRAVITY, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateAndReturnWithCoordinates() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double latitude = Math.toRadians(LATITUDE_DEGREES);
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final GravityNEDEstimator estimator = new GravityNEDEstimator();
        final GravityNED gravity = estimator.estimateAndReturnNew(latitude, height);

        final double g = Math.sqrt(Math.pow(gravity.getGn(), 2.0)
                + Math.pow(gravity.getGe(), 2.0)
                + Math.pow(gravity.getGd(), 2.0));

        assertEquals(g, GRAVITY, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateWithNEDFrame() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double latitude = Math.toRadians(LATITUDE_DEGREES);
        final double longitude = Math.toRadians(LONGITUDE_DEGREES);
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDFrame frame = new NEDFrame(latitude, longitude, height);

        final GravityNEDEstimator estimator = new GravityNEDEstimator();
        final GravityNED gravity = new GravityNED();
        estimator.estimate(frame, gravity);

        final double g = Math.sqrt(Math.pow(gravity.getGn(), 2.0)
                + Math.pow(gravity.getGe(), 2.0)
                + Math.pow(gravity.getGd(), 2.0));

        assertEquals(g, GRAVITY, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateAndReturnNewWithNEDFrame() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double latitude = Math.toRadians(LATITUDE_DEGREES);
        final double longitude = Math.toRadians(LONGITUDE_DEGREES);
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDFrame frame = new NEDFrame(latitude, longitude, height);

        final GravityNEDEstimator estimator = new GravityNEDEstimator();
        final GravityNED gravity = estimator.estimateAndReturnNew(frame);

        final double g = Math.sqrt(Math.pow(gravity.getGn(), 2.0)
                + Math.pow(gravity.getGe(), 2.0)
                + Math.pow(gravity.getGd(), 2.0));

        assertEquals(g, GRAVITY, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateWithAngleAndDistance() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final Angle latitudeAngle = new Angle(LATITUDE_DEGREES, AngleUnit.DEGREES);
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);

        final GravityNEDEstimator estimator = new GravityNEDEstimator();
        final GravityNED gravity = new GravityNED();
        estimator.estimate(latitudeAngle, heightDistance, gravity);

        final double g = Math.sqrt(Math.pow(gravity.getGn(), 2.0)
                + Math.pow(gravity.getGe(), 2.0)
                + Math.pow(gravity.getGd(), 2.0));

        assertEquals(g, GRAVITY, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateAndReturnNewWithAngleAndDistance() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final Angle latitudeAngle = new Angle(LATITUDE_DEGREES, AngleUnit.DEGREES);
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);

        final GravityNEDEstimator estimator = new GravityNEDEstimator();
        final GravityNED gravity = estimator.estimateAndReturnNew(
                latitudeAngle, heightDistance);

        final double g = Math.sqrt(Math.pow(gravity.getGn(), 2.0)
                + Math.pow(gravity.getGe(), 2.0)
                + Math.pow(gravity.getGd(), 2.0));

        assertEquals(g, GRAVITY, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateForMultipleLatitudes() {
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double latitude = Math.toRadians(randomizer.nextDouble(
                    MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final GravityNED gravity = GravityNEDEstimator.estimateGravityAndReturnNew(
                    latitude, height);

            final double g = Math.sqrt(Math.pow(gravity.getGn(), 2.0)
                    + Math.pow(gravity.getGe(), 2.0)
                    + Math.pow(gravity.getGd(), 2.0));

            assertEquals(g, GRAVITY, ABSOLUTE_ERROR);
        }
    }
}
