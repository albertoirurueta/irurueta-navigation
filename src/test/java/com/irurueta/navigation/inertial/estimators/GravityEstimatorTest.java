package com.irurueta.navigation.inertial.estimators;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.Gravity;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class GravityEstimatorTest {
    private static final double LATITUDE_DEGREES = 41.3825;
    private static final double LONGITUDE_DEGREES = 2.176944;
    private static final double HEIGHT = 0.0;

    private static final double GRAVITY = 9.81;
    private static final double ABSOLUTE_ERROR = 0.03;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final int TIMES = 100;

    @Test
    public void testConstants() {
        assertEquals(GravityEstimator.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
        assertEquals(GravityEstimator.EARTH_GRAVITATIONAL_CONSTANT,
                Constants.EARTH_GRAVITATIONAL_CONSTANT, 0.0);
        assertEquals(GravityEstimator.EARTH_SECOND_GRAVITATIONAL_CONSTANT,
                Constants.EARTH_SECOND_GRAVITATIONAL_CONSTANT, 0.0);
        assertEquals(GravityEstimator.EARTH_ROTATION_RATE,
                Constants.EARTH_ROTATION_RATE, 0.0);
    }

    @Test
    public void testEstimateWithCoordinates() {
        final NEDFrame nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES),
                Math.toRadians(LONGITUDE_DEGREES), HEIGHT);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final double x = ecefFrame.getX();
        final double y = ecefFrame.getY();
        final double z = ecefFrame.getZ();

        final GravityEstimator estimator = new GravityEstimator();
        final Gravity gravity = new Gravity();
        estimator.estimate(x, y, z, gravity);

        final double g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                + Math.pow(gravity.getGy(), 2.0)
                + Math.pow(gravity.getGz(), 2.0));

        assertEquals(g, GRAVITY, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateAndReturnNewWithCoordinates() {
        final NEDFrame nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES),
                Math.toRadians(LONGITUDE_DEGREES), HEIGHT);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final double x = ecefFrame.getX();
        final double y = ecefFrame.getY();
        final double z = ecefFrame.getZ();

        final GravityEstimator estimator = new GravityEstimator();
        final Gravity gravity = estimator.estimateAndReturnNew(x, y, z);

        final double g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                + Math.pow(gravity.getGy(), 2.0)
                + Math.pow(gravity.getGz(), 2.0));

        assertEquals(g, GRAVITY, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateWithECEFFrame() {
        final NEDFrame nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES),
                Math.toRadians(LONGITUDE_DEGREES), HEIGHT);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final GravityEstimator estimator = new GravityEstimator();
        final Gravity gravity = new Gravity();
        estimator.estimate(ecefFrame, gravity);

        final double g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                + Math.pow(gravity.getGy(), 2.0)
                + Math.pow(gravity.getGz(), 2.0));

        assertEquals(g, GRAVITY, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateAndReturnNewWithECEFFrame() {
        final NEDFrame nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES),
                Math.toRadians(LONGITUDE_DEGREES), HEIGHT);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final GravityEstimator estimator = new GravityEstimator();
        final Gravity gravity = estimator.estimateAndReturnNew(ecefFrame);

        final double g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                + Math.pow(gravity.getGy(), 2.0)
                + Math.pow(gravity.getGz(), 2.0));

        assertEquals(g, GRAVITY, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateWithPosition() {
        final NEDFrame nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES),
                Math.toRadians(LONGITUDE_DEGREES), HEIGHT);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final double x = ecefFrame.getX();
        final double y = ecefFrame.getY();
        final double z = ecefFrame.getZ();

        final Point3D position = new InhomogeneousPoint3D(x, y, z);

        final GravityEstimator estimator = new GravityEstimator();
        final Gravity gravity = new Gravity();
        estimator.estimate(position, gravity);

        final double g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                + Math.pow(gravity.getGy(), 2.0)
                + Math.pow(gravity.getGz(), 2.0));

        assertEquals(g, GRAVITY, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateAndReturnNewWithPosition() {
        final NEDFrame nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES),
                Math.toRadians(LONGITUDE_DEGREES), HEIGHT);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final double x = ecefFrame.getX();
        final double y = ecefFrame.getY();
        final double z = ecefFrame.getZ();

        final Point3D position = new InhomogeneousPoint3D(x, y, z);

        final GravityEstimator estimator = new GravityEstimator();
        final Gravity gravity = estimator.estimateAndReturnNew(position);

        final double g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                + Math.pow(gravity.getGy(), 2.0)
                + Math.pow(gravity.getGz(), 2.0));

        assertEquals(g, GRAVITY, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateForAGivenLatitudeAndLongitude() {
        final NEDFrame nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES),
                Math.toRadians(LONGITUDE_DEGREES), HEIGHT);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final Gravity gravity = GravityEstimator.estimateGravityAndReturnNew(ecefFrame);

        final double g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                + Math.pow(gravity.getGy(), 2.0)
                + Math.pow(gravity.getGz(), 2.0));

        assertEquals(g, GRAVITY, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateForMultipleLatitudesAndLongitudes() {
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double latitude = Math.toRadians(randomizer.nextDouble(
                    MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(randomizer.nextDouble(
                    MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));

            final NEDFrame nedFrame = new NEDFrame(latitude, longitude, HEIGHT);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            final Gravity gravity = GravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            final double g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                    + Math.pow(gravity.getGy(), 2.0)
                    + Math.pow(gravity.getGz(), 2.0));

            assertEquals(g, GRAVITY, ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testEstimateAtCenterOfEarth() {
        final GravityEstimator estimator = new GravityEstimator();
        final Gravity gravity = estimator.estimateAndReturnNew(0.0, 0.0, 0.0);

        // check
        assertEquals(gravity.getGx(), 0.0, 0.0);
        assertEquals(gravity.getGy(), 0.0, 0.0);
        assertEquals(gravity.getGz(), 0.0, 0.0);
    }
}
