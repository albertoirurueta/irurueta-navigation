package com.irurueta.navigation.inertial.estimators;

import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.RadiiOfCurvature;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class RadiiOfCurvatureEstimatorTest {
    private static final double LATITUDE_DEGREES = 41.3825;
    private static final double EQUATOR_LATITUDE_DEGREES = 0.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    @Test
    public void testConstants() {
        assertEquals(RadiiOfCurvatureEstimator.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
        assertEquals(RadiiOfCurvatureEstimator.EARTH_ECCENTRICITY,
                Constants.EARTH_ECCENTRICITY, 0.0);
    }

    @Test
    public void testEstimate() {
        final RadiiOfCurvatureEstimator estimator = new RadiiOfCurvatureEstimator();

        final RadiiOfCurvature radii = new RadiiOfCurvature();
        estimator.estimate(Math.toRadians(LATITUDE_DEGREES), radii);

        final RadiiOfCurvature northPoleRadii = new RadiiOfCurvature();
        estimator.estimate(Math.toRadians(MAX_LATITUDE_DEGREES), northPoleRadii);

        final RadiiOfCurvature southPoleRadii = new RadiiOfCurvature();
        estimator.estimate(Math.toRadians(MIN_LATITUDE_DEGREES), southPoleRadii);

        final RadiiOfCurvature equatorRadii = new RadiiOfCurvature();
        estimator.estimate(Math.toRadians(EQUATOR_LATITUDE_DEGREES), equatorRadii);

        assertTrue(radii.getRn() > equatorRadii.getRn());
        assertTrue(radii.getRn() < northPoleRadii.getRn());
        assertTrue(radii.getRn() < southPoleRadii.getRn());
        assertEquals(northPoleRadii.getRn(), southPoleRadii.getRn(), ABSOLUTE_ERROR);

        assertTrue(radii.getRe() > equatorRadii.getRe());
        assertTrue(radii.getRe() < northPoleRadii.getRe());
        assertTrue(radii.getRe() < southPoleRadii.getRe());
        assertEquals(northPoleRadii.getRe(), southPoleRadii.getRe(), ABSOLUTE_ERROR);
        assertEquals(equatorRadii.getRe(), RadiiOfCurvatureEstimator.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
    }

    @Test
    public void testEstimateAndReturnNew() {
        final RadiiOfCurvatureEstimator estimator = new RadiiOfCurvatureEstimator();

        final RadiiOfCurvature radii = estimator.estimateAndReturnNew(LATITUDE_DEGREES);
        final RadiiOfCurvature northPoleRadii = estimator.estimateAndReturnNew(
                MAX_LATITUDE_DEGREES);
        final RadiiOfCurvature southPoleRadii = estimator.estimateAndReturnNew(
                MIN_LATITUDE_DEGREES);
        final RadiiOfCurvature equatorRadii = estimator.estimateAndReturnNew(
                EQUATOR_LATITUDE_DEGREES);

        assertTrue(radii.getRn() > equatorRadii.getRn());
        assertTrue(radii.getRn() < northPoleRadii.getRn());
        assertTrue(radii.getRn() < southPoleRadii.getRn());
        assertEquals(northPoleRadii.getRn(), southPoleRadii.getRn(), ABSOLUTE_ERROR);

        assertTrue(radii.getRe() > equatorRadii.getRe());
        assertTrue(radii.getRe() < northPoleRadii.getRe());
        assertTrue(radii.getRe() < southPoleRadii.getRe());
        assertEquals(northPoleRadii.getRe(), southPoleRadii.getRe(), ABSOLUTE_ERROR);
        assertEquals(equatorRadii.getRe(), RadiiOfCurvatureEstimator.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
    }

    @Test
    public void testEstimateRadiiOfCurvature() {
        final RadiiOfCurvature radii = new RadiiOfCurvature();
        RadiiOfCurvatureEstimator.estimateRadiiOfCurvature
                (Math.toRadians(LATITUDE_DEGREES), radii);

        final RadiiOfCurvature northPoleRadii = new RadiiOfCurvature();
        RadiiOfCurvatureEstimator.estimateRadiiOfCurvature(
                Math.toRadians(MAX_LATITUDE_DEGREES), northPoleRadii);

        final RadiiOfCurvature southPoleRadii = new RadiiOfCurvature();
        RadiiOfCurvatureEstimator.estimateRadiiOfCurvature(
                Math.toRadians(MIN_LATITUDE_DEGREES), southPoleRadii);

        final RadiiOfCurvature equatorRadii = new RadiiOfCurvature();
        RadiiOfCurvatureEstimator.estimateRadiiOfCurvature(
                Math.toRadians(EQUATOR_LATITUDE_DEGREES), equatorRadii);

        assertTrue(radii.getRn() > equatorRadii.getRn());
        assertTrue(radii.getRn() < northPoleRadii.getRn());
        assertTrue(radii.getRn() < southPoleRadii.getRn());
        assertEquals(northPoleRadii.getRn(), southPoleRadii.getRn(), ABSOLUTE_ERROR);

        assertTrue(radii.getRe() > equatorRadii.getRe());
        assertTrue(radii.getRe() < northPoleRadii.getRe());
        assertTrue(radii.getRe() < southPoleRadii.getRe());
        assertEquals(northPoleRadii.getRe(), southPoleRadii.getRe(), ABSOLUTE_ERROR);
        assertEquals(equatorRadii.getRe(), RadiiOfCurvatureEstimator.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
    }

    @Test
    public void testEstimateRadiiOfCurvatureAndAndReturnNew() {
        final RadiiOfCurvature radii = RadiiOfCurvatureEstimator
                .estimateRadiiOfCurvatureAndReturnNew(LATITUDE_DEGREES);
        final RadiiOfCurvature northPoleRadii = RadiiOfCurvatureEstimator
                .estimateRadiiOfCurvatureAndReturnNew(MAX_LATITUDE_DEGREES);
        final RadiiOfCurvature southPoleRadii = RadiiOfCurvatureEstimator
                .estimateRadiiOfCurvatureAndReturnNew(MIN_LATITUDE_DEGREES);
        final RadiiOfCurvature equatorRadii = RadiiOfCurvatureEstimator
                .estimateRadiiOfCurvatureAndReturnNew(EQUATOR_LATITUDE_DEGREES);

        assertTrue(radii.getRn() > equatorRadii.getRn());
        assertTrue(radii.getRn() < northPoleRadii.getRn());
        assertTrue(radii.getRn() < southPoleRadii.getRn());
        assertEquals(northPoleRadii.getRn(), southPoleRadii.getRn(), ABSOLUTE_ERROR);

        assertTrue(radii.getRe() > equatorRadii.getRe());
        assertTrue(radii.getRe() < northPoleRadii.getRe());
        assertTrue(radii.getRe() < southPoleRadii.getRe());
        assertEquals(northPoleRadii.getRe(), southPoleRadii.getRe(), ABSOLUTE_ERROR);
        assertEquals(equatorRadii.getRe(), RadiiOfCurvatureEstimator.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
    }
}
