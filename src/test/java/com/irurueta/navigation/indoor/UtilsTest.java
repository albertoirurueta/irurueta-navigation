/*
 * Copyright (C) 2018 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.navigation.indoor;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.indoor.radiosource.RssiRadioSourceEstimator;
import com.irurueta.numerical.*;
import com.irurueta.statistics.MultivariateNormalDist;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

@SuppressWarnings("Duplicates")
public class UtilsTest {

    private static final Logger LOGGER = Logger.getLogger(
            UtilsTest.class.getName());


    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-4;

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final double MIN_RSSI = -100.0;
    private static final double MAX_RSSI = 100.0;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double MIN_DISTANCE = 0.5;
    private static final double MAX_DISTANCE = 50.0;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double TX_POWER_VARIANCE = 0.1;
    private static final double RX_POWER_VARIANCE = 0.5;
    private static final double PATHLOSS_EXPONENT_VARIANCE = 0.001;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final int TIMES = 50;

    public UtilsTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testdBmToPowerAndPowerTodBm() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double value = randomizer.nextDouble();

        assertEquals(Utils.powerTodBm(Utils.dBmToPower(value)), value,
                ABSOLUTE_ERROR);
        assertEquals(Utils.dBmToPower(Utils.powerTodBm(value)), value,
                ABSOLUTE_ERROR);
    }

    @Test
    public void testPropagatePowerVarianceToDistanceVariance()
            throws EvaluationException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double minDistanceVariance = Double.MAX_VALUE;
        double maxDistanceVariance = -Double.MAX_VALUE;
        double avgDistanceVariance = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final double txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double txPower = Utils.dBmToPower(txPowerdBm);

            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            double rxPowerdBm = Utils.powerTodBm(
                    receivedPower(txPower, distance,
                            pathLossExponent));

            double distanceVariance = Utils.propagatePowerVarianceToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    RX_POWER_VARIANCE);
            assertTrue(distanceVariance > 0.0);

            double k = SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY);
            double kdB = 10.0 * Math.log10(k);
            double derivative = -Math.log(10.0) / (10.0 * pathLossExponent) *
                    Math.pow(10.0, (pathLossExponent * kdB + txPowerdBm - rxPowerdBm) /
                    (10.0 * pathLossExponent));

            DerivativeEstimator derivativeEstimator = new DerivativeEstimator(new SingleDimensionFunctionEvaluatorListener() {
                @Override
                public double evaluate(double point) {
                    double k = RssiRadioSourceEstimator.SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY);
                    double kdB = 10.0 * Math.log10(k);
                    double logSqrDistance = (pathLossExponent * kdB + txPowerdBm - point) / (5.0 * pathLossExponent);
                    return Math.pow(10.0,logSqrDistance / 2.0);
                }
            });

            double derivative2 = derivativeEstimator.derivative(rxPowerdBm);
            assertEquals(derivative, derivative2, LARGE_ABSOLUTE_ERROR);

            assertEquals(distanceVariance,
                    derivative * derivative * RX_POWER_VARIANCE, 0.0);

            if (distanceVariance < minDistanceVariance) {
                minDistanceVariance = distanceVariance;
            }
            if (distanceVariance > maxDistanceVariance) {
                maxDistanceVariance = distanceVariance;
            }
            avgDistanceVariance += distanceVariance / TIMES;

            //check that if rx power variance is zero, then distance variance is
            // zero as well
            distanceVariance = Utils.propagatePowerVarianceToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    0.0);
            assertEquals(distanceVariance, 0.0, 0.0);

            //test without rx power
            assertEquals(Utils.propagatePowerVarianceToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    null), 0.0, 0.0);
        }

        LOGGER.log(Level.INFO, "Min dist variance: {0}", minDistanceVariance);
        LOGGER.log(Level.INFO, "Max dist variance: {0}", maxDistanceVariance);
        LOGGER.log(Level.INFO, "Avg dist variance: {0}", avgDistanceVariance);
    }

    @Test
    public void testPropagateVariancesToDistanceVariance()
            throws IndoorException, EvaluationException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double minDistanceVariance = Double.MAX_VALUE;
        double maxDistanceVariance = -Double.MAX_VALUE;
        double avgDistanceVariance = 0.0;
        for (int t = 0; t < TIMES; t++) {
            double txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double txPower = Utils.dBmToPower(txPowerdBm);

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            double rxPowerdBm = Utils.powerTodBm(
                    receivedPower(txPower, distance,
                            pathLossExponent));

            MultivariateNormalDist dist = Utils.propagateVariancesToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    TX_POWER_VARIANCE, RX_POWER_VARIANCE, PATHLOSS_EXPONENT_VARIANCE);

            double k = RssiRadioSourceEstimator.SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY);
            final double kdB = 10.0 * Math.log10(k);

            double tenPathLossExponent = 10.0 * pathLossExponent;
            double tmp = (pathLossExponent * kdB + txPowerdBm - rxPowerdBm) / tenPathLossExponent;
            double derivativeTxPower = Math.log(10.0) / tenPathLossExponent * Math.pow(10.0,
                    tmp);
            double derivativeRxPower = -Math.log(10.0) / tenPathLossExponent * Math.pow(10.0,
                    tmp);

            double g = (pathLossExponent * kdB + txPowerdBm - rxPowerdBm) / (10.0 * pathLossExponent);
            double derivativeG = (kdB * 10.0 * pathLossExponent -
                    10.0 * (pathLossExponent * kdB + txPowerdBm - rxPowerdBm)) /
                    Math.pow(10.0 * pathLossExponent, 2.0);

            double derivativePathLossExponent = Math.log(10.0) * derivativeG * Math.pow(10.0, g);

            double[] gradient = new double[]{
                    derivativeTxPower,
                    derivativeRxPower,
                    derivativePathLossExponent
            };

            GradientEstimator gradientEstimator = new GradientEstimator(new MultiDimensionFunctionEvaluatorListener() {
                @Override
                public double evaluate(double[] point) {
                    double txPower = point[0];
                    double rxPower = point[1];
                    double pathLossExponent = point[2];

                    double logSqrDistance = (pathLossExponent * kdB + txPower - rxPower) / (5.0 * pathLossExponent);
                    return Math.pow(10.0,logSqrDistance / 2.0);
                }
            });

            double[] gradient2 = gradientEstimator.gradient(
                    new double[]{
                            txPowerdBm,
                            rxPowerdBm,
                            pathLossExponent});

            assertArrayEquals(gradient, gradient2, LARGE_ABSOLUTE_ERROR);

            assertEquals(dist.getMean()[0], distance, ABSOLUTE_ERROR);

            double distanceVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertTrue(distanceVariance > 0.0);

            if (distanceVariance < minDistanceVariance) {
                minDistanceVariance = distanceVariance;
            }
            if (distanceVariance > maxDistanceVariance) {
                maxDistanceVariance = distanceVariance;
            }
            avgDistanceVariance += distanceVariance / TIMES;

            //check without variances
            assertNull(Utils.propagateVariancesToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    null, null, null));
        }

        LOGGER.log(Level.INFO, "Min dist variance: {0}", minDistanceVariance);
        LOGGER.log(Level.INFO, "Max dist variance: {0}", maxDistanceVariance);
        LOGGER.log(Level.INFO, "Avg dist variance: {0}", avgDistanceVariance);
    }

    @Test
    public void testPropagateTxPowerVarianceToDistanceVariance() throws IndoorException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double minDistanceVariance = Double.MAX_VALUE;
        double maxDistanceVariance = -Double.MAX_VALUE;
        double avgDistanceVariance = 0.0;
        for (int t = 0; t < TIMES; t++) {
            double txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double txPower = Utils.dBmToPower(txPowerdBm);

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            double rxPowerdBm = Utils.powerTodBm(
                    receivedPower(txPower, distance,
                            pathLossExponent));

            MultivariateNormalDist dist = Utils.propagateVariancesToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    TX_POWER_VARIANCE, 0.0, 0.0);

            assertEquals(dist.getMean()[0], distance, ABSOLUTE_ERROR);

            double distanceVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertTrue(distanceVariance > 0.0);

            if (distanceVariance < minDistanceVariance) {
                minDistanceVariance = distanceVariance;
            }
            if (distanceVariance > maxDistanceVariance) {
                maxDistanceVariance = distanceVariance;
            }
            avgDistanceVariance += distanceVariance / TIMES;
        }

        LOGGER.log(Level.INFO, "Min dist variance: {0}", minDistanceVariance);
        LOGGER.log(Level.INFO, "Max dist variance: {0}", maxDistanceVariance);
        LOGGER.log(Level.INFO, "Avg dist variance: {0}", avgDistanceVariance);
    }

    @Test
    public void testPropagateRxPowerVarianceToDistanceVariance() throws IndoorException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double minDistanceVariance = Double.MAX_VALUE;
        double maxDistanceVariance = -Double.MAX_VALUE;
        double avgDistanceVariance = 0.0;
        for (int t = 0; t < TIMES; t++) {
            double txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double txPower = Utils.dBmToPower(txPowerdBm);

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            double rxPowerdBm = Utils.powerTodBm(
                    receivedPower(txPower, distance,
                            pathLossExponent));

            MultivariateNormalDist dist = Utils.propagateVariancesToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    0.0, RX_POWER_VARIANCE, 0.0);

            assertEquals(dist.getMean()[0], distance, ABSOLUTE_ERROR);

            double distanceVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertTrue(distanceVariance > 0.0);

            double distanceVariance2 = Utils.propagatePowerVarianceToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    RX_POWER_VARIANCE);
            assertEquals(distanceVariance, distanceVariance2, ABSOLUTE_ERROR);

            if (distanceVariance < minDistanceVariance) {
                minDistanceVariance = distanceVariance;
            }
            if (distanceVariance > maxDistanceVariance) {
                maxDistanceVariance = distanceVariance;
            }
            avgDistanceVariance += distanceVariance / TIMES;
        }

        LOGGER.log(Level.INFO, "Min dist variance: {0}", minDistanceVariance);
        LOGGER.log(Level.INFO, "Max dist variance: {0}", maxDistanceVariance);
        LOGGER.log(Level.INFO, "Avg dist variance: {0}", avgDistanceVariance);
    }

    @Test
    public void testPropagatePathlossExponentVarianceToDistanceVariance() throws IndoorException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double minDistanceVariance = Double.MAX_VALUE;
        double maxDistanceVariance = -Double.MAX_VALUE;
        double avgDistanceVariance = 0.0;
        for (int t = 0; t < TIMES; t++) {
            double txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double txPower = Utils.dBmToPower(txPowerdBm);

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            double rxPowerdBm = Utils.powerTodBm(
                    receivedPower(txPower, distance,
                            pathLossExponent));

            MultivariateNormalDist dist = Utils.propagateVariancesToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    0.0, 0.0, PATHLOSS_EXPONENT_VARIANCE);

            assertEquals(dist.getMean()[0], distance, ABSOLUTE_ERROR);

            double distanceVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertTrue(distanceVariance > 0.0);

            if (distanceVariance < minDistanceVariance) {
                minDistanceVariance = distanceVariance;
            }
            if (distanceVariance > maxDistanceVariance) {
                maxDistanceVariance = distanceVariance;
            }
            avgDistanceVariance += distanceVariance / TIMES;
        }

        LOGGER.log(Level.INFO, "Min dist variance: {0}", minDistanceVariance);
        LOGGER.log(Level.INFO, "Max dist variance: {0}", maxDistanceVariance);
        LOGGER.log(Level.INFO, "Avg dist variance: {0}", avgDistanceVariance);
    }

    @Test
    public void testPropagateNoVarianceToDistanceVariance() throws IndoorException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            double txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double txPower = Utils.dBmToPower(txPowerdBm);

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            double rxPowerdBm = Utils.powerTodBm(
                    receivedPower(txPower, distance,
                            pathLossExponent));

            MultivariateNormalDist dist = Utils.propagateVariancesToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    0.0, 0.0, 0.0);

            assertEquals(dist.getMean()[0], distance, ABSOLUTE_ERROR);

            double distanceVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(distanceVariance, 0.0, ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testPropagateVariancesToRssiVarianceFirstOrderNonLinear2D()
            throws IndoorException, AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            double fingerprintRssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            double x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            double y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            Point2D fingerprintPosition = new InhomogeneousPoint2D(x1, y1);

            double xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            double ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            Point2D radioSourcePosition = new InhomogeneousPoint2D(xa, ya);

            double xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            double yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            Point2D estimatedPosition = new InhomogeneousPoint2D(xi, yi);

            //test without variance values
            MultivariateNormalDist dist = Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, null,
                    null, null,
                    null, null);

            double diffX1a = x1 - xa;
            double diffY1a = y1 - ya;

            double diffXi1 = xi - x1;
            double diffYi1 = yi - y1;

            double diffX1a2 = diffX1a * diffX1a;
            double diffY1a2 = diffY1a * diffY1a;

            double d1a2 = diffX1a2 + diffY1a2;

            double rssi = fingerprintRssi - 10.0 * pathLossExponent *
                    (diffX1a * diffXi1 + diffY1a * diffYi1)/(Math.log(10.0) * d1a2);

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);

            double rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            //test with variance values
            dist = Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, 0.0,
                    0.0, new Matrix(2,2),
                    new Matrix(2, 2), new Matrix(2, 2));

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);
            rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, null, radioSourcePosition,
                    estimatedPosition, null,
                    null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, null,
                    estimatedPosition, null, null,
                    null, null,
                    null));
            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, radioSourcePosition,
                    null, null,
                    null, null,
                    null, null));
        }
    }

    @Test
    public void testPropagateVariancesToRssiVarianceFirstOrderNonLinear3D()
            throws IndoorException, AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            double fingerprintRssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            double x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            double y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            double z1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            Point3D fingerprintPosition = new InhomogeneousPoint3D(x1, y1, z1);

            double xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            double ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            double za = randomizer.nextDouble(MIN_POS, MAX_POS);
            Point3D radioSourcePosition = new InhomogeneousPoint3D(xa, ya, za);

            double xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            double yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            double zi = randomizer.nextDouble(MIN_POS, MAX_POS);
            Point3D estimatedPosition = new InhomogeneousPoint3D(xi, yi, zi);

            //test without variance values
            MultivariateNormalDist dist = Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, null,
                    null, null,
                    null, null);

            double diffX1a = x1 - xa;
            double diffY1a = y1 - ya;
            double diffZ1a = z1 - za;

            double diffXi1 = xi - x1;
            double diffYi1 = yi - y1;
            double diffZi1 = zi - z1;

            double diffX1a2 = diffX1a * diffX1a;
            double diffY1a2 = diffY1a * diffY1a;
            double diffZ1a2 = diffZ1a * diffZ1a;

            double d1a2 = diffX1a2 + diffY1a2 + diffZ1a2;

            double rssi = fingerprintRssi - 10.0 * pathLossExponent *
                    (diffX1a * diffXi1 + diffY1a * diffYi1 + diffZ1a * diffZi1) /
                    (Math.log(10.0) * d1a2);

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);

            double rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            //test with variance values
            dist = Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, 0.0,
                    0.0, new Matrix(3,3),
                    new Matrix(3, 3), new Matrix(3, 3));

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);
            rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, null, radioSourcePosition,
                    estimatedPosition, null,
                    null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, null,
                    estimatedPosition, null, null,
                    null, null,
                    null));
            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, radioSourcePosition,
                    null, null,
                    null, null,
                    null, null));
        }
    }

    @Test
    public void testPropagateVariancesToRssiVarianceSecondOrderNonLinear2D()
            throws IndoorException, AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            double fingerprintRssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            double x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            double y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            Point2D fingerprintPosition = new InhomogeneousPoint2D(x1, y1);

            double xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            double ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            Point2D radioSourcePosition = new InhomogeneousPoint2D(xa, ya);

            double xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            double yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            Point2D estimatedPosition = new InhomogeneousPoint2D(xi, yi);

            //test without variance values
            MultivariateNormalDist dist = Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, null,
                    null, null,
                    null, null);

            double diffX1a = x1 - xa;
            double diffY1a = y1 - ya;

            double diffXi1 = xi - x1;
            double diffYi1 = yi - y1;

            double diffX1a2 = diffX1a * diffX1a;
            double diffY1a2 = diffY1a * diffY1a;

            double diffXi12 = diffXi1 * diffXi1;
            double diffYi12 = diffYi1 * diffYi1;

            double d1a2 = diffX1a2 + diffY1a2;
            double d1a4 = d1a2 * d1a2;
            double ln10 = Math.log(10.0);

            double rssi = fingerprintRssi - 10.0 * pathLossExponent *
                    (diffX1a * diffXi1 + diffY1a * diffYi1)/(ln10 * d1a2) +
                    5.0 * pathLossExponent * ((diffX1a2 - diffY1a2) * (diffXi12 - diffYi12)) / (ln10 * d1a4) +
                    20.0 * pathLossExponent * diffX1a * diffY1a * diffXi1 * diffYi1 / (ln10 * d1a4);

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);

            double rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            //test with variance values
            dist = Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, 0.0,
                    0.0, new Matrix(2,2),
                    new Matrix(2, 2), new Matrix(2, 2));

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);
            rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            assertNull(Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, null, radioSourcePosition,
                    estimatedPosition, null,
                    null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, null,
                    estimatedPosition, null, null,
                    null, null,
                    null));
            assertNull(Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, radioSourcePosition,
                    null, null,
                    null, null,
                    null, null));
        }
    }

    private double receivedPower(double equivalentTransmittedPower,
                                 double distance, double pathLossExponent) {
        //Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        //Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY), pathLossExponent);
        return equivalentTransmittedPower * k /
                Math.pow(distance, pathLossExponent);
    }
}
