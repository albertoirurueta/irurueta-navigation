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
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.indoor.radiosource.RssiRadioSourceEstimator;
import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.JacobianEstimator;
import com.irurueta.numerical.MultiVariateFunctionEvaluatorListener;
import com.irurueta.statistics.MultivariateNormalDist;
import com.irurueta.statistics.StatisticsException;

@SuppressWarnings("Duplicates")
public class Utils {

    /**
     * Speed of light expressed in meters per second (m/s).
     */
    public static final double SPEED_OF_LIGHT = RssiRadioSourceEstimator.SPEED_OF_LIGHT;

    /**
     * Prevents instantiation
     */
    private Utils() {
    }

    /**
     * Converts from dBm's to linear power value expressed in mW.
     *
     * @param dBm value to be converted expressed in dBm's.
     * @return converted value expressed in mW.
     */
    public static double dBmToPower(final double dBm) {
        return Math.pow(10.0, dBm / 10.0);
    }

    /**
     * Converts from mW to logarithmic power value expressed in dBm's.
     *
     * @param mW value to be converted expressed in mW's.
     * @return converted value expressed in dBm's.
     */
    public static double powerTodBm(final double mW) {
        return 10.0 * Math.log10(mW);
    }

    /**
     * Propagates variance on received power measure into distance variance by considering the following formula
     * for received power (expressed in dBm's):
     * rxPower = pathLossExponent * kdB + txPower - 5.0 * pathLossExponent * logSqrDistance,
     * where logSqrDistance is the logarithm in base 10 of the squared distance logSqrDistance = Math.log(d^2).
     * Taking into account the previous formula, distance can be expressed as:
     * d = 10.0^((pathLossExponent * kdB + txPower - rxPower)/(10.0 * pathLossExponent))
     * where kdB is a constant having the following expression:
     * kdB = 10.0 * log(c / (4 * pi * f)),
     * where c is the speed of light and f is the frequency.
     *
     * @param txPower          transmitted power expressed in dBm's.
     * @param rxPower          received power expressed in dBm's.
     * @param pathLossExponent path loss exponent.
     * @param frequency        frequency expressed in Hz.
     * @param rxPowerVariance  received power variance.
     * @return distance variance.
     */
    public static double propagatePowerVarianceToDistanceVariance(
            final double txPower, final double rxPower,
            final double pathLossExponent, final double frequency,
            final Double rxPowerVariance) {
        if (rxPowerVariance == null) {
            return 0.0;
        }

        final double k = SPEED_OF_LIGHT / (4.0 * Math.PI * frequency);
        final double kdB = 10.0 * Math.log10(k);

        //distance follows the following expression:
        //d = 10.0^((pathLossExponent * kdB + txPower - rxPower)/(10.0 * pathLossExponent))
        //where kdB is a constant having the following expression:
        //kdB = 10.0 * log(c / (4 * pi * f)),
        //where c is the speed of light and f is the frequency.

        //hence, if the only unknown is the received power (x = rxPower), we can express the distance as:
        //d = f(x) = 10.0^((pathLossExponent * kdB + txPower - x)/(10.0 * pathLossExponent))

        //if we know the variance of received power var(x), then the variance of the distance will be:
        //var(d) = var(f(x)) = (f'(E(x)))^2*var(x)
        //where f'(x) is the derivative of f(x) evaluated at E(x) = rxPower

        //the derivative f'(x) has the following expression:
        //f'(x) = -ln(10)/(10.0*pathLossExponent)*10^((pathLossExponent * kdB + txPower - x)/(10.0 * pathLossExponent))

        //evaluate derivative at E(x) = rxPower:
        final double tenPathLossExponent = 10.0 * pathLossExponent;
        final double derivativeF = -Math.log(10.0) / tenPathLossExponent * Math.pow(10.0,
                (pathLossExponent * kdB + txPower - rxPower) / tenPathLossExponent);

        return derivativeF * derivativeF * rxPowerVariance;
    }

    /**
     * Propagates provided variances (transmitted power variance, received power variance and pathloss variance) into
     * distance variance by considering the following formula for received power (expressed in dBm's):
     * rxPower = pathLossExponent * kdB + txPower - 5.0 * pathLossExponent * logSqrDistance,
     * where logSqrDistance is the logarithm in base 10 of the squared distance logSqrDistance = Math.log(d^2).
     * Taking into account the previous formula, distance can be expressed as:
     * d = 10.0^((pathLossExponent * kdB + txPower - rxPower)/(10.0 * pathLossExponent))
     * where kdB is a constant having the following expression:
     * kdB = 10.0 * log(c / (4 * pi * f)),
     * where c is the speed of light and f is the frequency.
     *
     * @param txPower                  transmitted power expressed in dBm's.
     * @param rxPower                  received power expressed in dBm's.
     * @param pathLossExponent         path loss exponent.
     * @param frequency                frequency expressed in Hz.
     * @param txPowerVariance          transmitted power variance.
     * @param rxPowerVariance          received power variance.
     * @param pathLossExponentVariance path loss exponent variance.
     * @return a normal distribution containing both expected distance and its variance.
     * @throws IndoorException if something fails.
     */
    public static MultivariateNormalDist propagateVariancesToDistanceVariance(
            final double txPower,
            final double rxPower, final double pathLossExponent, final double frequency,
            final Double txPowerVariance, final Double rxPowerVariance,
            final Double pathLossExponentVariance) throws IndoorException {
        if (txPowerVariance == null && rxPowerVariance == null && pathLossExponentVariance == null) {
            return null;
        }

        final double[] mean = new double[]{txPower, rxPower, pathLossExponent};
        final Matrix covariance = Matrix.diagonal(new double[]{
                txPowerVariance != null ? txPowerVariance : 0.0,
                rxPowerVariance != null ? rxPowerVariance : 0.0,
                pathLossExponentVariance != null ? pathLossExponentVariance : 0.0
        });

        try {
            return MultivariateNormalDist.propagate(new MultivariateNormalDist.JacobianEvaluator() {
                @Override
                public void evaluate(
                        final double[] x, final double[] y, final Matrix jacobian) {
                    final double k = RssiRadioSourceEstimator.SPEED_OF_LIGHT / (4.0 * Math.PI * frequency);
                    final double kdB = 10.0 * Math.log10(k);

                    //received power in dBm's follows the equation:
                    //rxPower = pathLossExponent * kdB + txPower - 5.0 * pathLossExponent * logSqrDistance

                    //hence, distance follows the following expression:
                    //d = 10.0^((pathLossExponent * kdB + txPower - rxPower)/(10.0 * pathLossExponent))
                    //where kdB is a constant having the following expression:
                    //kdB = 10.0 * log(c / (4 * pi * f)),
                    //where c is the speed of light and f is the frequency.

                    final double logSqrDistance = (pathLossExponent * kdB + txPower - rxPower)
                            / (5.0 * pathLossExponent);

                    //where logSqrDistance = Math.log10(sqrDistance)
                    //and sqrDistance = distance * distance, hence
                    //logSqrDistance = Math.log10(distance * distance) = 2 * Math.log10(distance)

                    y[0] = Math.pow(10.0, logSqrDistance / 2.0);


                    //compute gradient (is a jacobian having 1 row and 3 columns)

                    //derivative of distance respect to transmitted power is:

                    //if the only unknown is the transmitted power (x = txPower), then:
                    //d = f(x) = 10.0^((pathLossExponent * kdB + x - rxPower)/(10.0 * pathLossExponent))
                    //and the derivative is
                    //f'(x) = ln(10)/(10.0 * pathLossExponent)*10.0^((pathLossExponent * kdB + x - rxPower)/(10.0 * pathLossExponent))
                    final double tenPathLossExponent = 10.0 * pathLossExponent;
                    final double tenPowered = Math.pow(10.0,
                            (pathLossExponent * kdB + txPower - rxPower) / tenPathLossExponent);
                    final double derivativeTxPower = Math.log(10.0) / tenPathLossExponent * tenPowered;

                    //derivative of distance respect to received power is:

                    //if the only unknown is the received power (x = rxPower), then:
                    //d = f(x) = 10.0^((pathLossExponent * kdB + txPower - x)/(10.0 * pathLossExponent))
                    //and the derivative is
                    //f'(x) = -ln(10)/(10.0*pathLossExponent)*10^((pathLossExponent * kdB + txPower - x)/(10.0 * pathLossExponent))
                    final double derivativeRxPower = -Math.log(10.0) / tenPathLossExponent * tenPowered;


                    //derivative respect to path loss exponent is:

                    //if the only unknown is the path loss exponent (x = pathLossExponent), then:
                    //d = f(x) = 10.0^((x * kdB + txPower - rxPower)/(10.0 * x))
                    //and the derivative is:
                    //f'(x) = ln(10) * g'(x) * 10.0^(g(x))
                    //where g(x) is:
                    //g(x) = (x * kdB + txPower - rxPower) / (10.0 * x)
                    //and the derivative of g(x) is:
                    //g'(x) = (kdB * 10.0 * x - 10.0 * (x * kdB + txPower - rxPower)) / (10.0 * x)^2
                    //Hence:
                    //f'(x) = lng(10) * (kdB * 10.0 * x - 10.0 * (x * kdB + txPower - rxPower)) / (10.0 * x)^2 * 10.0^((x * kdB + txPower - rxPower)/(10.0 * x))

                    final double g = (pathLossExponent * kdB + txPower - rxPower) / (10.0 * pathLossExponent);
                    final double derivativeG = (kdB * 10.0 * pathLossExponent -
                            10.0 * (pathLossExponent * kdB + txPower - rxPower)) /
                            Math.pow(10.0 * pathLossExponent, 2.0);

                    final double derivativePathLossExponent = Math.log(10.0) * derivativeG * Math.pow(10.0, g);

                    jacobian.setElementAtIndex(0, derivativeTxPower);
                    jacobian.setElementAtIndex(1, derivativeRxPower);
                    jacobian.setElementAtIndex(2, derivativePathLossExponent);
                }

                @Override
                public int getNumberOfVariables() {
                    return 1;
                }
            }, mean, covariance);
        } catch (final AlgebraException | StatisticsException e) {
            throw new IndoorException(e);
        }
    }

    /**
     * Propagates provided variances (fingerprint rssi variance, path-loss exponent variance,
     * fingerprint position covariance and radio source position covariance) into
     * rssi variance by considering the 2D 1st order Taylor expression of received power.
     * Notice that any unknown variance is assumed to be zero.
     *
     * @param fingerprintRssi               closest located fingerprint reading RSSI expressed in dBm's.
     * @param pathLossExponent              path-loss exponent.
     * @param fingerprintPosition           position of closest fingerprint.
     * @param radioSourcePosition           radio source position associated to fingerprint reading.
     * @param estimatedPosition             position to be estimated. Usually this is equal to the
     *                                      initial position used by a non linear algorithm.
     * @param fingerprintRssiVariance       variance of fingerprint RSSI or null if unknown.
     * @param pathLossExponentVariance      variance of path-loss exponent or null if unknown.
     * @param fingerprintPositionCovariance covariance of fingerprint position or null if
     *                                      unknown.
     * @param radioSourcePositionCovariance covariance of radio source position or null
     *                                      if unknown.
     * @param estimatedPositionCovariance   covariance of position to be estimated or null
     *                                      if unknown. (This is usually unknown).
     * @return a normal distribution containing expected received RSSI value and its variance.
     * @throws IndoorException if something fails.
     */
    public static MultivariateNormalDist propagateVariancesToRssiVarianceFirstOrderNonLinear2D(
            final double fingerprintRssi, final double pathLossExponent,
            final Point2D fingerprintPosition, final Point2D radioSourcePosition,
            final Point2D estimatedPosition,
            final Double fingerprintRssiVariance,
            final Double pathLossExponentVariance,
            final Matrix fingerprintPositionCovariance,
            final Matrix radioSourcePositionCovariance,
            final Matrix estimatedPositionCovariance) throws IndoorException {

        if (fingerprintPosition == null || radioSourcePosition == null ||
                estimatedPosition == null) {
            return null;
        }

        //1st order Taylor expression of received power in 2D:
        //Pr(pi) = Pr(p1)
        //  - 10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
        //  - 10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
        //where d1a^2 = (x1 - xa)^2 + (y1 - ya)^2

        final double x1 = fingerprintPosition.getInhomX();
        final double y1 = fingerprintPosition.getInhomY();

        final double xa = radioSourcePosition.getInhomX();
        final double ya = radioSourcePosition.getInhomY();

        final double xi = estimatedPosition.getInhomX();
        final double yi = estimatedPosition.getInhomY();

        final double[] mean = new double[]{
                fingerprintRssi, pathLossExponent, x1, y1, xa, ya, xi, yi
        };
        final Matrix covariance = Matrix.diagonal(new double[]{
                fingerprintRssiVariance != null ? fingerprintRssiVariance : 0.0,
                pathLossExponentVariance != null ? pathLossExponentVariance : 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        });

        if (fingerprintPositionCovariance != null &&
                fingerprintPositionCovariance.getRows() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                fingerprintPositionCovariance.getColumns() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {

            covariance.setSubmatrix(2, 2,
                    3, 3,
                    fingerprintPositionCovariance);
        }

        if (radioSourcePositionCovariance != null &&
                radioSourcePositionCovariance.getRows() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                radioSourcePositionCovariance.getColumns() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            covariance.setSubmatrix(4, 4,
                    5, 5,
                    radioSourcePositionCovariance);
        }

        if (estimatedPositionCovariance != null &&
                estimatedPositionCovariance.getRows() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                estimatedPositionCovariance.getColumns() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            covariance.setSubmatrix(6, 6,
                    7, 7,
                    estimatedPositionCovariance);
        }

        try {
            return MultivariateNormalDist.propagate(new MultivariateNormalDist.JacobianEvaluator() {
                @Override
                public void evaluate(
                        final double[] x, final double[] y, final Matrix jacobian) {

                    //Pr(pi) = Pr(p1)
                    //  - 10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
                    //  - 10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
                    //where d1a^2 = (x1 - xa)^2 + (y1 - ya)^2

                    //Hence:
                    //Pr(pi) = Pr(p1) -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/
                    //      (ln(10)*((x1 - xa)^2 + (y1 - ya)^2))

                    final double diffX1a = x1 - xa;
                    final double diffY1a = y1 - ya;

                    final double diffXi1 = xi - x1;
                    final double diffYi1 = yi - y1;

                    final double diffX1a2 = diffX1a * diffX1a;
                    final double diffY1a2 = diffY1a * diffY1a;

                    final double d1a2 = diffX1a2 + diffY1a2;
                    final double d1a4 = d1a2 * d1a2;

                    final double ln10 = Math.log(10.0);
                    final double crossDiff = diffX1a * diffXi1 + diffY1a * diffYi1;

                    y[0] = fingerprintRssi
                            - 10.0 * pathLossExponent * crossDiff / (ln10 * d1a2);

                    //compute gradient (is a jacobian having 1 row and 8 columns)


                    //derivative of rssi respect to fingerprint rssi
                    final double derivativeFingerprintRssi = 1.0;

                    //derivative of rssi respect to path-loss exponent

                    //diff(Pr(pi))/diff(n) = -10*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
                    //  -10*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
                    final double derivativePathLossExponent = -10.0 * crossDiff /
                            (ln10 * d1a2);


                    //derivative of rssi respect to x1

                    //We have
                    //Pr(pi) = Pr(p1) -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/
                    //      (ln(10)*((x1 - xa)^2 + (y1 - ya)^2))

                    //and we know that: (f(x)/g(x))' = (f'(x)*g(x) - f(x)*g'(x))/g(x)^2
                    //and also that (f(x)*g(x))' = f'(x)*g(x) + f(x)*g'(x)

                    //Hence
                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/diff(x1) =
                    //  diff(x1*xi -xa*xi -x1^2 + xa*x1)/diff(x1) =
                    //  diff(-x1^2 + (xi + xa)*x1 - xa*xi)/diff(x1) =
                    //  -2*x1 + xi + xa

                    //diff(Pr(pi))/diff(x1) = -10*n/ln(10)*((-2*x1 + xi + xa)*((x1 - xa)^2 + (y1 - ya)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2)^2
                    //diff(Pr(pi))/diff(x1) = -10*n/ln(10)*((-2*x1 + xi + xa)*d1a^2 - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*2*(x1 - xa))/d1a^4
                    final double tmpX = 2.0 * crossDiff * diffX1a;
                    final double derivativeX1 = -10.0 * pathLossExponent / ln10 * ((-2.0 * x1 + xi + xa) * d1a2
                            - tmpX) / d1a4;

                    //derivative of rssi respect to y1

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/diff(y1) =
                    //  diff(y1*yi -ya*yi -y1^2 + ya*y1)/diff(y1) =
                    //  diff(-y1^2 + (yi + ya)*y1 - ya*yi)/diff(y1) =
                    //  -2*y1 + yi + ya

                    //diff(Pr(pi))/diff(y1) = -10*n/ln(10)*((-2*y1 + yi + ya)*((x1 - xa)^2 + (y1 - ya)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2)^2
                    //diff(Pr(pi))/diff(y1) = -10*n/ln(10)*((-2*y1 + yi + ya)*d1a^2 - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*2*(y1 - ya))/d1a^4
                    final double tmpY = 2.0 * crossDiff * diffY1a;
                    final double derivativeY1 = -10.0 * pathLossExponent / ln10 * ((-2.0 * y1 + yi + ya) * d1a2
                            - tmpY) / d1a4;


                    //derivative of rssi respect to xa

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/diff(xa) =
                    //  diff(x1*xi -xa*xi -x1^2 + xa*x1)/diff(xa) =
                    //  x1 - xi

                    //diff(Pr(pi))/diff(xa) = -10*n/ln(10)*((x1 - xi)*((x1 - xa)^2 + (y1 - ya)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*-2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2)^2
                    //diff(Pr(pi))/diff(xa) = -10*n/ln(10)*(-(xi - x1)*d1a^2 + ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*2*(x1 - xa))/d1a^4
                    final double derivativeXa = -10.0 * pathLossExponent / ln10 * (-diffXi1 * d1a2
                            + tmpX) / d1a4;


                    //derivative of rssi respect to ya

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/diff(ya) =
                    //  diff(y1*yi -y1^2 -ya*yi + ya*y1)/diff(ya) =
                    //  y1 - yi

                    //diff(Pr(pi))/diff(ya) = -10*n/ln(10)*((y1 - yi)*((x1 - xa)^2 + (y1 - ya)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*-2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2)^2
                    //diff(Pr(pi))/diff(ya) = -10*n/ln(10)*(-(yi - y1)*d1a^2 + ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*2*(y1 - ya))/d1a^4
                    final double derivativeYa = -10.0 * pathLossExponent / ln10 * (-diffYi1 * d1a2
                            + tmpY) / d1a4;


                    //derivative of rssi respect to xi

                    //Pr(pi) = Pr(p1) -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/
                    //      (ln(10)*((x1 - xa)^2 + (y1 - ya)^2))

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/diff(xi) =
                    //  diff(x1*xi -xa*xi -x1^2 + xa*x1)/diff(xi) =
                    //  x1 - xa

                    //diff(Pr(pi))/diff(xi) = -10*n/ln(10)*((x1 - xa)*((x1 - xa)^2 + (y1 - ya)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*0)/(x1 - xa)^2 + (y1 - ya)^2)^2
                    //diff(Pr(pi))/diff(xi) = -10*n/ln(10)*((x1 - xa)*d1a^2)/d1a^4
                    //diff(Pr(pi))/diff(xi) = -10*n*(x1 - xa)/(ln(10)*d1a^2)
                    final double derivativeXi = -10.0 * pathLossExponent * diffX1a / (ln10 * d1a2);


                    //derivative of rssi respect to yi

                    //Pr(pi) = Pr(p1) -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/
                    //      (ln(10)*((x1 - xa)^2 + (y1 - ya)^2))

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/diff(yi) =
                    //  diff(y1*yi -ya*yi -y1^2 + ya*y1)/diff(yi) =
                    //  y1 - ya

                    //diff(Pr(pi))/diff(yi) = -10*n/ln(10)*((y1 - ya)*((x1 - xa)^2 + (y1 - ya)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*0)/((x1 - xa)^2 + (y1 - ya)^2)^2
                    //diff(Pr(pi))/diff(yi) = -10*n/ln(10)*((y1 - ya)*d1a^2)/d1a^4
                    //diff(Pr(pi))/diff(yi) = -10*n*(y1 - ya)/(ln(10)*d1a^2)
                    final double derivativeYi = -10.0 * pathLossExponent * diffY1a / (ln10 * d1a2);

                    //set derivatives fingerprintRssi, pathLossExponent, x1, y1, xa, ya, xi, yi
                    jacobian.setElementAtIndex(0, derivativeFingerprintRssi);
                    jacobian.setElementAtIndex(1, derivativePathLossExponent);
                    jacobian.setElementAtIndex(2, derivativeX1);
                    jacobian.setElementAtIndex(3, derivativeY1);
                    jacobian.setElementAtIndex(4, derivativeXa);
                    jacobian.setElementAtIndex(5, derivativeYa);
                    jacobian.setElementAtIndex(6, derivativeXi);
                    jacobian.setElementAtIndex(7, derivativeYi);
                }

                @Override
                public int getNumberOfVariables() {
                    return 1;
                }
            }, mean, covariance);
        } catch (final AlgebraException | StatisticsException e) {
            throw new IndoorException(e);
        }
    }

    /**
     * Propagates provided variances (fingerprint rssi variance, path-loss exponent variance,
     * fingerprint position covariance and radio source position covariance) into
     * rssi variance by considering the 3D 1st order Taylor expression of received power.
     * Notice that any unknown variance is assumed to be zero.
     *
     * @param fingerprintRssi               closest located fingerprint reading RSSI expressed in dBm's.
     * @param pathLossExponent              path-loss exponent.
     * @param fingerprintPosition           position of closest fingerprint.
     * @param radioSourcePosition           radio source position associated to fingerprint reading.
     * @param estimatedPosition             position to be estimated. Usually this is equal to the
     *                                      initial position used by a non linear algorithm.
     * @param fingerprintRssiVariance       variance of fingerprint RSSI or null if unknown.
     * @param pathLossExponentVariance      variance of path-loss exponent or null if unknown.
     * @param fingerprintPositionCovariance covariance of fingerprint position or null if
     *                                      unknown.
     * @param radioSourcePositionCovariance covariance of radio source position or null
     *                                      if unknown.
     * @param estimatedPositionCovariance   covariance of position to be estimated or null
     *                                      if unknown. (This is usually unknown).
     * @return a normal distribution containing expected received RSSI value and its variance.
     * @throws IndoorException if something fails.
     */
    public static MultivariateNormalDist propagateVariancesToRssiVarianceFirstOrderNonLinear3D(
            final double fingerprintRssi, final double pathLossExponent,
            final Point3D fingerprintPosition, final Point3D radioSourcePosition,
            final Point3D estimatedPosition,
            final Double fingerprintRssiVariance,
            final Double pathLossExponentVariance,
            final Matrix fingerprintPositionCovariance,
            final Matrix radioSourcePositionCovariance,
            final Matrix estimatedPositionCovariance) throws IndoorException {

        if (fingerprintPosition == null || radioSourcePosition == null ||
                estimatedPosition == null) {
            return null;
        }

        //1st order Taylor expression of received power in 3D:
        //Pr(pi) = Pr(p1)
        //  - 10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
        //  - 10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
        //  - 10*n*(z1 - za)/(ln(10)*d1a^2)*(zi - z1)
        //where d1a^2 = (x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2

        final double x1 = fingerprintPosition.getInhomX();
        final double y1 = fingerprintPosition.getInhomY();
        final double z1 = fingerprintPosition.getInhomZ();

        final double xa = radioSourcePosition.getInhomX();
        final double ya = radioSourcePosition.getInhomY();
        final double za = radioSourcePosition.getInhomZ();

        final double xi = estimatedPosition.getInhomX();
        final double yi = estimatedPosition.getInhomY();
        final double zi = estimatedPosition.getInhomZ();

        final double[] mean = new double[]{
                fingerprintRssi, pathLossExponent, x1, y1, z1, xa, ya, za, xi, yi, zi
        };
        final Matrix covariance = Matrix.diagonal(new double[]{
                fingerprintRssiVariance != null ? fingerprintRssiVariance : 0.0,
                pathLossExponentVariance != null ? pathLossExponentVariance : 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        });

        if (fingerprintPositionCovariance != null &&
                fingerprintPositionCovariance.getRows() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                fingerprintPositionCovariance.getColumns() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {

            covariance.setSubmatrix(2, 2,
                    4, 4,
                    fingerprintPositionCovariance);
        }

        if (radioSourcePositionCovariance != null &&
                radioSourcePositionCovariance.getRows() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                radioSourcePositionCovariance.getColumns() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            covariance.setSubmatrix(5, 5,
                    7, 7,
                    radioSourcePositionCovariance);
        }

        if (estimatedPositionCovariance != null &&
                estimatedPositionCovariance.getRows() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                estimatedPositionCovariance.getColumns() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            covariance.setSubmatrix(8, 8,
                    10, 10,
                    estimatedPositionCovariance);
        }

        try {
            return MultivariateNormalDist.propagate(new MultivariateNormalDist.JacobianEvaluator() {
                @Override
                public void evaluate(
                        final double[] x, final double[] y, final Matrix jacobian) {

                    //Pr(pi) = Pr(p1)
                    //  - 10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
                    //  - 10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
                    //  - 10*n*(z1 - za)/(ln(10)*d1a^2)*(zi - z1)
                    //where d1a^2 = (x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2

                    //Hence:
                    //Pr(pi) = Pr(p1) -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/
                    //      (ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))

                    final double diffX1a = x1 - xa;
                    final double diffY1a = y1 - ya;
                    final double diffZ1a = z1 - za;

                    final double diffXi1 = xi - x1;
                    final double diffYi1 = yi - y1;
                    final double diffZi1 = zi - z1;

                    final double diffX1a2 = diffX1a * diffX1a;
                    final double diffY1a2 = diffY1a * diffY1a;
                    final double diffZ1a2 = diffZ1a * diffZ1a;

                    final double d1a2 = diffX1a2 + diffY1a2 + diffZ1a2;
                    final double d1a4 = d1a2 * d1a2;

                    final double ln10 = Math.log(10.0);
                    final double crossDiff = diffX1a * diffXi1 + diffY1a * diffYi1 + diffZ1a * diffZi1;

                    y[0] = fingerprintRssi
                            - 10.0 * pathLossExponent * crossDiff / (ln10 * d1a2);

                    //compute gradient (is a jacobian having 1 row and 11 columns)


                    //derivative of rssi respect to fingerprint rssi
                    final double derivativeFingerprintRssi = 1.0;

                    //derivative of rssi respect to path-loss exponent

                    //diff(Pr(pi))/diff(n) = -10*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
                    //  -10*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
                    final double derivativePathLossExponent = -10.0 * crossDiff /
                            (ln10 * d1a2);


                    //derivative of rssi respect to x1

                    //We have
                    //Pr(pi) = Pr(p1) -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/
                    //      (ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))

                    //and we know that: (f(x)/g(x))' = (f'(x)*g(x) - f(x)*g'(x))/g(x)^2
                    //and also that (f(x)*g(x))' = f'(x)*g(x) + f(x)*g'(x)

                    //Hence
                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(x1) =
                    //  diff(x1*xi -xa*xi -x1^2 + xa*x1)/diff(x1) =
                    //  diff(-x1^2 + (xi + xa)*x1 - xa*xi)/diff(x1) =
                    //  -2*x1 + xi + xa

                    //diff(Pr(pi))/diff(x1) = -10*n/ln(10)*((-2*x1 + xi + xa)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //diff(Pr(pi))/diff(x1) = -10*n/ln(10)*((-2*x1 + xi + xa)*d1a^2 - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(x1 - xa))/d1a^4
                    final double tmpX = 2.0 * crossDiff * diffX1a;
                    final double derivativeX1 = -10.0 * pathLossExponent / ln10 * ((-2.0 * x1 + xi + xa) * d1a2
                            - tmpX) / d1a4;


                    //derivative of rssi respect to y1

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(y1) =
                    //  diff(y1*yi -ya*yi -y1^2 + ya*y1)/diff(y1) =
                    //  diff(-y1^2 + (yi + ya)*y1 - ya*yi)/diff(y1) =
                    //  -2*y1 + yi + ya

                    //diff(Pr(pi))/diff(y1) = -10*n/ln(10)*((-2*y1 + yi + ya)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //diff(Pr(pi))/diff(y1) = -10*n/ln(10)*((-2*y1 + yi + ya)*d1a^2 - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(y1 - ya))/d1a^4
                    final double tmpY = 2.0 * crossDiff * diffY1a;
                    final double derivativeY1 = -10.0 * pathLossExponent / ln10 * ((-2.0 * y1 + yi + ya) * d1a2
                            - tmpY) / d1a4;


                    //derivative of rssi respect to z1

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(z1) =
                    //  diff(z1*zi -za*zi -z1^2 + za*z1)/diff(z1) =
                    //  diff(-z1^2 + (zi + za)*z1 - za*zi)/diff(z1) =
                    //  -2*z1 + zi + za

                    //diff(Pr(pi))/diff(z1) = -10*n/ln(10)*((-2*z1 + zi + za)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //diff(Pr(pi))/diff(z1) = -10*n/ln(10)*((-2*z1 + zi + za)*d1a^2 - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(z1 - za))/d1a^4
                    final double tmpZ = 2.0 * crossDiff * diffZ1a;
                    final double derivativeZ1 = -10.0 * pathLossExponent / ln10 * ((-2.0 * z1 + z1 + za) * d1a2
                            - tmpZ) / d1a4;


                    //derivative of rssi respect to xa

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(xa) =
                    //  diff(x1*xi -xa*xi -x1^2 + xa*x1)/diff(xa) =
                    //  x1 - xi

                    //diff(Pr(pi))/diff(xa) = -10*n/ln(10)*((x1 - xi)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*-2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //diff(Pr(pi))/diff(xa) = -10*n/ln(10)*(-(xi - x1)*d1a^2 + ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(x1 - xa))/d1a^4
                    final double derivativeXa = -10.0 * pathLossExponent / ln10 * (-diffXi1 * d1a2
                            + tmpX) / d1a4;


                    //derivative of rssi respect to ya

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(ya) =
                    //  diff(y1*yi -y1^2 -ya*yi + ya*y1)/diff(ya) =
                    //  y1 - yi

                    //diff(Pr(pi))/diff(ya) = -10*n/ln(10)*((y1 - yi)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*-2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //diff(Pr(pi))/diff(ya) = -10*n/ln(10)*(-(yi - y1)*d1a^2 + ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(y1 - ya))/d1a^4
                    final double derivativeYa = -10.0 * pathLossExponent / ln10 * (-diffYi1 * d1a2
                            + tmpY) / d1a4;


                    //derivative of rssi respect to za

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(za) =
                    //  diff(z1*zi -z1^2 -za*zi + za*z1)/diff(za) =
                    //  z1 - zi

                    //diff(Pr(pi))/diff(za) = -10*n/ln(10)*((z1 - zi)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*-2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //diff(Pr(pi))/diff(za) = -10*n/ln(10)*(-(zi - z1)*d1a^2 + ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(z1 - za))/d1a^4
                    final double derivativeZa = -10.0 * pathLossExponent / ln10 * (-diffZi1 * d1a2 +
                            +tmpZ) / d1a4;


                    //derivative of rssi respect to xi

                    //Pr(pi) = Pr(p1) -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/
                    //      (ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(xi) =
                    //  diff(x1*xi -xa*xi -x1^2 + xa*x1)/diff(xi) =
                    //  x1 - xa

                    //diff(Pr(pi))/diff(xi) = -10*n/ln(10)*((x1 - xa)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //diff(Pr(pi))/diff(xi) = -10*n/ln(10)*((x1 - xa)*d1a^2)/d1a^4
                    //diff(Pr(pi))/diff(xi) = -10*n*(x1 - xa)/(ln(10)*d1a^2)
                    final double derivativeXi = -10.0 * pathLossExponent * diffX1a / (ln10 * d1a2);


                    //derivative of rssi respect to yi

                    //Pr(pi) = Pr(p1) -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/
                    //      (ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(yi) =
                    //  diff(y1*yi -ya*yi -y1^2 + ya*y1)/diff(yi) =
                    //  y1 - ya

                    //diff(Pr(pi))/diff(yi) = -10*n/ln(10)*((y1 - ya)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //diff(Pr(pi))/diff(yi) = -10*n/ln(10)*((y1 - ya)*d1a^2)/d1a^4
                    //diff(Pr(pi))/diff(yi) = -10*n*(y1 - ya)/(ln(10)*d1a^2)
                    final double derivativeYi = -10.0 * pathLossExponent * diffY1a / (ln10 * d1a2);


                    //derivative of rssi respect to zi

                    //Pr(pi) = Pr(p1) -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/
                    //      (ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(zi) =
                    //  diff(z1*zi -za*zi -z1^2 + za*z1)/diff(zi) =
                    //  z1 - za

                    //diff(Pr(pi))/diff(zi) = -10*n/ln(10)*((z1 - za)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //diff(Pr(pi))/diff(zi) = -10*n/ln(10)*((z1 - za)*d1a^2)/d1a^4
                    //diff(Pr(pi))/diff(zi) = -10*n*(z1 - za)/(ln(10*d1a^2)
                    final double derivativeZi = -10.0 * pathLossExponent * diffZ1a / (ln10 * d1a2);

                    //set derivatives fingerprintRssi, pathLossExponent, x1, y1, z1, xa, ya, za, xi, yi, zi
                    jacobian.setElementAtIndex(0, derivativeFingerprintRssi);
                    jacobian.setElementAtIndex(1, derivativePathLossExponent);
                    jacobian.setElementAtIndex(2, derivativeX1);
                    jacobian.setElementAtIndex(3, derivativeY1);
                    jacobian.setElementAtIndex(4, derivativeZ1);
                    jacobian.setElementAtIndex(5, derivativeXa);
                    jacobian.setElementAtIndex(6, derivativeYa);
                    jacobian.setElementAtIndex(7, derivativeZa);
                    jacobian.setElementAtIndex(8, derivativeXi);
                    jacobian.setElementAtIndex(9, derivativeYi);
                    jacobian.setElementAtIndex(10, derivativeZi);
                }

                @Override
                public int getNumberOfVariables() {
                    return 1;
                }
            }, mean, covariance);
        } catch (final AlgebraException | StatisticsException e) {
            throw new IndoorException(e);
        }
    }

    /**
     * Propagates provided variances (fingerprint rssi variance, path-loss exponent variance,
     * fingerprint position covariance and radio source position covariance) into
     * rssi variance by considering the 2D 2nd order Taylor expression of received power.
     * Notice that any unknown variance is assumed to be zero.
     *
     * @param fingerprintRssi               closest located fingerprint reading RSSI expressed in dBm's.
     * @param pathLossExponent              path-loss exponent.
     * @param fingerprintPosition           position of closest fingerprint.
     * @param radioSourcePosition           radio source position associated to fingerprint reading.
     * @param estimatedPosition             position to be estimated. Usually this is equal to the
     *                                      initial position used by a non linear algorithm.
     * @param fingerprintRssiVariance       variance of fingerprint RSSI or null if unknown.
     * @param pathLossExponentVariance      variance of path-loss exponent or null if unknown.
     * @param fingerprintPositionCovariance covariance of fingerprint position or null if
     *                                      unknown.
     * @param radioSourcePositionCovariance covariance of radio source position or null
     *                                      if unknown.
     * @param estimatedPositionCovariance   covariance of position to be estimated or null
     *                                      if unknown. (This is usually unknown).
     * @return a normal distribution containing expected received RSSI value and its variance.
     * @throws IndoorException if something fails.
     */
    public static MultivariateNormalDist propagateVariancesToRssiVarianceSecondOrderNonLinear2D(
            final double fingerprintRssi, final double pathLossExponent,
            final Point2D fingerprintPosition, final Point2D radioSourcePosition,
            final Point2D estimatedPosition, final Double fingerprintRssiVariance,
            final Double pathLossExponentVariance,
            final Matrix fingerprintPositionCovariance,
            final Matrix radioSourcePositionCovariance,
            final Matrix estimatedPositionCovariance) throws IndoorException {

        if (fingerprintPosition == null || radioSourcePosition == null ||
                estimatedPosition == null) {
            return null;
        }

        //2nd order Taylor expression of received power in 2D:
        //Pr(pi) = Pr(p1)
        //  - 10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
        //  - 10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
        //  - 5*n*((y1 - ya)^2 - (x1 - xa)^2)/(ln(10)*d1a^4)*(xi - x1)^2
        //  - 5*n*((x1 - xa)^2 - (y1 - ya)^2)/(ln(10)*d1a^4)*(yi - y1)^2
        //  + 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4))*(xi - x1)*(yi - y1)
        //where d1a^2 = (x1 - xa)^2 + (y1 - ya)^2

        final double x1 = fingerprintPosition.getInhomX();
        final double y1 = fingerprintPosition.getInhomY();

        final double xa = radioSourcePosition.getInhomX();
        final double ya = radioSourcePosition.getInhomY();

        final double xi = estimatedPosition.getInhomX();
        final double yi = estimatedPosition.getInhomY();

        final double[] mean = new double[]{
                fingerprintRssi, pathLossExponent, x1, y1, xa, ya, xi, yi
        };
        final Matrix covariance = Matrix.diagonal(new double[]{
                fingerprintRssiVariance != null ? fingerprintRssiVariance : 0.0,
                pathLossExponentVariance != null ? pathLossExponentVariance : 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        });

        if (fingerprintPositionCovariance != null &&
                fingerprintPositionCovariance.getRows() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                fingerprintPositionCovariance.getColumns() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {

            covariance.setSubmatrix(2, 2,
                    3, 3,
                    fingerprintPositionCovariance);
        }

        if (radioSourcePositionCovariance != null &&
                radioSourcePositionCovariance.getRows() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                radioSourcePositionCovariance.getColumns() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            covariance.setSubmatrix(4, 4,
                    5, 5,
                    radioSourcePositionCovariance);
        }

        if (estimatedPositionCovariance != null &&
                estimatedPositionCovariance.getRows() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                estimatedPositionCovariance.getColumns() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            covariance.setSubmatrix(6, 6,
                    7, 7,
                    estimatedPositionCovariance);
        }

        try {
            return MultivariateNormalDist.propagate(new MultivariateNormalDist.JacobianEvaluator() {
                @Override
                public void evaluate(
                        final double[] x, final double[] y, final Matrix jacobian) {

                    //Pr(pi) = Pr(p1)
                    //  - 10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
                    //  - 10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
                    //  - 5*n*((y1 - ya)^2 - (x1 - xa)^2)/(ln(10)*d1a^4)*(xi - x1)^2
                    //  - 5*n*((x1 - xa)^2 - (y1 - ya)^2)/(ln(10)*d1a^4)*(yi - y1)^2
                    //  + 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4))*(xi - x1)*(yi - y1)
                    //where d1a^2 = (x1 - xa)^2 + (y1 - ya)^2

                    //Hence:
                    //Pr(pi) = Pr(p1)
                    //  - 10*n*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))*(xi - x1)
                    //  - 10*n*(y1 - ya)/(ln(10)*(x1 - xa)^2 + (y1 - ya)^2)*(yi - y1)
                    //  - 5*n*((y1 - ya)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(xi - x1)^2
                    //  - 5*n*((x1 - xa)^2 - (y1 - ya)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(yi - y1)^2
                    //  + 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(xi - x1)*(yi - y1)

                    final double diffX1a = x1 - xa;
                    final double diffY1a = y1 - ya;

                    final double diffXi1 = xi - x1;
                    final double diffYi1 = yi - y1;

                    final double diffX1a2 = diffX1a * diffX1a;
                    final double diffY1a2 = diffY1a * diffY1a;

                    final double diffXi12 = diffXi1 * diffXi1;
                    final double diffYi12 = diffYi1 * diffYi1;

                    final double d1a2 = diffX1a2 + diffY1a2;
                    final double d1a4 = d1a2 * d1a2;
                    final double d1a8 = d1a4 * d1a4;

                    final double ln10 = Math.log(10.0);

                    y[0] = fingerprintRssi
                            - 10.0 * pathLossExponent * diffX1a / (ln10 * d1a2) * diffXi1
                            - 10.0 * pathLossExponent * diffY1a / (ln10 * d1a2) * diffYi1
                            - 5.0 * pathLossExponent * (-diffX1a2 + diffY1a2) / (ln10 * d1a4) * diffXi12
                            - 5.0 * pathLossExponent * (diffX1a2 - diffY1a2) / (ln10 * d1a4) * diffYi12
                            + 20.0 * pathLossExponent * diffX1a * diffY1a / (ln10 * d1a4) * diffXi1 * diffYi1;

                    //compute gradient (is a jacobian having 1 row and 8 columns)


                    //derivative of rssi respect to fingerprint rssi
                    final double derivativeFingerprintRssi = 1.0;

                    //derivative of rssi respect to path-loss exponent

                    //diff(Pr(pi))/diff(n) = -10*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))*(xi - x1)
                    //  -10*(y1 - ya)/(ln(10)*(x1 - xa)^2 + (y1 - ya)^2)*(yi - y1)
                    //  -5*((y1 - ya)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(xi - x1)^2
                    //  -5*((x1 - xa)^2 - (y1 - ya)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(yi - y1)^2
                    //  +20*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(xi - x1)*(yi - y1)
                    final double derivativePathLossExponent = -10.0 * diffX1a / (ln10 * d1a2) * diffXi1
                            - 10.0 * diffY1a / (ln10 * d1a2) * diffYi1
                            - 5.0 * (-diffX1a2 + diffY1a2) / (ln10 * d1a4) * diffXi12
                            - 5.0 * (diffX1a2 - diffY1a2) / (ln10 * d1a4) * diffYi12
                            + 20.0 * diffX1a * diffY1a / (ln10 * d1a4) * diffXi1 * diffYi1;


                    //derivative of rssi respect to x1

                    //We have
                    //Pr(pi) = Pr(p1)
                    //  - 10*n*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))*(xi - x1)
                    //  - 10*n*(y1 - ya)/(ln(10)*(x1 - xa)^2 + (y1 - ya)^2)*(yi - y1)
                    //  - 5*n*((y1 - ya)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(xi - x1)^2
                    //  - 5*n*((x1 - xa)^2 - (y1 - ya)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(yi - y1)^2
                    //  + 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(xi - x1)*(yi - y1)

                    //Pr(pi) = Pr(p1)
                    //  - 10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))
                    //  + 5*n*((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2)
                    //  + 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(xi - x1)*(yi - y1)


                    //and we know that: (f(x)/g(x))' = (f'(x)*g(x) - f(x)*g'(x))/g(x)^2
                    //and also that (f(x)*g(x))' = f'(x)*g(x) + f(x)*g'(x)

                    //Hence
                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/diff(x1) =
                    //  diff(x1*xi -xa*xi -x1^2 + xa*x1)/diff(x1) =
                    //  diff(-x1^2 + (xi + xa)*x1 - xa*xi)/diff(x1) =
                    //  -2*x1 + xi + xa

                    //diff(((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2))/diff(x1) =
                    //  2*(x1 - xa)*((xi - x1)^2 - (yi - y1)^2) - 2*(xi - x1)*((x1 - xa)^2 - (y1 - ya)^2)

                    //diff((x1 - xa)*(y1 - ya))/diff(x1) =
                    //  y1 - ya

                    //diff((xi - x1)*(yi - y1))/diff(x1) =
                    // -(yi - y1)

                    //diff((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))/diff(x1) =
                    //  (y1 - ya)*(xi - x1)*(yi - y1) - (x1 - xa)*(y1 - ya)*(yi - y1) =
                    //  ((y1 - ya)*(xi - x1) - (x1 - xa)*(y1 - ya))*(yi - y1)

                    //diff(Pr(pi))/diff(x1) = -10*n/ln(10)*((-2*x1 + xi + xa)*((x1 - xa)^2 + (y1 - ya)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2)^2
                    //  + 5*n/ln(10)*(2*((x1 - xa)*((xi - x1)^2 - (yi - y1)^2) - (xi - x1)*((x1 - xa)^2 - (y1 - ya)^2))*((x1 - xa)^2 + (y1 - ya)^2)^2 - ((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2))*2*((x1 - xa)^2 + (y1 - ya)^2)*2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2)^4
                    //  + 20*n/ln(10)*(((y1 - ya)*(xi - x1) - (x1 - xa)*(y1 - ya))*(yi - y1)*((x1 - xa)^2 + (y1 - ya)^2)^2 - (x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*2*((x1 - xa)^2 + (y1 - ya)^2)*2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2)^4

                    //diff(Pr(pi))/diff(x1) = -10*n/ln(10)*((-2*x1 + xi + xa)*d1a^2 - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*2*(x1 - xa))/d1a^4
                    //  + 5*n/ln(10)*(2*((x1 - xa)*((xi - x1)^2 - (yi - y1)^2) - (xi - x1)*((x1 - xa)^2 - (y1 - ya)^2))*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2))*4*d1a^2*(x1 - xa))/d1a^8
                    //  + 20*n/ln(10)*(((y1 - ya)*(xi - x1) - (x1 - xa)*(y1 - ya))*(yi - y1)*d1a^4 - (x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*4*d1a^2*(x1 - xa))/d1a^8
                    final double crossDiff = diffX1a * diffXi1 + diffY1a * diffYi1;
                    final double tmpX = crossDiff * 2.0 * diffX1a;
                    final double tmpX2 = (diffX1a2 - diffY1a2) * (diffXi12 - diffYi12) * 4.0 * d1a2 * diffX1a;
                    final double tmpX3 = diffX1a * diffY1a * diffXi1 * diffYi1 * 4.0 * d1a2 * diffX1a;
                    final double derivativeX1 = -10.0 * pathLossExponent / ln10 * ((-2.0 * x1 + xi + xa) * d1a2 - tmpX) / d1a4
                            + 5.0 * pathLossExponent / ln10 * (2.0 * (diffX1a * (diffXi12 - diffYi12) - diffXi1 * (diffX1a2 - diffY1a2)) * d1a4 - tmpX2) / d1a8
                            + 20.0 * pathLossExponent / ln10 * ((diffY1a * diffXi1 - diffX1a * diffY1a) * diffYi1 * d1a4 - tmpX3) / d1a8;


                    //derivative of rssi respect to y1

                    //We have
                    //Pr(pi) = Pr(p1)
                    //  - 10*n*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))*(xi - x1)
                    //  - 10*n*(y1 - ya)/(ln(10)*(x1 - xa)^2 + (y1 - ya)^2)*(yi - y1)
                    //  - 5*n*((y1 - ya)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(xi - x1)^2
                    //  - 5*n*((x1 - xa)^2 - (y1 - ya)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(yi - y1)^2
                    //  + 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(xi - x1)*(yi - y1)

                    //Pr(pi) = Pr(p1)
                    //  - 10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))
                    //  + 5*n*((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2)
                    //  + 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(xi - x1)*(yi - y1)


                    //and we know that: (f(x)/g(x))' = (f'(x)*g(x) - f(x)*g'(x))/g(x)^2
                    //and also that (f(x)*g(x))' = f'(x)*g(x) + f(x)*g'(x)

                    //Hence
                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/diff(y1) =
                    //  diff(y1*yi -ya*yi -y1^2 + ya*y1)/diff(y1) =
                    //  diff(-y1^2 + (yi + ya)*y1 - ya*yi)/diff(y1) =
                    //  -2*y1 + yi + ya

                    //diff(((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2))/diff(y1) =
                    //  -2*(y1 - ya)*((xi - x1)^2 - (yi - y1)^2) + 2*(yi - y1)*((x1 - xa)^2 - (y1 - ya)^2)

                    //diff((x1 - xa)*(y1 - ya))/diff(y1) =
                    //  x1 - xa

                    //diff((xi - x1)*(yi - y1))/diff(y1) =
                    // -(xi - x1)

                    //diff((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))/diff(y1) =
                    //  (x1 - xa)*(xi - x1)*(yi - y1) - (x1 - xa)*(y1 - ya)*(xi - x1) =
                    //  ((x1 - xa)*(yi - y1) - (x1 - xa)*(y1 - ya))*(xi - x1)

                    //diff(Pr(pi))/diff(y1) = -10*n/ln(10)*((-2*y1 + yi + ya)*((x1 - xa)^2 + (y1 - ya)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2)^2
                    //  + 5*n/ln(10)*(2*(-(y1 - ya)*((xi - x1)^2 - (yi - y1)^2) + (yi - y1)*((x1 - xa)^2 - (y1 - ya)^2))*((x1 - xa)^2 + (y1 - ya)^2)^2 - ((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2)*2*((x1 - xa)^2 + (y1 - ya)^2)*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2)^4
                    //  + 20*n/ln(10)*(((x1 - xa)*(yi - y1) - (x1 - xa)*(y1 - ya))*(xi - x1)*((x1 - xa)^2 + (y1 - ya)^2)^2 - (x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*2*((x1 - xa)^2 + (y1 - ya)^2)*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2)^4

                    //diff(Pr(pi))/diff(y1) = -10*n/ln(10)*((-2*y1 + yi + ya)*d1a^2 - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*2*(y1 - ya))/d1a^4
                    //  + 5*n/ln(10)*(2*(-(y1 - ya)*((xi - x1)^2 - (yi - y1)^2) + (yi - y1)*((x1 - xa)^2 - (y1 - ya)^2))*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2)*4*d1a^2*(y1 - ya))/d1a^8
                    //  + 20*n/ln(10)*(((x1 - xa)*(yi - y1) - (x1 - xa)*(y1 - ya))*(xi - x1)*d1a^4 - (x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*4*d1a^2*(y1 - ya))/d1a^8
                    final double tmpY = crossDiff * 2.0 * diffY1a;
                    final double tmpY2 = (diffX1a2 - diffY1a2) * (diffXi12 - diffYi12) * 4.0 * d1a2 * diffY1a;
                    final double tmpY3 = diffX1a * diffY1a * diffXi1 * diffYi1 * 4.0 * d1a2 * diffY1a;
                    final double derivativeY1 = -10.0 * pathLossExponent / ln10 * ((-2.0 * y1 + yi + ya) * d1a2 - tmpY) / d1a4
                            + 5.0 * pathLossExponent / ln10 * (2.0 * (-diffY1a * (diffXi12 - diffYi12) + diffYi1 * (diffX1a2 - diffY1a2)) * d1a4 - tmpY2) / d1a8
                            + 20.0 * pathLossExponent / ln10 * ((diffX1a * diffYi1 - diffX1a * diffY1a) * diffXi1 * d1a4 - tmpY3) / d1a8;


                    //derivative of rssi respect to xa

                    //Pr(pi) = Pr(p1)
                    //  - 10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))
                    //  + 5*n*((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2)
                    //  + 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(xi - x1)*(yi - y1)

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/diff(xa) =
                    //  -(xi - x1)

                    //diff(((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2))/diff(xa) =
                    //  -2*(x1 - xa)*((xi - x1)^2 - (yi - y1)^2)

                    //diff((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))/diff(xa) =
                    //  -(y1 - ya)*(xi - x1)*(yi - y1)

                    //diff(Pr(pi))/diff(xa) = -10*n/ln(10)*(-(xi - x1)*((x1 - xa)^2 + (y1 - ya)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*-2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2)^2
                    //  + 5*n/ln(10)*(-2*(x1 - xa)*((xi - x1)^2 - (yi - y1)^2)*((x1 - xa)^2 + (y1 - ya)^2)^2 - ((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2)*2*((x1 - xa)^2 + (y1 - ya)^2)*-2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2)^4
                    //  + 20*n*(-(y1 - ya)*(xi - x1)*(yi - y1)*((x1 - xa)^2 + (y1 - ya)^2)^2 - (x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*2*((x1 - xa)^2 + (y1 - ya)^2)*-2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2)^4

                    //diff(Pr(pi))/diff(xa) = -10*n/ln(10)*(-(xi - x1)*d1a^2 + ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*2*(x1 - xa))/d1a^4
                    //  + 5*n/ln(10)*(-2*(x1 - xa)*((xi - x1)^2 - (yi - y1)^2)*d1a^4 + ((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2)*4*d1a^2*(x1 - xa))/d1a^8
                    //  + 20*n/ln(10)*(-(y1 - ya)*(xi - x1)*(yi - y1)*d1a^4 + (x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*4*d1a^2*(x1 - xa))/d1a^8
                    final double derivativeXa = -10.0 * pathLossExponent / ln10 * (-diffXi1 * d1a2 + tmpX) / d1a4
                            + 5.0 * pathLossExponent / ln10 * (-2.0 * diffX1a * (diffXi12 - diffYi12) * d1a4 + tmpX2) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (-diffY1a * diffXi1 * diffYi1 * d1a4 + tmpX3) / d1a8;


                    //derivative of rssi respect to ya

                    //Pr(pi) = Pr(p1)
                    //  - 10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))
                    //  + 5*n*((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2)
                    //  + 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(xi - x1)*(yi - y1)

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/diff(ya) =
                    //  -(yi - y1)

                    //diff(((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2))/diff(ya) =
                    //  2*(y1 - ya)*((xi - x1)^2 - (yi - y1)^2)

                    //diff((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))/diff(ya) =
                    //  -(x1 - xa)*(xi - x1)*(yi - y1)

                    //diff(Pr(pi))/diff(ya) = -10*n/ln(10)*(-(yi - y1)*((x1 - xa)^2 + (y1 - ya)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*-2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2)^2
                    //  + 5*n/ln(10)*(2*(y1 - ya)*((xi - x1)^2 - (yi - y1)^2)*((x1 - xa)^2 + (y1 - ya)^2)^2 - ((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2)*2*((x1 - xa)^2 + (y1 - ya)^2)*-2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2)^4
                    //  + 20*n/ln(10)*(-(x1 - xa)*(xi - x1)*(yi - y1)*((x1 - xa)^2 + (y1 - ya)^2)^2 - (x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*2*((x1 - xa)^2 + (y1 - ya)^2)*-2(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2)^4

                    //diff(Pr(pi))/diff(ya) = -10*n/ln(10)*(-(yi - y1)*d1a^2 + ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*2*(y1 - ya))/d1a^4
                    //  + 5*n/ln(10)*(2*(y1 - ya)*((xi - x1)^2 - (yi - y1)^2)*d1a^4 + ((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2)*4*d1a^2*(y1 - ya))/d1a^8
                    //  + 20*n/ln(10)*(-(x1 - xa)*(xi - x1)*(yi - y1)*d1a^4 + (x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*4*d1a^2*(y1 - ya))/d1a^8
                    final double derivativeYa = -10.0 * pathLossExponent / ln10 * (-diffYi1 * d1a2 + tmpY) / d1a4
                            + 5.0 * pathLossExponent / ln10 * (2.0 * diffY1a * (diffXi12 - diffYi12) * d1a4 + tmpY2) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (-diffX1a * diffXi1 * diffYi1 * d1a4 + tmpY3) / d1a8;


                    //derivative of rssi respect to xi

                    //Pr(pi) = Pr(p1)
                    //  - 10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))
                    //  + 5*n*((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2)
                    //  + 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(xi - x1)*(yi - y1)

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/diff(xi) =
                    //  x1 - xa

                    //diff(((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2))/diff(xi) =
                    //  2*((x1 - xa)^2 - (y1 - ya)^2)*(xi - x1)

                    //diff((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))/diff(xi) =
                    //  (x1 - xa)*(y1 - ya)*(yi - y1)

                    //diff(Pr(pi))/diff(xi) = -10*n/ln(10)*((x1 - xa)*((x1 - xa)^2 + (y1 - ya)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*0)/((x1 - xa)^2 + (y1 - ya)^2)^2
                    //  + 5*n/ln(10)*(2*((x1 - xa)^2 - (y1 - ya)^2)*(xi - x1)*((x1 - xa)^2 + (y1 - ya)^2)^2 - ((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2)*0)/((x1 - xa)^2 + (y1 - ya)^2)^4
                    //  + 20*n/ln(10)*((x1 - xa)*(y1 - ya)*(yi - y1)*((x1 - xa)^2 + (y1 - ya)^2)^2 - (x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*0)/((x1 - xa)^2 + (y1 - ya)^2)^4

                    //diff(Pr(pi))/diff(xi) = -10*n/ln(10)*((x1 - xa)*d1a^2)/d1a^4
                    //  + 5*n/ln(10)*(2*((x1 - xa)^2 - (y1 - ya)^2)*(xi - x1)*d1a^4)/d1a^8
                    //  + 20*n/ln(10)*((x1 - xa)*(y1 - ya)*(yi - y1)*d1a^4)/d1a^8

                    //diff(Pr(pi))/diff(xi) = -10*n/ln(10)*(x1 - xa)/d1a^2
                    //  + 10*n/ln(10)*(((x1 - xa)^2 - (y1 - ya)^2)*(xi - x1))/d1a^4
                    //  + 20*n/ln(10)*(x1 - xa)*(y1 - ya)*(yi - y1)/d1a^4
                    final double derivativeXi = -10.0 * pathLossExponent / ln10 * diffX1a / d1a2
                            + 10.0 * pathLossExponent / ln10 * ((diffX1a2 - diffY1a2) * diffXi1) / d1a4
                            + 20.0 * pathLossExponent / ln10 * diffX1a * diffY1a * diffYi1 / d1a4;


                    //derivative of rssi respect to yi

                    //Pr(pi) = Pr(p1)
                    //  - 10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))
                    //  + 5*n*((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2)
                    //  + 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(xi - x1)*(yi - y1)

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/diff(yi) =
                    //  y1 - ya

                    //diff(((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2))/diff(yi) =
                    //  2*((x1 - xa)^2 - (y1 - ya)^2)*(yi - y1)

                    //diff((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))/diff(yi) =
                    //  (x1 - xa)*(y1 - ya)*(xi - x1)

                    //diff(Pr(pi))/diff(yi) = -10*n/ln(10)*((y1 - ya)*((x1 - xa)^2 + (y1 - ya)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*0)/((x1 - xa)^2 + (y1 - ya)^2)^2
                    //  + 5*n/ln(10)*(2*((x1 - xa)^2 - (y1 - ya)^2)*(yi - y1)*((x1 - xa)^2 + (y1 - ya)^2)^2 - ((x1 - xa)^2 - (y1 - ya)^2)*((xi - x1)^2 - (yi - y1)^2)*0)/((x1 - xa)^2 + (y1 - ya)^2)^4
                    //  + 20*n/ln(10)*((x1 - xa)*(y1 - ya)*(xi - x1)*((x1 - xa)^2 + (y1 - ya)^2)^2 - (x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*0)/((x1 - xa)^2 + (y1 - ya)^2)^4

                    //diff(Pr(pi))/diff(yi) = -10*n/ln(10)*((y1 - ya)*d1a^2)/d1a^4
                    //  + 5*n/ln(10)*(2*((x1 - xa)^2 - (y1 - ya)^2)*(yi - y1)*d1a^4)/d1a^8
                    //  + 20*n/ln(10)*((x1 - xa)*(y1 - ya)*(xi - x1)*d1a^4)/d1a^8

                    //diff(Pr(pi))/diff(yi) = -10*n/ln(10)*(y1 - ya)/d1a^2
                    //  + 10*n/ln(10)*(((x1 - xa)^2 - (y1 - ya)^2)*(yi - y1))/d1a^4
                    //  + 20*n/ln(10)*(x1 - xa)*(y1 - ya)*(xi - x1)/d1a^4
                    final double derivativeYi = -10.0 * pathLossExponent / ln10 * diffY1a / d1a2
                            + 10.0 * pathLossExponent / ln10 * ((diffX1a2 - diffY1a2) * diffYi1) / d1a4
                            + 20.0 * pathLossExponent / ln10 * diffX1a * diffY1a * diffXi1 / d1a4;


                    //set derivatives fingerprintRssi, pathLossExponent, x1, y1, xa, ya, xi, yi
                    jacobian.setElementAtIndex(0, derivativeFingerprintRssi);
                    jacobian.setElementAtIndex(1, derivativePathLossExponent);
                    jacobian.setElementAtIndex(2, derivativeX1);
                    jacobian.setElementAtIndex(3, derivativeY1);
                    jacobian.setElementAtIndex(4, derivativeXa);
                    jacobian.setElementAtIndex(5, derivativeYa);
                    jacobian.setElementAtIndex(6, derivativeXi);
                    jacobian.setElementAtIndex(7, derivativeYi);
                }

                @Override
                public int getNumberOfVariables() {
                    return 1;
                }
            }, mean, covariance);
        } catch (final AlgebraException | StatisticsException e) {
            throw new IndoorException(e);
        }
    }

    /**
     * Propagates provided variances (fingerprint rssi variance, path-loss exponent variance,
     * fingerprint position covariance and radio source position covariance) into
     * rssi variance by considering the 3D 1st order Taylor expression of received power.
     * Notice that any unknown variance is assumed to be zero.
     *
     * @param fingerprintRssi               closest located fingerprint reading RSSI expressed in dBm's.
     * @param pathLossExponent              path-loss exponent.
     * @param fingerprintPosition           position of closest fingerprint.
     * @param radioSourcePosition           radio source position associated to fingerprint reading.
     * @param estimatedPosition             position to be estimated. Usually this is equal to the
     *                                      initial position used by a non linear algorithm.
     * @param fingerprintRssiVariance       variance of fingerprint RSSI or null if unknown.
     * @param pathLossExponentVariance      variance of path-loss exponent or null if unknown.
     * @param fingerprintPositionCovariance covariance of fingerprint position or null if
     *                                      unknown.
     * @param radioSourcePositionCovariance covariance of radio source position or null
     *                                      if unknown.
     * @param estimatedPositionCovariance   covariance of position to be estimated or null
     *                                      if unknown. (This is usually unknown).
     * @return a normal distribution containing expected received RSSI value and its variance.
     * @throws IndoorException if something fails.
     */
    public static MultivariateNormalDist propagateVariancesToRssiVarianceSecondOrderNonLinear3D(
            final double fingerprintRssi, final double pathLossExponent,
            final Point3D fingerprintPosition, final Point3D radioSourcePosition,
            final Point3D estimatedPosition,
            final Double fingerprintRssiVariance,
            final Double pathLossExponentVariance,
            final Matrix fingerprintPositionCovariance,
            final Matrix radioSourcePositionCovariance,
            final Matrix estimatedPositionCovariance) throws IndoorException {

        if (fingerprintPosition == null || radioSourcePosition == null ||
                estimatedPosition == null) {
            return null;
        }

        //2nd order Taylor expression of received power in 3D:
        //Pr(pi) = Pr(p1)
        // - 10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
        // - 10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
        // - 10*n*(z1 - za)/(ln(10)*d1a^2)*(zi - z1)
        // - 5*n*((y1 - ya)^2 + (z1 - za)^2) - (x1 - xa)^2)/(ln(10)*d1a^4)*(xi - x1)^2
        // - 5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*d1a^4)*(yi - y1)^2
        // - 5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*d1a^4)*(zi - z1)^2
        // + 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4)*(xi - x1)*(yi - y1)
        // + 20*n*(y1 - ya)*(z1 - za)/(ln(10)*d1a^4)*(yi - y1)*(zi - z1)
        // + 20*n*(x1 - xa)*(z1 - za)/(ln(10)*d1a^4)*(xi - x1)*(zi - z1)
        //where d1a^2 = (x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2

        final double x1 = fingerprintPosition.getInhomX();
        final double y1 = fingerprintPosition.getInhomY();
        final double z1 = fingerprintPosition.getInhomZ();

        final double xa = radioSourcePosition.getInhomX();
        final double ya = radioSourcePosition.getInhomY();
        final double za = radioSourcePosition.getInhomZ();

        final double xi = estimatedPosition.getInhomX();
        final double yi = estimatedPosition.getInhomY();
        final double zi = estimatedPosition.getInhomZ();

        final double[] mean = new double[]{
                fingerprintRssi, pathLossExponent, x1, y1, z1, xa, ya, za, xi, yi, zi
        };
        final Matrix covariance = Matrix.diagonal(new double[]{
                fingerprintRssiVariance != null ? fingerprintRssiVariance : 0.0,
                pathLossExponentVariance != null ? pathLossExponentVariance : 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        });

        if (fingerprintPositionCovariance != null &&
                fingerprintPositionCovariance.getRows() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                fingerprintPositionCovariance.getColumns() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {

            covariance.setSubmatrix(2, 2,
                    4, 4,
                    fingerprintPositionCovariance);
        }

        if (radioSourcePositionCovariance != null &&
                radioSourcePositionCovariance.getRows() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                radioSourcePositionCovariance.getColumns() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            covariance.setSubmatrix(5, 5,
                    7, 7,
                    radioSourcePositionCovariance);
        }

        if (estimatedPositionCovariance != null &&
                estimatedPositionCovariance.getRows() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                estimatedPositionCovariance.getColumns() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            covariance.setSubmatrix(8, 8,
                    10, 10,
                    estimatedPositionCovariance);
        }

        try {
            return MultivariateNormalDist.propagate(new MultivariateNormalDist.JacobianEvaluator() {
                @Override
                public void evaluate(
                        final double[] x, final double[] y, final Matrix jacobian) {

                    //Pr(pi) = Pr(p1)
                    // - 10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
                    // - 10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
                    // - 10*n*(z1 - za)/(ln(10)*d1a^2)*(zi - z1)
                    // - 5*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*d1a^4)*(xi - x1)^2
                    // - 5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*d1a^4)*(yi - y1)^2
                    // - 5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*d1a^4)*(zi - z1)^2
                    // + 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4)*(xi - x1)*(yi - y1)
                    // + 20*n*(y1 - ya)*(z1 - za)/(ln(10)*d1a^4)*(yi - y1)*(zi - z1)
                    // + 20*n*(x1 - xa)*(z1 - za)/(ln(10)*d1a^4)*(xi - x1)*(zi - z1)
                    //where d1a^2 = (x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2

                    //Hence:
                    //Pr(pi) = Pr(p1)
                    //  -10*n*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*(xi - x1)
                    //  -10*n*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*(yi - y1)
                    //  -10*n*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*(zi - z1)
                    //  -5*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)^2
                    //  -5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)^2
                    //  -5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(zi - z1)^2
                    //  +20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(yi - y1)
                    //  +20*n*(y1 - ya)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)*(zi - z1)
                    //  +20*n*(x1 - xa)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(zi - z1)

                    final double diffX1a = x1 - xa;
                    final double diffY1a = y1 - ya;
                    final double diffZ1a = z1 - za;

                    final double diffXi1 = xi - x1;
                    final double diffYi1 = yi - y1;
                    final double diffZi1 = zi - z1;

                    final double diffX1a2 = diffX1a * diffX1a;
                    final double diffY1a2 = diffY1a * diffY1a;
                    final double diffZ1a2 = diffZ1a * diffZ1a;

                    final double diffXi12 = diffXi1 * diffXi1;
                    final double diffYi12 = diffYi1 * diffYi1;
                    final double diffZi12 = diffZi1 * diffZi1;

                    final double d1a2 = diffX1a2 + diffY1a2 + diffZ1a2;
                    final double d1a4 = d1a2 * d1a2;
                    final double d1a8 = d1a4 * d1a4;

                    final double ln10 = Math.log(10.0);
                    final double crossDiff = diffX1a * diffXi1 + diffY1a * diffYi1 + diffZ1a * diffZi1;

                    //Pr(pi) = Pr(p1)
                    //  -10*n*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*(xi - x1)
                    //  -10*n*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*(yi - y1)
                    //  -10*n*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*(zi - z1)
                    //  -5*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)^2
                    //  -5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)^2
                    //  -5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(zi - z1)^2
                    //  +20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(yi - y1)
                    //  +20*n*(y1 - ya)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)*(zi - z1)
                    //  +20*n*(x1 - xa)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(zi - z1)

                    y[0] = fingerprintRssi
                            - 10.0 * pathLossExponent * diffX1a / (ln10 * d1a2) * diffXi1
                            - 10.0 * pathLossExponent * diffY1a / (ln10 * d1a2) * diffYi1
                            - 10.0 * pathLossExponent * diffZ1a / (ln10 * d1a2) * diffZi1
                            - 5.0 * pathLossExponent * (-diffX1a2 + diffY1a2 + diffZ1a2) / (ln10 * d1a4) * diffXi12
                            - 5.0 * pathLossExponent * (diffX1a2 - diffY1a2 + diffZ1a2) / (ln10 * d1a4) * diffYi12
                            - 5.0 * pathLossExponent * (diffX1a2 + diffY1a2 - diffZ1a2) / (ln10 * d1a4) * diffZi12
                            + 20.0 * pathLossExponent * diffX1a * diffY1a / (ln10 * d1a4) * diffXi1 * diffYi1
                            + 20.0 * pathLossExponent * diffY1a * diffZ1a / (ln10 * d1a4) * diffYi1 * diffZi1
                            + 20.0 * pathLossExponent * diffX1a * diffZ1a / (ln10 * d1a4) * diffXi1 * diffZi1;

                    //compute gradient (is a jacobian having 1 row and 11 columns)


                    //derivative of rssi respect to fingerprint rssi
                    final double derivativeFingerprintRssi = 1.0;

                    //derivative of rssi respect to path-loss exponent

                    //diff(Pr(pi))/diff(n) = -10*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
                    //  -10*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
                    final double derivativePathLossExponent = -10.0 * diffX1a / (ln10 * d1a2) * diffXi1
                            - 10.0 * diffY1a / (ln10 * d1a2) * diffYi1
                            - 10.0 * diffZ1a / (ln10 * d1a2) * diffZi1
                            - 5.0 * (-diffX1a2 + diffY1a2 + diffZ1a2) / (ln10 * d1a4) * diffXi12
                            - 5.0 * (diffX1a2 - diffY1a2 + diffZ1a2) / (ln10 * d1a4) * diffYi12
                            - 5.0 * (diffX1a2 + diffY1a2 - diffZ1a2) / (ln10 * d1a4) * diffZi12
                            + 20.0 * diffX1a * diffY1a / (ln10 * d1a4) * diffXi1 * diffYi1
                            + 20.0 * diffY1a * diffZ1a / (ln10 * d1a4) * diffYi1 * diffZi1
                            + 20.0 * diffX1a * diffZ1a / (ln10 * d1a4) * diffXi1 * diffZi1;


                    //derivative of rssi respect to x1

                    //We have
                    //Pr(pi) = Pr(p1)
                    //  -10*n*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*(xi - x1)
                    //  -10*n*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*(yi - y1)
                    //  -10*n*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*(zi - z1)
                    //  -5*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)^2
                    //  -5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)^2
                    //  -5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(zi - z1)^2
                    //  +20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(yi - y1)
                    //  +20*n*(y1 - ya)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)*(zi - z1)
                    //  +20*n*(x1 - xa)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(zi - z1)

                    //Pr(pi) = Pr(p1)
                    //  -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))
                    //  -5*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)^2
                    //  -5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)^2
                    //  -5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(zi - z1)^2
                    //  +20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(yi - y1)
                    //  +20*n*(y1 - ya)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)*(zi - z1)
                    //  +20*n*(x1 - xa)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(zi - z1)


                    //and we know that: (f(x)/g(x))' = (f'(x)*g(x) - f(x)*g'(x))/g(x)^2
                    //and also that (f(x)*g(x))' = f'(x)*g(x) + f(x)*g'(x)

                    //Hence
                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(x1) =
                    //  diff(x1*xi -xa*xi -x1^2 + xa*x1)/diff(x1) =
                    //  diff(-x1^2 + (xi + xa)*x1 - xa*xi)/diff(x1) =
                    //  -2*x1 + xi + xa

                    //diff(((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)/diff(x1) =
                    //  diff(-(x1 - xa)^2*(xi - x1)^2)/diff(x1) =
                    //  -2*(x1 - xa)*(xi - x1)^2 - 2*(xi - x1)*(-(x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)
                    //  -2*((x1 - xa)*(xi - x1)^2 + (xi - x1)*(-(x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))

                    //diff(((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)/diff(x1) =
                    //  diff((x1 - xa)^2*(yi - y1)^2)/diff(x1) =
                    //  2*(x1 - xa)*(yi - y1)^2

                    //diff(((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(z1 - za)^2)/diff(x1) =
                    //  diff((x1 - xa)^2*(z1 - za)^2)/diff(x1) =
                    //  2*(x1 - xa)*(z1 - za)^2

                    //diff((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))/diff(x1) =
                    //  (y1 - ya)*(yi - y1)*((xi - x1) - (x1 - xa)) =
                    //  (y1 - ya)*(yi - y1)*(-2*x1 + xi + xa)

                    //diff((y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1))/diff(x1) = 0

                    //diff((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))/diff(x1) =
                    //  (z1 - za)*(zi - z1)*((xi - x1) - (x1 - xa)) =
                    //  (z1 - za)*(zi - z1)*(-2*x1 + xi + xa)

                    //diff(Pr(pi))/diff(x1) = -10*n/ln(10)*((-2*x1 + xi + xa)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //  -5*n/ln(10)*(-2*((x1 - xa)*(xi - x1)^2 + (xi - x1)*(-(x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(2*(x1 - xa)*(yi - y1)^2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(2*(x1 - xa)*(z1 - za)^2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(z1 - za)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*((y1 - ya)*(yi - y1)*(-2*x1 + xi + xa)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*(0*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*((z1 - za)*(zi - z1)*(-2*x1 + xi + xa)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4

                    //diff(Pr(pi))/diff(x1) = -10*n/ln(10)*((-2*x1 + xi + xa)*d1a^2 - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(x1 - xa))/d1a^4
                    //  -5*n/ln(10)*(-2*((x1 - xa)*(xi - x1)^2 + (xi - x1)*(-(x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*d1a^4 - (((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)*4*d1a^2*(x1 - xa))/d1a^8
                    //  -5*n/ln(10)*(2*(x1 - xa)*(yi - y1)^2*d1a^4 - (((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)*4*d1a^2*(x1 - xa))/d1a^8
                    //  -5*n/ln(10)*(2*(x1 - xa)*(z1 - za)^2*d1a^4 - (((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(z1 - za)^2)*4*d1a^2*(x1 - xa))/d1a^8
                    //  +20*n/ln(10)*((y1 - ya)*(yi - y1)*(-2*x1 + xi + xa)*d1a^4 - (x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*4*d1a^2*(x1 - xa))/d1a^8
                    //  +20*n/ln(10)*(-(y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1)*4*d1a^2*(x1 - xa))/d1a^8
                    //  +20*n/ln(10)*((z1 - za)*(zi - z1)*(-2*x1 + xi + xa)*d1a^4 - (x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1)*4*d1a^2*(x1 - xa))/d1a^8
                    final double tmp1 = (diffY1a2 + diffZ1a2 - diffX1a2) * diffXi12 * 4.0 * d1a2 * diffX1a;
                    final double tmp2 = diffX1a * diffY1a * diffXi1 * diffYi1 * 4.0 * d1a2 * diffX1a;
                    final double tmp3 = diffX1a * diffZ1a * diffXi1 * diffZi1 * 4.0 * d1a2 * diffX1a;
                    final double derivativeX1 = -10.0 * pathLossExponent / ln10 * ((-2 * x1 + xi + xa) * d1a2 - crossDiff * 2.0 * diffX1a) / d1a4
                            - 5.0 * pathLossExponent / ln10 * (-2.0 * (diffX1a * diffXi12 + diffXi1 * (-diffX1a2 + diffY1a2 + diffZ1a2)) * d1a4 - tmp1) / d1a8
                            - 5.0 * pathLossExponent / ln10 * (2.0 * diffX1a * diffYi12 * d1a4 - tmp1) / d1a8
                            - 5.0 * pathLossExponent / ln10 * (2.0 * diffX1a * diffZ1a2 * d1a4 - (diffX1a2 + diffY1a2 - diffZ1a2) * diffZ1a2 * 4.0 * d1a2 * diffX1a) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (diffY1a * diffYi1 * (-2.0 * x1 + xi + xa) * d1a4 - tmp2) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (-diffY1a * diffZ1a * diffYi1 * diffZi1 * 4.0 * d1a2 * diffX1a) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (diffZ1a * diffZi1 * (-2.0 * x1 + xi + xa) * d1a4 - tmp3) / d1a8;


                    //derivative of rssi respect to y1

                    //We have
                    //Pr(pi) = Pr(p1)
                    //  -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))
                    //  -5*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)^2
                    //  -5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)^2
                    //  -5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(zi - z1)^2
                    //  +20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(yi - y1)
                    //  +20*n*(y1 - ya)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)*(zi - z1)
                    //  +20*n*(x1 - xa)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(zi - z1)

                    //Hence
                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(y1) =
                    //  diff(y1*yi - ya*yi -y1^2 + ya*y1)/diff(y1) =
                    //  diff(-y1^2 + (yi + ya)*y1 - ya*yi)/diff(y1) =
                    //  -2*y1 + yi + ya

                    //diff(((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)/diff(y1) =
                    //  2*(y1 - ya)*(xi - x1)^2

                    //diff(((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)/diff(y1) =
                    //  -2*(y1 - ya)*(yi - y1)^2 -2*(yi - y1)*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)
                    //  -2*((y1 - ya)*(yi - y1)^2 + (yi - y1)*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2))

                    //diff(((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)/diff(y1) =
                    //  2*(y1 - ya)*(zi - z1)^2

                    //diff((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))/diff(y1) =
                    //  (x1 - xa)*(xi - x1)*(yi - y1) - (xi - x1)*(x1 - xa)*(y1 - ya) =
                    //  (x1 - xa)*((xi - x1)*(yi - y1) - (xi - x1)*(y1 - ya)) =
                    //  (x1 - xa)*(xi - x1)*(-2*y1 + yi + ya)

                    //diff((y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1))/diff(y1) =
                    //  (z1 - za)*(yi - y1)*(zi - z1) - (zi - z1)*(y1 - ya)*(z1 - za) =
                    //  (z1 - za)*((yi - y1)*(zi - z1) - (zi - z1)*(y1 - ya)) =
                    //  (z1 - za)*(zi - z1)*(-2*y1 + yi + ya)

                    //diff((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))/diff(y1) = 0

                    //diff(Pr(pi))/diff(y1) = -10*n/ln(10)*((-2*y1 + yi + ya)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //  -5*n/ln(10)*(2*(y1 - ya)*(xi - x1)^2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(-2*((y1 - ya)*(yi - y1)^2 + (yi - y1)*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2))*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2*2((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2(y1 -ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(2*(y1 - ya)*(zi - z1)^2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*((x1 - xa)*(xi - x1)*(-2*y1 + yi + ya)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*((z1 - za)*(zi - z1)*(-2*y1 + yi + ya)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*(0*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4

                    //diff(Pr(pi))/diff(y1) = -10*n/ln(10)*((-2*y1 + yi + ya)*d1a^2 - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(y1 - ya))/d1a^4
                    //  -5*n/ln(10)*(2*(y1 - ya)*(xi - x1)^2*d1a^4 - (((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)*4*d1a^2*(y1 - ya))/d1a^8
                    //  -5*n/ln(10)*(-2*((y1 - ya)*(yi - y1)^2 + (yi - y1)*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2))*d1a^4 - (((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)*4*d1a^2*(y1 -ya))/d1a^8
                    //  -5*n/ln(10)*(2*(y1 - ya)*(zi - z1)^2*d1a^4 - (((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)*4*d1a^2*(y1 - ya))/d1a^8
                    //  +20*n/ln(10)*((x1 - xa)*(xi - x1)*(-2*y1 + yi + ya)*d1a^4 - (x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*4*d1a^2*(y1 - ya))/d1a^8
                    //  +20*n/ln(10)*((z1 - za)*(zi - z1)*(-2*y1 + yi + ya)*d1a^4 - (y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1)*4*d1a^2*(y1 - ya))/d1a^8
                    //  +20*n/ln(10)*(-(x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1)*4*d1a^2*(y1 - ya))/d1a^8
                    final double tmp4 = diffY1a * diffZ1a * diffYi1 * diffZi1 * 4.0 * d1a2 * diffY1a;
                    final double derivativeY1 = -10.0 * pathLossExponent / ln10 * ((-2.0 * y1 + yi + ya) * d1a2 - crossDiff * 2.0 * diffY1a) / d1a4
                            - 5.0 * pathLossExponent / ln10 * (2.0 * diffY1a * diffXi12 * d1a4 - (-diffX1a2 + diffY1a2 + diffZ1a2) * diffXi12 * 4.0 * d1a2 * diffY1a) / d1a8
                            - 5.0 * pathLossExponent / ln10 * (-2.0 * (diffY1a * diffYi12 + diffYi1 * (diffX1a2 - diffY1a2 + diffZ1a2)) * d1a4 - (diffX1a2 - diffY1a2 + diffZ1a2) * diffYi12 * 4.0 * d1a2 * diffY1a) / d1a8
                            - 5.0 * pathLossExponent / ln10 * (2.0 * diffY1a * diffZi12 * d1a4 - (diffX1a2 + diffY1a2 - diffZ1a2) * diffZi12 * 4.0 * d1a2 * diffY1a) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (diffX1a * diffXi1 * (-2.0 * y1 + yi + ya) * d1a4 - diffX1a * diffY1a * diffXi1 * diffYi1 * 4.0 * d1a2 * diffY1a) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (diffZ1a * diffZi1 * (-2.0 * y1 + yi + ya) * d1a4 - tmp4) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (-diffX1a * diffZ1a * diffXi1 * diffZi1 * 4.0 * d1a2 * diffY1a) / d1a8;


                    //derivative of rssi respect to z1

                    //We have
                    //Pr(pi) = Pr(p1)
                    //  -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))
                    //  -5*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)^2
                    //  -5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)^2
                    //  -5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(zi - z1)^2
                    //  +20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(yi - y1)
                    //  +20*n*(y1 - ya)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)*(zi - z1)
                    //  +20*n*(x1 - xa)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(zi - z1)

                    //Hence
                    //diff(((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1)))/diff(z1) =
                    //  diff(z1*zi -za*zi -z1^2 + za*z1)/diff(z1) =
                    //  diff(-z1^2 + (zi + za)*z1 - za*zi)/diff(z1) =
                    //  -2*z1 + zi + za

                    //diff(((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)/diff(z1) =
                    //  2*(z1 - za)*(xi - x1)^2

                    //diff(((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)/diff(z1) =
                    //  2*(z1 - za)*(yi - y1)^2

                    //diff(((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)/diff(z1) =
                    //  2*(z1 - za)*(zi - z1)^2 + 2*(zi - z1)*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)
                    //  2*((z1 - za)*(zi - z1)^2 + (zi - z1)*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2))

                    //diff((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))/diff(z1) = 0

                    //diff((y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1))/diff(z1) =
                    //  (y1 - ya)*(yi - y1)*(zi - z1) - (yi - y1)*(y1 - ya)*(z1 - za) =
                    //  (y1 - ya)*((yi - y1)*(zi - z1) - (yi - y1)*(z1 - za)) =
                    //  (y1 - ya)*(yi - y1)*(-2*z1 + zi + za)

                    //diff((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))/diff(z1) =
                    //  (x1 - xa)*(xi - x1)*(zi - z1) - (xi - x1)*(x1 - xa)*(z1 - za) =
                    //  (x1 - xa)*((xi - x1)*(zi - z1) - (xi - x1)*(z1 - za)) =
                    //  (x1 - xa)*(xi - x1)*(-2*z1 + zi + za)

                    //diff(Pr(pi))/diff(z1) = -10*n/ln(10)*((-2*z1 + zi + za)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //  -5*n/ln(10)*(2*(z1 - za)*(xi - x1)^2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(2*(z1 - za)*(yi - y1)^2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(2*((z1 - za)*(zi - z1)^2 + (zi - z1)*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2))*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*(0*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2 - ((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*((y1 - ya)*(yi - y1)*(-2*z1 + zi + za)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1))*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*((x1 - xa)*(xi - x1)*(-2*z1 + zi + za)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4

                    //diff(Pr(pi))/diff(z1) = -10*n/ln(10)*((-2*z1 + zi + za)*d1a^2 - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(z1 - za))/d1a^4
                    //  -5*n/ln(10)*(2*(z1 - za)*(xi - x1)^2*d1a^4 - (((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)*4*d1a^2*(z1 - za))/d1a^8
                    //  -5*n/ln(10)*(2*(z1 - za)*(yi - y1)^2*d1a^4 - (((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)*4*d1a^2*(z1 - za))/d1a^8
                    //  -5*n/ln(10)*(2*((z1 - za)*(zi - z1)^2 + (zi - z1)*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2))*d1a^4 - (((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)*4*d1a^2*(z1 - za))/d1a^8
                    //  +20*n/ln(10)*(-(x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*4*d1a^2*(z1 - za))/d1a^8
                    //  +20*n/ln(10)*((y1 - ya)*(yi - y1)*(-2*z1 + zi + za)*d1a^4 - ((y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1))*4*d1a^2*(z1 - za))/d1a^8
                    //  +20*n/ln(10)*((x1 - xa)*(xi - x1)*(-2*z1 + zi + za)*d1a^4 - ((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))*4*d1a^2*(z1 - za))/d1a^8
                    final double derivativeZ1 = -10.0 * pathLossExponent / ln10 * ((-2.0 * z1 + zi + za) * d1a2 - crossDiff * 2.0 * diffZ1a) / d1a4
                            - 5.0 * pathLossExponent / ln10 * (2.0 * diffZ1a * diffXi12 * d1a4 - ((-diffX1a2 + diffY1a2 + diffZ1a2) * diffXi12) * 4.0 * d1a2 * diffZ1a) / d1a8
                            - 5.0 * pathLossExponent / ln10 * (2.0 * diffZ1a * diffYi12 * d1a4 - ((diffX1a2 - diffY1a2 + diffZ1a2) * diffYi12) * 4.0 * d1a2 * diffZ1a) / d1a8
                            - 5.0 * pathLossExponent / ln10 * (2.0 * (diffZ1a * diffZi12 + diffZi1 * (diffX1a2 + diffY1a2 - diffZ1a2)) * d1a4 - ((diffX1a2 + diffY1a2 - diffZ1a2) * diffZi12) * 4.0 * d1a2 * diffZ1a) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (-diffX1a * diffY1a * diffXi1 * diffYi1 * 4.0 * d1a2 * diffZ1a) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (diffY1a * diffYi1 * (-2.0 * z1 + zi + za) * d1a4 - diffY1a * diffZ1a * diffYi1 * diffZi1 * 4.0 * d1a2 * diffZ1a) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (diffX1a * diffXi1 * (-2.0 * z1 + zi + za) * d1a4 - diffX1a * diffZ1a * diffXi1 * diffZi1 * 4.0 * d1a2 * diffZ1a) / d1a8;


                    //derivative of rssi respect to xa

                    //We have
                    //Pr(pi) = Pr(p1)
                    //  -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))
                    //  -5*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)^2
                    //  -5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)^2
                    //  -5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(zi - z1)^2
                    //  +20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(yi - y1)
                    //  +20*n*(y1 - ya)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)*(zi - z1)
                    //  +20*n*(x1 - xa)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(zi - z1)

                    //Hence
                    //diff(((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1)))/diff(xa) =
                    //  -(xi - x1)

                    //diff(((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)/diff(xa) =
                    //  2*(x1 - xa)*(xi - x1)^2

                    //diff(((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)/diff(xa) =
                    //  -2*(x1 - xa)*(yi - y1)^2

                    //diff(((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)/diff(xa) =
                    //  -2*(x1 - xa)*(zi - z1)^2

                    //diff((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))/diff(xa) =
                    //  -(y1 - ya)*(xi - x1)*(yi - y1)

                    //diff((y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1))/diff(xa) = 0

                    //diff((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))/diff(xa) =
                    //  -(z1 - za)*(xi - x1)*(zi - z1)

                    //diff(Pr(pi))/diff(xa) = -10*n/ln(10)*(-(xi - x1)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - (((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1)))*-2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //  -5*n/ln(10)*(2*(x1 - xa)*(xi - x1)^2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(-2*(x1 - xa)*(yi - y1)^2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(-2*(x1 - xa)*(zi - z1)^2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*(-(y1 - ya)*(xi - x1)*(yi - y1)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*(0*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*(-(z1 - za)*(xi - x1)*(zi - z1)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4

                    //diff(Pr(pi))/diff(xa) = -10*n/ln(10)*(-(xi - x1)*d1a^2 + (((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1)))*2*(x1 - xa))/d1a^4
                    //  -5*n/ln(10)*(2*(x1 - xa)*(xi - x1)^2*d1a^4 + (((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)*4*d1a^2*(x1 - xa))/d1a^8
                    //  -5*n/ln(10)*(-2*(x1 - xa)*(yi - y1)^2*d1a^4 + (((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)*4*d1a^2*(x1 - xa))/d1a^8
                    //  -5*n/ln(10)*(-2*(x1 - xa)*(zi - z1)^2*d1a^4 + (((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)*4*d1a^2*(x1 - xa))/d1a^8
                    //  +20*n/ln(10)*(-(y1 - ya)*(xi - x1)*(yi - y1)*d1a^4 + (x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*4*d1a^2*(x1 - xa))/d1a^8
                    //  +20*n/ln(10)*((y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1)*4*d1a^2*(x1 - xa))/d1a^8
                    //  +20*n/ln(10)*(-(z1 - za)*(xi - x1)*(zi - z1)*d1a^4 + (x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1)*4*d1a^2*(x1 - xa))/d1a^8
                    final double derivativeXa = -10.0 * pathLossExponent / ln10 * (-diffXi1 * d1a2 + crossDiff * 2.0 * diffX1a) / d1a4
                            - 5.0 * pathLossExponent / ln10 * (2.0 * diffX1a * diffXi12 * d1a4 + ((diffY1a2 + diffZ1a2 - diffX1a2) * diffXi12) * 4.0 * d1a2 * diffX1a) / d1a8
                            - 5.0 * pathLossExponent / ln10 * (-2.0 * diffX1a * diffYi12 * d1a4 + ((diffX1a2 - diffY1a2 + diffZ1a2) * diffYi12) * 4.0 * d1a2 * diffX1a) / d1a8
                            - 5.0 * pathLossExponent / ln10 * (-2.0 * diffX1a * diffZi12 * d1a4 + ((diffX1a2 + diffY1a2 - diffZ1a2) * diffZi12) * 4.0 * d1a2 * diffX1a) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (-diffY1a * diffXi1 * diffYi1 * d1a4 + tmp2) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (diffY1a * diffZ1a * diffYi1 * diffZi1 * 4.0 * d1a2 * diffX1a) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (-diffZ1a * diffXi1 * diffZi1 * d1a4 + tmp3) / d1a8;


                    //derivative of rssi respect to ya

                    //We have
                    //Pr(pi) = Pr(p1)
                    //  -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))
                    //  -5*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)^2
                    //  -5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)^2
                    //  -5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(zi - z1)^2
                    //  +20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(yi - y1)
                    //  +20*n*(y1 - ya)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)*(zi - z1)
                    //  +20*n*(x1 - xa)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(zi - z1)

                    //Hence
                    //diff(((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1)))/diff(ya) =
                    //  -(yi - y1)

                    //diff(((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)/diff(ya) =
                    //  -2*(y1 - ya)*(xi - x1)^2

                    //diff(((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)/diff(ya) =
                    //  2*(y1 - ya)*(yi - y1)^2

                    //diff(((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)/diff(ya) =
                    //  -2*(y1 - ya)*(zi - z1)^2

                    //diff((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))/diff(ya) =
                    //  -(x1 - xa)*(xi - x1)*(yi - y1)

                    //diff((y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1))/diff(ya) =
                    //  -(z1 - za)*(yi - y1)*(zi - z1)

                    //diff((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))/diff(ya) = 0

                    //diff(Pr(pi))/diff(ya) = -10*n/ln(10)*(-(yi - y1)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - (((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1)))*-2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //  -5*n/ln(10)*(-2*(y1 - ya)*(xi - x1)^2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(2*(y1 - ya)*(yi - y1)^2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(-2*(y1 - ya)*(zi - z1)^2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*(-(x1 - xa)*(xi - x1)*(yi - y1)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*(-(z1 - za)*(yi - y1)*(zi - z1)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*(0*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4

                    //diff(Pr(pi))/diff(ya) = -10*n/ln(10)*(-(yi - y1)*d1a^2 + (((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1)))*2*(y1 - ya))/d1a^4
                    //  -5*n/ln(10)*(-2*(y1 - ya)*(xi - x1)^2*d1a^4 + (((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)*4*d1a^2*(y1 - ya))/d1a^8
                    //  -5*n/ln(10)*(2*(y1 - ya)*(yi - y1)^2*d1a^4 + (((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)*4*d1a^2*(y1 - ya))/d1a^8
                    //  -5*n/ln(10)*(-2*(y1 - ya)*(zi - z1)^2*d1a^4 + (((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)*4*d1a^2*(y1 - ya))/d1a^8
                    //  +20*n/ln(10)*(-(x1 - xa)*(xi - x1)*(yi - y1)*d1a^4 + ((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))*4*d1a^2*(y1 - ya))/d1a^8
                    //  +20*n/ln(10)*(-(z1 - za)*(yi - y1)*(zi - z1)*d1a^4 + (y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1)*4*d1a^2*(y1 - ya))/d1a^8
                    //  +20*n/ln(10)*(((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))*4*d1a^2*(y1 - ya))/d1a^8
                    final double derivativeYa = -10.0 * pathLossExponent / ln10 * (-diffYi1 * d1a2 + crossDiff * 2.0 * diffY1a) / d1a4
                            - 5.0 * pathLossExponent / ln10 * (-2.0 * diffY1a * diffXi12 * d1a4 + ((diffY1a2 + diffZ1a2 - diffX1a2) * diffXi12) * 4.0 * d1a2 * diffY1a) / d1a8
                            - 5.0 * pathLossExponent / ln10 * (2.0 * diffY1a * diffYi12 * d1a4 + ((diffX1a2 - diffY1a2 + diffZ1a2) * diffYi12) * 4.0 * d1a2 * diffY1a) / d1a8
                            - 5.0 * pathLossExponent / ln10 * (-2.0 * diffY1a * diffZi12 * d1a4 + ((diffX1a2 + diffY1a2 - diffZ1a2) * diffZi12) * 4.0 * d1a2 * diffY1a) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (-diffX1a * diffXi1 * diffYi1 * d1a4 + (diffX1a * diffY1a * diffXi1 * diffYi1) * 4.0 * d1a2 * diffY1a) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (-diffZ1a * diffYi1 * diffZi1 * d1a4 + tmp4) / d1a8
                            + 20.0 * pathLossExponent / ln10 * ((diffX1a * diffZ1a * diffXi1 * diffZi1) * 4.0 * d1a2 * diffY1a) / d1a8;


                    //derivative of rssi respect to za

                    //We have
                    //Pr(pi) = Pr(p1)
                    //  -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))
                    //  -5*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)^2
                    //  -5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)^2
                    //  -5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(zi - z1)^2
                    //  +20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(yi - y1)
                    //  +20*n*(y1 - ya)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)*(zi - z1)
                    //  +20*n*(x1 - xa)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(zi - z1)

                    //Hence
                    //diff(((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1)))/diff(za) =
                    //  -(zi - z1)

                    //diff(((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)/diff(za) =
                    //  -2*(z1 - za)*(xi - x1)^2

                    //diff(((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)/diff(za) =
                    //  -2*(z1 - za)*(yi - y1)^2

                    //diff(((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)/diff(za) =
                    //  2*(z1 - za)*(zi - z1)^2

                    //diff((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))/diff(za) = 0

                    //diff((y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1))/diff(za) =
                    //  -(y1 - ya)*(yi - y1)*(zi - z1)

                    //diff((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))/diff(za) =
                    //  -(x1 - xa)*(xi - x1)*(zi - z1)

                    //diff(Pr(pi))/diff(za) = -10*n/ln(10)*(-(zi - z1)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - (((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1)))*-2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //  -5*n/ln(10)*(-2*(z1 - za)*(xi - x1)^2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(-2*(z1 - za)*(yi - y1)^2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(2*(z1 - za)*(zi - z1)^2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*(0*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*(-(y1 - ya)*(yi - y1)*(zi - z1)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1))*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*(-(x1 - xa)*(xi - x1)*(zi - z1)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*-2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4

                    //diff(Pr(pi))/diff(za) = -10*n/ln(10)*(-(zi - z1)*d1a^2 + (((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1)))*2*(z1 - za))/d1a^4
                    //  -5*n/ln(10)*(-2*(z1 - za)*(xi - x1)^2*d1a^4 - (((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)*4*d1a^2*(z1 - za))/d1a^8
                    //  -5*n/ln(10)*(-2*(z1 - za)*(yi - y1)^2*d1a^4 + (((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)*4*d1a^2*(z1 - za))/d1a^8
                    //  -5*n/ln(10)*(2*(z1 - za)*(zi - z1)^2*d1a^4 + (((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)*4*d1a^2*(z1 - za))/d1a^8
                    //  +20*n/ln(10)*(((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))*4*d1a^2*(z1 - za))/d1a^8
                    //  +20*n/ln(10)*(-(y1 - ya)*(yi - y1)*(zi - z1)*d1a^4 + ((y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1))*4*d1a^2*(z1 - za))/d1a^8
                    //  +20*n/ln(10)*(-(x1 - xa)*(xi - x1)*(zi - z1)*d1a^4 + ((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))*4*d1a^2*(z1 - za))/d1a^8
                    final double derivativeZa = -10.0 * pathLossExponent / ln10 * (-diffZi1 * d1a2 + crossDiff * 2.0 * diffZ1a) / d1a4
                            - 5.0 * pathLossExponent / ln10 * (-2.0 * diffZ1a * diffXi12 * d1a4 - (-diffX1a2 + diffY1a2 + diffZ1a2) * diffXi12 * 4.0 * d1a2 * diffZ1a) / d1a8
                            - 5.0 * pathLossExponent / ln10 * (-2.0 * diffZ1a * diffYi12 * d1a4 + (diffX1a2 - diffY1a2 + diffZ1a2) * diffYi12 * 4.0 * d1a2 * diffZ1a) / d1a8
                            - 5.0 * pathLossExponent / ln10 * (2.0 * diffZ1a * diffZi12 * d1a4 + (diffX1a2 + diffY1a2 - diffZ1a2) * diffZi12 * 4.0 * d1a2 * diffZ1a) / d1a8
                            + 20.0 * pathLossExponent / ln10 * ((diffX1a * diffY1a * diffXi1 * diffYi1) * 4.0 * d1a2 * diffZ1a) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (-diffY1a * diffYi1 * diffZi1 * d1a4 + (diffY1a * diffZ1a * diffYi1 * diffZi1) * 4.0 * d1a2 * diffZ1a) / d1a8
                            + 20.0 * pathLossExponent / ln10 * (-diffX1a * diffXi1 * diffZi1 * d1a4 + (diffX1a * diffZ1a * diffXi1 * diffZi1) * 4.0 * d1a2 * diffZ1a) / d1a8;


                    //derivative of rssi respect to xi

                    //We have
                    //Pr(pi) = Pr(p1)
                    //  -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))
                    //  -5*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)^2
                    //  -5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)^2
                    //  -5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(zi - z1)^2
                    //  +20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(yi - y1)
                    //  +20*n*(y1 - ya)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)*(zi - z1)
                    //  +20*n*(x1 - xa)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(zi - z1)

                    //Hence
                    //diff(((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1)))/diff(xi) =
                    //  x1 - xa

                    //diff(((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)/diff(xi) =
                    //  2*(xi - x1)*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)

                    //diff(((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)/diff(xi) = 0

                    //diff(((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)/diff(xi) = 0

                    //diff((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))/diff(xi) =
                    //  (x1 - xa)*(y1 - ya)*(yi - y1)

                    //diff((y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1))/diff(xi) = 0

                    //diff((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))/diff(xi) =
                    //  (x1 - xa)*(z1 - za)*(zi - z1)

                    //diff(Pr(pi))/diff(xi) = -10*n/ln(10)*((x1 - xa)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - (((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1)))*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //  -5*n/ln(10)*(2*(xi - x1)*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(0*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(0*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*((x1 - xa)*(y1 - ya)*(yi - y1)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*(0*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1))*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*((x1 - xa)*(z1 - za)*(zi - z1)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4

                    //diff(Pr(pi))/diff(xi) = -10*n/ln(10)*(x1 - xa)*d1a^2/d1a^4
                    //  -5*n/ln(10)*(2*(xi - x1)*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*d1a^4)/d1a^8
                    //  +20*n/ln(10)*(x1 - xa)*(y1 - ya)*(yi - y1)*d1a^4/d1a^8
                    //  +20*n/ln(10)*(x1 - xa)*(z1 - za)*(zi - z1)*d1a^4)/d1a^8

                    //diff(Pr(pi))/diff(xi) = -10*n/ln(10)*(x1 - xa)/d1a^2
                    //  -10*n/ln(10)*(xi - x1)*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/d1a^4
                    //  +20*n/ln(10)*(x1 - xa)*(y1 - ya)*(yi - y1)/d1a^4
                    //  +20*n/ln(10)*(x1 - xa)*(z1 - za)*(zi - z1)/d1a^4
                    final double derivativeXi = -10.0 * pathLossExponent / ln10 * diffX1a / d1a2
                            - 10.0 * pathLossExponent / ln10 * diffXi1 * (-diffX1a2 + diffY1a2 + diffZ1a2) / d1a4
                            + 20.0 * pathLossExponent / ln10 * diffX1a * diffY1a * diffYi1 / d1a4
                            + 20.0 * pathLossExponent / ln10 * diffX1a * diffZ1a * diffZi1 / d1a4;


                    //derivative of rssi respect to yi

                    //We have
                    //Pr(pi) = Pr(p1)
                    //  -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))
                    //  -5*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)^2
                    //  -5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)^2
                    //  -5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(zi - z1)^2
                    //  +20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(yi - y1)
                    //  +20*n*(y1 - ya)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)*(zi - z1)
                    //  +20*n*(x1 - xa)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(zi - z1)

                    //Hence
                    //diff(((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1)))/diff(yi) =
                    //  y1 - ya

                    //diff(((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)/diff(yi) = 0

                    //diff(((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)/diff(yi) =
                    //  2*(yi - y1)*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)

                    //diff(((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)/diff(yi) = 0

                    //diff((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))/diff(yi) =
                    //  (x1 - xa)*(y1 - ya)*(xi - x1)

                    //diff((y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1))/diff(yi) =
                    //  (y1 - ya)*(z1 - za)*(zi - z1)

                    //diff((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))/diff(yi) = 0

                    //diff(Pr(pi))/diff(yi) = -10*n/ln(10)*((y1 - ya)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - (((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1)))*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //  -5*n/ln(10)*(0*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(2*(yi - y1)*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(0*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*((x1 - xa)*(y1 - ya)*(xi - x1)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*((y1 - ya)*(z1 - za)*(zi - z1)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1))*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  +20*n/ln(10)*(0*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4

                    //diff(Pr(pi))/diff(yi) = -10*n/ln(10)*(y1 - ya)*d1a^2/d1a^4
                    //  -5*n/ln(10)*(2*(yi - y1)*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*d1a^4)/d1a^8
                    //  +20*n/ln(10)*((x1 - xa)*(y1 - ya)*(xi - x1)*d1a^4)/d1a^8
                    //  +20*n/ln(10)*((y1 - ya)*(z1 - za)*(zi - z1)*d1a^4)/d1a^8

                    //diff(Pr(pi))/diff(yi) = -10*n/ln(10)*(y1 - ya)/d1a^2
                    //  -10*n/ln(10)*(yi - y1)*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/d1a^4
                    //  +20*n/ln(10)*(x1 - xa)*(y1 - ya)*(xi - x1)/d1a^4
                    //  +20*n/ln(10)*(y1 - ya)*(z1 - za)*(zi - z1)/d1a^4
                    final double derivativeYi = -10.0 * pathLossExponent / ln10 * diffY1a / d1a2
                            - 10.0 * pathLossExponent / ln10 * diffYi1 * (diffX1a2 - diffY1a2 + diffZ1a2) / d1a4
                            + 20.0 * pathLossExponent / ln10 * diffX1a * diffY1a * diffXi1 / d1a4
                            + 20.0 * pathLossExponent / ln10 * diffY1a * diffZ1a * diffZi1 / d1a4;


                    //derivative of rssi respect to zi

                    //We have
                    //Pr(pi) = Pr(p1)
                    //  -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))
                    //  -5*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)^2
                    //  -5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)^2
                    //  -5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(zi - z1)^2
                    //  +20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(yi - y1)
                    //  +20*n*(y1 - ya)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(yi - y1)*(zi - z1)
                    //  +20*n*(x1 - xa)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)*(xi - x1)*(zi - z1)

                    //Hence
                    //diff(((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1)))/diff(zi) =
                    //  z1 - za

                    //diff(((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)/diff(zi) = 0

                    //diff(((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)/diff(zi) = 0

                    //diff(((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)/diff(zi) =
                    //  2*(zi - z1)*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)

                    //diff((x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1))/diff(zi) = 0

                    //diff((y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1))/diff(zi) =
                    //  (y1 - ya)*(z1 - za)*(yi - y1)

                    //diff((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))/diff(zi) =
                    //  (x1 - xa)*(z1 - za)*(xi - x1)

                    //diff(Pr(pi))/diff(zi) = -10*n/ln(10)*((z1 - za)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - (((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1)))*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //  -5*n/ln(10)*(0*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*(xi - x1)^2)*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(0*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*(yi - y1)^2)*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  -5*n/ln(10)*(2*(zi - z1)*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*(zi - z1)^2)*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  20*n/ln(10)*(0*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - (x1 - xa)*(y1 - ya)*(xi - x1)*(yi - y1)*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  20*n/ln(10)*((y1 - ya)*(z1 - za)*(yi - y1)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((y1 - ya)*(z1 - za)*(yi - y1)*(zi - z1))*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
                    //  20*n/ln(10)*((x1 - xa)*(z1 - za)*(xi - x1)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)*(z1 - za)*(xi - x1)*(zi - z1))*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4

                    //diff(Pr(pi))/diff(zi) = -10*n/ln(10)*((z1 - za)*d1a^2)/d1a^4
                    //  -5*n/ln(10)*(2*(zi - z1)*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*d1a^4)/d1a^8
                    //  20*n/ln(10)*((y1 - ya)*(z1 - za)*(yi - y1)*d1a^4)/d1a^8
                    //  20*n/ln(10)*((x1 - xa)*(z1 - za)*(xi - x1)*d1a^4)/d1a^8

                    //diff(Pr(pi))/diff(zi) = -10*n/ln(10)*(z1 - za)/d1a^2
                    //  -10*n/ln(10)*(zi - z1)*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/d1a^4
                    //  20*n/ln(10)*(y1 - ya)*(z1 - za)*(yi - y1)/d1a^4
                    //  20*n/ln(10)*(x1 - xa)*(z1 - za)*(xi - x1)/d1a^4
                    final double derivativeZi = -10.0 * pathLossExponent / ln10 * diffZ1a / d1a2
                            - 10.0 * pathLossExponent / ln10 * diffZi1 * (diffX1a2 + diffY1a2 - diffZ1a2) / d1a4
                            + 20.0 * pathLossExponent / ln10 * diffY1a * diffZ1a * diffYi1 / d1a4
                            + 20.0 * pathLossExponent / ln10 * diffX1a * diffZ1a * diffXi1 / d1a4;


                    //set derivatives fingerprintRssi, pathLossExponent, x1, y1, z1, xa, ya, za, xi, yi, zi
                    jacobian.setElementAtIndex(0, derivativeFingerprintRssi);
                    jacobian.setElementAtIndex(1, derivativePathLossExponent);
                    jacobian.setElementAtIndex(2, derivativeX1);
                    jacobian.setElementAtIndex(3, derivativeY1);
                    jacobian.setElementAtIndex(4, derivativeZ1);
                    jacobian.setElementAtIndex(5, derivativeXa);
                    jacobian.setElementAtIndex(6, derivativeYa);
                    jacobian.setElementAtIndex(7, derivativeZa);
                    jacobian.setElementAtIndex(8, derivativeXi);
                    jacobian.setElementAtIndex(9, derivativeYi);
                    jacobian.setElementAtIndex(10, derivativeZi);
                }

                @Override
                public int getNumberOfVariables() {
                    return 1;
                }
            }, mean, covariance);
        } catch (final AlgebraException | StatisticsException e) {
            throw new IndoorException(e);
        }
    }

    /**
     * Propagates provided variances (fingerprint rssi variance, path-loss exponent variance,
     * fingerprint position covariance and radio source position covariance) into
     * rssi variance by considering the 2D 3rd order Taylor expression of received power.
     * Notice that any unknown variance is assumed to be zero.
     *
     * @param fingerprintRssi               closest located fingerprint reading RSSI expressed in dBm's.
     * @param pathLossExponent              path-loss exponent.
     * @param fingerprintPosition           position of closest fingerprint.
     * @param radioSourcePosition           radio source position associated to fingerprint reading.
     * @param estimatedPosition             position to be estimated. Usually this is equal to the
     *                                      initial position used by a non linear algorithm.
     * @param fingerprintRssiVariance       variance of fingerprint RSSI or null if unknown.
     * @param pathLossExponentVariance      variance of path-loss exponent or null if unknown.
     * @param fingerprintPositionCovariance covariance of fingerprint position or null if
     *                                      unknown.
     * @param radioSourcePositionCovariance covariance of radio source position or null
     *                                      if unknown.
     * @param estimatedPositionCovariance   covariance of position to be estimated or null
     *                                      if unknown. (This is usually unknown).
     * @return a normal distribution containing expected received RSSI value and its variance.
     * @throws IndoorException if something fails.
     */
    public static MultivariateNormalDist propagateVariancesToRssiVarianceThirdOrderNonLinear2D(
            final double fingerprintRssi, final double pathLossExponent,
            final Point2D fingerprintPosition, final Point2D radioSourcePosition,
            final Point2D estimatedPosition, final Double fingerprintRssiVariance,
            final Double pathLossExponentVariance,
            final Matrix fingerprintPositionCovariance,
            final Matrix radioSourcePositionCovariance,
            final Matrix estimatedPositionCovariance) throws IndoorException {

        if (fingerprintPosition == null || radioSourcePosition == null ||
                estimatedPosition == null) {
            return null;
        }

        //3rd order Taylor expression of received power in 2D:
        //Pr(pi) = Pr(p1)
        //  -10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1) +
        //  -10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1) +
        //  -5*n*((y1 - ya)^2 - (x1 - xa)^2)/(ln(10)*d1a^4)*(xi - x1)^2 +
        //  -5*n*((x1 - xa)^2 - (y1 - ya)^2)/(ln(10)*d1a^4)*(yi - y1)^2 +
        //  20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4)*(xi - x1)*(yi - y1) +
        //  -10/6*n/ln(10)*(-2*(x1 - xa)*dia^4 - ((y1 - ya)^2 - (x1 - xa)^2)*4*d1a^2*(x1 - xa))/d1a^8*(xi - x1)^3 +
        //  -10/6*n/ln(10)*(-2*(y1 - ya)*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2)*4*d1a^2*(y1 - ya))/d1a^8*(yi - y1)^3 +
        //  -5*n/ln(10)*(2*(y1 - ya)*d1a^4 - ((y1 - ya)^2 - (x1 - xa)^2)*4*d1a^2*(y1 - ya))/d1a^8*(xi - x1)^2*(yi - y1) +
        //  -5*n/ln(10)*(2*(x1 - xa)*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2)*4*d1a^2*(x1 - xa))/d1a^8*(xi - x1)*(yi - y1)^2
        //where d1a2 = (x1 - xa)^2 + (y1 - ya)^2

        final double x1 = fingerprintPosition.getInhomX();
        final double y1 = fingerprintPosition.getInhomY();

        final double xa = radioSourcePosition.getInhomX();
        final double ya = radioSourcePosition.getInhomY();

        final double xi = estimatedPosition.getInhomX();
        final double yi = estimatedPosition.getInhomY();

        final double[] mean = new double[]{
                fingerprintRssi, pathLossExponent, x1, y1, xa, ya, xi, yi
        };
        final Matrix covariance = Matrix.diagonal(new double[]{
                fingerprintRssiVariance != null ? fingerprintRssiVariance : 0.0,
                pathLossExponentVariance != null ? pathLossExponentVariance : 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        });

        if (fingerprintPositionCovariance != null &&
                fingerprintPositionCovariance.getRows() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                fingerprintPositionCovariance.getColumns() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {

            covariance.setSubmatrix(2, 2,
                    3, 3,
                    fingerprintPositionCovariance);
        }

        if (radioSourcePositionCovariance != null &&
                radioSourcePositionCovariance.getRows() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                radioSourcePositionCovariance.getColumns() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            covariance.setSubmatrix(4, 4,
                    5, 5,
                    radioSourcePositionCovariance);
        }

        if (estimatedPositionCovariance != null &&
                estimatedPositionCovariance.getRows() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                estimatedPositionCovariance.getColumns() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            covariance.setSubmatrix(6, 6,
                    7, 7,
                    estimatedPositionCovariance);
        }

        try {
            //although less precise, we use a jacobian estimator to simplify expressions
            final MultiVariateFunctionEvaluatorListener evaluator = new MultiVariateFunctionEvaluatorListener() {
                @Override
                public void evaluate(
                        final double[] point, final double[] result) {
                    //Pr(pi) = Pr(p1)
                    //  -10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
                    //  -10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
                    //  -5*n*((y1 - ya)^2 - (x1 - xa)^2)/(ln(10)*d1a^4)*(xi - x1)^2
                    //  -5*n*((x1 - xa)^2 - (y1 - ya)^2)/(ln(10)*d1a^4)*(yi - y1)^2
                    //  +20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4)*(xi - x1)*(yi - y1)
                    //  -10/6*n/ln(10)*(-2*(x1 - xa)*dia^4 - ((y1 - ya)^2 - (x1 - xa)^2)*4*d1a^2*(x1 - xa))/d1a^8*(xi - x1)^3
                    //  -10/6*n/ln(10)*(-2*(y1 - ya)*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2)*4*d1a^2*(y1 - ya))/d1a^8*(yi - y1)^3
                    //  -5*n/ln(10)*(2*(y1 - ya)*d1a^4 - ((y1 - ya)^2 - (x1 - xa)^2)*4*d1a^2*(y1 - ya))/d1a^8*(xi - x1)^2*(yi - y1)
                    //  -5*n/ln(10)*(2*(x1 - xa)*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2)*4*d1a^2*(x1 - xa))/d1a^8*(xi - x1)*(yi - y1)^2
                    //where d1a2 = (x1 - xa)^2 + (y1 - ya)^2

                    //Hence:
                    //Pr(pi) = Pr(p1)
                    //  -10*n*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))*(xi - x1)
                    //  -10*n*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))*(yi - y1)
                    //  -5*n*((y1 - ya)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2)*(xi - x1)^2
                    //  -5*n*((x1 - xa)^2 - (y1 - ya)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2)*(yi - y1)^2
                    //  +20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2)*(xi - x1)*(yi - y1)
                    //  -10/6*n/ln(10)*(-2*(x1 - xa)*((x1 - xa)^2 + (y1 - ya)^2)^2 - ((y1 - ya)^2 - (x1 - xa)^2)*4*((x1 - xa)^2 + (y1 - ya)^2)*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2)^4*(xi - x1)^3
                    //  -10/6*n/ln(10)*(-2*(y1 - ya)*((x1 - xa)^2 + (y1 - ya)^2)^2 - ((x1 - xa)^2 - (y1 - ya)^2)*4*((x1 - xa)^2 + (y1 - ya)^2)*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2)^4*(yi - y1)^3
                    //  -5*n/ln(10)*(2*(y1 - ya)*((x1 - xa)^2 + (y1 - ya)^2)^2 - ((y1 - ya)^2 - (x1 - xa)^2)*4*((x1 - xa)^2 + (y1 - ya)^2)*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2)^4*(xi - x1)^2*(yi - y1)
                    //  -5*n/ln(10)*(2*(x1 - xa)*((x1 - xa)^2 + (y1 - ya)^2)^2 - ((x1 - xa)^2 - (y1 - ya)^2)*4*((x1 - xa)^2 + (y1 - ya)^2)*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2)^4*(xi - x1)*(yi - y1)^2

                    final double fingerprintRssi = point[0];
                    final double pathLossExponent = point[1];
                    final double x1 = point[2];
                    final double y1 = point[3];
                    final double xa = point[4];
                    final double ya = point[5];
                    final double xi = point[6];
                    final double yi = point[7];

                    final double diffX1a = x1 - xa;
                    final double diffY1a = y1 - ya;

                    final double diffXi1 = xi - x1;
                    final double diffYi1 = yi - y1;

                    final double diffX1a2 = diffX1a * diffX1a;
                    final double diffY1a2 = diffY1a * diffY1a;

                    final double diffXi12 = diffXi1 * diffXi1;
                    final double diffYi12 = diffYi1 * diffYi1;

                    final double diffXi13 = diffXi12 * diffXi1;
                    final double diffYi13 = diffYi12 * diffYi1;

                    final double d1a2 = diffX1a2 + diffY1a2;
                    final double d1a4 = d1a2 * d1a2;
                    final double d1a8 = d1a4 * d1a4;

                    final double ln10 = Math.log(10.0);

                    result[0] = fingerprintRssi
                            - 10.0 * pathLossExponent * diffX1a / (ln10 * d1a2) * diffXi1
                            - 10.0 * pathLossExponent * diffY1a / (ln10 * d1a2) * diffYi1
                            - 5.0 * pathLossExponent * (-diffX1a2 + diffY1a2) / (ln10 * d1a4) * diffXi12
                            - 5.0 * pathLossExponent * (diffX1a2 - diffY1a2) / (ln10 * d1a4) * diffYi12
                            + 20.0 * pathLossExponent * diffX1a * diffY1a / (ln10 * d1a4) * diffXi1 * diffYi1
                            - 10.0 / 6.0 * pathLossExponent / ln10 * (-2.0 * diffX1a * d1a4 - (-diffX1a2 + diffY1a2) * 4.0 * d1a2 * diffX1a) / d1a8 * diffXi13
                            - 10.0 / 6.0 * pathLossExponent / ln10 * (-2.0 * diffY1a * d1a4 - (diffX1a2 - diffY1a2) * 4.0 * d1a2 * diffY1a) / d1a8 * diffYi13
                            - 5.0 * pathLossExponent / ln10 * (2.0 * diffY1a * d1a4 - (-diffX1a2 + diffY1a2) * 4.0 * d1a2 * diffY1a) / d1a8 * diffXi12 * diffYi1
                            - 5.0 * pathLossExponent / ln10 * (2.0 * diffX1a * d1a4 - (diffX1a2 - diffY1a2) * 4.0 * d1a2 * diffX1a) / d1a8 * diffXi1 * diffYi12;
                }

                @Override
                public int getNumberOfVariables() {
                    return 1;
                }
            };

            final JacobianEstimator jacobianEstimator = new JacobianEstimator(evaluator);

            return MultivariateNormalDist.propagate(new MultivariateNormalDist.JacobianEvaluator() {
                @Override
                public void evaluate(
                        final double[] x, final double[] y, final Matrix jacobian) {

                    try {
                        evaluator.evaluate(x, y);
                        jacobianEstimator.jacobian(x, jacobian);
                    } catch (final EvaluationException ignore) {
                        //never happens
                    }
                }

                @Override
                public int getNumberOfVariables() {
                    return 1;
                }
            }, mean, covariance);
        } catch (final AlgebraException | StatisticsException e) {
            throw new IndoorException(e);
        }
    }

    /**
     * Propagates provided variances (fingerprint rssi variance, path-loss exponent variance,
     * fingerprint position covariance and radio source position covariance) into
     * rssi variance by considering the 3D 3rd order Taylor expression of received power.
     * Notice that any unknown variance is assumed to be zero.
     *
     * @param fingerprintRssi               closest located fingerprint reading RSSI expressed in dBm's.
     * @param pathLossExponent              path-loss exponent.
     * @param fingerprintPosition           position of closest fingerprint.
     * @param radioSourcePosition           radio source position associated to fingerprint reading.
     * @param estimatedPosition             position to be estimated. Usually this is equal to the
     *                                      initial position used by a non linear algorithm.
     * @param fingerprintRssiVariance       variance of fingerprint RSSI or null if unknown.
     * @param pathLossExponentVariance      variance of path-loss exponent or null if unknown.
     * @param fingerprintPositionCovariance covariance of fingerprint position or null if
     *                                      unknown.
     * @param radioSourcePositionCovariance covariance of radio source position or null
     *                                      if unknown.
     * @param estimatedPositionCovariance   covariance of position to be estimated or null
     *                                      if unknown. (This is usually unknown).
     * @return a normal distribution containing expected received RSSI value and its variance.
     * @throws IndoorException if something fails.
     */
    public static MultivariateNormalDist propagateVariancesToRssiVarianceThirdOrderNonLinear3D(
            final double fingerprintRssi, final double pathLossExponent,
            final Point3D fingerprintPosition, final Point3D radioSourcePosition,
            final Point3D estimatedPosition,
            final Double fingerprintRssiVariance,
            final Double pathLossExponentVariance,
            final Matrix fingerprintPositionCovariance,
            final Matrix radioSourcePositionCovariance,
            final Matrix estimatedPositionCovariance) throws IndoorException {

        if (fingerprintPosition == null || radioSourcePosition == null ||
                estimatedPosition == null) {
            return null;
        }

        //1st order Taylor expression of received power in 3D:
        //Pr(pi) = Pr(p1)
        //  - 10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
        //  - 10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
        //  - 10*n*(z1 - za)/(ln(10)*d1a^2)*(zi - z1)
        //where d1a^2 = (x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2

        final double x1 = fingerprintPosition.getInhomX();
        final double y1 = fingerprintPosition.getInhomY();
        final double z1 = fingerprintPosition.getInhomZ();

        final double xa = radioSourcePosition.getInhomX();
        final double ya = radioSourcePosition.getInhomY();
        final double za = radioSourcePosition.getInhomZ();

        final double xi = estimatedPosition.getInhomX();
        final double yi = estimatedPosition.getInhomY();
        final double zi = estimatedPosition.getInhomZ();

        final double[] mean = new double[]{
                fingerprintRssi, pathLossExponent, x1, y1, z1, xa, ya, za, xi, yi, zi
        };
        final Matrix covariance = Matrix.diagonal(new double[]{
                fingerprintRssiVariance != null ? fingerprintRssiVariance : 0.0,
                pathLossExponentVariance != null ? pathLossExponentVariance : 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        });

        if (fingerprintPositionCovariance != null &&
                fingerprintPositionCovariance.getRows() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                fingerprintPositionCovariance.getColumns() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {

            covariance.setSubmatrix(2, 2,
                    4, 4,
                    fingerprintPositionCovariance);
        }

        if (radioSourcePositionCovariance != null &&
                radioSourcePositionCovariance.getRows() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                radioSourcePositionCovariance.getColumns() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            covariance.setSubmatrix(5, 5,
                    7, 7,
                    radioSourcePositionCovariance);
        }

        if (estimatedPositionCovariance != null &&
                estimatedPositionCovariance.getRows() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                estimatedPositionCovariance.getColumns() == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            covariance.setSubmatrix(8, 8,
                    10, 10,
                    estimatedPositionCovariance);
        }

        try {
            //although less precise, we use a jacobian estimator to simplify expressions
            final MultiVariateFunctionEvaluatorListener evaluator = new MultiVariateFunctionEvaluatorListener() {
                @Override
                public void evaluate(
                        final double[] point, final double[] result) {
                    //Pr(pi = (xi,yi)) = Pr(p1) +
                    //  -10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1) +
                    //  -10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1) +
                    //  -10*n*(z1 - za)/(ln(10)*d1a^2)*(zi - z1) +
                    //  -5*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*d1a^4)*(xi - x1)^2 +
                    //  -5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*d1a^4)*(yi - y1)^2 +
                    //  -5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*d1a^4)*(zi - z1)^2 +
                    //  20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4)*(xi - x1)*(yi - y1) +
                    //  20*n*(y1 - ya)*(z1 - za)/(ln(10)*d1a^4)*(yi - y1)*(zi - z1) +
                    //  20*n*(x1 - xa)*(z1 - za)/(ln(10)*d1a^4)*(xi - x1)*(zi - z1) +
                    //  -10/6*n/ln(10)*(-2*(x1 - xa)*d1a^4 - ((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*4*d1a^2*(x1 - xa))/d1a^8*(xi - x1)^3 +
                    //  -10/6*n/ln(10)*(-2*(y1 - ya)*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*4*d1a^2*(y1 - ya))/d1a^8*(yi - y1)^3 +
                    //  -10/6*n/ln(10)*(-2*(z1 - za)*d1a^4 - ((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*4*d1a^2*(z1 - za))/d1a^8*(zi - z1)^3 +
                    //  -5*n/ln(10)*(2*(y1 - ya)*d1a^4 - ((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*4*d1a^2*(y1 - ya))/d1a^8*(xi - x1)^2*(yi - y1) +
                    //  -5*n/ln(10)*(2*(z1 - za)*d1a^4 - ((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*4*d1a^2*(z1 - za))/d1a^8*(xi - x1)^2*(zi - z1) +
                    //  -5*n/ln(10)*(2*(x1 - xa)*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*4*d1a^2*(x1 - xa))/d1a^8*(xi - x1)*(yi - y1)^2 +
                    //  -5*n/ln(10)*(2*(x1 - xa)*d1a^4 - ((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*4*d1a^2*(x1 - xa))/d1a^8*(xi - x1)*(zi - z1)^2 +
                    //  -5*n/ln(10)*(2*(z1 - za)*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*4*d1a^2*(z1 - za))/d1a^8*(yi - y1)^2*(zi - z1) +
                    //  -5*n/ln(10)*(2*(y1 - ya)*d1a^4 - ((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*4*d1a^2*(y1 - ya))/d1a^8*(yi - y1)*(zi - z1)^2 +
                    //  -80*n/ln(10)*((x1 - xa)*(y1 - ya)*(z1 - za)*d1a^2)/d1a^8*(xi - x1)*(yi - y1)*(zi - z1)

                    final double fingerprintRssi = point[0];
                    final double pathLossExponent = point[1];
                    final double x1 = point[2];
                    final double y1 = point[3];
                    final double z1 = point[4];
                    final double xa = point[5];
                    final double ya = point[6];
                    final double za = point[7];
                    final double xi = point[8];
                    final double yi = point[9];
                    final double zi = point[10];

                    final double diffX1a = x1 - xa;
                    final double diffY1a = y1 - ya;
                    final double diffZ1a = z1 - za;

                    final double diffXi1 = xi - x1;
                    final double diffYi1 = yi - y1;
                    final double diffZi1 = zi - z1;

                    final double diffX1a2 = diffX1a * diffX1a;
                    final double diffY1a2 = diffY1a * diffY1a;
                    final double diffZ1a2 = diffZ1a * diffZ1a;

                    final double diffXi12 = diffXi1 * diffXi1;
                    final double diffYi12 = diffYi1 * diffYi1;
                    final double diffZi12 = diffZi1 * diffZi1;

                    final double diffXi13 = diffXi12 * diffXi1;
                    final double diffYi13 = diffYi12 * diffYi1;
                    final double diffZi13 = diffZi12 * diffZi1;

                    final double d1a2 = diffX1a2 + diffY1a2 + diffZ1a2;
                    final double d1a4 = d1a2 * d1a2;
                    final double d1a8 = d1a4 * d1a4;

                    final double ln10 = Math.log(10.0);

                    final double value1 = -10.0 * pathLossExponent * diffX1a / (ln10 * d1a2);
                    final double value2 = -10.0 * pathLossExponent * diffY1a / (ln10 * d1a2);
                    final double value3 = -10.0 * pathLossExponent * diffZ1a / (ln10 * d1a2);
                    final double value4 = -5.0 * pathLossExponent * (-diffX1a2 + diffY1a2 + diffZ1a2) / (ln10 * d1a4);
                    final double value5 = -5.0 * pathLossExponent * (diffX1a2 - diffY1a2 + diffZ1a2) / (ln10 * d1a4);
                    final double value6 = -5.0 * pathLossExponent * (diffX1a2 + diffY1a2 - diffZ1a2) / (ln10 * d1a4);
                    final double value7 = 20.0 * pathLossExponent * diffX1a * diffY1a / (ln10 * d1a4);
                    final double value8 = 20.0 * pathLossExponent * diffY1a * diffZ1a / (ln10 * d1a4);
                    final double value9 = 20.0 * pathLossExponent * diffX1a * diffZ1a / (ln10 * d1a4);
                    final double value10 = -10.0 / 6.0 * pathLossExponent / ln10 * (-2.0 * diffX1a * d1a4 - (-diffX1a2 + diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffX1a) / d1a8;
                    final double value11 = -10.0 / 6.0 * pathLossExponent / ln10 * (-2.0 * diffY1a * d1a4 - (diffX1a2 - diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffY1a) / d1a8;
                    final double value12 = -10.0 / 6.0 * pathLossExponent / ln10 * (-2.0 * diffZ1a * d1a4 - (diffX1a2 + diffY1a2 - diffZ1a2) * 4.0 * d1a2 * diffZ1a) / d1a8;
                    final double value13 = -5.0 * pathLossExponent / ln10 * (2.0 * diffY1a * d1a4 - (-diffX1a2 + diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffY1a) / d1a8;
                    final double value14 = -5.0 * pathLossExponent / ln10 * (2.0 * diffZ1a * d1a4 - (-diffX1a2 + diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffZ1a) / d1a8;
                    final double value15 = -5.0 * pathLossExponent / ln10 * (2.0 * diffX1a * d1a4 - (diffX1a2 - diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffX1a) / d1a8;
                    final double value16 = -5.0 * pathLossExponent / ln10 * (2.0 * diffX1a * d1a4 - (diffX1a2 + diffY1a2 - diffZ1a2) * 4.0 * d1a2 * diffX1a) / d1a8;
                    final double value17 = -5.0 * pathLossExponent / ln10 * (2.0 * diffZ1a * d1a4 - (diffX1a2 - diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffZ1a) / d1a8;
                    final double value18 = -5.0 * pathLossExponent / ln10 * (2.0 * diffY1a * d1a4 - (diffX1a2 + diffY1a2 - diffZ1a2) * 4.0 * d1a2 * diffY1a) / d1a8;
                    final double value19 = -80.0 * pathLossExponent / ln10 * (diffX1a * diffY1a * diffZ1a * d1a2) / d1a8;

                    result[0] = fingerprintRssi
                            + value1 * diffXi1
                            + value2 * diffYi1
                            + value3 * diffZi1
                            + value4 * diffXi12
                            + value5 * diffYi12
                            + value6 * diffZi12
                            + value7 * diffXi1 * diffYi1
                            + value8 * diffYi1 * diffZi1
                            + value9 * diffXi1 * diffZi1
                            + value10 * diffXi13
                            + value11 * diffYi13
                            + value12 * diffZi13
                            + value13 * diffXi12 * diffYi1
                            + value14 * diffXi12 * diffZi1
                            + value15 * diffXi1 * diffYi12
                            + value16 * diffXi1 * diffZi12
                            + value17 * diffYi12 * diffZi1
                            + value18 * diffYi1 * diffZi12
                            + value19 * diffXi1 * diffYi1 * diffZi1;
                }

                @Override
                public int getNumberOfVariables() {
                    return 1;
                }
            };

            final JacobianEstimator jacobianEstimator = new JacobianEstimator(evaluator);

            return MultivariateNormalDist.propagate(new MultivariateNormalDist.JacobianEvaluator() {
                @Override
                public void evaluate(
                        final double[] x, final double[] y, final Matrix jacobian) {

                    try {
                        evaluator.evaluate(x, y);
                        jacobianEstimator.jacobian(x, jacobian);
                    } catch (final EvaluationException ignore) {
                        //never happens
                    }
                }

                @Override
                public int getNumberOfVariables() {
                    return 1;
                }
            }, mean, covariance);
        } catch (final AlgebraException | StatisticsException e) {
            throw new IndoorException(e);
        }
    }

    /**
     * Propagates provided variances (path-loss exponent variance,
     * fingerprint position covariance and radio source position covariance) into
     * difference of rssi variance by considering the 2D expression.
     * Notice that any unknown variance is assumed to be zero.
     *
     * @param pathLossExponent              path-loss exponent.
     * @param fingerprintPosition           position of closest fingerprint.
     * @param radioSourcePosition           radio source position associated to fingerprint reading.
     * @param estimatedPosition             position to be estimated. Usually this is equal to the
     *                                      initial position used by a non linear algorithm.
     * @param pathLossExponentVariance      variance of path-loss exponent or null if unknown.
     * @param fingerprintPositionCovariance covariance of fingerprint position or null if
     *                                      unknown.
     * @param radioSourcePositionCovariance covariance of radio source position or null
     *                                      if unknown.
     * @param estimatedPositionCovariance   covariance of position to be estimated or null
     *                                      if unknown. (This is usually unknown).
     * @return a normal distribution containing expected received RSSI value and its variance.
     * @throws IndoorException if something fails.
     */
    public static MultivariateNormalDist propagateVariancesToRssiDifferenceVariance2D(
            final double pathLossExponent,
            final Point2D fingerprintPosition, final Point2D radioSourcePosition,
            final Point2D estimatedPosition,
            final Double pathLossExponentVariance,
            final Matrix fingerprintPositionCovariance,
            final Matrix radioSourcePositionCovariance,
            final Matrix estimatedPositionCovariance) throws IndoorException {

        if (fingerprintPosition == null || radioSourcePosition == null ||
                estimatedPosition == null) {
            return null;
        }

        //Expression being used is:
        //Prdiff1a = Pr(pi) - Pr(p1) = 5*n*log(d1a^2) - 5*n*log(dia^2) =
        //  = 5*n*log((x1 - xa)^2 + (y1 - ya)^2) - 5*n*log((xi - xa)^2 + (yi - ya)^2)


        final double x1 = fingerprintPosition.getInhomX();
        final double y1 = fingerprintPosition.getInhomY();

        final double xa = radioSourcePosition.getInhomX();
        final double ya = radioSourcePosition.getInhomY();

        final double xi = estimatedPosition.getInhomX();
        final double yi = estimatedPosition.getInhomY();

        final double[] mean = new double[]{
                pathLossExponent, x1, y1, xa, ya, xi, yi
        };
        final Matrix covariance = Matrix.diagonal(new double[]{
                pathLossExponentVariance != null ? pathLossExponentVariance : 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        });

        if (fingerprintPositionCovariance != null &&
                fingerprintPositionCovariance.getRows() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                fingerprintPositionCovariance.getColumns() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {

            covariance.setSubmatrix(1, 1,
                    2, 2,
                    fingerprintPositionCovariance);
        }

        if (radioSourcePositionCovariance != null &&
                radioSourcePositionCovariance.getRows() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                radioSourcePositionCovariance.getColumns() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            covariance.setSubmatrix(3, 3,
                    4, 4,
                    radioSourcePositionCovariance);
        }

        if (estimatedPositionCovariance != null &&
                estimatedPositionCovariance.getRows() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                estimatedPositionCovariance.getColumns() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            covariance.setSubmatrix(5, 5,
                    6, 6,
                    estimatedPositionCovariance);
        }

        try {
            return MultivariateNormalDist.propagate(new MultivariateNormalDist.JacobianEvaluator() {
                @Override
                public void evaluate(
                        final double[] x, final double[] y, final Matrix jacobian) {

                    //Expression being used is:
                    //Prdiff1a = Pr(pi) - Pr(p1) = 5*n*log(d1a^2) - 5*n*log(dia^2) =
                    //  = 5*n*log((x1 - xa)^2 + (y1 - ya)^2) - 5*n*log((xi - xa)^2 + (yi - ya)^2)

                    final double diffX1a = x1 - xa;
                    final double diffY1a = y1 - ya;

                    final double diffXia = xi - xa;
                    final double diffYia = yi - ya;

                    final double diffX1a2 = diffX1a * diffX1a;
                    final double diffY1a2 = diffY1a * diffY1a;

                    final double diffXia2 = diffXia * diffXia;
                    final double diffYia2 = diffYia * diffYia;

                    final double d1a2 = diffX1a2 + diffY1a2;
                    final double dia2 = diffXia2 + diffYia2;

                    final double ln10 = Math.log(10.0);


                    //compute gradient (is a jacobian having 1 row and 7 columns)


                    //derivative of diff rssi respect to fingerprint path-loss exponent "n"

                    final double derivativePathLossExponent = 5.0 *
                            (Math.log10(d1a2) - Math.log10(dia2));


                    //derivative of diff rssi respect to x1
                    //diff(Prdiff1a)/diff(x1) = 5*n/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))*2*(x1 - xa) =
                    //  = 10*n*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))

                    final double tmp1a = 10.0 * pathLossExponent / (ln10 * d1a2);
                    final double derivativeX1 = tmp1a * diffX1a;


                    //derivative of diff rssi respect to y1
                    //diff(Prdiff1a)/diff(y1) = 5*n/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))*2*(y1 - ya) =
                    //  = 10*n*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))

                    final double derivativeY1 = tmp1a * diffY1a;


                    //derivative of rssi respect to xi
                    //diff(Prdiff1a)/diff(xi) = -5*n/(ln(10)*((xi - xa)^2 + (yi - ya)^2))*2*(xi - xa)
                    //  = -10*n*(xi - xa)/(ln(10)*((xi - xa)^2 + (yi - ya)^2))

                    final double tmpia = 10.0 * pathLossExponent / (ln10 * dia2);
                    final double derivativeXi = -tmpia * diffXia;

                    //derivative of rssi respect to yi
                    //diff(Prdiff1a)/diff(yi) = -5*n/(ln(10)*((xi - xa)^2 + (yi - ya)^2))*2*(yi - ya)
                    //  = -10*n*(yi - ya)/(ln(10)*((xi - xa)^2 + (yi - ya)^2))

                    final double derivativeYi = -tmpia * diffYia;


                    //derivative of rssi respect to xa
                    //diff(Prdiff1a)/diff(xa) = 5*n/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))*-2*(x1 - xa) -5*n/(ln(10)*((xi - xa)^2 + (yi - ya)^2))*-2(xi - xa) =
                    //  = -10*n*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)) + 10*n*(xi - xa)/(ln(10)*((xi - xa)^2 + (yi - ya)^2))

                    final double derivativeXa = -derivativeX1 - derivativeXi;


                    //derivative of rssi respect to ya
                    //diff(Prdiff1a)/diff(ya) = 5*n/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))*-2*(y1 - ya) -5*n/(ln(10)*((xi - xa)^2 + (yi - ya)^2))*-2(yi - ya) =
                    //  = -10*n*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)) + 10*n*(yi - ya)/(ln(10)*((xi - xa)^2 + (yi - ya)^2))

                    final double derivativeYa = -derivativeY1 - derivativeYi;


                    //Prdiff1a = Pr(pi) - Pr(p1) = 5*n*log(d1a^2) - 5*n*log(dia^2) =
                    //  = 5*n*log((x1 - xa)^2 + (y1 - ya)^2) - 5*n*log((xi - xa)^2 + (yi - ya)^2)


                    //set derivatives pathLossExponent, x1, y1, xa, ya, xi, yi
                    jacobian.setElementAtIndex(0, derivativePathLossExponent);
                    jacobian.setElementAtIndex(1, derivativeX1);
                    jacobian.setElementAtIndex(2, derivativeY1);
                    jacobian.setElementAtIndex(3, derivativeXa);
                    jacobian.setElementAtIndex(4, derivativeYa);
                    jacobian.setElementAtIndex(5, derivativeXi);
                    jacobian.setElementAtIndex(6, derivativeYi);


                    y[0] = derivativePathLossExponent * pathLossExponent;
                }

                @Override
                public int getNumberOfVariables() {
                    return 1;
                }
            }, mean, covariance);
        } catch (final AlgebraException | StatisticsException e) {
            throw new IndoorException(e);
        }
    }

    /**
     * Propagates provided variances (path-loss exponent variance,
     * fingerprint position covariance and radio source position covariance) into
     * difference of rssi variance by considering the 3D expression.
     * Notice that any unknown variance is assumed to be zero.
     *
     * @param pathLossExponent              path-loss exponent.
     * @param fingerprintPosition           position of closest fingerprint.
     * @param radioSourcePosition           radio source position associated to fingerprint reading.
     * @param estimatedPosition             position to be estimated. Usually this is equal to the
     *                                      initial position used by a non linear algorithm.
     * @param pathLossExponentVariance      variance of path-loss exponent or null if unknown.
     * @param fingerprintPositionCovariance covariance of fingerprint position or null if
     *                                      unknown.
     * @param radioSourcePositionCovariance covariance of radio source position or null
     *                                      if unknown.
     * @param estimatedPositionCovariance   covariance of position to be estimated or null
     *                                      if unknown. (This is usually unknown).
     * @return a normal distribution containing expected received RSSI value and its variance.
     * @throws IndoorException if something fails.
     */
    public static MultivariateNormalDist propagateVariancesToRssiDifferenceVariance3D(
            final double pathLossExponent,
            final Point3D fingerprintPosition, final Point3D radioSourcePosition,
            final Point3D estimatedPosition,
            final Double pathLossExponentVariance,
            final Matrix fingerprintPositionCovariance,
            final Matrix radioSourcePositionCovariance,
            final Matrix estimatedPositionCovariance) throws IndoorException {

        if (fingerprintPosition == null || radioSourcePosition == null ||
                estimatedPosition == null) {
            return null;
        }

        //Expression being used is:
        //Prdiff1a = Pr(pi) - Pr(p1) = 5*n*log(d1a^2) - 5*n*log(dia^2) =
        //  = 5*n*log((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - 5*n*log((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2)


        final double x1 = fingerprintPosition.getInhomX();
        final double y1 = fingerprintPosition.getInhomY();
        final double z1 = fingerprintPosition.getInhomZ();

        final double xa = radioSourcePosition.getInhomX();
        final double ya = radioSourcePosition.getInhomY();
        final double za = radioSourcePosition.getInhomZ();

        final double xi = estimatedPosition.getInhomX();
        final double yi = estimatedPosition.getInhomY();
        final double zi = estimatedPosition.getInhomZ();

        final double[] mean = new double[]{
                pathLossExponent, x1, y1, z1, xa, ya, za, xi, yi, zi
        };
        final Matrix covariance = Matrix.diagonal(new double[]{
                pathLossExponentVariance != null ? pathLossExponentVariance : 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        });

        if (fingerprintPositionCovariance != null &&
                fingerprintPositionCovariance.getRows() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                fingerprintPositionCovariance.getColumns() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {

            covariance.setSubmatrix(1, 1,
                    3, 3,
                    fingerprintPositionCovariance);
        }

        if (radioSourcePositionCovariance != null &&
                radioSourcePositionCovariance.getRows() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                radioSourcePositionCovariance.getColumns() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            covariance.setSubmatrix(4, 4,
                    6, 6,
                    radioSourcePositionCovariance);
        }

        if (estimatedPositionCovariance != null &&
                estimatedPositionCovariance.getRows() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH &&
                estimatedPositionCovariance.getColumns() == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            covariance.setSubmatrix(7, 7,
                    9, 9,
                    estimatedPositionCovariance);
        }

        try {
            return MultivariateNormalDist.propagate(new MultivariateNormalDist.JacobianEvaluator() {
                @Override
                public void evaluate(
                        final double[] x, final double[] y, final Matrix jacobian) {

                    //Expression being used is:
                    //Prdiff1a = Pr(pi) - Pr(p1) = 5*n*log(d1a^2) - 5*n*log(dia^2) =
                    //  = 5*n*log((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - 5*n*log((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2)

                    final double diffX1a = x1 - xa;
                    final double diffY1a = y1 - ya;
                    final double diffZ1a = z1 - za;

                    final double diffXia = xi - xa;
                    final double diffYia = yi - ya;
                    final double diffZia = zi - za;

                    final double diffX1a2 = diffX1a * diffX1a;
                    final double diffY1a2 = diffY1a * diffY1a;
                    final double diffZ1a2 = diffZ1a * diffZ1a;

                    final double diffXia2 = diffXia * diffXia;
                    final double diffYia2 = diffYia * diffYia;
                    final double diffZia2 = diffZia * diffZia;

                    final double d1a2 = diffX1a2 + diffY1a2 + diffZ1a2;
                    final double dia2 = diffXia2 + diffYia2 + diffZia2;

                    final double ln10 = Math.log(10.0);


                    //compute gradient (is a jacobian having 1 row and 10 columns)

                    //derivative of diff rssi respect to fingerprint path-loss exponent "n"

                    final double derivativePathLossExponent = 5.0 *
                            (Math.log10(d1a2) - Math.log10(dia2));


                    //derivative of diff rssi respect to x1
                    //diff(Prdiff1a)/diff(x1) = 5*n/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*2*(x1 - xa) =
                    //  = 10*n*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))

                    final double tmp1a = 10.0 * pathLossExponent / (ln10 * d1a2);
                    final double derivativeX1 = tmp1a * diffX1a;


                    //derivative of diff rssi respect to y1
                    //diff(Prdiff1a)/diff(y1) = 5*n/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2) + (z1 - za)^2)*2*(y1 - ya) =
                    //  = 10*n*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))

                    final double derivativeY1 = tmp1a * diffY1a;


                    //derivative of diff rssi respect to z1
                    //diff(Prdiff1a)/diff(z1) = 5*n/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2) + (z1 - za)^2)*2*(z1 - za) =
                    //  = 10*n*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))

                    final double derivativeZ1 = tmp1a * diffZ1a;


                    //derivative of rssi respect to xi
                    //diff(Prdiff1a)/diff(xi) = -5*n/(ln(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*2*(xi - xa)
                    //  = -10*n*(xi - xa)/(ln(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))

                    final double tmpia = 10.0 * pathLossExponent / (ln10 * dia2);
                    final double derivativeXi = -tmpia * diffXia;


                    //derivative of rssi respect to yi
                    //diff(Prdiff1a)/diff(yi) = -5*n/(ln(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*2*(yi - ya)
                    //  = -10*n*(yi - ya)/(ln(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))

                    final double derivativeYi = -tmpia * diffYia;


                    //derivative of rssi respect to zi
                    //diff(Prdiff1a)/diff(zi) = -5*n/(ln(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*2*(zi - za)
                    //  = -10*n*(zi - za)/(ln(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))

                    final double derivativeZi = -tmpia * diffZia;


                    //Prdiff1a = Pr(pi) - Pr(p1) = 5*n*log(d1a^2) - 5*n*log(dia^2) =
                    //  = 5*n*log((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - 5*n*log((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2)

                    //derivative of rssi respect to xa
                    //diff(Prdiff1a)/diff(xa) = 5*n/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*-2*(x1 - xa) -5*n/(ln(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*-2(xi - xa) =
                    //  = -10*n*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)) + 10*n*(xi - xa)/(ln(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))

                    final double derivativeXa = -derivativeX1 - derivativeXi;


                    //derivative of rssi respect to ya
                    //diff(Prdiff1a)/diff(ya) = 5*n/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*-2*(y1 - ya) -5*n/(ln(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*-2(yi - ya) =
                    //  = -10*n*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)) + 10*n*(yi - ya)/(ln(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))

                    final double derivativeYa = -derivativeY1 - derivativeYi;


                    //derivative of rssi respect to za
                    //diff(Prdiff1a)/diff(za) = 5*n/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*-2*(z1 - za) -5*n/(ln(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*-2(zi - za) =
                    //  = -10*n*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)) + 10*n*(zi - za)/(ln(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))

                    final double derivativeZa = -derivativeZ1 - derivativeZi;


                    //set derivatives pathLossExponent, x1, y1, z1, xa, ya, za, xi, yi, zi
                    jacobian.setElementAtIndex(0, derivativePathLossExponent);
                    jacobian.setElementAtIndex(1, derivativeX1);
                    jacobian.setElementAtIndex(2, derivativeY1);
                    jacobian.setElementAtIndex(3, derivativeZ1);
                    jacobian.setElementAtIndex(4, derivativeXa);
                    jacobian.setElementAtIndex(5, derivativeYa);
                    jacobian.setElementAtIndex(6, derivativeZa);
                    jacobian.setElementAtIndex(7, derivativeXi);
                    jacobian.setElementAtIndex(8, derivativeYi);
                    jacobian.setElementAtIndex(9, derivativeZi);


                    y[0] = derivativePathLossExponent * pathLossExponent;
                }

                @Override
                public int getNumberOfVariables() {
                    return 1;
                }
            }, mean, covariance);
        } catch (final AlgebraException | StatisticsException e) {
            throw new IndoorException(e);
        }
    }

}
