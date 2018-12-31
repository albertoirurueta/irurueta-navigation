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
    private Utils() { }

    /**
     * Converts from dBm's to linear power value expressed in mW.
     * @param dBm value to be converted expressed in dBm's.
     * @return converted value expressed in mW.
     */
    public static double dBmToPower(double dBm) {
        return Math.pow(10.0, dBm / 10.0);
    }

    /**
     * Converts from mW to logarithmic power value expressed in dBm's.
     * @param mW value to be converted expressed in mW's.
     * @return converted value expressed in dBm's.
     */
    public static double powerTodBm(double mW) {
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
     * @param txPower transmitted power expressed in dBm's.
     * @param rxPower received power expressed in dBm's.
     * @param pathLossExponent path loss exponent.
     * @param frequency frequency expressed in Hz.
     * @param rxPowerVariance received power variance.
     * @return distance variance.
     */
    public static double propagatePowerVarianceToDistanceVariance(double txPower,
            double rxPower, double pathLossExponent, double frequency,
            Double rxPowerVariance) {
        if (rxPowerVariance == null) {
            return 0.0;
        }

        double k = SPEED_OF_LIGHT / (4.0 * Math.PI * frequency);
        double kdB = 10.0 * Math.log10(k);

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
        double tenPathLossExponent = 10.0 * pathLossExponent;
        double derivativeF = -Math.log(10.0) / tenPathLossExponent * Math.pow(10.0,
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
     * @param txPower transmitted power expressed in dBm's.
     * @param rxPower received power expressed in dBm's.
     * @param pathLossExponent path loss exponent.
     * @param frequency frequency expressed in Hz.
     * @param txPowerVariance transmitted power variance.
     * @param rxPowerVariance received power variance.
     * @param pathLossExponentVariance path loss exponent variance.
     * @return a normal distribution containing both expected distance and its variance.
     * @throws IndoorException if something fails.
     */
    public static MultivariateNormalDist propagateVariancesToDistanceVariance(final double txPower,
            final double rxPower, final double pathLossExponent, final double frequency,
            Double txPowerVariance, Double rxPowerVariance,
            Double pathLossExponentVariance) throws IndoorException {
        if (txPowerVariance == null && rxPowerVariance == null && pathLossExponentVariance == null) {
            return null;
        }

        double[] mean = new double[]{ txPower, rxPower, pathLossExponent };
        Matrix covariance = Matrix.diagonal(new double[]{
                txPowerVariance != null ? txPowerVariance : 0.0,
                rxPowerVariance != null ? rxPowerVariance : 0.0,
                pathLossExponentVariance != null ? pathLossExponentVariance : 0.0
        });

        try {
            return MultivariateNormalDist.propagate(new MultivariateNormalDist.JacobianEvaluator() {
                @Override
                public void evaluate(double[] x, double[] y, Matrix jacobian) {
                    double k = RssiRadioSourceEstimator.SPEED_OF_LIGHT / (4.0 * Math.PI * frequency);
                    double kdB = 10.0 * Math.log10(k);

                    //received power in dBm's follows the equation:
                    //rxPower = pathLossExponent * kdB + txPower - 5.0 * pathLossExponent * logSqrDistance

                    //hence, distance follows the following expression:
                    //d = 10.0^((pathLossExponent * kdB + txPower - rxPower)/(10.0 * pathLossExponent))
                    //where kdB is a constant having the following expression:
                    //kdB = 10.0 * log(c / (4 * pi * f)),
                    //where c is the speed of light and f is the frequency.

                    double logSqrDistance = (pathLossExponent * kdB + txPower - rxPower) / (5.0 * pathLossExponent);

                    //where logSqrDistance = Math.log10(sqrDistance)
                    //and sqrDistance = distance * distance, hence
                    //logSqrDistance = Math.log10(distance * distance) = 2 * Math.log10(distance)

                    y[0] = Math.pow(10.0,logSqrDistance / 2.0);



                    //compute gradient (is a jacobian having 1 row and 3 columns)

                    //derivative of distance respect to transmitted power is:

                    //if the only unknown is the transmitted power (x = txPower), then:
                    //d = f(x) = 10.0^((pathLossExponent * kdB + x - rxPower)/(10.0 * pathLossExponent))
                    //and the derivative is
                    //f'(x) = ln(10)/(10.0 * pathLossExponent)*10.0^((pathLossExponent * kdB + x - rxPower)/(10.0 * pathLossExponent))
                    double tenPathLossExponent = 10.0 * pathLossExponent;
                    double tenPowered = Math.pow(10.0,
                            (pathLossExponent * kdB + txPower - rxPower) / tenPathLossExponent);
                    double derivativeTxPower = Math.log(10.0) / tenPathLossExponent * tenPowered;

                    //derivative of distance respect to received power is:

                    //if the only unknown is the received power (x = rxPower), then:
                    //d = f(x) = 10.0^((pathLossExponent * kdB + txPower - x)/(10.0 * pathLossExponent))
                    //and the derivative is
                    //f'(x) = -ln(10)/(10.0*pathLossExponent)*10^((pathLossExponent * kdB + txPower - x)/(10.0 * pathLossExponent))
                    double derivativeRxPower = -Math.log(10.0) / tenPathLossExponent * tenPowered;


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

                    double g = (pathLossExponent * kdB + txPower - rxPower) / (10.0 * pathLossExponent);
                    double derivativeG = (kdB * 10.0 * pathLossExponent -
                            10.0 * (pathLossExponent * kdB + txPower - rxPower)) /
                            Math.pow(10.0 * pathLossExponent, 2.0);

                    double derivativePathLossExponent = Math.log(10.0) * derivativeG * Math.pow(10.0, g);

                    jacobian.setElementAtIndex(0, derivativeTxPower);
                    jacobian.setElementAtIndex(1, derivativeRxPower);
                    jacobian.setElementAtIndex(2, derivativePathLossExponent);
                }

                @Override
                public int getNumberOfVariables() {
                    return 1;
                }
            }, mean, covariance);
        } catch (AlgebraException | StatisticsException e) {
            throw new IndoorException(e);
        }
    }

    /**
     * Propagates provided variances (fingerprint rssi variance, path-loss exponent variance,
     * fingerprint position covariance and radio source position covariance) into
     * rssi variance by considering the 2D 1st order Taylor expression of received power.
     * Notice that any unknown variance is assumed to be zero.
     * @param fingerprintRssi closest located fingerprint reading RSSI expressed in dBm's.
     * @param pathLossExponent path-loss exponent.
     * @param fingerprintPosition position of closest fingerprint.
     * @param radioSourcePosition radio source position associated to fingerprint reading.
     * @param estimatedPosition  position to be estimated. Usually this is equal to the
     *                           initial position used by a non linear algorithm.
     * @param fingerprintRssiVariance variance of fingerprint RSSI or null if unknown.
     * @param pathLossExponentVariance variance of path-loss exponent or null if unknown.
     * @param fingerprintPositionCovariance covariance of fingerprint position or null if
     *                                      unknown.
     * @param radioSourcePositionCovariance covariance of radio source position or null
     *                                      if unknown.
     * @param estimatedPositionCovariance  covariance of position to be estimated or null
     *                                     if unknown. (This is usually unknown).
     * @return a normal distribution containing expected received RSSI value and its variance.
     * @throws IndoorException if something fails.
     */
    public static MultivariateNormalDist propagateVariancesToRssiVarianceFirstOrderNonLinear2D(
            final double fingerprintRssi, final double pathLossExponent,
            Point2D fingerprintPosition, Point2D radioSourcePosition,
            Point2D estimatedPosition,
            Double fingerprintRssiVariance,
            Double pathLossExponentVariance,
            Matrix fingerprintPositionCovariance,
            Matrix radioSourcePositionCovariance,
            Matrix estimatedPositionCovariance) throws IndoorException {

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

        double[] mean = new double[] {
                fingerprintRssi, pathLossExponent, x1, y1, xa, ya, xi, yi
        };
        Matrix covariance = Matrix.diagonal(new double[]{
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
                public void evaluate(double[] x, double[] y, Matrix jacobian) {

                    //Pr(pi) = Pr(p1)
                    //  - 10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
                    //  - 10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
                    //where d1a^2 = (x1 - xa)^2 + (y1 - ya)^2

                    //Hence:
                    //Pr(pi) = Pr(p1) -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/
                    //      (ln(10)*((x1 - xa)^2 + (y1 - ya)^2))

                    double diffX1a = x1 - xa;
                    double diffY1a = y1 - ya;

                    double diffXi1 = xi - x1;
                    double diffYi1 = yi - y1;

                    double diffX1a2 = diffX1a * diffX1a;
                    double diffY1a2 = diffY1a * diffY1a;

                    double d1a2 = diffX1a2 + diffY1a2;
                    double d1a4 = d1a2 * d1a2;

                    double ln10 = Math.log(10.0);
                    double crossDiff = diffX1a * diffXi1 + diffY1a * diffYi1;

                    y[0] = fingerprintRssi
                            - 10.0 * pathLossExponent * crossDiff / (ln10 * d1a2);

                    //compute gradient (is a jacobian having 1 row and 8 columns)


                    //derivative of rssi respect to fingerprint rssi
                    double derivativeFingerprintRssi = 1.0;

                    //derivative of rssi respect to path-loss exponent

                    //diff(Pr(pi))/diff(n) = -10*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
                    //  -10*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
                    double derivativePathLossExponent = - 10.0 * crossDiff /
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
                    double tmpX = 2.0 * crossDiff * diffX1a;
                    double derivativeX1 = -10.0 * pathLossExponent / ln10 * ((-2.0 * x1 + xi + xa) * d1a2
                            - tmpX) / d1a4;

                    //derivative of rssi respect to y1

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/diff(y1) =
                    //  diff(y1*yi -ya*yi -y1^2 + ya*y1)/diff(y1) =
                    //  diff(-y1^2 + (yi + ya)*y1 - ya*yi)/diff(y1) =
                    //  -2*y1 + yi + ya

                    //diff(Pr(pi))/diff(y1) = -10*n/ln(10)*((-2*y1 + yi + ya)*((x1 - xa)^2 + (y1 - ya)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2)^2
                    //diff(Pr(pi))/diff(y1) = -10*n/ln(10)*((-2*y1 + yi + ya)*d1a^2 - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*2*(y1 - ya))/d1a^4
                    double tmpY = 2.0 * crossDiff * diffY1a;
                    double derivativeY1 = -10.0 * pathLossExponent / ln10 * ((-2.0 * y1 + yi + ya) * d1a2
                            - tmpY) / d1a4;


                    //derivative of rssi respect to xa

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/diff(xa) =
                    //  diff(x1*xi -xa*xi -x1^2 + xa*x1)/diff(xa) =
                    //  x1 - xi

                    //diff(Pr(pi))/diff(xa) = -10*n/ln(10)*((x1 - xi)*((x1 - xa)^2 + (y1 - ya)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*-2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2)^2
                    //diff(Pr(pi))/diff(xa) = -10*n/ln(10)*(-(xi - x1)*d1a^2 + ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*2*(x1 - xa))/d1a^4
                    double derivativeXa = -10.0 * pathLossExponent / ln10 * (-diffXi1 * d1a2
                            + tmpX) / d1a4;


                    //derivative of rssi respect to ya

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/diff(ya) =
                    //  diff(y1*yi -y1^2 -ya*yi + ya*y1)/diff(ya) =
                    //  y1 - yi

                    //diff(Pr(pi))/diff(ya) = -10*n/ln(10)*((y1 - yi)*((x1 - xa)^2 + (y1 - ya)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*-2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2)^2
                    //diff(Pr(pi))/diff(ya) = -10*n/ln(10)*(-(yi - y1)*d1a^2 + ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*2*(y1 - ya))/d1a^4
                    double derivativeYa = -10.0 * pathLossExponent / ln10 *(-diffYi1 * d1a2
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
                    double derivativeXi = -10.0 * pathLossExponent * diffX1a / (ln10 * d1a2);


                    //derivative of rssi respect to yi

                    //Pr(pi) = Pr(p1) -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/
                    //      (ln(10)*((x1 - xa)^2 + (y1 - ya)^2))

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))/diff(yi) =
                    //  diff(y1*yi -ya*yi -y1^2 + ya*y1)/diff(yi) =
                    //  y1 - ya

                    //diff(Pr(pi))/diff(yi) = -10*n/ln(10)*((y1 - ya)*((x1 - xa)^2 + (y1 - ya)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1))*0)/((x1 - xa)^2 + (y1 - ya)^2)^2
                    //diff(Pr(pi))/diff(yi) = -10*n/ln(10)*((y1 - ya)*d1a^2)/d1a^4
                    //diff(Pr(pi))/diff(yi) = -10*n*(y1 - ya)/(ln(10)*d1a^2)
                    double derivativeYi = -10.0 * pathLossExponent * diffY1a / (ln10 * d1a2);

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
        } catch (AlgebraException | StatisticsException e) {
            throw new IndoorException(e);
        }
    }

    /**
     * Propagates provided variances (fingerprint rssi variance, path-loss exponent variance,
     * fingerprint position covariance and radio source position covariance) into
     * rssi variance by considering the 3D 1st order Taylor expression of received power.
     * Notice that any unknown variance is assumed to be zero.
     * @param fingerprintRssi closest located fingerprint reading RSSI expressed in dBm's.
     * @param pathLossExponent path-loss exponent.
     * @param fingerprintPosition position of closest fingerprint.
     * @param radioSourcePosition radio source position associated to fingerprint reading.
     * @param estimatedPosition  position to be estimated. Usually this is equal to the
     *                           initial position used by a non linear algorithm.
     * @param fingerprintRssiVariance variance of fingerprint RSSI or null if unknown.
     * @param pathLossExponentVariance variance of path-loss exponent or null if unknown.
     * @param fingerprintPositionCovariance covariance of fingerprint position or null if
     *                                      unknown.
     * @param radioSourcePositionCovariance covariance of radio source position or null
     *                                      if unknown.
     * @param estimatedPositionCovariance  covariance of position to be estimated or null
     *                                     if unknown. (This is usually unknown).
     * @return a normal distribution containing expected received RSSI value and its variance.
     * @throws IndoorException if something fails.
     */
    public static MultivariateNormalDist propagateVariancesToRssiVarianceFirstOrderNonLinear3D(
            final double fingerprintRssi, final double pathLossExponent,
            Point3D fingerprintPosition, Point3D radioSourcePosition,
            Point3D estimatedPosition,
            Double fingerprintRssiVariance,
            Double pathLossExponentVariance,
            Matrix fingerprintPositionCovariance,
            Matrix radioSourcePositionCovariance,
            Matrix estimatedPositionCovariance) throws IndoorException {

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

        double[] mean = new double[] {
                fingerprintRssi, pathLossExponent, x1, y1, z1, xa, ya, za, xi, yi, zi
        };
        Matrix covariance = Matrix.diagonal(new double[]{
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
                public void evaluate(double[] x, double[] y, Matrix jacobian) {

                    //Pr(pi) = Pr(p1)
                    //  - 10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
                    //  - 10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
                    //  - 10*n*(z1 - za)/(ln(10)*d1a^2)*(zi - z1)
                    //where d1a^2 = (x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2

                    //Hence:
                    //Pr(pi) = Pr(p1) -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/
                    //      (ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))

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
                    double d1a4 = d1a2 * d1a2;

                    double ln10 = Math.log(10.0);
                    double crossDiff = diffX1a * diffXi1 + diffY1a * diffYi1 + diffZ1a * diffZi1;

                    y[0] = fingerprintRssi
                            - 10.0 * pathLossExponent * crossDiff / (ln10 * d1a2);

                    //compute gradient (is a jacobian having 1 row and 11 columns)


                    //derivative of rssi respect to fingerprint rssi
                    double derivativeFingerprintRssi = 1.0;

                    //derivative of rssi respect to path-loss exponent

                    //diff(Pr(pi))/diff(n) = -10*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
                    //  -10*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
                    double derivativePathLossExponent = - 10.0 * crossDiff /
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
                    double tmpX = 2.0 * crossDiff * diffX1a;
                    double derivativeX1 = -10.0 * pathLossExponent / ln10 * ((-2.0 * x1 + xi + xa) * d1a2
                            - tmpX) / d1a4;


                    //derivative of rssi respect to y1

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(y1) =
                    //  diff(y1*yi -ya*yi -y1^2 + ya*y1)/diff(y1) =
                    //  diff(-y1^2 + (yi + ya)*y1 - ya*yi)/diff(y1) =
                    //  -2*y1 + yi + ya

                    //diff(Pr(pi))/diff(y1) = -10*n/ln(10)*((-2*y1 + yi + ya)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //diff(Pr(pi))/diff(y1) = -10*n/ln(10)*((-2*y1 + yi + ya)*d1a^2 - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(y1 - ya))/d1a^4
                    double tmpY = 2.0 * crossDiff * diffY1a;
                    double derivativeY1 = -10.0 * pathLossExponent / ln10 * ((-2.0 * y1 + yi + ya) * d1a2
                            - tmpY) / d1a4;


                    //derivative of rssi respect to z1

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(z1) =
                    //  diff(z1*zi -za*zi -z1^2 + za*z1)/diff(z1) =
                    //  diff(-z1^2 + (zi + za)*z1 - za*zi)/diff(z1) =
                    //  -2*z1 + zi + za

                    //diff(Pr(pi))/diff(z1) = -10*n/ln(10)*((-2*z1 + zi + za)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //diff(Pr(pi))/diff(z1) = -10*n/ln(10)*((-2*z1 + zi + za)*d1a^2 - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(z1 - za))/d1a^4
                    double tmpZ = 2.0 * crossDiff * diffZ1a;
                    double derivativeZ1 = -10.0 * pathLossExponent / ln10 * ((-2.0 * z1 + z1 + za) * d1a2
                            - tmpZ) / d1a4;


                    //derivative of rssi respect to xa

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(xa) =
                    //  diff(x1*xi -xa*xi -x1^2 + xa*x1)/diff(xa) =
                    //  x1 - xi

                    //diff(Pr(pi))/diff(xa) = -10*n/ln(10)*((x1 - xi)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*-2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //diff(Pr(pi))/diff(xa) = -10*n/ln(10)*(-(xi - x1)*d1a^2 + ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(x1 - xa))/d1a^4
                    double derivativeXa = -10.0 * pathLossExponent / ln10 * (-diffXi1 * d1a2
                            + tmpX) / d1a4;


                    //derivative of rssi respect to ya

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(ya) =
                    //  diff(y1*yi -y1^2 -ya*yi + ya*y1)/diff(ya) =
                    //  y1 - yi

                    //diff(Pr(pi))/diff(ya) = -10*n/ln(10)*((y1 - yi)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*-2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //diff(Pr(pi))/diff(ya) = -10*n/ln(10)*(-(yi - y1)*d1a^2 + ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(y1 - ya))/d1a^4
                    double derivativeYa = -10.0 * pathLossExponent / ln10 *(-diffYi1 * d1a2
                            + tmpY) / d1a4;


                    //derivative of rssi respect to za

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(za) =
                    //  diff(z1*zi -z1^2 -za*zi + za*z1)/diff(za) =
                    //  z1 - zi

                    //diff(Pr(pi))/diff(za) = -10*n/ln(10)*((z1 - zi)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*-2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //diff(Pr(pi))/diff(za) = -10*n/ln(10)*(-(zi - z1)*d1a^2 + ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*2*(z1 - za))/d1a^4
                    double derivativeZa = -10.0 * pathLossExponent / ln10 *(- diffZi1 * d1a2 +
                            + tmpZ) / d1a4;


                    //derivative of rssi respect to xi

                    //Pr(pi) = Pr(p1) -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/
                    //      (ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(xi) =
                    //  diff(x1*xi -xa*xi -x1^2 + xa*x1)/diff(xi) =
                    //  x1 - xa

                    //diff(Pr(pi))/diff(xi) = -10*n/ln(10)*((x1 - xa)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //diff(Pr(pi))/diff(xi) = -10*n/ln(10)*((x1 - xa)*d1a^2)/d1a^4
                    //diff(Pr(pi))/diff(xi) = -10*n*(x1 - xa)/(ln(10)*d1a^2)
                    double derivativeXi = -10.0 * pathLossExponent * diffX1a / (ln10 * d1a2);


                    //derivative of rssi respect to yi

                    //Pr(pi) = Pr(p1) -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/
                    //      (ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(yi) =
                    //  diff(y1*yi -ya*yi -y1^2 + ya*y1)/diff(yi) =
                    //  y1 - ya

                    //diff(Pr(pi))/diff(yi) = -10*n/ln(10)*((y1 - ya)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //diff(Pr(pi))/diff(yi) = -10*n/ln(10)*((y1 - ya)*d1a^2)/d1a^4
                    //diff(Pr(pi))/diff(yi) = -10*n*(y1 - ya)/(ln(10)*d1a^2)
                    double derivativeYi = -10.0 * pathLossExponent * diffY1a / (ln10 * d1a2);


                    //derivative of rssi respect to zi

                    //Pr(pi) = Pr(p1) -10*n*((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/
                    //      (ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))

                    //diff((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))/diff(zi) =
                    //  diff(z1*zi -za*zi -z1^2 + za*z1)/diff(zi) =
                    //  z1 - za

                    //diff(Pr(pi))/diff(zi) = -10*n/ln(10)*((z1 - za)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - ((x1 - xa)*(xi - x1) + (y1 - ya)*(yi - y1) + (z1 - za)*(zi - z1))*0)/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2
                    //diff(Pr(pi))/diff(zi) = -10*n/ln(10)*((z1 - za)*d1a^2)/d1a^4
                    //diff(Pr(pi))/diff(zi) = -10*n*(z1 - za)/(ln(10*d1a^2)
                    double derivativeZi = -10.0 * pathLossExponent * diffZ1a / (ln10 * d1a2);

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
        } catch (AlgebraException | StatisticsException e) {
            throw new IndoorException(e);
        }
    }

    /**
     * Propagates provided variances (fingerprint rssi variance, path-loss exponent variance,
     * fingerprint position covariance and radio source position covariance) into
     * rssi variance by considering the 2D 2nd order Taylor expression of received power.
     * Notice that any unknown variance is assumed to be zero.
     * @param fingerprintRssi closest located fingerprint reading RSSI expressed in dBm's.
     * @param pathLossExponent path-loss exponent.
     * @param fingerprintPosition position of closest fingerprint.
     * @param radioSourcePosition radio source position associated to fingerprint reading.
     * @param estimatedPosition  position to be estimated. Usually this is equal to the
     *                           initial position used by a non linear algorithm.
     * @param fingerprintRssiVariance variance of fingerprint RSSI or null if unknown.
     * @param pathLossExponentVariance variance of path-loss exponent or null if unknown.
     * @param fingerprintPositionCovariance covariance of fingerprint position or null if
     *                                      unknown.
     * @param radioSourcePositionCovariance covariance of radio source position or null
     *                                      if unknown.
     * @param estimatedPositionCovariance  covariance of position to be estimated or null
     *                                     if unknown. (This is usually unknown).
     * @return a normal distribution containing expected received RSSI value and its variance.
     * @throws IndoorException if something fails.
     */
    public static MultivariateNormalDist propagateVariancesToRssiVarianceSecondOrderNonLinear2D(
            final double fingerprintRssi, final double pathLossExponent,
            Point2D fingerprintPosition, Point2D radioSourcePosition,
            Point2D estimatedPosition, Double fingerprintRssiVariance,
            Double pathLossExponentVariance,
            Matrix fingerprintPositionCovariance,
            Matrix radioSourcePositionCovariance,
            Matrix estimatedPositionCovariance) throws IndoorException {

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

        double[] mean = new double[] {
                fingerprintRssi, pathLossExponent, x1, y1, xa, ya, xi, yi
        };
        Matrix covariance = Matrix.diagonal(new double[]{
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
                public void evaluate(double[] x, double[] y, Matrix jacobian) {

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
                    double d1a8 = d1a4 * d1a4;

                    double ln10 = Math.log(10.0);

                    y[0] = fingerprintRssi
                            - 10.0 * pathLossExponent * diffX1a / (ln10 * d1a2) * diffXi1
                            - 10.0 * pathLossExponent * diffY1a / (ln10 * d1a2) * diffYi1
                            - 5.0 * pathLossExponent * (-diffX1a2 + diffY1a2) / (ln10 * d1a4) * diffXi12
                            - 5.0 * pathLossExponent * (diffX1a2 - diffY1a2) / (ln10 * d1a4) * diffYi12
                            + 20.0 * pathLossExponent * diffX1a * diffY1a / (ln10 * d1a4) * diffXi1 * diffYi1;

                    //compute gradient (is a jacobian having 1 row and 8 columns)


                    //derivative of rssi respect to fingerprint rssi
                    double derivativeFingerprintRssi = 1.0;

                    //derivative of rssi respect to path-loss exponent

                    //diff(Pr(pi))/diff(n) = -10*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))*(xi - x1)
                    //  -10*(y1 - ya)/(ln(10)*(x1 - xa)^2 + (y1 - ya)^2)*(yi - y1)
                    //  -5*((y1 - ya)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(xi - x1)^2
                    //  -5*((x1 - xa)^2 - (y1 - ya)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(yi - y1)^2
                    //  +20*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2)^2))*(xi - x1)*(yi - y1)
                    double derivativePathLossExponent = - 10.0 * diffX1a / (ln10 * d1a2) * diffXi1
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
                    double crossDiff = diffX1a * diffXi1 + diffY1a * diffYi1;
                    double tmpX = crossDiff * 2.0 * diffX1a;
                    double tmpX2 = (diffX1a2 - diffY1a2) * (diffXi12 - diffYi12) * 4.0 * d1a2 * diffX1a;
                    double tmpX3 = diffX1a * diffY1a * diffXi1 * diffYi1 * 4.0 * d1a2 * diffX1a;
                    double derivativeX1 = -10.0 * pathLossExponent / ln10 * ((-2.0 * x1 + xi + xa) * d1a2 - tmpX) / d1a4
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
                    double tmpY = crossDiff * 2.0 * diffY1a;
                    double tmpY2 = (diffX1a2 - diffY1a2) * (diffXi12 - diffYi12) * 4.0 * d1a2 * diffY1a;
                    double tmpY3 = diffX1a * diffY1a * diffXi1 * diffYi1 * 4.0 * d1a2 * diffY1a;
                    double derivativeY1 = -10.0 * pathLossExponent / ln10 * ((-2.0 * y1 + yi + ya) * d1a2 - tmpY) / d1a4
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
                    double derivativeXa = -10.0 * pathLossExponent / ln10 * (-diffXi1 * d1a2 + tmpX) / d1a4
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
                    double derivativeYa = -10.0 * pathLossExponent / ln10 * (-diffYi1 * d1a2 + tmpY) / d1a4
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
                    double derivativeXi = -10.0 * pathLossExponent / ln10 * diffX1a / d1a2
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
                    double derivativeYi = -10.0 * pathLossExponent / ln10 * diffY1a / d1a2
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
        } catch (AlgebraException | StatisticsException e) {
            throw new IndoorException(e);
        }
    }
}
