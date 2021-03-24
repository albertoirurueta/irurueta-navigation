/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.frames.converters;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.EuclideanTransformation3D;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.RotationException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertTrue;

public class FrameToLocalTangentPlaneTransformationConverterTest {

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -50.0;
    private static final double MAX_HEIGHT_METERS = 50.0;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_POS_CHANGE_METERS = -10.0;
    private static final double MAX_POS_CHANGE_METERS = 10.0;

    private static final double MIN_ANGLE_CHANGE_DEGREES = -5.0;
    private static final double MAX_ANGLE_CHANGE_DEGREES = 5.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 10;

    @Test
    public void convert1() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException {
        final FrameToLocalTangentPlaneTransformationConverter converter =
                new FrameToLocalTangentPlaneTransformationConverter();

        // create reference and current frame
        final NEDFrame referenceNedFrame = createFrame();
        final ECEFFrame referenceEcefFrame = convertFrame(referenceNedFrame);

        final double[] translation = createTranslationChange();
        final Rotation3D rotation = createRotationChange();

        final ECEFFrame currentEcefFrame = transformFrame(
                referenceEcefFrame, translation, rotation);

        // convert
        final double[] translationResult = new double[3];
        final Quaternion rotationResult = new Quaternion();
        converter.convert(currentEcefFrame, referenceEcefFrame,
                translationResult, rotationResult);

        // check
        assertArrayEquals(translation, translationResult, ABSOLUTE_ERROR);
        assertTrue(rotationResult.equals(rotation, ABSOLUTE_ERROR));
    }

    @Test
    public void convert2() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final FrameToLocalTangentPlaneTransformationConverter converter =
                    new FrameToLocalTangentPlaneTransformationConverter();

            // create reference and current frame
            final NEDFrame referenceNedFrame = createFrame();
            final ECEFFrame referenceEcefFrame = convertFrame(referenceNedFrame);

            final double[] translation = createTranslationChange();
            final Rotation3D rotation = createRotationChange();

            final ECEFFrame currentEcefFrame = transformFrame(
                    referenceEcefFrame, translation, rotation);

            // convert
            final EuclideanTransformation3D result = new EuclideanTransformation3D();
            try {
                converter.convert(currentEcefFrame, referenceEcefFrame, result);
            } catch (final InvalidRotationMatrixException e) {
                continue;
            }

            // check
            assertArrayEquals(translation, result.getTranslation(), ABSOLUTE_ERROR);
            assertTrue(result.getRotation().equals(rotation, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void convert3() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException {
        final FrameToLocalTangentPlaneTransformationConverter converter =
                new FrameToLocalTangentPlaneTransformationConverter();

        // create reference and current frame
        final NEDFrame referenceNedFrame = createFrame();
        final ECEFFrame referenceEcefFrame = convertFrame(referenceNedFrame);

        final double[] translation = createTranslationChange();
        final Rotation3D rotation = createRotationChange();

        final ECEFFrame currentEcefFrame = transformFrame(
                referenceEcefFrame, translation, rotation);
        final NEDFrame currentNedFrame = convertFrame(currentEcefFrame);

        // convert
        final double[] translationResult = new double[3];
        final Quaternion rotationResult = new Quaternion();
        converter.convert(currentNedFrame, referenceEcefFrame,
                translationResult, rotationResult);

        // check
        assertArrayEquals(translation, translationResult, ABSOLUTE_ERROR);
        assertTrue(rotationResult.equals(rotation, ABSOLUTE_ERROR));
    }

    @Test
    public void convert4() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException {
        final FrameToLocalTangentPlaneTransformationConverter converter =
                new FrameToLocalTangentPlaneTransformationConverter();

        // create reference and current frame
        final NEDFrame referenceNedFrame = createFrame();
        final ECEFFrame referenceEcefFrame = convertFrame(referenceNedFrame);

        final double[] translation = createTranslationChange();
        final Rotation3D rotation = createRotationChange();

        final ECEFFrame currentEcefFrame = transformFrame(
                referenceEcefFrame, translation, rotation);
        final NEDFrame currentNedFrame = convertFrame(currentEcefFrame);

        // convert
        final EuclideanTransformation3D result = new EuclideanTransformation3D();
        converter.convert(currentNedFrame, referenceEcefFrame, result);

        // check
        assertArrayEquals(translation, result.getTranslation(), ABSOLUTE_ERROR);
        assertTrue(result.getRotation().equals(rotation, ABSOLUTE_ERROR));
    }

    @Test
    public void convert5() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException {
        final FrameToLocalTangentPlaneTransformationConverter converter =
                new FrameToLocalTangentPlaneTransformationConverter();

        // create reference and current frame
        final NEDFrame referenceNedFrame = createFrame();
        final ECEFFrame referenceEcefFrame = convertFrame(referenceNedFrame);

        final double[] translation = createTranslationChange();
        final Rotation3D rotation = createRotationChange();

        final ECEFFrame currentEcefFrame = transformFrame(
                referenceEcefFrame, translation, rotation);

        // convert
        final double[] translationResult = new double[3];
        final Quaternion rotationResult = new Quaternion();
        converter.convert(currentEcefFrame, referenceNedFrame,
                translationResult, rotationResult);

        // check
        assertArrayEquals(translation, translationResult, ABSOLUTE_ERROR);
        assertTrue(rotationResult.equals(rotation, ABSOLUTE_ERROR));
    }

    @Test
    public void convert6() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException {
        final FrameToLocalTangentPlaneTransformationConverter converter =
                new FrameToLocalTangentPlaneTransformationConverter();

        // create reference and current frame
        final NEDFrame referenceNedFrame = createFrame();
        final ECEFFrame referenceEcefFrame = convertFrame(referenceNedFrame);

        final double[] translation = createTranslationChange();
        final Rotation3D rotation = createRotationChange();

        final ECEFFrame currentEcefFrame = transformFrame(
                referenceEcefFrame, translation, rotation);

        // convert
        final EuclideanTransformation3D result = new EuclideanTransformation3D();
        converter.convert(currentEcefFrame, referenceNedFrame, result);

        // check
        assertArrayEquals(translation, result.getTranslation(), ABSOLUTE_ERROR);
        assertTrue(result.getRotation().equals(rotation, ABSOLUTE_ERROR));
    }

    @Test
    public void convert7() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final FrameToLocalTangentPlaneTransformationConverter converter =
                    new FrameToLocalTangentPlaneTransformationConverter();

            // create reference and current frame
            final NEDFrame referenceNedFrame = createFrame();
            final ECEFFrame referenceEcefFrame = convertFrame(referenceNedFrame);

            final double[] translation = createTranslationChange();
            final Rotation3D rotation = createRotationChange();

            final ECEFFrame currentEcefFrame;
            try {
                currentEcefFrame = transformFrame(
                        referenceEcefFrame, translation, rotation);
            } catch (final InvalidRotationMatrixException e) {
                continue;
            }

            final NEDFrame currentNedFrame = convertFrame(currentEcefFrame);

            // convert
            final double[] translationResult = new double[3];
            final Quaternion rotationResult = new Quaternion();
            converter.convert(currentNedFrame, referenceNedFrame,
                    translationResult, rotationResult);

            // check
            assertArrayEquals(translation, translationResult, ABSOLUTE_ERROR);
            assertTrue(rotationResult.equals(rotation, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void convert8() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException {
        final FrameToLocalTangentPlaneTransformationConverter converter =
                new FrameToLocalTangentPlaneTransformationConverter();

        // create reference and current frame
        final NEDFrame referenceNedFrame = createFrame();
        final ECEFFrame referenceEcefFrame = convertFrame(referenceNedFrame);

        final double[] translation = createTranslationChange();
        final Rotation3D rotation = createRotationChange();

        final ECEFFrame currentEcefFrame = transformFrame(
                referenceEcefFrame, translation, rotation);
        final NEDFrame currentNedFrame = convertFrame(currentEcefFrame);

        // convert
        final EuclideanTransformation3D result = new EuclideanTransformation3D();
        converter.convert(currentNedFrame, referenceNedFrame, result);

        // check
        assertArrayEquals(translation, result.getTranslation(), ABSOLUTE_ERROR);
        assertTrue(result.getRotation().equals(rotation, ABSOLUTE_ERROR));
    }

    @Test
    public void convertAndReturn1()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException {
        final FrameToLocalTangentPlaneTransformationConverter converter =
                new FrameToLocalTangentPlaneTransformationConverter();

        // create reference and current frame
        final NEDFrame referenceNedFrame = createFrame();
        final ECEFFrame referenceEcefFrame = convertFrame(referenceNedFrame);

        final double[] translation = createTranslationChange();
        final Rotation3D rotation = createRotationChange();

        final ECEFFrame currentEcefFrame = transformFrame(
                referenceEcefFrame, translation, rotation);

        // convert
        final EuclideanTransformation3D result = converter.convertAndReturn(
                currentEcefFrame, referenceEcefFrame);

        // check
        assertArrayEquals(translation, result.getTranslation(), ABSOLUTE_ERROR);
        assertTrue(result.getRotation().equals(rotation, ABSOLUTE_ERROR));
    }

    @Test
    public void convertAndReturn2()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException {
        final FrameToLocalTangentPlaneTransformationConverter converter =
                new FrameToLocalTangentPlaneTransformationConverter();

        // create reference and current frame
        final NEDFrame referenceNedFrame = createFrame();
        final ECEFFrame referenceEcefFrame = convertFrame(referenceNedFrame);

        final double[] translation = createTranslationChange();
        final Rotation3D rotation = createRotationChange();

        final ECEFFrame currentEcefFrame = transformFrame(
                referenceEcefFrame, translation, rotation);
        final NEDFrame currentNedFrame = convertFrame(currentEcefFrame);

        // convert
        final EuclideanTransformation3D result = converter.convertAndReturn(
                currentNedFrame, referenceEcefFrame);

        // check
        assertArrayEquals(translation, result.getTranslation(), ABSOLUTE_ERROR);
        assertTrue(result.getRotation().equals(rotation, ABSOLUTE_ERROR));
    }

    @Test
    public void convertAndReturn3() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException {
        final FrameToLocalTangentPlaneTransformationConverter converter =
                new FrameToLocalTangentPlaneTransformationConverter();

        // create reference and current frame
        final NEDFrame referenceNedFrame = createFrame();
        final ECEFFrame referenceEcefFrame = convertFrame(referenceNedFrame);

        final double[] translation = createTranslationChange();
        final Rotation3D rotation = createRotationChange();

        final ECEFFrame currentEcefFrame = transformFrame(
                referenceEcefFrame, translation, rotation);

        // convert
        final EuclideanTransformation3D result = converter.convertAndReturn(
                currentEcefFrame, referenceNedFrame);

        // check
        assertArrayEquals(translation, result.getTranslation(), ABSOLUTE_ERROR);
        assertTrue(result.getRotation().equals(rotation, ABSOLUTE_ERROR));
    }

    @Test
    public void convertAndReturn4() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final FrameToLocalTangentPlaneTransformationConverter converter =
                    new FrameToLocalTangentPlaneTransformationConverter();

            // create reference and current frame
            final NEDFrame referenceNedFrame = createFrame();
            final ECEFFrame referenceEcefFrame = convertFrame(referenceNedFrame);

            final double[] translation = createTranslationChange();
            final Rotation3D rotation = createRotationChange();

            final ECEFFrame currentEcefFrame = transformFrame(
                    referenceEcefFrame, translation, rotation);
            final NEDFrame currentNedFrame = convertFrame(currentEcefFrame);

            // convert
            final EuclideanTransformation3D result;
            try {
                result = converter.convertAndReturn(
                        currentNedFrame, referenceNedFrame);
            } catch (final InvalidRotationMatrixException e) {
                continue;
            }

            // check
            assertArrayEquals(translation, result.getTranslation(), ABSOLUTE_ERROR);
            assertTrue(result.getRotation().equals(rotation, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private NEDFrame createFrame() throws InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final CoordinateTransformation nedC = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        return new NEDFrame(nedPosition, nedC);
    }

    private ECEFFrame convertFrame(final NEDFrame nedFrame) {
        return NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
    }

    private NEDFrame convertFrame(final ECEFFrame ecefFrame) {
        return ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(ecefFrame);
    }

    private double[] createTranslationChange() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double diffX = randomizer.nextDouble(
                MIN_POS_CHANGE_METERS, MAX_POS_CHANGE_METERS);
        final double diffY = randomizer.nextDouble(
                MIN_POS_CHANGE_METERS, MAX_POS_CHANGE_METERS);
        final double diffZ = randomizer.nextDouble(
                MIN_POS_CHANGE_METERS, MAX_POS_CHANGE_METERS);
        return new double[]{ diffX, diffY, diffZ };
    }

    private Rotation3D createRotationChange() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = Math.toRadians(randomizer.nextDouble(
                MIN_ANGLE_CHANGE_DEGREES, MAX_ANGLE_CHANGE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(
                MIN_ANGLE_CHANGE_DEGREES, MAX_ANGLE_CHANGE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(
                MIN_ANGLE_CHANGE_DEGREES, MAX_ANGLE_CHANGE_DEGREES));
        return new Quaternion(roll, pitch, yaw);
    }

    private ECEFFrame transformFrame(final ECEFFrame inputFrame,
                                     final double[] translation,
                                     final Rotation3D rotation)
            throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {
        final double x = inputFrame.getX() + translation[0];
        final double y = inputFrame.getY() + translation[1];
        final double z = inputFrame.getZ() + translation[2];

        final Rotation3D inputR = inputFrame
                .getCoordinateTransformation().asRotation();
        final Rotation3D r = rotation.combineAndReturnNew(inputR);

        final Matrix rotationMatrix = r.asInhomogeneousMatrix();
        final CoordinateTransformation c = new CoordinateTransformation(
                rotationMatrix, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        return new ECEFFrame(x, y, z, c);
    }
}
