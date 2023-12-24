package com.irurueta.navigation.frames;

import com.irurueta.navigation.SerializationHelper;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import org.junit.Test;

import java.io.IOException;
import java.util.Random;

import static org.junit.Assert.*;

public class NEDPositionTest {

    private static final double THRESHOLD = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    @Test
    public void testConstructor() {

        // test empty constructor
        NEDPosition position = new NEDPosition();

        // check
        assertEquals(0.0, position.getLatitude(), 0.0);
        assertEquals(0.0, position.getLongitude(), 0.0);
        assertEquals(0.0, position.getHeight(), 0.0);

        assertEquals(0.0, position.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(0.0, position.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(0.0, position.getHeightDistance().getValue().doubleValue(), 0.0);


        // test constructor with coordinates
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        position = new NEDPosition(latitude, longitude, height);

        // check
        assertEquals(latitude, position.getLatitude(), 0.0);
        assertEquals(longitude, position.getLongitude(), 0.0);
        assertEquals(height, position.getHeight(), 0.0);

        assertEquals(latitude, position.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, position.getLatitudeAngle().getUnit());

        assertEquals(longitude, position.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, position.getLongitudeAngle().getUnit());

        assertEquals(height, position.getHeightDistance().getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, position.getHeightDistance().getUnit());


        // test constructor with angles and distance coordinates
        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);

        position = new NEDPosition(latitudeAngle, longitudeAngle, heightDistance);

        // check
        assertEquals(latitude, position.getLatitude(), 0.0);
        assertEquals(longitude, position.getLongitude(), 0.0);
        assertEquals(height, position.getHeight(), 0.0);

        assertEquals(latitude, position.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, position.getLatitudeAngle().getUnit());

        assertEquals(longitude, position.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, position.getLongitudeAngle().getUnit());

        assertEquals(height, position.getHeightDistance().getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, position.getHeightDistance().getUnit());


        // test copy constructor
        final NEDPosition position2 = new NEDPosition(position);

        // check
        assertEquals(latitude, position2.getLatitude(), 0.0);
        assertEquals(longitude, position2.getLongitude(), 0.0);
        assertEquals(height, position2.getHeight(), 0.0);

        assertEquals(latitude, position2.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, position2.getLatitudeAngle().getUnit());

        assertEquals(longitude, position2.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, position2.getLongitudeAngle().getUnit());

        assertEquals(height, position2.getHeightDistance().getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, position2.getHeightDistance().getUnit());
    }

    @Test
    public void testGetSetLatitude() {
        final NEDPosition position = new NEDPosition();

        // check default value
        assertEquals(0.0, position.getLatitude(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        position.setLatitude(latitude);

        // check
        assertEquals(latitude, position.getLatitude(), 0.0);
    }

    @Test
    public void testGetSetLongitude() {
        final NEDPosition position = new NEDPosition();

        // check default value
        assertEquals(0.0, position.getLongitude(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        position.setLongitude(longitude);

        // check
        assertEquals(longitude, position.getLongitude(), 0.0);
    }

    @Test
    public void testGetSetHeight() {
        final NEDPosition position = new NEDPosition();

        // check default value
        assertEquals(0.0, position.getHeight(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        position.setHeight(height);

        // check
        assertEquals(height, position.getHeight(), 0.0);
    }

    @Test
    public void testSetCoordinates() {
        final NEDPosition position = new NEDPosition();

        // check default values
        assertEquals(0.0, position.getLatitude(), 0.0);
        assertEquals(0.0, position.getLongitude(), 0.0);
        assertEquals(0.0, position.getHeight(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        position.setCoordinates(latitude, longitude, height);

        // check
        assertEquals(latitude, position.getLatitude(), 0.0);
        assertEquals(longitude, position.getLongitude(), 0.0);
        assertEquals(height, position.getHeight(), 0.0);
    }

    @Test
    public void testGetSetLatitudeAngle() {
        final NEDPosition position = new NEDPosition();

        // check default values
        assertEquals(0.0, position.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, position.getLatitudeAngle().getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final Angle latitudeAngle1 = new Angle(latitude, AngleUnit.RADIANS);
        position.setLatitudeAngle(latitudeAngle1);

        // check
        final Angle latitudeAngle2 = new Angle(0.0, AngleUnit.DEGREES);
        position.getLatitudeAngle(latitudeAngle2);
        final Angle latitudeAngle3 = position.getLatitudeAngle();

        assertEquals(latitude, latitudeAngle2.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, latitudeAngle2.getUnit());
        assertEquals(latitudeAngle2, latitudeAngle3);
    }

    @Test
    public void testGetSetLongitudeAngle() {
        final NEDPosition position = new NEDPosition();

        // check default values
        assertEquals(0.0, position.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, position.getLongitudeAngle().getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final Angle longitudeAngle1 = new Angle(longitude, AngleUnit.RADIANS);
        position.setLongitudeAngle(longitudeAngle1);

        // check
        final Angle longitudeAngle2 = new Angle(0.0, AngleUnit.DEGREES);
        position.getLongitudeAngle(longitudeAngle2);
        final Angle longitudeAngle3 = position.getLongitudeAngle();

        assertEquals(longitude, longitudeAngle2.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, longitudeAngle2.getUnit());
        assertEquals(longitudeAngle2, longitudeAngle3);
    }

    @Test
    public void testGetSetHeightDistance() {
        final NEDPosition position = new NEDPosition();

        // check default values
        assertEquals(0.0, position.getHeightDistance().getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, position.getHeightDistance().getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final Distance heightDistance1 = new Distance(height, DistanceUnit.METER);
        position.setHeightDistance(heightDistance1);

        // check
        final Distance heightDistance2 = new Distance(0.0, DistanceUnit.KILOMETER);
        position.getHeightDistance(heightDistance2);
        final Distance heightDistance3 = position.getHeightDistance();

        assertEquals(height, heightDistance2.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, heightDistance2.getUnit());
        assertEquals(heightDistance2, heightDistance3);
    }

    @Test
    public void testSetAngleAndDistanceCoordinates() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);

        final NEDPosition position = new NEDPosition();

        // check default values
        assertEquals(0.0, position.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, position.getLatitudeAngle().getUnit());
        assertEquals(0.0, position.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, position.getLongitudeAngle().getUnit());
        assertEquals(0.0, position.getHeightDistance().getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, position.getHeightDistance().getUnit());

        // set new values
        position.setCoordinates(latitudeAngle, longitudeAngle, heightDistance);

        // check
        final Angle latitudeAngle2 = position.getLatitudeAngle();
        final Angle longitudeAngle2 = position.getLongitudeAngle();
        final Distance heightDistance2 = position.getHeightDistance();

        assertEquals(latitude, latitudeAngle2.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, latitudeAngle2.getUnit());
        assertEquals(longitude, longitudeAngle2.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, longitudeAngle2.getUnit());
        assertEquals(height, heightDistance2.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, heightDistance2.getUnit());
        assertEquals(latitudeAngle, latitudeAngle2);
        assertEquals(longitudeAngle, longitudeAngle2);
        assertEquals(heightDistance, heightDistance2);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDPosition
            position1 = new NEDPosition(latitude, longitude, height);
        final NEDPosition position2 = new NEDPosition();
        position1.copyTo(position2);

        // check
        assertEquals(position1.getLatitude(), position2.getLatitude(), 0.0);
        assertEquals(position1.getLongitude(), position2.getLongitude(), 0.0);
        assertEquals(position1.getHeight(), position2.getHeight(), 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDPosition
            position1 = new NEDPosition(latitude, longitude, height);
        final NEDPosition position2 = new NEDPosition();
        position2.copyFrom(position1);

        // check
        assertEquals(position1.getLatitude(), position2.getLatitude(), 0.0);
        assertEquals(position1.getLongitude(), position2.getLongitude(), 0.0);
        assertEquals(position1.getHeight(), position2.getHeight(), 0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDPosition position1 = new NEDPosition(latitude, longitude, height);
        final NEDPosition position2 = new NEDPosition(latitude, longitude, height);
        final NEDPosition position3 = new NEDPosition();

        assertEquals(position1.hashCode(), position2.hashCode());
        assertNotEquals(position1.hashCode(), position3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDPosition position1 = new NEDPosition(latitude, longitude, height);
        final NEDPosition position2 = new NEDPosition(latitude, longitude, height);
        final NEDPosition position3 = new NEDPosition();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(position1.equals((Object) position1));
        //noinspection EqualsWithItself
        assertTrue(position1.equals(position1));
        assertTrue(position1.equals(position2));
        assertFalse(position1.equals(position3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(position1.equals((Object) null));
        assertFalse(position1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(position1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDPosition position1 = new NEDPosition(latitude, longitude, height);
        final NEDPosition position2 = new NEDPosition(latitude, longitude, height);
        final NEDPosition position3 = new NEDPosition();

        assertTrue(position1.equals(position1, THRESHOLD));
        assertTrue(position1.equals(position2, THRESHOLD));
        assertFalse(position1.equals(position3, THRESHOLD));
        assertFalse(position1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDPosition position1 = new NEDPosition(latitude, longitude, height);

        final Object position2 = position1.clone();

        assertEquals(position1, position2);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDPosition position1 = new NEDPosition(latitude, longitude, height);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(position1);
        final NEDPosition position2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(position1, position2);
        assertNotSame(position1, position2);
    }
}
