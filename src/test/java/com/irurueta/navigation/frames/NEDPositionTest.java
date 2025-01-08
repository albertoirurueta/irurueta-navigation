package com.irurueta.navigation.frames;

import com.irurueta.navigation.SerializationHelper;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class NEDPositionTest {

    private static final double THRESHOLD = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    @Test
    void testConstructor() {

        // test empty constructor
        var position = new NEDPosition();

        // check
        assertEquals(0.0, position.getLatitude(), 0.0);
        assertEquals(0.0, position.getLongitude(), 0.0);
        assertEquals(0.0, position.getHeight(), 0.0);

        assertEquals(0.0, position.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(0.0, position.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(0.0, position.getHeightDistance().getValue().doubleValue(), 0.0);


        // test constructor with coordinates
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

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
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

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
        final var position2 = new NEDPosition(position);

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
    void testGetSetLatitude() {
        final var position = new NEDPosition();

        // check default value
        assertEquals(0.0, position.getLatitude(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        position.setLatitude(latitude);

        // check
        assertEquals(latitude, position.getLatitude(), 0.0);
    }

    @Test
    void testGetSetLongitude() {
        final var position = new NEDPosition();

        // check default value
        assertEquals(0.0, position.getLongitude(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        position.setLongitude(longitude);

        // check
        assertEquals(longitude, position.getLongitude(), 0.0);
    }

    @Test
    void testGetSetHeight() {
        final var position = new NEDPosition();

        // check default value
        assertEquals(0.0, position.getHeight(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        position.setHeight(height);

        // check
        assertEquals(height, position.getHeight(), 0.0);
    }

    @Test
    void testSetCoordinates() {
        final var position = new NEDPosition();

        // check default values
        assertEquals(0.0, position.getLatitude(), 0.0);
        assertEquals(0.0, position.getLongitude(), 0.0);
        assertEquals(0.0, position.getHeight(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        position.setCoordinates(latitude, longitude, height);

        // check
        assertEquals(latitude, position.getLatitude(), 0.0);
        assertEquals(longitude, position.getLongitude(), 0.0);
        assertEquals(height, position.getHeight(), 0.0);
    }

    @Test
    void testGetSetLatitudeAngle() {
        final var position = new NEDPosition();

        // check default values
        assertEquals(0.0, position.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, position.getLatitudeAngle().getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var latitudeAngle1 = new Angle(latitude, AngleUnit.RADIANS);
        position.setLatitudeAngle(latitudeAngle1);

        // check
        final var latitudeAngle2 = new Angle(0.0, AngleUnit.DEGREES);
        position.getLatitudeAngle(latitudeAngle2);
        final var latitudeAngle3 = position.getLatitudeAngle();

        assertEquals(latitude, latitudeAngle2.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, latitudeAngle2.getUnit());
        assertEquals(latitudeAngle2, latitudeAngle3);
    }

    @Test
    void testGetSetLongitudeAngle() {
        final var position = new NEDPosition();

        // check default values
        assertEquals(0.0, position.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, position.getLongitudeAngle().getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var longitudeAngle1 = new Angle(longitude, AngleUnit.RADIANS);
        position.setLongitudeAngle(longitudeAngle1);

        // check
        final var longitudeAngle2 = new Angle(0.0, AngleUnit.DEGREES);
        position.getLongitudeAngle(longitudeAngle2);
        final var longitudeAngle3 = position.getLongitudeAngle();

        assertEquals(longitude, longitudeAngle2.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, longitudeAngle2.getUnit());
        assertEquals(longitudeAngle2, longitudeAngle3);
    }

    @Test
    void testGetSetHeightDistance() {
        final var position = new NEDPosition();

        // check default values
        assertEquals(0.0, position.getHeightDistance().getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, position.getHeightDistance().getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var heightDistance1 = new Distance(height, DistanceUnit.METER);
        position.setHeightDistance(heightDistance1);

        // check
        final var heightDistance2 = new Distance(0.0, DistanceUnit.KILOMETER);
        position.getHeightDistance(heightDistance2);
        final var heightDistance3 = position.getHeightDistance();

        assertEquals(height, heightDistance2.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, heightDistance2.getUnit());
        assertEquals(heightDistance2, heightDistance3);
    }

    @Test
    void testSetAngleAndDistanceCoordinates() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var position = new NEDPosition();

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
        final var latitudeAngle2 = position.getLatitudeAngle();
        final var longitudeAngle2 = position.getLongitudeAngle();
        final var heightDistance2 = position.getHeightDistance();

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
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var position1 = new NEDPosition(latitude, longitude, height);
        final var position2 = new NEDPosition();
        position1.copyTo(position2);

        // check
        assertEquals(position1.getLatitude(), position2.getLatitude(), 0.0);
        assertEquals(position1.getLongitude(), position2.getLongitude(), 0.0);
        assertEquals(position1.getHeight(), position2.getHeight(), 0.0);
    }

    @Test
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var position1 = new NEDPosition(latitude, longitude, height);
        final var position2 = new NEDPosition();
        position2.copyFrom(position1);

        // check
        assertEquals(position1.getLatitude(), position2.getLatitude(), 0.0);
        assertEquals(position1.getLongitude(), position2.getLongitude(), 0.0);
        assertEquals(position1.getHeight(), position2.getHeight(), 0.0);
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var position1 = new NEDPosition(latitude, longitude, height);
        final var position2 = new NEDPosition(latitude, longitude, height);
        final var position3 = new NEDPosition();

        assertEquals(position1.hashCode(), position2.hashCode());
        assertNotEquals(position1.hashCode(), position3.hashCode());
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var position1 = new NEDPosition(latitude, longitude, height);
        final var position2 = new NEDPosition(latitude, longitude, height);
        final var position3 = new NEDPosition();

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
        assertNotEquals(new Object(), position1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var position1 = new NEDPosition(latitude, longitude, height);
        final var position2 = new NEDPosition(latitude, longitude, height);
        final var position3 = new NEDPosition();

        assertTrue(position1.equals(position1, THRESHOLD));
        assertTrue(position1.equals(position2, THRESHOLD));
        assertFalse(position1.equals(position3, THRESHOLD));
        assertFalse(position1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var position1 = new NEDPosition(latitude, longitude, height);

        final var position2 = position1.clone();

        assertEquals(position1, position2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var position1 = new NEDPosition(latitude, longitude, height);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(position1);
        final var position2 = SerializationHelper.<NEDPosition>deserialize(bytes);

        // check
        assertEquals(position1, position2);
        assertNotSame(position1, position2);
    }
}
