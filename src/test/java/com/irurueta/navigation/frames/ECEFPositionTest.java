package com.irurueta.navigation.frames;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.SerializationHelper;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class ECEFPositionTest {

    private static final double THRESHOLD = 1e-6;

    private static final double MIN_POSITION_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_POSITION_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    void testConstructor() {

        // test empty constructor
        var position = new ECEFPosition();

        // check
        assertEquals(0.0, position.getX(), 0.0);
        assertEquals(0.0, position.getY(), 0.0);
        assertEquals(0.0, position.getZ(), 0.0);

        Distance distanceX = position.getDistanceX();
        Distance distanceY = position.getDistanceY();
        Distance distanceZ = position.getDistanceZ();

        assertEquals(0.0, distanceX.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceX.getUnit());
        assertEquals(0.0, distanceY.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceY.getUnit());
        assertEquals(0.0, distanceZ.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceZ.getUnit());

        // test constructor with coordinates
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        position = new ECEFPosition(x, y, z);

        // check
        assertEquals(x, position.getX(), 0.0);
        assertEquals(y, position.getY(), 0.0);
        assertEquals(z, position.getZ(), 0.0);

        distanceX = position.getDistanceX();
        distanceY = position.getDistanceY();
        distanceZ = position.getDistanceZ();

        assertEquals(x, distanceX.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceX.getUnit());
        assertEquals(y, distanceY.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceY.getUnit());
        assertEquals(z, distanceZ.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceZ.getUnit());


        // test constructor with coordinate distances
        final var distanceX2 = new Distance(x, DistanceUnit.METER);
        final var distanceY2 = new Distance(y, DistanceUnit.METER);
        final var distanceZ2 = new Distance(z, DistanceUnit.METER);

        position = new ECEFPosition(distanceX2, distanceY2, distanceZ2);

        // check
        assertEquals(x, position.getX(), 0.0);
        assertEquals(y, position.getY(), 0.0);
        assertEquals(z, position.getZ(), 0.0);

        distanceX = position.getDistanceX();
        distanceY = position.getDistanceY();
        distanceZ = position.getDistanceZ();

        assertEquals(distanceX, distanceX2);
        assertEquals(distanceY, distanceY2);
        assertEquals(distanceZ, distanceZ2);


        // test copy constructor
        final var position2 = new ECEFPosition(position);

        // check
        assertEquals(position.getX(), position2.getX(), 0.0);
        assertEquals(position.getY(), position2.getY(), 0.0);
        assertEquals(position.getZ(), position2.getZ(), 0.0);
        assertEquals(position, position2);
    }

    @Test
    void testGetSetX() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position = new ECEFPosition();

        // check default value
        assertEquals(0.0, position.getX(), 0.0);

        // set new value
        position.setX(x);

        // check
        assertEquals(x, position.getX(), 0.0);
    }

    @Test
    void testGetSetY() {
        final var randomizer = new UniformRandomizer();
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position = new ECEFPosition();

        // check default value
        assertEquals(0.0, position.getY(), 0.0);

        // set new value
        position.setY(y);

        // check
        assertEquals(y, position.getY(), 0.0);
    }

    @Test
    void testGetSetZ() {
        final var randomizer = new UniformRandomizer();
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position = new ECEFPosition();

        // check default value
        assertEquals(0.0, position.getZ(), 0.0);

        // set new value
        position.setZ(z);

        // check
        assertEquals(z, position.getZ(), 0.0);
    }

    @Test
    void testSetCoordinates() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position = new ECEFPosition();

        // check default values
        assertEquals(0.0, position.getX(), 0.0);
        assertEquals(0.0, position.getY(), 0.0);
        assertEquals(0.0, position.getZ(), 0.0);

        // set new values
        position.setCoordinates(x, y, z);

        // check
        assertEquals(x, position.getX(), 0.0);
        assertEquals(y, position.getY(), 0.0);
        assertEquals(z, position.getZ(), 0.0);
    }

    @Test
    void testGetSetPosition() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position = new ECEFPosition();

        // check default value
        assertEquals(Point3D.create(), position.getPosition());

        // set new vale
        final var pos1 = new InhomogeneousPoint3D(x, y, z);
        position.setPosition(pos1);

        // check
        final var pos2 = position.getPosition();
        final var pos3 = new InhomogeneousPoint3D();
        position.getPosition(pos3);

        assertEquals(pos1, pos2);
        assertEquals(pos1, pos3);
    }

    @Test
    void testGetSetDistanceX() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position = new ECEFPosition();

        // check default value
        final var distanceX1 = position.getDistanceX();

        assertEquals(0.0, distanceX1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceX1.getUnit());

        // set new value
        final var distanceX2 = new Distance(x, DistanceUnit.METER);
        position.setX(distanceX2);

        // check
        final var distanceX3 = new Distance(0.0, DistanceUnit.KILOMETER);
        position.getDistanceX(distanceX3);
        final var distanceX4 = position.getDistanceX();

        assertEquals(distanceX2, distanceX3);
        assertEquals(distanceX2, distanceX4);
    }

    @Test
    void testGetSetDistanceY() {
        final var randomizer = new UniformRandomizer();
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position = new ECEFPosition();

        // check default value
        final var distanceY1 = position.getDistanceY();

        assertEquals(0.0, distanceY1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceY1.getUnit());

        // set new value
        final var distanceY2 = new Distance(y, DistanceUnit.METER);
        position.setY(distanceY2);

        // check
        final var distanceY3 = new Distance(0.0, DistanceUnit.KILOMETER);
        position.getDistanceY(distanceY3);
        final var distanceY4 = position.getDistanceY();

        assertEquals(distanceY2, distanceY3);
        assertEquals(distanceY2, distanceY4);
    }

    @Test
    void testGetSetDistanceZ() {
        final var randomizer = new UniformRandomizer();
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position = new ECEFPosition();

        // check default values
        final var distanceZ1 = position.getDistanceZ();

        assertEquals(0.0, distanceZ1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceZ1.getUnit());

        // set new value
        final var distanceZ2 = new Distance(z, DistanceUnit.METER);
        position.setZ(distanceZ2);

        // check
        final var distanceZ3 = new Distance(0.0, DistanceUnit.KILOMETER);
        position.getDistanceZ(distanceZ3);
        final var distanceZ4 = position.getDistanceZ();

        assertEquals(distanceZ2, distanceZ3);
        assertEquals(distanceZ2, distanceZ4);
    }

    @Test
    void testSetDistanceCoordinates() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position = new ECEFPosition();

        // check default values
        final var distanceX = new Distance(x, DistanceUnit.METER);
        final var distanceY = new Distance(y, DistanceUnit.METER);
        final var distanceZ = new Distance(z, DistanceUnit.METER);
        position.setCoordinates(distanceX, distanceY, distanceZ);

        // check
        assertEquals(distanceX, position.getDistanceX());
        assertEquals(distanceY, position.getDistanceY());
        assertEquals(distanceZ, position.getDistanceZ());
    }

    @Test
    void testGetNorm() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position = new ECEFPosition(x, y, z);

        final var norm = Math.sqrt(Math.pow(x, 2.0) + Math.pow(y, 2.0) + Math.pow(z, 2.0));
        assertEquals(position.getNorm(), norm, ABSOLUTE_ERROR);

        final var normDistance1 = new Distance(0.0, DistanceUnit.KILOMETER);
        position.getNormAsDistance(normDistance1);
        final var normDistance2 = position.getNormAsDistance();

        assertEquals(norm, normDistance1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(DistanceUnit.METER, normDistance1.getUnit());
        assertEquals(normDistance1, normDistance2);
    }

    @Test
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position1 = new ECEFPosition(x, y, z);
        final var position2 = new ECEFPosition();

        position1.copyTo(position2);

        // check
        assertEquals(x, position2.getX(), 0.0);
        assertEquals(y, position2.getY(), 0.0);
        assertEquals(z, position2.getZ(), 0.0);
    }

    @Test
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position1 = new ECEFPosition(x, y, z);
        final var position2 = new ECEFPosition();

        position2.copyFrom(position1);

        // check
        assertEquals(x, position2.getX(), 0.0);
        assertEquals(y, position2.getY(), 0.0);
        assertEquals(z, position2.getZ(), 0.0);
    }

    @Test
    void testAsArray() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position = new ECEFPosition(x, y, z);

        final var result1 = new double[ECEFPosition.COMPONENTS];
        position.asArray(result1);

        final var result2 = position.asArray();

        assertEquals(x, result1[0], 0.0);
        assertEquals(y, result1[1], 0.0);
        assertEquals(z, result1[2], 0.0);

        assertArrayEquals(result1, result2, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> position.asArray(new double[1]));
    }

    @Test
    void testAsMatrix() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position = new ECEFPosition(x, y, z);

        final var result1 = new Matrix(ECEFPosition.COMPONENTS, 1);
        position.asMatrix(result1);

        final var result2 = position.asMatrix();

        final var result3 = new Matrix(1, 1);
        position.asMatrix(result3);

        assertEquals(x, result1.getElementAtIndex(0), 0.0);
        assertEquals(y, result1.getElementAtIndex(1), 0.0);
        assertEquals(z, result1.getElementAtIndex(2), 0.0);

        assertEquals(result1, result2);
        assertEquals(result1, result3);
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position1 = new ECEFPosition(x, y, z);
        final var position2 = new ECEFPosition(x, y, z);
        final var position3 = new ECEFPosition();

        assertEquals(position1.hashCode(), position2.hashCode());
        assertNotEquals(position1.hashCode(), position3.hashCode());
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position1 = new ECEFPosition(x, y, z);
        final var position2 = new ECEFPosition(x, y, z);
        final var position3 = new ECEFPosition();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(position1.equals((Object) position1));
        //noinspection EqualsWithItself
        assertTrue(position1.equals(position1));
        assertTrue(position1.equals(position2));
        assertFalse(position1.equals(position3));
        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertFalse(position1.equals((Object) null));
        assertFalse(position1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertNotEquals(new Object(), position1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position1 = new ECEFPosition(x, y, z);
        final var position2 = new ECEFPosition(x, y, z);
        final var position3 = new ECEFPosition();

        assertTrue(position1.equals(position1, THRESHOLD));
        assertTrue(position1.equals(position2, THRESHOLD));
        assertFalse(position1.equals(position3, THRESHOLD));
        assertFalse(position1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position1 = new ECEFPosition(x, y, z);

        final var position2 = position1.clone();

        assertEquals(position1, position2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var position1 = new ECEFPosition(x, y, z);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(position1);
        final var position2 = SerializationHelper.<ECEFPosition>deserialize(bytes);

        // check
        assertEquals(position1, position2);
        assertNotSame(position1, position2);
    }
}
