package com.irurueta.navigation.units;

/**
 * Contains a time value and unit.
 */
public class Time extends Measurement<TimeUnit> {

    /**
     * Constructor with value and unit.
     * @param value time value.
     * @param unit unit of distance.
     * @throws IllegalArgumentException if either value or unit is null.
     */
    public Time(Number value, TimeUnit unit) throws IllegalArgumentException {
        super(value, unit);
    }

    /**
     * Constructor.
     */
    Time() {
        super();
    }
}
