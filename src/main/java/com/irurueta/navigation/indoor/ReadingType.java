package com.irurueta.navigation.indoor;

/**
 * Contains radio source reading type.
 */
public enum ReadingType {
    /**
     * Radio source signal strength reading.
     */
    RSSI_READING,

    /**
     * Ranging reading containing distance to radio source.
     */
    RANGING_READING,

    /**
     * Reading containing both radio source signal strength and distance to radio source.
     */
    RANGING_AND_RSSI_READING
}
