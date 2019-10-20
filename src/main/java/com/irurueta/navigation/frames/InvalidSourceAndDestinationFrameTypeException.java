package com.irurueta.navigation.frames;

/**
 * Exception raised if provided source and destination frame types of a coordinate transformation matrix is not
 * valid for a given frame.
 */
public class InvalidSourceAndDestinationFrameTypeException extends FrameException {

    /**
     * Constructor.
     */
    public InvalidSourceAndDestinationFrameTypeException() {
        super();
    }

    /**
     * Constructor with message.
     *
     * @param message message indicating the cause of the exception.
     */
    public InvalidSourceAndDestinationFrameTypeException(final String message) {
        super(message);
    }

    /**
     * Constructor with message and cause.
     *
     * @param message message indicating the cause of the exception.
     * @param cause instance containing the cause of the exception.
     */
    public InvalidSourceAndDestinationFrameTypeException(final String message, final Throwable cause) {
        super(message, cause);
    }

    /**
     * Constructor with cause.
     *
     * @param cause instance containing the cause of the exception.
     */
    public InvalidSourceAndDestinationFrameTypeException(final Throwable cause) {
        super(cause);
    }
}
