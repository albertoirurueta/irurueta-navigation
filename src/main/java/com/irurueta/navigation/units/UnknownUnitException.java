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
package com.irurueta.navigation.units;

/**
 * Exception raised when a unit cannot be determined.
 */
public class UnknownUnitException extends UnitsException {

    /**
     * Constructor.
     */
    public UnknownUnitException() {
        super();
    }

    /**
     * Constructor with String containing message.
     * @param message message indicating the cause of the exception.
     */
    public UnknownUnitException(String message) {
        super(message);
    }

    /**
     * Constructor with message and cause.
     * @param message message describing the cause of the exception.
     * @param cause instance containing the cause of the exception.
     */
    public UnknownUnitException(String message, Throwable cause) {
        super(message, cause);
    }

    /**
     * Constructor with cause.
     * @param cause instance containing the cause of the exception.
     */
    public UnknownUnitException(Throwable cause) {
        super(cause);
    }
}
