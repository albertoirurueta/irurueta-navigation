/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.frames;

/**
 * Supported frames to describe position and orientation.
 */
public enum FrameType {
    /**
     * Earth Centered Inertial frame (aka ECI frame) is an almost inertial frame (which means it does not accelerate
     * or rotate with respect to the rest of the universe).
     * This frame is nominally centered at the Earth's center of mass and oriented with respect to Earth's
     * spin axis and the stars.
     * The z-axis always points along the Earth's axis of rotation from the frame's origin at the center of
     * mass to the true north pole (not the magnetic pole).
     * The x- and y-axes lie within the equatorial plane, but do not rotate with Earth.
     * The y-axis points 90º ahead of the x-axis in the direction of the Earth's rotation.
     * The x-axis is defined as the direction from the Earth to the Sun at the vernal equinox, which is
     * the spring equinox in the northern hemisphere.
     * Because of Earth rotation and orbit around the Sun, axes of this frame are continuously in
     * movement respect to Earth's surface.
     */
    EARTH_CENTERED_INERTIAL_FRAME,

    /**
     * Earth Centered Fixed frame (aka ECEF frame).
     * This frame is similar to ECI frame except that all axes remain fixed with respect to the Earth's
     * surface.
     * ECEF is also centered at Earth's center of mass.
     * The z-axis is the same as that of ECI's frame, pointing along Earth's axis of rotation from the
     * center to the north pole (true not magnetic).
     * The x-axis points from the center to the intersection of the equator with the IERS Reference
     * Meridian (IRM) or Conventional Zero Meridian (CZM), which defines 0º longitude.
     * The y-axis completes the right-handed orthogonal set, pointing from the center to the
     * intersection of the equator with the 90º east meridian.
     */
    EARTH_CENTERED_EARTH_FIXED_FRAME,

    /**
     * Local Navigation frame.
     * Its origin is the object described by the navigation solution. This could be part of the navigation
     * system itself or the center of mass of the host vehicle or user.
     * The axes are aligned with the topographic directions: north, east, and vertical.
     * By convention the z-axis, also known as the down (D) axis, is defined as the normal to the surface
     * of the reference ellipsoid in the direction pointing towards the Earth. Simple gravity models
     * assume that the gravity vector is coincident with the z-axis of the corresponding local navigation
     * frame. True gravity deviates from this slightly due to local anomalies.
     * The x-axis, or north (N) axis, is the projection in the plane orthogonal to the z-axis of the line
     * from the user to the North Pole.
     * The y-axis completes the orthogonal set by pointing east and is known as the east (E) axis.
     * North, east, down is the most common order of the axes in a local navigation coordinate system.
     * This frame is also known as NED frame, standing for North, East and Down.
     */
    LOCAL_NAVIGATION_FRAME,

    /**
     * Local Tangent-Plane frame.
     * Has a fixed origin with respect to the Earth, usually a point on the surface.
     * Like the local navigation frame, its z-axis is aligned with the vertical (pointing either up or
     * down). Its x- and y-axes may also be aligned with the topographic directions (i.e., north and east),
     * in which case it may be known as a local geodecit frame or topocentric frame. However,
     * the x- and y-axes may be also aligned with an environmental feature, such as a road or building.
     * As with the other frames, the axes form a right-handed orthogonal set.
     * This frame is Earth-fixed, but not Earth-centered.
     * This type of frame is used for navigation within a localized area. Examples include aircraft
     * landing and urban and indoor positioning.
     * A planar frame, can be used for two-dimensional positioning, where its third dimension is
     * neglected. It may comprise the horizontal components of the local tangent-plane frame or
     * may be used to express projected coordinates.
     */
    LOCAL_TANGENT_PLANE_FRAME,

    /**
     * Body frame.
     * Sometimes known as a vehicle frame.
     * Comprises the origin and orientation of the object described by the navigation solution.
     * The origin is coincident with that of the corresponding local navigation frame.
     * However, the axes remain fixed with respect to the body.
     * The most common convention is to set x-axis as the forward axis, pointing in the usual
     * direction of travel, z is the down axis, pointing in the usual direction of gravity, and y
     * is the right axis, completing the orthogonal set. For angular motion, the body-frame axes
     * are also known as roll, pitch, and yaw. Roll motion is about the x-axis, pitch motion is
     * about the y-axis, and yaw motion is about the z-axis.
     */
    BODY_FRAME
}
