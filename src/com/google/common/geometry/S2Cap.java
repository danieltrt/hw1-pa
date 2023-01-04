/*
 * Copyright 2005 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.google.common.geometry;

public final strictfp class S2Cap implements S2Region {

    private static final double ROUND_UP = 1.0 + 1.0 / (1L << 52);

    private final S2Point axis;
    private final double height;

    private S2Cap() {
        axis = new S2Point();
        height = 0;
    }

    private S2Cap(S2Point axis, double height) {
        this.axis = axis;
        this.height = height;
    }

    static S2Cap fromAxisHeight(S2Point axis, double height) {
        return new S2Cap(axis, height);
    }

    static S2Cap fromAxisAngle(S2Point axis, S1Angle angle) {
        double d = Math.sin(0.5 * angle.radians());
        return new S2Cap(axis, 2 * d * d);

    }

    static S2Cap fromAxisArea(S2Point axis, double area) {
        return new S2Cap(axis, area / (2 * S2.M_PI));
    }

    public static S2Cap empty() {
        return new S2Cap(new S2Point(1, 0, 0), -1);
    }

    static S2Cap full() {
        return new S2Cap(new S2Point(1, 0, 0), 2);
    }

    public S2Point axis() {
        return axis;
    }

    double height() {
        return height;
    }

    public double area() {
        return 2 * S2.M_PI * Math.max(0.0, height);
    }

    public S1Angle angle() {
        if (isEmpty()) {
            return S1Angle.radians(-1);
        }
        return S1Angle.radians(2 * Math.asin(Math.sqrt(0.5 * height)));
    }

    public boolean isValid() {
        return S2.isUnitLength(axis) && height <= 2;
    }

    public boolean isEmpty() {
        return height < 0;
    }

    public boolean isFull() {
        return height >= 2;
    }

    S2Cap complement() {
        double cHeight = isFull() ? -1 : 2 - Math.max(height, 0.0);
        return S2Cap.fromAxisHeight(S2Point.neg(axis), cHeight);
    }

    public boolean contains(S2Cap other) {
        if (isFull() || other.isEmpty()) {
            return true;
        }
        return angle().radians() >= axis.angle(other.axis)
                + other.angle().radians();
    }

    boolean interiorIntersects(S2Cap other) {
        return !complement().contains(other);
    }

    boolean interiorContains(S2Point p) {
        return isFull() || S2Point.sub(axis, p).norm2() < 2 * height;
    }

    public S2Cap addPoint(S2Point p) {
        if (isEmpty()) {
            return new S2Cap(p, 0);
        } else {
            double dist2 = S2Point.sub(axis, p).norm2();
            double newHeight = Math.max(height, ROUND_UP * 0.5 * dist2);
            return new S2Cap(axis, newHeight);
        }
    }

    S2Cap addCap(S2Cap other) {
        if (isEmpty()) {
            return new S2Cap(other.axis, other.height);
        } else {
            double angle = axis.angle(other.axis) + other.angle().radians();
            if (angle >= S2.M_PI) {
                return new S2Cap(axis, 2);
            } else {
                double d = Math.sin(0.5 * angle);
                double newHeight = Math.max(height, ROUND_UP * 2 * d * d);
                return new S2Cap(axis, newHeight);
            }
        }
    }

    @Override
    public S2Cap getCapBound() {
        return this;
    }

    @Override
    public S2LatLngRect getRectBound() {
        if (isEmpty()) {
            return S2LatLngRect.empty();
        }
        S2LatLng axisLatLng = new S2LatLng(axis);
        double capAngle = angle().radians();
        boolean allLongitudes = false;
        double[] lat = new double[2], lng = new double[2];
        lng[0] = -S2.M_PI;
        lng[1] = S2.M_PI;
        lat[0] = axisLatLng.lat().radians() - capAngle;
        if (lat[0] <= -S2.M_PI_2) {
            lat[0] = -S2.M_PI_2;
            allLongitudes = true;
        }
        lat[1] = axisLatLng.lat().radians() + capAngle;
        if (lat[1] >= S2.M_PI_2) {
            lat[1] = S2.M_PI_2;
            allLongitudes = true;
        }
        if (!allLongitudes) {
            double sinA = Math.sqrt(height * (2 - height));
            double sinC = Math.cos(axisLatLng.lat().radians());
            if (sinA <= sinC) {
                double angleA = Math.asin(sinA / sinC);
                lng[0] = Math.IEEEremainder(axisLatLng.lng().radians() - angleA,
                        2 * S2.M_PI);
                lng[1] = Math.IEEEremainder(axisLatLng.lng().radians() + angleA,
                        2 * S2.M_PI);
            }
        }
        return new S2LatLngRect(new R1Interval(lat[0], lat[1]), new S1Interval(
                lng[0], lng[1]));
    }

    @Override
    public boolean contains(S2Cell cell) {
        S2Point[] vertices = new S2Point[4];
        for (int k = 0; k < 4; ++k) {
            vertices[k] = cell.getVertex(k);
            if (!contains(vertices[k])) {
                return false;
            }
        }
        return !complement().intersects(cell, vertices);
    }

    @Override
    public boolean mayIntersect(S2Cell cell) {
        S2Point[] vertices = new S2Point[4];
        for (int k = 0; k < 4; ++k) {
            vertices[k] = cell.getVertex(k);
            if (contains(vertices[k])) {
                return true;
            }
        }
        return intersects(cell, vertices);
    }

    public boolean intersects(S2Cell cell, S2Point[] vertices) {
        if (height >= 1) {
            return false;
        }
        if (isEmpty()) {
            return false;
        }
        if (cell.contains(axis)) {
            return true;
        }
        double sin2Angle = height * (2 - height);
        for (int k = 0; k < 4; ++k) {
            S2Point edge = cell.getEdgeRaw(k);
            double dot = axis.dotProd(edge);
            if (dot > 0) {
                continue;
            }
            if (dot * dot > sin2Angle * edge.norm2()) {
                return false;
            }
            S2Point dir = S2Point.crossProd(edge, axis);
            if (dir.dotProd(vertices[k]) < 0
                    && dir.dotProd(vertices[(k + 1) & 3]) > 0) {
                return true;
            }
        }
        return false;
    }

    public boolean contains(S2Point p) {
        return S2Point.sub(axis, p).norm2() <= 2 * height;
    }

    @Override
    public boolean equals(Object that) {
        if (!(that instanceof S2Cap)) {
            return false;
        }
        S2Cap other = (S2Cap) that;
        return (axis.equals(other.axis) && height == other.height)
                || (isEmpty() && other.isEmpty()) || (isFull() && other.isFull());
    }

    @Override
    public int hashCode() {
        if (isFull()) {
            return 17;
        } else if (isEmpty()) {
            return 37;
        }
        int result = 17;
        result = 37 * result + axis.hashCode();
        long heightBits = Double.doubleToLongBits(height);
        result = 37 * result + (int) ((heightBits >>> 32) ^ heightBits);
        return result;
    }

    private boolean approxEquals(S2Cap other, double maxError) {
        return (axis.aequal(other.axis, maxError) && Math.abs(height - other.height) <= maxError)
                || (isEmpty() && other.height <= maxError)
                || (other.isEmpty() && height <= maxError)
                || (isFull() && other.height >= 2 - maxError)
                || (other.isFull() && height >= 2 - maxError);
    }

    boolean approxEquals(S2Cap other) {
        return approxEquals(other, 1e-14);
    }

    @Override
    public String toString() {
        return "[Point = " + axis.toString() + " Height = " + height + "]";
    }
}
