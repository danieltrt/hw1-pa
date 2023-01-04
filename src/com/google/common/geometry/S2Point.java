/*
 * Copyright 2006 Google Inc.
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

public strictfp class S2Point implements Comparable<S2Point> {
    final double x;
    final double y;
    final double z;

    public S2Point() {
        x = y = z = 0;
    }

    public S2Point(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    static S2Point minus(S2Point p1, S2Point p2) {
        return sub(p1, p2);
    }

    static S2Point neg(S2Point p) {
        return new S2Point(-p.x, -p.y, -p.z);
    }

    public static S2Point crossProd(final S2Point p1, final S2Point p2) {
        return new S2Point(
                p1.y * p2.z - p1.z * p2.y, p1.z * p2.x - p1.x * p2.z, p1.x * p2.y - p1.y * p2.x);
    }

    public static S2Point add(final S2Point p1, final S2Point p2) {
        return new S2Point(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
    }

    public static S2Point sub(final S2Point p1, final S2Point p2) {
        return new S2Point(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
    }

    public static S2Point mul(final S2Point p, double m) {
        return new S2Point(m * p.x, m * p.y, m * p.z);
    }

    static S2Point div(final S2Point p, double m) {
        return new S2Point(p.x / m, p.y / m, p.z / m);
    }

    static S2Point fabs(S2Point p) {
        return new S2Point(Math.abs(p.x), Math.abs(p.y), Math.abs(p.z));
    }

    public static S2Point normalize(S2Point p) {
        double norm = p.norm();
        if (norm != 0) {
            norm = 1.0 / norm;
        }
        return S2Point.mul(p, norm);
    }

    double norm2() {
        return x * x + y * y + z * z;
    }

    double norm() {
        return Math.sqrt(norm2());
    }

    public double dotProd(S2Point that) {
        return this.x * that.x + this.y * that.y + this.z * that.z;
    }

    S2Point ortho() {
        int k = largestAbsComponent();
        S2Point temp;
        if (k == 1) {
            temp = new S2Point(1, 0, 0);
        } else if (k == 2) {
            temp = new S2Point(0, 1, 0);
        } else {
            temp = new S2Point(0, 0, 1);
        }
        return S2Point.normalize(crossProd(this, temp));
    }

    int largestAbsComponent() {
        S2Point temp = fabs(this);
        if (temp.x > temp.y) {
            if (temp.x > temp.z) {
                return 0;
            } else {
                return 2;
            }
        } else {
            if (temp.y > temp.z) {
                return 1;
            } else {
                return 2;
            }
        }
    }

    public double get(int axis) {
        return (axis == 0) ? x : (axis == 1) ? y : z;
    }

    public double angle(S2Point va) {
        return Math.atan2(crossProd(this, va).norm(), this.dotProd(va));
    }

    boolean aequal(S2Point that, double margin) {
        return (Math.abs(x - that.x) < margin) && (Math.abs(y - that.y) < margin)
                && (Math.abs(z - that.z) < margin);
    }

    @Override
    public boolean equals(Object that) {
        if (!(that instanceof S2Point)) {
            return false;
        }
        S2Point thatPoint = (S2Point) that;
        return this.x == thatPoint.x && this.y == thatPoint.y && this.z == thatPoint.z;
    }

    public boolean lessThan(S2Point vb) {
        if (x < vb.x) {
            return true;
        }
        if (vb.x < x) {
            return false;
        }
        if (y < vb.y) {
            return true;
        }
        if (vb.y < y) {
            return false;
        }
        return z < vb.z;
    }

    @Override
    public int compareTo(S2Point other) {
        return (lessThan(other) ? -1 : (equals(other) ? 0 : 1));
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ", " + z + ")";
    }

    String toDegreesString() {
        S2LatLng s2LatLng = new S2LatLng(this);
        return "(" + s2LatLng.latDegrees() + ", "
                + s2LatLng.lngDegrees() + ")";
    }

    @Override
    public int hashCode() {
        long value = 17;
        value += 37 * value + Double.doubleToLongBits(Math.abs(x));
        value += 37 * value + Double.doubleToLongBits(Math.abs(y));
        value += 37 * value + Double.doubleToLongBits(Math.abs(z));
        return (int) (value ^ (value >>> 32));
    }
}
