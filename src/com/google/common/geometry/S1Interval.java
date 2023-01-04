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

public final strictfp class S1Interval implements Cloneable {

    private final double lo;
    private final double hi;

    S1Interval(double lo, double hi) {
        this(lo, hi, false);
    }

    private S1Interval(S1Interval interval) {
        this.lo = interval.lo;
        this.hi = interval.hi;
    }

    private S1Interval(double lo, double hi, boolean checked) {
        double newLo = lo;
        double newHi = hi;
        if (!checked) {
            if (lo == -S2.M_PI && hi != S2.M_PI) {
                newLo = S2.M_PI;
            }
            if (hi == -S2.M_PI && lo != S2.M_PI) {
                newHi = S2.M_PI;
            }
        }
        this.lo = newLo;
        this.hi = newHi;
    }

    public static S1Interval empty() {
        return new S1Interval(S2.M_PI, -S2.M_PI, true);
    }

    static S1Interval full() {
        return new S1Interval(-S2.M_PI, S2.M_PI, true);
    }

    public static S1Interval fromPoint(double p) {
        if (p == -S2.M_PI) {
            p = S2.M_PI;
        }
        return new S1Interval(p, p, true);
    }

    static S1Interval fromPointPair(double p1, double p2) {
        if (p1 == -S2.M_PI) {
            p1 = S2.M_PI;
        }
        if (p2 == -S2.M_PI) {
            p2 = S2.M_PI;
        }
        if (positiveDistance(p1, p2) <= S2.M_PI) {
            return new S1Interval(p1, p2, true);
        } else {
            return new S1Interval(p2, p1, true);
        }
    }

    private static double positiveDistance(double a, double b) {
        double d = b - a;
        if (d >= 0) {
            return d;
        }
        return (b + S2.M_PI) - (a - S2.M_PI);
    }

    public double lo() {
        return lo;
    }

    public double hi() {
        return hi;
    }

    public boolean isValid() {
        return (Math.abs(lo()) <= S2.M_PI && Math.abs(hi()) <= S2.M_PI
                && !(lo() == -S2.M_PI && hi() != S2.M_PI) && !(hi() == -S2.M_PI && lo() != S2.M_PI));
    }

    public boolean isFull() {
        return hi() - lo() == 2 * S2.M_PI;
    }

    public boolean isEmpty() {
        return lo() - hi() == 2 * S2.M_PI;
    }

    boolean isInverted() {
        return lo() > hi();
    }

    public double getCenter() {
        double center = 0.5 * (lo() + hi());
        if (!isInverted()) {
            return center;
        }
        return (center <= 0) ? (center + S2.M_PI) : (center - S2.M_PI);
    }

    public double getLength() {
        double length = hi() - lo();
        if (length >= 0) {
            return length;
        }
        length += 2 * S2.M_PI;
        return (length > 0) ? length : -1;
    }

    S1Interval complement() {
        if (lo() == hi()) {
            return full();
        }
        return new S1Interval(hi(), lo(), true);
    }

    public boolean contains(double p) {
        if (p == -S2.M_PI) {
            p = S2.M_PI;
        }
        return fastContains(p);
    }

    private boolean fastContains(double p) {
        if (isInverted()) {
            return (p >= lo() || p <= hi()) && !isEmpty();
        } else {
            return p >= lo() && p <= hi();
        }
    }

    boolean interiorContains(double p) {
        if (p == -S2.M_PI) {
            p = S2.M_PI;
        }
        if (isInverted()) {
            return p > lo() || p < hi();
        } else {
            return (p > lo() && p < hi()) || isFull();
        }
    }

    public boolean contains(final S1Interval y) {
        if (isInverted()) {
            if (y.isInverted()) {
                return y.lo() >= lo() && y.hi() <= hi();
            }
            return (y.lo() >= lo() || y.hi() <= hi()) && !isEmpty();
        } else {
            if (y.isInverted()) {
                return isFull() || y.isEmpty();
            }
            return y.lo() >= lo() && y.hi() <= hi();
        }
    }

    boolean interiorContains(final S1Interval y) {
        if (isInverted()) {
            if (!y.isInverted()) {
                return y.lo() > lo() || y.hi() < hi();
            }
            return (y.lo() > lo() && y.hi() < hi()) || y.isEmpty();
        } else {
            if (y.isInverted()) {
                return isFull() || y.isEmpty();
            }
            return (y.lo() > lo() && y.hi() < hi()) || isFull();
        }
    }

    public boolean intersects(final S1Interval y) {
        if (isEmpty() || y.isEmpty()) {
            return false;
        }
        if (isInverted()) {
            return y.isInverted() || y.lo() <= hi() || y.hi() >= lo();
        } else {
            if (y.isInverted()) {
                return y.lo() <= hi() || y.hi() >= lo();
            }
            return y.lo() <= hi() && y.hi() >= lo();
        }
    }

    boolean interiorIntersects(final S1Interval y) {
        if (isEmpty() || y.isEmpty() || lo() == hi()) {
            return false;
        }
        if (isInverted()) {
            return y.isInverted() || y.lo() < hi() || y.hi() > lo();
        } else {
            if (y.isInverted()) {
                return y.lo() < hi() || y.hi() > lo();
            }
            return (y.lo() < hi() && y.hi() > lo()) || isFull();
        }
    }

    public S1Interval addPoint(double p) {
        if (p == -S2.M_PI) {
            p = S2.M_PI;
        }

        if (fastContains(p)) {
            return new S1Interval(this);
        }

        if (isEmpty()) {
            return S1Interval.fromPoint(p);
        } else {
            double dlo = positiveDistance(p, lo());
            double dhi = positiveDistance(hi(), p);
            if (dlo < dhi) {
                return new S1Interval(p, hi());
            } else {
                return new S1Interval(lo(), p);
            }
        }
    }

    S1Interval expanded(double radius) {
        if (isEmpty()) {
            return this;
        }
        if (getLength() + 2 * radius >= 2 * S2.M_PI - 1e-15) {
            return full();
        }
        double lo = Math.IEEEremainder(lo() - radius, 2 * S2.M_PI);
        double hi = Math.IEEEremainder(hi() + radius, 2 * S2.M_PI);
        if (lo == -S2.M_PI) {
            lo = S2.M_PI;
        }
        return new S1Interval(lo, hi);
    }

    public S1Interval union(final S1Interval y) {
        if (y.isEmpty()) {
            return this;
        }
        if (fastContains(y.lo())) {
            if (fastContains(y.hi())) {
                if (contains(y)) {
                    return this;
                }
                return full();
            }
            return new S1Interval(lo(), y.hi(), true);
        }
        if (fastContains(y.hi())) {
            return new S1Interval(y.lo(), hi(), true);
        }
        if (isEmpty() || y.fastContains(lo())) {
            return y;
        }
        double dlo = positiveDistance(y.hi(), lo());
        double dhi = positiveDistance(hi(), y.lo());
        if (dlo < dhi) {
            return new S1Interval(y.lo(), hi(), true);
        } else {
            return new S1Interval(lo(), y.hi(), true);
        }
    }

    public S1Interval intersection(final S1Interval y) {
        if (y.isEmpty()) {
            return empty();
        }
        if (fastContains(y.lo())) {
            if (fastContains(y.hi())) {
                if (y.getLength() < getLength()) {
                    return y;
                }
                return this;
            }
            return new S1Interval(y.lo(), hi(), true);
        }
        if (fastContains(y.hi())) {
            return new S1Interval(lo(), y.hi(), true);
        }
        if (y.fastContains(lo())) {
            return this;
        }
        return empty();
    }

    public boolean approxEquals(final S1Interval y, double maxError) {
        if (isEmpty()) {
            return y.getLength() <= maxError;
        }
        if (y.isEmpty()) {
            return getLength() <= maxError;
        }
        return (Math.abs(Math.IEEEremainder(y.lo() - lo(), 2 * S2.M_PI))
                + Math.abs(Math.IEEEremainder(y.hi() - hi(), 2 * S2.M_PI))) <= maxError;
    }

    public boolean approxEquals(final S1Interval y) {
        return approxEquals(y, 1e-9);
    }

    @Override
    public boolean equals(Object that) {
        if (that instanceof S1Interval) {
            S1Interval thatInterval = (S1Interval) that;
            return lo() == thatInterval.lo() && hi() == thatInterval.hi();
        }
        return false;
    }

    @Override
    public int hashCode() {
        long value = 17;
        value = 37 * value + Double.doubleToLongBits(lo());
        value = 37 * value + Double.doubleToLongBits(hi());
        return (int) ((value >>> 32) ^ value);
    }

    @Override
    public String toString() {
        return "[" + this.lo() + ", " + this.hi() + "]";
    }
}
