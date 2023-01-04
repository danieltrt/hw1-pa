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

public final strictfp class R1Interval {

    private final double lo;
    private final double hi;

    public R1Interval(double lo, double hi) {
        this.lo = lo;
        this.hi = hi;
    }

    public static R1Interval empty() {
        return new R1Interval(1, 0);
    }

    public static R1Interval fromPoint(double p) {
        return new R1Interval(p, p);
    }

    public static R1Interval fromPointPair(double p1, double p2) {
        if (p1 <= p2) {
            return new R1Interval(p1, p2);
        } else {
            return new R1Interval(p2, p1);
        }
    }

    public double lo() {
        return lo;
    }

    public double hi() {
        return hi;
    }

    public boolean isEmpty() {
        return lo() > hi();
    }

    public double getCenter() {
        return 0.5 * (lo() + hi());
    }

    public double getLength() {
        return hi() - lo();
    }

    public boolean contains(double p) {
        return p >= lo() && p <= hi();
    }

    public boolean interiorContains(double p) {
        return p > lo() && p < hi();
    }

    public boolean contains(R1Interval y) {
        if (y.isEmpty()) {
            return true;
        }
        return y.lo() >= lo() && y.hi() <= hi();
    }

    public boolean interiorContains(R1Interval y) {
        if (y.isEmpty()) {
            return true;
        }
        return y.lo() > lo() && y.hi() < hi();
    }

    public boolean intersects(R1Interval y) {
        if (lo() <= y.lo()) {
            return y.lo() <= hi() && y.lo() <= y.hi();
        } else {
            return lo() <= y.hi() && lo() <= hi();
        }
    }

    public boolean interiorIntersects(R1Interval y) {
        return y.lo() < hi() && lo() < y.hi() && lo() < hi() && y.lo() <= y.hi();
    }

    public R1Interval addPoint(double p) {
        if (isEmpty()) {
            return R1Interval.fromPoint(p);
        } else if (p < lo()) {
            return new R1Interval(p, hi());
        } else if (p > hi()) {
            return new R1Interval(lo(), p);
        } else {
            return new R1Interval(lo(), hi());
        }
    }

    public R1Interval expanded(double radius) {
        if (isEmpty()) {
            return this;
        }
        return new R1Interval(lo() - radius, hi() + radius);
    }

    public R1Interval union(R1Interval y) {
        if (isEmpty()) {
            return y;
        }
        if (y.isEmpty()) {
            return this;
        }
        return new R1Interval(Math.min(lo(), y.lo()), Math.max(hi(), y.hi()));
    }

    public R1Interval intersection(R1Interval y) {
        return new R1Interval(Math.max(lo(), y.lo()), Math.min(hi(), y.hi()));
    }

    @Override
    public boolean equals(Object that) {
        if (that instanceof R1Interval) {
            R1Interval y = (R1Interval) that;
            return (lo() == y.lo() && hi() == y.hi()) || (isEmpty() && y.isEmpty());

        }
        return false;
    }

    @Override
    public int hashCode() {
        if (isEmpty()) {
            return 17;
        }

        long value = 17;
        value = 37 * value + Double.doubleToLongBits(lo);
        value = 37 * value + Double.doubleToLongBits(hi);
        return (int) (value ^ (value >>> 32));
    }

    public boolean approxEquals(R1Interval y) {
        return approxEquals(y, 1e-15);
    }

    public boolean approxEquals(R1Interval y, double maxError) {
        if (isEmpty()) {
            return y.getLength() <= maxError;
        }
        if (y.isEmpty()) {
            return getLength() <= maxError;
        }
        return Math.abs(y.lo() - lo()) + Math.abs(y.hi() - hi()) <= maxError;
    }

    @Override
    public String toString() {
        return "[" + lo() + ", " + hi() + "]";
    }
}
