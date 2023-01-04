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


public final strictfp class S1Angle implements Comparable<S1Angle> {

    private final double radians;

    public S1Angle() {
        this.radians = 0;
    }

    private S1Angle(double radians) {
        this.radians = radians;
    }

    public S1Angle(S2Point x, S2Point y) {
        this.radians = x.angle(y);
    }

    public static S1Angle max(S1Angle left, S1Angle right) {
        return right.greaterThan(left) ? right : left;
    }

    public static S1Angle min(S1Angle left, S1Angle right) {
        return right.greaterThan(left) ? left : right;
    }

    public static S1Angle radians(double radians) {
        return new S1Angle(radians);
    }

    public static S1Angle degrees(double degrees) {
        return new S1Angle(degrees * (Math.PI / 180));
    }

    static S1Angle e5(long e5) {
        return degrees(e5 * 1e-5);
    }

    static S1Angle e6(long e6) {
        return degrees(e6 * 1e-6);
    }

    static S1Angle e7(long e7) {
        return degrees(e7 * 1e-7);
    }

    public double radians() {
        return radians;
    }

    public double degrees() {
        return radians * (180 / Math.PI);
    }

    long e5() {
        return Math.round(degrees() * 1e5);
    }

    long e6() {
        return Math.round(degrees() * 1e6);
    }

    long e7() {
        return Math.round(degrees() * 1e7);
    }

    @Override
    public boolean equals(Object that) {
        if (that instanceof S1Angle) {
            return this.radians() == ((S1Angle) that).radians();
        }
        return false;
    }

    @Override
    public int hashCode() {
        long value = Double.doubleToLongBits(radians);
        return (int) (value ^ (value >>> 32));
    }

    public boolean lessThan(S1Angle that) {
        return this.radians() < that.radians();
    }

    private boolean greaterThan(S1Angle that) {
        return this.radians() > that.radians();
    }

    public boolean lessOrEquals(S1Angle that) {
        return this.radians() <= that.radians();
    }

    public boolean greaterOrEquals(S1Angle that) {
        return this.radians() >= that.radians();
    }

    @Override
    public String toString() {
        return degrees() + "d";
    }

    @Override
    public int compareTo(S1Angle that) {
        return Double.compare(this.radians, that.radians);
    }
}
