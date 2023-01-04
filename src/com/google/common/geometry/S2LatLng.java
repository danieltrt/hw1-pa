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

public strictfp class S2LatLng {

    static final double EARTH_RADIUS_METERS = 6367000.0;

    public static final S2LatLng CENTER = new S2LatLng(0.0, 0.0);

    private final double latRadians;
    private final double lngRadians;

    private S2LatLng(double latRadians, double lngRadians) {
        this.latRadians = latRadians;
        this.lngRadians = lngRadians;
    }

    public S2LatLng(S1Angle lat, S1Angle lng) {
        this(lat.radians(), lng.radians());
    }

    public S2LatLng() {
        this(0, 0);
    }

    public S2LatLng(S2Point p) {
        this(Math.atan2(p.z, Math.sqrt(p.x * p.x + p.y * p.y)), Math.atan2(p.y, p.x));
    }

    static S2LatLng fromRadians(double latRadians, double lngRadians) {
        return new S2LatLng(latRadians, lngRadians);
    }

    public static S2LatLng fromDegrees(double latDegrees, double lngDegrees) {
        return new S2LatLng(S1Angle.degrees(latDegrees), S1Angle.degrees(lngDegrees));
    }

    static S2LatLng fromE5(long latE5, long lngE5) {
        return new S2LatLng(S1Angle.e5(latE5), S1Angle.e5(lngE5));
    }

    public static S2LatLng fromE6(long latE6, long lngE6) {
        return new S2LatLng(S1Angle.e6(latE6), S1Angle.e6(lngE6));
    }

    public static S2LatLng fromE7(long latE7, long lngE7) {
        return new S2LatLng(S1Angle.e7(latE7), S1Angle.e7(lngE7));
    }

    public static S1Angle latitude(S2Point p) {
        return S1Angle.radians(
                Math.atan2(p.get(2), Math.sqrt(p.get(0) * p.get(0) + p.get(1) * p.get(1))));
    }

    static S1Angle longitude(S2Point p) {
        return S1Angle.radians(Math.atan2(p.get(1), p.get(0)));
    }

    public S1Angle lat() {
        return S1Angle.radians(latRadians);
    }

    public double latRadians() {
        return latRadians;
    }

    double latDegrees() {
        return 180.0 / Math.PI * latRadians;
    }

    public S1Angle lng() {
        return S1Angle.radians(lngRadians);
    }

    public double lngRadians() {
        return lngRadians;
    }

    double lngDegrees() {
        return 180.0 / Math.PI * lngRadians;
    }

    public boolean isValid() {
        return Math.abs(lat().radians()) <= S2.M_PI_2 && Math.abs(lng().radians()) <= S2.M_PI;
    }

    S2LatLng normalized() {
        return new S2LatLng(Math.max(-S2.M_PI_2, Math.min(S2.M_PI_2, lat().radians())),
                Math.IEEEremainder(lng().radians(), 2 * S2.M_PI));
    }

    public S2Point toPoint() {
        double phi = lat().radians();
        double theta = lng().radians();
        double cosphi = Math.cos(phi);
        return new S2Point(Math.cos(theta) * cosphi, Math.sin(theta) * cosphi, Math.sin(phi));
    }

    public S1Angle getDistance(final S2LatLng o) {
        double lat1 = lat().radians();
        double lat2 = o.lat().radians();
        double lng1 = lng().radians();
        double lng2 = o.lng().radians();
        double dlat = Math.sin(0.5 * (lat2 - lat1));
        double dlng = Math.sin(0.5 * (lng2 - lng1));
        double x = dlat * dlat + dlng * dlng * Math.cos(lat1) * Math.cos(lat2);
        return S1Angle.radians(2 * Math.atan2(Math.sqrt(x), Math.sqrt(Math.max(0.0, 1.0 - x))));
    }

    public double getDistance(final S2LatLng o, double radius) {
        return getDistance(o).radians() * radius;
    }

    public double getEarthDistance(final S2LatLng o) {
        return getDistance(o, EARTH_RADIUS_METERS);
    }

    public S2LatLng add(final S2LatLng o) {
        return new S2LatLng(latRadians + o.latRadians, lngRadians + o.lngRadians);
    }

    public S2LatLng sub(final S2LatLng o) {
        return new S2LatLng(latRadians - o.latRadians, lngRadians - o.lngRadians);
    }

    public S2LatLng mul(final double m) {
        return new S2LatLng(latRadians * m, lngRadians * m);
    }

    @Override
    public boolean equals(Object that) {
        if (that instanceof S2LatLng) {
            S2LatLng o = (S2LatLng) that;
            return (latRadians == o.latRadians) && (lngRadians == o.lngRadians);
        }
        return false;
    }

    @Override
    public int hashCode() {
        long value = 17;
        value += 37 * value + Double.doubleToLongBits(latRadians);
        value += 37 * value + Double.doubleToLongBits(lngRadians);
        return (int) (value ^ (value >>> 32));
    }

    public boolean approxEquals(S2LatLng o, double maxError) {
        return (Math.abs(latRadians - o.latRadians) < maxError)
                && (Math.abs(lngRadians - o.lngRadians) < maxError);
    }

    public boolean approxEquals(S2LatLng o) {
        return approxEquals(o, 1e-9);
    }

    @Override
    public String toString() {
        return "(" + latRadians + ", " + lngRadians + ")";
    }

    String toStringDegrees() {
        return "(" + latDegrees() + ", " + lngDegrees() + ")";
    }
}
