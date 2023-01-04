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

import com.google.common.base.Preconditions;

public strictfp class S2LatLngRect implements S2Region {

    private final R1Interval lat;
    private final S1Interval lng;
    public S2LatLngRect(final S2LatLng lo, final S2LatLng hi) {
        lat = new R1Interval(lo.lat().radians(), hi.lat().radians());
        lng = new S1Interval(lo.lng().radians(), hi.lng().radians());
    }

    public S2LatLngRect(R1Interval lat, S1Interval lng) {
        this.lat = lat;
        this.lng = lng;
    }

    public static S2LatLngRect empty() {
        return new S2LatLngRect(R1Interval.empty(), S1Interval.empty());
    }

    static S2LatLngRect full() {
        return new S2LatLngRect(fullLat(), fullLng());
    }

    static R1Interval fullLat() {
        return new R1Interval(-S2.M_PI_2, S2.M_PI_2);
    }

    private static S1Interval fullLng() {
        return S1Interval.full();
    }

    static S2LatLngRect fromCenterSize(S2LatLng center, S2LatLng size) {
        return fromPoint(center).expanded(size.mul(0.5));
    }

    public static S2LatLngRect fromPoint(S2LatLng p) {
        return new S2LatLngRect(p, p);
    }

    static S2LatLngRect fromPointPair(S2LatLng p1, S2LatLng p2) {
        return new S2LatLngRect(R1Interval.fromPointPair(p1.lat().radians(), p2
                .lat().radians()), S1Interval.fromPointPair(p1.lng().radians(), p2.lng()
                .radians()));
    }

    static S2LatLngRect fromEdge(S2Point a, S2Point b) {
        S2LatLngRect r = fromPointPair(new S2LatLng(a), new S2LatLng(b));
        S2Point ab = S2.robustCrossProd(a, b);
        S2Point dir = S2Point.crossProd(ab, new S2Point(0, 0, 1));
        double da = dir.dotProd(a);
        double db = dir.dotProd(b);
        if (da * db >= 0) {
            return r;
        }
        double absLat = Math.acos(Math.abs(ab.z / ab.norm()));
        if (da < 0) {
            return new S2LatLngRect(new R1Interval(r.lat().lo(), absLat), r.lng());
        } else {
            return new S2LatLngRect(new R1Interval(-absLat, r.lat().hi()), r.lng());
        }
    }

    private static boolean intersectsLngEdge(S2Point a, S2Point b,
                                             R1Interval lat, double lng) {
        return S2.simpleCrossing(a, b, S2LatLng.fromRadians(lat.lo(), lng)
                .toPoint(), S2LatLng.fromRadians(lat.hi(), lng).toPoint());
    }

    private static boolean intersectsLatEdge(S2Point a, S2Point b, double lat,
                                             S1Interval lng) {
        S2Point z = S2Point.normalize(S2.robustCrossProd(a, b));
        if (z.z < 0) {
            z = S2Point.neg(z);
        }
        S2Point y = S2Point.normalize(S2.robustCrossProd(z, new S2Point(0, 0, 1)));
        S2Point x = S2Point.crossProd(y, z);
        double sinLat = Math.sin(lat);
        if (Math.abs(sinLat) >= x.z) {
            return false;
        }
        double cosTheta = sinLat / x.z;
        double sinTheta = Math.sqrt(1 - cosTheta * cosTheta);
        double theta = Math.atan2(sinTheta, cosTheta);
        S1Interval abTheta = S1Interval.fromPointPair(Math.atan2(
                a.dotProd(y), a.dotProd(x)), Math.atan2(b.dotProd(y), b.dotProd(x)));
        if (abTheta.contains(theta)) {
            S2Point isect = S2Point.add(S2Point.mul(x, cosTheta), S2Point.mul(y,
                    sinTheta));
            if (lng.contains(Math.atan2(isect.y, isect.x))) {
                return true;
            }
        }
        if (abTheta.contains(-theta)) {
            S2Point intersection = S2Point.sub(S2Point.mul(x, cosTheta), S2Point.mul(y, sinTheta));
            return lng.contains(Math.atan2(intersection.y, intersection.x));
        }
        return false;

    }

    public boolean isValid() {
        return (Math.abs(lat.lo()) <= S2.M_PI_2 && Math.abs(lat.hi()) <= S2.M_PI_2
                && lng.isValid() && lat.isEmpty() == lng.isEmpty());
    }

    S1Angle latLo() {
        return S1Angle.radians(lat.lo());
    }

    S1Angle latHi() {
        return S1Angle.radians(lat.hi());
    }

    S1Angle lngLo() {
        return S1Angle.radians(lng.lo());
    }

    S1Angle lngHi() {
        return S1Angle.radians(lng.hi());
    }

    public R1Interval lat() {
        return lat;
    }

    public S1Interval lng() {
        return lng;
    }

    public S2LatLng lo() {
        return new S2LatLng(latLo(), lngLo());
    }

    public S2LatLng hi() {
        return new S2LatLng(latHi(), lngHi());
    }

    public boolean isEmpty() {
        return lat.isEmpty();
    }

    public boolean isFull() {
        return lat.equals(fullLat()) && lng.isFull();
    }

    public boolean isInverted() {
        return lng.isInverted();
    }

    S2LatLng getVertex(int k) {
        switch (k) {
            case 0:
                return S2LatLng.fromRadians(lat.lo(), lng.lo());
            case 1:
                return S2LatLng.fromRadians(lat.lo(), lng.hi());
            case 2:
                return S2LatLng.fromRadians(lat.hi(), lng.hi());
            case 3:
                return S2LatLng.fromRadians(lat.hi(), lng.lo());
            default:
                throw new IllegalArgumentException("Invalid vertex index.");
        }
    }

    public S2LatLng getCenter() {
        return S2LatLng.fromRadians(lat.getCenter(), lng.getCenter());
    }

    public S1Angle getDistance(S2LatLng p) {
        S2LatLngRect a = this;
        Preconditions.checkState(!a.isEmpty());
        Preconditions.checkArgument(p.isValid());
        if (a.lng().contains(p.lng().radians())) {
            return S1Angle.radians(Math.max(0.0, Math.max(p.lat().radians() - a.lat().hi(),
                    a.lat().lo() - p.lat().radians())));
        }
        S1Interval interval = new S1Interval(a.lng().hi(), a.lng().complement().getCenter());
        double aLng = a.lng().lo();
        if (interval.contains(p.lng().radians())) {
            aLng = a.lng().hi();
        }
        S2Point lo = S2LatLng.fromRadians(a.lat().lo(), aLng).toPoint();
        S2Point hi = S2LatLng.fromRadians(a.lat().hi(), aLng).toPoint();
        S2Point loCrossHi =
                S2LatLng.fromRadians(0, aLng - S2.M_PI_2).normalized().toPoint();
        return S2EdgeUtil.getDistance(p.toPoint(), lo, hi, loCrossHi);
    }

    public S1Angle getDistance(S2LatLngRect other) {
        S2LatLngRect a = this;
        Preconditions.checkState(!a.isEmpty());
        Preconditions.checkArgument(!other.isEmpty());
        if (a.lng().intersects(other.lng())) {
            if (a.lat().intersects(other.lat())) {
                return S1Angle.radians(0);
            }
            S1Angle lo, hi;
            if (a.lat().lo() > other.lat().hi()) {
                lo = other.latHi();
                hi = a.latLo();
            } else {
                lo = a.latHi();
                hi = other.latLo();
            }
            return S1Angle.radians(hi.radians() - lo.radians());
        }
        S1Angle aLng, bLng;
        S1Interval loHi = S1Interval.fromPointPair(a.lng().lo(), other.lng().hi());
        S1Interval hiLo = S1Interval.fromPointPair(a.lng().hi(), other.lng().lo());
        if (loHi.getLength() < hiLo.getLength()) {
            aLng = a.lngLo();
            bLng = other.lngHi();
        } else {
            aLng = a.lngHi();
            bLng = other.lngLo();
        }
        S2Point aLo = new S2LatLng(a.latLo(), aLng).toPoint();
        S2Point aHi = new S2LatLng(a.latHi(), aLng).toPoint();
        S2Point aLoCrossHi =
                S2LatLng.fromRadians(0, aLng.radians() - S2.M_PI_2).normalized().toPoint();
        S2Point bLo = new S2LatLng(other.latLo(), bLng).toPoint();
        S2Point bHi = new S2LatLng(other.latHi(), bLng).toPoint();
        S2Point bLoCrossHi =
                S2LatLng.fromRadians(0, bLng.radians() - S2.M_PI_2).normalized().toPoint();
        return S1Angle.min(S2EdgeUtil.getDistance(aLo, bLo, bHi, bLoCrossHi),
                S1Angle.min(S2EdgeUtil.getDistance(aHi, bLo, bHi, bLoCrossHi),
                        S1Angle.min(S2EdgeUtil.getDistance(bLo, aLo, aHi, aLoCrossHi),
                                S2EdgeUtil.getDistance(bHi, aLo, aHi, aLoCrossHi))));
    }

    public S2LatLng getSize() {
        return S2LatLng.fromRadians(lat.getLength(), lng.getLength());
    }

    public boolean contains(S2LatLng ll) {
        return (lat.contains(ll.lat().radians()) && lng.contains(ll.lng()
                .radians()));

    }

    public boolean interiorContains(S2Point p) {
        return interiorContains(new S2LatLng(p));
    }

    boolean interiorContains(S2LatLng ll) {
        return (lat.interiorContains(ll.lat().radians()) && lng
                .interiorContains(ll.lng().radians()));
    }

    public boolean contains(S2LatLngRect other) {
        return lat.contains(other.lat) && lng.contains(other.lng);
    }

    boolean interiorContains(S2LatLngRect other) {
        return (lat.interiorContains(other.lat) && lng
                .interiorContains(other.lng));
    }

    public boolean intersects(S2LatLngRect other) {
        return lat.intersects(other.lat) && lng.intersects(other.lng);
    }

    public boolean intersects(S2Cell cell) {
        if (isEmpty()) {
            return false;
        }
        if (contains(cell.getCenter())) {
            return true;
        }
        if (cell.contains(getCenter().toPoint())) {
            return true;
        }
        if (!intersects(cell.getRectBound())) {
            return false;
        }
        S2Point[] cellV = new S2Point[4];
        S2LatLng[] cellLl = new S2LatLng[4];
        for (int i = 0; i < 4; ++i) {
            cellV[i] = cell.getVertex(i);
            cellLl[i] = new S2LatLng(cellV[i]);
            if (contains(cellLl[i])) {
                return true;
            }
        }
        for (int i = 0; i < 4; ++i) {
            S1Interval edgeLng = S1Interval.fromPointPair(
                    cellLl[i].lng().radians(), cellLl[(i + 1) & 3].lng().radians());
            if (!lng.intersects(edgeLng)) {
                continue;
            }
            final S2Point a = cellV[i];
            final S2Point b = cellV[(i + 1) & 3];
            if (edgeLng.contains(lng.lo())) {
                if (intersectsLngEdge(a, b, lat, lng.lo())) {
                    return true;
                }
            }
            if (edgeLng.contains(lng.hi())) {
                if (intersectsLngEdge(a, b, lat, lng.hi())) {
                    return true;
                }
            }
            if (intersectsLatEdge(a, b, lat.lo(), lng)) {
                return true;
            }
            if (intersectsLatEdge(a, b, lat.hi(), lng)) {
                return true;
            }
        }
        return false;
    }

    boolean interiorIntersects(S2LatLngRect other) {
        return (lat.interiorIntersects(other.lat) && lng
                .interiorIntersects(other.lng));
    }

    public S2LatLngRect addPoint(S2Point p) {
        return addPoint(new S2LatLng(p));
    }

    public S2LatLngRect addPoint(S2LatLng ll) {
        R1Interval newLat = lat.addPoint(ll.lat().radians());
        S1Interval newLng = lng.addPoint(ll.lng().radians());
        return new S2LatLngRect(newLat, newLng);
    }

    S2LatLngRect expanded(S2LatLng margin) {
        if (isEmpty()) {
            return this;
        }
        return new S2LatLngRect(lat.expanded(margin.lat().radians()).intersection(
                fullLat()), lng.expanded(margin.lng().radians()));
    }

    public S2LatLngRect union(S2LatLngRect other) {
        return new S2LatLngRect(lat.union(other.lat), lng.union(other.lng));
    }

    public S2LatLngRect intersection(S2LatLngRect other) {
        R1Interval intersectLat = lat.intersection(other.lat);
        S1Interval intersectLng = lng.intersection(other.lng);
        if (intersectLat.isEmpty() || intersectLng.isEmpty()) {
            return empty();
        }
        return new S2LatLngRect(intersectLat, intersectLng);
    }

    S2LatLngRect convolveWithCap(S1Angle angle) {
        S2Cap cap = S2Cap.fromAxisAngle(new S2Point(1, 0, 0), angle);
        S2LatLngRect r = this;
        for (int k = 0; k < 4; ++k) {
            S2Cap vertexCap = S2Cap.fromAxisHeight(getVertex(k).toPoint(), cap
                    .height());
            r = r.union(vertexCap.getRectBound());
        }
        return r;
    }

    public double area() {
        if (isEmpty()) {
            return 0;
        }
        return lng().getLength() * Math.abs(Math.sin(latHi().radians()) - Math.sin(latLo().radians()));
    }

    @Override
    public boolean equals(Object that) {
        if (!(that instanceof S2LatLngRect)) {
            return false;
        }
        S2LatLngRect otherRect = (S2LatLngRect) that;
        return lat().equals(otherRect.lat()) && lng().equals(otherRect.lng());
    }

    public boolean approxEquals(S2LatLngRect other, double maxError) {
        return (lat.approxEquals(other.lat, maxError) && lng.approxEquals(
                other.lng, maxError));
    }

    public boolean approxEquals(S2LatLngRect other) {
        return approxEquals(other, 1e-15);
    }

    @Override
    public int hashCode() {
        int value = 17;
        value = 37 * value + lat.hashCode();
        return (37 * value + lng.hashCode());
    }

    @Override
    public S2Region clone() {
        return new S2LatLngRect(this.lo(), this.hi());
    }

    @Override
    public S2Cap getCapBound() {
        if (isEmpty()) {
            return S2Cap.empty();
        }
        double poleZ, poleAngle;
        if (lat.lo() + lat.hi() < 0) {
            poleZ = -1;
            poleAngle = S2.M_PI_2 + lat.hi();
        } else {
            poleZ = 1;
            poleAngle = S2.M_PI_2 - lat.lo();
        }
        S2Cap poleCap = S2Cap.fromAxisAngle(new S2Point(0, 0, poleZ), S1Angle
                .radians(poleAngle));
        double lngSpan = lng.hi() - lng.lo();
        if (Math.IEEEremainder(lngSpan, 2 * S2.M_PI) >= 0) {
            if (lngSpan < 2 * S2.M_PI) {
                S2Cap midCap = S2Cap.fromAxisAngle(getCenter().toPoint(), S1Angle
                        .radians(0));
                for (int k = 0; k < 4; ++k) {
                    midCap = midCap.addPoint(getVertex(k).toPoint());
                }
                if (midCap.height() < poleCap.height()) {
                    return midCap;
                }
            }
        }
        return poleCap;
    }

    @Override
    public S2LatLngRect getRectBound() {
        return this;
    }

    @Override
    public boolean contains(S2Cell cell) {
        return contains(cell.getRectBound());
    }

    @Override
    public boolean mayIntersect(S2Cell cell) {
        return intersects(cell.getRectBound());
    }

    public boolean contains(S2Point p) {
        return contains(new S2LatLng(p));
    }

    @Override
    public String toString() {
        return "[Lo=" + lo() + ", Hi=" + hi() + "]";
    }
}
