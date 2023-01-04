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

import com.google.common.base.Preconditions;

public strictfp class S2EdgeUtil {
    static final S1Angle DEFAULT_INTERSECTION_TOLERANCE = S1Angle.radians(1.5e-15);

    private S2EdgeUtil() {
    }

    static boolean simpleCrossing(S2Point a, S2Point b, S2Point c, S2Point d) {
        S2Point ab = S2Point.crossProd(a, b);
        double acb = -(ab.dotProd(c));
        double bda = ab.dotProd(d);
        if (acb * bda <= 0) {
            return false;
        }
        S2Point cd = S2Point.crossProd(c, d);
        double cbd = -(cd.dotProd(b));
        double dac = cd.dotProd(a);
        return (acb * cbd > 0) && (acb * dac > 0);
    }

    static int robustCrossing(S2Point a, S2Point b, S2Point c, S2Point d) {
        S2Point aCrossB = S2Point.crossProd(a, b);
        int acb = -S2.robustCCW(a, b, c, aCrossB);
        int bda = S2.robustCCW(a, b, d, aCrossB);
        if ((bda & acb) == 0) {
            return 0;
        }
        if (bda != acb) {
            return -1;
        }
        S2Point cCrossD = S2Point.crossProd(c, d);
        int cbd = -S2.robustCCW(c, d, b, cCrossD);
        if (cbd != acb) {
            return -1;
        }
        int dac = S2.robustCCW(c, d, a, cCrossD);
        return (dac == acb) ? 1 : -1;
    }

    static boolean vertexCrossing(S2Point a, S2Point b, S2Point c, S2Point d) {
        if (a.equals(b) || c.equals(d)) {
            return false;
        }
        if (a.equals(d)) {
            return S2.orderedCCW(S2.ortho(a), c, b, a);
        }
        if (b.equals(c)) {
            return S2.orderedCCW(S2.ortho(b), d, a, b);
        }
        if (a.equals(c)) {
            return S2.orderedCCW(S2.ortho(a), d, b, a);
        }
        if (b.equals(d)) {
            return S2.orderedCCW(S2.ortho(b), c, a, b);
        }
        return false;
    }

    static boolean edgeOrVertexCrossing(S2Point a, S2Point b, S2Point c, S2Point d) {
        int crossing = robustCrossing(a, b, c, d);
        if (crossing < 0) {
            return false;
        }
        if (crossing > 0) {
            return true;
        }
        return vertexCrossing(a, b, c, d);
    }

    static S2Point getIntersection(S2Point a0, S2Point a1, S2Point b0, S2Point b1) {
        Preconditions.checkArgument(robustCrossing(a0, a1, b0, b1) > 0,
                "Input edges a0a1 and b0b1 muct have a true robustCrossing.");
        S2Point aNorm = S2Point.normalize(S2.robustCrossProd(a0, a1));
        S2Point bNorm = S2Point.normalize(S2.robustCrossProd(b0, b1));
        S2Point x = S2Point.normalize(S2.robustCrossProd(aNorm, bNorm));
        if (x.dotProd(S2Point.add(S2Point.add(a0, a1), S2Point.add(b0, b1))) < 0) {
            x = S2Point.neg(x);
        }
        if (S2.orderedCCW(a0, x, a1, aNorm) && S2.orderedCCW(b0, x, b1, bNorm)) {
            return x;
        }
        CloserResult r = new CloserResult(10, x);
        if (S2.orderedCCW(b0, a0, b1, bNorm)) {
            r.replaceIfCloser(x, a0);
        }
        if (S2.orderedCCW(b0, a1, b1, bNorm)) {
            r.replaceIfCloser(x, a1);
        }
        if (S2.orderedCCW(a0, b0, a1, aNorm)) {
            r.replaceIfCloser(x, b0);
        }
        if (S2.orderedCCW(a0, b1, a1, aNorm)) {
            r.replaceIfCloser(x, b1);
        }
        return r.getVmin();
    }

    static double getDistanceFraction(S2Point x, S2Point a0, S2Point a1) {
        Preconditions.checkArgument(!a0.equals(a1));
        double d0 = x.angle(a0);
        double d1 = x.angle(a1);
        return d0 / (d0 + d1);
    }

    public static S1Angle getDistance(S2Point x, S2Point a, S2Point b) {
        return getDistance(x, a, b, S2.robustCrossProd(a, b));
    }

    public static S1Angle getDistance(S2Point x, S2Point a, S2Point b, S2Point aCrossB) {
        Preconditions.checkArgument(S2.isUnitLength(x));
        Preconditions.checkArgument(S2.isUnitLength(a));
        Preconditions.checkArgument(S2.isUnitLength(b));
        if (S2.simpleCCW(aCrossB, a, x) && S2.simpleCCW(x, b, aCrossB)) {
            double sinDist = Math.abs(x.dotProd(aCrossB)) / aCrossB.norm();
            return S1Angle.radians(Math.asin(Math.min(1.0, sinDist)));
        }
        double linearDist2 = Math.min(S2Point.minus(x, a).norm2(), S2Point.minus(x, b).norm2());
        return S1Angle.radians(2 * Math.asin(Math.min(1.0, 0.5 * Math.sqrt(linearDist2))));
    }

    static S2Point getClosestPoint(S2Point x, S2Point a, S2Point b) {
        Preconditions.checkArgument(S2.isUnitLength(x));
        Preconditions.checkArgument(S2.isUnitLength(a));
        Preconditions.checkArgument(S2.isUnitLength(b));
        S2Point crossProd = S2.robustCrossProd(a, b);
        S2Point p = S2Point.minus(x, S2Point.mul(crossProd, x.dotProd(crossProd) / crossProd.norm2()));
        if (S2.simpleCCW(crossProd, a, p) && S2.simpleCCW(p, b, crossProd)) {
            return S2Point.normalize(p);
        }
        return S2Point.minus(x, a).norm2() <= S2Point.minus(x, b).norm2() ? a : b;
    }

    public interface WedgeRelation {
        int test(S2Point a0, S2Point ab1, S2Point a2, S2Point b0, S2Point b2);
    }

    static class EdgeCrosser {
        private final S2Point a;
        private final S2Point b;
        private final S2Point aCrossB;
        private S2Point c;
        private int acb;

        EdgeCrosser(S2Point a, S2Point b, S2Point c) {
            this.a = a;
            this.b = b;
            this.aCrossB = S2Point.crossProd(a, b);
            restartAt(c);
        }

        void restartAt(S2Point c) {
            this.c = c;
            acb = -S2.robustCCW(a, b, c, aCrossB);
        }

        int robustCrossing(S2Point d) {
            int bda = S2.robustCCW(a, b, d, aCrossB);
            int result;
            if (bda == -acb && bda != 0) {
                result = -1;
            } else if ((bda & acb) == 0) {
                result = 0;
            } else {
                result = robustCrossingInternal(d); // Slow path.
            }
            c = d;
            acb = -bda;
            return result;
        }

        boolean edgeOrVertexCrossing(S2Point d) {
            S2Point c2 = new S2Point(c.get(0), c.get(1), c.get(2));
            int crossing = robustCrossing(d);
            if (crossing < 0) {
                return false;
            }
            if (crossing > 0) {
                return true;
            }
            return vertexCrossing(a, b, c2, d);
        }

        private int robustCrossingInternal(S2Point d) {
            S2Point cCrossD = S2Point.crossProd(c, d);
            int cbd = -S2.robustCCW(c, d, b, cCrossD);
            if (cbd != acb) {
                return -1;
            }
            int dac = S2.robustCCW(c, d, a, cCrossD);
            return (dac == acb) ? 1 : -1;
        }
    }

    public static class RectBounder {
        private S2Point a;
        private S2LatLng aLatLng;
        private S2LatLngRect bound;

        RectBounder() {
            this.bound = S2LatLngRect.empty();
        }

        public void addPoint(S2Point b) {
            S2LatLng bLatLng = new S2LatLng(b);
            if (bound.isEmpty()) {
                bound = bound.addPoint(bLatLng);
            } else {
                bound = bound.union(S2LatLngRect.fromPointPair(aLatLng, bLatLng));
                S2Point aCrossB = S2.robustCrossProd(a, b);
                S2Point dir = S2Point.crossProd(aCrossB, new S2Point(0, 0, 1));
                double da = dir.dotProd(a);
                double db = dir.dotProd(b);
                if (da * db < 0) {
                    double absLat = Math.acos(Math.abs(aCrossB.get(2) / aCrossB.norm()));
                    R1Interval lat = bound.lat();
                    if (da < 0) {
                        lat = new R1Interval(lat.lo(), Math.max(absLat, bound.lat().hi()));
                    } else {
                        lat = new R1Interval(Math.min(-absLat, bound.lat().lo()), lat.hi());
                    }
                    bound = new S2LatLngRect(lat, bound.lng());
                }
            }
            a = b;
            aLatLng = bLatLng;
        }

        S2LatLngRect getBound() {
            return bound;
        }
    }

    public static class XYZPruner {
        private S2Point lastVertex;
        private boolean boundSet;
        private double xmin;
        private double ymin;
        private double zmin;
        private double xmax;
        private double ymax;
        private double zmax;
        private double maxDeformation;

        XYZPruner() {
            boundSet = false;
        }

        void addEdgeToBounds(S2Point from, S2Point to) {
            if (!boundSet) {
                boundSet = true;
                xmin = xmax = from.x;
                ymin = ymax = from.y;
                zmin = zmax = from.z;
            }
            xmin = Math.min(xmin, Math.min(to.x, from.x));
            ymin = Math.min(ymin, Math.min(to.y, from.y));
            zmin = Math.min(zmin, Math.min(to.z, from.z));
            xmax = Math.max(xmax, Math.max(to.x, from.x));
            ymax = Math.max(ymax, Math.max(to.y, from.y));
            zmax = Math.max(zmax, Math.max(to.z, from.z));
            double approxArcLen =
                    Math.abs(from.x - to.x) + Math.abs(from.y - to.y) + Math.abs(from.z - to.z);
            if (approxArcLen < 0.025) {
                maxDeformation = Math.max(maxDeformation, approxArcLen * 0.0025);
            } else if (approxArcLen < 1.0) {
                maxDeformation = Math.max(maxDeformation, approxArcLen * 0.11);
            } else {
                maxDeformation = approxArcLen * 0.5;
            }
        }

        void setFirstIntersectPoint(S2Point v0) {
            xmin = xmin - maxDeformation;
            ymin = ymin - maxDeformation;
            zmin = zmin - maxDeformation;
            xmax = xmax + maxDeformation;
            ymax = ymax + maxDeformation;
            zmax = zmax + maxDeformation;
            this.lastVertex = v0;
        }

        public boolean intersects(S2Point v1) {
            boolean result = true;
            if ((v1.x < xmin && lastVertex.x < xmin) || (v1.x > xmax && lastVertex.x > xmax)) {
                result = false;
            } else if ((v1.y < ymin && lastVertex.y < ymin) || (v1.y > ymax && lastVertex.y > ymax)) {
                result = false;
            } else if ((v1.z < zmin && lastVertex.z < zmin) || (v1.z > zmax && lastVertex.z > zmax)) {
                result = false;
            }
            lastVertex = v1;
            return result;
        }
    }

    public static class LongitudePruner {
        private S1Interval interval;

        private double lng0;

        LongitudePruner(S1Interval interval, S2Point v0) {
            this.interval = interval;
            this.lng0 = S2LatLng.longitude(v0).radians();
        }

        public boolean intersects(S2Point v1) {
            double lng1 = S2LatLng.longitude(v1).radians();
            boolean result = interval.intersects(S1Interval.fromPointPair(lng0, lng1));
            lng0 = lng1;
            return result;
        }
    }

    public static class WedgeContains implements WedgeRelation {
        @Override
        public int test(S2Point a0, S2Point ab1, S2Point a2, S2Point b0, S2Point b2) {
            return S2.orderedCCW(a2, b2, b0, ab1) && S2.orderedCCW(b0, a0, a2, ab1) ? 1 : 0;
        }
    }

    public static class WedgeIntersects implements WedgeRelation {
        @Override
        public int test(S2Point a0, S2Point ab1, S2Point a2, S2Point b0, S2Point b2) {
            return (S2.orderedCCW(a0, b2, b0, ab1) && S2.orderedCCW(b0, a2, a0, ab1) ? 0 : -1);
        }
    }

    public static class WedgeContainsOrIntersects implements WedgeRelation {
        @Override
        public int test(S2Point a0, S2Point ab1, S2Point a2, S2Point b0, S2Point b2) {
            if (S2.orderedCCW(a0, a2, b2, ab1)) {
                return S2.orderedCCW(b2, b0, a0, ab1) ? 1 : -1;
            }
            if (!S2.orderedCCW(a2, b0, b2, ab1)) {
                return 0;
            }
            return (a2.equals(b0)) ? 0 : -1;
        }
    }

    public static class WedgeContainsOrCrosses implements WedgeRelation {
        @Override
        public int test(S2Point a0, S2Point ab1, S2Point a2, S2Point b0, S2Point b2) {
            if (S2.orderedCCW(a0, a2, b2, ab1)) {
                if (S2.orderedCCW(b2, b0, a0, ab1)) {
                    return 1;
                }
                return (a2.equals(b2)) ? 0 : -1;
            }
            return S2.orderedCCW(a0, b0, a2, ab1) ? 0 : -1;
        }
    }

    static class CloserResult {
        private double dmin2;
        private S2Point vmin;

        CloserResult(double dmin2, S2Point vmin) {
            this.dmin2 = dmin2;
            this.vmin = vmin;
        }

        public double getDmin2() {
            return dmin2;
        }

        S2Point getVmin() {
            return vmin;
        }

        void replaceIfCloser(S2Point x, S2Point y) {
            double d2 = S2Point.minus(x, y).norm2();
            if (d2 < dmin2 || (d2 == dmin2 && y.lessThan(vmin))) {
                dmin2 = d2;
                vmin = y;
            }
        }
    }
}
