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

import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;

public final strictfp class S2 {

    // Declare some frequently used constants
    public static final double M_PI = Math.PI;
    static final double M_1_PI = 1.0 / Math.PI;
    public static final double M_PI_2 = Math.PI / 2.0;
    static final double M_PI_4 = Math.PI / 4.0;
    static final double M_SQRT2 = Math.sqrt(2);
    static final double M_E = Math.E;
    static final int SWAP_MASK = 0x01;
    static final int INVERT_MASK = 0x02;
    private static final int EXPONENT_SHIFT = 52;
    private static final long EXPONENT_MASK = 0x7ff0000000000000L;

    private static final int[] POS_TO_ORIENTATION =
            {SWAP_MASK, 0, 0, INVERT_MASK + SWAP_MASK};

    private static final int[][] POS_TO_IJ = {
            {0, 1, 3, 2},
            {0, 2, 3, 1},
            {3, 2, 0, 1},
            {3, 1, 0, 2},
    };

    private static final int[][] IJ_TO_POS = {
            {0, 1, 3, 2},
            {0, 3, 1, 2},
            {2, 3, 1, 0},
            {2, 1, 3, 0},
    };

    private S2() {
    }

    @VisibleForTesting
    static int exp(double v) {
        if (v == 0) {
            return 0;
        }
        long bits = Double.doubleToLongBits(v);
        return (int) ((EXPONENT_MASK & bits) >> EXPONENT_SHIFT) - 1022;
    }

    static int posToOrientation(int position) {
        Preconditions.checkArgument(0 <= position && position < 4);
        return POS_TO_ORIENTATION[position];
    }

    static int posToIJ(int orientation, int position) {
        Preconditions.checkArgument(0 <= orientation && orientation < 4);
        Preconditions.checkArgument(0 <= position && position < 4);
        return POS_TO_IJ[orientation][position];
    }

    static int ijToPos(int orientation, int ijIndex) {
        Preconditions.checkArgument(0 <= orientation && orientation < 4);
        Preconditions.checkArgument(0 <= ijIndex && ijIndex < 4);
        return IJ_TO_POS[orientation][ijIndex];
    }

    static S2Point origin() {
        return new S2Point(0, 1, 0);
    }

    static boolean isUnitLength(S2Point p) {
        return Math.abs(p.norm2() - 1) <= 1e-15;
    }

    static boolean simpleCrossing(S2Point a, S2Point b, S2Point c, S2Point d) {
        S2Point ab = S2Point.crossProd(a, b);
        S2Point cd = S2Point.crossProd(c, d);
        double acb = -ab.dotProd(c);
        double cbd = -cd.dotProd(b);
        double bda = ab.dotProd(d);
        double dac = cd.dotProd(a);
        return (acb * cbd > 0) && (cbd * bda > 0) && (bda * dac > 0);
    }

    static S2Point robustCrossProd(S2Point a, S2Point b) {
        S2Point x = S2Point.crossProd(S2Point.add(b, a), S2Point.sub(b, a));
        if (!x.equals(new S2Point(0, 0, 0))) {
            return x;
        }
        return ortho(a);
    }

    static S2Point ortho(S2Point a) {
        return a.ortho();
    }

    static double area(S2Point a, S2Point b, S2Point c) {
        final double sa = b.angle(c);
        final double sb = c.angle(a);
        final double sc = a.angle(b);
        final double s = 0.5 * (sa + sb + sc);
        if (s >= 3e-4) {
            double s2 = s * s;
            double dmin = s - Math.max(sa, Math.max(sb, sc));
            if (dmin < 1e-2 * s * s2 * s2) {
                double area = girardArea(a, b, c);
                if (dmin < s * (0.1 * area)) {
                    return area;
                }
            }
        }
        return 4
                * Math.atan(
                Math.sqrt(
                        Math.max(0.0,
                                Math.tan(0.5 * s) * Math.tan(0.5 * (s - sa)) * Math.tan(0.5 * (s - sb))
                                        * Math.tan(0.5 * (s - sc)))));
    }

    static double girardArea(S2Point a, S2Point b, S2Point c) {
        S2Point ab = S2Point.crossProd(a, b);
        S2Point bc = S2Point.crossProd(b, c);
        S2Point ac = S2Point.crossProd(a, c);
        return Math.max(0.0, ab.angle(ac) - ab.angle(bc) + bc.angle(ac));
    }

    static double signedArea(S2Point a, S2Point b, S2Point c) {
        return area(a, b, c) * robustCCW(a, b, c);
    }

    public static S2Point planarCentroid(S2Point a, S2Point b, S2Point c) {
        return new S2Point((a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0, (a.z + b.z + c.z) / 3.0);
    }

    static S2Point trueCentroid(S2Point a, S2Point b, S2Point c) {
        double sina = S2Point.crossProd(b, c).norm();
        double sinb = S2Point.crossProd(c, a).norm();
        double sinc = S2Point.crossProd(a, b).norm();
        double ra = (sina == 0) ? 1 : (Math.asin(sina) / sina);
        double rb = (sinb == 0) ? 1 : (Math.asin(sinb) / sinb);
        double rc = (sinc == 0) ? 1 : (Math.asin(sinc) / sinc);
        S2Point x = new S2Point(a.x, b.x, c.x);
        S2Point y = new S2Point(a.y, b.y, c.y);
        S2Point z = new S2Point(a.z, b.z, c.z);
        S2Point r = new S2Point(ra, rb, rc);
        return new S2Point(0.5 * S2Point.crossProd(y, z).dotProd(r),
                0.5 * S2Point.crossProd(z, x).dotProd(r), 0.5 * S2Point.crossProd(x, y).dotProd(r));
    }

    static boolean simpleCCW(S2Point a, S2Point b, S2Point c) {
        return S2Point.crossProd(c, a).dotProd(b) > 0;
    }

    static int robustCCW(S2Point a, S2Point b, S2Point c) {
        return robustCCW(a, b, c, S2Point.crossProd(a, b));
    }

    static int robustCCW(S2Point a, S2Point b, S2Point c, S2Point aCrossB) {
        final double kMinAbsValue = 1.6e-15;
        double det = aCrossB.dotProd(c);
        if (det > kMinAbsValue) {
            return 1;
        }
        if (det < -kMinAbsValue) {
            return -1;
        }
        return expensiveCCW(a, b, c);
    }

    private static int expensiveCCW(S2Point a, S2Point b, S2Point c) {
        if (a.equals(b) || b.equals(c) || c.equals(a)) {
            return 0;
        }
        double sab = (a.dotProd(b) > 0) ? -1 : 1;
        double sbc = (b.dotProd(c) > 0) ? -1 : 1;
        double sca = (c.dotProd(a) > 0) ? -1 : 1;
        S2Point vab = S2Point.add(a, S2Point.mul(b, sab));
        S2Point vbc = S2Point.add(b, S2Point.mul(c, sbc));
        S2Point vca = S2Point.add(c, S2Point.mul(a, sca));
        double dab = vab.norm2();
        double dbc = vbc.norm2();
        double dca = vca.norm2();
        double sign;
        if (dca < dbc || (dca == dbc && a.lessThan(b))) {
            if (dab < dbc || (dab == dbc && a.lessThan(c))) {
                sign = S2Point.crossProd(vab, vca).dotProd(a) * sab;
            } else {
                sign = S2Point.crossProd(vca, vbc).dotProd(c) * sca;
            }
        } else {
            if (dab < dca || (dab == dca && b.lessThan(c))) {
                sign = S2Point.crossProd(vbc, vab).dotProd(b) * sbc;
            } else {
                sign = S2Point.crossProd(vca, vbc).dotProd(c) * sca;
            }
        }
        if (sign > 0) {
            return 1;
        }
        if (sign < 0) {
            return -1;
        }
        int ccw =
                planarOrderedCCW(new R2Vector(a.y, a.z), new R2Vector(b.y, b.z), new R2Vector(c.y, c.z));
        if (ccw == 0) {
            ccw =
                    planarOrderedCCW(new R2Vector(a.z, a.x), new R2Vector(b.z, b.x), new R2Vector(c.z, c.x));
            if (ccw == 0) {
                ccw = planarOrderedCCW(
                        new R2Vector(a.x, a.y), new R2Vector(b.x, b.y), new R2Vector(c.x, c.y));
            }
        }
        return ccw;
    }


    private static int planarCCW(R2Vector a, R2Vector b) {
        double sab = (a.dotProd(b) > 0) ? -1 : 1;
        R2Vector vab = R2Vector.add(a, R2Vector.mul(b, sab));
        double da = a.norm2();
        double db = b.norm2();
        double sign;
        if (da < db || (da == db && a.lessThan(b))) {
            sign = a.crossProd(vab) * sab;
        } else {
            sign = vab.crossProd(b);
        }
        if (sign > 0) {
            return 1;
        }
        if (sign < 0) {
            return -1;
        }
        return 0;
    }

    private static int planarOrderedCCW(R2Vector a, R2Vector b, R2Vector c) {
        int sum = 0;
        sum += planarCCW(a, b);
        sum += planarCCW(b, c);
        sum += planarCCW(c, a);
        return Integer.compare(sum, 0);
    }

    static boolean orderedCCW(S2Point a, S2Point b, S2Point c, S2Point o) {
        int sum = 0;
        if (robustCCW(b, o, a) >= 0) {
            ++sum;
        }
        if (robustCCW(c, o, b) >= 0) {
            ++sum;
        }
        if (robustCCW(a, o, c) > 0) {
            ++sum;
        }
        return sum >= 2;
    }

    public static double angle(S2Point a, S2Point b, S2Point c) {
        return S2Point.crossProd(a, b).angle(S2Point.crossProd(c, b));
    }

    public static double turnAngle(S2Point a, S2Point b, S2Point c) {
        double outAngle = S2Point.crossProd(b, a).angle(S2Point.crossProd(c, b));
        return (robustCCW(a, b, c) > 0) ? outAngle : -outAngle;
    }

    public static boolean approxEquals(S2Point a, S2Point b, double maxError) {
        return a.angle(b) <= maxError;
    }

    public static boolean approxEquals(S2Point a, S2Point b) {
        return approxEquals(a, b, 1e-15);
    }

    public static boolean approxEquals(double a, double b, double maxError) {
        return Math.abs(a - b) <= maxError;
    }

    public static boolean approxEquals(double a, double b) {
        return approxEquals(a, b, 1e-15);
    }

    static class Metric {
        private final double deriv;
        private final int dim;
        Metric(int dim, double deriv) {
            this.deriv = deriv;
            this.dim = dim;
        }
        double deriv() {
            return deriv;
        }
        double getValue(int level) {
            return StrictMath.scalb(deriv, dim * (1 - level));
        }
        int getClosestLevel(double value) {
            return getMinLevel(M_SQRT2 * value);
        }
        int getMinLevel(double value) {
            if (value <= 0) {
                return S2CellId.MAX_LEVEL;
            }
            int exponent = exp(value / ((1 << dim) * deriv));
            return Math.max(0,
                    Math.min(S2CellId.MAX_LEVEL, -((exponent - 1) >> (dim - 1))));
        }
        int getMaxLevel(double value) {
            if (value <= 0) {
                return S2CellId.MAX_LEVEL;
            }
            int exponent = exp((1 << dim) * deriv / value);
            return Math.max(0,
                    Math.min(S2CellId.MAX_LEVEL, ((exponent - 1) >> (dim - 1))));
        }

    }
}
