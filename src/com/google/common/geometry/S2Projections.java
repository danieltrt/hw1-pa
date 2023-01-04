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

import com.google.common.geometry.S2.Metric;

public final strictfp class S2Projections {
    static final Metric AVG_AREA = new Metric(2, S2.M_PI / 6);
    static final Metric AVG_ANGLE_SPAN = new Metric(1, S2.M_PI / 4);
    static final double MAX_DIAG_ASPECT = Math.sqrt(3);
    private static final Projections S2_PROJECTION = Projections.S2_QUADRATIC_PROJECTION;
    static final Metric MIN_AREA = new Metric(2,
            S2_PROJECTION == Projections.S2_LINEAR_PROJECTION ? 1 / (3 * Math.sqrt(3)) :
                    S2_PROJECTION == Projections.S2_TAN_PROJECTION ? (S2.M_PI * S2.M_PI)
                            / (16 * S2.M_SQRT2) :
                            S2_PROJECTION == Projections.S2_QUADRATIC_PROJECTION
                                    ? 2 * S2.M_SQRT2 / 9 :
                                    0);
    static final Metric MAX_AREA = new Metric(2,
            S2_PROJECTION == Projections.S2_LINEAR_PROJECTION ? 1 :
                    S2_PROJECTION == Projections.S2_TAN_PROJECTION ? S2.M_PI * S2.M_PI / 16 :
                            S2_PROJECTION == Projections.S2_QUADRATIC_PROJECTION
                                    ? 0.65894981424079037 :
                                    0);
    static final Metric MIN_ANGLE_SPAN = new Metric(1,
            S2_PROJECTION == Projections.S2_LINEAR_PROJECTION ? 0.5 :
                    S2_PROJECTION == Projections.S2_TAN_PROJECTION ? S2.M_PI / 4 :
                            S2_PROJECTION == Projections.S2_QUADRATIC_PROJECTION ? 2. / 3 :
                                    0);
    static final Metric MAX_ANGLE_SPAN = new Metric(1,
            S2_PROJECTION == Projections.S2_LINEAR_PROJECTION ? 1 :
                    S2_PROJECTION == Projections.S2_TAN_PROJECTION ? S2.M_PI / 4 :
                            S2_PROJECTION == Projections.S2_QUADRATIC_PROJECTION
                                    ? 0.85244858959960922 :
                                    0);
    static final Metric MAX_WIDTH = new Metric(1, MAX_ANGLE_SPAN.deriv());
    static final Metric MAX_EDGE = new Metric(1, MAX_ANGLE_SPAN.deriv());
    static final Metric MIN_WIDTH = new Metric(1,
            (S2Projections.S2_PROJECTION == Projections.S2_LINEAR_PROJECTION ? 1 / Math.sqrt(6) :
                    S2_PROJECTION == Projections.S2_TAN_PROJECTION ? S2.M_PI / (4 * S2.M_SQRT2) :
                            S2_PROJECTION == Projections.S2_QUADRATIC_PROJECTION ? S2.M_SQRT2 / 3 :
                                    0));
    static final Metric AVG_WIDTH = new Metric(1,
            S2_PROJECTION == Projections.S2_LINEAR_PROJECTION ? 0.70572967292222848 :
                    S2_PROJECTION == Projections.S2_TAN_PROJECTION ? 0.71865931946258044 :
                            S2_PROJECTION == Projections.S2_QUADRATIC_PROJECTION
                                    ? 0.71726183644304969 :
                                    0);
    static final Metric MIN_EDGE = new Metric(1,
            S2_PROJECTION == Projections.S2_LINEAR_PROJECTION ? S2.M_SQRT2 / 3 :
                    S2_PROJECTION == Projections.S2_TAN_PROJECTION ? S2.M_PI / (4 * S2.M_SQRT2) :
                            S2_PROJECTION == Projections.S2_QUADRATIC_PROJECTION ? S2.M_SQRT2 / 3 :
                                    0);
    static final Metric AVG_EDGE = new Metric(1,
            S2_PROJECTION == Projections.S2_LINEAR_PROJECTION ? 0.72001709647780182 :
                    S2_PROJECTION == Projections.S2_TAN_PROJECTION ? 0.73083351627336963 :
                            S2_PROJECTION == Projections.S2_QUADRATIC_PROJECTION
                                    ? 0.72960687319305303 :
                                    0);
    static final Metric MIN_DIAG = new Metric(1,
            S2_PROJECTION == Projections.S2_LINEAR_PROJECTION ? S2.M_SQRT2 / 3 :
                    S2_PROJECTION == Projections.S2_TAN_PROJECTION ? S2.M_PI / (3 * S2.M_SQRT2) :
                            S2_PROJECTION == Projections.S2_QUADRATIC_PROJECTION
                                    ? 4 * S2.M_SQRT2 / 9 :
                                    0);
    static final Metric MAX_DIAG = new Metric(1,
            S2_PROJECTION == Projections.S2_LINEAR_PROJECTION ? S2.M_SQRT2 :
                    S2_PROJECTION == Projections.S2_TAN_PROJECTION ? S2.M_PI / Math.sqrt(6) :
                            S2_PROJECTION == Projections.S2_QUADRATIC_PROJECTION
                                    ? 1.2193272972170106 :
                                    0);
    static final Metric AVG_DIAG = new Metric(1,
            S2_PROJECTION == Projections.S2_LINEAR_PROJECTION ? 1.0159089332094063 :
                    S2_PROJECTION == Projections.S2_TAN_PROJECTION ? 1.0318115985978178 :
                            S2_PROJECTION == Projections.S2_QUADRATIC_PROJECTION
                                    ? 1.03021136949923584 :
                                    0);
    static final double MAX_EDGE_ASPECT =
            S2_PROJECTION == Projections.S2_LINEAR_PROJECTION ? S2.M_SQRT2 :
                    S2_PROJECTION == Projections.S2_TAN_PROJECTION ? S2.M_SQRT2 :
                            S2_PROJECTION == Projections.S2_QUADRATIC_PROJECTION ? 1.44261527445268292 :
                                    0;
    private S2Projections() {
    }

    static double stToUV(double s) {
        switch (S2_PROJECTION) {
            case S2_LINEAR_PROJECTION:
                return s;
            case S2_TAN_PROJECTION:
                s = Math.tan(S2.M_PI_4 * s);
                return s + (1.0 / (1L << 53)) * s;
            case S2_QUADRATIC_PROJECTION:
                if (s >= 0) {
                    return (1 / 3.) * ((1 + s) * (1 + s) - 1);
                } else {
                    return (1 / 3.) * (1 - (1 - s) * (1 - s));
                }
            default:
                throw new IllegalStateException("Invalid value for S2_PROJECTION");
        }
    }

    static double uvToST(double u) {
        switch (S2_PROJECTION) {
            case S2_LINEAR_PROJECTION:
                return u;
            case S2_TAN_PROJECTION:
                return (4 * S2.M_1_PI) * Math.atan(u);
            case S2_QUADRATIC_PROJECTION:
                if (u >= 0) {
                    return Math.sqrt(1 + 3 * u) - 1;
                } else {
                    return 1 - Math.sqrt(1 - 3 * u);
                }
            default:
                throw new IllegalStateException("Invalid value for S2_PROJECTION");
        }
    }

    static S2Point faceUvToXyz(int face, double u, double v) {
        switch (face) {
            case 0:
                return new S2Point(1, u, v);
            case 1:
                return new S2Point(-u, 1, v);
            case 2:
                return new S2Point(-u, -v, 1);
            case 3:
                return new S2Point(-1, -v, -u);
            case 4:
                return new S2Point(v, -1, -u);
            default:
                return new S2Point(v, u, -1);
        }
    }

    static R2Vector validFaceXyzToUv(int face, S2Point p) {
        double pu;
        double pv;
        switch (face) {
            case 0:
                pu = p.y / p.x;
                pv = p.z / p.x;
                break;
            case 1:
                pu = -p.x / p.y;
                pv = p.z / p.y;
                break;
            case 2:
                pu = -p.x / p.z;
                pv = -p.y / p.z;
                break;
            case 3:
                pu = p.z / p.x;
                pv = p.y / p.x;
                break;
            case 4:
                pu = p.z / p.y;
                pv = -p.x / p.y;
                break;
            default:
                pu = -p.y / p.z;
                pv = -p.x / p.z;
                break;
        }
        return new R2Vector(pu, pv);
    }

    static int xyzToFace(S2Point p) {
        int face = p.largestAbsComponent();
        if (p.get(face) < 0) {
            face += 3;
        }
        return face;
    }

    static R2Vector faceXyzToUv(int face, S2Point p) {
        if (face < 3) {
            if (p.get(face) <= 0) {
                return null;
            }
        } else {
            if (p.get(face - 3) >= 0) {
                return null;
            }
        }
        return validFaceXyzToUv(face, p);
    }

    static S2Point getUNorm(int face, double u) {
        switch (face) {
            case 0:
                return new S2Point(u, -1, 0);
            case 1:
                return new S2Point(1, u, 0);
            case 2:
                return new S2Point(1, 0, u);
            case 3:
                return new S2Point(-u, 0, 1);
            case 4:
                return new S2Point(0, -u, 1);
            default:
                return new S2Point(0, -1, -u);
        }
    }

    static S2Point getVNorm(int face, double v) {
        switch (face) {
            case 0:
                return new S2Point(-v, 0, 1);
            case 1:
                return new S2Point(0, -v, 1);
            case 2:
                return new S2Point(0, -1, -v);
            case 3:
                return new S2Point(v, -1, 0);
            case 4:
                return new S2Point(1, v, 0);
            default:
                return new S2Point(1, 0, v);
        }
    }

    static S2Point getNorm(int face) {
        return faceUvToXyz(face, 0, 0);
    }

    static S2Point getUAxis(int face) {
        switch (face) {
            case 0:
                return new S2Point(0, 1, 0);
            case 1:
                return new S2Point(-1, 0, 0);
            case 2:
                return new S2Point(-1, 0, 0);
            case 3:
                return new S2Point(0, 0, -1);
            case 4:
                return new S2Point(0, 0, -1);
            default:
                return new S2Point(0, 1, 0);
        }
    }

    static S2Point getVAxis(int face) {
        switch (face) {
            case 0:
                return new S2Point(0, 0, 1);
            case 1:
                return new S2Point(0, 0, 1);
            case 2:
                return new S2Point(0, -1, 0);
            case 3:
                return new S2Point(0, -1, 0);
            case 4:
                return new S2Point(1, 0, 0);
            default:
                return new S2Point(1, 0, 0);
        }
    }

    public enum Projections {
        S2_LINEAR_PROJECTION, S2_TAN_PROJECTION, S2_QUADRATIC_PROJECTION
    }
}
