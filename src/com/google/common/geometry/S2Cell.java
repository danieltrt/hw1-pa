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

public final strictfp class S2Cell implements S2Region {
    private static final int MAX_CELL_SIZE = 1 << S2CellId.MAX_LEVEL;
    private static final double MAX_ERROR = 1.0 / (1L << 51);
    private static final double POLE_MIN_LAT = Math.asin(Math.sqrt(1.0 / 3.0)) - MAX_ERROR;
    private byte face;
    private byte level;
    private byte orientation;
    private S2CellId cellId;
    private double[][] uv = new double[2][2];

    S2Cell() {
    }

    public S2Cell(S2CellId id) {
        init(id);
    }

    public S2Cell(S2Point p) {
        init(S2CellId.fromPoint(p));
    }

    public S2Cell(S2LatLng ll) {
        init(S2CellId.fromLatLng(ll));
    }

    public static S2Cell fromFacePosLevel(int face, byte pos, int level) {
        return new S2Cell(S2CellId.fromFacePosLevel(face, pos, level));
    }

    static double averageArea(int level) {
        return S2Projections.AVG_AREA.getValue(level);
    }

    public S2CellId id() {
        return cellId;
    }

    public int face() {
        return face;
    }

    public byte level() {
        return level;
    }

    byte orientation() {
        return orientation;
    }

    boolean isLeaf() {
        return level == S2CellId.MAX_LEVEL;
    }

    S2Point getVertex(int k) {
        return S2Point.normalize(getVertexRaw(k));
    }

    S2Point getVertexRaw(int k) {
        return S2Projections.faceUvToXyz(face, uv[0][(k >> 1) ^ (k & 1)], uv[1][k >> 1]);
    }

    S2Point getEdge(int k) {
        return S2Point.normalize(getEdgeRaw(k));
    }

    S2Point getEdgeRaw(int k) {
        switch (k) {
            case 0:
                return S2Projections.getVNorm(face, uv[1][0]);
            case 1:
                return S2Projections.getUNorm(face, uv[0][1]);
            case 2:
                return S2Point.neg(S2Projections.getVNorm(face, uv[1][1]));
            default:
                return S2Point.neg(S2Projections.getUNorm(face, uv[0][0]));
        }
    }

    boolean subdivide(S2Cell[] children) {
        if (cellId.isLeaf()) {
            return false;
        }
        R2Vector uvMid = getCenterUV();
        S2CellId id = cellId.childBegin();
        for (int pos = 0; pos < 4; ++pos, id = id.next()) {
            S2Cell child = children[pos];
            child.face = face;
            child.level = (byte) (level + 1);
            child.orientation = (byte) (orientation ^ S2.posToOrientation(pos));
            child.cellId = id;
            int ij = S2.posToIJ(orientation, pos);
            for (int d = 0; d < 2; ++d) {
                int m = 1 - ((ij >> (1 - d)) & 1);
                child.uv[d][m] = uvMid.get(d);
                child.uv[d][1 - m] = uv[d][1 - m];
            }
        }
        return true;
    }

    public S2Point getCenter() {
        return S2Point.normalize(getCenterRaw());
    }

    S2Point getCenterRaw() {
        return cellId.toPointRaw();
    }

    private R2Vector getCenterUV() {
        MutableInteger i = new MutableInteger(0);
        MutableInteger j = new MutableInteger(0);
        cellId.toFaceIJOrientation(i, j, null);
        int cellSize = 1 << (S2CellId.MAX_LEVEL - level);
        int si = (i.intValue() & -cellSize) * 2 + cellSize - MAX_CELL_SIZE;
        double x = S2Projections.stToUV((1.0 / MAX_CELL_SIZE) * si);
        int sj = (j.intValue() & -cellSize) * 2 + cellSize - MAX_CELL_SIZE;
        double y = S2Projections.stToUV((1.0 / MAX_CELL_SIZE) * sj);
        return new R2Vector(x, y);
    }

    double averageArea() {
        return averageArea(level);
    }

    double approxArea() {
        if (level < 2) {
            return averageArea(level);
        }
        double flatArea = 0.5 * S2Point.crossProd(
                S2Point.sub(getVertex(2), getVertex(0)), S2Point.sub(getVertex(3), getVertex(1))).norm();
        return flatArea * 2 / (1 + Math.sqrt(1 - Math.min(S2.M_1_PI * flatArea, 1.0)));
    }

    double exactArea() {
        S2Point v0 = getVertex(0);
        S2Point v1 = getVertex(1);
        S2Point v2 = getVertex(2);
        S2Point v3 = getVertex(3);
        return S2.area(v0, v1, v2) + S2.area(v0, v2, v3);
    }

    @Override
    public S2Region clone() {
        S2Cell clone = new S2Cell();
        clone.face = this.face;
        clone.level = this.level;
        clone.orientation = this.orientation;
        clone.uv = this.uv.clone();
        return clone;
    }

    @Override
    public S2Cap getCapBound() {
        double u = 0.5 * (uv[0][0] + uv[0][1]);
        double v = 0.5 * (uv[1][0] + uv[1][1]);
        S2Cap cap = S2Cap.fromAxisHeight(S2Point.normalize(S2Projections.faceUvToXyz(face, u, v)), 0);
        for (int k = 0; k < 4; ++k) {
            cap = cap.addPoint(getVertex(k));
        }
        return cap;
    }

    @Override
    public S2LatLngRect getRectBound() {
        if (level > 0) {
            double u = uv[0][0] + uv[0][1];
            double v = uv[1][0] + uv[1][1];
            int i = S2Projections.getUAxis(face).z == 0 ? (u < 0 ? 1 : 0) : (u > 0 ? 1 : 0);
            int j = S2Projections.getVAxis(face).z == 0 ? (v < 0 ? 1 : 0) : (v > 0 ? 1 : 0);
            R1Interval lat = R1Interval.fromPointPair(getLatitude(i, j), getLatitude(1 - i, 1 - j));
            lat = lat.expanded(MAX_ERROR).intersection(S2LatLngRect.fullLat());
            if (lat.lo() == -S2.M_PI_2 || lat.hi() == S2.M_PI_2) {
                return new S2LatLngRect(lat, S1Interval.full());
            }
            S1Interval lng = S1Interval.fromPointPair(getLongitude(i, 1 - j), getLongitude(1 - i, j));
            return new S2LatLngRect(lat, lng.expanded(MAX_ERROR));
        }
        switch (face) {
            case 0:
                return new S2LatLngRect(
                        new R1Interval(-S2.M_PI_4, S2.M_PI_4), new S1Interval(-S2.M_PI_4, S2.M_PI_4));
            case 1:
                return new S2LatLngRect(
                        new R1Interval(-S2.M_PI_4, S2.M_PI_4), new S1Interval(S2.M_PI_4, 3 * S2.M_PI_4));
            case 2:
                return new S2LatLngRect(
                        new R1Interval(POLE_MIN_LAT, S2.M_PI_2), new S1Interval(-S2.M_PI, S2.M_PI));
            case 3:
                return new S2LatLngRect(
                        new R1Interval(-S2.M_PI_4, S2.M_PI_4), new S1Interval(3 * S2.M_PI_4, -3 * S2.M_PI_4));
            case 4:
                return new S2LatLngRect(
                        new R1Interval(-S2.M_PI_4, S2.M_PI_4), new S1Interval(-3 * S2.M_PI_4, -S2.M_PI_4));
            default:
                return new S2LatLngRect(
                        new R1Interval(-S2.M_PI_2, -POLE_MIN_LAT), new S1Interval(-S2.M_PI, S2.M_PI));
        }
    }

    @Override
    public boolean mayIntersect(S2Cell cell) {
        return cellId.intersects(cell.cellId);
    }

    public boolean contains(S2Point p) {
        R2Vector uvPoint = S2Projections.faceXyzToUv(face, p);
        if (uvPoint == null) {
            return false;
        }
        return (uvPoint.x() >= uv[0][0] && uvPoint.x() <= uv[0][1]
                && uvPoint.y() >= uv[1][0] && uvPoint.y() <= uv[1][1]);
    }

    @Override
    public boolean contains(S2Cell cell) {
        return cellId.contains(cell.cellId);
    }

    private void init(S2CellId id) {
        cellId = id;
        MutableInteger[] ij = new MutableInteger[2];
        MutableInteger mOrientation = new MutableInteger(0);
        for (int d = 0; d < 2; ++d) {
            ij[d] = new MutableInteger(0);
        }
        face = (byte) id.toFaceIJOrientation(ij[0], ij[1], mOrientation);
        orientation = (byte) mOrientation.intValue();
        level = (byte) id.level();
        int cellSize = 1 << (S2CellId.MAX_LEVEL - level);
        for (int d = 0; d < 2; ++d) {
            int sijLo = (ij[d].intValue() & -cellSize) * 2 - MAX_CELL_SIZE;
            int sijHi = sijLo + cellSize * 2;
            uv[d][0] = S2Projections.stToUV((1.0 / MAX_CELL_SIZE) * sijLo);
            uv[d][1] = S2Projections.stToUV((1.0 / MAX_CELL_SIZE) * sijHi);
        }
    }

    private double getLatitude(int i, int j) {
        S2Point p = S2Projections.faceUvToXyz(face, uv[0][i], uv[1][j]);
        return Math.atan2(p.z, Math.sqrt(p.x * p.x + p.y * p.y));
    }

    private double getLongitude(int i, int j) {
        S2Point p = S2Projections.faceUvToXyz(face, uv[0][i], uv[1][j]);
        return Math.atan2(p.y, p.x);
    }

    @Override
    public String toString() {
        return "[" + face + ", " + level + ", " + orientation + ", " + cellId + "]";
    }

    @Override
    public int hashCode() {
        int value = 17;
        value = 37 * (37 * (37 * value + face) + orientation) + level;
        return 37 * value + id().hashCode();
    }

    @Override
    public boolean equals(Object that) {
        if (that instanceof S2Cell) {
            S2Cell thatCell = (S2Cell) that;
            return this.face == thatCell.face && this.level == thatCell.level
                    && this.orientation == thatCell.orientation && this.cellId.equals(thatCell.cellId);
        }
        return false;
    }
}
