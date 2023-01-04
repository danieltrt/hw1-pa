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

import com.google.common.base.Objects;
import com.google.common.base.Preconditions;

import java.util.Arrays;
import java.util.List;
import java.util.logging.Logger;

public final strictfp class S2Polyline implements S2Region {
    private static final Logger log = Logger.getLogger(S2Polyline.class.getCanonicalName());
    private final int numVertices;
    private final S2Point[] vertices;

    S2Polyline(List<S2Point> vertices) {
        this.numVertices = vertices.size();
        this.vertices = vertices.toArray(new S2Point[numVertices]);
    }

    public S2Polyline(S2Polyline src) {
        this.numVertices = src.numVertices();
        this.vertices = src.vertices.clone();
    }
    public boolean isValid(List<S2Point> vertices) {
        int n = vertices.size();
        for (int i = 0; i < n; ++i) {
            if (!S2.isUnitLength(vertices.get(i))) {
                log.info("Vertex " + i + " is not unit length");
                return false;
            }
        }
        for (int i = 1; i < n; ++i) {
            if (vertices.get(i - 1).equals(vertices.get(i))
                    || vertices.get(i - 1).equals(S2Point.neg(vertices.get(i)))) {
                log.info("Vertices " + (i - 1) + " and " + i + " are identical or antipodal");
                return false;
            }
        }
        return true;
    }

    int numVertices() {
        return numVertices;
    }

    public S2Point vertex(int k) {
        return vertices[k];
    }

    S1Angle getArclengthAngle() {
        double lengthSum = 0;
        for (int i = 1; i < numVertices(); ++i) {
            lengthSum += vertex(i - 1).angle(vertex(i));
        }
        return S1Angle.radians(lengthSum);
    }

    S2Point interpolate(double fraction) {
        if (fraction <= 0) {
            return vertex(0);
        }
        double lengthSum = 0;
        for (int i = 1; i < numVertices(); ++i) {
            lengthSum += vertex(i - 1).angle(vertex(i));
        }
        double target = fraction * lengthSum;
        for (int i = 1; i < numVertices(); ++i) {
            double length = vertex(i - 1).angle(vertex(i));
            if (target < length) {
                double f = Math.sin(target) / Math.sin(length);
                return S2Point.add(S2Point.mul(vertex(i - 1), (Math.cos(target) - f * Math.cos(length))),
                        S2Point.mul(vertex(i), f));
            }
            target -= length;
        }
        return vertex(numVertices() - 1);
    }

    @Override
    public S2Cap getCapBound() {
        return getRectBound().getCapBound();
    }

    @Override
    public S2LatLngRect getRectBound() {
        S2EdgeUtil.RectBounder bounder = new S2EdgeUtil.RectBounder();
        for (int i = 0; i < numVertices(); ++i) {
            bounder.addPoint(vertex(i));
        }
        return bounder.getBound();
    }

    @Override
    public boolean contains(S2Cell cell) {
        throw new UnsupportedOperationException(
                "'containment' is not numerically well-defined " + "except at the polyline vertices");
    }

    @Override
    public boolean mayIntersect(S2Cell cell) {
        if (numVertices() == 0) {
            return false;
        }
        for (int i = 0; i < numVertices(); ++i) {
            if (cell.contains(vertex(i))) {
                return true;
            }
        }
        S2Point[] cellVertices = new S2Point[4];
        for (int i = 0; i < 4; ++i) {
            cellVertices[i] = cell.getVertex(i);
        }
        for (int j = 0; j < 4; ++j) {
            S2EdgeUtil.EdgeCrosser crosser =
                    new S2EdgeUtil.EdgeCrosser(cellVertices[j], cellVertices[(j + 1) & 3], vertex(0));
            for (int i = 1; i < numVertices(); ++i) {
                if (crosser.robustCrossing(vertex(i)) >= 0) {
                    return true;
                }
            }
        }
        return false;
    }

    int getNearestEdgeIndex(S2Point point) {
        Preconditions.checkState(numVertices() > 0, "Empty polyline");
        if (numVertices() == 1) {
            return 0;
        }
        S1Angle minDistance = S1Angle.radians(10);
        int minIndex = -1;
        for (int i = 0; i < numVertices() - 1; ++i) {
            S1Angle distanceToSegment = S2EdgeUtil.getDistance(point, vertex(i), vertex(i + 1));
            if (distanceToSegment.lessThan(minDistance)) {
                minDistance = distanceToSegment;
                minIndex = i;
            }
        }
        return minIndex;
    }

    S2Point projectToEdge(S2Point point, int index) {
        Preconditions.checkState(numVertices() > 0, "Empty polyline");
        Preconditions.checkState(numVertices() == 1 || index < numVertices() - 1, "Invalid edge index");
        if (numVertices() == 1) {
            return vertex(0);
        }
        return S2EdgeUtil.getClosestPoint(point, vertex(index), vertex(index + 1));
    }

    @Override
    public boolean equals(Object that) {
        if (!(that instanceof S2Polyline)) {
            return false;
        }
        S2Polyline thatPolygon = (S2Polyline) that;
        if (numVertices != thatPolygon.numVertices) {
            return false;
        }
        for (int i = 0; i < vertices.length; i++) {
            if (!vertices[i].equals(thatPolygon.vertices[i])) {
                return false;
            }
        }
        return true;
    }

    @Override
    public int hashCode() {
        return Objects.hashCode(numVertices, Arrays.deepHashCode(vertices));
    }
}
