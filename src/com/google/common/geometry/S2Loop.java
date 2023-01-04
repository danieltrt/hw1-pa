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
import com.google.common.collect.Maps;
import com.google.common.geometry.S2EdgeIndex.DataEdgeIterator;
import com.google.common.geometry.S2EdgeUtil.EdgeCrosser;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.logging.Logger;

public final strictfp class S2Loop implements S2Region, Comparable<S2Loop> {
    private static final double MAX_INTERSECTION_ERROR = 1e-15;
    private static final Logger log = Logger.getLogger(S2Loop.class.getCanonicalName());
    private final S2Point[] vertices;
    private final int numVertices;
    private S2EdgeIndex index;
    private Map<S2Point, Integer> vertexToIndex;
    private int firstLogicalVertex;
    private S2LatLngRect bound;
    private boolean originInside;
    private int depth;

    S2Loop(final List<S2Point> vertices) {
        this.numVertices = vertices.size();
        this.vertices = new S2Point[numVertices];
        this.bound = S2LatLngRect.full();
        this.depth = 0;
        vertices.toArray(this.vertices);
        initOrigin();
        initBound();
        initFirstLogicalVertex();
    }

    public S2Loop(S2Cell cell) {
        this(cell, cell.getRectBound());
    }

    S2Loop(S2Cell cell, S2LatLngRect bound) {
        this.bound = bound;
        numVertices = 4;
        vertices = new S2Point[numVertices];
        vertexToIndex = null;
        index = null;
        depth = 0;
        for (int i = 0; i < 4; ++i) {
            vertices[i] = cell.getVertex(i);
        }
        initOrigin();
        initFirstLogicalVertex();
    }

    S2Loop(S2Loop src) {
        this.numVertices = src.numVertices();
        this.vertices = src.vertices.clone();
        this.vertexToIndex = src.vertexToIndex;
        this.index = src.index;
        this.firstLogicalVertex = src.firstLogicalVertex;
        this.bound = src.getRectBound();
        this.originInside = src.originInside;
        this.depth = src.depth();
    }

    public static boolean isValid(List<S2Point> vertices) {
        return new S2Loop(vertices).isValid();
    }

    int depth() {
        return depth;
    }

    void setDepth(int depth) {
        this.depth = depth;
    }

    boolean isHole() {
        return (depth & 1) != 0;
    }

    int sign() {
        return isHole() ? -1 : 1;
    }

    int numVertices() {
        return numVertices;
    }

    public S2Point vertex(int i) {
        try {
            return vertices[i >= vertices.length ? i - vertices.length : i];
        } catch (ArrayIndexOutOfBoundsException e) {
            throw new IllegalStateException("Invalid vertex index");
        }
    }

    @Override
    public int compareTo(S2Loop other) {
        if (numVertices() != other.numVertices()) {
            return this.numVertices() - other.numVertices();
        }
        int maxVertices = numVertices();
        int iThis = firstLogicalVertex;
        int iOther = other.firstLogicalVertex;
        for (int i = 0; i < maxVertices; ++i, ++iThis, ++iOther) {
            int compare = vertex(iThis).compareTo(other.vertex(iOther));
            if (compare != 0) {
                return compare;
            }
        }
        return 0;
    }

    private void initFirstLogicalVertex() {
        int first = 0;
        for (int i = 1; i < numVertices; ++i) {
            if (vertex(i).compareTo(vertex(first)) < 0) {
                first = i;
            }
        }
        firstLogicalVertex = first;
    }

    boolean isNormalized() {
        return getArea() <= 2 * S2.M_PI + 1e-14;
    }

    public void normalize() {
        if (!isNormalized()) {
            invert();
        }
    }

    void invert() {
        int last = numVertices() - 1;
        for (int i = (last - 1) / 2; i >= 0; --i) {
            S2Point t = vertices[i];
            vertices[i] = vertices[last - i];
            vertices[last - i] = t;
        }
        vertexToIndex = null;
        index = null;
        originInside ^= true;
        if (bound.lat().lo() > -S2.M_PI_2 && bound.lat().hi() < S2.M_PI_2) {
            bound = S2LatLngRect.full();
        } else {
            initBound();
        }
        initFirstLogicalVertex();
    }

    private S2AreaCentroid getAreaCentroid(boolean doCentroid) {
        S2Point centroid = null;
        if (numVertices() < 3) {
            return new S2AreaCentroid(0D, centroid);
        }
        S2Point origin = vertex(0);
        int axis = (origin.largestAbsComponent() + 1) % 3;
        double slightlyDisplaced = origin.get(axis) + S2.M_E * 1e-10;
        origin =
                new S2Point((axis == 0) ? slightlyDisplaced : origin.x,
                        (axis == 1) ? slightlyDisplaced : origin.y, (axis == 2) ? slightlyDisplaced : origin.z);
        origin = S2Point.normalize(origin);

        double areaSum = 0;
        S2Point centroidSum = new S2Point(0, 0, 0);
        for (int i = 1; i <= numVertices(); ++i) {
            areaSum += S2.signedArea(origin, vertex(i - 1), vertex(i));
            if (doCentroid) {
                S2Point trueCentroid = S2.trueCentroid(origin, vertex(i - 1), vertex(i));
                centroidSum = S2Point.add(centroidSum, trueCentroid);
            }
        }
        if (areaSum < 0) {
            areaSum += 4 * S2.M_PI;
        }
        if (doCentroid) {
            centroid = centroidSum;
        }
        return new S2AreaCentroid(areaSum, centroid);
    }

    S2AreaCentroid getAreaAndCentroid() {
        return getAreaCentroid(true);
    }

    public double getArea() {
        return getAreaCentroid(false).getArea();
    }

    S2Point getCentroid() {
        return getAreaCentroid(true).getCentroid();
    }

    public boolean contains(S2Loop b) {
        if (!bound.contains(b.getRectBound())) {
            return false;
        }
        if (!contains(b.vertex(0)) && findVertex(b.vertex(0)) < 0) {
            return false;
        }
        if (checkEdgeCrossings(b, new S2EdgeUtil.WedgeContains()) <= 0) {
            return false;
        }
        if (bound.union(b.getRectBound()).isFull()) {
            return !b.contains(vertex(0)) || b.findVertex(vertex(0)) >= 0;
        }
        return true;
    }

    public boolean intersects(S2Loop b) {
        if (!bound.intersects(b.getRectBound())) {
            return false;
        }
        if (b.getRectBound().lng().getLength() > bound.lng().getLength()) {
            return b.intersects(this);
        }
        if (contains(b.vertex(0)) && findVertex(b.vertex(0)) < 0) {
            return true;
        }
        if (checkEdgeCrossings(b, new S2EdgeUtil.WedgeIntersects()) < 0) {
            return true;
        }
        if (b.getRectBound().contains(bound)) {
            return b.contains(vertex(0)) && b.findVertex(vertex(0)) < 0;
        }
        return false;
    }

    boolean containsNested(S2Loop b) {
        if (!bound.contains(b.getRectBound())) {
            return false;
        }
        int m = findVertex(b.vertex(1));
        if (m < 0) {
            return contains(b.vertex(1));
        }
        return (new S2EdgeUtil.WedgeContains()).test(
                vertex(m - 1), vertex(m), vertex(m + 1), b.vertex(0), b.vertex(2)) > 0;
    }

    int containsOrCrosses(S2Loop b) {
        if (!bound.intersects(b.getRectBound())) {
            return 0;
        }
        int result = checkEdgeCrossings(b, new S2EdgeUtil.WedgeContainsOrCrosses());
        if (result <= 0) {
            return result;
        }
        if (!bound.contains(b.getRectBound())) {
            return 0;
        }
        if (!contains(b.vertex(0)) && findVertex(b.vertex(0)) < 0) {
            return 0;
        }
        return 1;
    }

    boolean boundaryApproxEquals(S2Loop b, double maxError) {
        if (numVertices() != b.numVertices()) {
            return false;
        }
        int maxVertices = numVertices();
        int iThis = firstLogicalVertex;
        int iOther = b.firstLogicalVertex;
        for (int i = 0; i < maxVertices; ++i, ++iThis, ++iOther) {
            if (!S2.approxEquals(vertex(iThis), b.vertex(iOther), maxError)) {
                return false;
            }
        }
        return true;
    }

    @Override
    public S2Cap getCapBound() {
        return bound.getCapBound();
    }

    @Override
    public S2LatLngRect getRectBound() {
        return bound;
    }

    @Override
    public boolean contains(S2Cell cell) {
        S2LatLngRect cellBound = cell.getRectBound();
        if (!bound.contains(cellBound)) {
            return false;
        }
        S2Loop cellLoop = new S2Loop(cell, cellBound);
        return contains(cellLoop);
    }

    @Override
    public boolean mayIntersect(S2Cell cell) {
        S2LatLngRect cellBound = cell.getRectBound();
        if (!bound.intersects(cellBound)) {
            return false;
        }
        return new S2Loop(cell, cellBound).intersects(this);
    }

    public boolean contains(S2Point p) {
        if (!bound.contains(p)) {
            return false;
        }
        boolean inside = originInside;
        S2Point origin = S2.origin();
        S2EdgeUtil.EdgeCrosser crosser = new S2EdgeUtil.EdgeCrosser(origin, p,
                vertices[numVertices - 1]);
        if (numVertices < 2000) {
            for (int i = 0; i < numVertices; i++) {
                inside ^= crosser.edgeOrVertexCrossing(vertices[i]);
            }
        } else {
            DataEdgeIterator it = getEdgeIterator(numVertices);
            int previousIndex = -2;
            for (it.getCandidates(origin, p); it.hasNext(); it.next()) {
                int ai = it.index();
                if (previousIndex != ai - 1) {
                    crosser.restartAt(vertices[ai]);
                }
                previousIndex = ai;
                inside ^= crosser.edgeOrVertexCrossing(vertex(ai + 1));
            }
        }
        return inside;
    }

    public S1Angle getDistance(S2Point p) {
        S2Point normalized = S2Point.normalize(p);
        S1Angle minDistance = S1Angle.radians(Math.PI);
        for (int i = 0; i < numVertices(); i++) {
            minDistance =
                    S1Angle.min(minDistance, S2EdgeUtil.getDistance(normalized, vertex(i), vertex(i + 1)));
        }
        return minDistance;
    }

    private DataEdgeIterator getEdgeIterator(int expectedQueries) {
        if (index == null) {
            index = new S2EdgeIndex() {
                @Override
                protected int getNumEdges() {
                    return numVertices;
                }

                @Override
                protected S2Point edgeFrom(int index) {
                    return vertex(index);
                }

                @Override
                protected S2Point edgeTo(int index) {
                    return vertex(index + 1);
                }
            };
        }
        index.predictAdditionalCalls(expectedQueries);
        return new S2EdgeIndex.DataEdgeIterator(index);
    }

    public boolean isValid() {
        if (numVertices < 3) {
            log.info("Degenerate loop");
            return false;
        }
        for (int i = 0; i < numVertices; ++i) {
            if (!S2.isUnitLength(vertex(i))) {
                log.info("Vertex " + i + " is not unit length");
                return false;
            }
        }
        HashMap<S2Point, Integer> vmap = Maps.newHashMap();
        for (int i = 0; i < numVertices; ++i) {
            Integer previousVertexIndex = vmap.put(vertex(i), i);
            if (previousVertexIndex != null) {
                log.info("Duplicate vertices: " + previousVertexIndex + " and " + i);
                return false;
            }
        }
        boolean crosses;
        DataEdgeIterator it = getEdgeIterator(numVertices);
        for (int a1 = 0; a1 < numVertices; a1++) {
            int a2 = (a1 + 1) % numVertices;
            EdgeCrosser crosser = new EdgeCrosser(vertex(a1), vertex(a2), vertex(0));
            int previousIndex = -2;
            for (it.getCandidates(vertex(a1), vertex(a2)); it.hasNext(); it.next()) {
                int b1 = it.index();
                int b2 = (b1 + 1) % numVertices;
                if (a1 != b2 && a2 != b1 && a1 != b1) {
                    double abc = S2.angle(vertex(a1), vertex(a2), vertex(b1));
                    boolean abcNearlyLinear = S2.approxEquals(abc, 0D, MAX_INTERSECTION_ERROR) ||
                            S2.approxEquals(abc, S2.M_PI, MAX_INTERSECTION_ERROR);
                    double abd = S2.angle(vertex(a1), vertex(a2), vertex(b2));
                    boolean abdNearlyLinear = S2.approxEquals(abd, 0D, MAX_INTERSECTION_ERROR) ||
                            S2.approxEquals(abd, S2.M_PI, MAX_INTERSECTION_ERROR);
                    if (abcNearlyLinear && abdNearlyLinear) {
                        continue;
                    }
                    if (previousIndex != b1) {
                        crosser.restartAt(vertex(b1));
                    }
                    crosses = crosser.robustCrossing(vertex(b2)) > 0;
                    previousIndex = b2;
                    if (crosses) {
                        log.info("Edges " + a1 + " and " + b1 + " cross");
                        log.info(String.format("Edge locations in degrees: " + "%s-%s and %s-%s",
                                new S2LatLng(vertex(a1)).toStringDegrees(),
                                new S2LatLng(vertex(a2)).toStringDegrees(),
                                new S2LatLng(vertex(b1)).toStringDegrees(),
                                new S2LatLng(vertex(b2)).toStringDegrees()));
                        return false;
                    }
                }
            }
        }
        return true;
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder("S2Loop, ");
        builder.append(vertices.length).append(" points. [");
        for (S2Point v : vertices) {
            builder.append(v.toString()).append(" ");
        }
        builder.append("]");
        return builder.toString();
    }

    private void initOrigin() {
        Preconditions.checkState(bound.contains(vertex(1)));
        originInside = false;
        boolean v1Inside = S2.orderedCCW(S2.ortho(vertex(1)), vertex(0), vertex(2), vertex(1));
        if (v1Inside != contains(vertex(1))) {
            originInside = true;
        }
    }

    private void initBound() {
        S2EdgeUtil.RectBounder bounder = new S2EdgeUtil.RectBounder();
        for (int i = 0; i <= numVertices(); ++i) {
            bounder.addPoint(vertex(i));
        }
        S2LatLngRect b = bounder.getBound();
        bound = S2LatLngRect.full();
        if (contains(new S2Point(0, 0, 1))) {
            b = new S2LatLngRect(new R1Interval(b.lat().lo(), S2.M_PI_2), S1Interval.full());
        }
        if (b.lng().isFull() && contains(new S2Point(0, 0, -1))) {
            b = new S2LatLngRect(new R1Interval(-S2.M_PI_2, b.lat().hi()), b.lng());
        }
        bound = b;
    }

    private int findVertex(S2Point p) {
        if (vertexToIndex == null) {
            vertexToIndex = new HashMap<>();
            for (int i = 1; i <= numVertices; i++) {
                vertexToIndex.put(vertex(i), i);
            }
        }
        Integer index = vertexToIndex.get(p);
        if (index == null) {
            return -1;
        } else {
            return index;
        }
    }

    private int checkEdgeCrossings(S2Loop b, S2EdgeUtil.WedgeRelation relation) {
        DataEdgeIterator it = getEdgeIterator(b.numVertices);
        int result = 1;
        for (int j = 0; j < b.numVertices(); ++j) {
            S2EdgeUtil.EdgeCrosser crosser =
                    new S2EdgeUtil.EdgeCrosser(b.vertex(j), b.vertex(j + 1), vertex(0));
            int previousIndex = -2;
            for (it.getCandidates(b.vertex(j), b.vertex(j + 1)); it.hasNext(); it.next()) {
                int i = it.index();
                if (previousIndex != i - 1) {
                    crosser.restartAt(vertex(i));
                }
                previousIndex = i;
                int crossing = crosser.robustCrossing(vertex(i + 1));
                if (crossing < 0) {
                    continue;
                }
                if (crossing > 0) {
                    return -1;
                }
                if (vertex(i + 1).equals(b.vertex(j + 1))) {
                    result = Math.min(result, relation.test(
                            vertex(i), vertex(i + 1), vertex(i + 2), b.vertex(j), b.vertex(j + 2)));
                    if (result < 0) {
                        return result;
                    }
                }
            }
        }
        return result;
    }
}
