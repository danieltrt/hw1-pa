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
import com.google.common.collect.*;

import java.util.*;
import java.util.logging.Logger;

public final strictfp class S2Polygon implements S2Region, Comparable<S2Polygon> {
    private static final Logger log = Logger.getLogger(S2Polygon.class.getCanonicalName());
    private List<S2Loop> loops;
    private S2LatLngRect bound;
    private boolean hasHoles;
    private int numVertices;

    S2Polygon() {
        this.loops = Lists.newArrayList();
        this.bound = S2LatLngRect.empty();
        this.hasHoles = false;
        this.numVertices = 0;
    }

    S2Polygon(List<S2Loop> loops) {
        this.loops = Lists.newArrayList();
        this.bound = S2LatLngRect.empty();
        init(loops);
    }

    private S2Polygon(S2Loop loop) {
        this.loops = Lists.newArrayList();
        this.bound = loop.getRectBound();
        this.hasHoles = false;
        this.numVertices = loop.numVertices();

        loops.add(loop);
    }

    S2Polygon(S2Polygon src) {
        this.loops = Lists.newArrayList();
        this.bound = src.getRectBound();
        this.hasHoles = src.hasHoles;
        this.numVertices = src.numVertices;

        for (int i = 0; i < src.numLoops(); ++i) {
            loops.add(new S2Loop(src.loop(i)));
        }
    }

    public static boolean isValid(final List<S2Loop> loops) {
        if (loops.size() > 1) {
            Map<UndirectedEdge, LoopVertexIndexPair> edges = Maps.newHashMap();
            for (int i = 0; i < loops.size(); ++i) {
                S2Loop lp = loops.get(i);
                for (int j = 0; j < lp.numVertices(); ++j) {
                    UndirectedEdge key = new UndirectedEdge(lp.vertex(j), lp.vertex(j + 1));
                    LoopVertexIndexPair value = new LoopVertexIndexPair(i, j);
                    if (edges.containsKey(key)) {
                        LoopVertexIndexPair other = edges.get(key);
                        log.info(
                                "Duplicate edge: loop " + i + ", edge " + j + " and loop " + other.getLoopIndex()
                                        + ", edge " + other.getVertexIndex());
                        return false;
                    } else {
                        edges.put(key, value);
                    }
                }
            }
        }
        for (int i = 0; i < loops.size(); ++i) {
            if (!loops.get(i).isNormalized()) {
                log.info("Loop " + i + " encloses more than half the sphere");
                return false;
            }
            for (int j = i + 1; j < loops.size(); ++j) {
                if (loops.get(i).containsOrCrosses(loops.get(j)) < 0) {
                    log.info("Loop " + i + " crosses loop " + j);
                    return false;
                }
            }
        }
        return true;
    }

    private static void addIntersection(S2Point a0,
                                        S2Point a1,
                                        S2Point b0,
                                        S2Point b1,
                                        boolean addSharedEdges,
                                        int crossing,
                                        List<ParametrizedS2Point> intersections) {
        if (crossing > 0) {
            S2Point x = S2EdgeUtil.getIntersection(a0, a1, b0, b1);
            double t = S2EdgeUtil.getDistanceFraction(x, a0, a1);
            intersections.add(new ParametrizedS2Point(t, x));
        } else if (S2EdgeUtil.vertexCrossing(a0, a1, b0, b1)) {
            double t = (a0 == b0 || a0 == b1) ? 0 : 1;
            if (!addSharedEdges && a1 == b1) {
                t = 1;
            }
            intersections.add(new ParametrizedS2Point(t, t == 0 ? a0 : a1));
        }
    }

    private static void clipEdge(final S2Point a0, final S2Point a1, S2LoopSequenceIndex bIndex,
                                 boolean addSharedEdges, List<ParametrizedS2Point> intersections) {
        S2LoopSequenceIndex.DataEdgeIterator it = new S2LoopSequenceIndex.DataEdgeIterator(bIndex);
        it.getCandidates(a0, a1);
        S2EdgeUtil.EdgeCrosser crosser = new S2EdgeUtil.EdgeCrosser(a0, a1, a0);
        S2Point from;
        S2Point to = null;
        for (; it.hasNext(); it.next()) {
            S2Point previousTo = to;
            S2Edge fromTo = bIndex.edgeFromTo(it.index());
            from = fromTo.getStart();
            to = fromTo.getEnd();
            if (previousTo != from) {
                crosser.restartAt(from);
            }
            int crossing = crosser.robustCrossing(to);
            if (crossing < 0) {
                continue;
            }
            addIntersection(a0, a1, from, to, addSharedEdges, crossing, intersections);
        }
    }

    private static void clipBoundary(final S2Polygon a,
                                     boolean reverseA,
                                     final S2Polygon b,
                                     boolean reverseB,
                                     boolean invertB,
                                     boolean addSharedEdges,
                                     S2PolygonBuilder builder) {
        S2PolygonIndex bIndex = new S2PolygonIndex(b, reverseB);
        bIndex.predictAdditionalCalls(a.getNumVertices());
        List<ParametrizedS2Point> intersections = Lists.newArrayList();
        for (S2Loop aLoop : a.loops) {
            int n = aLoop.numVertices();
            int dir = (aLoop.isHole() ^ reverseA) ? -1 : 1;
            boolean inside = b.contains(aLoop.vertex(0)) ^ invertB;
            for (int j = (dir > 0) ? 0 : n; n > 0; --n, j += dir) {
                S2Point a0 = aLoop.vertex(j);
                S2Point a1 = aLoop.vertex(j + dir);
                intersections.clear();
                clipEdge(a0, a1, bIndex, addSharedEdges, intersections);
                if (inside) {
                    intersections.add(new ParametrizedS2Point(0.0, a0));
                }
                inside = ((intersections.size() & 0x1) == 0x1);
                if (inside) {
                    intersections.add(new ParametrizedS2Point(1.0, a1));
                }
                Collections.sort(intersections);
                for (int size = intersections.size(), i = 1; i < size; i += 2) {
                    builder.addEdge(intersections.get(i - 1).getPoint(), intersections.get(i).getPoint());
                }
            }
        }
    }

    static S2Polygon destructiveUnion(List<S2Polygon> polygons) {
        return destructiveUnionSloppy(polygons, S2EdgeUtil.DEFAULT_INTERSECTION_TOLERANCE);
    }

    static S2Polygon destructiveUnionSloppy(
            List<S2Polygon> polygons, S1Angle vertexMergeRadius) {
        TreeMultimap<Integer, S2Polygon> queue = TreeMultimap.create();
        for (S2Polygon polygon : polygons) {
            queue.put(polygon.getNumVertices(), polygon);
        }
        polygons.clear();
        Set<Map.Entry<Integer, S2Polygon>> queueSet = queue.entries();
        while (queueSet.size() > 1) {
            queueSet = queue.entries();
            Iterator<Map.Entry<Integer, S2Polygon>> smallestIter = queueSet.iterator();
            Map.Entry<Integer, S2Polygon> smallest = smallestIter.next();
            int aSize = smallest.getKey();
            S2Polygon aPolygon = smallest.getValue();
            smallestIter.remove();
            smallest = smallestIter.next();
            int bSize = smallest.getKey();
            S2Polygon bPolygon = smallest.getValue();
            smallestIter.remove();
            S2Polygon unionPolygon = new S2Polygon();
            unionPolygon.initToUnionSloppy(aPolygon, bPolygon, vertexMergeRadius);
            int unionSize = aSize + bSize;
            queue.put(unionSize, unionPolygon);
        }
        if (queue.isEmpty()) {
            return new S2Polygon();
        } else {
            return queue.get(queue.asMap().firstKey()).first();
        }
    }

    private static void sortValueLoops(Map<S2Loop, List<S2Loop>> loopMap) {
        for (S2Loop key : loopMap.keySet()) {
            Collections.sort(loopMap.get(key));
        }
    }

    private static void insertLoop(S2Loop newLoop, S2Loop parent, Map<S2Loop, List<S2Loop>> loopMap) {
        List<S2Loop> children = loopMap.computeIfAbsent(parent, k -> Lists.newArrayList());
        for (S2Loop child : children) {
            if (child.containsNested(newLoop)) {
                insertLoop(newLoop, child, loopMap);
                return;
            }
        }
        List<S2Loop> newChildren = loopMap.get(newLoop);
        for (int i = 0; i < children.size(); ) {
            S2Loop child = children.get(i);
            if (newLoop.containsNested(child)) {
                if (newChildren == null) {
                    newChildren = Lists.newArrayList();
                    loopMap.put(newLoop, newChildren);
                }
                newChildren.add(child);
                children.remove(i);
            } else {
                ++i;
            }
        }
        children.add(newLoop);
    }

    @Override
    public int compareTo(S2Polygon other) {
        if (this.numLoops() != other.numLoops()) {
            return this.numLoops() - other.numLoops();
        }
        for (int i = 0; i < this.numLoops(); ++i) {
            int compare = this.loops.get(i).compareTo(other.loops.get(i));
            if (compare != 0) {
                return compare;
            }
        }
        return 0;
    }

    void init(List<S2Loop> loops) {
        Map<S2Loop, List<S2Loop>> loopMap = Maps.newHashMap();
        loopMap.put(null, Lists.newArrayList());
        for (S2Loop loop : loops) {
            insertLoop(loop, null, loopMap);
            this.numVertices += loop.numVertices();
        }
        loops.clear();
        sortValueLoops(loopMap);
        initLoop(null, -1, loopMap);
        hasHoles = false;
        bound = S2LatLngRect.empty();
        for (int i = 0; i < numLoops(); ++i) {
            if (loop(i).sign() < 0) {
                hasHoles = true;
            } else {
                bound = bound.union(loop(i).getRectBound());
            }
        }
    }

    void release(List<S2Loop> loops) {
        loops.addAll(this.loops);
        this.loops.clear();
        bound = S2LatLngRect.empty();
        hasHoles = false;
        numVertices = 0;
    }

    int numLoops() {
        return loops.size();
    }

    S2Loop loop(int k) {
        return loops.get(k);
    }

    private int getParent(int k) {
        int depth = loop(k).depth();
        if (depth == 0) {
            return -1;
        }
        while (--k >= 0 && loop(k).depth() >= depth) { }
        return k;
    }

    public int getLastDescendant(int k) {
        if (k < 0) {
            return numLoops() - 1;
        }
        int depth = loop(k).depth();
        while (++k < numLoops() && loop(k).depth() > depth) { }
        return k - 1;
    }

    private S2AreaCentroid getAreaCentroid(boolean doCentroid) {
        double areaSum = 0;
        S2Point centroidSum = new S2Point(0, 0, 0);
        for (int i = 0; i < numLoops(); ++i) {
            S2AreaCentroid areaCentroid = doCentroid ? loop(i).getAreaAndCentroid() : null;
            double loopArea = doCentroid ? areaCentroid.getArea() : loop(i).getArea();
            int loopSign = loop(i).sign();
            areaSum += loopSign * loopArea;
            if (doCentroid) {
                S2Point currentCentroid = areaCentroid.getCentroid();
                assert currentCentroid != null;
                centroidSum =
                        new S2Point(centroidSum.x + loopSign * currentCentroid.x,
                                centroidSum.y + loopSign * currentCentroid.y,
                                centroidSum.z + loopSign * currentCentroid.z);
            }
        }
        return new S2AreaCentroid(areaSum, doCentroid ? centroidSum : null);
    }

    public S2AreaCentroid getAreaAndCentroid() {
        return getAreaCentroid(true);
    }

    public double getArea() {
        return getAreaCentroid(false).getArea();
    }

    public S2Point getCentroid() {
        return getAreaCentroid(true).getCentroid();
    }

    public S1Angle getDistance(S2Point p) {
        if (contains(p)) {
            return S1Angle.radians(0);
        }
        S1Angle minDistance = S1Angle.radians(Math.PI);
        for (int i = 0; i < numLoops(); i++) {
            minDistance = S1Angle.min(minDistance, loop(i).getDistance(p));
        }
        return minDistance;
    }

    public boolean contains(S2Polygon b) {
        if (numLoops() == 1 && b.numLoops() == 1) {
            return loop(0).contains(b.loop(0));
        }
        if (!bound.contains(b.getRectBound())) {
            if (!bound.lng().union(b.getRectBound().lng()).isFull()) {
                return false;
            }
        }
        if (!hasHoles && !b.hasHoles) {
            for (int j = 0; j < b.numLoops(); ++j) {
                if (!anyLoopContains(b.loop(j))) {
                    return false;
                }
            }
            return true;
        }
        return containsAllShells(b) && b.excludesAllHoles(this);
    }

    public boolean intersects(S2Polygon b) {
        if (numLoops() == 1 && b.numLoops() == 1) {
            return loop(0).intersects(b.loop(0));
        }
        if (!bound.intersects(b.getRectBound())) {
            return false;
        }
        if (!hasHoles && !b.hasHoles) {
            for (int i = 0; i < numLoops(); ++i) {
                for (int j = 0; j < b.numLoops(); ++j) {
                    if (loop(i).intersects(b.loop(j))) {
                        return true;
                    }
                }
            }
            return false;
        }
        return intersectsAnyShell(b) || b.intersectsAnyShell(this);
    }

    private int getNumVertices() {
        return this.numVertices;
    }

    public void initToIntersection(final S2Polygon a, final S2Polygon b) {
        initToIntersectionSloppy(a, b, S2EdgeUtil.DEFAULT_INTERSECTION_TOLERANCE);
    }

    private void initToIntersectionSloppy(
            final S2Polygon a, final S2Polygon b, S1Angle vertexMergeRadius) {
        Preconditions.checkState(numLoops() == 0);
        if (!a.bound.intersects(b.bound)) {
            return;
        }
        S2PolygonBuilder.Options options = S2PolygonBuilder.Options.DIRECTED_XOR;
        options.setMergeDistance(vertexMergeRadius);
        S2PolygonBuilder builder = new S2PolygonBuilder(options);
        clipBoundary(a, false, b, false, false, true, builder);
        clipBoundary(b, false, a, false, false, false, builder);
        if (!builder.assemblePolygon(this, null)) {
            log.severe("Bad directed edges");
        }
    }

    void initToUnion(final S2Polygon a, final S2Polygon b) {
        initToUnionSloppy(a, b, S2EdgeUtil.DEFAULT_INTERSECTION_TOLERANCE);
    }

    private void initToUnionSloppy(final S2Polygon a, final S2Polygon b, S1Angle vertexMergeRadius) {
        Preconditions.checkState(numLoops() == 0);
        S2PolygonBuilder.Options options = S2PolygonBuilder.Options.DIRECTED_XOR;
        options.setMergeDistance(vertexMergeRadius);
        S2PolygonBuilder builder = new S2PolygonBuilder(options);
        clipBoundary(a, false, b, false, true, true, builder);
        clipBoundary(b, false, a, false, true, false, builder);
        if (!builder.assemblePolygon(this, null)) {
            log.severe("Bad directed edges");
        }
    }

    boolean isNormalized() {
        Multiset<S2Point> vertices = HashMultiset.create();
        S2Loop lastParent = null;
        for (int i = 0; i < numLoops(); ++i) {
            S2Loop child = loop(i);
            if (child.depth() == 0) {
                continue;
            }
            S2Loop parent = loop(getParent(i));
            if (parent != lastParent) {
                vertices.clear();
                for (int j = 0; j < parent.numVertices(); ++j) {
                    vertices.add(parent.vertex(j));
                }
                lastParent = parent;
            }
            int count = 0;
            for (int j = 0; j < child.numVertices(); ++j) {
                if (vertices.count(child.vertex(j)) > 0) {
                    ++count;
                }
            }
            if (count > 1) {
                return false;
            }
        }
        return true;
    }

    boolean boundaryApproxEquals(S2Polygon b, double maxError) {
        if (numLoops() != b.numLoops()) {
            log.severe(
                    "!= loops: " + numLoops() + " vs. " + b.numLoops());
            return false;
        }
        for (int i = 0; i < numLoops(); ++i) {
            S2Loop aLoop = loop(i);
            boolean success = false;
            for (int j = 0; j < numLoops(); ++j) {
                S2Loop bLoop = b.loop(j);
                if (bLoop.depth() == aLoop.depth() && bLoop.boundaryApproxEquals(aLoop, maxError)) {
                    success = true;
                    break;
                }
            }
            if (!success) {
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
        if (numLoops() == 1) {
            return loop(0).contains(cell);
        }
        S2LatLngRect cellBound = cell.getRectBound();
        if (!bound.contains(cellBound)) {
            return false;
        }
        S2Loop cellLoop = new S2Loop(cell, cellBound);
        S2Polygon cellPoly = new S2Polygon(cellLoop);
        return contains(cellPoly);
    }

    @Override
    public boolean mayIntersect(S2Cell cell) {
        if (numLoops() == 1) {
            return loop(0).mayIntersect(cell);
        }
        S2LatLngRect cellBound = cell.getRectBound();
        if (!bound.intersects(cellBound)) {
            return false;
        }
        S2Loop cellLoop = new S2Loop(cell, cellBound);
        S2Polygon cellPoly = new S2Polygon(cellLoop);
        return intersects(cellPoly);
    }

    public boolean contains(S2Point p) {
        if (numLoops() == 1) {
            return loop(0).contains(p);
        }
        if (!bound.contains(p)) {
            return false;
        }
        boolean inside = false;
        for (int i = 0; i < numLoops(); ++i) {
            inside ^= loop(i).contains(p);
            if (inside && !hasHoles) {
                break;
            }
        }
        return inside;
    }

    private void initLoop(S2Loop loop, int depth, Map<S2Loop, List<S2Loop>> loopMap) {
        if (loop != null) {
            loop.setDepth(depth);
            loops.add(loop);
        }
        List<S2Loop> children = loopMap.get(loop);
        if (children != null) {
            for (S2Loop child : children) {
                initLoop(child, depth + 1, loopMap);
            }
        }
    }

    private int containsOrCrosses(S2Loop b) {
        boolean inside = false;
        for (int i = 0; i < numLoops(); ++i) {
            int result = loop(i).containsOrCrosses(b);
            if (result < 0) {
                return -1;
            }
            if (result > 0) {
                inside ^= true;
            }
        }
        return inside ? 1 : 0;
    }

    private boolean anyLoopContains(S2Loop b) {
        for (int i = 0; i < numLoops(); ++i) {
            if (loop(i).contains(b)) {
                return true;
            }
        }
        return false;
    }

    private boolean containsAllShells(S2Polygon b) {
        for (int j = 0; j < b.numLoops(); ++j) {
            if (b.loop(j).sign() < 0) {
                continue;
            }
            if (containsOrCrosses(b.loop(j)) <= 0) {
                return false;
            }
        }
        return true;
    }

    private boolean excludesAllHoles(S2Polygon b) {
        for (int j = 0; j < b.numLoops(); ++j) {
            if (b.loop(j).sign() > 0) {
                continue;
            }
            if (containsOrCrosses(b.loop(j)) != 0) {
                return false;
            }
        }
        return true;
    }

    private boolean intersectsAnyShell(S2Polygon b) {
        for (int j = 0; j < b.numLoops(); ++j) {
            if (b.loop(j).sign() < 0) {
                continue;
            }
            if (containsOrCrosses(b.loop(j)) != 0) {
                return true;
            }
        }
        return false;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("Polygon: (").append(numLoops()).append(") loops:\n");
        for (int i = 0; i < numLoops(); ++i) {
            S2Loop s2Loop = loop(i);
            sb.append("loop <\n");
            for (int v = 0; v < s2Loop.numVertices(); ++v) {
                S2Point s2Point = s2Loop.vertex(v);
                sb.append(s2Point.toDegreesString());
                sb.append("\n");
            }
            sb.append(">\n");
        }
        return sb.toString();
    }

    private abstract static class S2LoopSequenceIndex extends S2EdgeIndex {
        private final int[] indexToLoop;

        private final int[] loopToFirstIndex;

        S2LoopSequenceIndex(int[] numVertices) {
            int totalEdges = 0;
            for (int edges : numVertices) {
                totalEdges += edges;
            }
            indexToLoop = new int[totalEdges];
            loopToFirstIndex = new int[numVertices.length];
            totalEdges = 0;
            for (int j = 0; j < numVertices.length; j++) {
                loopToFirstIndex[j] = totalEdges;
                for (int i = 0; i < numVertices[j]; i++) {
                    indexToLoop[totalEdges] = j;
                    totalEdges++;
                }
            }
        }

        final LoopVertexIndexPair decodeIndex(int index) {
            int loopIndex = indexToLoop[index];
            int vertexInLoop = index - loopToFirstIndex[loopIndex];
            return new LoopVertexIndexPair(loopIndex, vertexInLoop);
        }

        public abstract S2Edge edgeFromTo(int index);

        @Override
        public final int getNumEdges() {
            return indexToLoop.length;
        }

        @Override
        public S2Point edgeFrom(int index) {
            S2Edge fromTo = edgeFromTo(index);
            return fromTo.getStart();
        }

        @Override
        protected S2Point edgeTo(int index) {
            S2Edge fromTo = edgeFromTo(index);
            return fromTo.getEnd();
        }
    }

    private static final class S2PolygonIndex extends S2LoopSequenceIndex {
        private final S2Polygon poly;
        private final boolean reverse;

        S2PolygonIndex(S2Polygon poly, boolean reverse) {
            super(getVertices(poly));
            this.poly = poly;
            this.reverse = reverse;
        }

        private static int[] getVertices(S2Polygon poly) {
            int[] vertices = new int[poly.numLoops()];
            for (int i = 0; i < vertices.length; i++) {
                vertices[i] = poly.loop(i).numVertices();
            }
            return vertices;
        }

        @Override
        public S2Edge edgeFromTo(int index) {
            LoopVertexIndexPair indices = decodeIndex(index);
            int loopIndex = indices.getLoopIndex();
            int vertexInLoop = indices.getVertexIndex();
            S2Loop loop = poly.loop(loopIndex);
            int fromIndex;
            int toIndex;
            if (loop.isHole() ^ reverse) {
                fromIndex = loop.numVertices() - 1 - vertexInLoop;
                toIndex = 2 * loop.numVertices() - 2 - vertexInLoop;
            } else {
                fromIndex = vertexInLoop;
                toIndex = vertexInLoop + 1;
            }
            S2Point from = loop.vertex(fromIndex);
            S2Point to = loop.vertex(toIndex);
            return new S2Edge(from, to);
        }
    }

    private static final class UndirectedEdge {
        private final S2Point a;
        private final S2Point b;

        UndirectedEdge(S2Point start, S2Point end) {
            this.a = start;
            this.b = end;
        }

        S2Point getStart() {
            return a;
        }

        public S2Point getEnd() {
            return b;
        }

        @Override
        public String toString() {
            return String.format("Edge: (%s <-> %s)\n   or [%s <-> %s]",
                    a.toDegreesString(), b.toDegreesString(), a, b);
        }

        @Override
        public boolean equals(Object o) {
            if (!(o instanceof UndirectedEdge)) {
                return false;
            }
            UndirectedEdge other = (UndirectedEdge) o;
            return ((getStart().equals(other.getStart()) && getEnd().equals(other.getEnd()))
                    || (getStart().equals(other.getEnd()) && getEnd().equals(other.getStart())));
        }

        @Override
        public int hashCode() {
            return getStart().hashCode() + getEnd().hashCode();
        }
    }

    private static final class LoopVertexIndexPair {
        private final int loopIndex;
        private final int vertexIndex;

        LoopVertexIndexPair(int loopIndex, int vertexIndex) {
            this.loopIndex = loopIndex;
            this.vertexIndex = vertexIndex;
        }

        int getLoopIndex() {
            return loopIndex;
        }

        int getVertexIndex() {
            return vertexIndex;
        }
    }

    private static final class ParametrizedS2Point implements Comparable<ParametrizedS2Point> {
        private final double time;
        private final S2Point point;

        ParametrizedS2Point(double time, S2Point point) {
            this.time = time;
            this.point = point;
        }

        public double getTime() {
            return time;
        }

        public S2Point getPoint() {
            return point;
        }

        @Override
        public int compareTo(ParametrizedS2Point o) {
            int compareTime = Double.compare(time, o.time);
            if (compareTime != 0) {
                return compareTime;
            }
            return point.compareTo(o.point);
        }
    }
}
