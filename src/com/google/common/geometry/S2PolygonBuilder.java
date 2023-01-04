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

import com.google.common.collect.*;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Stack;
import java.util.logging.Logger;

public strictfp class S2PolygonBuilder {
    private static final Logger log = Logger.getLogger(S2PolygonBuilder.class.getCanonicalName());

    private Options options;

    private Map<S2Point, Multiset<S2Point>> edges;

    public S2PolygonBuilder() {
        this(Options.DIRECTED_XOR);
    }

    S2PolygonBuilder(Options options) {
        this.options = options;
        this.edges = Maps.newHashMap();
    }

    public Options options() {
        return options;
    }

    void addEdge(S2Point v0, S2Point v1) {
        if (v0.equals(v1)) {
            return;
        }
        if (options.getXorEdges()) {
            Multiset<S2Point> candidates = edges.get(v1);
            if (candidates != null && candidates.count(v0) > 0) {
                eraseEdge(v1, v0);
                return;
            }
        }
        if (edges.get(v0) == null) {
            edges.put(v0, HashMultiset.create());
        }
        edges.get(v0).add(v1);
        if (options.getUndirectedEdges()) {
            if (edges.get(v1) == null) {
                edges.put(v1, HashMultiset.create());
            }
            edges.get(v1).add(v0);
        }
    }

    private void addLoop(S2Loop loop) {
        int sign = loop.sign();
        for (int i = loop.numVertices(); i > 0; --i) {
            addEdge(loop.vertex(i), loop.vertex(i + sign));
        }
    }

    void addPolygon(S2Polygon polygon) {
        for (int i = 0; i < polygon.numLoops(); ++i) {
            addLoop(polygon.loop(i));
        }
    }

    boolean assembleLoops(List<S2Loop> loops, List<S2Edge> unusedEdges) {
        if (options.getMergeDistance().radians() > 0) {
            mergeVertices();
        }
        List<S2Edge> dummyUnusedEdges = Lists.newArrayList();
        if (unusedEdges == null) {
            unusedEdges = dummyUnusedEdges;
        }
        unusedEdges.clear();
        while (!edges.isEmpty()) {
            Map.Entry<S2Point, Multiset<S2Point>> edge = edges.entrySet().iterator().next();
            S2Point v0 = edge.getKey();
            S2Point v1 = edge.getValue().iterator().next();
            S2Loop loop = assembleLoop(v0, v1, unusedEdges);
            if (loop == null) {
                continue;
            }
            while (options.getUndirectedEdges() && !loop.isNormalized()) {
                loop = assembleLoop(loop.vertex(1), loop.vertex(0), unusedEdges);
            }
            loops.add(loop);
            eraseLoop(loop, loop.numVertices());
        }
        return unusedEdges.isEmpty();
    }

    boolean assemblePolygon(S2Polygon polygon, List<S2Edge> unusedEdges) {
        List<S2Loop> loops = Lists.newArrayList();
        boolean success = assembleLoops(loops, unusedEdges);
        if (!options.getUndirectedEdges()) {
            for (S2Loop loop : loops) {
                loop.normalize();
            }
        }
        if (options.getValidate() && !S2Polygon.isValid(loops)) {
            if (unusedEdges != null) {
                for (S2Loop loop : loops) {
                    rejectLoop(loop, loop.numVertices(), unusedEdges);
                }
            }
            return false;
        }
        polygon.init(loops);
        return success;
    }

    public S2Polygon assemblePolygon() {
        S2Polygon polygon = new S2Polygon();
        List<S2Edge> unusedEdges = Lists.newArrayList();
        assemblePolygon(polygon, unusedEdges);
        return polygon;
    }

    private void dumpEdges(S2Point v0) {
        log.info(v0.toString());
        Multiset<S2Point> vset = edges.get(v0);
        if (vset != null) {
            for (S2Point v : vset) {
                log.info("    " + v.toString());
            }
        }
    }

    protected void dump() {
        for (S2Point v : edges.keySet()) {
            dumpEdges(v);
        }
    }

    private void eraseEdge(S2Point v0, S2Point v1) {
        Multiset<S2Point> vset = edges.get(v0);
        vset.remove(v1);
        if (vset.isEmpty()) {
            edges.remove(v0);
        }
        if (options.getUndirectedEdges()) {
            vset = edges.get(v1);
            vset.remove(v0);
            if (vset.isEmpty()) {
                edges.remove(v1);
            }
        }
    }

    private void eraseLoop(List<S2Point> v, int n) {
        for (int i = n - 1, j = 0; j < n; i = j++) {
            eraseEdge(v.get(i), v.get(j));
        }
    }

    private void eraseLoop(S2Loop v, int n) {
        for (int i = n - 1, j = 0; j < n; i = j++) {
            eraseEdge(v.vertex(i), v.vertex(j));
        }
    }

    private S2Loop assembleLoop(S2Point v0, S2Point v1, List<S2Edge> unusedEdges) {
        List<S2Point> path = Lists.newArrayList();
        Map<S2Point, Integer> index = Maps.newHashMap();
        path.add(v0);
        path.add(v1);
        index.put(v1, 1);
        while (path.size() >= 2) {
            v0 = path.get(path.size() - 2);
            v1 = path.get(path.size() - 1);
            S2Point v2 = null;
            boolean v2Found = false;
            Multiset<S2Point> vset = edges.get(v1);
            if (vset != null) {
                for (S2Point v : vset) {
                    if (v.equals(v0)) {
                        continue;
                    }
                    if (!v2Found || S2.orderedCCW(v0, v2, v, v1)) {
                        v2 = v;
                    }
                    v2Found = true;
                }
            }
            if (!v2Found) {
                unusedEdges.add(new S2Edge(v0, v1));
                eraseEdge(v0, v1);
                index.remove(v1);
                path.remove(path.size() - 1);
            } else if (index.get(v2) == null) {
                index.put(v2, path.size());
                path.add(v2);
            } else {
                path = path.subList(index.get(v2), path.size());
                if (options.getValidate() && !S2Loop.isValid(path)) {
                    rejectLoop(path, path.size(), unusedEdges);
                    eraseLoop(path, path.size());
                    return null;
                }
                return new S2Loop(path);
            }
        }
        return null;
    }

    private void rejectLoop(S2Loop v, int n, List<S2Edge> unusedEdges) {
        for (int i = n - 1, j = 0; j < n; i = j++) {
            unusedEdges.add(new S2Edge(v.vertex(i), v.vertex(j)));
        }
    }

    private void rejectLoop(List<S2Point> v, int n, List<S2Edge> unusedEdges) {
        for (int i = n - 1, j = 0; j < n; i = j++) {
            unusedEdges.add(new S2Edge(v.get(i), v.get(j)));
        }
    }

    private void moveVertices(Map<S2Point, S2Point> mergeMap) {
        if (mergeMap.isEmpty()) {
            return;
        }
        List<S2Edge> edgesCopy = Lists.newArrayList();
        for (Map.Entry<S2Point, Multiset<S2Point>> edge : this.edges.entrySet()) {
            S2Point v0 = edge.getKey();
            Multiset<S2Point> vset = edge.getValue();
            for (S2Point v1 : vset) {
                if (mergeMap.get(v0) != null || mergeMap.get(v1) != null) {
                    if (!options.getUndirectedEdges() || v0.lessThan(v1)) {
                        edgesCopy.add(new S2Edge(v0, v1));
                    }
                }
            }
        }
        for (S2Edge s2Edge : edgesCopy) {
            S2Point v0 = s2Edge.getStart();
            S2Point v1 = s2Edge.getEnd();
            eraseEdge(v0, v1);
            if (mergeMap.get(v0) != null) {
                v0 = mergeMap.get(v0);
            }
            if (mergeMap.get(v1) != null) {
                v1 = mergeMap.get(v1);
            }
            addEdge(v0, v1);
        }
    }

    private void mergeVertices() {
        PointIndex index = new PointIndex(options.getMergeDistance().radians());
        for (Map.Entry<S2Point, Multiset<S2Point>> edge : edges.entrySet()) {
            index.add(edge.getKey());
            Multiset<S2Point> vset = edge.getValue();
            for (S2Point v : vset) {
                index.add(v);
            }
        }
        Map<S2Point, S2Point> mergeMap = Maps.newHashMap();
        Stack<S2Point> frontier = new Stack<>();
        List<S2Point> mergeable = Lists.newArrayList();
        for (Map.Entry<S2CellId, MarkedS2Point> entry : index.entries()) {
            MarkedS2Point point = entry.getValue();
            if (point.isMarked()) {
                continue;
            }
            point.mark();
            S2Point vstart = point.getPoint();
            frontier.push(vstart);
            while (!frontier.isEmpty()) {
                S2Point v0 = frontier.pop();
                index.query(v0, mergeable);
                for (S2Point v1 : mergeable) {
                    frontier.push(v1);
                    mergeMap.put(v1, vstart);
                }
            }
        }
        moveVertices(mergeMap);
    }

    public enum Options {
        DIRECTED_XOR(false, true),
        UNDIRECTED_XOR(true, true),
        UNDIRECTED_UNION(true, false),
        DIRECTED_UNION(false, false);

        private boolean undirectedEdges;
        private boolean xorEdges;
        private boolean validate;
        private S1Angle mergeDistance;

        Options(boolean undirectedEdges, boolean xorEdges) {
            this.undirectedEdges = undirectedEdges;
            this.xorEdges = xorEdges;
            this.validate = false;
            this.mergeDistance = S1Angle.radians(0);
        }

        public boolean getUndirectedEdges() {
            return undirectedEdges;
        }

        void setUndirectedEdges(boolean undirectedEdges) {
            this.undirectedEdges = undirectedEdges;
        }

        public boolean getXorEdges() {
            return xorEdges;
        }

        void setXorEdges(boolean xorEdges) {
            this.xorEdges = xorEdges;
        }

        public boolean getValidate() {
            return validate;
        }

        public void setValidate(boolean validate) {
            this.validate = validate;
        }

        public S1Angle getMergeDistance() {
            return mergeDistance;
        }

        public void setMergeDistance(S1Angle mergeDistance) {
            this.mergeDistance = mergeDistance;
        }
    }

    private static class PointIndex extends ForwardingMultimap<S2CellId, MarkedS2Point> {
        private final Multimap<S2CellId, MarkedS2Point> delegate = HashMultimap.create();
        private double searchRadius;
        private int level;

        PointIndex(double searchRadius) {
            this.searchRadius = searchRadius;
            this.level =
                    Math.min(S2Projections.MIN_WIDTH.getMaxLevel(2 * searchRadius), S2CellId.MAX_LEVEL - 1);
        }

        @Override
        protected Multimap<S2CellId, MarkedS2Point> delegate() {
            return delegate;
        }

        public void add(S2Point p) {
            S2CellId id = S2CellId.fromPoint(p).parent(level);
            Collection<MarkedS2Point> pointSet = get(id);
            for (MarkedS2Point point : pointSet) {
                if (point.getPoint().equals(p)) {
                    return;
                }
            }
            put(id, new MarkedS2Point(p));
        }

        void query(S2Point center, List<S2Point> output) {
            output.clear();
            List<S2CellId> neighbors = Lists.newArrayList();
            S2CellId.fromPoint(center).getVertexNeighbors(level, neighbors);
            for (S2CellId id : neighbors) {
                for (MarkedS2Point mp : get(id)) {
                    if (mp.isMarked()) {
                        continue;
                    }
                    S2Point p = mp.getPoint();
                    if (center.angle(p) <= searchRadius) {
                        output.add(p);
                        mp.mark();
                    }
                }
            }
        }
    }

    private static class MarkedS2Point {
        private S2Point point;
        private boolean mark;

        MarkedS2Point(S2Point point) {
            this.point = point;
            this.mark = false;
        }

        boolean isMarked() {
            return mark;
        }

        public S2Point getPoint() {
            return point;
        }

        void mark() {
            this.mark = true;
        }
    }
}
