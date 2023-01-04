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
import com.google.common.collect.Lists;
import com.google.common.collect.Sets;

import java.util.*;

public abstract strictfp class S2EdgeIndex {
    private static final double THICKENING = 0.01;

    private static final double MAX_DET_ERROR = 1e-14;

    private long[] cells;

    private int[] edges;

    private int minimumS2LevelUsed;

    private boolean indexComputed;

    private int queryCount;

    private static int compare(long cell1, int edge1, long cell2, int edge2) {
        if (cell1 < cell2) {
            return -1;
        } else if (cell1 > cell2) {
            return 1;
        } else return Integer.compare(edge1, edge2);
    }

    private static S2CellId containingCell(S2Point pa, S2Point pb, S2Point pc, S2Point pd) {
        S2CellId a = S2CellId.fromPoint(pa);
        S2CellId b = S2CellId.fromPoint(pb);
        S2CellId c = S2CellId.fromPoint(pc);
        S2CellId d = S2CellId.fromPoint(pd);
        if (a.face() != b.face() || a.face() != c.face() || a.face() != d.face()) {
            return S2CellId.sentinel();
        }
        while (!a.equals(b) || !a.equals(c) || !a.equals(d)) {
            a = a.parent();
            b = b.parent();
            c = c.parent();
            d = d.parent();
        }
        return a;
    }

    private static S2CellId containingCell(S2Point pa, S2Point pb) {
        S2CellId a = S2CellId.fromPoint(pa);
        S2CellId b = S2CellId.fromPoint(pb);
        if (a.face() != b.face()) {
            return S2CellId.sentinel();
        }
        while (!a.equals(b)) {
            a = a.parent();
            b = b.parent();
        }
        return a;
    }

    private static boolean lenientCrossing(S2Point a, S2Point b, S2Point c, S2Point d) {
        double acb = S2Point.crossProd(a, c).dotProd(b);
        double bda = S2Point.crossProd(b, d).dotProd(a);
        if (Math.abs(acb) < MAX_DET_ERROR || Math.abs(bda) < MAX_DET_ERROR) {
            return true;
        }
        if (acb * bda < 0) {
            return false;
        }
        double cbd = S2Point.crossProd(c, b).dotProd(d);
        double dac = S2Point.crossProd(c, a).dotProd(c);
        if (Math.abs(cbd) < MAX_DET_ERROR || Math.abs(dac) < MAX_DET_ERROR) {
            return true;
        }
        return (acb * cbd >= 0) && (acb * dac >= 0);
    }

    private static boolean edgeIntersectsCellBoundary(S2Point a, S2Point b, S2Cell cell) {
        S2Point[] vertices = new S2Point[4];
        for (int i = 0; i < 4; ++i) {
            vertices[i] = cell.getVertex(i);
        }
        for (int i = 0; i < 4; ++i) {
            S2Point fromPoint = vertices[i];
            S2Point toPoint = vertices[(i + 1) % 4];
            if (lenientCrossing(a, b, fromPoint, toPoint)) {
                return true;
            }
        }
        return false;
    }

    public void reset() {
        minimumS2LevelUsed = S2CellId.MAX_LEVEL;
        indexComputed = false;
        queryCount = 0;
        cells = null;
        edges = null;
    }

    final void computeIndex() {
        if (indexComputed) {
            return;
        }
        List<Long> cellList = Lists.newArrayList();
        List<Integer> edgeList = Lists.newArrayList();
        for (int i = 0; i < getNumEdges(); ++i) {
            S2Point from = edgeFrom(i);
            S2Point to = edgeTo(i);
            ArrayList<S2CellId> cover = Lists.newArrayList();
            int level = getCovering(from, to, true, cover);
            minimumS2LevelUsed = Math.min(minimumS2LevelUsed, level);
            for (S2CellId cellId : cover) {
                cellList.add(cellId.id());
                edgeList.add(i);
            }
        }
        cells = new long[cellList.size()];
        edges = new int[edgeList.size()];
        for (int i = 0; i < cells.length; i++) {
            cells[i] = cellList.get(i);
            edges[i] = edgeList.get(i);
        }
        sortIndex();
        indexComputed = true;
    }

    private void sortIndex() {
        Integer[] indices = new Integer[cells.length];
        for (int i = 0; i < indices.length; i++) {
            indices[i] = i;
        }
        Arrays.sort(indices, (index1, index2) -> compare(cells[index1], edges[index1], cells[index2], edges[index2]));
        long[] newCells = new long[cells.length];
        int[] newEdges = new int[edges.length];
        for (int i = 0; i < indices.length; i++) {
            newCells[i] = cells[indices[i]];
            newEdges[i] = edges[indices[i]];
        }
        cells = newCells;
        edges = newEdges;
    }

    private boolean isIndexComputed() {
        return indexComputed;
    }

    private void incrementQueryCount() {
        ++queryCount;
    }

    final void predictAdditionalCalls(int n) {
        if (indexComputed) {
            return;
        }
        if (getNumEdges() > 100 && (queryCount + n) > 30) {
            computeIndex();
        }
    }

    protected abstract int getNumEdges();

    protected abstract S2Point edgeFrom(int index);

    protected abstract S2Point edgeTo(int index);

    private void findCandidateCrossings(S2Point a, S2Point b, List<Integer> candidateCrossings) {
        Preconditions.checkState(indexComputed);
        ArrayList<S2CellId> cover = Lists.newArrayList();
        getCovering(a, b, false, cover);
        Set<Integer> uniqueSet = new HashSet<Integer>();
        getEdgesInParentCells(cover, uniqueSet);
        getEdgesInChildrenCells(a, b, cover, uniqueSet);
        candidateCrossings.clear();
        candidateCrossings.addAll(uniqueSet);
    }

    private int getCovering(
            S2Point a, S2Point b, boolean thickenEdge, ArrayList<S2CellId> edgeCovering) {
        edgeCovering.clear();
        double edgeLength = a.angle(b);
        int idealLevel = S2Projections.MIN_WIDTH.getMaxLevel(edgeLength * (1 + 2 * THICKENING));
        S2CellId containingCellId;
        if (!thickenEdge) {
            containingCellId = containingCell(a, b);
        } else {
            if (idealLevel == S2CellId.MAX_LEVEL) {
                containingCellId = (new S2CellId(0xFFF0)).parent(3);
            } else {
                S2Point pq = S2Point.mul(S2Point.minus(b, a), THICKENING);
                S2Point ortho =
                        S2Point.mul(S2Point.normalize(S2Point.crossProd(pq, a)), edgeLength * THICKENING);
                S2Point p = S2Point.minus(a, pq);
                S2Point q = S2Point.add(b, pq);
                containingCellId =
                        containingCell(S2Point.minus(p, ortho), S2Point.add(p, ortho), S2Point.minus(q, ortho),
                                S2Point.add(q, ortho));
            }
        }
        if (!containingCellId.equals(S2CellId.sentinel())
                && containingCellId.level() >= idealLevel - 2) {
            edgeCovering.add(containingCellId);
            return containingCellId.level();
        }
        if (idealLevel == 0) {
            for (S2CellId cellid = S2CellId.begin(0); !cellid.equals(S2CellId.end(0));
                 cellid = cellid.next()) {
                edgeCovering.add(cellid);
            }
            return 0;
        }
        S2Point middle = S2Point.normalize(S2Point.div(S2Point.add(a, b), 2));
        int actualLevel = Math.min(idealLevel, S2CellId.MAX_LEVEL - 1);
        S2CellId.fromPoint(middle).getVertexNeighbors(actualLevel, edgeCovering);
        return actualLevel;
    }

    private int[] getEdges(long cell1, long cell2) {
        if (cell1 > cell2) {
            long temp = cell1;
            cell1 = cell2;
            cell2 = temp;
        }
        return new int[]{
                -1 - binarySearch(cell1, Integer.MIN_VALUE),
                -1 - binarySearch(cell2, Integer.MAX_VALUE)};
    }

    private int binarySearch(long cell, int edge) {
        int low = 0;
        int high = cells.length - 1;
        while (low <= high) {
            int mid = (low + high) >>> 1;
            int cmp = compare(cells[mid], edges[mid], cell, edge);
            if (cmp < 0) {
                low = mid + 1;
            } else if (cmp > 0) {
                high = mid - 1;
            } else {
                return mid;
            }
        }
        return -(low + 1);
    }

    private void getEdgesInParentCells(List<S2CellId> cover, Set<Integer> candidateCrossings) {
        Set<S2CellId> parentCells = Sets.newHashSet();
        for (S2CellId coverCell : cover) {
            for (int parentLevel = coverCell.level() - 1; parentLevel >= minimumS2LevelUsed;
                 --parentLevel) {
                if (!parentCells.add(coverCell.parent(parentLevel))) {
                    break;
                }
            }
        }
        for (S2CellId parentCell : parentCells) {
            int[] bounds = getEdges(parentCell.id(), parentCell.id());
            for (int i = bounds[0]; i < bounds[1]; i++) {
                candidateCrossings.add(edges[i]);
            }
        }
    }

    private void getEdgesInChildrenCells(S2Point a, S2Point b, List<S2CellId> cover,
                                         Set<Integer> candidateCrossings) {
        S2Cell[] children = null;
        while (!cover.isEmpty()) {
            S2CellId cell = cover.remove(cover.size() - 1);
            int[] bounds = getEdges(cell.rangeMin().id(), cell.rangeMax().id());
            if (bounds[1] - bounds[0] <= 16) {
                for (int i = bounds[0]; i < bounds[1]; i++) {
                    candidateCrossings.add(edges[i]);
                }
            } else {
                bounds = getEdges(cell.id(), cell.id());
                for (int i = bounds[0]; i < bounds[1]; i++) {
                    candidateCrossings.add(edges[i]);
                }
                if (children == null) {
                    children = new S2Cell[4];
                    for (int i = 0; i < 4; ++i) {
                        children[i] = new S2Cell();
                    }
                }
                new S2Cell(cell).subdivide(children);
                for (S2Cell child : children) {
                    if (edgeIntersectsCellBoundary(a, b, child)) {
                        cover.add(child.id());
                    }
                }
            }
        }
    }

    public static class DataEdgeIterator {
        private final S2EdgeIndex edgeIndex;
        ArrayList<Integer> candidates;
        private boolean isBruteForce;
        private int currentIndex;
        private int numEdges;
        private int currentIndexInCandidates;

        DataEdgeIterator(S2EdgeIndex edgeIndex) {
            this.edgeIndex = edgeIndex;
            candidates = Lists.newArrayList();
        }

        void getCandidates(S2Point a, S2Point b) {
            edgeIndex.predictAdditionalCalls(1);
            isBruteForce = !edgeIndex.isIndexComputed();
            if (isBruteForce) {
                edgeIndex.incrementQueryCount();
                currentIndex = 0;
                numEdges = edgeIndex.getNumEdges();
            } else {
                candidates.clear();
                edgeIndex.findCandidateCrossings(a, b, candidates);
                currentIndexInCandidates = 0;
                if (!candidates.isEmpty()) {
                    currentIndex = candidates.get(0);
                }
            }
        }

        int index() {
            Preconditions.checkState(hasNext());
            return currentIndex;
        }

        boolean hasNext() {
            if (isBruteForce) {
                return (currentIndex < numEdges);
            } else {
                return currentIndexInCandidates < candidates.size();
            }
        }

        public void next() {
            Preconditions.checkState(hasNext());
            if (isBruteForce) {
                ++currentIndex;
            } else {
                ++currentIndexInCandidates;
                if (currentIndexInCandidates < candidates.size()) {
                    currentIndex = candidates.get(currentIndexInCandidates);
                }
            }
        }
    }
}
