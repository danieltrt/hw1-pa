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

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.PriorityQueue;

final strictfp class S2RegionCoverer {

    private static final int DEFAULT_MAX_CELLS = 8;

    private static final S2Cell[] FACE_CELLS = new S2Cell[6];

    static {
        for (int face = 0; face < 6; ++face) {
            FACE_CELLS[face] = S2Cell.fromFacePosLevel(face, (byte) 0, 0);
        }
    }

    private S2Region region;
    private ArrayList<S2CellId> result;
    private int minLevel;
    private int maxLevel;
    private int levelMod;
    private int maxCells;
    private boolean interiorCovering;
    private int candidatesCreatedCounter;
    private PriorityQueue<QueueEntry> candidateQueue;

    S2RegionCoverer() {
        minLevel = 0;
        maxLevel = S2CellId.MAX_LEVEL;
        levelMod = 1;
        maxCells = DEFAULT_MAX_CELLS;
        this.region = null;
        result = new ArrayList<>();
        candidateQueue = new PriorityQueue<>(10, new QueueEntriesComparator());
    }

    static void getSimpleCovering(
            S2Region region, S2Point start, int level, ArrayList<S2CellId> output) {
        floodFill(region, S2CellId.fromPoint(start).parent(level), output);
    }

    private static void floodFill(S2Region region, S2CellId start, ArrayList<S2CellId> output) {
        HashSet<S2CellId> all = new HashSet<>();
        ArrayList<S2CellId> frontier = new ArrayList<>();
        output.clear();
        all.add(start);
        frontier.add(start);
        while (!frontier.isEmpty()) {
            S2CellId id = frontier.get(frontier.size() - 1);
            frontier.remove(frontier.size() - 1);
            if (!region.mayIntersect(new S2Cell(id))) {
                continue;
            }
            output.add(id);
            S2CellId[] neighbors = new S2CellId[4];
            id.getEdgeNeighbors(neighbors);
            for (int edge = 0; edge < 4; ++edge) {
                S2CellId nbr = neighbors[edge];
                if (!all.contains(nbr)) {
                    frontier.add(nbr);
                    all.add(nbr);
                }
            }
        }
    }

    void setMinLevel(int minLevel) {
        this.minLevel = Math.max(0, Math.min(S2CellId.MAX_LEVEL, minLevel));
    }

    void setMaxLevel(int maxLevel) {
        this.maxLevel = Math.max(0, Math.min(S2CellId.MAX_LEVEL, maxLevel));
    }

    int minLevel() {
        return minLevel;
    }

    int maxLevel() {
        return maxLevel;
    }

    int maxCells() {
        return maxCells;
    }

    void setLevelMod(int levelMod) {
        this.levelMod = Math.max(1, Math.min(3, levelMod));
    }

    int levelMod() {
        return levelMod;
    }

    void setMaxCells(int maxCells) {
        this.maxCells = maxCells;
    }

    void getCovering(S2Region region, ArrayList<S2CellId> covering) {
        S2CellUnion tmp = getCovering(region);
        tmp.denormalize(minLevel(), levelMod(), covering);
    }

    void getInteriorCovering(S2Region region, ArrayList<S2CellId> interior) {
        S2CellUnion tmp = getInteriorCovering(region);
        tmp.denormalize(minLevel(), levelMod(), interior);
    }

    private S2CellUnion getCovering(S2Region region) {
        S2CellUnion covering = new S2CellUnion();
        getCovering(region, covering);
        return covering;
    }

    void getCovering(S2Region region, S2CellUnion covering) {
        interiorCovering = false;
        getCoveringInternal(region);
        covering.initSwap(result);
    }

    private S2CellUnion getInteriorCovering(S2Region region) {
        S2CellUnion covering = new S2CellUnion();
        getInteriorCovering(region, covering);
        return covering;
    }

    private void getInteriorCovering(S2Region region, S2CellUnion covering) {
        interiorCovering = true;
        getCoveringInternal(region);
        covering.initSwap(result);
    }

    private Candidate newCandidate(S2Cell cell) {
        if (!region.mayIntersect(cell)) {
            return null;
        }
        boolean isTerminal = false;
        if (cell.level() >= minLevel) {
            if (interiorCovering) {
                if (region.contains(cell)) {
                    isTerminal = true;
                } else if (cell.level() + levelMod > maxLevel) {
                    return null;
                }
            } else {
                if (cell.level() + levelMod > maxLevel || region.contains(cell)) {
                    isTerminal = true;
                }
            }
        }
        Candidate candidate = new Candidate();
        candidate.cell = cell;
        candidate.isTerminal = isTerminal;
        if (!isTerminal) {
            candidate.children = new Candidate[1 << maxChildrenShift()];
        }
        candidatesCreatedCounter++;
        return candidate;
    }

    private int maxChildrenShift() {
        return 2 * levelMod;
    }

    private void addCandidate(Candidate candidate) {
        if (candidate == null) {
            return;
        }
        if (candidate.isTerminal) {
            result.add(candidate.cell.id());
            return;
        }
        int numLevels = (candidate.cell.level() < minLevel) ? 1 : levelMod;
        int numTerminals = expandChildren(candidate, candidate.cell, numLevels);
        if (candidate.numChildren == 0) {
        } else if (!interiorCovering && numTerminals == 1 << maxChildrenShift()
                && candidate.cell.level() >= minLevel) {
            candidate.isTerminal = true;
            addCandidate(candidate);
        } else {
            int priority = -((((candidate.cell.level() << maxChildrenShift()) + candidate.numChildren)
                    << maxChildrenShift()) + numTerminals);
            candidateQueue.add(new QueueEntry(priority, candidate));
        }
    }

    private int expandChildren(Candidate candidate, S2Cell cell, int numLevels) {
        numLevels--;
        S2Cell[] childCells = new S2Cell[4];
        for (int i = 0; i < 4; ++i) {
            childCells[i] = new S2Cell();
        }
        cell.subdivide(childCells);
        int numTerminals = 0;
        for (int i = 0; i < 4; ++i) {
            if (numLevels > 0) {
                if (region.mayIntersect(childCells[i])) {
                    numTerminals += expandChildren(candidate, childCells[i], numLevels);
                }
                continue;
            }
            Candidate child = newCandidate(childCells[i]);
            if (child != null) {
                candidate.children[candidate.numChildren++] = child;
                if (child.isTerminal) {
                    ++numTerminals;
                }
            }
        }
        return numTerminals;
    }

    private void getInitialCandidates() {
        if (maxCells >= 4) {
            S2Cap cap = region.getCapBound();
            int level = Math.min(S2Projections.MIN_WIDTH.getMaxLevel(2 * cap.angle().radians()),
                    Math.min(maxLevel(), S2CellId.MAX_LEVEL - 1));
            if (levelMod() > 1 && level > minLevel()) {
                level -= (level - minLevel()) % levelMod();
            }
            if (level > 0) {
                ArrayList<S2CellId> base = new ArrayList<>(4);
                S2CellId id = S2CellId.fromPoint(cap.axis());
                id.getVertexNeighbors(level, base);
                for (S2CellId s2CellId : base) {
                    addCandidate(newCandidate(new S2Cell(s2CellId)));
                }
                return;
            }
        }
        for (int face = 0; face < 6; ++face) {
            addCandidate(newCandidate(FACE_CELLS[face]));
        }
    }

    private void getCoveringInternal(S2Region region) {
        Preconditions.checkState(candidateQueue.isEmpty() && result.isEmpty());
        this.region = region;
        candidatesCreatedCounter = 0;
        getInitialCandidates();
        while (!candidateQueue.isEmpty() && (!interiorCovering || result.size() < maxCells)) {
            Candidate candidate = candidateQueue.poll().candidate;
            // logger.info("Pop: " + candidate.cell.id());
            if (candidate.cell.level() < minLevel || candidate.numChildren == 1
                    || result.size() + (interiorCovering ? 0 : candidateQueue.size()) + candidate.numChildren
                    <= maxCells) {
                // Expand this candidate into its children.
                for (int i = 0; i < candidate.numChildren; ++i) {
                    addCandidate(candidate.children[i]);
                }
            } else if (interiorCovering) {
            } else {
                candidate.isTerminal = true;
                addCandidate(candidate);
            }
        }
        candidateQueue.clear();
        this.region = null;
    }

    private static class Candidate {
        private S2Cell cell;
        private boolean isTerminal;
        private int numChildren;
        private Candidate[] children;
    }

    static class QueueEntry {
        private int id;
        private Candidate candidate;
        QueueEntry(int id, Candidate candidate) {
            this.id = id;
            this.candidate = candidate;
        }
    }

    static class QueueEntriesComparator implements Comparator<QueueEntry> {
        @Override
        public int compare(S2RegionCoverer.QueueEntry x, S2RegionCoverer.QueueEntry y) {
            return Integer.compare(y.id, x.id);
        }
    }
}
