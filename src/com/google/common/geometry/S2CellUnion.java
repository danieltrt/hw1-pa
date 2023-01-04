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

import com.google.common.collect.Lists;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;

public strictfp class S2CellUnion implements S2Region, Iterable<S2CellId> {

    private ArrayList<S2CellId> cellIds = new ArrayList<S2CellId>();

    S2CellUnion() {
    }

    void initFromCellIds(ArrayList<S2CellId> cellIds) {
        initRawCellIds(cellIds);
        normalize();
    }

    void initFromIds(ArrayList<Long> cellIds) {
        initRawIds(cellIds);
        normalize();
    }

    void initSwap(ArrayList<S2CellId> cellIds) {
        initRawSwap(cellIds);
        normalize();
    }

    private void initRawCellIds(ArrayList<S2CellId> cellIds) {
        this.cellIds = cellIds;
    }

    private void initRawIds(ArrayList<Long> cellIds) {
        int size = cellIds.size();
        this.cellIds = new ArrayList<S2CellId>(size);
        for (Long id : cellIds) {
            this.cellIds.add(new S2CellId(id));
        }
    }

    private void initRawSwap(ArrayList<S2CellId> cellIds) {
        this.cellIds = new ArrayList<S2CellId>(cellIds);
        cellIds.clear();
    }

    public int size() {
        return cellIds.size();
    }

    S2CellId cellId(int i) {
        return cellIds.get(i);
    }

    @Override
    public Iterator<S2CellId> iterator() {
        return cellIds.iterator();
    }

    ArrayList<S2CellId> cellIds() {
        return cellIds;
    }

    void denormalize(int minLevel, int levelMod, ArrayList<S2CellId> output) {
        output.clear();
        output.ensureCapacity(size());
        for (S2CellId id : this) {
            int level = id.level();
            int newLevel = Math.max(minLevel, level);
            if (levelMod > 1) {
                newLevel += (S2CellId.MAX_LEVEL - (newLevel - minLevel)) % levelMod;
                newLevel = Math.min(S2CellId.MAX_LEVEL, newLevel);
            }
            if (newLevel == level) {
                output.add(id);
            } else {
                S2CellId end = id.childEnd(newLevel);
                for (id = id.childBegin(newLevel); !id.equals(end); id = id.next()) {
                    output.add(id);
                }
            }
        }
    }

    public void pack() {
        cellIds.trimToSize();
    }

    public boolean contains(S2CellId id) {
        int pos = Collections.binarySearch(cellIds, id);
        if (pos < 0) {
            pos = -pos - 1;
        }
        if (pos < cellIds.size() && cellIds.get(pos).rangeMin().lessOrEquals(id)) {
            return true;
        }
        return pos != 0 && cellIds.get(pos - 1).rangeMax().greaterOrEquals(id);
    }

    public boolean intersects(S2CellId id) {
        int pos = Collections.binarySearch(cellIds, id);
        if (pos < 0) {
            pos = -pos - 1;
        }
        if (pos < cellIds.size() && cellIds.get(pos).rangeMin().lessOrEquals(id.rangeMax())) {
            return true;
        }
        return pos != 0 && cellIds.get(pos - 1).rangeMax().greaterOrEquals(id.rangeMin());
    }

    public boolean contains(S2CellUnion that) {
        for (S2CellId id : that) {
            if (!this.contains(id)) {
                return false;
            }
        }
        return true;
    }

    @Override
    public boolean contains(S2Cell cell) {
        return contains(cell.id());
    }

    public boolean intersects(S2CellUnion union) {
        for (S2CellId id : union) {
            if (intersects(id)) {
                return true;
            }
        }
        return false;
    }

    void getUnion(S2CellUnion x, S2CellUnion y) {
        cellIds.clear();
        cellIds.ensureCapacity(x.size() + y.size());
        cellIds.addAll(x.cellIds);
        cellIds.addAll(y.cellIds);
        normalize();
    }

    void getIntersection(S2CellUnion x, S2CellId id) {
        cellIds.clear();
        if (x.contains(id)) {
            cellIds.add(id);
        } else {
            int pos = Collections.binarySearch(x.cellIds, id.rangeMin());
            if (pos < 0) {
                pos = -pos - 1;
            }
            S2CellId idmax = id.rangeMax();
            int size = x.cellIds.size();
            while (pos < size && x.cellIds.get(pos).lessOrEquals(idmax)) {
                cellIds.add(x.cellIds.get(pos++));
            }
        }
    }

    void getIntersection(S2CellUnion x, S2CellUnion y) {
        cellIds.clear();
        int i = 0;
        int j = 0;
        while (i < x.cellIds.size() && j < y.cellIds.size()) {
            S2CellId imin = x.cellId(i).rangeMin();
            S2CellId jmin = y.cellId(j).rangeMin();
            if (imin.greaterThan(jmin)) {
                if (x.cellId(i).lessOrEquals(y.cellId(j).rangeMax())) {
                    cellIds.add(x.cellId(i++));
                } else {
                    j = indexedBinarySearch(y.cellIds, imin, j + 1);
                    if (x.cellId(i).lessOrEquals(y.cellId(j - 1).rangeMax())) {
                        --j;
                    }
                }
            } else if (jmin.greaterThan(imin)) {
                if (y.cellId(j).lessOrEquals(x.cellId(i).rangeMax())) {
                    cellIds.add(y.cellId(j++));
                } else {
                    i = indexedBinarySearch(x.cellIds, jmin, i + 1);
                    if (y.cellId(j).lessOrEquals(x.cellId(i - 1).rangeMax())) {
                        --i;
                    }
                }
            } else {
                if (x.cellId(i).lessThan(y.cellId(j))) {
                    cellIds.add(x.cellId(i++));
                } else {
                    cellIds.add(y.cellId(j++));
                }
            }
        }
    }

    private int indexedBinarySearch(List<S2CellId> l, S2CellId key, int low) {
        int high = l.size() - 1;
        while (low <= high) {
            int mid = (low + high) >> 1;
            S2CellId midVal = l.get(mid);
            int cmp = midVal.compareTo(key);

            if (cmp < 0) {
                low = mid + 1;
            } else if (cmp > 0) {
                high = mid - 1;
            } else {
                return mid;
            }
        }
        return low;
    }

    void expand(int level) {
        ArrayList<S2CellId> output = new ArrayList<S2CellId>();
        long levelLsb = S2CellId.lowestOnBitForLevel(level);
        int i = size() - 1;
        do {
            S2CellId id = cellId(i);
            if (id.lowestOnBit() < levelLsb) {
                id = id.parent(level);
                while (i > 0 && id.contains(cellId(i - 1))) {
                    --i;
                }
            }
            output.add(id);
            id.getAllNeighbors(level, output);
        } while (--i >= 0);
        initSwap(output);
    }

    void expand(S1Angle minRadius, int maxLevelDiff) {
        int minLevel = S2CellId.MAX_LEVEL;
        for (S2CellId id : this) {
            minLevel = Math.min(minLevel, id.level());
        }
        int radiusLevel = S2Projections.MIN_WIDTH.getMaxLevel(minRadius.radians());
        if (radiusLevel == 0 && minRadius.radians() > S2Projections.MIN_WIDTH.getValue(0)) {
            expand(0);
        }
        expand(Math.min(minLevel + maxLevelDiff, radiusLevel));
    }

    @Override
    public S2Region clone() {
        S2CellUnion copy = new S2CellUnion();
        copy.initRawCellIds(Lists.newArrayList(cellIds));
        return copy;
    }

    @Override
    public S2Cap getCapBound() {
        if (cellIds.isEmpty()) {
            return S2Cap.empty();
        }
        S2Point centroid = new S2Point(0, 0, 0);
        for (S2CellId id : this) {
            double area = S2Cell.averageArea(id.level());
            centroid = S2Point.add(centroid, S2Point.mul(id.toPoint(), area));
        }
        if (centroid.equals(new S2Point(0, 0, 0))) {
            centroid = new S2Point(1, 0, 0);
        } else {
            centroid = S2Point.normalize(centroid);
        }
        S2Cap cap = S2Cap.fromAxisHeight(centroid, 0);
        for (S2CellId id : this) {
            cap = cap.addCap(new S2Cell(id).getCapBound());
        }
        return cap;
    }

    @Override
    public S2LatLngRect getRectBound() {
        S2LatLngRect bound = S2LatLngRect.empty();
        for (S2CellId id : this) {
            bound = bound.union(new S2Cell(id).getRectBound());
        }
        return bound;
    }

    @Override
    public boolean mayIntersect(S2Cell cell) {
        return intersects(cell.id());
    }

    public boolean contains(S2Point p) {
        return contains(S2CellId.fromPoint(p));
    }

    long leafCellsCovered() {
        long numLeaves = 0;
        for (S2CellId cellId : cellIds) {
            int invertedLevel = S2CellId.MAX_LEVEL - cellId.level();
            numLeaves += (1L << (invertedLevel << 1));
        }
        return numLeaves;
    }

    double averageBasedArea() {
        return S2Cell.averageArea(S2CellId.MAX_LEVEL) * leafCellsCovered();
    }

    double approxArea() {
        double area = 0;
        for (S2CellId cellId : cellIds) {
            area += new S2Cell(cellId).approxArea();
        }
        return area;
    }

    double exactArea() {
        double area = 0;
        for (S2CellId cellId : cellIds) {
            area += new S2Cell(cellId).exactArea();
        }
        return area;
    }

    @Override
    public boolean equals(Object that) {
        if (!(that instanceof S2CellUnion)) {
            return false;
        }
        S2CellUnion union = (S2CellUnion) that;
        return this.cellIds.equals(union.cellIds);
    }

    @Override
    public int hashCode() {
        int value = 17;
        for (S2CellId id : this) {
            value = 37 * value + id.hashCode();
        }
        return value;
    }

    public boolean normalize() {
        ArrayList<S2CellId> output = new ArrayList<>(cellIds.size());
        output.ensureCapacity(cellIds.size());
        Collections.sort(cellIds);
        for (S2CellId id : this) {
            int size = output.size();
            if (!output.isEmpty() && output.get(size - 1).contains(id)) {
                continue;
            }
            while (!output.isEmpty() && id.contains(output.get(output.size() - 1))) {
                output.remove(output.size() - 1);
            }
            while (output.size() >= 3) {
                size = output.size();
                if ((output.get(size - 3).id() ^ output.get(size - 2).id() ^ output.get(size - 1).id())
                        != id.id()) {
                    break;
                }
                long mask = id.lowestOnBit() << 1;
                mask = ~(mask + (mask << 1));
                long idMasked = (id.id() & mask);
                if ((output.get(size - 3).id() & mask) != idMasked
                        || (output.get(size - 2).id() & mask) != idMasked
                        || (output.get(size - 1).id() & mask) != idMasked || id.isFace()) {
                    break;
                }
                output.remove(size - 1);
                output.remove(size - 2);
                output.remove(size - 3);
                id = id.parent();
            }
            output.add(id);
        }
        if (output.size() < size()) {
            initRawSwap(output);
            return true;
        }
        return false;
    }
}
