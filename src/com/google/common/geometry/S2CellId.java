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

import java.util.List;
import java.util.Locale;

public final strictfp class S2CellId implements Comparable<S2CellId> {
    static final int FACE_BITS = 3;
    static final int NUM_FACES = 6;
    public static final int MAX_LEVEL = 30;
    private static final int POS_BITS = 2 * MAX_LEVEL + 1;
    private static final int MAX_SIZE = 1 << MAX_LEVEL;
    private static final long MAX_UNSIGNED = -1L;
    private static final int LOOKUP_BITS = 4;
    private static final int SWAP_MASK = 0x01;
    private static final int INVERT_MASK = 0x02;
    private static final long LONGS_CAN_SHIFT_MORE_THAN_32 = 1L << 55;
    private static final int[] LOOKUP_POS = new int[1 << (2 * LOOKUP_BITS + 2)];
    private static final int[] LOOKUP_IJ = new int[1 << (2 * LOOKUP_BITS + 2)];
    private static final long WRAP_OFFSET = (long) (NUM_FACES) << POS_BITS;
    private static final long[] maxValueDivs = {0, 0,
            9223372036854775807L, 6148914691236517205L, 4611686018427387903L,
            3689348814741910323L, 3074457345618258602L, 2635249153387078802L,
            2305843009213693951L, 2049638230412172401L, 1844674407370955161L,
            1676976733973595601L, 1537228672809129301L, 1418980313362273201L,
            1317624576693539401L, 1229782938247303441L, 1152921504606846975L,
            1085102592571150095L, 1024819115206086200L, 970881267037344821L,
            922337203685477580L, 878416384462359600L, 838488366986797800L,
            802032351030850070L, 768614336404564650L, 737869762948382064L,
            709490156681136600L, 683212743470724133L, 658812288346769700L,
            636094623231363848L, 614891469123651720L, 595056260442243600L,
            576460752303423487L, 558992244657865200L, 542551296285575047L,
            527049830677415760L, 512409557603043100L};
    private static final int[] maxValueMods = {0, 0,
            1, 0, 3, 0, 3, 1, 7, 6, 5, 4, 3, 2, 1, 0, 15, 0, 15, 16, 15, 15,
            15, 5, 15, 15, 15, 24, 15, 23, 15, 15, 31, 15, 17, 15, 15};
    static {
        initLookupCell(0, 0, 0, 0, 0, 0);
        initLookupCell(0, 0, 0, SWAP_MASK, 0, SWAP_MASK);
        initLookupCell(0, 0, 0, INVERT_MASK, 0, INVERT_MASK);
        initLookupCell(0, 0, 0, SWAP_MASK | INVERT_MASK, 0, SWAP_MASK | INVERT_MASK);
    }

    private final long id;

    public S2CellId(long id) {
        this.id = id;
    }

    public S2CellId() {
        this.id = 0;
    }

    static S2CellId none() {
        return new S2CellId();
    }

    static S2CellId sentinel() {
        return new S2CellId(MAX_UNSIGNED); // -1
    }

    public static S2CellId fromFacePosLevel(int face, long pos, int level) {
        return new S2CellId((((long) face) << POS_BITS) + (pos | 1)).parent(level);
    }

    public static S2CellId fromPoint(S2Point p) {
        int face = S2Projections.xyzToFace(p);
        R2Vector uv = S2Projections.validFaceXyzToUv(face, p);
        int i = stToIJ(S2Projections.uvToST(uv.x()));
        int j = stToIJ(S2Projections.uvToST(uv.y()));
        return fromFaceIJ(face, i, j);
    }

    static S2CellId fromLatLng(S2LatLng ll) {
        return fromPoint(ll.toPoint());
    }

    static S2CellId begin(int level) {
        return fromFacePosLevel(0, 0, 0).childBegin(level);
    }

    public static S2CellId end(int level) {
        return fromFacePosLevel(5, 0, 0).childEnd(level);
    }

    static S2CellId fromToken(String token) {
        if (token == null) {
            throw new NumberFormatException("Null string in S2CellId.fromToken");
        }
        if (token.length() == 0) {
            throw new NumberFormatException("Empty string in S2CellId.fromToken");
        }
        if (token.length() > 16 || "X".equals(token)) {
            return none();
        }
        long value = 0;
        for (int pos = 0; pos < 16; pos++) {
            int digit = 0;
            if (pos < token.length()) {
                digit = Character.digit(token.charAt(pos), 16);
                if (digit == -1) {
                    throw new NumberFormatException(token);
                }
                if (overflowInParse(value, digit)) {
                    throw new NumberFormatException("Too large for unsigned long: " + token);
                }
            }
            value = (value * 16) + digit;
        }
        return new S2CellId(value);
    }

    private static boolean overflowInParse(long current, int digit) {
        return overflowInParse(current, digit, 10);
    }

    private static boolean overflowInParse(long current, int digit, int radix) {
        if (current >= 0) {
            if (current < maxValueDivs[radix]) {
                return false;
            }
            if (current > maxValueDivs[radix]) {
                return true;
            }
            return (digit > maxValueMods[radix]);
        }
        return true;
    }

    static S2CellId fromFaceIJ(int face, int i, int j) {
        long[] n = {0, face << (POS_BITS - 33)};
        int bits = (face & SWAP_MASK);
        for (int k = 7; k >= 0; --k) {
            bits = getBits(n, i, j, k, bits);
        }
        return new S2CellId((((n[1] << 32) + n[0]) << 1) + 1);
    }

    private static int getBits(long[] n, int i, int j, int k, int bits) {
        final int mask = (1 << LOOKUP_BITS) - 1;
        bits += (((i >> (k * LOOKUP_BITS)) & mask) << (LOOKUP_BITS + 2));
        bits += (((j >> (k * LOOKUP_BITS)) & mask) << 2);
        bits = LOOKUP_POS[bits];
        n[k >> 2] |= ((((long) bits) >> 2) << ((k & 3) * 2 * LOOKUP_BITS));
        bits &= (SWAP_MASK | INVERT_MASK);
        return bits;
    }

    static long lowestOnBitForLevel(int level) {
        return 1L << (2 * (MAX_LEVEL - level));
    }

    private static int stToIJ(double s) {
        final int m = MAX_SIZE / 2;
        return (int) Math
                .max(0, Math.min(2 * m - 1, Math.round(m * s + (m - 0.5))));
    }

    private static S2Point faceSiTiToXYZ(int face, int si, int ti) {
        final double kScale = 1.0 / MAX_SIZE;
        double u = S2Projections.stToUV(kScale * si);
        double v = S2Projections.stToUV(kScale * ti);
        return S2Projections.faceUvToXyz(face, u, v);
    }

    private static S2CellId fromFaceIJWrap(int face, int i, int j) {
        i = Math.max(-1, Math.min(MAX_SIZE, i));
        j = Math.max(-1, Math.min(MAX_SIZE, j));
        final double kScale = 1.0 / MAX_SIZE;
        double s = kScale * ((i << 1) + 1 - MAX_SIZE);
        double t = kScale * ((j << 1) + 1 - MAX_SIZE);
        S2Point p = S2Projections.faceUvToXyz(face, s, t);
        face = S2Projections.xyzToFace(p);
        R2Vector st = S2Projections.validFaceXyzToUv(face, p);
        return fromFaceIJ(face, stToIJ(st.x()), stToIJ(st.y()));
    }

    private static S2CellId fromFaceIJSame(int face, int i, int j,
                                           boolean sameFace) {
        if (sameFace) {
            return S2CellId.fromFaceIJ(face, i, j);
        } else {
            return S2CellId.fromFaceIJWrap(face, i, j);
        }
    }

    private static boolean unsignedLongLessThan(long x1, long x2) {
        return (x1 + Long.MIN_VALUE) < (x2 + Long.MIN_VALUE);
    }

    private static boolean unsignedLongGreaterThan(long x1, long x2) {
        return (x1 + Long.MIN_VALUE) > (x2 + Long.MIN_VALUE);
    }

    private static void initLookupCell(int level, int i, int j,
                                       int origOrientation, int pos, int orientation) {
        if (level == LOOKUP_BITS) {
            int ij = (i << LOOKUP_BITS) + j;
            LOOKUP_POS[(ij << 2) + origOrientation] = (pos << 2) + orientation;
            LOOKUP_IJ[(pos << 2) + origOrientation] = (ij << 33) + orientation;
        } else {
            level++;
            i <<= 1;
            j <<= 1;
            pos <<= 2;
            for (int subPos = 0; subPos < 4; subPos++) {
                int ij = S2.posToIJ(orientation, subPos);
                int orientationMask = S2.posToOrientation(subPos);
                initLookupCell(level, i + (ij >>> 1), j + (ij & 1), origOrientation,
                        pos + subPos, orientation ^ orientationMask);
            }
        }
    }

    public S2Point toPoint() {
        return S2Point.normalize(toPointRaw());
    }

    S2Point toPointRaw() {
        MutableInteger i = new MutableInteger(0);
        MutableInteger j = new MutableInteger(0);
        int face = toFaceIJOrientation(i, j, null);
        int delta = isLeaf() ? 1 : (((i.intValue() ^ (((int) id) >>> 2)) & 1) != 0)
                ? 2 : 0;
        int si = (i.intValue() << 1) + delta - MAX_SIZE;
        int ti = (j.intValue() << 1) + delta - MAX_SIZE;
        return faceSiTiToXYZ(face, si, ti);
    }

    S2LatLng toLatLng() {
        return new S2LatLng(toPointRaw());
    }

    public long id() {
        return id;
    }

    public boolean isValid() {
        return face() < NUM_FACES && ((lowestOnBit() & (0x1555555555555555L)) != 0);
    }

    public int face() {
        return (int) (id >>> POS_BITS);
    }

    long pos() {
        return (id & (-1L >>> FACE_BITS));
    }

    public int level() {
        if (isLeaf()) {
            return MAX_LEVEL;
        }
        int x = ((int) id);
        int level = -1;
        if (x != 0) {
            level += 16;
        } else {
            x = (int) (id >>> 32);
        }
        x &= -x;
        if ((x & 0x00005555) != 0) {
            level += 8;
        }
        if ((x & 0x00550055) != 0) {
            level += 4;
        }
        if ((x & 0x05050505) != 0) {
            level += 2;
        }
        if ((x & 0x11111111) != 0) {
            level += 1;
        }
        return level;
    }

    boolean isLeaf() {
        return ((int) id & 1) != 0;
    }

    public boolean isFace() {
        return (id & (lowestOnBitForLevel(0) - 1)) == 0;
    }

    public int childPosition(int level) {
        return (int) (id >>> (2 * (MAX_LEVEL - level) + 1)) & 3;
    }

    S2CellId rangeMin() {
        return new S2CellId(id - (lowestOnBit() - 1));
    }

    S2CellId rangeMax() {
        return new S2CellId(id + (lowestOnBit() - 1));
    }

    public boolean contains(S2CellId other) {
        return other.greaterOrEquals(rangeMin()) && other.lessOrEquals(rangeMax());
    }

    public boolean intersects(S2CellId other) {
        return other.rangeMin().lessOrEquals(rangeMax())
                && other.rangeMax().greaterOrEquals(rangeMin());
    }

    public S2CellId parent() {
        long newLsb = lowestOnBit() << 2;
        return new S2CellId((id & -newLsb) | newLsb);
    }

    public S2CellId parent(int level) {
        long newLsb = lowestOnBitForLevel(level);
        return new S2CellId((id & -newLsb) | newLsb);
    }

    S2CellId childBegin() {
        long oldLsb = lowestOnBit();
        return new S2CellId(id - oldLsb + (oldLsb >>> 2));
    }

    S2CellId childBegin(int level) {
        return new S2CellId(id - lowestOnBit() + lowestOnBitForLevel(level));
    }

    S2CellId childEnd() {
        long oldLsb = lowestOnBit();
        return new S2CellId(id + oldLsb + (oldLsb >>> 2));
    }

    S2CellId childEnd(int level) {
        return new S2CellId(id + lowestOnBit() + lowestOnBitForLevel(level));
    }

    public S2CellId next() {
        return new S2CellId(id + (lowestOnBit() << 1));
    }

    S2CellId prev() {
        return new S2CellId(id - (lowestOnBit() << 1));
    }

    S2CellId nextWrap() {
        S2CellId n = next();
        if (unsignedLongLessThan(n.id, WRAP_OFFSET)) {
            return n;
        }
        return new S2CellId(n.id - WRAP_OFFSET);
    }

    S2CellId prevWrap() {
        S2CellId p = prev();
        if (p.id < WRAP_OFFSET) {
            return p;
        }
        return new S2CellId(p.id + WRAP_OFFSET);
    }

    String toToken() {
        if (id == 0) {
            return "X";
        }
        String hex = Long.toHexString(id).toLowerCase(Locale.ENGLISH);
        StringBuilder sb = new StringBuilder(16);
        for (int i = hex.length(); i < 16; i++) {
            sb.append('0');
        }
        sb.append(hex);
        for (int len = 16; len > 0; len--) {
            if (sb.charAt(len - 1) != '0') {
                return sb.substring(0, len);
            }
        }
        throw new RuntimeException("Shouldn't make it here");
    }

    void getEdgeNeighbors(S2CellId[] neighbors) {
        MutableInteger i = new MutableInteger(0);
        MutableInteger j = new MutableInteger(0);
        int level = this.level();
        int size = 1 << (MAX_LEVEL - level);
        int face = toFaceIJOrientation(i, j, null);

        // Edges 0, 1, 2, 3 are in the S, E, N, W directions.
        neighbors[0] = fromFaceIJSame(face, i.intValue(), j.intValue() - size,
                j.intValue() - size >= 0).parent(level);
        neighbors[1] = fromFaceIJSame(face, i.intValue() + size, j.intValue(),
                i.intValue() + size < MAX_SIZE).parent(level);
        neighbors[2] = fromFaceIJSame(face, i.intValue(), j.intValue() + size,
                j.intValue() + size < MAX_SIZE).parent(level);
        neighbors[3] = fromFaceIJSame(face, i.intValue() - size, j.intValue(),
                i.intValue() - size >= 0).parent(level);
    }

    void getVertexNeighbors(int level, List<S2CellId> output) {
        MutableInteger i = new MutableInteger(0);
        MutableInteger j = new MutableInteger(0);
        int face = toFaceIJOrientation(i, j, null);
        int halfsize = 1 << (MAX_LEVEL - (level + 1));
        int size = halfsize << 1;
        boolean isame, jsame;
        int ioffset, joffset;
        if ((i.intValue() & halfsize) != 0) {
            ioffset = size;
            isame = (i.intValue() + size) < MAX_SIZE;
        } else {
            ioffset = -size;
            isame = (i.intValue() - size) >= 0;
        }
        if ((j.intValue() & halfsize) != 0) {
            joffset = size;
            jsame = (j.intValue() + size) < MAX_SIZE;
        } else {
            joffset = -size;
            jsame = (j.intValue() - size) >= 0;
        }
        output.add(parent(level));
        output
                .add(fromFaceIJSame(face, i.intValue() + ioffset, j.intValue(), isame)
                        .parent(level));
        output
                .add(fromFaceIJSame(face, i.intValue(), j.intValue() + joffset, jsame)
                        .parent(level));
        if (isame || jsame) {
            output.add(fromFaceIJSame(face, i.intValue() + ioffset,
                    j.intValue() + joffset, isame && jsame).parent(level));
        }
    }

    void getAllNeighbors(int nbrLevel, List<S2CellId> output) {
        MutableInteger i = new MutableInteger(0);
        MutableInteger j = new MutableInteger(0);
        int face = toFaceIJOrientation(i, j, null);
        int size = 1 << (MAX_LEVEL - level());
        i.setValue(i.intValue() & -size);
        j.setValue(j.intValue() & -size);
        int nbrSize = 1 << (MAX_LEVEL - nbrLevel);
        for (int k = -nbrSize; ; k += nbrSize) {
            boolean sameFace;
            if (k < 0) {
                sameFace = (j.intValue() + k >= 0);
            } else if (k >= size) {
                sameFace = (j.intValue() + k < MAX_SIZE);
            } else {
                sameFace = true;
                output.add(fromFaceIJSame(face, i.intValue() + k,
                        j.intValue() - nbrSize, j.intValue() - size >= 0).parent(nbrLevel));
                output.add(fromFaceIJSame(face, i.intValue() + k, j.intValue() + size,
                        j.intValue() + size < MAX_SIZE).parent(nbrLevel));
            }
            output.add(fromFaceIJSame(face, i.intValue() - nbrSize,
                    j.intValue() + k, sameFace && i.intValue() - size >= 0).parent(
                    nbrLevel));
            output.add(fromFaceIJSame(face, i.intValue() + size, j.intValue() + k,
                    sameFace && i.intValue() + size < MAX_SIZE).parent(nbrLevel));
            if (k >= size) {
                break;
            }
        }
    }

    public int toFaceIJOrientation(MutableInteger pi, MutableInteger pj,
                                   MutableInteger orientation) {
        int face = this.face();
        int bits = (face & SWAP_MASK);
        for (int k = 7; k >= 0; --k) {
            bits = getBits1(pi, pj, k, bits);
        }
        if (orientation != null) {
            if ((lowestOnBit() & 0x1111111111111110L) != 0) {
                bits ^= S2.SWAP_MASK;
            }
            orientation.setValue(bits);
        }
        return face;
    }

    private int getBits1(MutableInteger i, MutableInteger j, int k, int bits) {
        final int nbits = (k == 7) ? (MAX_LEVEL - 7 * LOOKUP_BITS) : LOOKUP_BITS;
        bits += (((int) (id >>> (k * 2 * LOOKUP_BITS + 1)) &
                ((1 << (2 * nbits)) - 1))) << 2;
        bits = LOOKUP_IJ[bits];
        i.setValue(i.intValue()
                + ((bits >> (LOOKUP_BITS + 2)) << (k * LOOKUP_BITS)));
        j.setValue(j.intValue()
                + ((((bits >> 2) & ((1 << LOOKUP_BITS) - 1))) << (k * LOOKUP_BITS)));
        bits &= (SWAP_MASK | INVERT_MASK);
        return bits;
    }

    public long lowestOnBit() {
        return id & -id;
    }

    @Override
    public boolean equals(Object that) {
        if (!(that instanceof S2CellId)) {
            return false;
        }
        S2CellId x = (S2CellId) that;
        return id() == x.id();
    }

    public boolean lessThan(S2CellId x) {
        return unsignedLongLessThan(id, x.id);
    }

    public boolean greaterThan(S2CellId x) {
        return unsignedLongGreaterThan(id, x.id);
    }

    public boolean lessOrEquals(S2CellId x) {
        return unsignedLongLessThan(id, x.id) || id == x.id;
    }

    public boolean greaterOrEquals(S2CellId x) {
        return unsignedLongGreaterThan(id, x.id) || id == x.id;
    }

    @Override
    public int hashCode() {
        return (int) ((id >>> 32) + id);
    }

    @Override
    public String toString() {
        return "(face=" + face() + ", pos=" + Long.toHexString(pos()) + ", level="
                + level() + ")";
    }

    @Override
    public int compareTo(S2CellId that) {
        return unsignedLongLessThan(this.id, that.id) ? -1 :
                unsignedLongGreaterThan(this.id, that.id) ? 1 : 0;
    }
}
