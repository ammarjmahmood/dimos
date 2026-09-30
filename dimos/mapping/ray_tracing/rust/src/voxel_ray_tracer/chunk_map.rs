// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Voxel storage in fixed 8x8x8 chunks: an occupancy mask plus the chunk's
//! voxels packed in mask order.

use ahash::AHashMap;

use super::{Voxel, VoxelKey};

pub type ChunkKey = (i32, i32, i32);

const CHUNK_BITS: i32 = 3;
/// Voxels per chunk edge.
pub const CHUNK_EDGE: i32 = 1 << CHUNK_BITS;
const LOCAL_MASK: i32 = CHUNK_EDGE - 1;
const SLOTS: usize = 1 << (3 * CHUNK_BITS);
const WORDS: usize = SLOTS / 64;

#[inline]
pub fn chunk_of(key: VoxelKey) -> ChunkKey {
    (
        key.0 >> CHUNK_BITS,
        key.1 >> CHUNK_BITS,
        key.2 >> CHUNK_BITS,
    )
}

#[inline]
fn slot_of(key: VoxelKey) -> usize {
    (((key.0 & LOCAL_MASK) << (2 * CHUNK_BITS))
        | ((key.1 & LOCAL_MASK) << CHUNK_BITS)
        | (key.2 & LOCAL_MASK)) as usize
}

#[inline]
fn key_of(chunk: ChunkKey, slot: usize) -> VoxelKey {
    let s = slot as i32;
    (
        (chunk.0 << CHUNK_BITS) | (s >> (2 * CHUNK_BITS)),
        (chunk.1 << CHUNK_BITS) | ((s >> CHUNK_BITS) & LOCAL_MASK),
        (chunk.2 << CHUNK_BITS) | (s & LOCAL_MASK),
    )
}

#[derive(Clone, Default)]
pub struct Chunk {
    occupied: [u64; WORDS],
    voxels: Vec<Voxel>,
}

impl Chunk {
    #[inline]
    fn has(&self, slot: usize) -> bool {
        self.occupied[slot >> 6] & (1 << (slot & 63)) != 0
    }

    /// Index into the packed voxels: occupied slots below this one.
    #[inline]
    fn rank(&self, slot: usize) -> usize {
        let word = slot >> 6;
        let below: u32 = self.occupied[..word].iter().map(|w| w.count_ones()).sum();
        (below + (self.occupied[word] & ((1 << (slot & 63)) - 1)).count_ones()) as usize
    }

    #[inline]
    pub fn get(&self, key: VoxelKey) -> Option<&Voxel> {
        let slot = slot_of(key);
        self.has(slot).then(|| &self.voxels[self.rank(slot)])
    }

    /// Voxels with their keys, in slot order.
    pub fn iter(&self, chunk: ChunkKey) -> impl Iterator<Item = (VoxelKey, &Voxel)> {
        let slots = (0..WORDS).flat_map(move |w| {
            let mut bits = self.occupied[w];
            std::iter::from_fn(move || {
                (bits != 0).then(|| {
                    let b = bits.trailing_zeros() as usize;
                    bits &= bits - 1;
                    w * 64 + b
                })
            })
        });
        slots
            .zip(&self.voxels)
            .map(move |(slot, v)| (key_of(chunk, slot), v))
    }

    pub fn len(&self) -> usize {
        self.voxels.len()
    }
}

/// Voxels keyed by grid position, stored chunk by chunk.
#[derive(Clone, Default)]
pub struct ChunkMap {
    chunks: AHashMap<ChunkKey, Chunk>,
    len: usize,
}

impl ChunkMap {
    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn clear(&mut self) {
        self.chunks.clear();
        self.len = 0;
    }

    #[inline]
    pub fn get(&self, key: &VoxelKey) -> Option<&Voxel> {
        self.chunks.get(&chunk_of(*key))?.get(*key)
    }

    #[inline]
    pub fn get_mut(&mut self, key: &VoxelKey) -> Option<&mut Voxel> {
        let chunk = self.chunks.get_mut(&chunk_of(*key))?;
        let slot = slot_of(*key);
        if !chunk.has(slot) {
            return None;
        }
        let i = chunk.rank(slot);
        Some(&mut chunk.voxels[i])
    }

    #[inline]
    pub fn contains_key(&self, key: &VoxelKey) -> bool {
        self.chunks
            .get(&chunk_of(*key))
            .is_some_and(|c| c.has(slot_of(*key)))
    }

    /// The voxel at `key`, inserted with defaults when absent.
    pub fn get_or_insert_default(&mut self, key: VoxelKey) -> &mut Voxel {
        let chunk = self.chunks.entry(chunk_of(key)).or_default();
        let slot = slot_of(key);
        let i = chunk.rank(slot);
        if !chunk.has(slot) {
            chunk.occupied[slot >> 6] |= 1 << (slot & 63);
            chunk.voxels.insert(i, Voxel::default());
            self.len += 1;
        }
        &mut chunk.voxels[i]
    }

    /// Store `voxel` at `key`, returning the voxel it replaced.
    pub fn insert(&mut self, key: VoxelKey, voxel: Voxel) -> Option<Voxel> {
        let chunk = self.chunks.entry(chunk_of(key)).or_default();
        let slot = slot_of(key);
        let i = chunk.rank(slot);
        if chunk.has(slot) {
            return Some(std::mem::replace(&mut chunk.voxels[i], voxel));
        }
        chunk.occupied[slot >> 6] |= 1 << (slot & 63);
        chunk.voxels.insert(i, voxel);
        self.len += 1;
        None
    }

    /// Remove the voxel at `key`, dropping its chunk once empty.
    pub fn remove(&mut self, key: &VoxelKey) -> Option<Voxel> {
        let ck = chunk_of(*key);
        let chunk = self.chunks.get_mut(&ck)?;
        let slot = slot_of(*key);
        if !chunk.has(slot) {
            return None;
        }
        let i = chunk.rank(slot);
        chunk.occupied[slot >> 6] &= !(1 << (slot & 63));
        let voxel = chunk.voxels.remove(i);
        if chunk.voxels.is_empty() {
            self.chunks.remove(&ck);
        }
        self.len -= 1;
        Some(voxel)
    }

    pub fn chunk(&self, key: &ChunkKey) -> Option<&Chunk> {
        self.chunks.get(key)
    }

    pub fn chunks(&self) -> impl Iterator<Item = (ChunkKey, &Chunk)> {
        self.chunks.iter().map(|(&k, c)| (k, c))
    }

    pub fn chunk_count(&self) -> usize {
        self.chunks.len()
    }

    pub fn iter(&self) -> impl Iterator<Item = (VoxelKey, &Voxel)> {
        self.chunks.iter().flat_map(|(&k, c)| c.iter(k))
    }

    pub fn keys(&self) -> impl Iterator<Item = VoxelKey> + '_ {
        self.iter().map(|(k, _)| k)
    }

    pub fn values(&self) -> impl Iterator<Item = &Voxel> {
        self.chunks.values().flat_map(|c| c.voxels.iter())
    }
}

impl std::ops::Index<&VoxelKey> for ChunkMap {
    type Output = Voxel;

    fn index(&self, key: &VoxelKey) -> &Voxel {
        self.get(key).expect("no voxel at key")
    }
}

/// Voxel lookups along a path, resolving the chunk only when the path enters a
/// new one.
pub struct ChunkCursor<'a> {
    map: &'a ChunkMap,
    key: Option<ChunkKey>,
    chunk: Option<&'a Chunk>,
}

impl<'a> ChunkCursor<'a> {
    pub fn new(map: &'a ChunkMap) -> Self {
        Self {
            map,
            key: None,
            chunk: None,
        }
    }

    #[inline]
    pub fn get(&mut self, key: VoxelKey) -> Option<&'a Voxel> {
        let ck = chunk_of(key);
        if self.key != Some(ck) {
            self.key = Some(ck);
            self.chunk = self.map.chunks.get(&ck);
        }
        self.chunk?.get(key)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slot_and_key_round_trip_across_negative_chunks() {
        for key in [
            (0, 0, 0),
            (-1, -1, -1),
            (7, -8, 9),
            (-9, 15, -16),
            (123, -456, 789),
        ] {
            assert_eq!(key_of(chunk_of(key), slot_of(key)), key);
        }
        assert_eq!(chunk_of((-1, 0, 7)), (-1, 0, 0));
        assert_eq!(chunk_of((-8, -9, 8)), (-1, -2, 1));
    }

    #[test]
    fn packed_order_survives_inserts_and_removes() {
        let mut map = ChunkMap::default();
        let keys = [
            (3, 1, 2),
            (0, 0, 0),
            (7, 7, 7),
            (-1, 2, 3),
            (1, 0, 0),
            (0, 0, 1),
        ];
        for (i, &k) in keys.iter().enumerate() {
            assert!(map.insert(k, Voxel::with_health(i as i32)).is_none());
        }
        assert_eq!(map.len(), keys.len());
        for (i, k) in keys.iter().enumerate() {
            assert_eq!(map[k].health, i as i32);
        }

        assert_eq!(map.remove(&(0, 0, 0)).map(|v| v.health), Some(1));
        assert!(map.remove(&(0, 0, 0)).is_none());
        map.get_mut(&(7, 7, 7)).unwrap().health = 70;
        assert_eq!(
            map.insert((1, 0, 0), Voxel::with_health(40))
                .map(|v| v.health),
            Some(4)
        );
        map.get_or_insert_default((2, 2, 2)).health = 22;

        let mut got: Vec<(VoxelKey, i32)> = map.iter().map(|(k, v)| (k, v.health)).collect();
        got.sort();
        assert_eq!(
            got,
            vec![
                ((-1, 2, 3), 3),
                ((0, 0, 1), 5),
                ((1, 0, 0), 40),
                ((2, 2, 2), 22),
                ((3, 1, 2), 0),
                ((7, 7, 7), 70),
            ]
        );
        assert_eq!(map.len(), 6);
    }

    #[test]
    fn emptied_chunk_is_dropped() {
        let mut map = ChunkMap::default();
        map.insert((-3, 4, 5), Voxel::default());
        assert_eq!(map.chunk_count(), 1);
        map.remove(&(-3, 4, 5));
        assert_eq!(map.chunk_count(), 0);
        assert!(map.is_empty());
    }

    #[test]
    fn cursor_matches_map_lookups() {
        let mut map = ChunkMap::default();
        for x in -20..20 {
            if x % 3 != 0 {
                map.insert((x, 1, -1), Voxel::with_health(x));
            }
        }
        let mut cursor = ChunkCursor::new(&map);
        for x in -20..20 {
            let key = (x, 1, -1);
            assert_eq!(
                cursor.get(key).map(|v| v.health),
                map.get(&key).map(|v| v.health)
            );
        }
    }
}
