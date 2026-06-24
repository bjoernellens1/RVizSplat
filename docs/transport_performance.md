# Transport Performance Benchmark

**Scene:** TUM fr3 desk (`kf_e4_cap300k_fr3_20260622`, iteration 30000)
**Splat count:** 21,570
**Host:** cps-wkstn-hpz2minig1a (AMD ROCm workstation)
**Middleware:** rmw_zenoh_cpp (loopback, host network)
**Measured:** `ros2 topic bw` over 8 s windows per rate

## Bandwidth by transport and publish rate

| Transport | Msg size | 3 Hz | 5 Hz | 10 Hz | 30 Hz |
|---|---|---|---|---|---|
| **SplatArray** (legacy) | 8.28 MB | 26.6 MB/s | 42.4 MB/s | 83.4 MB/s | 122 MB/s ⚠️ |
| **Blob (compact)** | 0.69 MB | 2.1 MB/s | 3.6 MB/s | 7.0 MB/s | 12.5 MB/s |
| **Tile stream** | 0.18–162 KB | 0.3 MB/s | 0.9 MB/s | 1.9 MB/s | 4.7 MB/s |
| **SnapshotRef** | 189 B | 600 B/s | 946 B/s | 1.9 KB/s | 5.7 KB/s |

## Size reduction

| Transport | Bytes / splat | vs SplatArray |
|---|---|---|
| SplatArray (legacy) | ~383 B | 1× (baseline) |
| Blob compact (`compact_dc_fp16_cov_rgba_v1`) | 32 B | **12× smaller** |
| Tile stream (same compact encoding, per-tile) | 32 B + framing | ~12× smaller |
| SnapshotRef | 189 B total (URI only) | no data in-band |

PLY file size for this scene: 5.2 MB → compact payload: 0.69 MB (**7.8× smaller than raw PLY**).

## Saturation behaviour

- **SplatArray at 30 Hz** requires 122 MB/s — saturates any real network link and DDS
  middleware long before 30 Hz. On a 1 Gbps LAN this tops out around 12 Hz.
- **Blob at 30 Hz** hits ~12.5 MB/s on loopback. On a 100 Mbps link the ceiling is
  ~1.5 Hz; on 1 Gbps ~15 Hz.
- **Tile stream** bandwidth scales with the number of changed tiles, not total splat
  count — ideal for incremental live SLAM updates where only a spatial subset changes.
- **SnapshotRef** transfers zero splat data over ROS; the subscriber loads the file
  directly from the path in the URI. Only viable when publisher and subscriber share
  a filesystem.

## Practical operating points (this scene, zenoh loopback)

| Use case | Recommended transport | Max safe Hz |
|---|---|---|
| One-shot static map | Blob (latched) | N/A (single publish) |
| Live SLAM snapshot updates | Blob | ≤ 10 Hz |
| Live SLAM incremental updates | Tile stream | ≤ 30 Hz (changed tiles only) |
| Co-located processes | SnapshotRef | any |
| Legacy / compatibility | SplatArray | ≤ 3 Hz (small scenes only) |

## Encoding throughput (Python publisher, no ROS)

Measured with `bench/bench_encoding.py` on the same host:

| Scene | Splats | Encode time | Throughput |
|---|---|---|---|
| fixture (golden) | 600 | 0.5 ms | 1.3M splats/s |
| floor3 iter 8000 | 47,758 | 24 ms | 2.0M splats/s |
| floor3 iter 18000 | 47,941 | 23 ms | 2.1M splats/s |

CRC-64/ECMA-182 is computed per chunk (pure-Python `crcmod`, no C extension available
on this host). For 1 MiB chunks the CRC adds ~80 ms/chunk — acceptable for latched
one-shot publishes, overhead for high-rate streaming. Disable with `enable_crc=False`
if throughput is the priority.

## Known issues (resolved)

- **Tile stream not rendering in RViz2** — fixed. Three root causes identified and
  patched:
  1. **QoS durability mismatch** (`tile_stream_source.cpp`): subscriber used
     `VOLATILE`, publisher used `TRANSIENT_LOCAL`. Subscriber now uses
     `TRANSIENT_LOCAL` with depth=100.
  2. **QoS history depth too small** (`rate_publisher_all.py`): `LATCHED_QOS`
     depth=10 could not hold a full CLEAR_ALL+UPSERT_TILE×N+COMMIT cycle for
     multi-tile scenes. `TILE_QOS` with depth=500 is now used for tile publishers.
  3. **Orphaned pending ops on failed COMMIT** (`tile_cache.cpp`): when COMMIT
     arrived with incomplete tile assemblies, pending ops and assembly entries were
     leaked. `TileAssembler::discardVersion()` now cleans them up on error.
  4. **Version not incremented between cycles** (`rate_publisher_all.py`): tile
     messages were pre-built once with version=1; now rebuilt each cycle with the
     incremented version.
- **SplatArray serialization** is done per-splat in Python (loop), making it slow for
  large scenes. The splatograph-rvizsplat-transport fork uses a vectorised CDR
  serialiser (~100× faster) for live publishing.
