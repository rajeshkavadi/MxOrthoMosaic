# MxOrthoMosaic — On-board Real-time Reflectance Orthomosaic
## Parrot Sequoia + NVIDIA Jetson Orin, no RTK

Status: methodology v0.1 (pre-code). Items marked **[verify]** are from memory or vendor docs not yet checked against our own hardware/data; each one has a Phase 0/1 test that settles it.

---

## 0. Scope and corrections to the brief

**Goal.** While the drone flies, produce on the Orin, within ~1 s of each capture:
- A georeferenced **reflectance orthomosaic** (4 bands).
- **Vegetation index** maps.

These are the Sequoia equivalents of the Agisoft "Mx" deliverables. A full-quality product set is then refined **on board after landing** (no data offload).

**Corrections to the brief:**

| Brief said | Actual Sequoia spec | Consequence |
|---|---|---|
| Bands R, NIR, RE and **B** | **Green 550±40, Red 660±40, Red-edge 735±10, NIR 790±40 nm**. There is no blue band. | No EVI or true-colour from the multispectral bands. Use NDVI, NDRE, GNDVI, OSAVI, MSAVI2, CIre. |
| "1.2 MB" per band | **1.2 MP** (1280×960) per band. 10-bit data in a 16-bit TIFF, about **2.4 MB** per band. | About 10 MB per capture for multispectral |
| "RGB 16 MB" | **16 MP** (4608×3456) JPEG, about 5–8 MB, **rolling shutter**, not radiometrically calibrated | RGB is used for visual context only, never for reflectance |
| "1 s is enough for all processing" | True for the per-capture chain (Tier A). False for dense reconstruction, DSM/DTM and global bundle adjustment. | Two-tier design (§2) |

**Not in scope in flight:**
- Thermal: the Sequoia has no thermal band.
- Dense DSM/DTM.
- Ground-control-point marking.
- Global optimisation over the whole flight.

These move to Tier B (post-landing, on board).

---

## 1. System overview

```
                 ┌──────────── Sunshine sensor (top of airframe; 4 bands + GPS + IMU) ────────────┐
                 │                                                                                │ cable
Flight controller (PX4/ArduPilot) ──MAVLink (UART/USB)──┐                                  Sequoia body (underneath)
  GNSS (no RTK), EKF attitude, time                     │                                           │ USB / Wi-Fi
                                                        ▼                                           ▼
                                  ┌─────────────────────── Jetson Orin ──────────────────────────────────┐
                                  │ P1 Acquire   → P2 Radiometry → P3 Geometry → P4 Pose → P5 Ortho+Blend │
                                  │ P0 Pose log  ─────────────────────────────────┘          → P6 Products│
                                  │ Raw store (NVMe, always)        Tile store (COG)      Telemetry preview│
                                  └──────────────────────────────────────────────────────────────────────┘
```

**Principle: never lose raw data.** Every capture and every pose sample is written to NVMe before processing. Real-time processing may degrade; the raw record never does. The raw record is also what we replay in Colab to test.

---

## 2. Two-tier product strategy

| Product | Tier A: in flight (≤ 1 s per capture) | Tier B: on board after landing |
|---|---|---|
| Per-image reflectance (G, R, RE, NIR) | ✅ pre-flight panel + sunshine sensor | ✅ pre- and post-flight panels with drift interpolation |
| Band co-registration | ✅ fixed transform per band (pre-computed) | ✅ refined per image |
| Image poses | GNSS/IMU prior + sliding-window visual refinement | Global bundle adjustment (+ GCPs if surveyed) |
| Surface model | Flat plane (paddy), optionally a coarse running height grid | Dense DSM; DTM from ground classification |
| Reflectance orthomosaic | ✅ approximate (no relief correction) | ✅ full orthorectification on the DSM |
| Index maps | ✅ | ✅ |
| RGB orthomosaic | ❌ (bandwidth; rolling shutter) | ✅ |
| Absolute position accuracy | ~2–5 m (consumer GNSS, no RTK) | ~2–5 m; **< 0.1 m with GCPs** |

The Tier A output serves scouting and spraying decisions during or right after the flight. Tier B is the archival product for comparisons across the season.

---

## 3. Hardware integration

### 3.1 Bill of materials
- **Parrot Sequoia** (body + sunshine sensor + calibration panel). Confirm whether it is **Sequoia or Sequoia+**. Sequoia+ has an absolutely calibrated sunshine sensor; the original's is relative only **[verify]**.
- **Jetson Orin:**
  - Orin NX 16 GB is the recommended size/weight/power choice (~10–25 W).
  - AGX Orin only if the payload allows (~1.4 kg dev kit, up to 60 W).
- **NVMe SSD** ≥ 256 GB.
- **Flight controller** with MAVLink over a dedicated UART to the Orin.
- **Power:** a regulated, isolated supply from the flight battery (Orin 9–20 V input; Sequoia supply per its spec).

### 3.2 Mounting
- **Sequoia body:** nadir, on a vibration-damped plate. Long image side across the flight track. Record the fixed rotation between the camera and the flight controller frame (boresight; §6.1).
- **Sunshine sensor:** on top of the airframe, level with the flight controller frame, with no shadowing from GNSS mast or props. Record its mounting angles.
- **Orin:** ventilated with forced airflow. At 36 °C ambient, a sealed box will throttle the GPU.

### 3.3 Interfaces
| Link | Purpose | Notes |
|---|---|---|
| Orin ↔ Sequoia | Trigger, image transfer | Sequoia exposes USB (mass storage / PTP) and an HTTP API over Wi-Fi/USB **[verify which modes allow capture during transfer]** |
| Orin ↔ FC | Pose, time, flight state | MAVLink 2 at ≥ 50 Hz: `ATTITUDE`, `GLOBAL_POSITION_INT`, `GPS_RAW_INT`, `SYSTEM_TIME` |
| Orin ↔ GCS | Preview, health | Optional low-rate NDVI thumbnails |

### 3.4 Electromagnetic interference
USB 3 radiates around 2.4 GHz and degrades GNSS on Jetsons. Use USB 2 for the Sequoia where possible, add shielding, and keep the GNSS antenna away from the Orin. Acceptance: GNSS C/N0 drop < 3 dB with everything powered.

---

## 4. Flight design for Sequoia

Camera constants (multispectral bands): focal length 3.98 mm, pixel 3.75 µm → **f ≈ 1061 px**; FOV ≈ 61.9° × 48.5° **[verify from XMP]**.

- **GSD** = H × 3.75 µm / 3.98 mm ≈ **0.094 × H cm** (H in metres).
- **Footprint** ≈ 1.20 H (across track) × 0.90 H (along track).
- At 1 Hz capture, **forward overlap** = 1 − v / (0.90 H).

| Altitude H | GSD | Footprint | Max speed for 80% forward overlap | Line spacing at 70% side overlap |
|---|---|---|---|---|
| 30 m | 2.8 cm | 36 × 27 m | 5.4 m/s | 10.8 m |
| **60 m (recommended)** | **5.7 cm** | 72 × 54 m | 10.8 m/s → **fly 8 m/s (85%)** | 21.6 m |
| 120 m | 11.3 cm | 144 × 108 m | 21.6 m/s | 43 m |

Rationale for 60 m:
- **Image matching.** Paddy canopy has little texture, so more ground per image helps matching.
- **Band alignment.** Lens-to-lens parallax ≈ 2 cm × 1061 px / 60 m ≈ **0.35 px**, so a fixed band transform is valid (§5.4).
- **Motion blur.** At 8 m/s with 1/500 s exposure, the drone moves 1.6 cm, well under the 5.7 cm GSD.

Operational rules (from the IIT Tirupati paddy procedure plus radiometric best practice):
- Fly with **sun elevation > 30°**.
- Wind < 5 m/s.
- Avoid broken cloud. If unavoidable, accept that sunshine correction is approximate and flag the flight.
- **Turns:** at line ends, keep turns gentle (bank < 15°) or mark captures taken during turns as "turn" (tilt flag, §5.6).

---

## 5. Radiometric methodology (Tier A, per capture)

### 5.1 Inputs per band image (from XMP/EXIF) **[verify tag names with `exiftool -X` on our files]**
| Tag | Meaning |
|---|---|
| `Camera:SensorModel` = A, B, C | Per-band sensor response coefficients |
| `FNumber` (f) | Aperture (fixed, ~2.2) |
| `ExposureTime` (ε, s) | Integration time |
| `ISOSpeed` → γ = ISO/100 | Gain |
| `Camera:VignettingPolynomial2DName` / `…2D` | Vignetting polynomial exponents and coefficients |
| `Camera:FisheyePolynomial`, `FisheyeAffineMatrix`, `PrincipalPoint` | Lens model (§6.2) |
| `Camera:IrradianceList` (or `Irradiance*`) | Sunshine samples: CH0 count, gain index, integration time, attitude, timestamp |
| GPS, `SubSecDateTimeOriginal` | Camera position and time |

### 5.2 Raw value → at-sensor radiance (Parrot application note SEQ-AN-01)
```
L(x,y) = f² · (p(x,y) − B) / (A · ε · γ + C)
```
- **p** is the raw value.
- **L** is proportional to radiance, in a band-specific arbitrary unit. That doesn't matter, because the panel step (§5.4) cancels the unit.
- Pixels at or near saturation (p ≥ 0.98 × max) are masked out.

### 5.3 Vignetting (SEQ-AN-02)
- Evaluate the 2D polynomial V(x,y) on normalised coordinates **once per band** and cache it on the GPU.
- Apply as **L_v = L / V** (or × V, whichever the note specifies) **[verify direction with a flat-field test]**: image a uniformly lit panel or overcast sky. The corrected image must be flat to within ±2% from centre to corner.

### 5.4 Sunshine irradiance
For each capture and band k:
- Take the irradiance sample nearest in time: **E_k ≈ CH0 / (gain(g_idx) · t_int)**, using the gain-index table **[verify]**.
- **Tilt correction.** Compute the sun position (zenith θs, azimuth φs) from the UTC time and location (NREL SPA algorithm). The sensor normal **n** comes from the sunshine IMU, or the FC attitude plus the mounting rotation. Correct the direct-beam part:
  ```
  E_corr = E_meas · (D·cosθs + (1−D)) / (D·cos∠(n, sun) + (1−D))
  ```
  - D is the direct fraction: ~0.8 on clear sky, ~0.0 under full overcast.
  - Estimate D from the pre-flight panel series or set it per flight.
- **Rejection.** Discard samples with tilt > 15° or |ΔE| > 3σ of a rolling 10 s median. Use the **rolling median** of the valid samples.
- **Original Sequoia:** E_k is relative only. It drives *changes* in light, never the absolute scale. The absolute scale comes from the panel.

### 5.5 Panel calibration (before take-off; repeat after landing for Tier B)
- **Panel reflectance:** the per-band certified values ρ_panel,k from the panel's label or certificate. Without these the whole chain is uncalibrated; **get them first**.
- **Capture protocol:**
  - Panel on the ground, level, in full sun, with no shadow from the operator.
  - Sequoia held about 1 m above the panel, nadir.
  - Sunshine sensor unobstructed.
  - 3 captures.
- **Panel detection:**
  - Stick an **ArUco marker** next to the panel; it gives automatic, deterministic detection.
  - Use the inner 60% of the panel area.
  - Require all 4 bands found, no saturation, and coefficient of variation < 3%. This is the Agisoft "5/5" rule adapted to 4 bands.
- **Calibration constant per band:**
  ```
  K_k = ρ_panel,k · E_panel,k / L̄_panel,k
  ```

### 5.6 Reflectance (in flight)
```
ρ_k(x,y) = K_k · L_v,k(x,y) / E_corr,k(t)
```
- **Storage:** float16 in memory. Export as **uint16 = round(ρ × 32768)**, matching the Agisoft convention in Mx_edited_1.docx (B ÷ 32768), or as float32 COG; that's a setting.
- **Tier B drift correction:** K_k(t) is linearly interpolated between the pre- and post-flight panels.

### 5.7 Quality flags per capture
| Flag | Rule |
|---|---|
| `SAT` | > 1% of pixels saturated in any band |
| `DARK` | Median signal < 5% of full scale |
| `BLUR` | Variance of the Laplacian of NIR < 30% of the flight median |
| `TILT` | FC tilt > 15° |
| `IRR` | Irradiance outlier or missing sample |
| `REG` | Visual registration failed (§7) |

Flagged captures are still mosaicked, but at lower priority in blending (§8).

---

## 6. Geometric methodology (Tier A, per capture)

### 6.1 Calibrations done once, on the ground
1. **Lens intrinsics per band.** Start from the XMP fisheye model; refine with a checkerboard/ChArUco calibration if the reprojection error is > 0.5 px.
2. **Band-to-band transform.**
   - Choose a reference band: **NIR** (best canopy texture) or Green.
   - On 20 textured captures at mission altitude, estimate a homography per band with ECC on gradient images, then take the robust average.
   - Validity check: the residual must be < 0.5 px over the 20 captures. Parallax is about 0.35 px at 60 m, so a fixed transform holds above ~40 m. Below that, enable per-capture ECC refinement.
3. **Boresight.** The rotation R_cam←FC between the camera and the FC body frame comes from a calibration flight over a textured site, estimated in the Tier B bundle adjustment. Store it in config.
4. **Time offset** between Sequoia timestamps and FC time: estimated from the same calibration flight by minimising reprojection error over an offset Δt. Target |Δt| error < 10 ms. At 8 m/s, 10 ms ≈ 8 cm ≈ 1.4 px.

### 6.2 Per capture
1. **Undistort** each band with precomputed `cv2.remap` maps (GPU), directly into the **reference-band frame**. The band homography is folded into the same remap, so there is one resampling only.
2. **Pose prior:**
   - Interpolate the FC position (`GLOBAL_POSITION_INT`) and attitude (`ATTITUDE`) at the capture timestamp: SLERP for attitude, linear for position.
   - Apply the boresight.
   - Convert to local ENU, then to the UTM zone of the field.
3. **Ground model:**
   - A plane z = z₀, with z₀ = take-off ground height from the FC home altitude.
   - For paddy, terrain varies by under 0.5 m across a field. Canopy height is added as a configurable offset that is updated per crop stage.

---

## 7. Pose refinement (sliding window, no RTK)

**Why.** Consumer GNSS gives 1.5–3 m errors that wander, and attitude errors of ~1° project to ~1 m at 60 m. Mosaic seams would show multi-metre jumps. Visual refinement fixes **relative** consistency. Absolute accuracy stays at GNSS level (Tier B with GCPs fixes that).

**Per new capture i:**
1. **Features** on reference-band reflectance downsampled to 640×480:
   - Default: ORB 2000 keypoints.
   - Upgrade: SuperPoint + LightGlue with TensorRT.
   - Contrast-limited histogram equalisation before detection; paddy texture is weak.
2. **Match** against the 3 previous captures, plus the nearest capture from the previous flight line (side overlap), selected by pose prior.
3. **Geometric check:** a homography with RANSAC. Inliers ≥ 40 and inlier ratio ≥ 0.3, else flag `REG` and use the pose prior alone.
4. **Sliding-window least squares** over the last N = 10 captures:
   - **Unknowns:** 6-DoF pose per capture. The plane height z₀ is shared.
   - **Residuals:**
     - Tie-point reprojection through the plane-induced homography.
     - GNSS position prior (σ = 2 m horizontal, 4 m vertical).
     - Attitude prior (σ = 1°).
     - Smoothness between consecutive poses.
   - **Solver:** `scipy.optimize.least_squares` (Huber loss) or GTSAM; ≤ 60 parameters, a few ms.
5. Poses older than the window are **frozen**. Their tiles are final for Tier A.

**Failure modes and responses:**
- **Uniform canopy (booting–flowering) or standing water with sun glint** → registration fails. Fall back to the pose prior. The mosaic gets seams but stays usable; the share of `REG` flags is reported.
- **Turns:** captures with `TILT` are excluded from matching.

---

## 8. Orthoprojection and mosaicking (Tier A)

1. **Output grid:**
   - UTM zone of the field.
   - GSD = round-up(capture GSD) (5 cm at 60 m).
   - **Tiles of 512 × 512 px** (25.6 m at 5 cm).
2. **Inverse projection** per capture: for each output pixel in the footprint, compute the ground point on the plane, project it into the image (pose + lens model), and sample the 4 reflectance bands bilinearly on the GPU. Footprint at 60 m ≈ 1440 × 1080 output px, about 1.5 M pixels × 4 bands.
3. **Per-pixel score**, Agisoft "Mosaic" blending emulation:
   ```
   s = cos(view angle) · w_flags
   ```
   - w_flags = 1, or 0.3 if the capture has any flag.
   - Keep the band values with the highest s: nearest-to-nadir wins. This minimises relief displacement and view-angle effects.
   - **Optional seam feathering:** 8 px linear blend where the winning capture changes.
4. **Memory:** one float16 tile set per band, plus score and capture-ID layers. A 5-acre field at 5 cm ≈ 8 M px → < 200 MB. A 100 ha field at 5 cm ≈ 400 M px → page tiles to NVMe with an LRU cache of about 2 GB.
5. **Tile finalisation:** a tile no longer reachable by the sliding window is written to disk as a COG tile (Deflate, predictor 2) and gets a preview update.

---

## 9. Products (Tier A, per finalised tile)

| Product | Formula | Note |
|---|---|---|
| Reflectance G, R, RE, NIR | §5.6 | uint16 × 32768 (Agisoft-compatible) or float32 |
| NDVI | (NIR − R)/(NIR + R) | |
| NDRE | (NIR − RE)/(NIR + RE) | Better than NDVI for dense canopy |
| GNDVI | (NIR − G)/(NIR + G) | |
| OSAVI | 1.16·(NIR − R)/(NIR + R + 0.16) | Less sensitive to soil and water background (nursery stage) |
| CIre | NIR/RE − 1 | Chlorophyll proxy |
| Quality mask | Flags, coverage, view angle | Required for any downstream analysis |

- **No-data** where the denominator < 0.02 or any band is masked.
- **End-of-flight assembly:** a GDAL VRT over the tiles, then `gdal_translate` to a single COG with overviews. Naming follows the Agisoft deliverable convention:
  `Reflectance_Orthomosaic_<GSD>_<EPSG>.tif`, `NDVI_<GSD>_<EPSG>.tif`, …

---

## 10. Real-time software architecture on the Orin

### 10.1 Processes (Python 3.10, JetPack 6)
| Process | Role | Key libraries |
|---|---|---|
| P0 `pose_logger` | MAVLink at ≥ 50 Hz → ring buffer + NVMe log; publishes the time offset | `pymavlink` |
| P1 `acquirer` | Triggers Sequoia at 1 Hz, pulls the 4 TIFFs (+ RGB if bandwidth allows), writes raw files, pushes the capture ID | `requests`/PTP **[verify]** |
| P2 `radiometry` | XMP parse, §5 chain on GPU | `cupy` or `torch`, `numpy` |
| P3 `geometry` | Undistort + band transform (one remap) | OpenCV (CUDA if available) |
| P4 `pose` | Features, matching, sliding-window solve | OpenCV, `scipy` |
| P5 `mosaic` | Inverse projection, scoring, tile store | `cupy`, `numpy` |
| P6 `products` | Indices, COG writing, preview | `rasterio`/GDAL |
| P7 `monitor` | Watchdog, latency budget, health → telemetry | — |

**Inter-process communication:** shared-memory ring buffers for images, lightweight queues for metadata. Stages run as a **pipeline**: capture i+1 is acquired while capture i is processed.

### 10.2 Latency budget per capture (targets; measured in Phase 3)
| Stage | Target (Orin NX) |
|---|---|
| Transfer of 4 TIFFs (~10 MB) | ≤ 300 ms **[verify; biggest unknown]** |
| XMP parse + radiometry | ≤ 40 ms |
| Undistort + band transform | ≤ 20 ms |
| Features + matching | ≤ 150 ms |
| Sliding-window solve | ≤ 30 ms |
| Ortho + blend | ≤ 50 ms |
| Products + tile I/O | ≤ 50 ms (asynchronous) |
| **Total critical path** | **≤ 650 ms (p95 < 800 ms)** |

### 10.3 Degradation ladder (if p95 latency > 900 ms)
Steps are applied in order until the budget is met:
1. Skip RGB transfer.
2. Reduce matching to the previous capture only.
3. Skip visual refinement (pose prior only).
4. Mosaic at 2× GSD.
5. Queue and catch up during turns.

Raw recording is never dropped.

---

## 11. Tier B: on board after landing

1. Post-flight panel → K_k(t) interpolation (§5.6); reprocess reflectance.
2. Write calibrated per-band reflectance TIFFs with refined geotags.
3. Run **OpenDroneMap** (arm64 container **[verify availability]**) on those reflectance images:
   - `--radiometric-calibration none` (already calibrated).
   - `--dsm --dtm`.
   - `--gcp gcp_list.txt` if GCPs were surveyed.
   - Orthophoto resolution = GSD.
4. **Deliverables:** DSM, DTM, reflectance orthomosaic, index maps, RGB orthomosaic.
5. **Quality report:**
   - Tier A vs Tier B reflectance difference per band.
   - Panel residuals.
   - Seam misalignment.
   - `REG` and `TILT` rates.
   - Latency statistics.

Expected runtime for 300–600 captures at 1.2 MP on an Orin NX: tens of minutes **[measure]**.

---

## 12. Validation and acceptance criteria

| Area | Test | Pass |
|---|---|---|
| Vignetting | Flat-field capture | Corner/centre within ±2% |
| Radiometry | 2–3 grey reference tarps of known reflectance in the field | Reflectance RMSE ≤ 0.02 per band |
| Index agreement | Same flight processed in Pix4D/Agisoft (reference) | NDVI MAE ≤ 0.03; NDRE MAE ≤ 0.03 |
| Band alignment | Edge overlay NIR vs R | ≤ 0.5 px |
| Relative geometry | Seam misalignment in mosaic | ≤ 2 px (Tier A), ≤ 1 px (Tier B) |
| Absolute geometry | Checkpoints not used in processing | Tier A ≤ 5 m; Tier B with GCPs ≤ 0.10 m |
| Real time | 20-min flight, 1 Hz | 0 dropped captures; p95 latency < 800 ms |
| Robustness | Booting-stage canopy, partial cloud | `REG` < 20%; flight still completes with a mosaic |

---

## 13. Implementation phases

| Phase | Where | Deliverable | Exit criterion |
|---|---|---|---|
| **0 Hardware spike** | Bench | Orin triggers the Sequoia at 1 Hz and pulls all TIFFs for 20 min; MAVLink logging; GNSS EMI check | 1200/1200 captures, transfer p95 ≤ 300 ms, C/N0 drop < 3 dB |
| 1 Radiometry | Colab, our Sequoia captures + panel | `radiometry/` module + notebook | Flat-field ±2%; panel CV < 3%; reproduces Pix4D reflectance on the same images within 0.02 |
| 2 Geometry + mosaic replay | Colab, a recorded flight replayed at 1 Hz | `geometry/`, `pose/`, `mosaic/` | Seams ≤ 2 px; NDVI vs Pix4D/ODM MAE ≤ 0.03 |
| 3 Orin real-time replay | Orin bench | Full pipeline, timing logs | p95 < 800 ms, no backlog over 1200 captures |
| 4 Ground and tethered tests | Field | Thermal, EMI, power | No throttling at 36 °C ambient |
| 5 Flight | Field | Tier A live products | §12 criteria |
| 6 Tier B | Orin after landing | ODM integration + quality report | §12 Tier B criteria |

---

## 14. Repository layout (planned)

```
mxortho/
  config/            camera.yaml (intrinsics, band transforms, boresight, Δt), flight.yaml, panel.yaml
  io/                sequoia.py (trigger/transfer), mavlink.py, xmp.py, rawstore.py
  radiometry/        sensor_model.py, vignetting.py, sunshine.py, panel.py, reflectance.py
  geometry/          lens.py, band_align.py, pose_prior.py, sun.py
  pose/              features.py, window_ba.py
  mosaic/            grid.py, project.py, blend.py, tilestore.py
  products/          indices.py, cog.py, preview.py
  runtime/           pipeline.py, shm.py, monitor.py
  postflight/        drift.py, odm.py, report.py
  tools/             calib_bands.py, calib_boresight.py, replay.py
notebooks/           01_radiometry.ipynb, 02_mosaic_replay.ipynb
tests/               unit tests per module + golden-capture regression
```

---

## 15. Inputs needed before coding

1. **Sequoia or Sequoia+?** Firmware version.
2. **Panel reflectance values** per band (label or certificate).
3. **Sample data:**
   - About 20 raw captures over the field.
   - 3 panel captures.
   - 1 flat-field capture.
   - Ideally one full flight already processed in Pix4D/Agisoft, as the reference.
4. **Orin model** (NX 16 GB or AGX), **flight controller** (PX4 or ArduPilot), and the airframe's spare payload.
5. **How the Sequoia connects to the Orin today** (USB, Wi-Fi), and whether a trigger from the Orin already works.
6. Planned **altitude and speed** (default 60 m, 8 m/s).
