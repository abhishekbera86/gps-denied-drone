#!/usr/bin/env python3
"""Generate vio_ground_mosaic.png — the vio_test.sdf ground texture.

A seeded (reproducible), non-periodic, multi-scale colored mosaic: nested
64px / 16px / 4px cells so a camera sees sharp FAST corners at ANY viewing
distance — from a 30deg-down-tilted D435i 0.2m off the ground during
OpenVINS init (where solid-color 0.25m tiles gave only ~5-7 corners at
fast_threshold=30, blocking initialization entirely — see the world file's
comment) out to 2m-altitude cruise. Non-periodic by construction (seeded
RNG, not a repeating pattern) so there is no checkerboard-style aliasing
for the KLT tracker (resource/Vio_Drift_analysis.txt item B).

Run inside the ros2-autonomy container (has cv2+numpy):
  python3 generate_vio_ground_mosaic.py [out.png]
The generated PNG is checked in; this script exists so it can be
regenerated/retuned reproducibly, not because the build runs it.
"""
import sys

import cv2
import numpy as np

SIZE = 2048          # px, mapped over the 8x8m pad => ~4mm/px
SEED = 20260710      # today's date — fixed for reproducibility

PALETTE = np.array([
    (31, 31, 31), (224, 224, 209), (140, 77, 51), (77, 115, 71),
    (191, 173, 89), (51, 56, 140), (166, 166, 166), (102, 38, 38),
    (89, 140, 166), (204, 122, 61), (61, 89, 46), (150, 150, 90),
], dtype=np.uint8)  # BGR-ish; exact hues don't matter, contrast does


def mosaic(rng, cells):
    idx = rng.integers(0, len(PALETTE), size=(cells, cells))
    img = PALETTE[idx]
    return cv2.resize(img, (SIZE, SIZE), interpolation=cv2.INTER_NEAREST)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'vio_ground_mosaic.png'
    rng = np.random.default_rng(SEED)
    coarse = mosaic(rng, SIZE // 64)   # 64px cells (~26cm on ground)
    mid = mosaic(rng, SIZE // 16)      # 16px cells (~6.5cm)
    fine = mosaic(rng, SIZE // 4)      # 4px cells (~1.6cm)
    # Blend: coarse base, mid patches on ~35% of area, fine speckle on ~12%
    mid_mask = (cv2.resize(
        (rng.random((SIZE // 64, SIZE // 64)) < 0.35).astype(np.uint8),
        (SIZE, SIZE), interpolation=cv2.INTER_NEAREST) > 0)
    fine_mask = (cv2.resize(
        (rng.random((SIZE // 16, SIZE // 16)) < 0.12).astype(np.uint8),
        (SIZE, SIZE), interpolation=cv2.INTER_NEAREST) > 0)
    img = coarse.copy()
    img[mid_mask] = mid[mid_mask]
    img[fine_mask] = fine[fine_mask]
    cv2.imwrite(out, img)
    print('wrote', out, img.shape)


if __name__ == '__main__':
    main()
