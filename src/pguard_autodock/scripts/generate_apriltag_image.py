#!/usr/bin/env python3

import argparse
import sys

import cv2
import numpy as np


def generate_tag(tag_id: int, size_px: int, border_fraction: float) -> np.ndarray:
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

    # Compat OpenCV >=4.7 (generateImageMarker) vs versions plus anciennes (drawMarker).
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(aruco_dict, tag_id, size_px)
    else:
        marker = np.zeros((size_px, size_px), dtype=np.uint8)
        cv2.aruco.drawMarker(aruco_dict, tag_id, size_px, marker, 1)


    border_px = int(size_px * border_fraction)
    bordered = cv2.copyMakeBorder(
        marker, border_px, border_px, border_px, border_px,
        cv2.BORDER_CONSTANT, value=255,
    )
    return bordered


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--id", type=int, default=0, help="ID du tag (famille tag36h11)")
    parser.add_argument("--size", type=int, default=600, help="Taille du tag en pixels (hors marge)")
    parser.add_argument("--border", type=float, default=0.2, help="Marge blanche (fraction de la taille)")
    parser.add_argument("--output", type=str, required=True, help="Chemin du fichier PNG de sortie")
    args = parser.parse_args()

    img = generate_tag(args.id, args.size, args.border)
    ok = cv2.imwrite(args.output, img)
    if not ok:
        print(f"Echec de l'ecriture de {args.output}", file=sys.stderr)
        return 1

    print(f"Tag36h11 id={args.id} genere: {args.output} ({img.shape[1]}x{img.shape[0]} px)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
