import os
import argparse
from shutil import move

# we'll lazily import heavy libs only when needed

def compute_hash(path: str, method: str = "phash") -> int:
    """Compute perceptual hash of an image using Pillow + imagehash.

    ``method`` may be one of: "ahash", "phash", "dhash", "whash-haar".
    Returns the hash value as an integer.

    Internally ``imagehash`` produces an ``ImageHash`` object which does not
    implement ``__int__`` consistently across versions; older releases raise a
    ``TypeError`` when ``int()`` is called.  To be safe we convert via the
    hexadecimal string representation.
    """

    try:
        from PIL import Image
    except ImportError:  # pragma: no cover - explanation only
        raise RuntimeError(
            "Pillow is required; install with `pip install pillow imagehash`"
        )

    try:
        import imagehash
    except ImportError:  # pragma: no cover - explanation only
        raise RuntimeError(
            "imagehash is required; install with `pip install imagehash`"
        )

    im = Image.open(path)
    func = getattr(imagehash, method)
    h = func(im)

    # ``str(h)`` produces a hex string; convert that to an integer.  ``h.hash``
    # is a numpy array of bits if you prefer a different representation.
    return int(str(h), 16)


def remove_similar(folder: str, threshold: int = 5, method: str = "phash", move_to: str = None) -> int:
    """Remove (or move) images that are very similar.

    A perceptual hash is computed for each file; if the hamming distance
    between two hashes is <= ``threshold`` the later file is considered a
    duplicate of the earlier one and removed.

    Args:
        folder: directory to scan (non-recursively).
        threshold: maximum hamming distance for two images to be treated as
            duplicates.
        method: hashing algorithm used by ``imagehash``.
        move_to: optional directory where duplicates are relocated instead of
            being deleted.
    Returns:
        The number of images removed/moved.
    """

    if move_to:
        os.makedirs(move_to, exist_ok=True)

    hashes: dict[int, str] = {}
    removed = 0

    try:
        for fname in os.listdir(folder):
            path = os.path.join(folder, fname)
            if not os.path.isfile(path):
                continue
            if not fname.lower().endswith((
                ".png",
                ".jpg",
                ".jpeg",
                ".bmp",
                ".gif",
                ".tif",
                ".tiff",
            )):
                continue

            try:
                hval = compute_hash(path, method)
            except Exception as e:
                print(f"skipping {fname}: {e}")
                continue

            found = None
            for prev_hash, prev_name in hashes.items():
                # compute hamming distance between integers; ``bit_count`` is a
                # builtin and much faster than ``bin(...).count("1")``.
                dist = (hval ^ prev_hash).bit_count()
                if dist <= threshold:
                    found = prev_name
                    break
            if found is not None:
                if move_to:
                    dest = os.path.join(move_to, fname)
                    move(path, dest)
                else:
                    os.remove(path)
                removed += 1
            else:
                hashes[hval] = fname
    except KeyboardInterrupt:
        print("\ninterrupted by user; stopping early")
    return removed


def main():
    parser = argparse.ArgumentParser(
        description="Remove very similar images from a directory using perceptual hashes."
    )
    parser.add_argument("folder", help="Directory containing images to scan")
    parser.add_argument(
        "--threshold",
        type=int,
        default=5,
        help="max hamming distance for images to be considered duplicates (default 5)",
    )
    parser.add_argument(
        "--method",
        choices=["ahash", "phash", "dhash", "whash-haar"],
        default="phash",
        help="hashing method used by imagehash",
    )
    parser.add_argument(
        "--move-to",
        help="optional directory to move duplicates instead of deleting",
    )
    args = parser.parse_args()

    count = remove_similar(args.folder, args.threshold, args.method, args.move_to)
    if args.move_to:
        print(f"Moved {count} similar file(s) to '{args.move_to}'")
    else:
        print(f"Removed {count} similar file(s) from '{args.folder}'")


if __name__ == "__main__":
    main()
