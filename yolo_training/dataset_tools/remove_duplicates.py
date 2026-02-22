import os
import hashlib
import argparse
from shutil import move


def remove_duplicates(folder: str, move_to: str = None) -> int:
    """Scan the given folder (non-recursively) for files and remove duplicates.

    Two files are considered duplicates if their contents are exactly the same
    (byte for byte).  The first occurrence of a hash is kept and later files
    with the same hash will be deleted or moved.

    Args:
        folder: path to the directory containing images (or any files).
        move_to: optional destination directory to which duplicates are moved
            instead of being deleted. If provided and does not exist it will be
            created.

    Returns:
        The number of duplicates removed (or moved).
    """

    if move_to:
        os.makedirs(move_to, exist_ok=True)

    seen: dict[str, str] = {}
    removed = 0

    for fname in os.listdir(folder):
        path = os.path.join(folder, fname)
        if not os.path.isfile(path):
            # skip subdirectories, symlinks, etc.
            continue

        # only operate on typical image extensions, but can process any file
        # if you prefer. adjust the tuple below as needed.
        if not fname.lower().endswith((".png", ".jpg", ".jpeg", ".bmp", ".gif", ".tif", ".tiff")):
            continue

        with open(path, "rb") as f:
            h = hashlib.md5(f.read()).hexdigest()
        if h in seen:
            if move_to:
                dest = os.path.join(move_to, fname)
                move(path, dest)
            else:
                os.remove(path)
            removed += 1
        else:
            seen[h] = fname

    return removed


def main():
    parser = argparse.ArgumentParser(
        description="Remove duplicate images from a directory by content hash."
    )
    parser.add_argument(
        "folder",
        help="Directory containing images to scan for duplicates",
    )
    parser.add_argument(
        "--move-to",
        help="Optional directory to move duplicates into instead of deleting.",
    )
    args = parser.parse_args()

    count = remove_duplicates(args.folder, args.move_to)
    if args.move_to:
        print(f"Moved {count} duplicate file(s) to '{args.move_to}'")
    else:
        print(f"Removed {count} duplicate file(s) from '{args.folder}'")


if __name__ == "__main__":
    main()