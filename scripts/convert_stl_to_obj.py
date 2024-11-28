#!/usr/bin/env python3

import os
import trimesh
import argparse


def convert_stl_to_obj(directory):
    """
    Finds all STL files in the given directory and subdirectories, and converts each to an OBJ file.

    Args:
        directory (str): The path to the directory to search.
    """
    for root, _, files in os.walk(directory):
        for file in files:
            if file.lower().endswith(".stl"):
                stl_path = os.path.join(root, file)
                obj_path = os.path.splitext(stl_path)[0] + ".obj"

                try:
                    # Load the STL file
                    mesh = trimesh.load(stl_path)

                    # Export the mesh as OBJ
                    mesh.export(obj_path)

                    print(f"Converted: {stl_path} -> {obj_path}")
                except Exception as e:
                    print(f"Failed to convert {stl_path}: {e}")


def main():
    parser = argparse.ArgumentParser(description="Convert STL files to OBJ format.")
    parser.add_argument(
        "directory", type=str, help="The directory to search for STL files."
    )
    args = parser.parse_args()

    if os.path.isdir(args.directory):
        convert_stl_to_obj(args.directory)
    else:
        print("The provided path is not a directory. Please check and try again.")


if __name__ == "__main__":
    main()
