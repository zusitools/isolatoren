#!/usr/bin/env python3
"""
Sucht in den angegebenen LS3-Dateien rekursiv nach falsch herum eingebauten Isolatoren
und schreibt die korrigierten Dateien in eine Verzeichnishierarchie unterhalb des AKTUELLEN Verzeichnisses.
"""

import sys
import os
import math

from lxml import etree
import numpy as np
from scipy.spatial.transform import Rotation as R

from zusicommon import zusicommon

DATAPATH = zusicommon.get_zusi_data_path()
print(f"Datenverzeichnis eigene Daten: {DATAPATH}")
DATAPATH_OFFICIAL = zusicommon.get_zusi_data_path_official()
print(f"Datenverzeichnis offizielle Daten: {DATAPATH_OFFICIAL}")
PROCESSED_FILES = set()
DRY_RUN = False

UP_VECTORS = {
    r"Catenary\Deutschland\Einzelteile_Re75-200\Isolator.lod.ls3".lower(): np.array(
        [0, -1, 0, 1]
    ),
    r"Catenary\Deutschland\Einzelteile_Re75-200\IsolatorTragseil.lod.ls3".lower(): np.array(
        [0, -1, 0, 1]
    ),
    # r"Catenary\Deutschland\Turmmaste\Anbauteile_bis14m\Isolatoren\Isolator.lod.ls3".lower(): np.array([1, 0, 0, 1]),  # würde andere Verschiebung brauchen
    # r"Catenary\Deutschland\Turmmaste\Anbauteile_bis14m\Isolatoren\Isolator_Doppelklemme.lod.ls3".lower(): np.array([1, 0, 0, 1]),  # nicht symmetrisch
}


def transform_from_p_phi(p, phi):
    """
    Create a 4x4 homogeneous transform from:
      p   = (tx, ty, tz)
      phi = intrinsic ZYX Euler angles (rx, ry, rz)
    """
    rx, ry, rz = phi

    # SciPy uses intrinsic rotations for uppercase axis names
    rot = R.from_euler("ZYX", [rz, ry, rx])
    Rmat = rot.as_matrix()

    T = np.eye(4)
    T[:3, :3] = Rmat
    T[:3, 3] = p
    return T


def p_phi_from_transform(T):
    """
    Extract:
      p   = (tx, ty, tz)
      phi = intrinsic ZYX Euler angles (rx, ry, rz)
    from a 4x4 homogeneous transform T.
    """
    # Translation
    p = T[:3, 3].copy()

    # Rotation matrix
    Rmat = T[:3, :3]

    # Recover intrinsic ZYX Euler angles
    # SciPy returns them in the order specified: [rz, ry, rx]
    rot = R.from_matrix(Rmat)
    rz, ry, rx = rot.as_euler("ZYX")

    phi = np.array([rx, ry, rz])
    return p, phi


def parse_vec(node):
    if node is None:
        return np.zeros(3)
    return np.array(
        [
            float(node.get("X", "0")),
            float(node.get("Y", "0")),
            float(node.get("Z", "0")),
        ]
    )


def angle(v1, v2):
    return np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))


def process_verknuepfte(node, base_path, parent_T):
    p = parse_vec(node.find("p"))
    phi = parse_vec(node.find("phi"))

    local_T = transform_from_p_phi(p, phi)
    world_T = parent_T @ local_T
    world_T_only_rot = np.eye(4)
    world_T_only_rot[:3, :3] = world_T[:3, :3]

    datei = node.find("Datei")
    if datei is None:
        return False
    filename = datei.get("Dateiname")

    changed = False
    up_vector = UP_VECTORS.get(filename.lower())
    if up_vector is not None:
        transformed = world_T_only_rot @ up_vector
        transformed /= transformed[3]
        phi = angle(transformed[:3], np.array([0, 0, 1]))
        print(f"{filename}, phi={math.degrees(phi)} deg")
        if phi < 1.3:
            # Turn around: Move by 0.60230 units along local y and rotate 180deg around local z.
            translation = np.eye(4)
            translation[:3, 3] = [0.0, 0.60230, 0.0]
            rotation = np.eye(4)
            rotation[:3, :3] = R.from_euler("z", 180, degrees=True).as_matrix()

            # Local-space transform: first move, then rotate
            local_T_new = local_T @ rotation @ translation

            # Sanity check
            world_T_new = parent_T @ local_T_new
            world_T_new_only_rot = np.eye(4)
            world_T_new_only_rot[:3, :3] = world_T_new[:3, :3]
            transformed_new = world_T_new_only_rot @ up_vector
            transformed_new /= transformed_new[3]
            phi = angle(transformed_new[:3], np.array([0, 0, 1]))
            print(f" -> phi={math.degrees(phi)} deg")
            assert phi >= 1.3

            p2, phi2 = p_phi_from_transform(local_T_new)
            node.find("p").attrib["X"] = str(p2[0])
            node.find("p").attrib["Y"] = str(p2[1])
            node.find("p").attrib["Z"] = str(p2[2])
            node.find("phi").attrib["X"] = str(phi2[0])
            node.find("phi").attrib["Y"] = str(phi2[1])
            node.find("phi").attrib["Z"] = str(phi2[2])

            # zum Testen
            # datei.set(
            #    "Dateiname",
            #    filename.replace("Isolator", "Isolator_rot"),
            # )
            changed = True

    # Recurse into nested file
    nested_path = zusicommon.resolve_file_path(
        filename, base_path, DATAPATH, DATAPATH_OFFICIAL
    )
    assert os.path.isfile(nested_path), f"{base_path}->{filename} not resolved"
    process_file(nested_path, world_T)

    return changed


def process_file(xml_path, parent_T=np.eye(4)):
    """Parse and modify a Zusi XML file."""
    if xml_path in PROCESSED_FILES:
        return
    PROCESSED_FILES.add(xml_path)

    parser = etree.XMLParser(remove_blank_text=False, strip_cdata=False)
    tree = etree.parse(xml_path, parser)
    root = tree.getroot()

    base_path = os.path.dirname(xml_path)

    changed = False
    for v in root.findall(".//Verknuepfte"):
        if process_verknuepfte(v, base_path, parent_T):
            changed = True

    # Write back, preserving formatting
    if changed:
        dest_path = None
        try:
            if os.path.normpath(
                os.path.commonpath([xml_path, DATAPATH])
            ) == os.path.normpath(DATAPATH):
                dest_path = os.path.relpath(xml_path, DATAPATH)
        except ValueError:
            pass
        try:
            if not dest_path and os.path.normpath(
                os.path.commonpath([xml_path, DATAPATH_OFFICIAL])
            ) == os.path.normpath(DATAPATH_OFFICIAL):
                dest_path = os.path.relpath(xml_path, DATAPATH_OFFICIAL)
        except ValueError:
            pass

        assert (
            dest_path is not None
        ), f"{xml_path} scheint nicht im eigenen oder offiziellen Verzeichnis zu liegen"

        if DRY_RUN:
            print(f"would write {dest_path}")
        else:
            print(f"write {dest_path}")
            os.makedirs(os.path.dirname(dest_path), exist_ok=True)
            tree.write(
                dest_path, encoding="UTF-8", xml_declaration=True, pretty_print=False
            )


if __name__ == "__main__":
    for f in sys.argv[1:]:
        process_file(os.path.abspath(f))
