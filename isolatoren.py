#!/usr/bin/env python3
"""
Sucht in den angegebenen LS3-Dateien rekursiv nach falsch herum eingebauten Isolatoren
und schreibt die korrigierten Dateien in eine Verzeichnishierarchie unterhalb des AKTUELLEN Verzeichnisses.
"""

import sys
import os
import math
import contextlib
import traceback
import tkinter
from tkinter import filedialog, messagebox, scrolledtext, ttk

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


def set_attrib_float(node, name, value, digits):
    value = round(value, digits)
    if value == 0:
        try:
            del node.attrib[name]
        except KeyError:
            pass
    else:
        node.attrib[name] = str(value).rstrip(".0")


def process_verknuepfte(node, dirname_rel_to_data_dir, parent_T):
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
            set_attrib_float(node.find("p"), "X", p2[0], 4)
            set_attrib_float(node.find("p"), "Y", p2[1], 4)
            set_attrib_float(node.find("p"), "Z", p2[2], 4)
            set_attrib_float(node.find("phi"), "X", phi2[0], 6)
            set_attrib_float(node.find("phi"), "Y", phi2[1], 6)
            set_attrib_float(node.find("phi"), "Z", phi2[2], 6)

            # zum Testen
            # datei.set(
            #    "Dateiname",
            #    filename.replace("Isolator", "Isolator_rot"),
            # )
            changed = True

    if "\\" not in filename:
        process_file(dirname_rel_to_data_dir, filename, world_T)
    else:
        filename = filename.lstrip("\\").replace("\\", os.sep)
        process_file(*os.path.split(filename), world_T)

    return changed


def process_file(dirname_rel_to_data_dir, filename, parent_T=np.eye(4)):
    """Parse and modify a Zusi XML file."""

    xml_path_rel = os.path.join(dirname_rel_to_data_dir, filename)
    if xml_path_rel in PROCESSED_FILES:
        return
    PROCESSED_FILES.add(xml_path_rel)

    absolute_path = os.path.join(DATAPATH, xml_path_rel)
    if not os.path.isfile(absolute_path):
        absolute_path_try1 = absolute_path
        absolute_path = os.path.join(DATAPATH_OFFICIAL, xml_path_rel)
        assert os.path.isfile(
            absolute_path
        ), f"{xml_path_rel} not resolved: neither {absolute_path_try1} nor {absolute_path} exist"

    parser = etree.XMLParser(remove_blank_text=False, strip_cdata=False)
    tree = etree.parse(absolute_path, parser)
    root = tree.getroot()

    base_path = os.path.dirname(absolute_path)

    changed = False
    for v in root.findall(".//Verknuepfte"):
        if process_verknuepfte(v, dirname_rel_to_data_dir, parent_T):
            changed = True

    # Write back, preserving formatting
    if changed:
        dest_path = os.path.join(dirname_rel_to_data_dir, filename)
        if DRY_RUN:
            print(f"would write {dest_path}")
        else:
            print(f"write {dest_path}")
            os.makedirs(dirname_rel_to_data_dir, exist_ok=True)
            tree.write(
                dest_path, encoding="UTF-8", xml_declaration=True, pretty_print=False
            )


def process_input_file(filename):
    f = os.path.abspath(filename)
    rel_path = None
    try:
        if os.path.normpath(os.path.commonpath([f, DATAPATH])) == os.path.normpath(
            DATAPATH
        ):
            rel_path = os.path.relpath(f, DATAPATH)
    except ValueError:
        pass
    try:
        if not rel_path and os.path.normpath(
            os.path.commonpath([f, DATAPATH_OFFICIAL])
        ) == os.path.normpath(DATAPATH_OFFICIAL):
            rel_path = os.path.relpath(f, DATAPATH_OFFICIAL)
    except ValueError:
        pass

    assert (
        rel_path is not None
    ), f"{f} scheint nicht im eigenen oder offiziellen Verzeichnis zu liegen"
    process_file(*os.path.split(rel_path))


class TextOutput:
    def __init__(self, widget):
        self.widget = widget

    def write(self, text):
        if text:
            self.widget.insert(tkinter.END, text)
            self.widget.see(tkinter.END)
            self.widget.update_idletasks()

    def flush(self):
        pass


def gui():
    root = tkinter.Tk()
    root.title("Isolatoren korrigieren")
    root.minsize(620, 420)

    frame = ttk.Frame(root, padding=8)
    frame.pack(fill=tkinter.BOTH, expand=True)
    frame.columnconfigure(0, weight=1)
    frame.rowconfigure(1, weight=1)
    frame.rowconfigure(4, weight=1)

    ttk.Label(frame, text="LS3-Dateien (aus den Zusi-Datenverzeichnissen):").grid(
        row=0, column=0, columnspan=2, sticky=tkinter.W
    )
    files = tkinter.Listbox(frame, selectmode=tkinter.EXTENDED, height=4)
    files.grid(row=1, column=0, sticky="nsew")
    scrollbar = ttk.Scrollbar(frame, orient=tkinter.VERTICAL, command=files.yview)
    scrollbar.grid(row=1, column=1, sticky="ns")
    files.configure(yscrollcommand=scrollbar.set)

    def add_files():
        selected = filedialog.askopenfilenames(
            parent=root,
            title="LS3-Dateien auswählen",
            initialdir=DATAPATH,
            filetypes=[("LS3-Dateien", "*.ls3"), ("Alle Dateien", "*")],
        )
        existing = set(files.get(0, tkinter.END))
        for path in selected:
            if path not in existing:
                files.insert(tkinter.END, path)
                existing.add(path)

    def remove_files():
        for index in reversed(files.curselection()):
            files.delete(index)

    file_buttons = ttk.Frame(frame)
    file_buttons.grid(row=1, column=2, padx=(6, 0), sticky="n")
    ttk.Button(file_buttons, text="Dateien hinzufügen...", command=add_files).pack(
        fill=tkinter.X
    )
    ttk.Button(file_buttons, text="Auswahl entfernen", command=remove_files).pack(
        fill=tkinter.X, pady=(4, 0)
    )

    ttk.Label(
        frame,
        text=f"Korrigierte Dateien werden unterhalb des aktuellen Verzeichnisses "
        f"({os.getcwd()}) gespeichert.",
        wraplength=590,
    ).grid(row=2, column=0, columnspan=3, sticky="nw", pady=(6, 8))

    output = scrolledtext.ScrolledText(frame, height=10, state=tkinter.NORMAL)
    output.grid(row=4, column=0, columnspan=3, sticky="nsew")

    def run():
        paths = files.get(0, tkinter.END)
        if not paths:
            messagebox.showwarning(
                "Keine Dateien ausgewählt", "Bitte zuerst LS3-Dateien auswählen.", parent=root
            )
            return

        output.delete("1.0", tkinter.END)
        PROCESSED_FILES.clear()
        stream = TextOutput(output)
        with contextlib.redirect_stdout(stream), contextlib.redirect_stderr(stream):
            for path in paths:
                print(f"\nVerarbeite {path}")
                try:
                    process_input_file(path)
                except Exception:
                    traceback.print_exc()
            print("Verarbeitung abgeschlossen.")

    ttk.Button(frame, text="Verarbeitung starten", command=run).grid(
        row=3, column=0, columnspan=3, sticky="ew", pady=(0, 8)
    )
    root.mainloop()


if __name__ == "__main__":
    if len(sys.argv) == 1:
        gui()
    else:
        for f in sys.argv[1:]:
            process_input_file(f)
