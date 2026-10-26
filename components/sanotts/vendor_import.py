"""Compatibility entry point for the canonical optional archive importer."""
from pathlib import Path
import runpy

if __name__ == "__main__":
    runpy.run_path(str(Path(__file__).resolve().parents[2] / "tests/sanotts/import_pinned.py"),
                   run_name="__main__")
