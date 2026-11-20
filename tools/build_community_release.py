"""Build the standalone plugin archive without core files or downloaded models."""

# AVLite 0.6.7 discovers all Python files recursively; CLI imports must be inert.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import hashlib
    import json
    import zipfile
    from pathlib import Path

    ROOT = Path(__file__).resolve().parents[1]
    EXCLUDED = {".git", "__pycache__", ".pytest_cache", ".ruff_cache", "build", "dist", "logs"}

    def build(output):
        output = Path(output).expanduser().resolve()
        if output.is_relative_to(ROOT):
            raise ValueError("Write releases outside the source plugin directory")
        files = sorted(
            p
            for p in ROOT.rglob("*")
            if p.is_file()
            and not p.is_symlink()
            and not any(x in EXCLUDED or x.endswith(".egg-info") for x in p.relative_to(ROOT).parts)
            and p.suffix not in {".pyc", ".onnx", ".whl"}
        )
        manifest = {
            p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in files
        }
        for required in (
            "__init__.py",
            "README.md",
            "LICENSE",
            "requirements.txt",
            "avlite-behavior-hdmap.yaml",
        ):
            if required not in manifest:
                raise FileNotFoundError(required)
        output.parent.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(output, "x", zipfile.ZIP_DEFLATED) as archive:
            for path in files:
                archive.write(path, "avlite-behavior-hdmap/" + path.relative_to(ROOT).as_posix())
            archive.writestr(
                "avlite-behavior-hdmap/SOURCE_MANIFEST.json", json.dumps(manifest, indent=2) + "\n"
            )
        with zipfile.ZipFile(output) as archive:
            assert archive.testzip() is None
            for name, digest in manifest.items():
                assert (
                    hashlib.sha256(archive.read("avlite-behavior-hdmap/" + name)).hexdigest()
                    == digest
                )
        return {
            "archive": str(output),
            "files": len(files),
            "bytes": output.stat().st_size,
            "sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
        }

    if __name__ == "__main__":
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("output", type=Path)
        print(json.dumps(build(parser.parse_args().output), indent=2))
