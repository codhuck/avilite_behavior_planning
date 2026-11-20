"""Load this checkout for offline command-line tools."""

# AVLite scans tools during discovery; importing this file must have no effects.
if not __name__.startswith("avlite.plugins."):
    import importlib.util
    import sys
    from pathlib import Path

    def load_plugin():
        name = "avlite_coordination_demo"
        root = Path(__file__).resolve().parents[1]
        if name in sys.modules:
            return sys.modules[name]
        spec = importlib.util.spec_from_file_location(
            name, root / "__init__.py", submodule_search_locations=[str(root)]
        )
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        try:
            spec.loader.exec_module(module)
        except Exception:
            sys.modules.pop(name, None)
            raise
        return module
