"""Keep pip's charset-normalizer modules coherent before Kit adds its bundles.

Isaac 6.1's pip environment pins 3.3.2 while omni.services.pip_archive bundles
3.5.1. Loading parts of both gives a Cython ABI error during SciPy startup.
Preloading the complete pip package avoids modifying either installed package.
"""
import charset_normalizer  # noqa: F401
from isaacsim import main

if __name__ == "__main__":
    main()
