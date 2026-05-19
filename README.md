# a750-control

Python bindings for the `a750_control` robotic arm library.

## Development install

Create and activate a Python 3.12 virtual environment:

```bash
cd /home/alex/a750_control
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
```

Install the package in editable mode:

```bash
pip install -e . -Cbuild-dir=build/python -Ccmake.args="-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
```

## Build a PyPI wheel

Linux binary wheels uploaded to PyPI must use a portable platform tag such as
`manylinux`, not the local `linux_x86_64` tag produced by `python -m build`.
Use `cibuildwheel` to build inside a manylinux Docker container and repair the
wheel with `auditwheel`.

Install release tooling:

```bash
python -m pip install -U build cibuildwheel twine
```

Build the Python 3.12 manylinux wheel:

```bash
CIBW_BUILD=cp312-manylinux_x86_64 python -m cibuildwheel --platform linux --output-dir wheelhouse
```

The output should look like:

```text
wheelhouse/a750_control-0.1.0-cp312-cp312-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl
```

`pyproject.toml` passes `-DBUILD_TESTING=OFF` to CMake for package builds so
dependency test targets are not built as part of the wheel.

## Check and upload

Check the built wheel:

```bash
python -m twine check wheelhouse/*.whl
```

Upload to TestPyPI first:

```bash
python -m twine upload --repository testpypi wheelhouse/*.whl
```

Install from TestPyPI in a clean environment:

```bash
python -m pip install --index-url https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple/ a750-control
python -c "import a750_control; print(a750_control.Robot)"
```

If the TestPyPI package works, upload to PyPI:

```bash
python -m twine upload wheelhouse/*.whl
```

For authentication, use a PyPI API token with username `__token__`, or configure
PyPI Trusted Publishing for the GitHub repository.
