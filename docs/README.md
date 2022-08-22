## Documentation

### Install Sphinx and Read the Docs Sphinx Theme

```bash
pip install sphinx
pip install sphinx_rtd_theme
pip install sphinx-autobuild
pip install sphinx-tabs==3.2.0
```

### Building the documentation

```bash
cd docs
make html
```

Building each time a file is changed:

```bash
cd docs
sphinx-autobuild ./source/ _build/html
```

## Useful links

- [Sphinx directives](https://www.sphinx-doc.org/en/master/usage/restructuredtext/directives.html)
- [Math support in Sphinx](https://www.sphinx-doc.org/en/1.0/ext/math.html)
