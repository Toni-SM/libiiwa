import os
import sys

# libiiwa library
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "libiiwa")))
print("[DOCS] libiiwa library path: {}".format(sys.path[0]))

import libiiwa

# -- Project information

project = 'libiiwa'
copyright = '2022, Toni-SM'
author = 'Toni-SM'

release = libiiwa.__version__
version = libiiwa.__version__

master_doc = 'index'

# -- General configuration

extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    'sphinx_tabs.tabs'
]

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}
intersphinx_disabled_domains = ['std']

templates_path = ['_templates']

rst_prolog = """
 .. include:: <s5defs.txt>

"""

# -- Options for HTML output

html_theme = 'sphinx_rtd_theme'

html_logo = '_static/data/libiiwa.png'

html_static_path = ['_static']

html_css_files = ['css/s5defs-roles.css',
                  'css/libiiwa.css']

# -- Options for EPUB output
epub_show_urls = 'footnote'
