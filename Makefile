# Makefile for mcnp2cad
#
# Needs to be called while setting the CGM_BASE_DIR variable.
# CGM_BASE_DIR should be the directory in which there is an
# 'include' and 'lib' directory with the CGM include files and
# libraries.
#
# prompt%> make CGM_BASE_DIR=/path/to/installed/cgm
#
# CGM_BASE_DIR = /path/to/installed/cgm
# ARMADILLO_BASE_DIR = /path/to/installed/armadillo

-include ${CGM_BASE_DIR}/lib/iGeom-Defs.inc


CXXSOURCES = mcnp2cad.cpp MCNPInput.cpp volumes.cpp geometry.cpp ProgOptions.cpp
CXXOBJS = mcnp2cad.o MCNPInput.o volumes.o geometry.o ProgOptions.o

# Remove HAVE_IGEOM_CONE from the next line if using old iGeom implementation
CXXFLAGS = -g -std=c++11 -Wall -Wextra -DUSING_CGMA -DHAVE_IGEOM_CONE -I${ARMADILLO_BASE_DIR}/include


LDFLAGS = ${IGEOM_LIBS} 
LDFLAGS += -L${ARMADILLO_BASE_DIR}/lib -larmadillo

mcnp2cad: ${CXXOBJS} Makefile
	${CXX} ${CXXFLAGS} -o $@ ${CXXOBJS} ${LDFLAGS}
# The following may be more convenient than the above on Linux
#	libtool --mode=link ${CXX} ${CXXFLAGS} -o $@ ${CXXOBJS}  ${LDFLAGS} 


geometry.o: geometry.cpp geometry.hpp dataref.hpp
volumes.o: volumes.cpp volumes.hpp geometry.hpp MCNPInput.hpp
MCNPInput.o: MCNPInput.cpp MCNPInput.hpp geometry.hpp dataref.hpp options.hpp 
mcnp2cad.o: mcnp2cad.cpp MCNPInput.hpp geometry.hpp dataref.hpp \
            options.hpp volumes.hpp ProgOptions.hpp version.hpp
ProgOptions.o: ProgOptions.cpp ProgOptions.hpp

.cpp.o:
	${CXX} ${CXXFLAGS} ${IGEOM_CPPFLAGS} -o $@ -c $<

clean:
	rm -rf mcnp2cad *.o

#
# Makefile for Sphinx documentation
#

# The included file 'gh-project.mk' should define the following:
# GH_SOURCE_DIR = top-level directory of all the ReST source files
# GH_SOURCE_BRANCH = repository branch that contains the source
# GH_PUBLISH_BRANCH = repository branch that contains the rendered HTML
# GH_UPSTREAM_REPO = repository that contains the rendered HTML
include gh-project.mk

# You can set these variables from the command line.
SPHINXOPTS    =
SPHINXBUILD   = sphinx-build
PAPER         =
GHBUILDDIR      = ./gh-build

# Internal variables.
PAPEROPT_a4     = -D latex_paper_size=a4
PAPEROPT_letter = -D latex_paper_size=letter
ALLSPHINXOPTS   = -d $(GHBUILDDIR)/doctrees $(PAPEROPT_$(PAPER)) $(SPHINXOPTS) $(GH_SOURCE_DIR)
# the i18n builder cannot share the environment and doctrees with the others
I18NSPHINXOPTS  = $(PAPEROPT_$(PAPER)) $(SPHINXOPTS) $(GH_SOURCE_DIR)

.PHONY: help clean html dirhtml singlehtml pickle json htmlhelp qthelp devhelp epub latex latexpdf text man changes linkcheck doctest gettext

help:
	@echo "Please use \`make <target>' where <target> is one of"
	@echo "  gh-preview to build HTML in directory $GHBUILDDIR for testing"
	@echo "  gh-revert  to cleanup HTML build in directory $GHBUILDDIR after testing"
	@echo "  gh-publish final build and push from source branch to master branch"
	@echo "  html       to make standalone HTML files"
	@echo "  dirhtml    to make HTML files named index.html in directories"
	@echo "  singlehtml to make a single large HTML file"
	@echo "  pickle     to make pickle files"
	@echo "  json       to make JSON files"
	@echo "  htmlhelp   to make HTML files and a HTML help project"
	@echo "  qthelp     to make HTML files and a qthelp project"
	@echo "  devhelp    to make HTML files and a Devhelp project"
	@echo "  epub       to make an epub"
	@echo "  latex      to make LaTeX files, you can set PAPER=a4 or PAPER=letter"
	@echo "  latexpdf   to make LaTeX files and run them through pdflatex"
	@echo "  text       to make text files"
	@echo "  man        to make manual pages"
	@echo "  texinfo    to make Texinfo files"
	@echo "  info       to make Texinfo files and run them through makeinfo"
	@echo "  gettext    to make PO message catalogs"
	@echo "  changes    to make an overview of all changed/added/deprecated items"
	@echo "  linkcheck  to check all external links for integrity"
	@echo "  doctest    to run all doctests embedded in the documentation (if enabled)"

gh-clean gh-revert:
	-rm -rf $(GHBUILDDIR)

gh-preview html:
	$(SPHINXBUILD) -b html $(ALLSPHINXOPTS) $(GHBUILDDIR)
	@echo
	@echo "Build finished. The HTML pages are in $(GHBUILDDIR)."

gh-publish:
	git checkout $(GH_PUBLISH_BRANCH)
	git checkout $(GH_SOURCE_BRANCH) -- $(GH_SOURCE_DIR)
	git reset HEAD
	make clean
	make html
	rsync -a $(GHBUILDDIR)/* .
	rsync -a $(GHBUILDDIR)/.* .
	git add `(cd $(GHBUILDDIR); find . -type f; cd ..)`
	rm -rf $(GH_SOURCE_DIR) $(GHBUILDDIR)
	git commit -m "Generated $(GH_PUBLISH_BRANCH) for `git log $(GH_SOURCE_BRANCH) -1 --pretty=short --abbrev-commit`" && git push $(GH_UPSTREAM_REPO) $(GH_PUBLISH_BRANCH)
	git checkout $(GH_SOURCE_BRANCH)

htmlclean cleanhtml: clean html

dirhtml:
	$(SPHINXBUILD) -b dirhtml $(ALLSPHINXOPTS) $(GHBUILDDIR)/dirhtml
	@echo
	@echo "Build finished. The HTML pages are in $(GHBUILDDIR)/dirhtml."

singlehtml:
	$(SPHINXBUILD) -b singlehtml $(ALLSPHINXOPTS) $(GHBUILDDIR)/singlehtml
	@echo
	@echo "Build finished. The HTML page is in $(GHBUILDDIR)/singlehtml."

pickle:
	$(SPHINXBUILD) -b pickle $(ALLSPHINXOPTS) $(GHBUILDDIR)/pickle
	@echo
	@echo "Build finished; now you can process the pickle files."

json:
	$(SPHINXBUILD) -b json $(ALLSPHINXOPTS) $(GHBUILDDIR)/json
	@echo
	@echo "Build finished; now you can process the JSON files."

htmlhelp:
	$(SPHINXBUILD) -b htmlhelp $(ALLSPHINXOPTS) $(GHBUILDDIR)/htmlhelp
	@echo
	@echo "Build finished; now you can run HTML Help Workshop with the" \
	      ".hhp project file in $(GHBUILDDIR)/htmlhelp."

qthelp:
	$(SPHINXBUILD) -b qthelp $(ALLSPHINXOPTS) $(GHBUILDDIR)/qthelp
	@echo
	@echo "Build finished; now you can run "qcollectiongenerator" with the" \
	      ".qhcp project file in $(GHBUILDDIR)/qthelp, like this:"
	@echo "# qcollectiongenerator $(GHBUILDDIR)/qthelp/DirectAccleratedGeometryMonteCarloToolkit.qhcp"
	@echo "To view the help file:"
	@echo "# assistant -collectionFile $(GHBUILDDIR)/qthelp/DirectAccleratedGeometryMonteCarloToolkit.qhc"

devhelp:
	$(SPHINXBUILD) -b devhelp $(ALLSPHINXOPTS) $(GHBUILDDIR)/devhelp
	@echo
	@echo "Build finished."
	@echo "To view the help file:"
	@echo "# mkdir -p $$HOME/.local/share/devhelp/DirectAccleratedGeometryMonteCarloToolkit"
	@echo "# ln -s $(GHBUILDDIR)/devhelp $$HOME/.local/share/devhelp/DirectAccleratedGeometryMonteCarloToolkit"
	@echo "# devhelp"

epub:
	$(SPHINXBUILD) -b epub $(ALLSPHINXOPTS) $(GHBUILDDIR)/epub
	@echo
	@echo "Build finished. The epub file is in $(GHBUILDDIR)/epub."

latex:
	$(SPHINXBUILD) -b latex $(ALLSPHINXOPTS) $(GHBUILDDIR)/latex
	@echo
	@echo "Build finished; the LaTeX files are in $(GHBUILDDIR)/latex."
	@echo "Run \`make' in that directory to run these through (pdf)latex" \
	      "(use \`make latexpdf' here to do that automatically)."

latexpdf:
	$(SPHINXBUILD) -b latex $(ALLSPHINXOPTS) $(GHBUILDDIR)/latex
	@echo "Running LaTeX files through pdflatex..."
	$(MAKE) -C $(GHBUILDDIR)/latex all-pdf
	@echo "pdflatex finished; the PDF files are in $(GHBUILDDIR)/latex."

text:
	$(SPHINXBUILD) -b text $(ALLSPHINXOPTS) $(GHBUILDDIR)/text
	@echo
	@echo "Build finished. The text files are in $(GHBUILDDIR)/text."

man:
	$(SPHINXBUILD) -b man $(ALLSPHINXOPTS) $(GHBUILDDIR)/man
	@echo
	@echo "Build finished. The manual pages are in $(GHBUILDDIR)/man."

texinfo:
	$(SPHINXBUILD) -b texinfo $(ALLSPHINXOPTS) $(GHBUILDDIR)/texinfo
	@echo
	@echo "Build finished. The Texinfo files are in $(GHBUILDDIR)/texinfo."
	@echo "Run \`make' in that directory to run these through makeinfo" \
	      "(use \`make info' here to do that automatically)."

info:
	$(SPHINXBUILD) -b texinfo $(ALLSPHINXOPTS) $(GHBUILDDIR)/texinfo
	@echo "Running Texinfo files through makeinfo..."
	make -C $(GHBUILDDIR)/texinfo info
	@echo "makeinfo finished; the Info files are in $(GHBUILDDIR)/texinfo."

gettext:
	$(SPHINXBUILD) -b gettext $(I18NSPHINXOPTS) $(GHBUILDDIR)/locale
	@echo
	@echo "Build finished. The message catalogs are in $(GHBUILDDIR)/locale."

changes:
	$(SPHINXBUILD) -b changes $(ALLSPHINXOPTS) $(GHBUILDDIR)/changes
	@echo
	@echo "The overview file is in $(GHBUILDDIR)/changes."

linkcheck:
	$(SPHINXBUILD) -b linkcheck $(ALLSPHINXOPTS) $(GHBUILDDIR)/linkcheck
	@echo
	@echo "Link check complete; look for any errors in the above output " \
	      "or in $(GHBUILDDIR)/linkcheck/output.txt."

doctest:
	$(SPHINXBUILD) -b doctest $(ALLSPHINXOPTS) $(GHBUILDDIR)/doctest
	@echo "Testing of doctests in the sources finished, look at the " \
	      "results in $(GHBUILDDIR)/doctest/output.txt."
