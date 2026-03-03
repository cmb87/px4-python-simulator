LATEX ?= pdflatex
DOC = architecture

.PHONY: all clean

all:
	$(LATEX) -output-directory . $(DOC).tex
	$(LATEX) -output-directory . $(DOC).tex

clean:
	rm -f $(DOC).aux $(DOC).log $(DOC).out $(DOC).toc $(DOC).pdf
