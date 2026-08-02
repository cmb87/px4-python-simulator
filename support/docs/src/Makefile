LATEX ?= pdflatex
DOC = architecture

.PHONY: all clean

all:
	mkdir -p ../build
	$(LATEX) -output-directory ../build $(DOC).tex
	$(LATEX) -output-directory ../build $(DOC).tex

clean:
	rm -f ../build/$(DOC).aux ../build/$(DOC).log ../build/$(DOC).out ../build/$(DOC).toc ../build/$(DOC).pdf
