### diverse Makefile ###

SHELL		= /bin/bash
CXX		= clang++
CXXFLAGS	= -Wall -Wextra -Werror -Wpedantic -std=c++11 \
		    -g -Qunused-arguments -fcolor-diagnostics \
		    -fdiagnostics-show-template-tree -march=native
LDFLAGS		= -lompl -lboost_graph

PROG		= diverse
EXEC		= build/bin/$(PROG)
SRCDIR		= src
INCDIR		= include
PCHINC		= $(INCDIR)/pch.h
DIRS		= build/bin build/$(SRCDIR)
INCS		= $(wildcard $(INCDIR)/*.h) $(wildcard $(INCDIR)/**/*.h)
CXXFLAGS	+= -I$(INCDIR)
PCH		= $(PCHINC).pch
SRCS		= $(shell echo $(wildcard $(SRCDIR)/*.cpp) | tr " " "\n" | sort | tr "\n" " ")
OBJS    	= $(subst $(SRCDIR)/,build/$(SRCDIR)/,$(SRCS:.cpp=.o))

all: $(PCH) $(INCS) $(SRCS) $(EXEC)

again:
	+@$(MAKE) clean
	+@$(MAKE) all

${EXEC}: $(OBJS) | dirs
	@echo -e "\033[1;34m[Linking ${EXEC}]\033[0m"
	${CXX} ${LDFLAGS} ${OBJS} -o $@

build/%.o: %.cpp $(PCH) $(INCS) | dirs
	@echo -e "\033[1;35m[Compiling $(subst $(SRCDIR)/,,$<)]\033[0m"
	${CXX} -c ${CXXFLAGS} -include ${PCHINC} $< -o $@

${PCH}: $(PCHINC)
	@echo -e "\033[1;36m[Precompiling headers]\033[0m"
	${CXX} -x c++-header ${CXXFLAGS} $< -o $@

.PHONY: dirs clean
.SILENT: dirs clean

dirs:
	for dir in ${DIRS}; do \
		mkdir -p $$dir; \
	done

clean:
	rm -f ${PCH} ${OBJS} ${EXEC}
	for dir in $(TOOLS); do \
		cd $$dir; \
		$(MAKE) clean; \
	done
