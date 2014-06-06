### diverse Makefile ###

SHELL		= /bin/bash
CXX		= g++
CXXFLAGS	= -Wall -Wextra -Werror -std=c++0x -g -march=native
LDFLAGS		= -lompl -lboost_graph

CURDIR		= /home/cav2/repos/diverse_short_paths
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

define SSH
	ssh -t dione "source .bashrc; cd ${CURDIR}; $(1)"
endef

all: $(PCH) $(INCS) $(SRCS) $(EXEC)

remote:
	$(call SSH,make -j 15)

again:
	+@$(MAKE) clean
	+@$(MAKE)
r.again:
	$(call SSH,make again -j 15)

test:
	$(call SSH,PYTHONUNBUFFERED=1 ./test.py)

${EXEC}: $(OBJS) | dirs
	@echo -e "\033[1;34m[Linking ${EXEC}]\033[0m"
	${CXX} ${OBJS} ${LOCALLIB} ${LDFLAGS} -o $@

build/%.o: %.cpp $(PCH) $(INCS) | dirs
	@echo -e "\033[1;35m[Compiling $(subst $(SRCDIR)/,,$<)]\033[0m"
	${CXX} -c ${CXXFLAGS} ${LOCALINC} -include ${PCHINC} $< -o $@

${PCH}: $(PCHINC)
	@echo -e "\033[1;36m[Precompiling headers]\033[0m"
	${CXX} -x c++-header ${CXXFLAGS} ${LOCALINC} $< -o $@

.PHONY: dirs clean link
.SILENT: dirs clean link

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

link:
	rm -R ${EXEC}
	+@$(MAKE) ${EXEC}
r.link:
	$(call SSH,make link)

