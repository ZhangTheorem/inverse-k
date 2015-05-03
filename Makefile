SHELL = /bin/sh
.SUFFIXES:
.SUFFIXES: .cpp .o .h .d

CXXFLAGS = -g -Wall -Iinclude -fopenmp -DGL_GLEXT_PROTOTYPES
LDFLAGS = -lglut -lGL -lGLU -pthread
LDLIBS =
VPATH = src

TARGET = inverse-k
SRCEXT = cpp
SRCDIR = src
BUILDDIR = build
DEPDIR = dep

SOURCES := $(wildcard $(SRCDIR)/*.$(SRCEXT))
OBJECTS := $(patsubst $(SRCDIR)/%.$(SRCEXT), $(BUILDDIR)/%.o, $(SOURCES))
DEPENDS := $(patsubst $(SRCDIR)/%.$(SRCEXT), $(DEPDIR)/%.d, $(SOURCES))

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) $(LDLIBS)

ifneq ($(MAKECMDGOALS), clean)
-include $(DEPENDS)
endif

$(BUILDDIR)/%.o: | $(BUILDDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILDDIR):
	mkdir $(BUILDDIR)

$(DEPDIR)/%.d: $(SRCDIR)/%.$(SRCEXT) | $(DEPDIR)
	$(CXX) -MM -MT $(patsubst $(DEPDIR)/%.d, $(BUILDDIR)/%.o, $@) $(CXXFLAGS) $< > $@

$(DEPDIR):
	mkdir $(DEPDIR)

.PHONY: clean all
clean:
	$(RM) -r $(DEPDIR) $(BUILDDIR) $(TARGET)
