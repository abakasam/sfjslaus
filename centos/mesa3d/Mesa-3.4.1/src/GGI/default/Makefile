# Generated automatically from Makefile.in by configure.
# Makefile.in generated automatically by automake 1.4 from Makefile.am

# Copyright (C) 1994, 1995-8, 1999 Free Software Foundation, Inc.
# This Makefile.in is free software; the Free Software Foundation
# gives unlimited permission to copy and/or distribute it,
# with or without modifications, as long as this notice is preserved.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY, to the extent permitted by law; without
# even the implied warranty of MERCHANTABILITY or FITNESS FOR A
# PARTICULAR PURPOSE.


SHELL = /bin/sh

srcdir = .
top_srcdir = ../../..
prefix = /usr/local
exec_prefix = ${prefix}

bindir = ${exec_prefix}/bin
sbindir = ${exec_prefix}/sbin
libexecdir = ${exec_prefix}/libexec
datadir = ${prefix}/share
sysconfdir = ${prefix}/etc
sharedstatedir = ${prefix}/com
localstatedir = ${prefix}/var
libdir = ${exec_prefix}/lib
infodir = ${prefix}/info
mandir = ${prefix}/man
includedir = ${prefix}/include
oldincludedir = /usr/include

DESTDIR =

pkgdatadir = $(datadir)/Mesa
pkglibdir = $(libdir)/Mesa
pkgincludedir = $(includedir)/Mesa

top_builddir = ../../..

ACLOCAL = /home/cabox/workspace/mesa/Mesa-3.4.1/missing aclocal
AUTOCONF = autoconf
AUTOMAKE = /home/cabox/workspace/mesa/Mesa-3.4.1/missing automake
AUTOHEADER = autoheader

INSTALL = /usr/bin/install -c
INSTALL_PROGRAM = ${INSTALL} $(AM_INSTALL_PROGRAM_FLAGS)
INSTALL_DATA = ${INSTALL} -m 644
INSTALL_SCRIPT = ${INSTALL_PROGRAM}
transform = s,x,x,

NORMAL_INSTALL = :
PRE_INSTALL = :
POST_INSTALL = :
NORMAL_UNINSTALL = :
PRE_UNINSTALL = :
POST_UNINSTALL = :
host_alias = i386-pc-solaris2
host_triplet = i386-pc-solaris2
AS = @AS@
CC = gcc
CPP = gcc -E
DLLTOOL = @DLLTOOL@
EXEEXT = 
FX_CFLAGS = 
FX_LIBS = 
GGI_CFLAGS = 
GGI_LIBS = 
GLUT_CFLAGS =  -DNDEBUG
GLUT_LIBS = $(top_builddir)/src-glut/libglut.la
LIBGLUT_VERSION = 3:7:0
LIBGLU_VERSION = 1:1:030401
LIBGL_VERSION = 1:2:030401
LIBTOOL = $(SHELL) $(top_builddir)/libtool
LN_S = ln -s
MAKEINFO = /home/cabox/workspace/mesa/Mesa-3.4.1/missing makeinfo
OBJDUMP = @OBJDUMP@
OBJEXT = o
PACKAGE = Mesa
RANLIB = ranlib
STRIP = strip
SVGA_CFLAGS = 
SVGA_LIBS = 
THREAD_LIBS =  -lpthread
VERSION = 3.4.1
X_LIBADD =  -lXext -lXi -lX11
ggi_confdir = /usr/local/etc
ggi_libdir = /usr/local/lib

INCLUDES = -I$(top_srcdir)/include -I$(top_srcdir)/src/GGI/include 	-I$(top_srcdir)/src $(GGI_CFLAGS)


sublibdir = $(libdir)/ggi/mesa/default/
KGILIB = genkgi.la
KGILIB = 

sublib_LTLIBRARIES = stubs.la linear_8.la linear_15.la 	linear_16.la linear_24.la linear_32.la $(KGILIB)


genkgi_la_SOURCES = genkgi_visual.c genkgi_mode.c
genkgi_la_LDFLAGS = -module -no-undefined -avoid-verison 	-export-symbols-regex "GGIMesa.*"


stubs_la_SOURCES = stubs.c
stubs_la_LDFLAGS = -module -no-undefined -avoid-verison 	-export-symbols-regex "GGIMesa.*"


linear_8_la_SOURCES = linear_8.c
linear_8_la_LDFLAGS = -module -no-undefined -avoid-verison 	-export-symbols-regex "GGIMesa.*"


linear_15_la_SOURCES = linear_15.c
linear_15_la_LDFLAGS = -module -no-undefined -avoid-verison 	-export-symbols-regex "GGIMesa.*"


linear_16_la_SOURCES = linear_16.c
linear_16_la_LDFLAGS = -module -no-undefined -avoid-verison 	-export-symbols-regex "GGIMesa.*"


linear_24_la_SOURCES = linear_24.c
linear_24_la_LDFLAGS = -module -no-undefined -avoid-verison 	-export-symbols-regex "GGIMesa.*"


linear_32_la_SOURCES = linear_32.c
linear_32_la_LDFLAGS = -module -no-undefined -avoid-verison 	-export-symbols-regex "GGIMesa.*"


genkgiconfdir = $(sysconfdir)/ggi/mesa/targets
genkgiconf_DATA = genkgi.conf

DISTCLEANFILES = genkgi.conf

EXTRA_DIST = genkgi.h d3dim.h linear.c
mkinstalldirs = $(SHELL) $(top_srcdir)/mkinstalldirs
CONFIG_HEADER = ../../../conf.h
CONFIG_CLEAN_FILES =  genkgi.conf
LTLIBRARIES =  $(sublib_LTLIBRARIES)


DEFS = -DHAVE_CONFIG_H -I. -I$(srcdir) -I../../..
CPPFLAGS = 
LDFLAGS = 
LIBS = 
X_CFLAGS = 
X_LIBS = 
X_EXTRA_LIBS = 
X_PRE_LIBS = 
stubs_la_LIBADD = 
stubs_la_OBJECTS =  stubs.lo
linear_8_la_LIBADD = 
linear_8_la_OBJECTS =  linear_8.lo
linear_15_la_LIBADD = 
linear_15_la_OBJECTS =  linear_15.lo
linear_16_la_LIBADD = 
linear_16_la_OBJECTS =  linear_16.lo
linear_24_la_LIBADD = 
linear_24_la_OBJECTS =  linear_24.lo
linear_32_la_LIBADD = 
linear_32_la_OBJECTS =  linear_32.lo
genkgi_la_LIBADD = 
genkgi_la_OBJECTS =  genkgi_visual.lo genkgi_mode.lo
CFLAGS = -g -O2 -Wall -fomit-frame-pointer -ffast-math -fexpensive-optimizations -fstrict-aliasing -malign-loops=2 -malign-jumps=2 -malign-functions=2 -D_REENTRANT
COMPILE = $(CC) $(DEFS) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(AM_CFLAGS) $(CFLAGS)
LTCOMPILE = $(LIBTOOL) --mode=compile $(CC) $(DEFS) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(AM_CFLAGS) $(CFLAGS)
CCLD = $(CC)
LINK = $(LIBTOOL) --mode=link $(CCLD) $(AM_CFLAGS) $(CFLAGS) $(LDFLAGS) -o $@
DATA =  $(genkgiconf_DATA)

DIST_COMMON =  Makefile.am Makefile.in genkgi.conf.in


DISTFILES = $(DIST_COMMON) $(SOURCES) $(HEADERS) $(TEXINFOS) $(EXTRA_DIST)

TAR = gtar
GZIP_ENV = --best
DEP_FILES =  .deps/genkgi_mode.P .deps/genkgi_visual.P .deps/linear_15.P \
.deps/linear_16.P .deps/linear_24.P .deps/linear_32.P .deps/linear_8.P \
.deps/stubs.P
SOURCES = $(stubs_la_SOURCES) $(linear_8_la_SOURCES) $(linear_15_la_SOURCES) $(linear_16_la_SOURCES) $(linear_24_la_SOURCES) $(linear_32_la_SOURCES) $(genkgi_la_SOURCES)
OBJECTS = $(stubs_la_OBJECTS) $(linear_8_la_OBJECTS) $(linear_15_la_OBJECTS) $(linear_16_la_OBJECTS) $(linear_24_la_OBJECTS) $(linear_32_la_OBJECTS) $(genkgi_la_OBJECTS)

all: all-redirect
.SUFFIXES:
.SUFFIXES: .S .c .lo .o .obj .s
$(srcdir)/Makefile.in: Makefile.am $(top_srcdir)/configure.in $(ACLOCAL_M4) 
	cd $(top_srcdir) && $(AUTOMAKE) --gnu src/GGI/default/Makefile

Makefile: $(srcdir)/Makefile.in  $(top_builddir)/config.status $(BUILT_SOURCES)
	cd $(top_builddir) \
	  && CONFIG_FILES=$(subdir)/$@ CONFIG_HEADERS= $(SHELL) ./config.status

genkgi.conf: $(top_builddir)/config.status genkgi.conf.in
	cd $(top_builddir) && CONFIG_FILES=$(subdir)/$@ CONFIG_HEADERS= $(SHELL) ./config.status

mostlyclean-sublibLTLIBRARIES:

clean-sublibLTLIBRARIES:
	-test -z "$(sublib_LTLIBRARIES)" || rm -f $(sublib_LTLIBRARIES)

distclean-sublibLTLIBRARIES:

maintainer-clean-sublibLTLIBRARIES:

install-sublibLTLIBRARIES: $(sublib_LTLIBRARIES)
	@$(NORMAL_INSTALL)
	$(mkinstalldirs) $(DESTDIR)$(sublibdir)
	@list='$(sublib_LTLIBRARIES)'; for p in $$list; do \
	  if test -f $$p; then \
	    echo "$(LIBTOOL)  --mode=install $(INSTALL) $$p $(DESTDIR)$(sublibdir)/$$p"; \
	    $(LIBTOOL)  --mode=install $(INSTALL) $$p $(DESTDIR)$(sublibdir)/$$p; \
	  else :; fi; \
	done

uninstall-sublibLTLIBRARIES:
	@$(NORMAL_UNINSTALL)
	list='$(sublib_LTLIBRARIES)'; for p in $$list; do \
	  $(LIBTOOL)  --mode=uninstall rm -f $(DESTDIR)$(sublibdir)/$$p; \
	done

# FIXME: We should only use cygpath when building on Windows,
# and only if it is available.
.c.obj:
	$(COMPILE) -c `cygpath -w $<`

.s.o:
	$(COMPILE) -c $<

.S.o:
	$(COMPILE) -c $<

mostlyclean-compile:
	-rm -f *.o core *.core
	-rm -f *.$(OBJEXT)

clean-compile:

distclean-compile:
	-rm -f *.tab.c

maintainer-clean-compile:

.s.lo:
	$(LIBTOOL) --mode=compile $(COMPILE) -c $<

.S.lo:
	$(LIBTOOL) --mode=compile $(COMPILE) -c $<

mostlyclean-libtool:
	-rm -f *.lo

clean-libtool:
	-rm -rf .libs _libs

distclean-libtool:

maintainer-clean-libtool:

stubs.la: $(stubs_la_OBJECTS) $(stubs_la_DEPENDENCIES)
	$(LINK) -rpath $(sublibdir) $(stubs_la_LDFLAGS) $(stubs_la_OBJECTS) $(stubs_la_LIBADD) $(LIBS)

linear_8.la: $(linear_8_la_OBJECTS) $(linear_8_la_DEPENDENCIES)
	$(LINK) -rpath $(sublibdir) $(linear_8_la_LDFLAGS) $(linear_8_la_OBJECTS) $(linear_8_la_LIBADD) $(LIBS)

linear_15.la: $(linear_15_la_OBJECTS) $(linear_15_la_DEPENDENCIES)
	$(LINK) -rpath $(sublibdir) $(linear_15_la_LDFLAGS) $(linear_15_la_OBJECTS) $(linear_15_la_LIBADD) $(LIBS)

linear_16.la: $(linear_16_la_OBJECTS) $(linear_16_la_DEPENDENCIES)
	$(LINK) -rpath $(sublibdir) $(linear_16_la_LDFLAGS) $(linear_16_la_OBJECTS) $(linear_16_la_LIBADD) $(LIBS)

linear_24.la: $(linear_24_la_OBJECTS) $(linear_24_la_DEPENDENCIES)
	$(LINK) -rpath $(sublibdir) $(linear_24_la_LDFLAGS) $(linear_24_la_OBJECTS) $(linear_24_la_LIBADD) $(LIBS)

linear_32.la: $(linear_32_la_OBJECTS) $(linear_32_la_DEPENDENCIES)
	$(LINK) -rpath $(sublibdir) $(linear_32_la_LDFLAGS) $(linear_32_la_OBJECTS) $(linear_32_la_LIBADD) $(LIBS)

genkgi.la: $(genkgi_la_OBJECTS) $(genkgi_la_DEPENDENCIES)
	$(LINK) -rpath $(sublibdir) $(genkgi_la_LDFLAGS) $(genkgi_la_OBJECTS) $(genkgi_la_LIBADD) $(LIBS)

install-genkgiconfDATA: $(genkgiconf_DATA)
	@$(NORMAL_INSTALL)
	$(mkinstalldirs) $(DESTDIR)$(genkgiconfdir)
	@list='$(genkgiconf_DATA)'; for p in $$list; do \
	  if test -f $(srcdir)/$$p; then \
	    echo " $(INSTALL_DATA) $(srcdir)/$$p $(DESTDIR)$(genkgiconfdir)/$$p"; \
	    $(INSTALL_DATA) $(srcdir)/$$p $(DESTDIR)$(genkgiconfdir)/$$p; \
	  else if test -f $$p; then \
	    echo " $(INSTALL_DATA) $$p $(DESTDIR)$(genkgiconfdir)/$$p"; \
	    $(INSTALL_DATA) $$p $(DESTDIR)$(genkgiconfdir)/$$p; \
	  fi; fi; \
	done

uninstall-genkgiconfDATA:
	@$(NORMAL_UNINSTALL)
	list='$(genkgiconf_DATA)'; for p in $$list; do \
	  rm -f $(DESTDIR)$(genkgiconfdir)/$$p; \
	done

tags: TAGS

ID: $(HEADERS) $(SOURCES) $(LISP)
	list='$(SOURCES) $(HEADERS)'; \
	unique=`for i in $$list; do echo $$i; done | \
	  awk '    { files[$$0] = 1; } \
	       END { for (i in files) print i; }'`; \
	here=`pwd` && cd $(srcdir) \
	  && mkid -f$$here/ID $$unique $(LISP)

TAGS:  $(HEADERS) $(SOURCES)  $(TAGS_DEPENDENCIES) $(LISP)
	tags=; \
	here=`pwd`; \
	list='$(SOURCES) $(HEADERS)'; \
	unique=`for i in $$list; do echo $$i; done | \
	  awk '    { files[$$0] = 1; } \
	       END { for (i in files) print i; }'`; \
	test -z "$(ETAGS_ARGS)$$unique$(LISP)$$tags" \
	  || (cd $(srcdir) && etags $(ETAGS_ARGS) $$tags  $$unique $(LISP) -o $$here/TAGS)

mostlyclean-tags:

clean-tags:

distclean-tags:
	-rm -f TAGS ID

maintainer-clean-tags:

distdir = $(top_builddir)/$(PACKAGE)-$(VERSION)/$(subdir)

subdir = src/GGI/default

distdir: $(DISTFILES)
	here=`cd $(top_builddir) && pwd`; \
	top_distdir=`cd $(top_distdir) && pwd`; \
	distdir=`cd $(distdir) && pwd`; \
	cd $(top_srcdir) \
	  && $(AUTOMAKE) --include-deps --build-dir=$$here --srcdir-name=$(top_srcdir) --output-dir=$$top_distdir --gnu src/GGI/default/Makefile
	@for file in $(DISTFILES); do \
	  d=$(srcdir); \
	  if test -d $$d/$$file; then \
	    cp -pr $$d/$$file $(distdir)/$$file; \
	  else \
	    test -f $(distdir)/$$file \
	    || ln $$d/$$file $(distdir)/$$file 2> /dev/null \
	    || cp -p $$d/$$file $(distdir)/$$file || :; \
	  fi; \
	done

DEPS_MAGIC := $(shell mkdir .deps > /dev/null 2>&1 || :)

-include $(DEP_FILES)

mostlyclean-depend:

clean-depend:

distclean-depend:
	-rm -rf .deps

maintainer-clean-depend:

%.o: %.c
	@echo '$(COMPILE) -c $<'; \
	$(COMPILE) -Wp,-MD,.deps/$(*F).pp -c $<
	@-cp .deps/$(*F).pp .deps/$(*F).P; \
	tr ' ' '\012' < .deps/$(*F).pp \
	  | sed -e 's/^\\$$//' -e '/^$$/ d' -e '/:$$/ d' -e 's/$$/ :/' \
	    >> .deps/$(*F).P; \
	rm .deps/$(*F).pp

%.lo: %.c
	@echo '$(LTCOMPILE) -c $<'; \
	$(LTCOMPILE) -Wp,-MD,.deps/$(*F).pp -c $<
	@-sed -e 's/^\([^:]*\)\.o[ 	]*:/\1.lo \1.o :/' \
	  < .deps/$(*F).pp > .deps/$(*F).P; \
	tr ' ' '\012' < .deps/$(*F).pp \
	  | sed -e 's/^\\$$//' -e '/^$$/ d' -e '/:$$/ d' -e 's/$$/ :/' \
	    >> .deps/$(*F).P; \
	rm -f .deps/$(*F).pp
info-am:
info: info-am
dvi-am:
dvi: dvi-am
check-am: all-am
check: check-am
installcheck-am:
installcheck: installcheck-am
install-exec-am:
install-exec: install-exec-am

install-data-am: install-sublibLTLIBRARIES install-genkgiconfDATA
install-data: install-data-am

install-am: all-am
	@$(MAKE) $(AM_MAKEFLAGS) install-exec-am install-data-am
install: install-am
uninstall-am: uninstall-sublibLTLIBRARIES uninstall-genkgiconfDATA
uninstall: uninstall-am
all-am: Makefile $(LTLIBRARIES) $(DATA)
all-redirect: all-am
install-strip:
	$(MAKE) $(AM_MAKEFLAGS) AM_INSTALL_PROGRAM_FLAGS=-s install
installdirs:
	$(mkinstalldirs)  $(DESTDIR)$(sublibdir) $(DESTDIR)$(genkgiconfdir)


mostlyclean-generic:

clean-generic:

distclean-generic:
	-rm -f Makefile $(CONFIG_CLEAN_FILES)
	-rm -f config.cache config.log stamp-h stamp-h[0-9]*
	-test -z "$(DISTCLEANFILES)" || rm -f $(DISTCLEANFILES)

maintainer-clean-generic:
mostlyclean-am:  mostlyclean-sublibLTLIBRARIES mostlyclean-compile \
		mostlyclean-libtool mostlyclean-tags mostlyclean-depend \
		mostlyclean-generic

mostlyclean: mostlyclean-am

clean-am:  clean-sublibLTLIBRARIES clean-compile clean-libtool \
		clean-tags clean-depend clean-generic mostlyclean-am

clean: clean-am

distclean-am:  distclean-sublibLTLIBRARIES distclean-compile \
		distclean-libtool distclean-tags distclean-depend \
		distclean-generic clean-am
	-rm -f libtool

distclean: distclean-am

maintainer-clean-am:  maintainer-clean-sublibLTLIBRARIES \
		maintainer-clean-compile maintainer-clean-libtool \
		maintainer-clean-tags maintainer-clean-depend \
		maintainer-clean-generic distclean-am
	@echo "This command is intended for maintainers to use;"
	@echo "it deletes files that may require special tools to rebuild."

maintainer-clean: maintainer-clean-am

.PHONY: mostlyclean-sublibLTLIBRARIES distclean-sublibLTLIBRARIES \
clean-sublibLTLIBRARIES maintainer-clean-sublibLTLIBRARIES \
uninstall-sublibLTLIBRARIES install-sublibLTLIBRARIES \
mostlyclean-compile distclean-compile clean-compile \
maintainer-clean-compile mostlyclean-libtool distclean-libtool \
clean-libtool maintainer-clean-libtool uninstall-genkgiconfDATA \
install-genkgiconfDATA tags mostlyclean-tags distclean-tags clean-tags \
maintainer-clean-tags distdir mostlyclean-depend distclean-depend \
clean-depend maintainer-clean-depend info-am info dvi-am dvi check \
check-am installcheck-am installcheck install-exec-am install-exec \
install-data-am install-data install-am install uninstall-am uninstall \
all-redirect all-am all installdirs mostlyclean-generic \
distclean-generic clean-generic maintainer-clean-generic clean \
mostlyclean distclean maintainer-clean


# Tell versions [3.59,3.63) of GNU make to not export all variables.
# Otherwise a system limit (for SysV at least) may be exceeded.
.NOEXPORT:
