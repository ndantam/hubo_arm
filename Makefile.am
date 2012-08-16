INCLUDES = -I$(top_srcdir)/include

EXTRA_DIST = src/hubo_kin.mac src/hubo_kin.f

include_HEADERS = include/hubo_kin.h



lib_LTLIBRARIES = libhubokin.la
bin_PROGRAMS = harmctrl hlegctrl



libhubokin_la_SOURCES = src/hubo_kin.f

harmctrl_SOURCES = src/harmctrl.c
harmctrl_LDADD = libhubokin.la


hlegctrl_SOURCES = src/hlegctrl.c
hlegctrl_LDADD = libhubokin.la

src/hubo_kin.f: src/hubo_kin.mac
	@echo [maxima start]
	cd src && maxima --very-quiet -b hubo_kin.mac
	@echo [maxima end]
