all: installed

#SVN_DIR = libdc1394v2-svn
#SVN_URL = https://libdc1394.svn.sourceforge.net/svnroot/libdc1394/trunk/libdc1394
#SVN_REVISION = -r 580
#SVN_PATCH = videre_fix.patch
#include $(shell rospack find mk)/svn_checkout.mk

FILENAME = libdc1394v2-svn-r580.tar.gz
TARBALL = build/$(FILENAME)
TARBALL_URL = http://pr.willowgarage.com/downloads/$(FILENAME)
SOURCE_DIR = build/libdc1394v2-svn
UNPACK_CMD = tar xzf
MD5SUM_FILE = $(FILENAME).md5sum
TARBALL_PATCH = videre_fix.patch configure_in.patch
include $(shell rospack find mk)/download_unpack_build.mk

installed: wiped $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && autoreconf -i -s
	cd $(SOURCE_DIR) && CFLAGS="-g -O2 -fPIC" ./configure --prefix=`rospack find libdc1394v2`/libdc1394v2 --enable-shared=no
	cd $(SOURCE_DIR) && ln -fs /usr/bin/libtool
	cd $(SOURCE_DIR) && make
	cd $(SOURCE_DIR) && make install
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean: 
	rm -rf libdc1394v2 installed

wipe: clean
	rm -rf build

.PHONY : clean wipe
