#!/bin/csh
setenv TARGET loongarch
setenv TARGET_ARCH loongarch64
setenv SRCCONF $PWD/notos/src.conf
#setenv CROSS_TOOLCHAIN llvm17

cd $PWD
sudo make buildworld

#cd /usr/src; time env MACHINE_ARCH=loongarch64  MACHINE=loongarch  CPUTYPE= LOCAL_MODULES= CC="cc -target loongarch64-unknown-freebsd14.0 --sysroot=/usr/obj/usr/src/loongarch.loongarch64/tmp -B/usrsr/bin"  CPP="cpp -target loongarch64-unknown-freebsd14.0 --sysroot=/usr/obj/usr/src/loongarch.loongarch64/tmp -B/usr/obj/usr/src/loongarch.loongarch64/tmp/usr/bin"  AS="as" AR="ar" ELFCTL="elfctl" LD="ld"  LLVM_LINK="" NM=nm OBJCOPY="objcopy"  RANLIB=ranlib STRINGS=  SIZE="size" STRIPBIN="strip"  INSTALL="install -U"  PATH=/usr/obj/usr/src/loongarch.loongarch64/tmp/bin:/usr/obj/usr/src/loongarch.loongarch64/tmp/usr/sbin:/usr/obj/usr/src/loongarch.loongarch64/tmp/usr/bin:/usr/obj/usr/src/loongarch.loongarch64/tmp/legacy/usr/sbin:/usr/obj/usr/src/loongarch.loongarch64/tmp/legacy/usr/bin:/usr/obj/usr/src/loongarch.loongarch64/tmp/legacy/bin:/usr/obj/usr/src/loongarch.loongarch64/tmp/legacy/usr/libexec::/sbin:/bin:/usr/sbin:/usr/bin  SYSROOT=/usr/obj/usr/src/loongarch.loongarch64/tmp make  -f Makefile.inc1  BWPHASE=cleanobj  DESTDIR=/usr/obj/usr/src/loongarch.loongarch64/tmp _NO_INCLUDE_COMPILERMK=t cleandir

