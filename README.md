yum

c++
xlib
sdl
openal
alut (freealut)
glut (freeglut - FAILED) / mesa (FAILED)
libpng
vamos

LGP

Mesa3D
./configure --host="i386-pc-solaris2" --build="i386-pc-solaris2"


sudo ln -s /usr/lib/libglut.so.3  /usr/lib/libglut.so

yum search ?
yum install --downloadonly --downloaddir="." ?
rpm -ivh *.rpm --nodeps --replacefiles