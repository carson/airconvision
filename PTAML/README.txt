The PTAM variant used for the IROS submission with logging extensions.

Initial checkin notes by: carson@k2.t.u-tokyo.ac.jp

Currently builds on Ubuntu 11.10

I had to install:

- TooN
- libcvd
- gvars

(according to http://www.robots.ox.ac.uk/~bob/software/ptamm/manual.pdf)

- lib3ds-20080909 

The Makefile was modifed to link

To cause ubuntu recognize /usr/local/lib libraries I also:

# edited
sudo vim /etc/ld.so.conf

# appended
/usr/local/lib

# reload ld
sudo ldconfig
