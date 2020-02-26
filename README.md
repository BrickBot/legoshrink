# legoshrink

[Archived Website](http://web.archive.org/web/20070825203927/http://tarpit.rmc.ca/lesauvage/eee243/labs/resources/legoshrink.html)

This archive contains the Windows binary for legoshrink, a program for
receiving integrity or addressed messages from a Lego Mindstorms RCX brick
using the brickOS LNP protocol.

This particular binary has been compiled with English output.  You may
recompile for an French output by uncommenting the #define FRENCH 1
at the top of the file legoshrink.c.

The source code has been included as per the terms of the Mozilla Public
License.  If you alter this code for your own purposes, then you are also
bound by the MPL.  Check the header in legoshrink.c for more information.
If you only want to run the program under Windows, then the only file you need
from this archive is legoshrink.exe.  Try legoshrink -h to see the available
options.

You may be able to compile this program to run under Linux, but that has not
been tested.  Testing was only performed on Win 2000 and Win XP PCs with the
USB tower.

To compile under either platform, place the source in a subdirectory of your
Cygwin directory.  Rename the appropriate windows or linux makefile to
Makefile and type make on the command line.  Good luck!

Reports of success/failure for linux and/or the serial tower would be
appreciated.

Good luck and enjoy!


Mike LeSauvage
michael <dot> lesauvage <at> rmc <dot> ca        //This hides me from spam!
14 Feb 2004.
