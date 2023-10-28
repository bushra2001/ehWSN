// ------------------------------
// ehWSN.cc - file for the EH-WSN project
//
// E. Elmallah: July 2023
// ------------------------------

#include "ehWSN.h"	// includes <omentpp.h> and other header files

// ------------------------------
// Some "one-liners"
//
void newWS (int x, char c) {for (int i=0; i < x; i++) printf ("%c",c); }

void WARNING (const char *fmt, ... )
{
    va_list  ap; fflush (stdout);
    va_start (ap, fmt);  vfprintf (stderr, fmt, ap);  va_end(ap);
}    

void FATAL (const char *fmt, ... )
{
    va_list  ap; fflush (stdout);
    va_start (ap, fmt);  vfprintf (stderr, fmt, ap);  va_end(ap);
    fflush (NULL);
    exit(1);
}              
// ------------------------------
// Other helper functions are in file "Util.cc"
