// https://gist.github.com/cslarsen/1870641

// Slightly modified by me to remove long scale and made the buffer a bit longer

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace iray::fmt {

    /*
     * Short scale units
     * http://en.wikipedia.org/wiki/Short_scale
     */
    static const char* short_scale[] = {"",          "thousand",    "million",     "billion",
                                        "trillion",  "quadrillion", "quintillion", "sextillion",
                                        "septillion"};

    /*
     * Convert number to human readable string using
     * the given naming system.
     */
    const char* scale(double n, int decimals = 1) {
        /*
         * Number of digits in n is given by
         * 10^x = n ==> x = log(n)/log(10) = log_10(n).
         *
         * So 1000 would be 1 + floor(log_10(10^3)) = 4 digits.
         */
        int digits = n == 0 ? 0 : 1 + floor(log10l(fabs(n)));

        // determine base 10 exponential
        int exp = digits <= 4 ? 0 : 3 * ((digits - 1) / 3);

        // normalized number
        double m = n / powl(10, exp);

        // no decimals? then don't print any
        if (m - static_cast<long>(n) == 0)
            decimals = 0;

        // don't print unit for exp<3
        static char        s[32];
        static const char* fmt[] = {"%1.*lf %s", "%1.*lf"};
        sprintf(s, fmt[exp < 3], decimals, m, short_scale[exp / 3]);
        return s;
    }

    /*
     * Convert number to short scale representation
     */
    const char* sscale(double n, int decimals = 1) {
        static char s[64];
        strcpy(s, scale(n, decimals));
        return s;
    }

} // namespace iray::fmt