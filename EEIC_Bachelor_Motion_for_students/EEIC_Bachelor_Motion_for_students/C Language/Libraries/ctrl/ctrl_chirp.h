#ifndef _CTRL_IDENT_h
#define _CTRL_IDENT_h

// チャープサインの式に使える定数集
#define f0 (101) //initial frequency[Hz]
#define ff (500) //final frequency [Hz]
#define A (2) //amplitude [V]
#define t_c (1) //chirptime[s]

void ctrl_chirp(double t_ch, double *out);
#endif