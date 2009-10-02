#define FIN	7370000L

#define BIT(x) (1<<(x))
#define sbi(x,y)	((x) |= (1<<(y)))
#define cbi(x,y)	((x) &= ~(1<<(y)))
#define tbi(x,y) 	((x) ^= (1<<(y)))

#define _NOPSV __attribute__((no_auto_psv))

#define TRUE	1
#define FALSE	0

#define ON      1
#define OFF     0

#define SGN(x)      ( (x) >= 0) ? (1) : (-1) )
#define ABS(x)      ( ((x) >= 0) ? (x) : (-(x)) )

