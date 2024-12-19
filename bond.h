/*This song is supposed to be the notes and notelenghts for James Bond theme 
The note lengths are not right, so the song doesn't sound quite right */

// nuotin taajuuksiksi
#define C4 261
#define D4 293
#define E4 329
#define F4 349
#define Fs4 370
#define G4 392
#define A4 440
#define B4b 466
#define B4 493
#define C5 523
#define D5 587
#define EB5 622
#define E5 659
#define F5 698
#define G5 784
#define A5 880
#define REST 0

// nuottien kestot
#define SIXTEEN 40
#define EIGHTH 80
#define QUARTER 160
#define PIKKUTAUKO 120
#define FOURTH 320
#define HALF 640

// Melody
float nuotit_bond[] = {
 E4, REST, Fs4, REST,
 Fs4, REST, Fs4, REST,
 E4, REST, E4, REST,
 E4, REST, E4, REST,
 G4, REST, G4, REST,
 G4, REST, Fs4, REST,
 Fs4, REST, E4, REST,
 EB5, REST, D5, REST,
 B4, REST, A4, REST, B4,
 0 // loppu
 };
int kesto_bond[] = {
 QUARTER, EIGHTH, EIGHTH, EIGHTH,
 EIGHTH, EIGHTH, QUARTER, EIGHTH,
 QUARTER, EIGHTH, QUARTER, EIGHTH,
 QUARTER, EIGHTH, QUARTER, EIGHTH,
 QUARTER, EIGHTH, QUARTER, EIGHTH,
 EIGHTH, EIGHTH, EIGHTH, EIGHTH,
 QUARTER, EIGHTH, QUARTER, EIGHTH,
 QUARTER, EIGHTH, QUARTER, EIGHTH,
 QUARTER, EIGHTH, QUARTER, EIGHTH,
 HALF,
 0 //loppu
 };
