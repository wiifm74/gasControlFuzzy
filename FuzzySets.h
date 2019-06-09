#ifndef FUZZYSETS_H_

#define FUZZYSETS_H_

int errorMultiplier = 0.5
int changeMultiplier = 1

FuzzySet* errorN = new FuzzySet( -100 * errorMultiplier, -100 * errorMultiplier, -10 * errorMultiplier, 0 * errorMultiplier ); // Negative Error
FuzzySet* errorZero = new FuzzySet( -15 * errorMultiplier, -10 * errorMultiplier, 0 * errorMultiplier, 10 * errorMultiplier ); // Zero Error
FuzzySet* errorP = new FuzzySet( 0 * errorMultiplier, 10 * errorMultiplier, 100 * errorMultiplier, 100 * errorMultiplier ); // Positive Error

FuzzySet* errorChangeN = new FuzzySet( -5 * changeMultiplier, -5 * changeMultiplier, -1 * changeMultiplier, 0 * changeMultiplier ); // Negative ErrorChange
FuzzySet* errorChangeZero = new FuzzySet( -2 * changeMultiplier, 0 * changeMultiplier, 0 * changeMultiplier, 2 * changeMultiplier ); // Zero ErrorChange
FuzzySet* errorChangeP = new FuzzySet( 0 * changeMultiplier, 1 * changeMultiplier, 5 * changeMultiplier, 5 * changeMultiplier ); // Positive ErrorChange

FuzzySet* decrease = new FuzzySet( -70, -60, -60, -50 ); // decrease gas
FuzzySet* decreaseSmall = new FuzzySet( -45, -35, -35, -25 ); // small decrease gas
FuzzySet* zeroChange = new FuzzySet( -20, 0, 0, 10 ); // zero change
FuzzySet* increaseSmall = new FuzzySet( 15, 25, 25, 35 ); // small increase gas
FuzzySet* increase = new FuzzySet( 40, 50, 50, 60 ); // increase gas

#endif
