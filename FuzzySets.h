#ifndef FUZZYSETS_H_

#define FUZZYSETS_H_

FuzzySet* errorN = new FuzzySet( -100, -100, -10, 0 ); // Negative Error
FuzzySet* errorZero = new FuzzySet( -15, -10, 0, 10 ); // Zero Error
FuzzySet* errorP = new FuzzySet( 0, 10, 100, 100 ); // Positive Error

FuzzySet* errorChangeN = new FuzzySet( -5, -5, -0.1, 0 ); // Negative ErrorChange
FuzzySet* errorChangeZero = new FuzzySet( -0.1, 0, 0, 0.1 ); // Zero ErrorChange
FuzzySet* errorChangeP = new FuzzySet( 0, 0.1, 5, 5 ); // Positive ErrorChange

FuzzySet* decrease = new FuzzySet( -70, -60, -60, -50 ); // decrease gas
FuzzySet* decreaseSmall = new FuzzySet( -45, -35, -35, -25 ); // small decrease gas
FuzzySet* zeroChange = new FuzzySet( -20, 0, 0, 10 ); // zero change
FuzzySet* increaseSmall = new FuzzySet( 15, 25, 25, 35 ); // small increase gas
FuzzySet* increase = new FuzzySet( 40, 50, 50, 60 ); // increase gas

#endif
