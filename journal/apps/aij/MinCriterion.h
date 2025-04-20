#ifndef MINCRITERION_H
#define MINCRITERION_H

enum MinCriterion {
    MinG,
    MinF,
    MinD,
    MinB,
    MinRD,
    MinRF,
    MinGF,
    MinGD,
    MinFRD,
    MinRFD,
    MinRFRD,
    MinGRF,
    MinGRD,
    MinGB,
    MinGFRD,
    MinGRFD,
    MinGRFRD,
    MinTot, MaxTot
};


enum SideCriterion {
    Alt, Cardinality, OptCount, Decay
};


#endif //MINCRITERION_H
