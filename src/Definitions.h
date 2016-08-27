//
//  Definitions.h
//  roadLaneMarkings
//
//  Created by Mauricio de Paula on 21/08/14.
//  Copyright (c) 2014 ___HOME___. All rights reserved.
//

#ifndef roadLaneMarkings_Definitions_h
#define roadLaneMarkings_Definitions_h

enum boundary {
    LEFT,
    RIGHT,
    BOTH
};

/* from Brazilian Traffic Signs Manual */
const float __l_mk = 2.0f;   // maximum length of a dashed mark
const float __l_adj = 6.0f; // maximum length between two adjacent lane markings

/* For Gaussian Filter */
const float SIGMA_W = 0.020; // meters
const float K_SIGMA_I = 10;  // number of standard deviations for sigma_i
const float VAREPSILON = 35; // retrieve only the local extrema whose height is larger than VAREPSILON (15%)

/* For temporal information */
const float T = 5;
const float CHI_SQUARE_LIMIT = 7.377758908227871; //	degrees of freedom = 2; probability level you set for this test is p < 0.9750

/* Temporal window size for classified classes*/
const int T2 = 20;

/* Number of line segments used for C2 */
const float CUTOFF = 0.7;

#endif
