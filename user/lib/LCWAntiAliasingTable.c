#include "LCWAntiAliasingTable.h"

const int16_t gLcwAntiAiliasingTable[LCW_AA_TABLE_SIZE] = {
    16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384,
    16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384,
    16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384,
    16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384,
    16384, 16384, 16384, 16384, 16384, 16383, 16383, 16383,
    16383, 16383, 16383, 16383, 16383, 16383, 16383, 16383,
    16383, 16383, 16383, 16383, 16383, 16383, 16383, 16383,
    16382, 16382, 16382, 16382, 16382, 16382, 16382, 16382,
    16382, 16381, 16381, 16381, 16381, 16381, 16380, 16380,
    16380, 16380, 16380, 16379, 16379, 16379, 16378, 16378,
    16378, 16377, 16377, 16376, 16376, 16376, 16375, 16375,
    16374, 16373, 16373, 16372, 16371, 16371, 16370, 16369,
    16368, 16367, 16366, 16365, 16364, 16363, 16361, 16360,
    16359, 16357, 16355, 16354, 16352, 16350, 16348, 16346,
    16343, 16341, 16339, 16336, 16333, 16330, 16327, 16323,
    16320, 16316, 16312, 16307, 16303, 16298, 16293, 16287,
    16282, 16276, 16269, 16262, 16255, 16248, 16240, 16231,
    16222, 16212, 16202, 16192, 16180, 16168, 16156, 16142,
    16128, 16113, 16097, 16081, 16063, 16044, 16025, 16004,
    15982, 15959, 15934, 15908, 15881, 15852, 15822, 15790,
    15757, 15721, 15684, 15645, 15604, 15560, 15515, 15467,
    15417, 15364, 15309, 15251, 15190, 15127, 15060, 14991,
    14918, 14842, 14763, 14681, 14595, 14505, 14412, 14315,
    14215, 14111, 14003, 13891, 13775, 13656, 13533, 13405,
    13275, 13140, 13002, 12860, 12714, 12565, 12413, 12257,
    12099, 11937, 11773, 11606, 11436, 11264, 11090, 10914,
    10737, 10558, 10377, 10196, 10014,  9831,  9647,  9464,
     9280,  9097,  8914,  8732,  8550,  8369,  8190,  8012,
     7835,  7660,  7486,  7315,  7145,  6978,  6813,  6650,
     6490,  6331,  6176,  6023,  5873,  5725,  5580,  5438,
     5298,  5162,  5028,  4896,  4768,  4642,  4519,  4399,
     4282,  4167,  4055,  3945,  3838,  3734,  3632,  3533,
     3436,  3342,  3249,  3160,  3072,  2987,  2904,  2823,
     2745,  2668,  2593,  2521,  2450,  2381,  2314,  2249,
     2186,  2124,  2064,  2006,  1949,  1894,  1840,  1788,
     1737,  1688,  1640,  1594,  1548,  1504,  1461,  1420,
     1379,  1340,  1302,  1265,  1229,  1194,  1159,  1126,
     1094,  1063,  1033,  1003,   974,   946,   919,   893,
      868,   843,   819,   795,   772,   750,   729,   708,
      688,   668,   649,   630,   612,   595,   578,   561,
      545,   529,   514,   499,   485,   471,   458,   445,
      432,   419,   407,   396,   384,   373,   363,   352,
      342,   332,   323,   314,   305,   296,   287,   279,
      271,   263,   256,   248,   241,   234,   228,   221,
      215,   209,   203,   197,   191,   186,   180,   175,
      170,   165,   161,   156,   151,   147,   143,   139,
      135,   131,   127,   124,   120,   117,   113,   110,
      107,   104,   101,    98,    95,    92,    90,    87,
       85,    82,    80,    78,    75,    73,    71,    69,
       67,    65,    63,    61,    60,    58,    56,    55,
       53,    52,    50,    49,    47,    46,    45,    43,
       42,    41,    40,    39,    37,    36,    35,    34,
       33,    32,    31,    31,    30,    29,    28,    27,
       26,    26,    25,    24,    24,    23,    22,    22,
       21,    20,    20,    19,    19,    18,    18,    17,
       17,    16,    16,    15,    15,    14,    14,    14,
       13,    13,    12,    12,    12,    11,    11,    11,
       10,    10,    10,    10,     9,     9,     9,     8,
        8,     8,     8,     8,     7,     7,     7,     7,
        7,     6,     6,     6,     6,     6,     5,     5,
        5,     5,     5,     5,     5,     4,     4,     4,
        4,     4,     4,     4,     4,     4,     3,     3,
        3,     3,     3,     3,     3,     3,     3,     3,
        3,     2,     2,     2,     2,     2,     2,     2,
        2,     2,     2,     2,     2,     2,     2,     2,
        2,     2,     2,     1,     1,     1,     1,     1,
        1,     1,     1,     1,     1,     1,     1,     1,
        1,     1,     1,     1,     1,     1,     1,     1,
        1,     1,     1,     1,     1,     1,     1,     1,
        1,     1,     1,     1,     1,     1,     1,     1,
        1,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,     0
};

