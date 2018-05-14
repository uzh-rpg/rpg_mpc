/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"


real_t rk_dim20_swap;

/** Column vector of size: 20 */
real_t rk_dim20_bPerm[ 20 ];

real_t rk_ttt;

/** Row vector of size: 24 */
real_t rk_xxx[ 24 ];

/** Matrix of size: 10 x 2 (row major format) */
real_t rk_kkk[ 20 ];

/** Matrix of size: 20 x 20 (row major format) */
real_t rk_A[ 400 ];

/** Column vector of size: 20 */
real_t rk_b[ 20 ];

/** Row vector of size: 20 */
int rk_dim20_perm[ 20 ];

/** Column vector of size: 10 */
real_t rk_rhsTemp[ 10 ];

/** Matrix of size: 2 x 140 (row major format) */
real_t rk_diffsTemp2[ 280 ];

/** Matrix of size: 10 x 2 (row major format) */
real_t rk_diffK[ 20 ];

/** Matrix of size: 10 x 14 (row major format) */
real_t rk_diffsNew2[ 140 ];

#pragma omp threadprivate( auxVar, rk_ttt, rk_xxx, rk_kkk, rk_diffK, rk_rhsTemp, rk_dim20_perm, rk_A, rk_b, rk_diffsNew2, rk_diffsTemp2, rk_dim20_swap, rk_dim20_bPerm )

void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 10;

/* Compute outputs: */
out[0] = xd[7];
out[1] = xd[8];
out[2] = xd[9];
out[3] = ((real_t)(5.0000000000000000e-01)*(((((real_t)(0.0000000000000000e+00)-u[1])*xd[4])-(u[2]*xd[5]))-(u[3]*xd[6])));
out[4] = ((real_t)(5.0000000000000000e-01)*(((u[1]*xd[3])+(u[3]*xd[5]))-(u[2]*xd[6])));
out[5] = ((real_t)(5.0000000000000000e-01)*(((u[2]*xd[3])-(u[3]*xd[4]))+(u[1]*xd[6])));
out[6] = ((real_t)(5.0000000000000000e-01)*(((u[3]*xd[3])+(u[2]*xd[4]))+(u[3]*xd[5])));
out[7] = (((real_t)(2.0000000000000000e+00)*((xd[3]*xd[5])+(xd[4]*xd[6])))*u[0]);
out[8] = (((real_t)(2.0000000000000000e+00)*((xd[5]*xd[6])-(xd[3]*xd[4])))*u[0]);
out[9] = (((((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[4])*xd[4]))-(((real_t)(2.0000000000000000e+00)*xd[5])*xd[5]))*u[0])-(real_t)(9.8065999999999995e+00));
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 10;

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(1.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(1.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(1.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[1]));
out[47] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[2]));
out[48] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[3]));
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = ((real_t)(5.0000000000000000e-01)*(((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*xd[4]));
out[54] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[5]));
out[55] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[6]));
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = ((real_t)(5.0000000000000000e-01)*u[1]);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = ((real_t)(5.0000000000000000e-01)*u[3]);
out[62] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[2]));
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = ((real_t)(5.0000000000000000e-01)*xd[3]);
out[68] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[6]));
out[69] = ((real_t)(5.0000000000000000e-01)*xd[5]);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = ((real_t)(5.0000000000000000e-01)*u[2]);
out[74] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[3]));
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = ((real_t)(5.0000000000000000e-01)*u[1]);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = ((real_t)(5.0000000000000000e-01)*xd[6]);
out[82] = ((real_t)(5.0000000000000000e-01)*xd[3]);
out[83] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[4]));
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = ((real_t)(5.0000000000000000e-01)*u[3]);
out[88] = ((real_t)(5.0000000000000000e-01)*u[2]);
out[89] = ((real_t)(5.0000000000000000e-01)*u[3]);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(0.0000000000000000e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = ((real_t)(5.0000000000000000e-01)*xd[4]);
out[97] = ((real_t)(5.0000000000000000e-01)*(xd[3]+xd[5]));
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (((real_t)(2.0000000000000000e+00)*xd[5])*u[0]);
out[102] = (((real_t)(2.0000000000000000e+00)*xd[6])*u[0]);
out[103] = (((real_t)(2.0000000000000000e+00)*xd[3])*u[0]);
out[104] = (((real_t)(2.0000000000000000e+00)*xd[4])*u[0]);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = ((real_t)(2.0000000000000000e+00)*((xd[3]*xd[5])+(xd[4]*xd[6])));
out[109] = (real_t)(0.0000000000000000e+00);
out[110] = (real_t)(0.0000000000000000e+00);
out[111] = (real_t)(0.0000000000000000e+00);
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(0.0000000000000000e+00);
out[114] = (real_t)(0.0000000000000000e+00);
out[115] = (((real_t)(2.0000000000000000e+00)*((real_t)(0.0000000000000000e+00)-xd[4]))*u[0]);
out[116] = (((real_t)(2.0000000000000000e+00)*((real_t)(0.0000000000000000e+00)-xd[3]))*u[0]);
out[117] = (((real_t)(2.0000000000000000e+00)*xd[6])*u[0]);
out[118] = (((real_t)(2.0000000000000000e+00)*xd[5])*u[0]);
out[119] = (real_t)(0.0000000000000000e+00);
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = ((real_t)(2.0000000000000000e+00)*((xd[5]*xd[6])-(xd[3]*xd[4])));
out[123] = (real_t)(0.0000000000000000e+00);
out[124] = (real_t)(0.0000000000000000e+00);
out[125] = (real_t)(0.0000000000000000e+00);
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (((real_t)(0.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[4])+((real_t)(2.0000000000000000e+00)*xd[4])))*u[0]);
out[131] = (((real_t)(0.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[5])+((real_t)(2.0000000000000000e+00)*xd[5])))*u[0]);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = (real_t)(0.0000000000000000e+00);
out[136] = (((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[4])*xd[4]))-(((real_t)(2.0000000000000000e+00)*xd[5])*xd[5]));
out[137] = (real_t)(0.0000000000000000e+00);
out[138] = (real_t)(0.0000000000000000e+00);
out[139] = (real_t)(0.0000000000000000e+00);
}



void acado_solve_dim20_triangular( real_t* const A, real_t* const b )
{

b[19] = b[19]/A[399];
b[18] -= + A[379]*b[19];
b[18] = b[18]/A[378];
b[17] -= + A[359]*b[19];
b[17] -= + A[358]*b[18];
b[17] = b[17]/A[357];
b[16] -= + A[339]*b[19];
b[16] -= + A[338]*b[18];
b[16] -= + A[337]*b[17];
b[16] = b[16]/A[336];
b[15] -= + A[319]*b[19];
b[15] -= + A[318]*b[18];
b[15] -= + A[317]*b[17];
b[15] -= + A[316]*b[16];
b[15] = b[15]/A[315];
b[14] -= + A[299]*b[19];
b[14] -= + A[298]*b[18];
b[14] -= + A[297]*b[17];
b[14] -= + A[296]*b[16];
b[14] -= + A[295]*b[15];
b[14] = b[14]/A[294];
b[13] -= + A[279]*b[19];
b[13] -= + A[278]*b[18];
b[13] -= + A[277]*b[17];
b[13] -= + A[276]*b[16];
b[13] -= + A[275]*b[15];
b[13] -= + A[274]*b[14];
b[13] = b[13]/A[273];
b[12] -= + A[259]*b[19];
b[12] -= + A[258]*b[18];
b[12] -= + A[257]*b[17];
b[12] -= + A[256]*b[16];
b[12] -= + A[255]*b[15];
b[12] -= + A[254]*b[14];
b[12] -= + A[253]*b[13];
b[12] = b[12]/A[252];
b[11] -= + A[239]*b[19];
b[11] -= + A[238]*b[18];
b[11] -= + A[237]*b[17];
b[11] -= + A[236]*b[16];
b[11] -= + A[235]*b[15];
b[11] -= + A[234]*b[14];
b[11] -= + A[233]*b[13];
b[11] -= + A[232]*b[12];
b[11] = b[11]/A[231];
b[10] -= + A[219]*b[19];
b[10] -= + A[218]*b[18];
b[10] -= + A[217]*b[17];
b[10] -= + A[216]*b[16];
b[10] -= + A[215]*b[15];
b[10] -= + A[214]*b[14];
b[10] -= + A[213]*b[13];
b[10] -= + A[212]*b[12];
b[10] -= + A[211]*b[11];
b[10] = b[10]/A[210];
b[9] -= + A[199]*b[19];
b[9] -= + A[198]*b[18];
b[9] -= + A[197]*b[17];
b[9] -= + A[196]*b[16];
b[9] -= + A[195]*b[15];
b[9] -= + A[194]*b[14];
b[9] -= + A[193]*b[13];
b[9] -= + A[192]*b[12];
b[9] -= + A[191]*b[11];
b[9] -= + A[190]*b[10];
b[9] = b[9]/A[189];
b[8] -= + A[179]*b[19];
b[8] -= + A[178]*b[18];
b[8] -= + A[177]*b[17];
b[8] -= + A[176]*b[16];
b[8] -= + A[175]*b[15];
b[8] -= + A[174]*b[14];
b[8] -= + A[173]*b[13];
b[8] -= + A[172]*b[12];
b[8] -= + A[171]*b[11];
b[8] -= + A[170]*b[10];
b[8] -= + A[169]*b[9];
b[8] = b[8]/A[168];
b[7] -= + A[159]*b[19];
b[7] -= + A[158]*b[18];
b[7] -= + A[157]*b[17];
b[7] -= + A[156]*b[16];
b[7] -= + A[155]*b[15];
b[7] -= + A[154]*b[14];
b[7] -= + A[153]*b[13];
b[7] -= + A[152]*b[12];
b[7] -= + A[151]*b[11];
b[7] -= + A[150]*b[10];
b[7] -= + A[149]*b[9];
b[7] -= + A[148]*b[8];
b[7] = b[7]/A[147];
b[6] -= + A[139]*b[19];
b[6] -= + A[138]*b[18];
b[6] -= + A[137]*b[17];
b[6] -= + A[136]*b[16];
b[6] -= + A[135]*b[15];
b[6] -= + A[134]*b[14];
b[6] -= + A[133]*b[13];
b[6] -= + A[132]*b[12];
b[6] -= + A[131]*b[11];
b[6] -= + A[130]*b[10];
b[6] -= + A[129]*b[9];
b[6] -= + A[128]*b[8];
b[6] -= + A[127]*b[7];
b[6] = b[6]/A[126];
b[5] -= + A[119]*b[19];
b[5] -= + A[118]*b[18];
b[5] -= + A[117]*b[17];
b[5] -= + A[116]*b[16];
b[5] -= + A[115]*b[15];
b[5] -= + A[114]*b[14];
b[5] -= + A[113]*b[13];
b[5] -= + A[112]*b[12];
b[5] -= + A[111]*b[11];
b[5] -= + A[110]*b[10];
b[5] -= + A[109]*b[9];
b[5] -= + A[108]*b[8];
b[5] -= + A[107]*b[7];
b[5] -= + A[106]*b[6];
b[5] = b[5]/A[105];
b[4] -= + A[99]*b[19];
b[4] -= + A[98]*b[18];
b[4] -= + A[97]*b[17];
b[4] -= + A[96]*b[16];
b[4] -= + A[95]*b[15];
b[4] -= + A[94]*b[14];
b[4] -= + A[93]*b[13];
b[4] -= + A[92]*b[12];
b[4] -= + A[91]*b[11];
b[4] -= + A[90]*b[10];
b[4] -= + A[89]*b[9];
b[4] -= + A[88]*b[8];
b[4] -= + A[87]*b[7];
b[4] -= + A[86]*b[6];
b[4] -= + A[85]*b[5];
b[4] = b[4]/A[84];
b[3] -= + A[79]*b[19];
b[3] -= + A[78]*b[18];
b[3] -= + A[77]*b[17];
b[3] -= + A[76]*b[16];
b[3] -= + A[75]*b[15];
b[3] -= + A[74]*b[14];
b[3] -= + A[73]*b[13];
b[3] -= + A[72]*b[12];
b[3] -= + A[71]*b[11];
b[3] -= + A[70]*b[10];
b[3] -= + A[69]*b[9];
b[3] -= + A[68]*b[8];
b[3] -= + A[67]*b[7];
b[3] -= + A[66]*b[6];
b[3] -= + A[65]*b[5];
b[3] -= + A[64]*b[4];
b[3] = b[3]/A[63];
b[2] -= + A[59]*b[19];
b[2] -= + A[58]*b[18];
b[2] -= + A[57]*b[17];
b[2] -= + A[56]*b[16];
b[2] -= + A[55]*b[15];
b[2] -= + A[54]*b[14];
b[2] -= + A[53]*b[13];
b[2] -= + A[52]*b[12];
b[2] -= + A[51]*b[11];
b[2] -= + A[50]*b[10];
b[2] -= + A[49]*b[9];
b[2] -= + A[48]*b[8];
b[2] -= + A[47]*b[7];
b[2] -= + A[46]*b[6];
b[2] -= + A[45]*b[5];
b[2] -= + A[44]*b[4];
b[2] -= + A[43]*b[3];
b[2] = b[2]/A[42];
b[1] -= + A[39]*b[19];
b[1] -= + A[38]*b[18];
b[1] -= + A[37]*b[17];
b[1] -= + A[36]*b[16];
b[1] -= + A[35]*b[15];
b[1] -= + A[34]*b[14];
b[1] -= + A[33]*b[13];
b[1] -= + A[32]*b[12];
b[1] -= + A[31]*b[11];
b[1] -= + A[30]*b[10];
b[1] -= + A[29]*b[9];
b[1] -= + A[28]*b[8];
b[1] -= + A[27]*b[7];
b[1] -= + A[26]*b[6];
b[1] -= + A[25]*b[5];
b[1] -= + A[24]*b[4];
b[1] -= + A[23]*b[3];
b[1] -= + A[22]*b[2];
b[1] = b[1]/A[21];
b[0] -= + A[19]*b[19];
b[0] -= + A[18]*b[18];
b[0] -= + A[17]*b[17];
b[0] -= + A[16]*b[16];
b[0] -= + A[15]*b[15];
b[0] -= + A[14]*b[14];
b[0] -= + A[13]*b[13];
b[0] -= + A[12]*b[12];
b[0] -= + A[11]*b[11];
b[0] -= + A[10]*b[10];
b[0] -= + A[9]*b[9];
b[0] -= + A[8]*b[8];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim20_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 20; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (19); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*20+i]);
	for( j=(i+1); j < 20; j++ ) {
		temp = fabs(A[j*20+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 20; ++k)
{
	rk_dim20_swap = A[i*20+k];
	A[i*20+k] = A[indexMax*20+k];
	A[indexMax*20+k] = rk_dim20_swap;
}
	rk_dim20_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = rk_dim20_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*20+i];
	for( j=i+1; j < 20; j++ ) {
		A[j*20+i] = -A[j*20+i]/A[i*20+i];
		for( k=i+1; k < 20; k++ ) {
			A[j*20+k] += A[j*20+i] * A[i*20+k];
		}
		b[j] += A[j*20+i] * b[i];
	}
}
det *= A[399];
det = fabs(det);
acado_solve_dim20_triangular( A, b );
return det;
}

void acado_solve_dim20_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

rk_dim20_bPerm[0] = b[rk_perm[0]];
rk_dim20_bPerm[1] = b[rk_perm[1]];
rk_dim20_bPerm[2] = b[rk_perm[2]];
rk_dim20_bPerm[3] = b[rk_perm[3]];
rk_dim20_bPerm[4] = b[rk_perm[4]];
rk_dim20_bPerm[5] = b[rk_perm[5]];
rk_dim20_bPerm[6] = b[rk_perm[6]];
rk_dim20_bPerm[7] = b[rk_perm[7]];
rk_dim20_bPerm[8] = b[rk_perm[8]];
rk_dim20_bPerm[9] = b[rk_perm[9]];
rk_dim20_bPerm[10] = b[rk_perm[10]];
rk_dim20_bPerm[11] = b[rk_perm[11]];
rk_dim20_bPerm[12] = b[rk_perm[12]];
rk_dim20_bPerm[13] = b[rk_perm[13]];
rk_dim20_bPerm[14] = b[rk_perm[14]];
rk_dim20_bPerm[15] = b[rk_perm[15]];
rk_dim20_bPerm[16] = b[rk_perm[16]];
rk_dim20_bPerm[17] = b[rk_perm[17]];
rk_dim20_bPerm[18] = b[rk_perm[18]];
rk_dim20_bPerm[19] = b[rk_perm[19]];
rk_dim20_bPerm[1] += A[20]*rk_dim20_bPerm[0];

rk_dim20_bPerm[2] += A[40]*rk_dim20_bPerm[0];
rk_dim20_bPerm[2] += A[41]*rk_dim20_bPerm[1];

rk_dim20_bPerm[3] += A[60]*rk_dim20_bPerm[0];
rk_dim20_bPerm[3] += A[61]*rk_dim20_bPerm[1];
rk_dim20_bPerm[3] += A[62]*rk_dim20_bPerm[2];

rk_dim20_bPerm[4] += A[80]*rk_dim20_bPerm[0];
rk_dim20_bPerm[4] += A[81]*rk_dim20_bPerm[1];
rk_dim20_bPerm[4] += A[82]*rk_dim20_bPerm[2];
rk_dim20_bPerm[4] += A[83]*rk_dim20_bPerm[3];

rk_dim20_bPerm[5] += A[100]*rk_dim20_bPerm[0];
rk_dim20_bPerm[5] += A[101]*rk_dim20_bPerm[1];
rk_dim20_bPerm[5] += A[102]*rk_dim20_bPerm[2];
rk_dim20_bPerm[5] += A[103]*rk_dim20_bPerm[3];
rk_dim20_bPerm[5] += A[104]*rk_dim20_bPerm[4];

rk_dim20_bPerm[6] += A[120]*rk_dim20_bPerm[0];
rk_dim20_bPerm[6] += A[121]*rk_dim20_bPerm[1];
rk_dim20_bPerm[6] += A[122]*rk_dim20_bPerm[2];
rk_dim20_bPerm[6] += A[123]*rk_dim20_bPerm[3];
rk_dim20_bPerm[6] += A[124]*rk_dim20_bPerm[4];
rk_dim20_bPerm[6] += A[125]*rk_dim20_bPerm[5];

rk_dim20_bPerm[7] += A[140]*rk_dim20_bPerm[0];
rk_dim20_bPerm[7] += A[141]*rk_dim20_bPerm[1];
rk_dim20_bPerm[7] += A[142]*rk_dim20_bPerm[2];
rk_dim20_bPerm[7] += A[143]*rk_dim20_bPerm[3];
rk_dim20_bPerm[7] += A[144]*rk_dim20_bPerm[4];
rk_dim20_bPerm[7] += A[145]*rk_dim20_bPerm[5];
rk_dim20_bPerm[7] += A[146]*rk_dim20_bPerm[6];

rk_dim20_bPerm[8] += A[160]*rk_dim20_bPerm[0];
rk_dim20_bPerm[8] += A[161]*rk_dim20_bPerm[1];
rk_dim20_bPerm[8] += A[162]*rk_dim20_bPerm[2];
rk_dim20_bPerm[8] += A[163]*rk_dim20_bPerm[3];
rk_dim20_bPerm[8] += A[164]*rk_dim20_bPerm[4];
rk_dim20_bPerm[8] += A[165]*rk_dim20_bPerm[5];
rk_dim20_bPerm[8] += A[166]*rk_dim20_bPerm[6];
rk_dim20_bPerm[8] += A[167]*rk_dim20_bPerm[7];

rk_dim20_bPerm[9] += A[180]*rk_dim20_bPerm[0];
rk_dim20_bPerm[9] += A[181]*rk_dim20_bPerm[1];
rk_dim20_bPerm[9] += A[182]*rk_dim20_bPerm[2];
rk_dim20_bPerm[9] += A[183]*rk_dim20_bPerm[3];
rk_dim20_bPerm[9] += A[184]*rk_dim20_bPerm[4];
rk_dim20_bPerm[9] += A[185]*rk_dim20_bPerm[5];
rk_dim20_bPerm[9] += A[186]*rk_dim20_bPerm[6];
rk_dim20_bPerm[9] += A[187]*rk_dim20_bPerm[7];
rk_dim20_bPerm[9] += A[188]*rk_dim20_bPerm[8];

rk_dim20_bPerm[10] += A[200]*rk_dim20_bPerm[0];
rk_dim20_bPerm[10] += A[201]*rk_dim20_bPerm[1];
rk_dim20_bPerm[10] += A[202]*rk_dim20_bPerm[2];
rk_dim20_bPerm[10] += A[203]*rk_dim20_bPerm[3];
rk_dim20_bPerm[10] += A[204]*rk_dim20_bPerm[4];
rk_dim20_bPerm[10] += A[205]*rk_dim20_bPerm[5];
rk_dim20_bPerm[10] += A[206]*rk_dim20_bPerm[6];
rk_dim20_bPerm[10] += A[207]*rk_dim20_bPerm[7];
rk_dim20_bPerm[10] += A[208]*rk_dim20_bPerm[8];
rk_dim20_bPerm[10] += A[209]*rk_dim20_bPerm[9];

rk_dim20_bPerm[11] += A[220]*rk_dim20_bPerm[0];
rk_dim20_bPerm[11] += A[221]*rk_dim20_bPerm[1];
rk_dim20_bPerm[11] += A[222]*rk_dim20_bPerm[2];
rk_dim20_bPerm[11] += A[223]*rk_dim20_bPerm[3];
rk_dim20_bPerm[11] += A[224]*rk_dim20_bPerm[4];
rk_dim20_bPerm[11] += A[225]*rk_dim20_bPerm[5];
rk_dim20_bPerm[11] += A[226]*rk_dim20_bPerm[6];
rk_dim20_bPerm[11] += A[227]*rk_dim20_bPerm[7];
rk_dim20_bPerm[11] += A[228]*rk_dim20_bPerm[8];
rk_dim20_bPerm[11] += A[229]*rk_dim20_bPerm[9];
rk_dim20_bPerm[11] += A[230]*rk_dim20_bPerm[10];

rk_dim20_bPerm[12] += A[240]*rk_dim20_bPerm[0];
rk_dim20_bPerm[12] += A[241]*rk_dim20_bPerm[1];
rk_dim20_bPerm[12] += A[242]*rk_dim20_bPerm[2];
rk_dim20_bPerm[12] += A[243]*rk_dim20_bPerm[3];
rk_dim20_bPerm[12] += A[244]*rk_dim20_bPerm[4];
rk_dim20_bPerm[12] += A[245]*rk_dim20_bPerm[5];
rk_dim20_bPerm[12] += A[246]*rk_dim20_bPerm[6];
rk_dim20_bPerm[12] += A[247]*rk_dim20_bPerm[7];
rk_dim20_bPerm[12] += A[248]*rk_dim20_bPerm[8];
rk_dim20_bPerm[12] += A[249]*rk_dim20_bPerm[9];
rk_dim20_bPerm[12] += A[250]*rk_dim20_bPerm[10];
rk_dim20_bPerm[12] += A[251]*rk_dim20_bPerm[11];

rk_dim20_bPerm[13] += A[260]*rk_dim20_bPerm[0];
rk_dim20_bPerm[13] += A[261]*rk_dim20_bPerm[1];
rk_dim20_bPerm[13] += A[262]*rk_dim20_bPerm[2];
rk_dim20_bPerm[13] += A[263]*rk_dim20_bPerm[3];
rk_dim20_bPerm[13] += A[264]*rk_dim20_bPerm[4];
rk_dim20_bPerm[13] += A[265]*rk_dim20_bPerm[5];
rk_dim20_bPerm[13] += A[266]*rk_dim20_bPerm[6];
rk_dim20_bPerm[13] += A[267]*rk_dim20_bPerm[7];
rk_dim20_bPerm[13] += A[268]*rk_dim20_bPerm[8];
rk_dim20_bPerm[13] += A[269]*rk_dim20_bPerm[9];
rk_dim20_bPerm[13] += A[270]*rk_dim20_bPerm[10];
rk_dim20_bPerm[13] += A[271]*rk_dim20_bPerm[11];
rk_dim20_bPerm[13] += A[272]*rk_dim20_bPerm[12];

rk_dim20_bPerm[14] += A[280]*rk_dim20_bPerm[0];
rk_dim20_bPerm[14] += A[281]*rk_dim20_bPerm[1];
rk_dim20_bPerm[14] += A[282]*rk_dim20_bPerm[2];
rk_dim20_bPerm[14] += A[283]*rk_dim20_bPerm[3];
rk_dim20_bPerm[14] += A[284]*rk_dim20_bPerm[4];
rk_dim20_bPerm[14] += A[285]*rk_dim20_bPerm[5];
rk_dim20_bPerm[14] += A[286]*rk_dim20_bPerm[6];
rk_dim20_bPerm[14] += A[287]*rk_dim20_bPerm[7];
rk_dim20_bPerm[14] += A[288]*rk_dim20_bPerm[8];
rk_dim20_bPerm[14] += A[289]*rk_dim20_bPerm[9];
rk_dim20_bPerm[14] += A[290]*rk_dim20_bPerm[10];
rk_dim20_bPerm[14] += A[291]*rk_dim20_bPerm[11];
rk_dim20_bPerm[14] += A[292]*rk_dim20_bPerm[12];
rk_dim20_bPerm[14] += A[293]*rk_dim20_bPerm[13];

rk_dim20_bPerm[15] += A[300]*rk_dim20_bPerm[0];
rk_dim20_bPerm[15] += A[301]*rk_dim20_bPerm[1];
rk_dim20_bPerm[15] += A[302]*rk_dim20_bPerm[2];
rk_dim20_bPerm[15] += A[303]*rk_dim20_bPerm[3];
rk_dim20_bPerm[15] += A[304]*rk_dim20_bPerm[4];
rk_dim20_bPerm[15] += A[305]*rk_dim20_bPerm[5];
rk_dim20_bPerm[15] += A[306]*rk_dim20_bPerm[6];
rk_dim20_bPerm[15] += A[307]*rk_dim20_bPerm[7];
rk_dim20_bPerm[15] += A[308]*rk_dim20_bPerm[8];
rk_dim20_bPerm[15] += A[309]*rk_dim20_bPerm[9];
rk_dim20_bPerm[15] += A[310]*rk_dim20_bPerm[10];
rk_dim20_bPerm[15] += A[311]*rk_dim20_bPerm[11];
rk_dim20_bPerm[15] += A[312]*rk_dim20_bPerm[12];
rk_dim20_bPerm[15] += A[313]*rk_dim20_bPerm[13];
rk_dim20_bPerm[15] += A[314]*rk_dim20_bPerm[14];

rk_dim20_bPerm[16] += A[320]*rk_dim20_bPerm[0];
rk_dim20_bPerm[16] += A[321]*rk_dim20_bPerm[1];
rk_dim20_bPerm[16] += A[322]*rk_dim20_bPerm[2];
rk_dim20_bPerm[16] += A[323]*rk_dim20_bPerm[3];
rk_dim20_bPerm[16] += A[324]*rk_dim20_bPerm[4];
rk_dim20_bPerm[16] += A[325]*rk_dim20_bPerm[5];
rk_dim20_bPerm[16] += A[326]*rk_dim20_bPerm[6];
rk_dim20_bPerm[16] += A[327]*rk_dim20_bPerm[7];
rk_dim20_bPerm[16] += A[328]*rk_dim20_bPerm[8];
rk_dim20_bPerm[16] += A[329]*rk_dim20_bPerm[9];
rk_dim20_bPerm[16] += A[330]*rk_dim20_bPerm[10];
rk_dim20_bPerm[16] += A[331]*rk_dim20_bPerm[11];
rk_dim20_bPerm[16] += A[332]*rk_dim20_bPerm[12];
rk_dim20_bPerm[16] += A[333]*rk_dim20_bPerm[13];
rk_dim20_bPerm[16] += A[334]*rk_dim20_bPerm[14];
rk_dim20_bPerm[16] += A[335]*rk_dim20_bPerm[15];

rk_dim20_bPerm[17] += A[340]*rk_dim20_bPerm[0];
rk_dim20_bPerm[17] += A[341]*rk_dim20_bPerm[1];
rk_dim20_bPerm[17] += A[342]*rk_dim20_bPerm[2];
rk_dim20_bPerm[17] += A[343]*rk_dim20_bPerm[3];
rk_dim20_bPerm[17] += A[344]*rk_dim20_bPerm[4];
rk_dim20_bPerm[17] += A[345]*rk_dim20_bPerm[5];
rk_dim20_bPerm[17] += A[346]*rk_dim20_bPerm[6];
rk_dim20_bPerm[17] += A[347]*rk_dim20_bPerm[7];
rk_dim20_bPerm[17] += A[348]*rk_dim20_bPerm[8];
rk_dim20_bPerm[17] += A[349]*rk_dim20_bPerm[9];
rk_dim20_bPerm[17] += A[350]*rk_dim20_bPerm[10];
rk_dim20_bPerm[17] += A[351]*rk_dim20_bPerm[11];
rk_dim20_bPerm[17] += A[352]*rk_dim20_bPerm[12];
rk_dim20_bPerm[17] += A[353]*rk_dim20_bPerm[13];
rk_dim20_bPerm[17] += A[354]*rk_dim20_bPerm[14];
rk_dim20_bPerm[17] += A[355]*rk_dim20_bPerm[15];
rk_dim20_bPerm[17] += A[356]*rk_dim20_bPerm[16];

rk_dim20_bPerm[18] += A[360]*rk_dim20_bPerm[0];
rk_dim20_bPerm[18] += A[361]*rk_dim20_bPerm[1];
rk_dim20_bPerm[18] += A[362]*rk_dim20_bPerm[2];
rk_dim20_bPerm[18] += A[363]*rk_dim20_bPerm[3];
rk_dim20_bPerm[18] += A[364]*rk_dim20_bPerm[4];
rk_dim20_bPerm[18] += A[365]*rk_dim20_bPerm[5];
rk_dim20_bPerm[18] += A[366]*rk_dim20_bPerm[6];
rk_dim20_bPerm[18] += A[367]*rk_dim20_bPerm[7];
rk_dim20_bPerm[18] += A[368]*rk_dim20_bPerm[8];
rk_dim20_bPerm[18] += A[369]*rk_dim20_bPerm[9];
rk_dim20_bPerm[18] += A[370]*rk_dim20_bPerm[10];
rk_dim20_bPerm[18] += A[371]*rk_dim20_bPerm[11];
rk_dim20_bPerm[18] += A[372]*rk_dim20_bPerm[12];
rk_dim20_bPerm[18] += A[373]*rk_dim20_bPerm[13];
rk_dim20_bPerm[18] += A[374]*rk_dim20_bPerm[14];
rk_dim20_bPerm[18] += A[375]*rk_dim20_bPerm[15];
rk_dim20_bPerm[18] += A[376]*rk_dim20_bPerm[16];
rk_dim20_bPerm[18] += A[377]*rk_dim20_bPerm[17];

rk_dim20_bPerm[19] += A[380]*rk_dim20_bPerm[0];
rk_dim20_bPerm[19] += A[381]*rk_dim20_bPerm[1];
rk_dim20_bPerm[19] += A[382]*rk_dim20_bPerm[2];
rk_dim20_bPerm[19] += A[383]*rk_dim20_bPerm[3];
rk_dim20_bPerm[19] += A[384]*rk_dim20_bPerm[4];
rk_dim20_bPerm[19] += A[385]*rk_dim20_bPerm[5];
rk_dim20_bPerm[19] += A[386]*rk_dim20_bPerm[6];
rk_dim20_bPerm[19] += A[387]*rk_dim20_bPerm[7];
rk_dim20_bPerm[19] += A[388]*rk_dim20_bPerm[8];
rk_dim20_bPerm[19] += A[389]*rk_dim20_bPerm[9];
rk_dim20_bPerm[19] += A[390]*rk_dim20_bPerm[10];
rk_dim20_bPerm[19] += A[391]*rk_dim20_bPerm[11];
rk_dim20_bPerm[19] += A[392]*rk_dim20_bPerm[12];
rk_dim20_bPerm[19] += A[393]*rk_dim20_bPerm[13];
rk_dim20_bPerm[19] += A[394]*rk_dim20_bPerm[14];
rk_dim20_bPerm[19] += A[395]*rk_dim20_bPerm[15];
rk_dim20_bPerm[19] += A[396]*rk_dim20_bPerm[16];
rk_dim20_bPerm[19] += A[397]*rk_dim20_bPerm[17];
rk_dim20_bPerm[19] += A[398]*rk_dim20_bPerm[18];


acado_solve_dim20_triangular( A, rk_dim20_bPerm );
b[0] = rk_dim20_bPerm[0];
b[1] = rk_dim20_bPerm[1];
b[2] = rk_dim20_bPerm[2];
b[3] = rk_dim20_bPerm[3];
b[4] = rk_dim20_bPerm[4];
b[5] = rk_dim20_bPerm[5];
b[6] = rk_dim20_bPerm[6];
b[7] = rk_dim20_bPerm[7];
b[8] = rk_dim20_bPerm[8];
b[9] = rk_dim20_bPerm[9];
b[10] = rk_dim20_bPerm[10];
b[11] = rk_dim20_bPerm[11];
b[12] = rk_dim20_bPerm[12];
b[13] = rk_dim20_bPerm[13];
b[14] = rk_dim20_bPerm[14];
b[15] = rk_dim20_bPerm[15];
b[16] = rk_dim20_bPerm[16];
b[17] = rk_dim20_bPerm[17];
b[18] = rk_dim20_bPerm[18];
b[19] = rk_dim20_bPerm[19];
}



/** Matrix of size: 2 x 2 (row major format) */
static const real_t acado_Ah_mat[ 4 ] = 
{ 2.5000000000000001e-02, 5.3867513459481292e-02, 
-3.8675134594812867e-03, 2.5000000000000001e-02 };


/* Fixed step size:0.1 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

rk_ttt = 0.0000000000000000e+00;
rk_xxx[10] = rk_eta[150];
rk_xxx[11] = rk_eta[151];
rk_xxx[12] = rk_eta[152];
rk_xxx[13] = rk_eta[153];
rk_xxx[14] = rk_eta[154];
rk_xxx[15] = rk_eta[155];
rk_xxx[16] = rk_eta[156];
rk_xxx[17] = rk_eta[157];
rk_xxx[18] = rk_eta[158];
rk_xxx[19] = rk_eta[159];
rk_xxx[20] = rk_eta[160];
rk_xxx[21] = rk_eta[161];
rk_xxx[22] = rk_eta[162];
rk_xxx[23] = rk_eta[163];

for (run = 0; run < 1; ++run)
{
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 10; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 140 ]) );
for (j = 0; j < 10; ++j)
{
tmp_index1 = (run1 * 10) + (j);
rk_A[tmp_index1 * 20] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14)];
rk_A[tmp_index1 * 20 + 1] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 1)];
rk_A[tmp_index1 * 20 + 2] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 2)];
rk_A[tmp_index1 * 20 + 3] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 3)];
rk_A[tmp_index1 * 20 + 4] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 4)];
rk_A[tmp_index1 * 20 + 5] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 5)];
rk_A[tmp_index1 * 20 + 6] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 6)];
rk_A[tmp_index1 * 20 + 7] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 7)];
rk_A[tmp_index1 * 20 + 8] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 8)];
rk_A[tmp_index1 * 20 + 9] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 9)];
if( 0 == run1 ) rk_A[(tmp_index1 * 20) + (j)] -= 1.0000000000000000e+00;
rk_A[tmp_index1 * 20 + 10] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14)];
rk_A[tmp_index1 * 20 + 11] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 1)];
rk_A[tmp_index1 * 20 + 12] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 2)];
rk_A[tmp_index1 * 20 + 13] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 3)];
rk_A[tmp_index1 * 20 + 14] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 4)];
rk_A[tmp_index1 * 20 + 15] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 5)];
rk_A[tmp_index1 * 20 + 16] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 6)];
rk_A[tmp_index1 * 20 + 17] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 7)];
rk_A[tmp_index1 * 20 + 18] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 8)];
rk_A[tmp_index1 * 20 + 19] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 9)];
if( 1 == run1 ) rk_A[(tmp_index1 * 20) + (j + 10)] -= 1.0000000000000000e+00;
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 10] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 10 + 1] = rk_kkk[run1 + 2] - rk_rhsTemp[1];
rk_b[run1 * 10 + 2] = rk_kkk[run1 + 4] - rk_rhsTemp[2];
rk_b[run1 * 10 + 3] = rk_kkk[run1 + 6] - rk_rhsTemp[3];
rk_b[run1 * 10 + 4] = rk_kkk[run1 + 8] - rk_rhsTemp[4];
rk_b[run1 * 10 + 5] = rk_kkk[run1 + 10] - rk_rhsTemp[5];
rk_b[run1 * 10 + 6] = rk_kkk[run1 + 12] - rk_rhsTemp[6];
rk_b[run1 * 10 + 7] = rk_kkk[run1 + 14] - rk_rhsTemp[7];
rk_b[run1 * 10 + 8] = rk_kkk[run1 + 16] - rk_rhsTemp[8];
rk_b[run1 * 10 + 9] = rk_kkk[run1 + 18] - rk_rhsTemp[9];
}
det = acado_solve_dim20_system( rk_A, rk_b, rk_dim20_perm );
for (j = 0; j < 2; ++j)
{
rk_kkk[j] += rk_b[j * 10];
rk_kkk[j + 2] += rk_b[j * 10 + 1];
rk_kkk[j + 4] += rk_b[j * 10 + 2];
rk_kkk[j + 6] += rk_b[j * 10 + 3];
rk_kkk[j + 8] += rk_b[j * 10 + 4];
rk_kkk[j + 10] += rk_b[j * 10 + 5];
rk_kkk[j + 12] += rk_b[j * 10 + 6];
rk_kkk[j + 14] += rk_b[j * 10 + 7];
rk_kkk[j + 16] += rk_b[j * 10 + 8];
rk_kkk[j + 18] += rk_b[j * 10 + 9];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 10; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 10] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 10 + 1] = rk_kkk[run1 + 2] - rk_rhsTemp[1];
rk_b[run1 * 10 + 2] = rk_kkk[run1 + 4] - rk_rhsTemp[2];
rk_b[run1 * 10 + 3] = rk_kkk[run1 + 6] - rk_rhsTemp[3];
rk_b[run1 * 10 + 4] = rk_kkk[run1 + 8] - rk_rhsTemp[4];
rk_b[run1 * 10 + 5] = rk_kkk[run1 + 10] - rk_rhsTemp[5];
rk_b[run1 * 10 + 6] = rk_kkk[run1 + 12] - rk_rhsTemp[6];
rk_b[run1 * 10 + 7] = rk_kkk[run1 + 14] - rk_rhsTemp[7];
rk_b[run1 * 10 + 8] = rk_kkk[run1 + 16] - rk_rhsTemp[8];
rk_b[run1 * 10 + 9] = rk_kkk[run1 + 18] - rk_rhsTemp[9];
}
acado_solve_dim20_system_reuse( rk_A, rk_b, rk_dim20_perm );
for (j = 0; j < 2; ++j)
{
rk_kkk[j] += rk_b[j * 10];
rk_kkk[j + 2] += rk_b[j * 10 + 1];
rk_kkk[j + 4] += rk_b[j * 10 + 2];
rk_kkk[j + 6] += rk_b[j * 10 + 3];
rk_kkk[j + 8] += rk_b[j * 10 + 4];
rk_kkk[j + 10] += rk_b[j * 10 + 5];
rk_kkk[j + 12] += rk_b[j * 10 + 6];
rk_kkk[j + 14] += rk_b[j * 10 + 7];
rk_kkk[j + 16] += rk_b[j * 10 + 8];
rk_kkk[j + 18] += rk_b[j * 10 + 9];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 10; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 140 ]) );
for (j = 0; j < 10; ++j)
{
tmp_index1 = (run1 * 10) + (j);
rk_A[tmp_index1 * 20] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14)];
rk_A[tmp_index1 * 20 + 1] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 1)];
rk_A[tmp_index1 * 20 + 2] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 2)];
rk_A[tmp_index1 * 20 + 3] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 3)];
rk_A[tmp_index1 * 20 + 4] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 4)];
rk_A[tmp_index1 * 20 + 5] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 5)];
rk_A[tmp_index1 * 20 + 6] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 6)];
rk_A[tmp_index1 * 20 + 7] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 7)];
rk_A[tmp_index1 * 20 + 8] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 8)];
rk_A[tmp_index1 * 20 + 9] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 9)];
if( 0 == run1 ) rk_A[(tmp_index1 * 20) + (j)] -= 1.0000000000000000e+00;
rk_A[tmp_index1 * 20 + 10] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14)];
rk_A[tmp_index1 * 20 + 11] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 1)];
rk_A[tmp_index1 * 20 + 12] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 2)];
rk_A[tmp_index1 * 20 + 13] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 3)];
rk_A[tmp_index1 * 20 + 14] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 4)];
rk_A[tmp_index1 * 20 + 15] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 5)];
rk_A[tmp_index1 * 20 + 16] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 6)];
rk_A[tmp_index1 * 20 + 17] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 7)];
rk_A[tmp_index1 * 20 + 18] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 8)];
rk_A[tmp_index1 * 20 + 19] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 140) + (j * 14 + 9)];
if( 1 == run1 ) rk_A[(tmp_index1 * 20) + (j + 10)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 10; ++run1)
{
for (i = 0; i < 2; ++i)
{
rk_b[i * 10] = - rk_diffsTemp2[(i * 140) + (run1)];
rk_b[i * 10 + 1] = - rk_diffsTemp2[(i * 140) + (run1 + 14)];
rk_b[i * 10 + 2] = - rk_diffsTemp2[(i * 140) + (run1 + 28)];
rk_b[i * 10 + 3] = - rk_diffsTemp2[(i * 140) + (run1 + 42)];
rk_b[i * 10 + 4] = - rk_diffsTemp2[(i * 140) + (run1 + 56)];
rk_b[i * 10 + 5] = - rk_diffsTemp2[(i * 140) + (run1 + 70)];
rk_b[i * 10 + 6] = - rk_diffsTemp2[(i * 140) + (run1 + 84)];
rk_b[i * 10 + 7] = - rk_diffsTemp2[(i * 140) + (run1 + 98)];
rk_b[i * 10 + 8] = - rk_diffsTemp2[(i * 140) + (run1 + 112)];
rk_b[i * 10 + 9] = - rk_diffsTemp2[(i * 140) + (run1 + 126)];
}
if( 0 == run1 ) {
det = acado_solve_dim20_system( rk_A, rk_b, rk_dim20_perm );
}
 else {
acado_solve_dim20_system_reuse( rk_A, rk_b, rk_dim20_perm );
}
for (i = 0; i < 2; ++i)
{
rk_diffK[i] = rk_b[i * 10];
rk_diffK[i + 2] = rk_b[i * 10 + 1];
rk_diffK[i + 4] = rk_b[i * 10 + 2];
rk_diffK[i + 6] = rk_b[i * 10 + 3];
rk_diffK[i + 8] = rk_b[i * 10 + 4];
rk_diffK[i + 10] = rk_b[i * 10 + 5];
rk_diffK[i + 12] = rk_b[i * 10 + 6];
rk_diffK[i + 14] = rk_b[i * 10 + 7];
rk_diffK[i + 16] = rk_b[i * 10 + 8];
rk_diffK[i + 18] = rk_b[i * 10 + 9];
}
for (i = 0; i < 10; ++i)
{
rk_diffsNew2[(i * 14) + (run1)] = (i == run1-0);
rk_diffsNew2[(i * 14) + (run1)] += + rk_diffK[i * 2]*(real_t)5.0000000000000003e-02 + rk_diffK[i * 2 + 1]*(real_t)5.0000000000000003e-02;
}
}
for (run1 = 0; run1 < 4; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 10; ++j)
{
tmp_index1 = (i * 10) + (j);
tmp_index2 = (run1) + (j * 14);
rk_b[tmp_index1] = - rk_diffsTemp2[(i * 140) + (tmp_index2 + 10)];
}
}
acado_solve_dim20_system_reuse( rk_A, rk_b, rk_dim20_perm );
for (i = 0; i < 2; ++i)
{
rk_diffK[i] = rk_b[i * 10];
rk_diffK[i + 2] = rk_b[i * 10 + 1];
rk_diffK[i + 4] = rk_b[i * 10 + 2];
rk_diffK[i + 6] = rk_b[i * 10 + 3];
rk_diffK[i + 8] = rk_b[i * 10 + 4];
rk_diffK[i + 10] = rk_b[i * 10 + 5];
rk_diffK[i + 12] = rk_b[i * 10 + 6];
rk_diffK[i + 14] = rk_b[i * 10 + 7];
rk_diffK[i + 16] = rk_b[i * 10 + 8];
rk_diffK[i + 18] = rk_b[i * 10 + 9];
}
for (i = 0; i < 10; ++i)
{
rk_diffsNew2[(i * 14) + (run1 + 10)] = + rk_diffK[i * 2]*(real_t)5.0000000000000003e-02 + rk_diffK[i * 2 + 1]*(real_t)5.0000000000000003e-02;
}
}
rk_eta[0] += + rk_kkk[0]*(real_t)5.0000000000000003e-02 + rk_kkk[1]*(real_t)5.0000000000000003e-02;
rk_eta[1] += + rk_kkk[2]*(real_t)5.0000000000000003e-02 + rk_kkk[3]*(real_t)5.0000000000000003e-02;
rk_eta[2] += + rk_kkk[4]*(real_t)5.0000000000000003e-02 + rk_kkk[5]*(real_t)5.0000000000000003e-02;
rk_eta[3] += + rk_kkk[6]*(real_t)5.0000000000000003e-02 + rk_kkk[7]*(real_t)5.0000000000000003e-02;
rk_eta[4] += + rk_kkk[8]*(real_t)5.0000000000000003e-02 + rk_kkk[9]*(real_t)5.0000000000000003e-02;
rk_eta[5] += + rk_kkk[10]*(real_t)5.0000000000000003e-02 + rk_kkk[11]*(real_t)5.0000000000000003e-02;
rk_eta[6] += + rk_kkk[12]*(real_t)5.0000000000000003e-02 + rk_kkk[13]*(real_t)5.0000000000000003e-02;
rk_eta[7] += + rk_kkk[14]*(real_t)5.0000000000000003e-02 + rk_kkk[15]*(real_t)5.0000000000000003e-02;
rk_eta[8] += + rk_kkk[16]*(real_t)5.0000000000000003e-02 + rk_kkk[17]*(real_t)5.0000000000000003e-02;
rk_eta[9] += + rk_kkk[18]*(real_t)5.0000000000000003e-02 + rk_kkk[19]*(real_t)5.0000000000000003e-02;
for (i = 0; i < 10; ++i)
{
for (j = 0; j < 10; ++j)
{
tmp_index2 = (j) + (i * 10);
rk_eta[tmp_index2 + 10] = rk_diffsNew2[(i * 14) + (j)];
}
for (j = 0; j < 4; ++j)
{
tmp_index2 = (j) + (i * 4);
rk_eta[tmp_index2 + 110] = rk_diffsNew2[(i * 14) + (j + 10)];
}
}
resetIntegrator = 0;
rk_ttt += 1.0000000000000000e+00;
}
for (i = 0; i < 10; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



