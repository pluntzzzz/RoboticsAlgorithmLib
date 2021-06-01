/**
 * @brief			Description: Algorithm module of robotics, according to the
 *					book[modern robotics : mechanics, planning, and control].
 * @file:			RobotAlgorithmModule.c
 * @author:			Brian
 * @date:			2019/03/01 12:23
 * Copyright(c) 	2019 Brian. All rights reserved.
 *					  
 * Contact 			https://blog.csdn.net/Galaxy_Robot
 * @note:     
 *@warning:
*/

#include "RobotAlgorithmModule.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


////matrix exponential为李代数中的概念：矩阵指数，详见知乎：https://www.zhihu.com/question/356466246/answer/931315125
////矩阵指数定义为：exp(A*t)的泰勒展开，实际上就是机器人动力学与控制-霍伟，书中第7页欧拉公式。矩阵指数其实等价于旋转矩阵。
////SO(3)为正交基组成的3*3矩阵，SE(3)为SO(3)+R3 (旋转+平移)组成的4*4矩阵。
////MatrixExp6,MatrixLog6是一组对偶方法，即把SE(3)与T4*4相互转换的函数。

/**
*@brief					Description:use GrublersFormula calculate The number of degrees of 
*						freedom of a mechanism with links and joints.
*@param[in]		N		the number of links( including the ground ).
*@param[in]		m		the number of degrees of freedom of a rigid body.
*						(m = 3 for planar mechanisms and m = 6 for spatial mechanisms).
*@param[in]		J		the number of joints.
*@param[in]		f		the number of freedoms provided by each joint..
*@return				The number of degrees of freedom of a mechanism
*@note:					This formula holds only if all joint constraints are independent.
*						If they are not independent then the formula provides a lower 
*						bound on the number of degrees of freedom.
*@warning:
*/
int GrublersFormula(int m, int N, int J, int *f) {
    int i;
    int dof;
    int c = 0;
    for (i = 1; i <= J; i++) {
        c = c + f[i - 1];
    }
    dof = m * (N - 1 - J) + c;
    return dof;
}

/**
*@brief Description: Make  matrix b equal to  matrix a.
*@param[in]			a		a 3 x 3  matrix.
*@param[out]		b		result,b=a.
*@return		No return value.
*@note:
*@warning:
*/
void Matrix3Equal(double a[][3], double b[][3]) {
    int i;
    int j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            b[i][j] = a[i][j];
        }
    }
    return;
}

/**
*@brief Description: Make  matrix b equal to  matrix a.
*@param[in]			a		a 4 x 4  matrix.
*@param[out]		b		result,b=a.
*@return		No return value.
*@note:
*@warning:
*/
////把A矩阵传给B矩阵
void Matrix4Equal(double a[][4], double b[][4]) {
    int i;
    int j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            b[i][j] = a[i][j];
        }
    }
    return;
}


/**
*@brief Description:Calculate a 3 x 3  matrix add a 3 x 3  matrix.
*@param[in]		a		a 3 x 3  matrix.
*@param[in]		b		a 3 x 3  matrix.
*@param[out]	c		result of a+b.
*@return		No return value.
*@note:
*@warning:
*/
void Matrix3Add(double a[][3], double b[][3], double c[][3]) {
    int i;
    int j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            c[i][j] = a[i][j] + b[i][j];
        }
    }
    return;
}

/**
*@brief Description:Calculate a 4 x 4  matrix add a 4 x 4  matrix.
*@param[in]		a		a 4 x 4  matrix.
*@param[in]		b		a 4 x 4  matrix.
*@param[out]	c		result of a+b.
*@return		No return value.
*@note:
*@warning:
*/
void Matrix4Add(double a[][4], double b[][4], double c[][4]) {
    int i;
    int j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            c[i][j] = a[i][j] + b[i][j];
        }
    }
    return;
}

/**
*@brief Description:Calculate a 3 x 3  matrix Subtract a 3 x 3  matrix.
*@param[in]		a		a 3 x 3  matrix.
*@param[in]		b		a 3 x 3  matrix.
*@param[out]	c		result of a-b.
*@return		No return value.
*@note:
*@warning:
*/
void Matrix3Sub(double a[][3], double b[][3], double c[][3]) {
    int i;
    int j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            c[i][j] = a[i][j] - b[i][j];
        }
    }
    return;
}

/**
*@brief Description:Calculate a 4 x 4  matrix Subtract a 4 x 4  matrix.
*@param[in]		a		a 4 x 4  matrix.
*@param[in]		b		a 4 x 4  matrix.
*@param[out]	c		result of a-b.
*@return		No return value.
*@note:
*@warning:
*/
void Matrix4Sub(double a[][4], double b[][4], double c[][4]) {
    int i;
    int j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            c[i][j] = a[i][j] - b[i][j];
        }
    }
    return;
}

/**
*@brief Description: Calculate two 3 x 3  matrix multiplication. 
*@param[in]		a		a 3 x 3  matrix.		
*@param[in]		b		a 3 x 3  matrix.	
*@param[out]	c		result of a*b.
*@return		No return value.
*@note:
*@warning:
*/
void Matrix3Mult(double a[][3], double b[][3], double c[][3]) {
    int i;
    int j;
    int k;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            c[i][j] = 0.0;
            for (k = 0; k < 3; k++) {
                c[i][j] = c[i][j] + a[i][k] * b[k][j];
            }

        }
    }
    return;
}

/**
*@brief Description: Calculate two 4 x 4  matrix multiplication.
*@param[in]		a		a 4 x 4  matrix.
*@param[in]		b		a 4 x 4  matrix.
*@param[out]	c		result of a*b.
*@return		No return value.
*@note:
*@warning:
*/
void Matrix4Mult(double a[][4], double b[][4], double c[][4]) {
    int i;
    int j;
    int k;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            c[i][j] = 0.0;
            for (k = 0; k < 4; k++) {
                c[i][j] = c[i][j] + a[i][k] * b[k][j];
            }

        }
    }
    return;
}


/**
*@brief Description: Calculate  3 x 3  matrix multiply a value.
*@param[in]		a		a 3 x 3  matrix.
*@param[in]		Value	a scalar value.
*@param[out]	c		result of a*Value.
*@return		No return value.
*@note:
*@warning:
*/
void Matrix3MultValue(double a[][3], double Value, double c[][3]) {
    int i;
    int j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            c[i][j] = a[i][j] * Value;
        }
    }
    return;
}

/**
*@brief Description: Calculate  4 x 4  matrix multiply a value.
*@param[in]		a		a 4 x 4  matrix.
*@param[in]		Value	a scalar value.
*@param[out]	c		result of a*Value.
*@return		No return value.
*@note:
*@warning:
*/
////矩阵乘一个常数
void Matrix4MultValue(double a[][4], double Value, double c[][4]) {
    int i;
    int j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            c[i][j] = a[i][j] * Value;
        }
    }
    return;
}


/**
*@brief Description:Computes the result of a 3 x 3 Matrix multiply a 3-vector.
*@param[in]		R			a 3 x 3 Matrix.
*@param[in]		vec1		an input of 3-vector.
*@param[out]	vec2		the output result of 3-vector.
*@return		No return value.
*@note:
*@warning:
*/
void Matrix3MultVec(double R[3][3], double vec1[3], double vec2[3]) {
    int i;
    int j;
    for (i = 0; i < 3; i++) {
        vec2[i] = 0;
        for (j = 0; j < 3; j++) {
            vec2[i] = vec2[i] + R[i][j] * vec1[j];
        }
    }
    return;
}

/**
*@brief Description:Computes a 3-vector add a 3-vector.
*@param[in]		vec1		a 3-vector.
*@param[in]		vec2		a 3-vector.
*@param[out]	vec3		result of vec1 + vec2;
*@return		No return value.
*@note:
*@warning:
*/
void Vec3Add(double vec1[3], double vec2[3], double vec3[3]) {
    vec3[0] = vec1[0] + vec2[0];
    vec3[1] = vec1[1] + vec2[1];
    vec3[2] = vec1[2] + vec2[2];
    return;
}


/**
*@brief Description:Computes result of two 3-vector cross product.
*@param[in]		vec1		a first 3-vector.
*@param[in]		vec2		a first 3-vector.
*@param[out]	vec3		result of vec1 cross product vec2.
*@return		No return value.
*@note:
*@warning:
*/
void Vec3Cross(double vec1[3], double vec2[3], double vec3[3]) {
    vec3[0] = vec1[1] * vec2[2] - vec2[1] * vec1[2];
    vec3[1] = -(vec1[0] * vec2[2] - vec2[0] * vec1[2]);
    vec3[2] = vec1[0] * vec2[1] - vec2[0] * vec1[1];
    return;
}

/**
*@brief Description:Computes a 3-vector multiply a scalar value.
*@param[in]		vec1			a 3-vector.
*@param[in]		value			a scalar value.
*@param[out]	vec2			result of vec1 * value. 
*@return		No return value.
*@note:
*@warning:
*/
void Vec3MultValue(double vec1[3], double value, double vec2[3]) {
    vec2[0] = vec1[0] * value;
    vec2[1] = vec1[1] * value;
    vec2[2] = vec1[2] * value;
    return;
}

/**
*@brief			Description: Computes the norm of a 3-vector.
*@param[in]		vec			an input 3-vector.
*@return		the norm of a an input 3-vector.
*@note:
*@warning:
*/
////计算三维矢量模长
double Vec3Norm2(double vec[3]) {
    return sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

/**
*@brief			Description: Computes the 2-norm of a n-vector.
*@param[in]		n			dimension of vector.
*@param[in]		vec			an input n-vector.
*@return		the norm of a an input 3-vector.
*@note:
*@warning:
*/
double VecNorm2(int n, double *vec) {
    int i;
    double tmp = 0.0;
    for (i = 0; i < n; i++) {
        tmp = tmp + vec[i] * vec[i];
    }
    if (tmp < 0.0) {
        tmp = 0;
    }
    return sqrt(tmp);
}


/**
*@brief			Copy  matrix from a to  b .
*@param[in]		a		First matrix.
*@param[in]		m		rows of matrix a.
*@param[in]		n		columns of matrix a.
*@param[in]		b		Second matrix.
*@return		No return value.
*@note:
*@warning:
*/
void MatrixCopy(const double *a, int m, int n, double *b) {
    memcpy(b, a, m * n * sizeof(double));
    return;
}

/**
*@brief			Computes Matrix transpose.
*@param[in]		a			a matrix.
*@param[in]		m			rows of matrix a.
*@param[in]		n			columns of matrix a.
*@param[out]	b			transpose of matrix a.
*@return		No return value.
*@note:
*@waring:
*/
////计算矩阵转置
void MatrixT(double *a, int m, int n, double *b) {
    int i, j;
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            b[j * m + i] = a[i * n + j];
        }
    }
    return;
}



/**
*@brief			Computes matrix multiply.
*@param[in]		a		First matrix.
*@param[in]		m		rows of matrix a.
*@param[in]		n		columns of matrix b.
*@param[in]		b		Second matrix.
*@param[in]		l		columns of matrix b.
*@param[out]	c		result of a*b.
*@return		No return value.
*@note:
*@waring:
*/
////计算矩阵A*B
void MatrixMult(const double *a, int m, int n, const double *b, int l, double *c) {
    int i, j, k;
    for (i = 0; i < m; i++) {
        for (j = 0; j < l; j++) {
            c[i * l + j] = 0.0;
            for (k = 0; k < n; k++) {
                c[i * l + j] = c[i * l + j] + a[i * n + k] * b[k * l + j];
            }
        }
    }
    return;
}


/**
* @brief 			Description: Algorithm for Computing the roll-pitch-yaw angles(rotate around fix reference X,Y,Z axis).
* @param[in]		R				Rotation matrix.
* @param[out]		roll			Angles for rotate around fix reference X axis.
* @param[out]		pitch			Angles for rotate around fix reference Y axis.
* @param[out]		yaw				Angles for rotate around fix reference Z axis.
* @return			No return value.
* @note:
*@warning:
*/
////将R转换为roll, pitch, yaw 三个角度
void RotToRPY(double R[3][3], double *roll, double *pitch, double *yaw) {
    if (fabs(1.0 + R[2][0]) < ZERO_ELEMENT) {
        *yaw = 0;
        *pitch = PI / 2.0;
        *roll = atan2(R[0][1], R[1][1]);

    } else if (fabs(1.0 - R[2][0]) < ZERO_ELEMENT) {
        *yaw = 0;
        *pitch = -PI / 2.0;
        *roll = atan2(R[0][1], R[1][1]);
    } else {

        *yaw = atan2(R[1][0], R[0][0]);
        *pitch = atan2(-R[2][0], sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0]));
        *roll = atan2(R[2][1], R[2][2]);
    }
    return;
}

/**
* @brief 			Description: Algorithm for Computing the rotation matrix of the roll-pitch-yaw angles.
* @param[in]		roll			Angles for rotate around fix reference X axis.
* @param[in]		pitch			Angles for rotate around fix reference Y axis.
* @param[in]		yaw				Angles for rotate around fix reference Z axis.
* @param[out]		R				Rotation matrix.
* @return			No return value.
* @note:
*@warning:
*/
////将roll, pitch, yaw 三个角度转换为R
void RPYToRot(double roll, double pitch, double yaw, double R[3][3]) {
    double alpha = yaw;
    double beta = pitch;
    double gamma = roll;
    R[0][0] = cos(alpha) * cos(beta);
    R[0][1] = cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma);
    R[0][2] = cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma);
    R[1][0] = sin(alpha) * cos(beta);
    R[1][1] = sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma);
    R[1][2] = sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma);
    R[2][0] = -sin(beta);
    R[2][1] = cos(beta) * sin(gamma);
    R[2][2] = cos(beta) * cos(gamma);
    return;
}

/**
*@brief			Computes the inverse of the rotation matrix R.
*@param[in]		R		rotation matrix .
*@param[out]	InvR	inverse matrix of R.
*@return		No return value.
*@note			Input R must be a 3 x 3 rotation matrix.
*/
void RotInv(double R[3][3], double InvR[3][3]) {
    int i;
    int j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            InvR[i][j] = R[j][i];
        }
    }
    return;
}

/**
*@brief Description:Returns the 3 x 3 skew-symmetric matrix corresponding to omg.
*@param[in]		omg		a 3-vector.
*@param[out]	so3Mat	the 3 x 3 skew-symmetric matrix corresponding to omg.
*@return		No return value.
*@note:
*@warning:
*/
void VecToso3(double omg[3], double so3Mat[3][3]) {
    so3Mat[0][0] = 0.0;
    so3Mat[0][1] = -omg[2];
    so3Mat[0][2] = omg[1];
    so3Mat[1][0] = omg[2];
    so3Mat[1][1] = 0;
    so3Mat[1][2] = -omg[0];
    so3Mat[2][0] = -omg[1];
    so3Mat[2][1] = omg[0];
    so3Mat[2][2] = 0.0;
    return;
}


/**
*@brief			Description:Returns the 3-vector corresponding to the 3 x 3 skew-symmetric matrix so3mat.
*@param[in]		so3Mat		a 3 x 3 skew-symmetric matrix so3mat.
*@param[out]	omg			the 3-vector.
*@return		No return value.
*@note:
*@warning:
*/
////提取出S(w)矩阵中的非对角线元素，ax,ay,az
void so3ToVec(double so3Mat[3][3], double omg[3]) {
    omg[0] = so3Mat[2][1];
    omg[1] = so3Mat[0][2];
    omg[2] = so3Mat[1][0];
    return;
}


/**
*@brief			Description:Extracts  the unit rotation axis omghat and the corresponding
*				rotation angle theta from exponential coordinates omghat*theta for rotation, expc3.
*@param[in]		expc3			
*@param[out]	omghat		the unit vector of rotation axis .
*@param[out]	theta		rotation angle.
*@return		No return value.
*@note:
*@warning:
*/
////输入的expc3参数是一组正交基的三个基的长度
////输出的omghat其实是z，是单位化后的正交基长度，theta是原正交基向量合成长度
void AxisAng3(double expc3[3], double omghat[3], double *theta) {
    int i;
    int ret = 0;
    *theta = sqrt(expc3[0] * expc3[0] + expc3[1] * expc3[1] + expc3[2] * expc3[2]);
    if (*theta < ZERO_ANGLE) {
        omghat[0] = 0.0;
        omghat[1] = 0.0;
        omghat[2] = 0.0;
        *theta = 0.0;
        return;
    }
    for (i = 0; i < 3; i++) {
        omghat[i] = expc3[i] / (*theta);
    }
    return;
}

/**
*@brief			Description:Computes the rotation matrix R in SO(3) corresponding to
*				the matrix exponential of so3mat in so(3).
*@param[in]		so3Mat		[omghat]*theta,matrix exponential of so3mat in so(3).			
*@param[out]	R			rotation matrix R in SO(3).
*@return		No return value.
*@note:
*@warning:
*/

////通过改变SO（3）的正交基，表示变换后的R
////矩阵指数即matrix exponential，见代码22行注释
////输入参数so3mat为反对称矩阵S(w)
void MatrixExp3(double so3Mat[3][3], double R[3][3]) {
    double omgtheta[3];
    double omghat[3] = {0};
    double theta = 0;
    int ret = 0;
    int i;
    int j;
    double MatI3[3][3] =
            {
                    1, 0, 0,
                    0, 1, 0,
                    0, 0, 1
            };
    so3ToVec(so3Mat, omgtheta);
    AxisAng3(omgtheta, omghat, &theta);
    ////正交基矩阵每个基的模长 = 0，则R不变化
    if (theta < ZERO_ANGLE) {
        Matrix3Equal(MatI3, R);
        return;
    } else {

        //calculate formula(3.51) in book [modern robotics : mechanics,planning,and control]
        //同霍伟书的第七页
        double omgmat[3][3];
        double temp[3][3];
        Matrix3MultValue(so3Mat, 1.0 / theta, omgmat);
        Matrix3Mult(omgmat, omgmat, temp);
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                R[i][j] = MatI3[i][j] + sin(theta) * so3Mat[i][j] / theta + (1 - cos(theta)) * temp[i][j];
            }
        }
    }
    return;
}

/**
*@brief			Description:Computes the matrix logarithm so3mat in so(3) of the rotation matrix R in SO(3).
*@param[in]		R		the rotation matrix.
*@param[out]	so3Mat	matrix logarithm.
*@return		No return value.
*@note:
*@warning:
*/

////计算矩阵对数，详见知乎：https://www.zhihu.com/question/405831993/answer/1409378299
void MatrixLog3(double R[3][3], double so3Mat[3][3]) {
    double omg[3] = {0};
    double acosinput = (R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0;
    if (fabs(acosinput - 1.0) < ZERO_VALUE) {
        memset(so3Mat, 0, 9 * sizeof(double));
    } else if (acosinput <= -1.0) {
        if ((1.0 + R[2][2]) >= ZERO_VALUE) {
            omg[0] = 1.0 / sqrt(2 * (1.0 + R[2][2])) * R[0][2];
            omg[1] = 1.0 / sqrt(2 * (1.0 + R[2][2])) * R[1][2];
            omg[2] = 1.0 / sqrt(2 * (1.0 + R[2][2])) * (1.0 + R[2][2]);
        } else if ((1.0 + R[1][1] >= ZERO_VALUE)) {
            omg[0] = 1.0 / sqrt(2 * (1.0 + R[1][1])) * R[0][1];
            omg[1] = 1.0 / sqrt(2 * (1.0 + R[1][1])) * (1.0 + R[1][1]);
            omg[2] = 1.0 / sqrt(2 * (1.0 + R[1][1])) * R[2][1];
        } else {
            omg[0] = 1.0 / sqrt(2 * (1.0 + R[0][0])) * (1.0 + R[0][0]);
            omg[1] = 1.0 / sqrt(2 * (1.0 + R[0][0])) * R[1][0];
            omg[2] = 1.0 / sqrt(2 * (1.0 + R[0][0])) * R[2][0];
        }
        omg[0] = PI * omg[0];
        omg[1] = PI * omg[1];
        omg[2] = PI * omg[2];
        VecToso3(omg, so3Mat);
    } else {
        int i;
        int j;
        double theta = acos(acosinput);
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                so3Mat[i][j] = 1.0 / (2.0 * sin(theta)) * (R[i][j] - R[j][i]) * theta;
            }
        }
    }

    return;
}

/**
*@brief					Description:Builds the homogeneous transformation matrix T corresponding to a rotation 
*						matrix R in SO(3) and a position vector p in R3.
*@param[in]		R		a rotation matrix R in SO(3).
*@param[in]		p		a position vector p in R3.
*@param[out]	T		the homogeneous transformation matrix T.
*@return		No return value.
*@note:
*@warning:
*/
////////将平移与旋转旋量用T4*4表示
void RpToTrans(double R[3][3], double p[3], double T[4][4]) {
    int i;
    int j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            T[i][j] = R[i][j];
        }
    }
    T[0][3] = p[0];
    T[1][3] = p[1];
    T[2][3] = p[2];
    T[3][0] = 0.0;
    T[3][1] = 0.0;
    T[3][2] = 0.0;
    T[3][3] = 1.0;
    return;
}

/**
*@brief Description: Extracts the rotation matrix R and position vector p 
from a homogeneous transformation matrix T.
*@param[in]		T		a homogeneous transformation matrix.
*@param[out]	R		the rotation matrix.
*@param[out]	p		position vector.
*@return		No return value.
*@note:
*@waring:
*/
////把T4*4还原回平移与旋转旋量
void TransToRp(double T[4][4], double R[3][3], double p[3]) {
    int i;
    int j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            R[i][j] = T[i][j];
        }
    }
    p[0] = T[0][3];
    p[1] = T[1][3];
    p[2] = T[2][3];
    return;
}

/**
*@brief Description:Computes the inverse of a homogeneous transformation matrix T.
*@param[in]		T		a homogeneous transformation matrix.
*@param[out]	InvT	the inverse of T.
*@return		No return value.
*@note:
*@warning:
*/
////计算T4*4的逆
void TransInv(double T[4][4], double InvT[4][4]) {
    double R[3][3];
    double InvR[3][3];
    double p[3];
    double p2[3];
    TransToRp(T, R, p);
    RotInv(R, InvR);
    Matrix3MultValue(InvR, -1.0, R);
    Matrix3MultVec(R, p, p2);
    RpToTrans(InvR, p2, InvT);
    return;
}

/**
*@brief Description:Computes the se(3) matrix corresponding to a 6-vector twist V.
*@param[in]		V		a 6-vector twist V.
*@param[out]	se3Mat	the se(3) matrix.
*@return		No return value.
*@note:
*@warning:
*/
////将六维空间向量写成SE3
void VecTose3(double V[6], double se3Mat[4][4]) {
    double so3Mat[3][3];
    double omg[3];
    int i;
    int j;
    omg[0] = V[0];
    omg[1] = V[1];
    omg[2] = V[2];
    VecToso3(omg, so3Mat);
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            se3Mat[i][j] = so3Mat[i][j];
        }

    }
    se3Mat[0][3] = V[3];
    se3Mat[1][3] = V[4];
    se3Mat[2][3] = V[5];
    se3Mat[3][0] = 0.0;
    se3Mat[3][1] = 0.0;
    se3Mat[3][2] = 0.0;
    se3Mat[3][3] = 0.0;
    return;
}

/**
*@brief Description:Computes the 6-vector twist corresponding to an se(3) matrix se3mat.
*@param[in]		se3Mat			an se(3) matrix.
*@param[out]	V				he 6-vector twist.
*@return		No return value.
*@note:
*@warning:
*/
////将T4*4还原为六维空间向量
void se3ToVec(double se3Mat[4][4], double V[6]) {
    V[0] = se3Mat[2][1];
    V[1] = se3Mat[0][2];
    V[2] = se3Mat[1][0];
    V[3] = se3Mat[0][3];
    V[4] = se3Mat[1][3];
    V[5] = se3Mat[2][3];
    return;
}

/**
*@brief Description:Computes the 6 x 6 adjoint representation [AdT ] of the homogeneous transformation matrix T.
*@param[in]		T		a homogeneous transformation matrix.
*@param[out]	AdT		the 6 x 6 adjoint representation [AdT ].
*@return		No return value.
*@note:
*@warning:
*/
////T4*4的伴随矩阵
void Adjoint(double T[4][4], double AdT[6][6]) {
    double R[3][3];
    double p[3];
    double so3Mat[3][3];
    int i;
    int j;
    TransToRp(T, R, p);
    VecToso3(p, so3Mat);

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            AdT[i][j] = R[i][j];
        }
    }
    for (i = 0; i < 3; i++) {
        for (j = 3; j < 6; j++) {
            AdT[i][j] = 0.0;
        }
    }
    for (i = 3; i < 6; i++) {
        for (j = 3; j < 6; j++) {
            AdT[i][j] = R[i - 3][j - 3];
        }
    }
    double mat[3][3];
    Matrix3Mult(so3Mat, R, mat);
    for (i = 3; i < 6; i++) {
        for (j = 0; j < 3; j++) {
            AdT[i][j] = mat[i - 3][j];
        }
    }

    return;
}

/**
*@brief			Description: Computes a normalized screw axis representation S of a screw described by 
*				a unit vector s in the direction of the screw axis, located at the point q, with pitch h.
*@param[in]		q		a point lying on the screw axis.
*@param[in]		s		a unit vector in the direction of the screw axis.
*@param[in]		h		the pitch of the screw axis.
*@param[in]		S		a normalized screw axis representation.
*@return		No return value.
*@note:
*@warning:
*/
void ScrewToAxis(double q[3], double s[3], double h, double S[6]) {
    double v[3];
    double temp[3];
    Vec3Cross(s, q, temp);
    Vec3MultValue(temp, -1.0, temp);
    Vec3MultValue(s, h, v);
    Vec3Add(v, temp, v);
    S[0] = s[0];
    S[1] = s[1];
    S[2] = s[2];
    S[3] = v[0];
    S[4] = v[1];
    S[5] = v[2];
    return;
}

/**
*@brief Description: Extracts the normalized screw axis S and the distance traveled along the screw
theta from the 6-vector of exponential coordinates S*theta.
*@param[in]		expc6		the 6-vector of exponential coordinates.
*@param[out]	S			the normalized screw axis.
*@param[out]	theta		the distance traveled along the screw.
*@return		No return value.
*@note:
*@warning:
*/
////将六维空间向量归一化
void AxisAng6(double expc6[6], double S[6], double *theta) {
    ////计算三维矢量模长
    *theta = Vec3Norm2(expc6);
    if (*theta < ZERO_ANGLE) {
        *theta = Vec3Norm2(&expc6[3]);
        if (*theta < ZERO_DISTANCE) {
            *theta = 0.0;
            //S is undefine,no motion at all.
            S[0] = 0.0;
            S[1] = 0.0;
            S[2] = 0.0;
            S[3] = 0.0;
            S[4] = 0.0;
            S[5] = 0.0;
            return;
        }
    }
    Vec3MultValue(expc6, 1.0 / (*theta), S);
    Vec3MultValue(&expc6[3], 1.0 / (*theta), &S[3]);
    return;
}

/**
*@brief			Description: Computes the homogeneous transformation matrix T in SE(3) corresponding to
*				the matrix exponential of se3mat in se(3).
*@param[in]		se3mat			the matrix exponential of se3mat in se(3).
*@param[out]	T				the homogeneous transformation matrix T in SE(3).
*@return		No return value.
*@note:
*@warning:
*/
////把SE(3)转换为T4*4（矩阵指数）
void MatrixExp6(double se3Mat[4][4], double T[4][4]) {
    int i;
    int j;
    double so3mat[3][3];
    double omgmat[3][3];
    double temp[3][3];
    double Gtheta[3][3];
    double p[3];
    double v[3];
    double omgtheta[3];
    double omghat[3];
    double theta;
    double MatI3[3][3] =
            {
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0
            };
    TransToRp(se3Mat, so3mat, p);//extracts so3mat from se3mat
    so3ToVec(so3mat, omgtheta);
    if (Vec3Norm2(omgtheta) < ZERO_ANGLE) {
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                T[i][j] = MatI3[i][j];
            }
        }
        T[0][3] = se3Mat[0][3];
        T[1][3] = se3Mat[1][3];
        T[2][3] = se3Mat[2][3];
        T[3][0] = 0.0;
        T[3][1] = 0.0;
        T[3][2] = 0.0;
        T[3][3] = 1.0;
    } else {
        AxisAng3(omgtheta, omghat, &theta);
        MatrixExp3(so3mat, temp);
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                T[i][j] = temp[i][j];
            }
        }
        Matrix3MultValue(so3mat, 1.0 / theta, omgmat);
        Matrix3Mult(omgmat, omgmat, temp);
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                Gtheta[i][j] =
                        MatI3[i][j] * theta + (1.0 - cos(theta)) * omgmat[i][j] + (theta - sin(theta)) * temp[i][j];
            }
        }
        v[0] = p[0] / theta;
        v[1] = p[1] / theta;
        v[2] = p[2] / theta;
        Matrix3MultVec(Gtheta, v, p);
        T[0][3] = p[0];
        T[1][3] = p[1];
        T[2][3] = p[2];
        T[3][0] = 0.0;
        T[3][1] = 0.0;
        T[3][2] = 0.0;
        T[3][3] = 1.0;
    }
    return;
}


/**
*@brief Description: Computes the matrix logarithm se3mat in se(3) of 
the homogeneous transformation matrix T in SE(3)
*@param[in]		T			the homogeneous transformation matrix.			
*@param[out]	se3Mat		the matrix logarithm of T.
*@return		No return value.
*@note:
*@warning:
*/
////////把T4*4转换为SE(3)（矩阵对数）
void MatrixLog6(double T[4][4], double se3Mat[4][4]) {
    int i;
    int j;
    double R[3][3];
    double so3mat[3][3];
    double p[3];
    TransToRp(T, R, p);
    MatrixLog3(R, so3mat);
    int flag = 0;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            if (fabs(so3mat[i][j]) < ZERO_ELEMENT) {
                continue;
            } else {
                flag = 1;
                break;
            }
        }
    }
    if (!flag) {
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                se3Mat[i][j] = 0.0;
            }

        }
        se3Mat[0][3] = T[0][3];
        se3Mat[1][3] = T[1][3];
        se3Mat[2][3] = T[2][3];
        se3Mat[3][0] = 0.0;
        se3Mat[3][1] = 0.0;
        se3Mat[3][2] = 0.0;
        se3Mat[3][3] = 0.0;
    } else {
        double MatI3[3][3] =
                {
                        1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0
                };
        double v[3];
        double temp[3][3];
        double InvGtheta[3][3];
        double theta = acos((R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0);
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                se3Mat[i][j] = so3mat[i][j];
            }
        }

        Matrix3Mult(so3mat, so3mat, temp);
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                InvGtheta[i][j] = MatI3[i][j] - 0.5 * so3mat[i][j] +
                                  (1.0 / theta - 0.5 * 1.0 / tan(0.5 * theta)) * temp[i][j] / theta;
            }
        }
        Matrix3MultVec(InvGtheta, p, v);
        se3Mat[0][3] = v[0];
        se3Mat[1][3] = v[1];
        se3Mat[2][3] = v[2];
        se3Mat[3][0] = 0.0;
        se3Mat[3][1] = 0.0;
        se3Mat[3][2] = 0.0;
        se3Mat[3][3] = 0.0;
    }
    return;
}



////===================================

////FKinSpace，FKinBody这两个函数的参数区别在于输入的螺旋轴单位矢量Slist是在末端物体坐标系下的表达，Blist是在固定坐标系下表达，其他参数含义相同。
////内部实现的区别在于FKinSpace内部使用固定坐标系下（笛卡尔原点）的雅可比，IKinSpaceNR内部使用物体坐标系下（ee原点）的雅可比。
////===================================


/**
*@brief			Description: Computes the end-effector frame given the zero position of the end-effector M,
*				the list of joint screws Slist expressed in the fixed-space frame, and the list of joint values thetalist.
*@param[in]		M			the zero position of the end-effector expressed in the fixed-space frame.
*@param[in]		Joint		Num	the number of joints.
*@param[in]		Slist		the list of joint screws Slist expressed in the fixed-space frame.
*							in the format of a matrix with the screw axes as the column.
*@param[in]		thetalist   the list of joint values.
*@param[out]	T			the end-effector frame expressed in the fixed-space frame.
*@return		No return value.
*@note:			when Slist is a matrix ,make sure that columns number of Slist is equal to JointNum,
*				rows number of Slist 6 .The function call should be written as
*				FKinBody(...,(double *)Slist,...).
*@warning:
*/
////给定初始姿态M，给定Slist（相当于给定DH表），再给定theta，计算转动theta后的正解T
////笛卡尔空间正解，Slist为从每个笛卡尔原点指向坐标系原点的六维向量
void FKinSpace(double M[4][4], int JointNum, double *Slist, double *thetalist, double T[4][4]) {
    int i, j, k, l;
    double se3mat[4][4];
    double T2[4][4];
    double exp6[4][4];
    double V[6];
    ////把M传给T
    Matrix4Equal(M, T);
    for (i = JointNum - 1; i >= 0; i--) {
        for (l = 0; l < 6; l++) {
            V[l] = Slist[l * JointNum + i];//get each column of Slist.获取每一列六维向量
        }
        ////将六维向量转换为SE3
        VecTose3(V, se3mat);
        for (j = 0; j < 4; j++) {
            for (k = 0; k < 4; k++) {
                se3mat[j][k] = se3mat[j][k] * thetalist[i];
            }
        }
        ////把SE(3)转换为T4*4（矩阵指数）
        MatrixExp6(se3mat, exp6);

        ////初始姿态T	乘变换矩阵T2
        Matrix4Mult(exp6, T, T2);
        Matrix4Equal(T2, T);
    }
    return;
}

/**
*@brief			Description:Computes the end-effector frame given the zero position of the end-effector M,
*				the list of joint screws Blist expressed in the end-effector frame, and the list of joint values thetalist.
*@param[in]		M			the zero position of the end-effector expressed in the end-effector frame.
*@param[in]		JointNum	the number of joints.
*@param[in]		Blist		the list of joint screws Slist expressed in the end-effector frame.
*							in the format of a matrix with the screw axes as the column.
*@param[in]		thetalist   the list of joint values.
*@param[out]	T			the end-effector frame expressed in the end-effector frame.
*@return		No return value.
*@note:			when Blist is a matrix ,make sure that columns number of Slist is equal to JointNum,
*				rows number of Slist 6 .The function call should be written as
*				FKinBody(...,(double *)Blist,...).
*@warning:
*/
////给定初始姿态M，给定Blist（相当于给定DH表），再给定theta，计算转动theta后的正解T (相对ee原点)
void FKinBody(double M[4][4], int JointNum, double *Blist, double thetalist[], double T[4][4]) {
    int i, j, k, l;
    double se3mat[4][4];
    double T2[4][4];
    double exp6[4][4];
    double V[6];
    Matrix4Equal(M, T);
    for (i = 0; i < JointNum; i++) {
        for (l = 0; l < 6; l++) {
            V[l] = Blist[l * JointNum + i];//get each column of Slist.
        }
        VecTose3(V, se3mat);
        for (j = 0; j < 4; j++) {
            for (k = 0; k < 4; k++) {
                se3mat[j][k] = se3mat[j][k] * thetalist[i];
            }
        }
        MatrixExp6(se3mat, exp6);
        Matrix4Mult(T, exp6, T2);
        Matrix4Equal(T2, T);
    }
    return;
}


/**
*@brief Description: Computes the body Jacobian Jb(theta) in 6×n given a list of joint screws Bi 
expressed in the body frame and a list of joint angles.
*@param[in]		Blist		The joint screw axes in the end - effector frame when the manipulator is 
*							at the home position, in the format of a matrix with the screw axes as the row.
*@param[in]		thetalist	A list of joint coordinates.
*@param[out]	Jb			Body Jacobian matrix.
*@return        No return value.
*@note:			 when Blist and Jb are matrixes ,make sure that columns number of Slist or Jb is equal to JointNum,
*				 rows number of Slist or Jb is 6 .The function call should be written as
*				 JacobianSpace(JointNum,(double *)Slist,thetalist,(double *)Jb).
*@warning:
*/
////计算ee空间下的雅克比矩阵
void JacobianBody(int JointNum, double *Blist, double *thetalist, double *Jb) {
    int i;
    int j;
    int k;
    double T1[4][4];
    double T2[4][4];
    double se3mat[4][4];
    double V[6];
    double AdT[6][6];
    double T[4][4] = {
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
    };
    //Fist column of Jbn.
    for (i = 0; i < 6; i++) {
        Jb[i * JointNum + JointNum - 1] = Blist[i * JointNum + JointNum - 1];
    }
    //Jbi for i=n-1,n-2,...,1.
    for (i = JointNum - 2; i >= 0; i--) {
        for (j = 0; j < 6; j++) {
            V[j] = -1.0 * Blist[j * JointNum + i + 1];
        }
        VecTose3(V, se3mat);
        Matrix4MultValue(se3mat, thetalist[i + 1], se3mat);
        MatrixExp6(se3mat, T1);
        Matrix4Mult(T, T1, T2);
        Matrix4Equal(T2, T);
        Adjoint(T, AdT);
        for (j = 0; j < 6; j++) {
            Jb[j * JointNum + i] = 0;
            for (k = 0; k < 6; k++) {
                Jb[j * JointNum + i] = Jb[j * JointNum + i] + AdT[j][k] * Blist[k * JointNum + i];
            }
        }
    }
    return;
}

/**
*@brief			Description:Computes the space Jacobian Js(theta) in R6 x n given a list of joint screws Si 
*				expressed in the fixed space frame and a list of joint angles.
*@param[in]		JointNum	joints number.
*@param[in]		Slist		The joint screw axes expressed in the fixed space frame when the manipulator is
*							at the home position, in the format of a matrix with the screw axes as the column.
*@param[in]		thetalist	A list of joint coordinates.
*@param[out]	Js			Space Jacobian matrix.
*@return		 No return value.
*@note:			 when Slist and Js are matrices ,make sure that columns number of Slist or Js is equal to JointNum,
*				 rows number of Slist or Js is 6 .The function call should be written as 
*				 JacobianSpace(JointNum,(double *)Slist,thetalist,(double *)Js).
*@warning:
*/
////计算笛卡尔空间下的雅克比矩阵
void JacobianSpace(int JointNum, double *Slist, double *thetalist, double *Js) {
    int i;
    int j;
    int k;
    double T1[4][4];
    double T2[4][4];
    double se3mat[4][4];
    double V[6];
    double AdT[6][6];
    double T[4][4] = {
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
    };
    //Fist column of Js.
    ////C++的矩阵也是数组 0 4 8其实就是1行1列，2行1列，2行1列
    for (i = 0; i < 6; i++) {
        //printf("i = %d\n", i * JointNum + 0);
        Js[i * JointNum + 0] = Slist[i * JointNum + 0];
        //printf("Js = %lf\n", Js[i * JointNum + 0]);

    }
    //Jsi for i=2,3,...,n.
    for (i = 1; i < JointNum; i++) {
        for (j = 0; j < 6; j++) {
            V[j] = Slist[j * JointNum + i - 1];
        }
        ////现将六维空间向量转为SE3
        VecTose3(V, se3mat);
        ////SE3*theta(i-1)
        Matrix4MultValue(se3mat, thetalist[i - 1], se3mat);
        ////把SE(3)转换为T4*4（矩阵指数）
        MatrixExp6(se3mat, T1);
        ////T1*T2
        Matrix4Mult(T, T1, T2);
        Matrix4Equal(T2, T);
        ////求T的伴随矩阵
        Adjoint(T, AdT);
        for (j = 0; j < 6; j++) {
            Js[j * JointNum + i] = 0;
            for (k = 0; k < 6; k++) {
                Js[j * JointNum + i] = Js[j * JointNum + i] + AdT[j][k] * Slist[k * JointNum + i];
            }
        }
    }
    return;
}

#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define MAX(a, b) (a>b?a:b)
#define MIN(a, b) (a<b?a:b)

/**
*@brief			Computes (a^2+b^2)^(1/2),
*@param[in]		a
*@param[in]		b
*@note:
*@warning:
*/
double pythag(const double a, const double b) {
    double absa, absb;
    absa = fabs(a);
    absb = fabs(b);
    if (absa > absb) return absa * sqrt(1.0 + (absb / absa) * (absb / absa));
    else return (absb == 0.0 ? 0.0 : absb * sqrt(1.0 + (absa / absb) * (absa / absb)));

}

void reorder(double *u, int m, int n, double *w, double *v) {
    /*Given the output of decompose, this routine sorts the singular values, and corresponding columns
        of u and v, by decreasing magnitude.Also, signs of corresponding columns are flipped so as to
        maximize the number of positive elements.*/
    int i, j, k, s, inc = 1;
    double sw;
    double *su = (double *) malloc(m * sizeof(double));
    if (su == NULL) {
        return;
    }
    double *sv = (double *) malloc(n * sizeof(double));
    if (sv == NULL) {
        free(su);
        return;
    }
    do {
        inc *= 3;
        inc++;
    } while (inc <=
             n);// Sort.The method is Shell’s sort.(The work is negligible as compared to that already done in decompose.)
    do {
        inc /= 3;
        for (i = inc; i < n; i++) {
            sw = w[i];
            for (k = 0; k < m; k++) su[k] = u[k * n + i];
            for (k = 0; k < n; k++) sv[k] = v[k * n + i];
            j = i;
            while (w[j - inc] < sw) {
                w[j] = w[j - inc];
                for (k = 0; k < m; k++) u[k * n + j] = u[k * n + j - inc];
                for (k = 0; k < n; k++) v[k * n + j] = v[k * n + j - inc];
                j -= inc;
                if (j < inc) break;
            }
            w[j] = sw;
            for (k = 0; k < m; k++) u[k * n + j] = su[k];
            for (k = 0; k < n; k++) v[k * n + j] = sv[k];
        }
    } while (inc > 1);
    for (k = 0; k < n; k++) {
        //Flip signs.
        s = 0;
        for (i = 0; i < m; i++) if (u[i * n + k] < 0.0) s++;
        for (j = 0; j < n; j++) if (v[j * n + k] < 0.0) s++;
        if (s > (m + n) / 2) {
            for (i = 0; i < m; i++) u[i * n + k] = -u[i * n + k];
            for (j = 0; j < n; j++) v[j * n + k] = -v[j * n + k];
        }
    }
    free(su);
    free(sv);
    return;
}



/**
*@brief			Given the matrix a,this routine computes its singular value decomposition.
*@param[in/out]	a		input matrix a or output matrix u.
*@param[in]		m		number of rows of matrix a.
*@param[in]		n		number of columns of matrix a.
*@param[in]		tol		any	singular values less than a tolerance are treated as zero.
*@param[out]	w		output vector w.
*@param[out]	v		output matrix v.	
*@return        No return value.
*@note:			input a must be a  m rows and n columns matrix,w is n-vector,v is a n rows and n columns matrix.
*@warning:
*/
////svd奇异值分解
int svdcmp(double *a, int m, int n, double tol, double *w, double *v) {

    int flag, i, its, j, jj, k, l, nm;
    double anorm, c, f, g, h, s, scale, x, y, z;
    double *rv1 = (double *) malloc(n * sizeof(double));
    if (rv1 == NULL) {
        return 1;
    }
    g = scale = anorm = 0.0;
    for (i = 0; i < n; i++)//Householder reduction to bidiagonal form.
    {
        l = i + 2;
        rv1[i] = scale * g;
        g = s = scale = 0.0;
        if (i < m) {
            for (k = i; k < m; k++)scale += fabs(a[k * n + i]);
            if (scale != 0.0) {
                for (k = i; k < m; k++) {
                    a[k * n + i] /= scale;
                    s += a[k * n + i] * a[k * n + i];
                }
                f = a[i * n + i];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                a[i * n + i] = f - g;
                for (j = l - 1; j < n; j++) {
                    for (s = 0.0, k = i; k < m; k++) {
                        s += a[k * n + i] * a[k * n + j];
                    }
                    f = s / h;
                    for (k = i; k < m; k++) {
                        a[k * n + j] += f * a[k * n + i];
                    }
                }
                for (k = i; k < m; k++) {
                    a[k * n + i] *= scale;
                }
            }
        }
        w[i] = scale * g;
        g = s = scale = 0.0;
        if (i + 1 <= m && (i + 1) != n) {
            for (k = l - 1; k < n; k++) {
                scale += fabs(a[i * n + k]);
            }
            if (scale != 0.0) {
                for (k = l - 1; k < n; k++) {
                    a[i * n + k] /= scale;
                    s += a[i * n + k] * a[i * n + k];
                }
                f = a[i * n + l - 1];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                a[i * n + l - 1] = f - g;
                for (k = l - 1; k < n; k++) {
                    rv1[k] = a[i * n + k] / h;
                }
                for (j = l - 1; j < m; j++) {
                    for (s = 0.0, k = l - 1; k < n; k++) {
                        s += a[j * n + k] * a[i * n + k];
                    }
                    for (k = l - 1; k < n; k++) {
                        a[j * n + k] += s * rv1[k];
                    }
                }
                for (k = l - 1; k < n; k++) {
                    a[i * n + k] *= scale;
                }
            }
        }
        anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
    }

    for (i = n - 1; i >= 0; i--)//Accumulation of right-hand transformations.
    {
        if (i < n - 1) {
            if (g != 0.0) {
                for (j = l; j < n; j++)//Double division to avoid possible underflow.
                {
                    v[j * n + i] = (a[i * n + j] / a[i * n + l]) / g;
                }
                for (j = l; j < n; j++) {
                    for (s = 0.0, k = l; k < n; k++) {
                        s += a[i * n + k] * v[k * n + j];
                    }
                    for (k = l; k < n; k++) {
                        v[k * n + j] += s * v[k * n + i];
                    }
                }
            }
            for (j = l; j < n; j++) {
                v[i * n + j] = v[j * n + i] = 0.0;
            }
        }
        v[i * n + i] = 1.0;
        g = rv1[i];
        l = i;
    }

    for (i = MIN(m, n) - 1; i >= 0; i--)//Accumulation of left-hand transformations.
    {
        l = i + 1;
        g = w[i];
        for (j = l; j < n; j++) {
            a[i * n + j] = 0.0;
        }
        if (g != 0.0) {
            g = 1.0 / g;
            for (j = l; j < n; j++) {
                for (s = 0.0, k = l; k < m; k++) {
                    s += a[k * n + i] * a[k * n + j];
                }
                f = (s / a[i * n + i]) * g;
                for (k = i; k < m; k++) {
                    a[k * n + j] += f * a[k * n + i];
                }
            }
            for (j = i; j < m; j++) {
                a[j * n + i] *= g;
            }
        } else {
            for (j = i; j < m; j++) {
                a[j * n + i] = 0.0;
            }
        }
        ++a[i * n + i];
    }

    for (k = n - 1; k >= 0; k--) {
        /* Diagonalization of the bidiagonal form: Loop over
        singular values, and over allowed iterations. */
        for (its = 0; its < 30; its++) {
            flag = 1;
            for (l = k; l >= 0; l--)//Test for splitting.
            {
                nm = l - 1;
                if (l == 0 || fabs(rv1[l]) <= tol * anorm) {
                    flag = 0;
                    break;
                }
                if (fabs(w[nm]) <= tol * anorm)break;

            }
            if (flag)//Cancellation of rv1[l], if l > 0.
            {
                c = 0.0;
                s = 1.0;
                for (i = l; i < k + 1; i++) {
                    f = s * rv1[i];
                    rv1[i] = c * rv1[i];
                    if (fabs(f) <= tol * anorm)break;
                    g = w[i];
                    h = pythag(f, g);
                    w[i] = h;
                    h = 1.0 / h;
                    c = g * h;
                    s = -f * h;
                    for (j = 0; j < m; j++) {
                        y = a[j * n + nm];
                        z = a[j * n + i];
                        a[j * n + nm] = y * c + z * s;
                        a[j * n + i] = z * c - y * s;
                    }
                }
            }
            z = w[k];
            if (l == k)//Convergence
            {
                if (z < 0.0)//Singular value is made nonnegative..
                {
                    w[k] = -z;
                    for (j = 0; j < n; j++) {
                        v[j * n + k] = -v[j * n + k];
                    }
                }
                break;
            }
            //if (its==29)
            //{
            //	printf("no convergence in 30 svdcmp iterations\n");
            //}
            x = w[l]; //Shift from bottom 2-by-2 minor.
            nm = k - 1;
            y = w[nm];
            g = rv1[nm];
            h = rv1[k];
            f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
            g = pythag(f, 1.0);
            f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
            c = s = 1.0;
            for (j = l; j <= nm; j++)//Next QR transformation:
            {
                i = j + 1;
                g = rv1[i];
                y = w[i];
                h = s * g;
                g = c * g;
                z = pythag(f, h);
                rv1[j] = z;
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = g * c - x * s;
                h = y * s;
                y *= c;
                for (jj = 0; jj < n; jj++) {
                    x = v[jj * n + j];
                    z = v[jj * n + i];
                    v[jj * n + j] = x * c + z * s;
                    v[jj * n + i] = z * c - x * s;
                }
                z = pythag(f, h);
                w[j] = z;
                if (z)//Rotation can be arbitrary if z = 0.
                {
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                }
                f = c * g + s * y;
                x = c * y - s * g;
                for (jj = 0; jj < m; jj++) {
                    y = a[jj * n + j];
                    z = a[jj * n + i];
                    a[jj * n + j] = y * c + z * s;
                    a[jj * n + i] = z * c - y * s;
                }
            }
            rv1[l] = 0.0;
            rv1[k] = f;
            w[k] = x;
        }
    }
    free(rv1);
    reorder(a, m, n, w, v);//对特征值按降序排列，且调整对应矩阵
    return 0;
}

double u[PINV_MAX * PINV_MAX] = {0};
double w[PINV_MAX] = {0};
double vw[PINV_MAX * PINV_MAX] = {0};
double v[PINV_MAX * PINV_MAX] = {0};
double uT[PINV_MAX * PINV_MAX] = {0};
/**
*@brief			Computes Moore-Penrose pseudoinverse by  Singular Value Decomposition (SVD) algorithm.
*@param[in]		a			an arbitrary order matrix.
*@param[in]		m			rows.
*@param[in]		n			columns.
*@param[in]		tol			any	singular values less than a tolerance are treated as zero.
*@param[out]	b			Moore-Penrose pseudoinverse of matrix a.
*@return        0:success,Nonzero:failure.
*@retval		0			success.
*@retval		1			failure when use malloc to allocate memory.
*@note:		
*@warning:
*/
////计算矩阵伪逆（svd）
int MatrixPinv(const double *a, int m, int n, double tol, double *b) {
    int i, j;
    int ret;
    memcpy(u, a, m * n * sizeof(double));
    ret = svdcmp(u, m, n, tol, w, v);
    for (i = 0; i < n; i++) {
        if (w[i] <= tol) {
            w[i] = 0;
        } else {
            w[i] = 1.0 / w[i];
        }
    }
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            vw[i * n + j] = v[i * n + j] * w[j];
        }
    }
    MatrixT(u, m, n, uT);
    MatrixMult(vw, n, n, uT, m, b);
    return 0;
}

/**
*@brief 		Description:use an iterative Newton-Raphson root-finding method.
*@param[in]		JointNum		Number of joints.
*@param[in]		Blist			The joint screw axes in the Body frame when the manipulator
*								is at the home position, in the format of a matrix with the
*								screw axes as the columns.
*@param[in]		M				The home configuration of the end-effector.
*@param[in]		T				The desired end-effector configuration Tsd.
*@param[in]		thetalist0		An initial guess of joint angles that are close to satisfying Tsd.
*@param[in]		eomg			A small positive tolerance on the end-effector orientation error.
*								The returned joint angles must give an end-effector orientation error less than eomg.
*@param[in]		ev				A small positive tolerance on the end-effector linear position error. The returned
*								joint angles must give an end-effector position error less than ev.
*@param[in]		maxiter			The maximum number of iterations before the algorithm is terminated.
*@param[out]	thetalist		Joint angles that achieve T within the specified tolerances.
*@return		0:success,Nonzero:failure
*@retval		0				success.
*@retval		1				The maximum number of iterations reach before the algorithm is terminated.
*@retval		2				failure to allocate memory for Jacobian matrix.
*@note:
*@warning:
*/
////计算ee坐标系下的逆运动学
int IKinBodyNR(int JointNum, double *Blist, double M[4][4], double T[4][4], double *thetalist0, double eomg, double ev,
               int maxiter, double *thetalist) {
    int i, j;
    double Tsb[4][4];
    double iTsb[4][4];
    double Tbd[4][4];
    double se3Mat[4][4];

    double Vb[6];
    int ErrFlag;
    double *Jb = (double *) malloc(JointNum * 6 * sizeof(double));
    if (Jb == NULL) {
        return 2;
    }
    double *pJb = (double *) malloc(JointNum * 6 * sizeof(double));
    if (pJb == NULL) {
        free(Jb);
        return 2;
    }
    double *dtheta = (double *) malloc(JointNum * sizeof(double));
    if (dtheta == NULL) {
        free(Jb);
        free(pJb);
        return 2;
    }
    MatrixCopy(thetalist0, JointNum, 1, thetalist);
    FKinBody(M, JointNum, Blist, thetalist, Tsb);
    TransInv(Tsb, iTsb);
    Matrix4Mult(iTsb, T, Tbd);
    MatrixLog6(Tbd, se3Mat);
    se3ToVec(se3Mat, Vb);
    ErrFlag = VecNorm2(3, &Vb[0]) > eomg || VecNorm2(3, &Vb[3]) > ev;
    i = 0;

#if(DEBUGMODE)
    ///////////////////Debug///////////
    printf("iteration:%4d  thetalist:  ", i);
    int k;
    for (k = 0; k < JointNum; k++) {
        printf("%4.6lf\t", thetalist[k]);
    }
    printf("\n");
    ////////////////////////////////////
#endif

    while (ErrFlag && i < maxiter) {
        JacobianBody(JointNum, Blist, thetalist, Jb);
        MatrixPinv(Jb, 6, JointNum, 2.2E-15, pJb);
        MatrixMult(pJb, JointNum, 6, Vb, 1, dtheta);
        for (j = 0; j < JointNum; j++) {
            thetalist[j] = thetalist[j] + dtheta[j];
        }
        i++;
        FKinBody(M, JointNum, Blist, thetalist, Tsb);
        TransInv(Tsb, iTsb);
        Matrix4Mult(iTsb, T, Tbd);
        MatrixLog6(Tbd, se3Mat);
        se3ToVec(se3Mat, Vb);
        ErrFlag = VecNorm2(3, &Vb[0]) > eomg || VecNorm2(3, &Vb[3]) > ev;
#if (DEBUGMODE)
        ///////////////////////DEBUG///////////
        printf("iteration:%4d  thetalist:  ", i);
        for (k = 0; k < JointNum; k++) {
            printf("%4.6lf\t", thetalist[k]);
        }
        printf("\n");
        //////////////////////////////////////
#endif

    }
    free(Jb);
    free(pJb);
    free(dtheta);
    return ErrFlag;
}



/**
*@brief 		Description:use an iterative Newton-Raphson root-finding method.
*@param[in]		JointNum		Number of joints.
*@param[in]		Slist			The joint screw axes in the space frame when the manipulator
*								is at the home position, in the format of a matrix with the
*								screw axes as the columns.
*@param[in]		M				The home configuration of the end-effector.
*@param[in]		T				The desired end-effector configuration Tsd.
*@param[in]		thetalist0		An initial guess of joint angles that are close to satisfying Tsd.
*@param[in]		eomg			A small positive tolerance on the end-effector orientation error. 
*								The returned joint angles must give an end-effector orientation error less than eomg.
*@param[in]		ev				A small positive tolerance on the end-effector linear position error. The returned 
*								joint angles must give an end-effector position error less than ev.
*@param[in]		maxiter			The maximum number of iterations before the algorithm is terminated.
*@param[out]	thetalist		Joint angles that achieve T within the specified tolerances.
*@return		0:success,Nonzero:failure
*@retval		0
*@note :
*@warning:
*/
////计算笛卡尔坐标系下的逆运动学
int IKinSpaceNR(int JointNum, double *Slist, double M[4][4], double T[4][4], double *thetalist0, double eomg, double ev,
                int maxiter, double *thetalist) {
    int i, j;
    double Tsb[4][4];
    double iTsb[4][4];
    double Tbd[4][4];
    double AdT[6][6];
    double se3Mat[4][4];
    double Vb[6];
    double Vs[6];
    int ErrFlag;
    double *Js = (double *) malloc(JointNum * 6 * sizeof(double));
    if (Js == NULL) {
        return 2;
    }
    double *pJs = (double *) malloc(JointNum * 6 * sizeof(double));
    if (pJs == NULL) {
        free(Js);
        return 2;
    }
    double *dtheta = (double *) malloc(JointNum * sizeof(double));
    if (dtheta == NULL) {
        free(Js);
        free(pJs);
        return 2;
    }
    MatrixCopy(thetalist0, JointNum, 1, thetalist);
    FKinSpace(M, JointNum, Slist, thetalist, Tsb);
    TransInv(Tsb, iTsb);
    Matrix4Mult(iTsb, T, Tbd);
    MatrixLog6(Tbd, se3Mat);
    se3ToVec(se3Mat, Vb);//Vb旋转矢量
    Adjoint(Tsb, AdT);//求Tsb的伴随矩阵AdT
    MatrixMult((double *) AdT, 6, 6, Vb, 1, Vs);//Vs = AdT * Vb
    ErrFlag = VecNorm2(3, &Vs[0]) > eomg || VecNorm2(3, &Vs[3]) > ev;
    i = 0;

#if(DEBUGMODE)
    ///////////////////Debug///////////
    printf("iteration:%4d  thetalist:  ", i);
    int k;
    for (k = 0; k < JointNum; k++) {
        printf("%4.4lf, ", thetalist[k]);
    }
    printf("\n");
    ////////////////////////////////////
#endif

    while (ErrFlag && i < maxiter) {
        JacobianSpace(JointNum, Slist, thetalist, Js);
        MatrixPinv(Js, 6, JointNum, 2.2E-15, pJs);
        MatrixMult(pJs, JointNum, 6, Vs, 1, dtheta);
        for (j = 0; j < JointNum; j++) {
            thetalist[j] = thetalist[j] + dtheta[j];
        }
        i++;
        FKinSpace(M, JointNum, Slist, thetalist, Tsb);
        TransInv(Tsb, iTsb);
        Matrix4Mult(iTsb, T, Tbd);
        MatrixLog6(Tbd, se3Mat);
        se3ToVec(se3Mat, Vb);
        Adjoint(Tsb, AdT);
        MatrixMult((double *) AdT, 6, 6, Vb, 1, Vs);
        ErrFlag = VecNorm2(3, &Vs[0]) > eomg || VecNorm2(3, &Vs[3]) > ev;
#if (DEBUGMODE)
        ///////////////////////DEBUG///////////
        printf("iteration:%4d  thetalist:  ", i);
        for (k = 0; k < JointNum; k++) {
            printf("%4.4lf, ", thetalist[k]);
        }
        printf("\n");
        //////////////////////////////////////
#endif

    }
    free(Js);
    free(pJs);
    free(dtheta);
    return ErrFlag;
}


/**
* @brief 			Description: Algorithm for Computing the ZYX Euler Angles according to rotation matrix.
* @param[in]		R				Rotation matrix.
* @param[out]		alpha			Angles for rotation around Z axis.
* @param[out]		beta			Angles for rotation around Y axis.
* @param[out]		gamma			Angles for rotation around X axis.
* @return			No return value.
* @note:
*@warning:
*/
void RotToZYXEulerAngle(double R[3][3], double *alpha, double *beta, double *gamma) {

    if (fabs(1.0 + R[2][0]) < ZERO_ELEMENT) {
        *alpha = 0;
        *beta = PI / 2.0;
        *gamma = atan2(R[0][1], R[1][1]);

    } else if (fabs(1.0 - R[2][0]) < ZERO_ELEMENT) {
        *alpha = 0;
        *beta = -PI / 2.0;
        *gamma = atan2(R[0][1], R[1][1]);
    } else {

        *alpha = atan2(R[1][0], R[0][0]);
        *beta = atan2(-R[2][0], sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0]));
        *gamma = atan2(R[2][1], R[2][2]);
    }
    return;

}

/**
* @brief 			Description: Algorithm for Computing the rotation matrix of ZYX Euler Angles.
* @param[in]		alpha			Angles for rotation around Z axis.
* @param[in]		beta			Angles for rotation around Y axis.
* @param[in]		gamma			Angles for rotation around X axis.
* @param[out]		R				Rotation matrix.
* @return			No return value.
* @note:
* @waring:
*/
void ZYXEulerAngleToRot(double alpha, double beta, double gamma, double R[3][3]) {
    R[0][0] = cos(alpha) * cos(beta);
    R[0][1] = cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma);
    R[0][2] = cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma);
    R[1][0] = sin(alpha) * cos(beta);
    R[1][1] = sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma);
    R[1][2] = sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma);
    R[2][0] = -sin(beta);
    R[2][1] = cos(beta) * sin(gamma);
    R[2][2] = cos(beta) * cos(gamma);
    return;
}


/**
 * @brief 			Description: Computes the unit vector of Euler axis and rotation angle corresponding to rotation matrix.
 * @param[in]		R				A rotation matrix.
 * @param[out]		omghat			the unit vector of Euler axis .
 * @param[out]		theta			the rotation angle.
 * @return			No return value.
 * @retval			0
 * @note:			if  theta is zero ,the unit axis is undefined and set it as a zero vector [0;0;0].
 *@warning:
*/
void RotToAxisAng(double R[3][3], double omghat[3], double *theta) {
    double tmp;
    double omg[3] = {0};
    double acosinput = (R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0;
    if (fabs(acosinput - 1.0) < ZERO_VALUE) {
        memset(omghat, 0, 3 * sizeof(double));
        *theta = 0.0;
    } else if (acosinput <= -1.0) {
        if ((1.0 + R[2][2]) >= ZERO_VALUE) {
            omg[0] = 1.0 / sqrt(2 * (1.0 + R[2][2])) * R[0][2];
            omg[1] = 1.0 / sqrt(2 * (1.0 + R[2][2])) * R[1][2];
            omg[2] = 1.0 / sqrt(2 * (1.0 + R[2][2])) * (1.0 + R[2][2]);
        } else if ((1.0 + R[1][1] >= ZERO_VALUE)) {
            omg[0] = 1.0 / sqrt(2 * (1.0 + R[1][1])) * R[0][1];
            omg[1] = 1.0 / sqrt(2 * (1.0 + R[1][1])) * (1.0 + R[1][1]);
            omg[2] = 1.0 / sqrt(2 * (1.0 + R[1][1])) * R[2][1];
        } else {
            omg[0] = 1.0 / sqrt(2 * (1.0 + R[0][0])) * (1.0 + R[0][0]);
            omg[1] = 1.0 / sqrt(2 * (1.0 + R[0][0])) * R[1][0];
            omg[2] = 1.0 / sqrt(2 * (1.0 + R[0][0])) * R[2][0];
        }
        omghat[0] = omg[0];
        omghat[1] = omg[1];
        omghat[2] = omg[2];
        *theta = PI;
    } else {
        *theta = acos(acosinput);
        tmp = 2.0 * sin(*theta);
        omghat[0] = (R[2][1] - R[1][2]) / tmp;
        omghat[1] = (R[0][2] - R[2][0]) / tmp;
        omghat[2] = (R[1][0] - R[0][1]) / tmp;

    }

    return;
}


/**
 * @brief 			Description: Computes the unit quaternion corresponding to the Euler axis and rotation angle.
 * @param[in]		omg				Unit vector of Euler axis.
 * @param[in]		theta			Rotation angle.
 * @param[out]		q				The unit quaternion
 * @return			No return value.
 * @note:
 *@warning:
*/
////给定旋转轴omg和转角theta，计算单位四元数
void AxisAngToQuaternion(double omg[3], double theta, double q[4]) {
    q[0] = cos(theta / 2.0);
    q[1] = omg[0] * sin(theta / 2.0);
    q[2] = omg[1] * sin(theta / 2.0);
    q[3] = omg[2] * sin(theta / 2.0);
    return;
}


/**
 * @brief 			Description:Computes the unit quaternion corresponding to a rotation matrix.
 * @param[in]		q				Unit quaternion.
 * @param[out]		R				Rotation matrix.
 * @return			No return value.
 * @note:
 * @warning:
*/
////给定单位四元数，计算旋转矩阵R
void QuaternionToRot(double q[4], double R[3][3]) {
    R[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    R[0][1] = 2.0 * (q[1] * q[2] - q[0] * q[3]);
    R[0][2] = 2.0 * (q[0] * q[2] + q[1] * q[3]);
    R[1][0] = 2.0 * (q[0] * q[3] + q[1] * q[2]);
    R[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
    R[1][2] = 2.0 * (q[2] * q[3] - q[0] * q[1]);
    R[2][0] = 2.0 * (q[1] * q[3] - q[0] * q[2]);
    R[2][1] = 2.0 * (q[0] * q[1] + q[2] * q[3]);
    R[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    return;
}

/**
 * @brief 			Description: Computes the unit quaternion corresponding to the rotation matrix.
 * @param[in]		R				The rotation matrix.
 * @param[out]		q				The unit quaternion.
 * @return			No return value.
 * @note:
 * @warning:
*/
////给定旋转矩阵R，计算单位四元数
void RotToQuaternion(double R[3][3], double q[4]) {
    double omghat[3];
    double theta;
    RotToAxisAng(R, omghat, &theta);
    AxisAngToQuaternion(omghat, theta, q);
    return;
}


/**
* @brief 			单位四元数转换为旋转矩阵。
* @param[in]		q				单位四元数[ox,oy,oz,ow].
* @param[out]		R				旋转矩阵.
* @return			无返回值.
* @note:
* @warning:         (1)此函数不对输入参数做检查，需保证输入为单位四元数。
					(2)四元数顺序为[ox,oy,oz,w]。
*/
void q2rot(double q[4], double R[3][3]) {
    double q1[4] = {0};

    q1[0] = q[3];
    q1[1] = q[0];
    q1[2] = q[1];
    q1[3] = q[2];
    QuaternionToRot(q1, R);
    return;
}

/**
* @brief 			旋转矩阵转单位四元数。
* @param[in]		R				旋转矩阵
* @param[out]		q				单位四元数.[ox,oy,oz,ow].
* @return			无返回值.
* @note:
* @warning:         (1)此函数不对输入参数做检查，需保证输入为旋转矩阵。
					(2)四元数顺序为[ox,oy,oz,w]。
*/
void rot2q(double R[3][3], double q[4]) {
    double q1[4] = {0};
    RotToQuaternion(R, q1);
    q[0] = q1[1];
    q[1] = q1[2];
    q[2] = q1[3];
    q[3] = q1[0];
    return;
}


/**
 * @brief 			Description: Computes the parameters of  orientation interpolation between two orientations.
 * @param[in]		Rs				Start orientation.
 * @param[in]		Re				End orientation. 
 * @param[in]		Param			structure of orientation interpolation parameters..
 * @return			No return value.
 * @note:
 * @warning:
*/
////给定当前和期望旋转矩阵RS,Re,初始化旋转矩阵参数（转轴omg和转角theta）
void InitialOrientInpParam(double Rs[3][3], double Re[3][3], OrientInpParam *Param) {
    double InvR[3][3];
    MatrixCopy((double *) Rs, 3, 3, (double *) Param->Rs);
    MatrixCopy((double *) Re, 3, 3, (double *) Param->Re);
    RotInv(Rs, InvR);
    ////Re = Rs * R; Inv(Rs) * Re = R;计算R
    MatrixMult((double *) InvR, 3, 3, (double *) Re, 3, (double *) Param->R);
    ////根据R算出转轴和转角
    RotToAxisAng(Param->R, Param->omg, &Param->theta);
    MatrixCopy((double *) Param->R, 3, 3, (double *) Param->Ri);
    Param->thetai = 0.0;
    Param->InpFlag = 1;
    return;
}


/**
 * @brief 			Description: Computes orientations in each interpolation cycle.
 * @param[in]		Param			Interpolation parameter structure.
 * @param[out]		dtheta			angle  need to rotate from previous orientation to next orientation in next time step.
 * @return			Ri1				orientations in next interpolation cycle.
 * @retval			0
 * @note:
 * @warning:
*/
////计算四元数插补
void QuaternionOrientInp(OrientInpParam *Param, double dtheta, double Ri1[3][3]) {
    double q[4];
    double R[3][3];
    Param->InpFlag = 2;
    Param->thetai = Param->thetai + dtheta;
    if (Param->thetai >= Param->theta) {
        Param->thetai = Param->theta;
        Param->InpFlag = 3;
    }
    AxisAngToQuaternion(Param->omg, Param->thetai, q);
    QuaternionToRot(q, R);
    MatrixMult((double *) Param->Rs, 3, 3, (double *) R, 3, (double *) Ri1);
    MatrixCopy((double *) Ri1, 3, 3, (double *) Param->Ri);
    return;
}

/**
 * @brief 			Description:Computes the parameters of line path for interpolation.
 * @param[in]		p1				Coordinates of start point.
 * @param[in]		p2				Coordinates of end point.
 * @param[out]		p				Line path parameters structure.
 * @return			No return value.
 * @note:
 * @warning:
*/
////位置变换参数初始化
void InitialLinePathParam(double p1[3], double p2[3], LineInpParam *p) {
    int i;
    for (i = 0; i < 3; i++) {
        p->p1[i] = p1[i];
        p->p2[i] = p2[i];
        p->pi[i] = p1[i];
    }
    p->L = sqrt(
            (p2[0] - p1[0]) * (p2[0] - p1[0]) + (p2[1] - p1[1]) * (p2[1] - p1[1]) + (p2[2] - p1[2]) * (p2[2] - p1[2]));

    if (p->L < ZERO_DISTANCE) {
        p->t[0] = 0.0;
        p->t[1] = 0.0;
        p->t[2] = 0.0;
    } else {
        ////将归一化后的正交基底传给每一个基
        p->t[0] = (p2[0] - p1[0]) / p->L;
        p->t[1] = (p2[1] - p1[1]) / p->L;
        p->t[2] = (p2[2] - p1[2]) / p->L;
    }
    ////Flag变为初始化完成
    p->InpFlag = 1;
    ////令当前位置为0
    p->Li = 0;
    return;
}

/**
 * @brief 			Description:Computes the line path interpolation coordinates in each interpolation cycle.
 * @param[in]		p				Line path parameters structure.
 * @param[in]		dL				step length in next interpolation cycle. 
 * @param[out]		pi1				coordinates in next interpolation cycle.
 * @return			No return value.
 * @note:
 * @warning:
*/
////计算直线插补
void LinePathInp(LineInpParam *p, double dL, double pi1[3]) {
    p->InpFlag = 2;                    ////状态变为正在计算插补
    if (p->Li + dL >= p->L)              ////当前模长+步长 >= 总模长
    {
        ////将期望模长传给下一周期的模长
        pi1[0] = p->p2[0];
        pi1[1] = p->p2[1];
        pi1[2] = p->p2[2];
        p->Li = p->L;
        p->InpFlag = 3;
    } else if (p->L - p->Li < 2.0 * dL)     ////若总模长 - 当前模长 >= 2*步长
    {
        ////避免最终步长过小，将步长/2，p->t 为六维变换的归一化正交基，将当前模长 + 归一化正交基*步长 传给下一周期的期望插补点pi1
        dL = 0.5 * dL;
        pi1[0] = p->pi[0] + p->t[0] * dL;
        pi1[1] = p->pi[1] + p->t[1] * dL;
        pi1[2] = p->pi[2] + p->t[2] * dL;
        p->Li = p->Li + dL;
    } else {
        ////向前插补1个步长
        pi1[0] = p->pi[0] + p->t[0] * dL;
        pi1[1] = p->pi[1] + p->t[1] * dL;
        pi1[2] = p->pi[2] + p->t[2] * dL;
        p->Li = p->Li + dL;
    }

    p->pi[0] = pi1[0];
    p->pi[1] = pi1[1];
    p->pi[2] = pi1[2];
    return;
}

/**
 * @brief 			Description: Computes the parameters of both line path and orientation for interpolation.
 * @param[in]		p1				Start coordinates,including x,y,z coordinates and orientation angles roll-pitch-yaw angles.
 * @param[in]		p2				End coordinates,including x,y,z coordinates and orientation angles roll-pitch-yaw angles.
 * @param[out]		LPO				Parameters of both line path and orientation for interpolation.
 * @return			No return value.
 * @note:
 * @warning:
*/
////给定始末六维向量，计算初始插补矩阵Ts,期望插补矩阵Te，当前插补矩阵Ti
void InitialLinePOInpParam(double p1[6], double p2[6], LinePOParam *LPO) {
    double Rs[3][3];
    double Re[3][3];
    RPYToRot(p1[3], p1[4], p1[5], Rs);
    RPYToRot(p2[3], p2[4], p2[5], Re);
    InitialLinePathParam(p1, p2, &(LPO->Line));
    InitialOrientInpParam(Rs, Re, &(LPO->Orient));
    RpToTrans(Rs, p1, LPO->Ts);
    RpToTrans(Rs, p1, LPO->Ti);
    RpToTrans(Re, p2, LPO->Te);
    LPO->InpFlag = 1;
    return;
}

/**
 * @brief 			Description:Computes the line path interpolation coordinates and orientation in each interpolation cycle.
 * @param[out]		LPO				Line path and orientation parameters structure.
 * @param[in]		dL				Line path interpolation step length.
 * @param[out]		dtheta			angle interpolation step length for Orientation interpolation.
 * @return			No return value.
 * @note:
 * @warning:
*/
////将 InitialLinePOInpParam 初始化后的参数传入函数，计算每个插补周期中的线路径插补坐标和方向。
void LinePOInp(LinePOParam *LPO, double dL, double dtheta, double Ti[4][4]) {
    double pi[3];
    double Ri[3][3];
    ////Flag变为插补中
    LPO->InpFlag = 2;
    ////计算直线插补
    LinePathInp(&LPO->Line, dL, pi);
    ////计算四元数插补
    QuaternionOrientInp(&LPO->Orient, dtheta, Ri);
    if (LPO->Line.InpFlag == 3 && LPO->Orient.InpFlag == 3) {
        LPO->InpFlag = 3;
    }
    RpToTrans(Ri, pi, Ti);
    MatrixCopy((double *) Ti, 4, 4, (double *) LPO->Ti);
    return;
}

void VectorProduct(double a[3], double b[3], double c[3]) {
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
    return;
}

void MDHTrans(double alpha, double a, double d, double theta, double T[4][4]) {

    T[0][0] = cos(theta);
    T[0][1] = -sin(theta);
    T[0][2] = 0;
    T[0][3] = a;
    T[1][0] = sin(theta) * cos(alpha);
    T[1][1] = cos(theta) * cos(alpha);
    T[1][2] = -sin(alpha);
    T[1][3] = -sin(alpha) * d;
    T[2][0] = sin(theta) * sin(alpha);
    T[2][1] = cos(theta) * sin(alpha);
    T[2][2] = cos(alpha);
    T[2][3] = cos(alpha) * d;
    T[3][0] = 0;
    T[3][1] = 0;
    T[3][2] = 0;
    T[3][3] = 1;
    return;
}

void
DescartesTrans(MDHParam7 *MP, double T01[4][4], double T02[4][4], double T03[4][4], double T04[4][4], double T05[4][4],
               double T06[4][4], double T07[4][4]) {

    double T12[4][4];
    double T23[4][4];
    double T34[4][4];
    double T45[4][4];
    double T56[4][4];
    double T67[4][4];
    MDHTrans(MP->alpha[0], MP->a[0], MP->d[0], MP->theta[0], T01);
    MDHTrans(MP->alpha[1], MP->a[1], MP->d[1], MP->theta[1], T12);
    MDHTrans(MP->alpha[2], MP->a[2], MP->d[2], MP->theta[2], T23);
    MDHTrans(MP->alpha[3], MP->a[3], MP->d[3], MP->theta[3], T34);
    MDHTrans(MP->alpha[4], MP->a[4], MP->d[4], MP->theta[4], T45);
    MDHTrans(MP->alpha[5], MP->a[5], MP->d[5], MP->theta[5], T56);
    MDHTrans(MP->alpha[6], MP->a[6], MP->d[6], MP->theta[6], T67);
    Matrix4Mult(T01, T12, T02);
    Matrix4Mult(T02, T23, T03);
    Matrix4Mult(T03, T34, T04);
    Matrix4Mult(T04, T45, T05);
    Matrix4Mult(T05, T56, T06);
    Matrix4Mult(T06, T67, T07);
    return;
}

void
JacobSpace7(double T01[4][4], double T02[4][4], double T03[4][4], double T04[4][4], double T05[4][4],
            double T06[4][4], double T07[4][4], double Js7[6][7]) {


}

void JacobBody7(double T01[4][4], double T02[4][4], double T03[4][4], double T04[4][4], double T05[4][4],
                double T06[4][4], double T07[4][4], double Jb_7[6][7]) {
    double z0[3] = {0, 0, 1};
    double z[3][7];

    double o0[3] = {0, 0, 1};
    double o[3][7];
    double oee[3];

    double p[3][7];

    for (int i = 0; i < 3; i++) {
        z[i][0] = T01[i][2];
        z[i][1] = T02[i][2];
        z[i][2] = T03[i][2];
        z[i][3] = T04[i][2];
        z[i][4] = T05[i][2];
        z[i][5] = T06[i][2];
        z[i][6] = T07[i][2];
        o[i][0] = T01[i][3];
        o[i][1] = T02[i][3];
        o[i][2] = T03[i][3];
        o[i][3] = T04[i][3];
        o[i][4] = T05[i][3];
        o[i][5] = T06[i][3];
        o[i][6] = T07[i][3];
        oee[i] = T07[i][3];
    }

    double Jb[6][7];
    double cross[3][7],cross1[3],cross2[3],cross3[3],cross4[3],cross5[3],cross6[3],cross7[3];
    double a1[3],a2[3],a3[3],a4[3],a5[3],a6[3],a7[3];
    double b1[3],b2[3],b3[3],b4[3],b5[3],b6[3],b7[3];

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) {
            Jb[i][j] = z[i][j];
            p[i][j] = oee[i] - o[i][j];
        }
        a1[i] = z[i][0];
        a2[i] = z[i][1];
        a3[i] = z[i][2];
        a4[i] = z[i][3];
        a5[i] = z[i][4];
        a6[i] = z[i][5];
        a7[i] = z[i][6];
        b1[i] = p[i][0];
        b2[i] = p[i][1];
        b3[i] = p[i][2];
        b4[i] = p[i][3];
        b5[i] = p[i][4];
        b6[i] = p[i][5];
        b7[i] = p[i][6];
    }

    for (int i = 0; i < 3; i++) {
        VectorProduct(a1,b1,cross1);
        VectorProduct(a2,b2,cross2);
        VectorProduct(a3,b3,cross3);
        VectorProduct(a4,b4,cross4);
        VectorProduct(a5,b5,cross5);
        VectorProduct(a6,b6,cross6);
        VectorProduct(a7,b7,cross7);
        Jb[i+3][0] =cross1[i];
        Jb[i+3][1] =cross2[i];
        Jb[i+3][2] =cross3[i];
        Jb[i+3][3] =cross4[i];
        Jb[i+3][4] =cross5[i];
        Jb[i+3][5] =cross6[i];
        Jb[i+3][6] =cross7[i];
    }


}
