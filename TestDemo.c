/**
 * @brief			Description: Test Demos for robot algorithm modules.
 * @file:			TestDemo.c
 * @author:			Brian
 * @date:			2019/03/01 12:23
 * Copyright(c) 	2019 Brian. All rights reserved.
 *
 * Contact 			https://blog.csdn.net/Galaxy_Robot
 * @note:     
 * @warning: 		
*/


#include <stdio.h>
#include <math.h>
#include "RobotAlgorithmModule.h"


////测试自由度数，N是连杆数（包括大底连杆），m刚体提供的私自由度数（平面3，空间6），J是关节数，f是每个关节提供的自由度数
void test_GrublersFormula() {
    int N = 8;
    int m = 3;
    int J = 9;
    int f[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
    int dof = GrublersFormula(m, N, J, f);
    printf("dof=%d\n", dof);
}

////计算R的逆矩阵 ，InvR = R
void test_RotInv() {
    double R[3][3] =
            {
                    0, 0, 1,
                    1, 0, 0,
                    0, 1, 0
            };
    double InvR[3][3] = {{0}};
    RotInv(R, InvR);
    int i;
    printf("R:\n");
    for (i = 0; i < 3; i++) {
        printf("%lf %lf %lf\n", R[i][0], R[i][1], R[i][2]);
    }
    printf("InvR:\n");
    for (i = 0; i < 3; i++) {
        printf("%lf %lf %lf\n", InvR[i][0], InvR[i][1], InvR[i][2]);
    }
}


////SO(n)表示n*n的特殊正交群，多用于旋转正交基的表示；SE(n)表示所有刚性运动群组成的特殊欧式群(包括了平移与旋转)，SE(3) = R3 X SO(3)
////将三维速度向量转换为3*3的SO(3)矩阵
void test_VecToso3() {
    double omg[3] = {1, 2, 3};
    double so3[3][3] = {{0}};
    int i;
    VecToso3(omg, so3);
    printf("omg:\n");

    printf("%lf %lf %lf\n", omg[0], omg[1], omg[2]);

    printf("so3:\n");
    for (i = 0; i < 3; i++) {
        printf("%lf %lf %lf\n", so3[i][0], so3[i][1], so3[i][2]);
    }
}


////3*3的SO(3)矩阵转换为3将三维速度向量
void test_so3ToVec() {
    double so3[3][3] =
            {
                    0, -3, 2,
                    3, 0, -1,
                    -2, 1, 0
            };
    double omg[3] = {0};
    int i;
    so3ToVec(so3, omg);

    printf("so3:\n");
    for (i = 0; i < 3; i++) {
        printf("%lf %lf %lf\n", so3[i][0], so3[i][1], so3[i][2]);
    }
    printf("omg:\n");
    printf("%lf %lf %lf\n", omg[0], omg[1], omg[2]);

}


////输入的expc3参数是一组正交基的三个基的长度，输出的omghat其实是z，是单位化后的正交基长度，theta是原正交基向量合成长度
////返回值为omghat，theta
void test_AxisAng3() {
    double expc3[3] = {1, 2, 3};
    double omghat[3] = {0};
    double theta = 0;
    printf("omghat:\n");
    printf("%lf %lf %lf\n", omghat[0], omghat[1], omghat[2]);

    AxisAng3(expc3, omghat, &theta);
    printf("expc3:\n");
    printf("%lf %lf %lf\n", expc3[0], expc3[1], expc3[2]);
    printf("omghat:\n");
    printf("%lf %lf %lf\n", omghat[0], omghat[1], omghat[2]);
    printf("theta:\n");
    printf("%lf\n", theta);
    return;
}


////通过改变SO（3）的正交基，表示变换后的R
void test_MatrixExp3() {
    double so3mat[3][3] =
            {
                    0, -3, 2,
                    3, 0, -1,
                    -2, 1, 0
            };
    double R[3][3] = {{0}};
    MatrixExp3(so3mat, R);
    printf("R:\n");
    int i;
    for (i = 0; i < 3; i++) {
        printf("%lf %lf %lf\n", R[i][0], R[i][1], R[i][2]);
    }

    return;
}


////计算矩阵对数，详见知乎：https://www.zhihu.com/question/405831993/answer/1409378299
void test_MatrixLog3() {
    double theta = 30 * PI / 180;
    ////绕z轴旋转的旋转矩阵
    double R[3][3] =
            {
                    cos(theta), -sin(theta), 0,
                    sin(theta), cos(theta), 0,
                    0, 0, 1
            };
    double so3Mat[3][3] = {0};
    MatrixLog3(R, so3Mat);
    printf("so3Mat:\n");
    int i;
    for (i = 0; i < 3; i++) {
        printf("%lf %lf %lf\n", so3Mat[i][0], so3Mat[i][1], so3Mat[i][2]);
    }
    return;
}

////将平移与旋转旋量用T4*4表示
void test_RpToTrans() {
    double R[3][3] =
            {
                    1, 0, 0,
                    0, 0, -1,
                    0, 1, 0
            };
    double p[3] = {1, 2, 5};
    double T[4][4] = {{0}};
    RpToTrans(R, p, T);
    printf("T:\n");
    int i;
    for (i = 0; i < 4; i++) {
        printf("%lf %lf %lf %lf\n", T[i][0], T[i][1], T[i][2], T[i][3]);
    }
    return;
}

////把T4*4还原回平移与旋转旋量
void test_TransToRp() {
    double T[4][4] = {
            1, 0, 0, 0,
            0, 0, -1, 0,
            0, 1, 0, 3,
            0, 0, 0, 1
    };
    double R[3][3] = {{0}};
    double p[3] = {0};
    TransToRp(T, R, p);
    printf("R:\n");
    int i;
    for (i = 0; i < 3; i++) {
        printf("%lf %lf %lf\n", R[i][0], R[i][1], R[i][2]);
    }
    printf("p:\n");
    printf("%lf %lf %lf\n", p[0], p[1], p[2]);
    return;

}

////计算T4*4的逆
void test_TransInv() {
    double T[4][4] = {
            1, 0, 0, 0,
            0, 0, -1, 0,
            0, 1, 0, 3,
            0, 0, 0, 1
    };
    double InvT[4][4];
    TransInv(T, InvT);
    printf("InvT:\n");
    int i;
    for (i = 0; i < 4; i++) {
        printf("%lf %lf %lf %lf\n", InvT[i][0], InvT[i][1], InvT[i][2], InvT[i][3]);
    }
    return;
}

////将六维空间向量写成T4*4
void test_VecTose3() {
    double V[6] = {1, 2, 3, 4, 5, 6};
    double se3Mat[4][4];
    VecTose3(V, se3Mat);
    printf("se3Mat:\n");
    int i;
    for (i = 0; i < 4; i++) {
        printf("%lf %lf %lf %lf\n", se3Mat[i][0], se3Mat[i][1], se3Mat[i][2], se3Mat[i][3]);
    }
    return;
}


////将T4*4还原为六维空间向量
void test_se3ToVec() {
    double V[6] = {0};
    double se3Mat[4][4] = {
            0, -3, 2, 4,
            3, 0, -1, 5,
            -2, 1, 0, 6,
            0, 0, 0, 0
    };
    se3ToVec(se3Mat, V);
    int i;
    printf("V:\n");
    for (i = 0; i < 6; i++) {
        printf("%lf\n", V[i]);
    }
    return;
}

////T4*4的伴随矩阵
void test_Adjoint() {
    double T[4][4] = {
            1, 0, 0, 0,
            0, 0, -1, 0,
            0, 1, 0, 3,
            0, 0, 0, 1
    };
    double AdT[6][6] = {{0}};
    Adjoint(T, AdT);
    int i;
    int j;
    printf("AdT:\n");
    for (i = 0; i < 6; i++) {
        for (j = 0; j < 6; j++) {
            printf("%lf  ", AdT[i][j]);
        }
        printf("\n");
    }

    return;
}


////将六维空间向量归一化
void test_AxisAng6() {
    double expc6[6] = {1, 2, 3, 4, 5, 6};
    double S[6];
    double theta;
    AxisAng6(expc6, S, &theta);

    printf("S:\n");
    int i;
    for (i = 0; i < 6; i++) {
        printf("%lf\n", S[i]);
    }
    printf("theta:\n");
    printf("%lf\n", theta);
    return;
}


////把SE(3)转换为T4*4（矩阵指数）
void test_MatrixExp6() {
    double se3Mat[4][4] =
            {
                    0, 0, 0, 0,
                    0, 0, -1.570796, 2.3562,
                    0, 1.570796, 0, 2.3562,
                    0, 0, 0, 0
            };
    double T[4][4];
    MatrixExp6(se3Mat, T);
    int i;
    printf("T:\n");
    for (i = 0; i < 4; i++) {
        printf("%lf %lf %lf %lf\n", T[i][0], T[i][1], T[i][2], T[i][3]);
    }
    return;
}

////把T4*4转换为SE(3)（矩阵对数）
void test_MatrixLog6() {
    double T[4][4] =
            {
                    1, 0, 0, 0,
                    0, 0, -1, 0,
                    0, 1, 0, 3,
                    0, 0, 0, 1
            };
    double se3mat[4][4];
    MatrixLog6(T, se3mat);
    int i;
    printf("se3mat:\n");
    for (i = 0; i < 4; i++) {
        printf("%lf %lf %lf %lf\n", se3mat[i][0], se3mat[i][1], se3mat[i][2], se3mat[i][3]);
    }

    return;
}


////给定初始姿态M，给定Slist（相当于给定DH表），再给定theta，计算转动theta后的正解T (相对笛卡尔原点)
void test_FKinSpace() {
    int i;
    double M[4][4] = {
            1.0000, 0, 0, 213.0000,
            0, 1.0000, 0, 267.8000,
            0, 0, 1.0000, 478.9500,
            0, 0, 0, 1.0000};
    int JoinNum = 6;
    double Slist[6][6] = {
            0, 0, 0, 0, 0, 0,
            0, 1.0000, 1.0000, 1.0000, 0, 1.0000,
            1.0000, 0, 0, 0, 1.0000, 0,
            0, -151.9000, -395.5500, -395.5500, 110.4000, -478.9500,
            0, 0, 0, 0, -213.0000, 0,
            0, 0, 0, 213.0000, 0, 213.0000
    };
    //double thetalist[6] = {0,0,0,0,0,0};
    double thetalist[6] = {0.1, 0.2, 0.3, 0.4, 0.5, PI / 4};
    printf("thetalist:\n");
    for (i = 0; i < 6; i++) {
        printf("%lf ", thetalist[i]);
    }
    printf("\n");
    double T[4][4];
    FKinSpace(M, JoinNum, (double *) Slist, thetalist, T);

    printf("T:\n");
    for (i = 0; i < 4; i++) {
        printf("%lf %lf %lf %lf\n", T[i][0], T[i][1], T[i][2], T[i][3]);
    }
    return;
}


////给定初始姿态M，给定Blist（相当于给定DH表），再给定theta，计算转动theta后的正解T (相对ee原点)
void test_FKinBody() {
    double M[4][4] = {
            -1, 0, 0, 0,
            0, 1, 0, 6,
            0, 0, -1, 2,
            0, 0, 0, 1};
    int JoinNum = 3;
    //double Blist[3][6] = {
    //	0,	 0,  -1, 2, 0, 0,
    //	0,   0,   0, 0, 1, 0,
    //	0,   0,   1, 0, 0, 0.1 };
    double Blist[6][3] = {
            0, 0, 0,
            0, 0, 0,
            -1, 0, 1,
            2, 0, 0,
            0, 1, 0,
            0, 0, 0.1
    };
    double thetalist[3] = {PI / 2, 3, PI};
    double T[4][4];
    FKinBody(M, JoinNum, (double *) Blist, thetalist, T);

    int i;
    printf("T:\n");
    for (i = 0; i < 4; i++) {
        printf("%lf %lf %lf %lf\n", T[i][0], T[i][1], T[i][2], T[i][3]);
    }
    return;
}


////计算ee空间下的雅克比矩阵
void test_JacobianBody() {
    int JointNum = 4;//JointNum must be the actual number of joints.
    int i;
    int j;
    double Blist[6][4];
    double Bi[6][4] = {
            0, 1, 0, 1,
            0, 0, 1, 0,
            1, 0, 0, 0,
            0, 2, 0, 0.2,
            0.2, 0, 2, 0.3,
            0.2, 3, 1, 0.4
    };
    //initial Blist.
    for (i = 0; i < 6; i++) {
        for (j = 0; j < JointNum; j++) {
            Blist[i][j] = Bi[i][j];
        }
    }
    //initial thetalist.
    double thetalist[4] = {0.2, 1.1, 0.1, 1.2};
    double Jb[6][4] = {{0}};
    JacobianBody(JointNum, (double *) Blist, thetalist, (double *) Jb);

    printf("Jb:\n");

    for (i = 0; i < 6; i++) {
        for (j = 0; j < JointNum; j++) {
            printf("%lf  ", Jb[i][j]);
        }
        printf("\n");
    }
    return;
}


////计算笛卡尔空间下的雅克比矩阵
void test_JacobianSpace() {
    int JointNum = 4;//JointNum must be the actual number of joints.
    int i;
    int j;
    double Slist[6][4];
    double Si[6][4] = {
            0, 1, 0, 1,
            0, 0, 1, 0,
            1, 0, 0, 0,
            0, 2, 0, 0.2,
            0.2, 0, 2, 0.3,
            0.2, 3, 1, 0.4
    };
    //initial Blist.
    for (i = 0; i < 6; i++) {
        for (j = 0; j < JointNum; j++) {
            Slist[i][j] = Si[i][j];
        }
    }
    //initial thetalist.
    double thetalist[4] = {0.2, 1.1, 0.1, 1.2};
    double Js[6][4] = {{0}};
    JacobianSpace(JointNum, (double *) Slist, thetalist, (double *) Js);

    printf("Js:\n");

    for (i = 0; i < 6; i++) {
        for (j = 0; j < JointNum; j++) {
            printf("%lf  ", Js[i][j]);
        }
        printf("\n");
    }

    return;
}


////复制矩阵
void test_MatrixCopy() {
    double a[3][4] = {
            1, 2, 3, 4,
            5, 6, 7, 8,
            9, 10, 11, 12
    };
    double b[3][4];
    MatrixCopy((double *) a, 3, 4, (double *) b);
    int i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 4; j++) {
            printf("%lf\t", b[i][j]);
        }
        printf("\n");
    }


    return;
}

////svd奇异值分解
void test_svdcmp() {
    double a[4][4] = {
            1, 2, 3, 4,
            5, 6, 7, 8,
            9, 10, 11, 12,
            13, 14, 15, 16
    };
    double w[4], v[4][4];
    double tol = 2.2e-15;
    svdcmp((double *) a, 4, 4, tol, w, (double *) v);
    int i, j, k;
    printf("svdcmp,u:\n");
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            printf("%lf   ", a[i][j]);
        }
        printf("\n");
    }
    printf("w:\n");
    for (i = 0; i < 1; i++) {
        for (j = 0; j < 4; j++) {
            printf("%lf   ", w[j]);
        }
        printf("\n");
    }
    printf("v:\n");
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            printf("%lf   ", v[i][j]);
        }
        printf("\n");
    }
    double vT[4][4];
    //验证分解是否正确
    MatrixT((double *) v, 4, 4, (double *) vT);
    double tmp[4][4];
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            tmp[i][j] = 0;
            for (k = 0; k < 4; k++) {
                tmp[i][j] = tmp[i][j] + w[k] * a[i][k] * vT[k][j];
            }
        }
    }
    printf("uwvT:\n");
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            printf("%lf   ", tmp[i][j]);
        }
        printf("\n");
    }


    return;
}


////计算矩阵转置
void test_MatrixT() {
    double a[4][3] = {
            1, 2, 3,
            4, 5, 6,
            7, 8, 9,
            10, 11, 12
    };
    double c[3][4];
    MatrixT((double *) a, 4, 3, (double *) c);
    int i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 4; j++) {
            printf("%lf  ", c[i][j]);
        }
        printf("\n");
    }

    return;
}


////计算矩阵A*B
void test_MatrixMult() {
    double a[4][3] = {
            1, 2, 3,
            4, 5, 6,
            7, 8, 9,
            10, 11, 12
    };
    double b[3][2] = {
            1, 2,
            4, 5,
            7, 8
    };
    double c[4][2];
    MatrixMult((double *) a, 4, 3, (double *) b, 2, (double *) c);
    int i, j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 2; j++) {
            printf("%lf  ", c[i][j]);
        }
        printf("\n");
    }

    return;
}


////计算矩阵伪逆（svd）
void test_MatrixPinv() {
    double a[4][3] = {
            1, 2, 3,
            5, 6, 7,
            9, 10, 11,
            13, 14, 15
    };
    double b[3][4];
    double tol = 2.22e-15;
    MatrixPinv((double *) a, 4, 3, tol, (double *) b);
    printf("MatrixPinv(a):\n");
    int i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 4; j++) {
            printf("%lf  ", b[i][j]);
        }
        printf("\n");
    }

    return;
}

////计算矩阵伪逆（svd）
void test_MatrixPinv67() {
    double a[6][7] = {
            1, 2, 3, 4, 5, 6, 9,
            5, 6, 7, 8, 9, 9, 93,
            9, 10, 11, 15, 16, 17, 55,
            13, 14, 15, 16, 17, 18, 66,
            9, 8, 7, 1, 3, 2, 44,
            88, 22, 66, 33, 11, 44, 11
    };
    double b[7][6];
    double tol = 2.22e-15;
    MatrixPinv((double *) a, 6, 7, tol, (double *) b);
    printf("MatrixPinv(a):\n");
    int i, j;
    for (i = 0; i < 6; i++) {
        for (j = 0; j < 7; j++) {
            printf("%lf  ", b[i][j]);
        }
        printf("\n");
    }

    return;
}


////计算ee坐标系下的逆运动学
void test_IKinBodyNR() {

    int JointNum = 2;
    double Blist[6][2] = {
            0, 0,
            0, 0,
            1, 1,
            0, 0,
            2, 1,
            0, 0,
    };
    double M[4][4] =
            {
                    1, 0, 0, 2,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1
            };
    double T[4][4] = {
            -0.5, -0.866, 0, 0.366,
            0.866, -0.5, 0, 1.366,
            0, 0, 1, 0,
            0, 0, 0, 1
    };
    double thetalist0[2] = {0, 0};
    double eomg = 0.001;
    double ev = 0.0001;
    double thetalist[2] = {0};
    int maxiter = 20;
    int ret = IKinBodyNR(JointNum, (double *) Blist, M, T, thetalist0, eomg, ev, maxiter, thetalist);
    if (ret) {
        printf("IKinBody error %d\n", ret);
        return;
    }
    printf("solution thetalist:\n");
    int i;
    for (i = 0; i < JointNum; i++) {
        printf("%lf  ", thetalist[i]);
    }
    printf("\n");
    return;
}


////计算笛卡尔坐标系下的逆运动学
void test_IKinSpaceNR() {

    int JointNum = 3;
    double Slist[6][3] = {
            0, 0, 0,
            0, 0, 0,
            1, 0, -1,
            4, 0, -6,
            0, 1, 0,
            0, 0, -0.1
    };
    double M[4][4] =
            {
                    -1, 0, 0, 0,
                    0, 1, 0, 6,
                    0, 0, -1, 2,
                    0, 0, 0, 1
            };
    double T[4][4] = {
            0, 1, 0, -5,
            1, 0, 0, 4,
            0, 0, -1, 1.6858,
            0, 0, 0, 1
    };
    double thetalist0[3] = {1.5, 2.5, 3};
    double eomg = 0.01;
    double ev = 0.001;
    double thetalist[3] = {0};
    int maxiter = 20;
    int ret = IKinSpaceNR(JointNum, (double *) Slist, M, T, thetalist0, eomg, ev, maxiter, thetalist);
    if (ret) {
        printf("IKinSpace error %d\n", ret);
        return;
    }
    printf("solution thetalist:\n");
    int i;
    for (i = 0; i < JointNum; i++) {
        printf("%lf  ", thetalist[i]);
    }
    printf("\n");
    return;
}


////计算笛卡尔坐标系下的UR3逆运动学
void test_IKOnUR3() {
    int JointNum = 6;
    double Slist[6][6] =
            {
                    0, 0, 0, 0, 0, 0,
                    0, 1.0000, 1.0000, 1.0000, 0, 1.0000,
                    1.0000, 0, 0, 0, 1.0000, 0,
                    0, -151.9000, -395.5500, -395.5500, 110.4000, -478.9500,
                    0, 0, 0, 0, -213.0000, 0,
                    0, 0, 0, 213.0000, 0, 213.0000
            };
    double M[4][4] =
            {
                    1, 0, 0, 213.0000,
                    0, 1, 0, 267.8000,
                    0, 0, 1, 478.9500,
                    0, 0, 0, 1,
            };
    double T[4][4] =
            {
                    1, 0, 0, 10.0000,
                    0, 0, 1, 375.0000,
                    0, -1, 0, 200.0000,
                    0, 0, 0, 1.0000
            };
//    double thetalist0[6] = {0, 0, 0, 0, 0, 0};
    double thetalist0[6] = {-1, -1, -1, 0, 1, -2};
    double eomg = 1.0E-4;
    double ev = 1.0E-4;
    double thetalist[6] = {0};
    int maxiter = 200;
    int ret = IKinSpaceNR(JointNum, (double *) Slist, M, T, thetalist0, eomg, ev, maxiter, thetalist);
    if (ret) {
        printf("IKinSpace error %d\n", ret);
        return;
    }
    printf("solution thetalist for C(单位：弧度):\n");
    int i;
    for (i = 0; i < JointNum; i++) {
        printf("%lf, ", thetalist[i]);
    }
    printf("\n\n");
    return;
}


////输入一个旋转矩阵R,计算旋转矩阵相对于原坐标系(1,1,1)以哪个轴旋转，返回该轴的单位向量，并计算出旋转角度
void test_RotToAxisAng() {
    //绕z轴旋转30度的旋转矩阵.
    double R[3][3] =
            {
                    cos(PI / 6.0), -sin(PI / 6.0), 0,
                    sin(PI / 6.0), cos(PI / 6.0), 0,
                    0, 0, 1
            };
    double omg[3];
    double theta;
    RotToAxisAng(R, omg, &theta);
    printf("omg:\n%lf\n%lf\n%lf\n", omg[0], omg[1], omg[2]);
    printf("theta:\n%lf\n", theta);
    return;
}


////给定旋转轴omg和转角theta，计算单位四元数
void test_AxisAngToQuaternion() {
    double R[3][3] = {0};
    //绕z轴旋转30度的欧拉轴和角度.
    double omg[3] = {0, 0, 1};
    double theta = PI / 6.0;
    double q[4];
    AxisAngToQuaternion(omg, theta, q);
    printf("q:\n%lf\n%lf\n%lf\n%lf\n", q[0], q[1], q[2], q[3]);
    return;
}


////给定单位四元数，计算旋转矩阵R
void test_QuaternionToRot() {
    double theta = PI / 6;
    //绕z轴旋转30度的四元数.
    double q[4] = {cos(theta / 2), 0 * sin(theta / 2), 0 * sin(theta / 2), 1.0 * sin(theta / 2)};
    double R[3][3];
    QuaternionToRot(q, R);
    int i;
    printf("R:\n");
    for (i = 0; i < 3; i++) {
        printf("%lf %lf %lf\n", R[i][0], R[i][1], R[i][2]);
    }
    return;
}

////给定旋转矩阵R，计算单位四元数
void test_RotToQuaternion() {
    //绕z轴旋转30度的旋转矩阵.
    double R[3][3] =
            {
                    cos(PI / 6.0), -sin(PI / 6.0), 0,
                    sin(PI / 6.0), cos(PI / 6.0), 0,
                    0, 0, 1
            };
    double q[4];
    RotToQuaternion(R, q);
    int i;
    printf("q:\n");
    for (i = 0; i < 4; i++) {
        printf("%lf\n", q[i]);
    }
    return;
}

////给定初始和末端Rs Re，计算R，并返回转轴转角等参数，详见结构体OrientInpParam
void test_InitialOrientInpParam() {
    double Rs[3][3] =
            {
                    cos(PI / 6.0), -sin(PI / 6.0), 0,
                    sin(PI / 6.0), cos(PI / 6.0), 0,
                    0, 0, 1
            };
    double Re[3][3] =
            {
                    cos(PI / 3.0), -sin(PI / 3.0), 0,
                    sin(PI / 3.0), cos(PI / 3.0), 0,
                    0, 0, 1
            };
    OrientInpParam p;
    InitialOrientInpParam(Rs, Re, &p);
    int i;
    printf("OrientInpParam Rs:\n");
    for (i = 0; i < 3; i++) {
        printf("%lf %lf %lf\n", p.Rs[i][0], p.Rs[i][1], p.Rs[i][2]);
    }
    printf("OrientInpParam Re:\n");
    for (i = 0; i < 3; i++) {
        printf("%lf %lf %lf\n", p.Re[i][0], p.Re[i][1], p.Re[i][2]);
    }
    printf("OrientInpParam R:\n");
    for (i = 0; i < 3; i++) {
        printf("%lf %lf %lf\n", p.R[i][0], p.R[i][1], p.R[i][2]);
    }
    printf("OrientInpParam omg:\n");
    printf("%lf %lf %lf\n", p.omg[0], p.omg[1], p.omg[2]);

    printf("OrientInpParam theta:\n");
    printf("%lf\n", p.theta);
    printf("OrientInpParam thetai:\n");
    printf("%lf\n", p.thetai);

    printf("OrientInpParam Ri:\n");
    for (i = 0; i < 3; i++) {
        printf("%lf %lf %lf\n", p.Ri[i][0], p.Ri[i][1], p.Ri[i][2]);
    }

    return;

}


////四元数插补
void test_QuaternionOrientInp() {
    double Rs[3][3] =
            {
                    cos(PI / 6.0), -sin(PI / 6.0), 0,
                    sin(PI / 6.0), cos(PI / 6.0), 0,
                    0, 0, 1
            };
    double Re[3][3] =
            {
                    cos(PI / 3.0), -sin(PI / 3.0), 0,
                    sin(PI / 3.0), cos(PI / 3.0), 0,
                    0, 0, 1
            };
    OrientInpParam p;
    double Ri[3][3];
    MatrixCopy((double *) Rs, 3, 3, (double *) Ri);
    InitialOrientInpParam(Rs, Re, &p);
    double dtheta = 0.01;
    int k = 0;
    printf("Orientation Ri:k=%d\n", k);
    int i;
    for (i = 0; i < 3; i++) {
        printf("%lf %lf %lf\n", Ri[i][0], Ri[i][1], Ri[i][2]);
    }
    //Orientation interpolation
    while (p.InpFlag != 3 && p.InpFlag != 0) {
        k++;
        QuaternionOrientInp(&p, dtheta, Ri);
        printf("Orientation Ri:k=%d\n", k);
        for (i = 0; i < 3; i++) {
            printf("%lf %lf %lf\n", Ri[i][0], Ri[i][1], Ri[i][2]);
        }
    }
    return;
}

////位置变换参数初始化
void test_InitialLinePathParam() {
    double p1[3] = {0, 0, 0};
    double p2[3] = {1, 2, 3};
    double dL = 0.05;
    LineInpParam p;
    InitialLinePathParam(p1, p2, &p);
    printf("LinePathParam p1;\n");
    printf("%lf\n%lf\n%lf\n", p.p1[0], p.p1[1], p.p1[2]);
    printf("LinePathParam p2;\n");
    printf("%lf\n%lf\n%lf\n", p.p2[0], p.p2[1], p.p2[2]);
    printf("LinePathParam L;\n%lf\n", p.L);
    printf("LinePathParam Li;\n%lf\n", p.Li);
    printf("LinePathParam pi;\n");
    printf("%lf\n%lf\n%lf\n", p.pi[0], p.pi[1], p.pi[2]);
    printf("LinePathParam InpFlag;\n%d\n", p.InpFlag);

    return;
}

////位置变换插补
void test_LinePathInp() {
    double p1[3] = {0, 0, 0};
    double p2[3] = {1, 2, 3};
    LineInpParam p;
    double pi[3] = {p1[0], p1[2], p1[2]};
    InitialLinePathParam(p1, p2, &p);
    double dL = 0.05;
    int k = 0;
    printf("LinePathInp pi: k=%d,%lf %lf %lf\n", k, pi[0], pi[1], pi[2]);
    while (p.InpFlag != 3 && p.InpFlag != 0) {
        k++;
        LinePathInp(&p, dL, pi);
        printf("LinePathInp pi: k=%d,%lf %lf %lf\n", k, pi[0], pi[1], pi[2]);
    }
    return;
}

////笛卡尔插补
void test_LinePOInp() {
    //double p1[6] = { 213.0,267.8,478.95,0,0,0 };
    //double p2[6] = { 10,425,200, -PI / 2,0 ,0 };
    //double p1[6] = { 10,425,200, -PI / 2,0 ,0 };
    //double p2[6] = { -10,525,200,-PI / 4,0,-PI / 6 };
    double p1[6] = {10, 425, 200, -PI / 2, 0, 0};
    double p2[6] = {10, 425, 200, -PI * 3 / 4, 0, PI / 2};
    double Ti[4][4];
    double dL = 1;
    FILE *fp1;
    int ret = fopen_s(&fp1, "LineTrajectory.txt", "w");
    if (ret) {
        printf("fopen_s error %d\n", ret);
    }
    LinePOParam pt;
    InitialLinePOInpParam(p1, p2, &pt);
    //double dtheta =pt.Orient.theta/(pt.Line.L / dL);
    double dtheta = PI / 100;
    int JointNum = 6;
    double Slist[6][6] = {
            0, 0, 0, 0, 0, 0,
            0, 1.0000, 1.0000, 1.0000, 0, 1.0000,
            1.0000, 0, 0, 0, 1.0000, 0,
            0, -151.9000, -395.5500, -395.5500, 110.4000, -478.9500,
            0, 0, 0, 0, -213.0000, 0,
            0, 0, 0, 213.0000, 0, 213.0000
    };
    double M[4][4] =
            {
                    1.0000, 0, 0, 213.0000,
                    0, 1.0000, 0, 267.8000,
                    0, 0, 1.0000, 478.9500,
                    0, 0, 0, 1.0000,
            };
    //double thetalist0[6] = { 0 };
    double thetalist0[6] = {1.284569, 0.488521, -0.443200, 1.525477, -1.570797, -0.286227};

    double thetalist[6];
    double eomg = 0.001;
    double ev = 0.01;
    while (pt.InpFlag != 3) {
        ////计算出每个插补矩阵T
        LinePOInp(&pt, dL, dtheta, Ti);
        ////数值逆解
        IKinSpaceNR(JointNum, (double *) Slist, M, Ti, thetalist0, eomg, ev, 10, thetalist);
        //MatrixCopy((double *)Ti, 4, 4, (double *)M);
        MatrixCopy(thetalist, 6, 1, thetalist0);
        fprintf(fp1, "%lf %lf %lf %lf %lf %lf\n", thetalist[0], thetalist[1], thetalist[2], thetalist[3], thetalist[4],
                thetalist[5]);
    }
    fclose(fp1);
    return;
}


////将T4*4还原为六维空间向量
void test_HITse3ToVec1() {
    double V[6] = {0};
    double se3Mat[4][4] = {
            1.0000, 0, 0, 613.3800,
            0, -0.8660, 0.5000, 154.9957,
            0, -0.5000, -0.8660, -485.9044,
            0, 0, 0, 1.0000,
    };
    se3ToVec(se3Mat, V);
    int i;
    printf("V:\n");
    for (i = 0; i < 6; i++) {
        printf("%lf\n", V[i]);
    }
    return;
}


////将T4*4还原为六维空间向量
void test_HITse3ToVec2() {
    double V[6] = {0};
    double se3Mat[4][4] = {
            0.000000, -0.999978, 0.000013, 179.997287,
            -1.000000, -0.000000, 0.000000, -280.004300,
            -0.000000, -0.000013, -0.999978, -664.995554,
            0.000000, 0.000000, 0.000000, 1.000000,
    };
    se3ToVec(se3Mat, V);
    int i;
    printf("V:\n");
    for (i = 0; i < 6; i++) {
        printf("%lf\n", V[i]);
    }
    return;
}


////给定初始姿态M，给定Slist（相当于给定DH表），再给定theta，计算转动theta后的正解T (相对笛卡尔原点)
void test_HITEXOFKinSpace() {
    int i;
    double M[4][4] = {
            0.000000, -0.999978, 0.000013, 179.997287,
            -1.000000, -0.000000, 0.000000, -280.004300,
            -0.000000, -0.000013, -0.999978, -664.995554,
            0.000000, 0.000000, 0.000000, 1.000000,
    };

    int JoinNum = 7;

    double Slist[6][7] =
            {
                    0, 0, 0, 0, 0, 0, 0,
                    0, 1.0000, -0.8321, 0, 0.8660, 0.8660, 0.5,
                    1.0000, 0, 0.5547, 1.0000, 0.5000, 0.5000, -0.8660,
                    0, 0, -180.9318, -0.0046, 188.2981, 188.2981, 108.7264,
                    0, 0, -184.9259, -333.3800, -166.6900, -306.6900, 531.1871,
                    0, 0, -277.4055, 0, 288.7071, 531.1871, 306.6900
            };

    //double thetalist[7] = {0, 0, 0, 0, 0, 0, 0};
    double thetalist[7] = {1 * PI / 180, 1 * PI / 180, 1 * PI / 180, 1 * PI / 180, 1 * PI / 180, 1 * PI / 180,
                           1 * PI / 180};

    printf("thetalist:\n");
    for (i = 0; i < 7; i++) {
        printf("%lf ", thetalist[i]);
    }
    printf("\n");
    double T[4][4];
    FKinSpace(M, JoinNum, (double *) Slist, thetalist, T);

    printf("T:\n");
    for (i = 0; i < 4; i++) {
        printf("%lf %lf %lf %lf\n", T[i][0], T[i][1], T[i][2], T[i][3]);
    }
    return;
}


////计算笛卡尔坐标系下的HITEXO逆运动学
void test_IKOnHITEXO() {
    int JointNum = 7;
    double Slist[6][7] =
            {
                    0.000000, 0.000000, 0.000000, 0.000000, 0.866000, 0.866000, 0.500000,
                    0.000000, 1.000000, -0.832100, 0.000000, 0.000000, 0.000000, 0.000000,
                    1.000000, 0.000000, 0.554700, 1.000000, 0.500000, 0.500000, -0.866000,
                    0.000000, 0.000000, -180.931800, -0.004600, -0.002300, -140.002300, 242.484004,
                    0.000000, 0.000000, -184.925900, -333.380000, -354.990400, -354.990400, 179.984664,
                    0.000000, 0.000000, -277.405500, 0.000000, 0.004004, 242.484004, 140.002300
            };
    double M[4][4] = {
            0.000000, -0.999978, 0.000013, 179.997287,
            -1.000000, -0.000000, 0.000000, -280.004300,
            -0.000000, -0.000013, -0.999978, -664.995554,
            0.000000, 0.000000, 0.000000, 1.000000,
    };

    double T[4][4] = {
            0.046916, -0.998002, -0.041805, 170.935232,
            -0.998899, -0.046870, -0.002066, -280.263084,
            0.000102, 0.041856, -0.999102, -657.265194,
            0.000000, 0.000000, 0.000000, 1.000000
    };

    double thetalist0[7] = {0, 0, 0, 0, 0, 0, 0};
    double eomg = 1.0E-5;
    double ev = 1.0E-5;
    double thetalist[7] = {0};
    int maxiter = 50;
    int ret = IKinSpaceNR_DLS(JointNum,  * Slist, M, T, thetalist0, eomg, ev, maxiter, thetalist);
    if (ret) {
        printf("IKinSpace error %d\n", ret);
        return;
    }
    printf("solution thetalist for C(单位：弧度):\n");
    int i;
    for (i = 0; i < JointNum; i++) {
        printf("%lf, ", thetalist[i]);
    }
    printf("\n\n");
    return;
}


////笛卡尔插补
void test_HITLinePOInp() {

    double p1[6] = {613.380000, 154.995700, -485.904400, -0.500000, 0, 0};
    double p2[6] = {179.997287, -280.004300, -664.995554, -0.000013, 0.000013, -1.000000};
    double Ti[4][4];
    double dL = 1;
    double dtheta = PI / 100;

    LinePOParam pt;
    InitialLinePOInpParam(p1, p2, &pt);
    int JointNum = 7;
    while (pt.InpFlag != 3) {
        ////计算出每个插补矩阵T
        LinePOInp(&pt, dL, dtheta, Ti);
        int i;
        for (int i = 0; i < 4; i++) {
            //printf("Ti:\n");
            if ((i) % 4 == 0) {
                printf("Ti:\n");
            }
            printf("%lf %lf %lf %lf\n", Ti[i][0], Ti[i][1], Ti[i][2], Ti[i][3]);
        }
    }

    return;
}


////笛卡尔插补+逆解  不更新J
void test_HITLinePOInp_IK1() {
//    ////把T转换为SE3
//    MatrixLog6(double T[4][4], double se3Mat[4][4]);
//    ////将SE3转换成六维空间向量
//    se3ToVec(double se3Mat[4][4], double V[6]);
    ////初始姿态
    double M[4][4] =
            {
                    0.000000, -0.999978, 0.000013, 179.997287,
                    -1.000000, -0.000000, 0.000000, -280.004300,
                    -0.000000, -0.000013, -0.999978, -664.995554,
                    0.000000, 0.000000, 0.000000, 1.000000,
            };
    ////期望姿态
    double T[4][4] = {
            0.000000, -0.999978, 0.000013, 169.997287,
            -1.000000, -0.000000, 0.000000, -240.004300,
            -0.000000, -0.000013, -0.999978, -634.995554,
            0.000000, 0.000000, 0.000000, 1.000000,
    };
    double R[3][3], p[3];
    double roll, pitch, yaw;
    double p1[6], p2[6];

    ////把T还原回平移与旋转旋量
    TransToRp(M, R, p);
    ////将roll, pitch, yaw 三个角度转换为R
    RotToRPY(R, &roll, &pitch, &yaw);
    p1[0] = p[0];
    p1[1] = p[1];
    p1[2] = p[2];
    p1[3] = roll;
    p1[4] = pitch;
    p1[5] = yaw;

    printf("p1:\n");
    printf("%lf %lf %lf %lf %lf %lf\n", p1[0], p1[1], p1[2], p1[3], p1[4], p1[5]);

    ////把T还原回平移与旋转旋量
    TransToRp(T, R, p);
    ////将roll, pitch, yaw 三个角度转换为R
    RotToRPY(R, &roll, &pitch, &yaw);
    p2[0] = p[0];
    p2[1] = p[1];
    p2[2] = p[2];
    p2[3] = roll;
    p2[4] = pitch;
    p2[5] = yaw;

    printf("p2:\n");
    printf("%lf %lf %lf %lf %lf %lf\n", p2[0], p2[1], p2[2], p2[3], p2[4], p2[5]);

    double Ti[4][4];
    double dL = 1;
    FILE *fp1;
    int ret = fopen_s(&fp1, "LineTrajectory.txt", "w");
    if (ret) {
        printf("fopen_s error %d\n", ret);
    }
    LinePOParam pt;
    InitialLinePOInpParam(p1, p2, &pt);
    //double dtheta =pt.Orient.theta/(pt.Line.L / dL);
    double dtheta = PI / 100;
    int JointNum = 7;
    double Slist[6][7] =
            {
                    0.000000, 0.000000, 0.000000, 0.000000, 0.866000, 0.866000, 0.500000,
                    0.000000, 1.000000, -0.832100, 0.000000, 0.000000, 0.000000, 0.000000,
                    1.000000, 0.000000, 0.554700, 1.000000, 0.500000, 0.500000, -0.866000,
                    0.000000, 0.000000, -180.931800, -0.004600, -0.002300, -140.002300, 242.484004,
                    0.000000, 0.000000, -184.925900, -333.380000, -354.990400, -354.990400, 179.984664,
                    0.000000, 0.000000, -277.405500, 0.000000, 0.004004, 242.484004, 140.002300
            };

    double thetalist0[7] = {0, 0, 0, 0, 0, 0, 0};
    double thetalist[7];
    double eomg = 0.0001;
    double ev = 0.0001;
    while (pt.InpFlag != 3) {
        ////计算出每个插补矩阵T
        LinePOInp(&pt, dL, dtheta, Ti);
        for (int i = 0; i < 4; i++) {
            if ((i) % 4 == 0) {
                printf("Ti:\n");
            }
            printf("%lf %lf %lf %lf\n", Ti[i][0], Ti[i][1], Ti[i][2], Ti[i][3]);
        }
        ////数值逆解
        IKinSpaceNR_DLS(JointNum, (double *) Slist, M, Ti, thetalist0, eomg, ev, 20, thetalist);
        MatrixCopy(thetalist, 7, 1, thetalist0);
        printf("solution thetalist for C(单位：弧度):\n");
        int i;
        for (i = 0; i < JointNum; i++) {
            printf("%lf, ", thetalist[i]);
        }
        printf("\n\n");
    }
    fclose(fp1);
    return;
}


////笛卡尔插补+逆解  更新J
void test_HITLinePOInp_IK2() {
//    ////把T转换为SE3
//    MatrixLog6(double T[4][4], double se3Mat[4][4]);
//    ////将SE3转换成六维空间向量
//    se3ToVec(double se3Mat[4][4], double V[6]);
    ////初始姿态
    double M[4][4] =
            {
                    0.000000, -0.999978, 0.000013, 179.997287,
                    -1.000000, -0.000000, 0.000000, -280.004300,
                    -0.000000, -0.000013, -0.999978, -664.995554,
                    0.000000, 0.000000, 0.000000, 1.000000,
            };
    ////期望姿态
    double T[4][4] = {
            0.000000, -0.999978, 0.000013, 169.997287,
            -1.000000, -0.000000, 0.000000, -240.004300,
            -0.000000, -0.000013, -0.999978, -634.995554,
            0.000000, 0.000000, 0.000000, 1.000000,
    };
    double R[3][3], p[3];
    double roll, pitch, yaw;
    double p1[6], p2[6];

    ////把T还原回平移与旋转旋量
    TransToRp(M, R, p);
    ////将roll, pitch, yaw 三个角度转换为R
    RotToRPY(R, &roll, &pitch, &yaw);
    p1[0] = p[0];
    p1[1] = p[1];
    p1[2] = p[2];
    p1[3] = roll;
    p1[4] = pitch;
    p1[5] = yaw;

    printf("p1:\n");
    printf("%lf %lf %lf %lf %lf %lf\n", p1[0], p1[1], p1[2], p1[3], p1[4], p1[5]);

    ////把T还原回平移与旋转旋量
    TransToRp(T, R, p);
    ////将roll, pitch, yaw 三个角度转换为R
    RotToRPY(R, &roll, &pitch, &yaw);
    p2[0] = p[0];
    p2[1] = p[1];
    p2[2] = p[2];
    p2[3] = roll;
    p2[4] = pitch;
    p2[5] = yaw;

    printf("p2:\n");
    printf("%lf %lf %lf %lf %lf %lf\n", p2[0], p2[1], p2[2], p2[3], p2[4], p2[5]);

    double Ti[4][4];
    double dL = 1;
    FILE *fp1;
    int ret = fopen_s(&fp1, "LineTrajectory.txt", "w");
    if (ret) {
        printf("fopen_s error %d\n", ret);
    }
    LinePOParam pt;
    InitialLinePOInpParam(p1, p2, &pt);
    //double dtheta =pt.Orient.theta/(pt.Line.L / dL);
    double dtheta = PI / 100;
    int JointNum = 7;
    double Slist[6][7] =
            {
                    0.000000, 0.000000, 0.000000, 0.000000, 0.866000, 0.866000, 0.500000,
                    0.000000, 1.000000, -0.832100, 0.000000, 0.000000, 0.000000, 0.000000,
                    1.000000, 0.000000, 0.554700, 1.000000, 0.500000, 0.500000, -0.866000,
                    0.000000, 0.000000, -180.931800, -0.004600, -0.002300, -140.002300, 242.484004,
                    0.000000, 0.000000, -184.925900, -333.380000, -354.990400, -354.990400, 179.984664,
                    0.000000, 0.000000, -277.405500, 0.000000, 0.004004, 242.484004, 140.002300
            };

    double thetalist0[7] = {0, 0, 0, 0, 0, 0, 0};
    double thetalist[7];
    double eomg = 0.0001;
    double ev = 0.0001;
    while (pt.InpFlag != 3) {
        ////计算出每个插补矩阵T
        LinePOInp(&pt, dL, dtheta, Ti);
        for (int i = 0; i < 4; i++) {
            if ((i) % 4 == 0) {
                printf("Ti:\n");
            }
            printf("%lf %lf %lf %lf\n", Ti[i][0], Ti[i][1], Ti[i][2], Ti[i][3]);
        }
        ////数值逆解
        IKinSpaceNR_DLS_New(JointNum, (double *) Slist, M, Ti, thetalist0, eomg, ev, 20, thetalist);
        MatrixCopy(thetalist, 7, 1, thetalist0);
        printf("solution thetalist for C(单位：弧度):\n");
        int i;
        for (i = 0; i < JointNum; i++) {
            printf("%lf, ", thetalist[i]);
        }
        printf("\n\n");
    }
    fclose(fp1);
    return;
}

void test_1() {
    int JointNum = 7;

    double Slist[6][7] =
            {
                    0, 0, 0, 0, 0, 0, 0,
                    0, 1.0000, -0.8321, 0, 0.8660, 0.8660, 0.5,
                    1.0000, 0, 0.5547, 1.0000, 0.5000, 0.5000, -0.8660,
                    0, 0, -180.9318, -0.0046, 188.2981, 188.2981, 108.7264,
                    0, 0, -184.9259, -333.3800, -166.6900, -306.6900, 531.1871,
                    0, 0, -277.4055, 0, 288.7071, 531.1871, 306.6900
            };

    double M[4][4] =
            {
                    1.0000, 0, 0, 613.3800,
                    0, -0.8660, 0.5000, 154.9957,
                    0, -0.5000, -0.8660, -485.9044,
                    0, 0, 0, 1.0000,
            };

    ////10du
//    double M[4][4] =
//            {
//                    0.8155, 0.2195, -0.5355, 357.1,
//                    0.4157, -0.8660, 0.2781, 277.5,
//                    -0.4027, -0.4494, -0.7974, -568.3,
//                    0, 0, 0, 1
//           };
    double T[4][4] = {
            0, -1, 0, 180,
            -1, 0, 0, -280,
            0, 0, -1, -665,
            0, 0, 0, 1
    };
    //double thetalist0[7] = {0, 1, 0, -1.5708, 0, 0, 0};
    double thetalist0[7] = {-0.1, -0.2, 0.1, 0.030890, 0.5, -0.238, 0.239};
    double eomg = 1.0E-4;
    double ev = 1.0E-4;
    double thetalist[7] = {0};
    int maxiter = 100;
    int ret = IKinSpaceNR(JointNum, (double *) Slist, M, T, thetalist0, eomg, ev, maxiter, thetalist);
    if (ret) {
        printf("IKinSpace error %d\n", ret);
        return;
    }
    printf("solution thetalist for C(单位：弧度):\n");
    int i;
    for (i = 0; i < JointNum; i++) {
        printf("%lf, ", thetalist[i]);
    }
    printf("\n\n");

    double MM[4][4] =
            {
                    1.0000, 0, 0, 613.3800,
                    0, -0.8660, 0.5000, 154.9957,
                    0, -0.5000, -0.8660, -485.9044,
                    0, 0, 0, 1.0000,
            };

    for (i = 0; i < 7; i++) {
        printf("%lf ", thetalist[i]);
    }
    printf("\n");
    double TT[4][4];
//    FKinSpace(M, JointNum, (double *) Slist, thetalist, TT);
    double qqq[7] = {0};
    FKinSpace(M, JointNum, (double *) Slist, thetalist, TT);

    printf("TT:\n");
    for (i = 0; i < 4; i++) {
        printf("%lf %lf %lf %lf\n", TT[i][0], TT[i][1], TT[i][2], TT[i][3]);
    }
    return;
}


void test_JacobBody7() {
    MDHParam7 MP;
    double alpha[7] = {0, -90 * PI / 180, 146.30993 * PI / 180, -56.30993 * PI / 180, -60 * PI / 180, 0, 0};
    double a[7] = {0, 0, 333.38, 0, 0, 0, 0};
    double d[7] = {0, -326.16, -391.99, 0, 0, 0, 310};
    double theta[7] = {0, 0, 0, 0, 0, 0, 0};
    MP.alpha[7] = alpha[7];
    MP.a[7] = a[7];
    MP.d[7] = d[7];
    MP.theta[7] = theta[7];

//    DescartesTrans(MDHParam7 &MP, double T01[4][4],double T02[4][4],double T03[4][4],double T04[4][4],double T05[4][4],double T06[4][4],double T07[4][4]);
    double T01[4][4], T12[4][4], T23[4][4], T34[4][4], T45[4][4], T56[4][4], T67[4][4], T02[4][4], T03[4][4], T04[4][4], T05[4][4], T06[4][4], T07[4][4], Jb7[6][7];
    DescartesTrans(&MP, T01, T12, T23, T34, T45, T56, T67);
    JacobBody7(T01, T02, T03, T04, T05, T06, T07, Jb7);

    printf("Jb7:\n");
    for (int i = 0; i < 6; i++) {
        printf("%lf %lf %lf %lf\n", Jb7[i][0], Jb7[i][1], Jb7[i][2], Jb7[i][3], Jb7[i][4], Jb7[i][5], Jb7[i][6]);
    }
    return;
}

void test_JacobianSpace7() {

    int i;
    int j;
    int JointNum = 7;

    double Slist[6][7] =
            {
                    0, 0, 0, 0, 0, 0, 0,
                    0, 1.0000, -0.8321, 0, 0.8660, 0.8660, 0.5,
                    1.0000, 0, 0.5547, 1.0000, 0.5000, 0.5000, -0.8660,
                    0, 0, -180.9318, -0.0046, 188.2981, 188.2981, 108.7264,
                    0, 0, -184.9259, -333.3800, -166.6900, -306.6900, 531.1871,
                    0, 0, -277.4055, 0, 288.7071, 531.1871, 306.6900
            };
    //initial thetalist.
    //double thetalist[7] = {10* PI / 180, 10* PI / 180, 10* PI / 180, 10* PI / 180,10* PI / 180,10* PI / 180,10* PI / 180};
    double thetalist[7] = {0, 0 * PI / 180, 0, -90 * PI / 180, 0, 0, 0};
    double Js[6][7] = {{0}};
    JacobianSpace(JointNum, (double *) Slist, thetalist, (double *) Js);

    printf("Js:\n");

    for (i = 0; i < 6; i++) {
        for (j = 0; j < JointNum; j++) {
            printf("%lf  ", Js[i][j]);
        }
        printf("\n");
    }

    return;
}


void test_Ttop6() {
    //    ////把T转换为SE3
//    MatrixLog6(double T[4][4], double se3Mat[4][4]);
//    ////将SE3转换成六维空间向量
//    se3ToVec(double se3Mat[4][4], double V[6]);

    double R[3][3], p[3];
    double roll, pitch, yaw;
    double p1[6], p2[6];
    double M[4][4] =
            {
                    1.0000, 0, 0, 613.3800,
                    0, -0.8660, 0.5000, 154.9957,
                    0, -0.5000, -0.8660, -485.9044,
                    0, 0, 0, 1.0000,
            };
    ////把T还原回平移与旋转旋量
    TransToRp(M, R, p);
    ////将roll, pitch, yaw 三个角度转换为R
    RotToRPY(R, &roll, &pitch, &yaw);
    p1[0] = p[0];
    p1[1] = p[1];
    p1[2] = p[2];
    p1[3] = roll;
    p1[4] = pitch;
    p1[5] = yaw;

    printf("p1:\n");
    printf("%lf %lf %lf %lf %lf %lf\n", p1[0], p1[1], p1[2], p1[3], p1[4], p1[5]);


    double T[4][4] = {
            0.000000, -0.999978, 0.000013, 179.997287,
            -1.000000, -0.000000, 0.000000, -280.004300,
            -0.000000, -0.000013, -0.999978, -664.995554,
            0.000000, 0.000000, 0.000000, 1.000000,
    };
    ////把T还原回平移与旋转旋量
    TransToRp(T, R, p);
    ////将roll, pitch, yaw 三个角度转换为R
    RotToRPY(R, &roll, &pitch, &yaw);
    p2[0] = p[0];
    p2[1] = p[1];
    p2[2] = p[2];
    p2[3] = roll;
    p2[4] = pitch;
    p2[5] = yaw;

    printf("p2:\n");
    printf("%lf %lf %lf %lf %lf %lf\n", p2[0], p2[1], p2[2], p2[3], p2[4], p2[5]);

    return;
}

void GaussJordanInv(){
//    float **input,**Inverse,localVariable;
//    float* temprow;
//    int i,j,k,sizeOfMatrix;
//
//    printf("Enter matrix size. It's a square matrix. So enter value of n (nXn)\n");
//    scanf("%d",&sizeOfMatrix);
//
//    input = (float **)malloc(sizeOfMatrix*sizeof(float *));
//
//    for(i=0;i<sizeOfMatrix;i++)
//        input[i]=(float *)malloc(sizeOfMatrix*sizeof(float));
//    temprow=(float*)malloc(sizeOfMatrix*sizeof(float));
//    Inverse=(float **)malloc(sizeOfMatrix*sizeof(float *));
//
//    for(i=0;i<sizeOfMatrix;i++)
//    {
//
//        Inverse[i]=(float *)malloc(sizeOfMatrix*sizeof(float));
//    }
//
//    printf("Now enter the matrix:\n");
//    for(i=0;i<sizeOfMatrix;i++)
//        for(j=0;j<sizeOfMatrix;j++)
//            scanf("%f",&input[i][j]);
//
//    for(i=0;i<sizeOfMatrix;i++)
//        for(j=0;j<sizeOfMatrix;j++)
//            if(i==j)
//                Inverse[i][j]=1;
//            else
//                Inverse[i][j]=0;
//
//
//    for(k=0;k<sizeOfMatrix;k++)
//    {
//        if(input[k][k]==0)
//        {
//            for(j=k+1; j<sizeOfMatrix; j++)
//            {
//                if(input[j][k]!=0)
//                    break;
//            }
//
//            if(j==sizeOfMatrix)
//            {
//                printf("\nMATRIX IS NOT INVERSIBLE\n\n");
//                return ;
//            }
//            temprow=input[k];
//            input[k]=input[j];
//            input[j]=temprow;
//            temprow=Inverse[k];
//            Inverse[k]=Inverse[j];
//            Inverse[j]=temprow;
//        }
//        localVariable=input[k][k];
//        for(j=0;j<sizeOfMatrix;j++)
//        {
//            input[k][j]/=localVariable;
//            Inverse[k][j]/=localVariable;
//
//        }
//        for(i=0;i<sizeOfMatrix;i++)
//        {
//            localVariable = input[i][k];
//            for(j=0;j<sizeOfMatrix;j++)
//            {
//                if(i==k)
//                    break;
//                input[i][j] -= input[k][j]*localVariable;
//                Inverse[i][j] -= Inverse[k][j]*localVariable;
//            }
//        }
//    }
//
//    printf("The inverse matrix is:\n");
//
//    for(i=0;i<sizeOfMatrix;i++)
//    {
//        for(j=0;j<sizeOfMatrix;j++)
//            printf("%f	",Inverse[i][j]);
//        printf("\n");
//    }
//    return ;
}


static double argInit_real_T(void)
{
    return 0.0;
}


static void argInit_6x6_real_T(double result[36])
{
    int idx0;
    int idx1;

    /* Loop over the array to initialize each element. */
    for (idx0 = 0; idx0 < 6; idx0++) {
        for (idx1 = 0; idx1 < 6; idx1++) {
            /* Set the value of the array element.
               Change this value to the value that the application requires. */
            result[idx0 + 6 * idx1] = argInit_real_T();
        }
    }
}

void test_myInv() {
    double A[36] = {1,3,6,9,8,7,
                    2,2,3,3,6,4,
                    1,4,5,6,3,8,
                    4,7,2,1,6,9,
                    1,2,3,6,8,4,
                    2,6,3,7,8,1};

    double B[36] ;
    double InvA[6][6];
    myInv1(A, B);
    printf("B:\n");
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            InvA[i][j]  = B[i*6+j];
        }
//        printf("%lf %lf %lf %lf %lf %lf\n", B[i*6+0], B[i*6+1], B[i*6+2], B[i*6+3], B[i*6+4], B[i*6+5]);
        printf("%lf %lf %lf %lf %lf %lf\n", InvA[i][0], InvA[i][1], InvA[i][2], InvA[i][3], InvA[i][4], InvA[i][5]);
    }

//    printf("InvA:\n");
//    for (int i = 0; i < 6; i++) {
//        printf("%lf %lf %lf %lf %lf %lf\n", InvA[i][0], InvA[i][1], InvA[i][2], InvA[i][3], InvA[i][4], InvA[i][5]);
//    }

    return;
}

void test_InvJacoDLS(){
    double Slist[6][7] =
            {
                    0, 0, 0, 0, 0, 0, 0,
                    0, 1.0000, -0.8321, 0, 0.8660, 0.8660, 0.5,
                    1.0000, 0, 0.5547, 1.0000, 0.5000, 0.5000, -0.8660,
                    0, 0, -180.9318, -0.0046, 188.2981, 188.2981, 108.7264,
                    0, 0, -184.9259, -333.3800, -166.6900, -306.6900, 531.1871,
                    0, 0, -277.4055, 0, 288.7071, 531.1871, 306.6900
            };
    double invJ_DLS[7][6];
    InvJacoDLS(Slist,invJ_DLS);
    return;
}