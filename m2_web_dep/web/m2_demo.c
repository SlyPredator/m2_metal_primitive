#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "m2_host.c"

#define DEMO_DT 0.001
#define DEMO_DECIM 1
#define DEMO_NEWTON 100

#define FSM_STAND 1
#define FSM_STANCE 2
#define FSM_SWING 3
#define T_STAND 0.1
#define T_STEP 0.15
#define LZ0 -0.32
#define HCL 0.075
#define MASS 10.057
#define GRAVITY 9.81
#define C_YAW 0.183

static int fsm[4];
static double t_fsm[4], t_i[4], t_f[4];
static double lx_ref[4], ly_ref[4], lz_ref[4];
static double lxdot_ref[4], lydot_ref[4], lzdot_ref[4];
static double lx_i[4], lx_f[4], ly_i[4], ly_f[4], lz_i[4], lz_f[4];
static double q_ref[12], u_ref[12], q_act[12], u_act[12];
static double trq[12];
static double target_vx = 0.0, target_vy = 0.0, target_yaw = 0.0;
static double xdot_ref = 0.0, ydot_ref = 0.0, psidot_ref = 0.0;
static double sim_time = 0.0;
static int step_count = 0;
static double pos_quat_trunk[7], vel_angvel_trunk[6];
static double qpos[HC_NQ], qvel[HC_NV];

static Vector3 g_trail[4][100];
static int g_trail_idx = 0;
static int show_foot_trails = 1;

static void demo_reset(void) {
    for(int i=0; i<HC_NQ; i++) qpos[i] = hc_key_qpos[i];
    for(int i=0; i<HC_NV; i++) qvel[i] = 0.0;
    
    // Override the leg initial poses to exactly match lz_ref = -0.32
    // ik_analytic(0, 0, -0.32) -> [0.0, 0.58, -1.16]
    for(int i=0; i<4; i++) {
        qpos[7 + 3*i] = 0.0;
        qpos[7 + 3*i + 1] = 0.58;
        qpos[7 + 3*i + 2] = -1.16;
    }
    qpos[2] = 0.342;

    sim_time = 0.0;
    step_count = 0;
    target_vx = 0.0; target_vy = 0.0; target_yaw = 0.0;
    xdot_ref = 0.0; ydot_ref = 0.0; psidot_ref = 0.0;
    for(int i=0; i<4; i++) {
        fsm[i] = FSM_STAND;
        t_fsm[i] = 0;
        t_i[i] = 0;
        t_f[i] = T_STAND;
        lx_ref[i] = 0; ly_ref[i] = 0; lz_ref[i] = 0;
        lxdot_ref[i] = 0; lydot_ref[i] = 0; lzdot_ref[i] = 0;
        lx_i[i] = 0; lx_f[i] = 0; ly_i[i] = 0; ly_f[i] = 0; 
        lz_i[i] = LZ0; lz_f[i] = LZ0;
    }
    memset(q_ref, 0, sizeof(q_ref));
    memset(u_ref, 0, sizeof(u_ref));
    memset(trq, 0, sizeof(trq));
    memset(g_trail, 0, sizeof(g_trail));
    g_trail_idx = 0;
}

static void state_machine(void) {
    for (int leg_no = 0; leg_no < 4; leg_no++) {
        if (sim_time >= t_fsm[leg_no] + T_STAND && fsm[leg_no] == FSM_STAND) {
            if (xdot_ref != 0.0 || ydot_ref != 0.0 || psidot_ref != 0.0 || target_vx != 0.0 || target_vy != 0.0 || target_yaw != 0.0) {
                if (leg_no == 0 || leg_no == 3) {
                    fsm[leg_no] = FSM_SWING;
                    lz_f[leg_no] = LZ0 + HCL;
                } else {
                    fsm[leg_no] = FSM_STANCE;
                    lz_f[leg_no] = LZ0;
                }
                t_fsm[leg_no] = sim_time;
                lz_i[leg_no] = LZ0;
                t_i[leg_no] = 0;
                t_f[leg_no] = T_STEP;
            } else {
                t_fsm[leg_no] = sim_time;
            }
        }

        if (sim_time >= t_fsm[leg_no] + T_STEP && fsm[leg_no] == FSM_STANCE) {
            if (target_vx == 0.0 && target_vy == 0.0 && target_yaw == 0.0) {
                fsm[leg_no] = FSM_STAND;
                t_fsm[leg_no] = sim_time;
                lz_i[leg_no] = LZ0;
                lz_f[leg_no] = LZ0;
                t_i[leg_no] = 0;
                t_f[leg_no] = T_STAND;
            } else {
                fsm[leg_no] = FSM_SWING;
                t_fsm[leg_no] = sim_time;
                lz_i[leg_no] = LZ0;
                lz_f[leg_no] = LZ0 + HCL;
                lx_i[leg_no] = -0.5 * xdot_ref * T_STEP;
                lx_f[leg_no] = 0.5 * xdot_ref * T_STEP;
                ly_i[leg_no] = -0.5 * ydot_ref * T_STEP;
                ly_f[leg_no] = 0.5 * ydot_ref * T_STEP;
                if (leg_no == 0 || leg_no == 1) {
                    ly_i[leg_no] -= 0.5 * C_YAW * psidot_ref * T_STEP;
                    ly_f[leg_no] += 0.5 * C_YAW * psidot_ref * T_STEP;
                } else {
                    ly_i[leg_no] += 0.5 * C_YAW * psidot_ref * T_STEP;
                    ly_f[leg_no] -= 0.5 * C_YAW * psidot_ref * T_STEP;
                }
            }
        }

        if (sim_time >= t_fsm[leg_no] + T_STEP && fsm[leg_no] == FSM_SWING) {
            if (leg_no == 0 || leg_no == 1) {
                step_count++;
            }
            fsm[leg_no] = FSM_STANCE;
            t_fsm[leg_no] = sim_time;
            lz_i[leg_no] = LZ0;
            lz_f[leg_no] = LZ0;
            lx_i[leg_no] = 0.5 * xdot_ref * T_STEP;
            lx_f[leg_no] = -0.5 * xdot_ref * T_STEP;
            ly_i[leg_no] = 0.5 * ydot_ref * T_STEP;
            ly_f[leg_no] = -0.5 * ydot_ref * T_STEP;
            if (leg_no == 0 || leg_no == 1) {
                ly_i[leg_no] += 0.5 * C_YAW * psidot_ref * T_STEP;
                ly_f[leg_no] -= 0.5 * C_YAW * psidot_ref * T_STEP;
            } else {
                ly_i[leg_no] -= 0.5 * C_YAW * psidot_ref * T_STEP;
                ly_f[leg_no] += 0.5 * C_YAW * psidot_ref * T_STEP;
            }
        }
    }
}

static void quintic_poly(double t, double t0, double tf, double q0, double qf, double* q, double* qdot) {
    t = t < t0 ? t0 : (t > tf ? tf : t);
    double T = tf - t0;
    if (fabs(T) < 1e-9) {
        *q = q0;
        *qdot = 0.0;
        return;
    }
    double s = (t - t0) / T;
    double dq = qf - q0;
    *q = q0 + dq * (10 * s * s * s - 15 * s * s * s * s + 6 * s * s * s * s * s);
    *qdot = (dq / T) * (30 * s * s - 60 * s * s * s + 30 * s * s * s * s);
}

static void cartesian_traj(void) {
    for (int leg_no = 0; leg_no < 4; leg_no++) {
        double dt = sim_time - t_fsm[leg_no];
        double ti = t_i[leg_no];
        double tf = t_f[leg_no];
        if (fsm[leg_no] == FSM_STAND || fsm[leg_no] == FSM_STANCE) {
            quintic_poly(dt, ti, tf, lz_i[leg_no], lz_f[leg_no], &lz_ref[leg_no], &lzdot_ref[leg_no]);
            lx_ref[leg_no] = 0.0; lxdot_ref[leg_no] = 0.0;
            ly_ref[leg_no] = 0.0; lydot_ref[leg_no] = 0.0;
        } else if (fsm[leg_no] == FSM_SWING) {
            quintic_poly(dt, ti, tf, lx_i[leg_no], lx_f[leg_no], &lx_ref[leg_no], &lxdot_ref[leg_no]);
            quintic_poly(dt, ti, tf, ly_i[leg_no], ly_f[leg_no], &ly_ref[leg_no], &lydot_ref[leg_no]);
            double half = 0.5 * tf;
            if (dt <= half) {
                quintic_poly(dt, ti, half, lz_i[leg_no], lz_f[leg_no], &lz_ref[leg_no], &lzdot_ref[leg_no]);
            } else {
                quintic_poly(dt, half, tf, lz_f[leg_no], lz_i[leg_no], &lz_ref[leg_no], &lzdot_ref[leg_no]);
            }
        }
    }
}

static void ik_analytic(const double X[3], int leg_no, double q_leg[3]) {
    double L1 = 0.1915;
    double lx = X[0], ly = X[1], lz = X[2];
    double l = sqrt(lx * lx + ly * ly + lz * lz);
    
    double val_a = ly / l;
    val_a = val_a < -1.0 ? -1.0 : (val_a > 1.0 ? 1.0 : val_a);
    q_leg[0] = asin(val_a);
    
    double val_k = (2 * L1 * L1 - l * l) / (2 * L1 * L1);
    val_k = val_k < -1.0 ? -1.0 : (val_k > 1.0 ? 1.0 : val_k);
    q_leg[2] = -M_PI + acos(val_k);
    
    double val_h = -lx / l;
    val_h = val_h < -1.0 ? -1.0 : (val_h > 1.0 ? 1.0 : val_h);
    q_leg[1] = -0.5 * q_leg[2] + asin(val_h);
}

static void jac_leg(const double q[3], int leg_no, double J[3][3], double* e0_out) {
    double L1 = 0.1915;
    double L2 = 0.1915;
    double w = (leg_no == 0 || leg_no == 1) ? 0.0811 : 0.0796;
    if (leg_no == 0 || leg_no == 2) w = -w;
    double dx = 0.0;
    
    double c1 = cos(q[0]), s1 = sin(q[0]);
    double c2 = cos(q[1]), s2 = sin(q[1]);
    double c3 = cos(q[2]), s3 = sin(q[2]);
    double x3 = 0.0, y3 = 0.0, z3 = -L2;
    
    double x2 = c3 * x3 + s3 * z3;
    double y2 = y3;
    double z2 = -s3 * x3 + c3 * z3 - L1;
    double x1 = c2 * x2 + s2 * z2 + dx;
    double y1 = y2 + w;
    double z1 = -s2 * x2 + c2 * z2;
    double e0[3] = {x1, c1 * y1 - s1 * z1, s1 * y1 + c1 * z1};
    if (e0_out) {
        e0_out[0] = e0[0]; e0_out[1] = e0[1]; e0_out[2] = e0[2];
    }
    
    double o01[3] = {0.0, 0.0, 0.0};
    double o02[3] = {dx, w * c1, w * s1};
    double o03[3] = {dx - L1 * s2, w * c1 + L1 * s1 * c2, w * s1 - L1 * c1 * c2};
    
    double v1[3] = {e0[0] - o01[0], e0[1] - o01[1], e0[2] - o01[2]};
    double v2[3] = {e0[0] - o02[0], e0[1] - o02[1], e0[2] - o02[2]};
    double v3[3] = {e0[0] - o03[0], e0[1] - o03[1], e0[2] - o03[2]};
    
    J[0][0] = 0.0; J[1][0] = -v1[2]; J[2][0] = v1[1];
    J[0][1] = c1 * v2[2] - s1 * v2[1]; J[1][1] = s1 * v2[0]; J[2][1] = -c1 * v2[0];
    J[0][2] = c1 * v3[2] - s1 * v3[1]; J[1][2] = s1 * v3[0]; J[2][2] = -c1 * v3[0];
}

static void mat3_inv(const double m[3][3], double invOut[3][3]) {
    double det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
                 m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                 m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    double invdet = 1.0 / det;
    invOut[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invdet;
    invOut[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;
    invOut[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;
    invOut[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;
    invOut[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
    invOut[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invdet;
    invOut[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invdet;
    invOut[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invdet;
    invOut[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invdet;
}

static void joint_traj(void) {
    for (int leg_no = 0; leg_no < 4; leg_no++) {
        double X[3] = {lx_ref[leg_no], ly_ref[leg_no], lz_ref[leg_no]};
        double Xd[3] = {lxdot_ref[leg_no], lydot_ref[leg_no], lzdot_ref[leg_no]};
        double q_leg[3];
        ik_analytic(X, leg_no, q_leg);
        for(int k=0; k<3; k++) q_ref[3 * leg_no + k] = q_leg[k];
        
        double J[3][3], Jinv[3][3];
        jac_leg(q_leg, leg_no, J, NULL);
        mat3_inv(J, Jinv);
        for(int i=0; i<3; i++) {
            u_ref[3 * leg_no + i] = 0;
            for(int j=0; j<3; j++) {
                u_ref[3 * leg_no + i] += Jinv[i][j] * Xd[j];
            }
        }
    }
}

static void solve_6x6(const double A[36], const double b[6], double x[6]) {
    double ATA[36] = {0};
    double ATb[6] = {0};
    for(int i=0; i<6; ++i) {
        for(int j=0; j<6; ++j) {
            for(int k=0; k<6; ++k) {
                ATA[i*6 + j] += A[k*6 + i] * A[k*6 + j];
            }
        }
        for(int k=0; k<6; ++k) {
            ATb[i] += A[k*6 + i] * b[k];
        }
    }
    for(int i=0; i<6; ++i) ATA[i*6 + i] += 1e-6;
    for(int i=0; i<6; ++i) {
        int pivot = i;
        for(int j=i+1; j<6; ++j) {
            if (fabs(ATA[j*6 + i]) > fabs(ATA[pivot*6 + i])) pivot = j;
        }
        if (pivot != i) {
            for(int k=0; k<6; ++k) {
                double tmp = ATA[i*6 + k]; ATA[i*6 + k] = ATA[pivot*6 + k]; ATA[pivot*6 + k] = tmp;
            }
            double tmp = ATb[i]; ATb[i] = ATb[pivot]; ATb[pivot] = tmp;
        }
        for(int j=i+1; j<6; ++j) {
            double factor = ATA[j*6 + i] / ATA[i*6 + i];
            for(int k=i; k<6; ++k) ATA[j*6 + k] -= factor * ATA[i*6 + k];
            ATb[j] -= factor * ATb[i];
        }
    }
    for(int i=5; i>=0; --i) {
        x[i] = ATb[i];
        for(int j=i+1; j<6; ++j) x[i] -= ATA[i*6 + j] * x[j];
        x[i] /= ATA[i*6 + i];
    }
}

static void stance_force(int leg_no, double F_out[4][3]) {
    double quat[4] = {pos_quat_trunk[3], pos_quat_trunk[4], pos_quat_trunk[5], pos_quat_trunk[6]};
    // quat2bryant
    double qw = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];
    double val = 2 * (qw * qy - qz * qx);
    if (val > 1.0) val = 1.0; else if (val < -1.0) val = -1.0;
    double roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
    double pitch = asin(val);
    double yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
    
    // Rz
    double Rz[9] = {
        cos(yaw), -sin(yaw), 0,
        sin(yaw),  cos(yaw), 0,
        0,         0,        1
    };
    
    // vel_b = Rz.T @ vel_angvel[:3]
    double vel_b[3];
    for(int i=0; i<3; i++) {
        vel_b[i] = 0;
        for(int j=0; j<3; j++) {
            vel_b[i] += Rz[j*3 + i] * vel_angvel_trunk[j]; // Rz.T
        }
    }
    
    // Extract foot positions relative to COM in the yaw-aligned trunk frame
    double r_foot[4][3];
    for(int i=0; i<4; i++) {
        double J_tmp[3][3];
        double e0[3];
        double qa[3] = {q_act[3*i], q_act[3*i+1], q_act[3*i+2]};
        jac_leg(qa, i, J_tmp, e0);
        
        // Add trunk length offset (x = +/- 0.147)
        double hx = (i == 0 || i == 1) ? 0.147 : -0.147;
        r_foot[i][0] = e0[0] + hx;
        r_foot[i][1] = e0[1]; // y offset (w) is already in e0
        r_foot[i][2] = e0[2]; // z offset is just e0[2]
    }
    
    double r_L[3], r_R[3];
    if (leg_no == 0) { 
        // Pair A: FR (0) + RL (3)
        for(int k=0; k<3; k++) { r_L[k] = r_foot[0][k]; r_R[k] = r_foot[3][k]; }
    } else { 
        // Pair B: FL (1) + RR (2)
        for(int k=0; k<3; k++) { r_L[k] = r_foot[1][k]; r_R[k] = r_foot[2][k]; }
    }
    
    double A[36] = {0};
    // Block row 1: I3, I3
    A[0*6+0]=1; A[1*6+1]=1; A[2*6+2]=1;
    A[0*6+3]=1; A[1*6+4]=1; A[2*6+5]=1;
    // Block row 2: skew(r_L), skew(r_R)
    A[3*6+0]=0;        A[3*6+1]=-r_L[2];  A[3*6+2]=r_L[1];
    A[4*6+0]=r_L[2];   A[4*6+1]=0;        A[4*6+2]=-r_L[0];
    A[5*6+0]=-r_L[1];  A[5*6+1]=r_L[0];   A[5*6+2]=0;
    
    A[3*6+3]=0;        A[3*6+4]=-r_R[2];  A[3*6+5]=r_R[1];
    A[4*6+3]=r_R[2];   A[4*6+4]=0;        A[4*6+5]=-r_R[0];
    A[5*6+3]=-r_R[1];  A[5*6+4]=r_R[0];   A[5*6+5]=0;
    
    double b[6];
    b[0] = 100 * (xdot_ref - vel_b[0]);
    b[1] = 100 * (ydot_ref - vel_b[1]);
    b[2] = 50 * (-10 * (pos_quat_trunk[2] - (-LZ0)) - 1 * vel_angvel_trunk[2]) + MASS * GRAVITY;
    b[3] = 50 * (-10 * roll - 0.5 * vel_angvel_trunk[3]);
    b[4] = 50 * (-10 * pitch - 0.5 * vel_angvel_trunk[4]);
    b[5] = 10 * (psidot_ref - vel_angvel_trunk[5]);
    
    double x[6];
    solve_6x6(A, b, x);
    
    if (leg_no == 0) {
        for(int k=0; k<3; k++) { F_out[0][k] = x[k]; F_out[3][k] = x[3+k]; }
    } else {
        for(int k=0; k<3; k++) { F_out[1][k] = x[k]; F_out[2][k] = x[3+k]; }
    }
}

static void joint_control(void) {
    double F_out[4][3] = {0};
    if (fsm[0] == FSM_STANCE || fsm[3] == FSM_STANCE) stance_force(0, F_out);
    if (fsm[1] == FSM_STANCE || fsm[2] == FSM_STANCE) stance_force(1, F_out);

    for (int leg_no = 0; leg_no < 4; leg_no++) {
        double qa[3] = {q_act[3*leg_no], q_act[3*leg_no+1], q_act[3*leg_no+2]};
        double ua[3] = {u_act[3*leg_no], u_act[3*leg_no+1], u_act[3*leg_no+2]};
        double qr[3] = {q_ref[3*leg_no], q_ref[3*leg_no+1], q_ref[3*leg_no+2]};
        double ur[3] = {u_ref[3*leg_no], u_ref[3*leg_no+1], u_ref[3*leg_no+2]};
        
        double pd[3];
        for(int i=0; i<3; i++) pd[i] = 10.0 * (-10.0 * (qa[i] - qr[i]) - 1.0 * (ua[i] - ur[i]));
        
        if (fsm[leg_no] == FSM_STANCE) {
            double F[3] = {F_out[leg_no][0], F_out[leg_no][1], F_out[leg_no][2]};
            
            double J[3][3];
            jac_leg(qr, leg_no, J, NULL);
            for(int i=0; i<3; i++) {
                for(int j=0; j<3; j++) {
                    pd[i] -= J[j][i] * F[j]; // -J^T F
                }
            }
        } else if (fsm[leg_no] == FSM_STAND) {
            double F[3] = {0.0, 0.0, (MASS * GRAVITY) / 4.0};
            double J[3][3];
            jac_leg(qr, leg_no, J, NULL);
            for(int i=0; i<3; i++) {
                for(int j=0; j<3; j++) {
                    pd[i] -= J[j][i] * F[j]; // -J^T F
                }
            }
        }
        for(int i=0; i<3; i++) trq[3*leg_no + i] = pd[i];
    }
}

static void demo_control_step(void) {
    double max_dvx = 2.0 * DEMO_DT;
    if (xdot_ref < target_vx) { xdot_ref += max_dvx; if (xdot_ref > target_vx) xdot_ref = target_vx; }
    if (xdot_ref > target_vx) { xdot_ref -= max_dvx; if (xdot_ref < target_vx) xdot_ref = target_vx; }
    
    double max_dvy = 1.0 * DEMO_DT;
    if (ydot_ref < target_vy) { ydot_ref += max_dvy; if (ydot_ref > target_vy) ydot_ref = target_vy; }
    if (ydot_ref > target_vy) { ydot_ref -= max_dvy; if (ydot_ref < target_vy) ydot_ref = target_vy; }
    
    double max_dyaw = 4.0 * DEMO_DT;
    if (psidot_ref < target_yaw) { psidot_ref += max_dyaw; if (psidot_ref > target_yaw) psidot_ref = target_yaw; }
    if (psidot_ref > target_yaw) { psidot_ref -= max_dyaw; if (psidot_ref < target_yaw) psidot_ref = target_yaw; }

    int leg2qpos[4] = {10, 7, 16, 13}; // FR, FL, RR, RL -> qpos index
    for (int leg_no = 0; leg_no < 4; leg_no++) {
        int idx = leg2qpos[leg_no];
        for(int k=0; k<3; k++) {
            q_act[3*leg_no + k] = qpos[idx + k];
            u_act[3*leg_no + k] = qvel[idx - 1 + k];
        }
    }
    for(int i=0; i<7; i++) pos_quat_trunk[i] = qpos[i];
    for(int i=0; i<6; i++) vel_angvel_trunk[i] = qvel[i];
    
    state_machine();
    cartesian_traj();
    joint_traj();
    joint_control();
    
    double qpn[HC_NQ], qvn[HC_NV];
    double qacc_ws[HC_NV] = {0};
    double trq_mapped[12] = {0};
    int leg2qpos_out[4] = {10, 7, 16, 13};
    for (int leg_no = 0; leg_no < 4; leg_no++) {
        int idx = leg2qpos_out[leg_no] - 7;
        for(int k=0; k<3; k++) {
            trq_mapped[idx + k] = trq[3*leg_no + k];
        }
    }
    m2_full_step(qpos, qvel, trq_mapped, qacc_ws, DEMO_DT, DEMO_NEWTON, qpn, qvn);
    if (fmod(sim_time, 0.05) < 0.0015) {
        printf("t=%.3f z=%.3f FSM0=%d trq0=[%.1f %.1f %.1f] ncon=%d\n", sim_time, qpos[2], fsm[0], trq[0], trq[1], trq[2], ncon);
    }
    memcpy(qpos, qpn, sizeof(qpos));
    memcpy(qvel, qvn, sizeof(qvel));
    sim_time += DEMO_DT;
}

// Visuals
#define M2_MESH_MAGIC 0x47314D53 // Reused G1MS for now
static Mesh g_mesh[64];
static int g_nmesh = 0;
static int g_have_mesh = 0;
static Material g_mat;
static Shader g_shader;
static Font g_font;

#ifdef PLATFORM_WEB
#define GLSL_V "#version 300 es\nprecision mediump float;\n"
#else
#define GLSL_V "#version 330\n"
#endif

static const char* VS = GLSL_V
  "in vec3 vertexPosition; in vec3 vertexNormal; in vec4 vertexColor;"
  "uniform mat4 mvp; uniform mat4 matNormal;"
  "out vec3 fN; out vec4 fC;"
  "void main(){ fN=normalize(vec3(matNormal*vec4(vertexNormal,0.0))); fC=vertexColor;"
  " gl_Position=mvp*vec4(vertexPosition,1.0); }";
static const char* FS = GLSL_V
  "in vec3 fN; in vec4 fC; uniform vec4 colDiffuse; out vec4 o;"
  "void main(){ vec3 N=normalize(fN);"
  " vec3 Lkey=normalize(vec3(0.4,0.5,0.85)), Lfill=normalize(vec3(-0.4,-0.6,0.25));"
  " float key=max(dot(N,Lkey),0.0), fill=max(dot(N,Lfill),0.0);"
  " float amb=0.45+0.12*N.z;"
  " float s=min(amb+0.55*key+0.22*fill,1.15);"
  " o=vec4(fC.rgb*colDiffuse.rgb*s,1.0); }";

static int load_meshes(const char* path) {
    FILE* f=fopen(path,"rb"); if(!f) return 0;
    int magic=0,nmesh=0; fread(&magic,4,1,f); fread(&nmesh,4,1,f);
    if (magic!=M2_MESH_MAGIC || nmesh<=0 || nmesh>64){ fclose(f); return 0; }
    int* vn=malloc(nmesh*4); int* fn=malloc(nmesh*4);
    for(int i=0;i<nmesh;i++){ fread(&vn[i],4,1,f); fread(&fn[i],4,1,f); }
    long vbase=ftell(f); long fbase=vbase; for(int i=0;i<nmesh;i++) fbase+=(long)vn[i]*3*4;
    for(int i=0;i<nmesh;i++){
        float* V=malloc((long)vn[i]*3*4); fseek(f,vbase,SEEK_SET); fread(V,4,(long)vn[i]*3,f); vbase=ftell(f);
        int* F=malloc((long)fn[i]*3*4);   fseek(f,fbase,SEEK_SET); fread(F,4,(long)fn[i]*3,f); fbase=ftell(f);
        Mesh msh={0}; msh.triangleCount=fn[i]; msh.vertexCount=fn[i]*3;
        msh.vertices=malloc((long)msh.vertexCount*3*4);
        msh.normals =malloc((long)msh.vertexCount*3*4);
        msh.colors  =malloc((long)msh.vertexCount*4);
        for(int t=0;t<fn[i];t++){
            int a=F[t*3],b=F[t*3+1],c=F[t*3+2];
            float* pa=&V[a*3]; float* pb=&V[b*3]; float* pc=&V[c*3];
            float u[3]={pb[0]-pa[0],pb[1]-pa[1],pb[2]-pa[2]};
            float w[3]={pc[0]-pa[0],pc[1]-pa[1],pc[2]-pa[2]};
            float n[3]={u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0]};
            float ln=sqrtf(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]); if(ln>1e-9f){n[0]/=ln;n[1]/=ln;n[2]/=ln;}
            int idx[3]={a,b,c};
            for(int k=0;k<3;k++){ int o=(t*3+k)*3; int co=(t*3+k)*4; float* p=&V[idx[k]*3];
                msh.vertices[o]=p[0]; msh.vertices[o+1]=p[1]; msh.vertices[o+2]=p[2];
                msh.normals[o]=n[0]; msh.normals[o+1]=n[1]; msh.normals[o+2]=n[2];
                msh.colors[co]=255; msh.colors[co+1]=255; msh.colors[co+2]=255; msh.colors[co+3]=255; }
        }
        UploadMesh(&msh,false); g_mesh[i]=msh; free(V); free(F);
    }
    free(vn); free(fn); fclose(f); g_nmesh=nmesh;
    return 1;
}

static void draw_geoms(void) {
    fk(qpos);
    for (int g=0; g<HC_NGEOM; g++) {
        int ty=hc_geom_type[g];
        double gp[3], gm[9]; geom_pose(g, gp, gm);
        if (ty==7 /*mesh*/) {
            if (!g_have_mesh) continue;
            int mid=hc_geom_dataid[g]; if(mid<0||mid>=g_nmesh) continue;
            Matrix mat={ (float)gm[0],(float)gm[1],(float)gm[2],(float)gp[0],
                         (float)gm[3],(float)gm[4],(float)gm[5],(float)gp[1],
                         (float)gm[6],(float)gm[7],(float)gm[8],(float)gp[2],
                         0,0,0,1 };
            const double* gc=&hc_geom_color[g*3];
            g_mat.maps[MATERIAL_MAP_DIFFUSE].color=(Color){
                (unsigned char)(gc[0]*255),(unsigned char)(gc[1]*255),(unsigned char)(gc[2]*255),255};
            DrawMesh(g_mesh[mid], g_mat, mat);
            continue;
        }
    }
}

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#else
#define EMSCRIPTEN_KEEPALIVE
#endif

EMSCRIPTEN_KEEPALIVE
void set_target_velocity(double vx, double vy, double vyaw) {
    target_vx = vx;
    target_vy = vy;
    target_yaw = vyaw;
}


EMSCRIPTEN_KEEPALIVE
void set_show_trails(int show) {
    show_foot_trails = show;
    if (!show) {
        memset(g_trail, 0, sizeof(g_trail));
    }
}

EMSCRIPTEN_KEEPALIVE
void trigger_reset(void) {
    demo_reset();
}

int main(int argc, char** argv) {
    demo_reset();
    const char* mpath = "web/m2_meshes.bin";
    const char* fenv = getenv("M2_DEMO_FRAMES");
    int auto_frames = fenv ? atoi(fenv) : 0;
    if (!auto_frames) {
        InitWindow(1280, 720, "m2_metal WASM Controller");
        g_font=LoadFontEx("web/assets/font.ttf", 48, 0, 0);
        if (g_font.texture.id==0) g_font=GetFontDefault();
        SetTextureFilter(g_font.texture, TEXTURE_FILTER_BILINEAR);
        g_shader=LoadShaderFromMemory(VS,FS);
        g_mat=LoadMaterialDefault(); g_mat.shader=g_shader;
        g_mat.maps[MATERIAL_MAP_DIFFUSE].color=(Color){255,255,255,255};
        g_have_mesh=load_meshes(mpath);
        SetTargetFPS(50);
    } else {
        FILE* mf=fopen(mpath,"rb"); int mg=0,nm=0; if(mf){fread(&mg,4,1,mf);fread(&nm,4,1,mf);fclose(mf);}
    }
    int falls=0;
    int frame=0;
    
    float cam_yaw = PI;
    float cam_pitch = 0.4f;
    float cam_dist = 2.0f;

    while (auto_frames ? frame < auto_frames : !WindowShouldClose()) {
        if (!auto_frames) {
            // Keyboard controls disabled. Driven by JS/web interface or API.
        } else {
            xdot_ref = 0.5;
        }
        
        for(int k=0; k<20; k++) { // run at 1000Hz (20 steps per 50Hz frame)
            demo_control_step();
        }
        
        if (sim_time > 0.3 && (qpos[2]<0.1 || !isfinite(qpos[2]))) { 
            printf("FALL! frame=%d qpos[2]=%f\n", frame, qpos[2]);
            falls++; demo_reset(); 
        }

        if (!auto_frames) {
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                Vector2 delta = GetMouseDelta();
                cam_yaw -= delta.x * 0.01f;
                cam_pitch += delta.y * 0.01f;
                if (cam_pitch < 0.1f) cam_pitch = 0.1f;
                if (cam_pitch > 1.5f) cam_pitch = 1.5f;
            }
            cam_dist -= GetMouseWheelMove() * 0.2f;
            if (cam_dist < 0.5f) cam_dist = 0.5f;
            if (cam_dist > 5.0f) cam_dist = 5.0f;

            Camera3D cam={0}; 
            cam.target=(Vector3){(float)qpos[0],(float)qpos[1],0.3f};
            cam.position=(Vector3){
                cam.target.x + cam_dist * cosf(cam_pitch) * cosf(cam_yaw),
                cam.target.y + cam_dist * cosf(cam_pitch) * sinf(cam_yaw),
                cam.target.z + cam_dist * sinf(cam_pitch)
            };
            cam.up=(Vector3){0,0,1};
            cam.fovy=42; cam.projection=CAMERA_PERSPECTIVE;
        
        BeginDrawing(); ClearBackground((Color){0, 0, 0, 255}); BeginMode3D(cam);
        
        // Draw reflected robot
        rlPushMatrix();
        rlTranslatef(0, 0, -0.002f);
        rlScalef(1.0f, 1.0f, -1.0f);
        rlDisableBackfaceCulling();
        draw_geoms();
        rlEnableBackfaceCulling();
        rlPopMatrix();
        
        if (show_foot_trails) {
            fk(qpos);
            int bids[4] = {4, 7, 10, 13}; // FL, FR, RL, RR bodies
            Vector3 offset = {-0.0035f, 0.0f, -0.1901f};
            for(int i=0; i<4; i++) {
                int bid = bids[i];
                Vector3 pos = { xpos[bid][0], xpos[bid][1], xpos[bid][2] };
                Quaternion q = { xquat[bid][1], xquat[bid][2], xquat[bid][3], xquat[bid][0] }; // w,x,y,z to x,y,z,w
                Vector3 rotated = Vector3RotateByQuaternion(offset, q);
                pos.x += rotated.x; pos.y += rotated.y; pos.z += rotated.z;
                g_trail[i][g_trail_idx] = pos;
            }
            g_trail_idx = (g_trail_idx + 1) % 100;
            
            Color trail_cols[4] = {
                (Color){255, 25, 25, 255},   // FL: Red
                (Color){25, 255, 25, 255},   // FR: Green
                (Color){25, 128, 255, 255},  // RL: Blue
                (Color){255, 153, 25, 255}   // RR: Orange
            };
            for(int i=0; i<4; i++) {
                for(int j=0; j<99; j++) {
                    int idx1 = (g_trail_idx - 1 - j + 100) % 100;
                    int idx2 = (g_trail_idx - 1 - j - 1 + 100) % 100;
                    Vector3 p1 = g_trail[i][idx1];
                    Vector3 p2 = g_trail[i][idx2];
                    if (p1.z == 0 && p1.x == 0) continue;
                    if (p2.z == 0 && p2.x == 0) continue;
                    
                    Color c = trail_cols[i];
                    c.a = (unsigned char)(255 * (1.0f - j/100.0f));
                    DrawLine3D(p1, p2, c);
                }
            }
        }
        
        // MuJoCo Glossy Checkerboard Floor
        const float tile_size = 0.5f;
        const int N = 16;
        int gcx = (int)floorf((float)qpos[0] / tile_size);
        int gcy = (int)floorf((float)qpos[1] / tile_size);
        for (int x = gcx - N; x <= gcx + N; x++) {
            for (int y = gcy - N; y <= gcy + N; y++) {
                Color tile_color = ((x + y) % 2 == 0) ? (Color){51, 76, 102, 210} : (Color){25, 51, 76, 210};
                Vector3 pos = { x * tile_size + tile_size * 0.5f, y * tile_size + tile_size * 0.5f, -0.001f };
                DrawCubeV(pos, (Vector3){ tile_size, tile_size, 0.001f }, tile_color);
            }
        }
        
        // Subtle grid lines on top for depth
        const int LN = 8;
        int lcx = (int)floorf((float)qpos[0]);
        int lcy = (int)floorf((float)qpos[1]);
        for (int k = -LN; k <= LN; k++) {
            int wx = lcx + k, wy = lcy + k;
            Color line_color = (Color){255, 255, 255, 20}; // very subtle overlay
            DrawLine3D((Vector3){(float)wx, (float)(lcy - LN), 0.001f}, (Vector3){(float)wx, (float)(lcy + LN), 0.001f}, line_color);
            DrawLine3D((Vector3){(float)(lcx - LN), (float)wy, 0.001f}, (Vector3){(float)(lcx + LN), (float)wy, 0.001f}, line_color);
        }
        
        draw_geoms();
        EndMode3D();
        EndDrawing();
        }
        frame++;
    }
    
    if (auto_frames) {
        printf("RESULT m2_demo frames=%d falls=%d final_pelvis_z=%.3f pass=%d\\n", frame, falls, qpos[2], (isfinite(qpos[2]) && qpos[2]>0.2)?1:0);
        return (isfinite(qpos[2]) && qpos[2]>0.2)?0:1;
    }
    if (!auto_frames) CloseWindow();
    return 0;
}
