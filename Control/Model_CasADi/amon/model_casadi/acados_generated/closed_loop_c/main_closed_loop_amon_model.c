/*
 * Closed-loop PC test for generated acados NMPC code.
 *
 * This follows the same runtime order as the STM32 code:
 * - set measured/current x as lbx/ubx at stage 0
 * - set hover/reference yref values
 * - solve NMPC
 * - read u0, clamp defensively
 * - simulate plant one control period
 * - warm-start next NMPC step
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/sim_interface.h"
#include "acados_solver_amon_model.h"
#include "acados_sim_solver_amon_model.h"

#define NX AMON_MODEL_NX
#define NU AMON_MODEL_NU
#define N  AMON_MODEL_N
#define NY AMON_MODEL_NY
#define NYN AMON_MODEL_NYN

#define DT 0.01
#define T_SIM 10.0
#define U_HOVER 90.0
#define INITIAL_ROLL_DEG 5.0
#define INITIAL_PITCH_DEG 5.0
#define INITIAL_YAW_DEG 2.0
#define PI 3.14159265358979323846


static double clamp(double value, double min_value, double max_value)
{
    return fmax(min_value, fmin(max_value, value));
}


static double deg_to_rad(double deg)
{
    return deg * PI / 180.0;
}


static double rad_to_deg(double rad)
{
    return rad * 180.0 / PI;
}


static void euler_deg_to_quat(double roll_deg, double pitch_deg, double yaw_deg, double *q)
{
    double roll = deg_to_rad(roll_deg);
    double pitch = deg_to_rad(pitch_deg);
    double yaw = deg_to_rad(yaw_deg);

    double cr = cos(roll / 2.0);
    double sr = sin(roll / 2.0);
    double cp = cos(pitch / 2.0);
    double sp = sin(pitch / 2.0);
    double cy = cos(yaw / 2.0);
    double sy = sin(yaw / 2.0);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}


static void quat_to_euler_deg(const double *q, double *roll_deg, double *pitch_deg, double *yaw_deg)
{
    double qw = q[0];
    double qx = q[1];
    double qy = q[2];
    double qz = q[3];

    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    double roll = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (qw * qy - qz * qx);
    sinp = clamp(sinp, -1.0, 1.0);
    double pitch = asin(sinp);

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double yaw = atan2(siny_cosp, cosy_cosp);

    *roll_deg = rad_to_deg(roll);
    *pitch_deg = rad_to_deg(pitch);
    *yaw_deg = rad_to_deg(yaw);
}


static void set_hover_yrefs(
    ocp_nlp_config *nlp_config,
    ocp_nlp_dims *nlp_dims,
    ocp_nlp_in *nlp_in,
    const double *u_prev
)
{
    double yref[NY] = {0.0};
    double yref_N[NYN] = {0.0};

    // --------------------------------------------------------
    // Hover state reference
    // --------------------------------------------------------
    yref[2] = 1.0;      // z = 1 m
    yref[6] = 1.0;      // enotski quaternion
    yref_N[2] = 1.0;
    yref_N[6] = 1.0;

    // --------------------------------------------------------
    // Hover input reference
    // --------------------------------------------------------
    yref[NX + 0] = U_HOVER;
    yref[NX + 1] = 0.0;
    yref[NX + 2] = 0.0;
    yref[NX + 3] = 0.0;
    yref[NX + 4] = 0.0;

    for (int stage = 0; stage < N; stage++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "yref", yref);
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_N);
}


static void warm_start_shift(
    ocp_nlp_config *nlp_config,
    ocp_nlp_dims *nlp_dims,
    ocp_nlp_in *nlp_in,
    ocp_nlp_out *nlp_out,
    double *u_prev
)
{
    double x_stage[NX];
    double u_stage[NU];

    // --------------------------------------------------------
    // Warm-start za naslednji NMPC korak.
    // --------------------------------------------------------
    for (int stage = 0; stage < N; stage++)
    {
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, stage + 1, "x", x_stage);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, stage, "x", x_stage);
    }

    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, N, "x", x_stage);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, N, "x", x_stage);

    for (int stage = 0; stage < N - 1; stage++)
    {
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, stage + 1, "u", u_stage);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, stage, "u", u_stage);
    }

    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, N - 1, "u", u_prev);
}


int main(void)
{
    int status = 0;
    const int n_steps = (int)(T_SIM / DT);

    // --------------------------------------------------------
    // Create NMPC solver
    // --------------------------------------------------------
    amon_model_solver_capsule *ocp_capsule = amon_model_acados_create_capsule();
    status = amon_model_acados_create(ocp_capsule);
    if (status)
    {
        printf("amon_model_acados_create() returned status %d. Exiting.\n", status);
        return status;
    }

    ocp_nlp_config *nlp_config = amon_model_acados_get_nlp_config(ocp_capsule);
    ocp_nlp_dims *nlp_dims = amon_model_acados_get_nlp_dims(ocp_capsule);
    ocp_nlp_in *nlp_in = amon_model_acados_get_nlp_in(ocp_capsule);
    ocp_nlp_out *nlp_out = amon_model_acados_get_nlp_out(ocp_capsule);

    // --------------------------------------------------------
    // Create plant simulator
    // --------------------------------------------------------
    amon_model_sim_solver_capsule *sim_capsule = amon_model_acados_sim_solver_create_capsule();
    status = amon_model_acados_sim_create(sim_capsule);
    if (status)
    {
        printf("amon_model_acados_sim_create() returned status %d. Exiting.\n", status);
        return status;
    }

    sim_config *sim_config_ptr = amon_model_acados_get_sim_config(sim_capsule);
    sim_in *sim_in_ptr = amon_model_acados_get_sim_in(sim_capsule);
    sim_out *sim_out_ptr = amon_model_acados_get_sim_out(sim_capsule);
    void *sim_dims = amon_model_acados_get_sim_dims(sim_capsule);

    // --------------------------------------------------------
    // Začetno stanje
    // --------------------------------------------------------
    double x[NX] = {0.0};
    euler_deg_to_quat(
        INITIAL_ROLL_DEG,
        INITIAL_PITCH_DEG,
        INITIAL_YAW_DEG,
        &x[6]
    );

    double u_prev[NU] = {U_HOVER, 0.0, 0.0, 0.0, 0.0};

    for (int stage = 0; stage <= N; stage++)
    {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, stage, "x", x);
    }
    for (int stage = 0; stage < N; stage++)
    {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, stage, "u", u_prev);
    }

    // --------------------------------------------------------
    // Simulacijska zanka
    // --------------------------------------------------------
    printf("t,z,vz,roll,pitch,yaw,edf,servo_1,servo_2,servo_3,servo_4,status\n");

    for (int step = 0; step < n_steps; step++)
    {
        const double t = step * DT;

        // To je isti runtime korak, ki ga bos na MCU naredil z meritvijo trenutnega x.
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x);

        set_hover_yrefs(nlp_config, nlp_dims, nlp_in, u_prev);

        status = amon_model_acados_solve(ocp_capsule);
        if (status != ACADOS_SUCCESS)
        {
            printf("warning: solver status %d at t=%f\n", status, t);
        }

        double u[NU];
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", u);

        // Clamp before applying - defensive programming
        u[0] = clamp(u[0], 0.0, 90.0);
        u[1] = clamp(u[1], -45.0, 45.0);
        u[2] = clamp(u[2], -45.0, 45.0);
        u[3] = clamp(u[3], -45.0, 45.0);
        u[4] = clamp(u[4], -45.0, 45.0);

        // --------------------------------------------------------
        // Plant simulation: x_{k+1} = f_sim(x_k, u_k)
        // --------------------------------------------------------
        sim_in_set(sim_config_ptr, sim_dims, sim_in_ptr, "x", x);
        sim_in_set(sim_config_ptr, sim_dims, sim_in_ptr, "u", u);

        int sim_status = amon_model_acados_sim_solve(sim_capsule);
        if (sim_status != ACADOS_SUCCESS)
        {
            printf("warning: sim status %d at t=%f\n", sim_status, t);
        }

        sim_out_get(sim_config_ptr, sim_dims, sim_out_ptr, "x", x);

        for (int i = 0; i < NU; i++)
        {
            u_prev[i] = u[i];
        }

        warm_start_shift(nlp_config, nlp_dims, nlp_in, nlp_out, u_prev);

        if ((step % 50) == 0 || step == n_steps - 1)
        {
            double roll;
            double pitch;
            double yaw;
            quat_to_euler_deg(&x[6], &roll, &pitch, &yaw);
            printf(
                "t=%0.2f,z=%0.6fvz=,%0.6f,roll=%0.6f,pitch=%0.6f,yaw=%0.6f,edf=%0.6f,servo1=%0.6f,servo2=%0.6f,servo3=%0.6f,servo4=%0.6f,status:%d\n",
                t, x[2], x[5], roll, pitch, yaw, u[0], u[1], u[2], u[3], u[4], status
            );
        }
    }

    status = amon_model_acados_sim_free(sim_capsule);
    if (status)
    {
        printf("amon_model_acados_sim_free() returned status %d.\n", status);
    }
    amon_model_acados_sim_solver_free_capsule(sim_capsule);

    status = amon_model_acados_free(ocp_capsule);
    if (status)
    {
        printf("amon_model_acados_free() returned status %d.\n", status);
    }
    amon_model_acados_free_capsule(ocp_capsule);

    return status;
}
