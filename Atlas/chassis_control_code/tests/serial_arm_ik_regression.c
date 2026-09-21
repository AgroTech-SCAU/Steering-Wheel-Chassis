#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/domain/serial_arm/serial_arm_kine.c"

static int failures = 0;

#define CHECK(cond, fmt, ...) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL: " fmt "\n", ##__VA_ARGS__); \
        failures++; \
    } \
} while (0)

static float vec_norm3(const float v[3]) {
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static void rot_x(float a, float R[3][3]) {
    float c=cosf(a), s=sinf(a);
    float t[3][3]={{1,0,0},{0,c,-s},{0,s,c}};
    memcpy(R,t,sizeof(t));
}
static void rot_z(float a, float R[3][3]) {
    float c=cosf(a), s=sinf(a);
    float t[3][3]={{c,-s,0},{s,c,0},{0,0,1}};
    memcpy(R,t,sizeof(t));
}
static void mat_mul3(const float A[3][3], const float B[3][3], float C[3][3]) {
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) { C[i][j]=0; for(int k=0;k<3;k++) C[i][j]+=A[i][k]*B[k][j]; }
}

static void test_so3(void) {
    float I[3][3]={{1,0,0},{0,1,0},{0,0,1}};
    float R[3][3], e[3];

    rot_z(1e-4f,R);
    s_rotation_error(R,I,e);
    CHECK(fabsf(e[0])<1e-6f && fabsf(e[1])<1e-6f && fabsf(e[2]-1e-4f)<2e-7f,
          "small-angle log error [%g %g %g]", e[0],e[1],e[2]);

    rot_x((float)M_PI,R);
    s_rotation_error(R,I,e);
    CHECK(fabsf(vec_norm3(e)-(float)M_PI)<2e-4f,
          "pi log norm=%g expected=%g", vec_norm3(e),(float)M_PI);
    CHECK(fabsf(fabsf(e[0])-(float)M_PI)<2e-4f && fabsf(e[1])<2e-4f && fabsf(e[2])<2e-4f,
          "pi log axis [%g %g %g]",e[0],e[1],e[2]);

    /* Space Jacobian discriminator: R2 = Rx(eps) * R must yield base-x omega. */
    float baseR[3][3], dR[3][3], R2[3][3], omega[3];
    rot_z(0.5f*(float)M_PI,baseR);
    rot_x(1e-4f,dR);
    mat_mul3(dR,baseR,R2);
    s_angular_jacobian_col(baseR,R2,omega,1e-4f);
    CHECK(fabsf(omega[0]-1.0f)<2e-3f && fabsf(omega[1])<2e-3f && fabsf(omega[2])<2e-3f,
          "space angular Jacobian [%g %g %g]",omega[0],omega[1],omega[2]);
}

static void set_identity(SerialArmTransform *T) {
    memset(T,0,sizeof(*T));
    for(int i=0;i<4;i++) T->m[i][i]=1.0f;
}

static int build_atlas_model(SerialArmModel *model) {
    static const float a[5] = {0.0f,0.0276200491203067f,0.2167241256700170f,0.2002827243995208f,0.0451594898594991f};
    static const float d[5] = {0.0f,-0.0162679040568649f,-0.0192068569153542f,0.0014389528584892f,0.0f};
    static const float alpha[5] = {0.0f,M_PI*0.5f,M_PI,0.0f,M_PI*0.5f};
    static const float q_offset[5] = {-M_PI,-M_PI*0.5f,-3.3836013435535577f,-2.8616351199480290f,-M_PI};
    static const float q_sign[5] = {-1.0f,-1.0f,1.0f,1.0f,-1.0f};
    static const float tool_T[4][4] = {
        {0.9999619259637f,0.0f,-0.0087262032439f,0.0f},
        {0.0000761495224f,-0.9999619230642f,0.0087262032439f,0.0f},
        {-0.0087258709769f,-0.0087265354984f,-0.9999238504776f,-0.0184685931641f},
        {0,0,0,1}
    };
    if(serial_arm.model_reset(model,5,SERIAL_ARM_DH_MODIFIED)!=SERIAL_ARM_STATUS_SUCCESS) return 0;
    set_identity(&model->base_T); model->base_T.m[2][3]=0.0605f;
    memcpy(model->tool_T.m,tool_T,sizeof(tool_T));
    for(int i=0;i<5;i++) {
        if(serial_arm.model_set_revolute(model,(uint8_t)i,0,d[i],a[i],alpha[i],q_offset[i],0,2*M_PI)!=SERIAL_ARM_STATUS_SUCCESS) return 0;
        if(serial_arm.model_set_joint_sign(model,(uint8_t)i,q_sign[i])!=SERIAL_ARM_STATUS_SUCCESS) return 0;
    }
    model->ik.max_iterations=250.0f;
    model->ik.position_tolerance=1e-4f;
    model->ik.orientation_tolerance=2e-3f;
    model->ik.step_gain=0.45f;
    model->ik.damping=2e-3f;
    model->ik.numeric_eps=1e-5f;
    model->ik.position_weight=1.0f;
    model->ik.orientation_weight=0.25f;
    return serial_arm.init(model)==SERIAL_ARM_STATUS_SUCCESS;
}

static SerialArmQuaternion quat_mul(SerialArmQuaternion a, SerialArmQuaternion b) {
    SerialArmQuaternion q;
    q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    q.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    q.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    q.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return q;
}

static float tool_axis_error(const SerialArmPose *pose, float pitch, float yaw) {
    float R[3][3];
    s_pose_to_rotation(pose,R);
    float d[3]={cosf(pitch)*cosf(yaw),cosf(pitch)*sinf(yaw),sinf(pitch)};
    float dot=R[0][2]*d[0]+R[1][2]*d[1]+R[2][2]*d[2];
    return acosf(s_clampf(dot,-1,1));
}

static int solve_pick(const char *name, const float q0[5], float dz_target) {
    SerialArmJointArray seed={.dof=5};
    memcpy(seed.q,q0,5*sizeof(float));
    SerialArmPose start,target,solpose;
    SerialArmJointArray sol={0};
    CHECK(serial_arm.fk(&seed,&start)==SERIAL_ARM_STATUS_SUCCESS,"%s FK start",name);
    if(serial_arm.pose_from_xyz_tool_direction(start.position.x+0.03f,start.position.y-0.02f,dz_target,
                                               -0.5f*(float)M_PI,0.0f,&start,&target)!=SERIAL_ARM_STATUS_SUCCESS) {
        CHECK(0,"%s target construction",name); return 0;
    }
    SerialArmTaskInfo task={.task_dim=5,.row={0,1,2,3,4},.angular_frame=SERIAL_ARM_ANGULAR_FRAME_TOOL};
    CHECK(serial_arm.set_task_info(&task)==SERIAL_ARM_STATUS_SUCCESS,"%s set task",name);
    SerialArmStatus st=serial_arm.ik(&target,&sol,&seed);
    if(st!=SERIAL_ARM_STATUS_SUCCESS) {
        CHECK(0,"%s IK status=%s",name,serial_arm.status_str(st)); return 0;
    }
    CHECK(serial_arm.fk(&sol,&solpose)==SERIAL_ARM_STATUS_SUCCESS,"%s FK solution",name);
    float dx=solpose.position.x-target.position.x,dy=solpose.position.y-target.position.y,dz=solpose.position.z-target.position.z;
    float pe=sqrtf(dx*dx+dy*dy+dz*dz);
    float ae=tool_axis_error(&solpose,-0.5f*(float)M_PI,0.0f);
    printf("PICK %-6s pos_err=%.6f m tool_err=%.3f deg q=[%.4f %.4f %.4f %.4f %.4f]\n",name,pe,ae*180/M_PI,sol.q[0],sol.q[1],sol.q[2],sol.q[3],sol.q[4]);
    CHECK(pe<5e-4f,"%s position err=%g",name,pe);
    CHECK(ae<3.0f*(float)M_PI/180.0f,"%s tool axis err deg=%g",name,ae*180/M_PI);
    return 1;
}

static int solve_place(const char *name, const float q0[5], float x,float y,float z) {
    SerialArmJointArray seed={.dof=5}; memcpy(seed.q,q0,5*sizeof(float));
    SerialArmPose start,target,solpose; SerialArmJointArray sol={0};
    if(serial_arm.fk(&seed,&start)!=SERIAL_ARM_STATUS_SUCCESS) { CHECK(0,"%s park FK",name); return 0; }
    if(serial_arm.pose_from_xyz_tool_direction(x,y,z,-0.5f*(float)M_PI,0.0f,&start,&target)!=SERIAL_ARM_STATUS_SUCCESS) { CHECK(0,"%s target",name); return 0; }
    SerialArmTaskInfo task={.task_dim=5,.row={0,1,2,3,4},.angular_frame=SERIAL_ARM_ANGULAR_FRAME_TOOL};
    serial_arm.set_task_info(&task);
    SerialArmStatus st=serial_arm.ik(&target,&sol,&seed);
    if(st!=SERIAL_ARM_STATUS_SUCCESS){CHECK(0,"%s IK status=%s",name,serial_arm.status_str(st)); return 0;}
    serial_arm.fk(&sol,&solpose);
    float dx=solpose.position.x-x,dy=solpose.position.y-y,dz=solpose.position.z-z;
    float pe=sqrtf(dx*dx+dy*dy+dz*dz), ae=tool_axis_error(&solpose,-0.5f*(float)M_PI,0);
    printf("PLACE %-6s pos_err=%.6f m tool_err=%.3f deg\n",name,pe,ae*180/M_PI);
    CHECK(pe<5e-4f,"%s place pos err=%g",name,pe); CHECK(ae<3*M_PI/180,"%s place tool err=%g",name,ae*180/M_PI);
    return 1;
}

int main(void){
    test_so3();

    SerialArmPose ref = {0};
    SerialArmPose dir_pose = {0};
    ref.orientation.w = 1.0f;
    CHECK(serial_arm.pose_from_xyz_tool_direction(0.0f, 0.0f, 0.0f,
                                                  -0.5f*(float)M_PI, 0.0f,
                                                  &ref, &dir_pose) == SERIAL_ARM_STATUS_SUCCESS,
          "vertical-down tool direction accepted");
    CHECK(serial_arm.pose_from_xyz_tool_direction(0.0f, 0.0f, 0.0f,
                                                  -3.063f, -3.112f,
                                                  &ref, &dir_pose) == SERIAL_ARM_STATUS_INVALID_POSE,
          "legacy RPY-like pitch must be rejected by new tool-direction API");

    SerialArmModel model;
    CHECK(build_atlas_model(&model),"build atlas model");
    {
        const float q0[5] = {3.095574f,3.115515f,4.819768f,3.006603f,3.144661f};
        SerialArmJointArray seed = {.dof=5};
        memcpy(seed.q, q0, sizeof(q0));
        SerialArmPose start, spin_target;
        SerialArmJointArray spin_solution = {0};
        CHECK(serial_arm.fk(&seed, &start) == SERIAL_ARM_STATUS_SUCCESS, "free-spin FK start");
        spin_target = start;
        const float half = (120.0f * (float)M_PI / 180.0f) * 0.5f;
        SerialArmQuaternion local_spin = {cosf(half), 0.0f, 0.0f, sinf(half)};
        spin_target.orientation = quat_mul(start.orientation, local_spin);
        SerialArmTaskInfo tool_tilt_task = {
            .task_dim=5, .row={0,1,2,3,4}, .angular_frame=SERIAL_ARM_ANGULAR_FRAME_TOOL
        };
        CHECK(serial_arm.set_task_info(&tool_tilt_task) == SERIAL_ARM_STATUS_SUCCESS, "free-spin set task");
        CHECK(serial_arm.ik(&spin_target, &spin_solution, &seed) == SERIAL_ARM_STATUS_SUCCESS,
              "tool-z self spin must remain unconstrained");
        for(int i=0;i<5;i++) {
            CHECK(fabsf(spin_solution.q[i] - seed.q[i]) < 1e-5f,
                  "free-spin changed q[%d]: %.7f -> %.7f", i, seed.q[i], spin_solution.q[i]);
        }
    }

    const float obs[8][5]={
      {3.095574f,3.115515f,4.819768f,3.006603f,3.144661f},
      {3.489807f,3.255108f,4.910273f,2.759632f,3.136991f},
      {3.380894f,3.965341f,3.805807f,3.097108f,3.138525f},
      {3.035748f,4.172428f,3.718370f,3.009671f,3.138525f},
      {2.991263f,3.121651f,5.198661f,2.692136f,3.219826f},
      {3.406972f,3.046486f,5.140370f,2.784175f,3.250506f},
      {3.360952f,3.868700f,4.061981f,2.978991f,3.250506f},
      {2.991263f,3.992952f,4.074253f,2.830195f,3.250506f}
    };
    const char *names[8]={"A0","A1","A2","A3","B0","B1","B2","B3"};
    for(int i=0;i<8;i++) solve_pick(names[i],obs[i],0.034f);

    const float parks[4][5]={
      {4.819768f,3.443787f,4.497632f,2.885418f,3.143127f},
      {1.438874f,3.499010f,4.279807f,3.207554f,3.143127f},
      {1.356039f,3.402370f,4.482292f,3.072564f,3.247438f},
      {4.735399f,3.592583f,4.316623f,3.086370f,3.248972f}
    };
    /* Use representative first placement at approach height = first-layer + 0.06 m. */
    solve_place("A_P1",parks[0],0.049f,-0.283f,0.079f);
    solve_place("A_P2",parks[1],-0.046f,0.349f,0.075f);
    solve_place("B_P1",parks[2],-0.072f,0.290f,0.070f);
    solve_place("B_P2",parks[3],0.091f,-0.344f,0.072f);

    if(failures){fprintf(stderr,"TOTAL FAILURES: %d\n",failures); return 1;}
    puts("ALL TESTS PASSED"); return 0;
}
