#include "Self_aim.h"
#include "arm_math.h"
#include "shoot_task.h"
extern float shoot_speed;

struct SolveTrajectoryParams st;
struct tar_pos tar_position[4]; // ���ֻ���Ŀ�װ�װ�
float t = 0.5f;                 // ����ʱ��

const float g = 9.78f;
// �������ٶ�
float bullet_v = 25.0f; // �ӵ��ٶ�

/*
@brief �����������������ģ��
@param s:m ����
@param v:m/s �ٶ�
@param angle:rad �Ƕ�
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
    st.k = 0.092f; // ����ϵ��
    // tΪ����v��angleʱ�ķ���ʱ��
    t = (float)((expf(st.k * s) - 1) / (st.k * v * arm_cos_f32(angle)));
    if (t < 0)
    {
        // Ŀ��㳬��������
        t = 0;
        return 0;
    }
    // zΪ����v��angleʱ�ĸ߶�
    z = (float)(v * arm_sin_f32(angle) * t - GRAVITY * t * t / 2.0f);
    return z;
}

/*
@brief pitch�����
@param s:m ����
@param z:m �߶�
@param v:m/s
@return angle_pitch:rad
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;
    // iteration
    for (i = 0; i < 20; i++)
    {
        angle_pitch = atan2f(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        if (!z_actual)
        {
            angle_pitch = 0;
            break;
        }
        dz = 0.3f * (z - z_actual);
        z_temp = z_temp + dz;
        if (fabsf(dz) < 0.00001f)
        {
            break;
        }
    }
    // �������Ƶĸ�����ת��Ϊ�Ƕ���
    angle_pitch = (angle_pitch * 180.f / PI); // ����Ϊ��������Ϊ��
    return angle_pitch;
}

/**
 * �������������ƫ���ǣ�yaw����
 *
 * @param x ������x����
 * @param y ������y����
 * @param z ������z������δʹ�ã�
 * @return ����õ���ƫ���ǣ��ԽǶ��Ʊ�ʾ��
 */
float calc_yaw(float x, float y, float z)
{
    float aim_yaw = 0.0f;
    // ʹ�� atan2f �������㷴����ֵ���õ������Ƶ�ƫ����
    // float yaw = atan2f(y, x);//
    // arm_math.h���е�atan2f����
    arm_atan2_f32(y, x, &aim_yaw);

    // �������Ƶ�ƫ����ת��Ϊ�Ƕ���
    aim_yaw = -(aim_yaw * 180.f / PI); // ����Ϊ��������Ϊ��

    return aim_yaw;
}

/**
 * �������������ŷ����¾��롣
 *
 * @param x ������x����
 * @param y ������y����
 * @param z ������z����
 * @return ����õ���ŷ����¾���
 */
static inline float calc_distance(float x, float y, float z)
{
    // �����������ƽ���ͣ���ȡ��ƽ�����õ�ŷ����¾���
    // float distance = sqrtf(x * x + y * y + z * z);
    float distance = 0.0f;
    arm_sqrt_f32(x * x + y * y + z * z, &distance);

    return distance;
}

/**
 * ������������ĸ����ǣ�pitch����
 *
 * @param x ������x����
 * @param y ������y����
 * @param z ������z����
 * @return ����õ��ĸ����ǣ��ԽǶ��Ʊ�ʾ��
 */
float calc_pitch(float x, float y, float z)
{
    float pitch = 0.0f;
    // ���� x��y ���������ƽ��ͶӰ��ģ���� z ��������ķ�����ֵ���õ������Ƶĸ�����
    //  arm_atan2_f32(z, sqrtf(x * x + y * y),&pitch);
    float temp_sqrt = 0.0f;
    arm_sqrt_f32(x * x + y * y, &temp_sqrt);
    // ���� x��y ���������ƽ��ͶӰ��ģ���� z ��������ķ�����ֵ���õ������Ƶĸ�����
    arm_atan2_f32(z, temp_sqrt, &pitch);

    if (shoot_speed > 10.0f)
    {
        bullet_v = shoot_speed;
    }

#if defined(TEST_SHOOT)
    bullet_v = 30.0f;
#endif

    // ʹ���������ٶ�ģ�͵������¸�����
    for (int i = 0; i < 20; i++)
    {

        float v_x = bullet_v * arm_cos_f32(pitch);
        float v_y = bullet_v * arm_sin_f32(pitch);

        // float t = sqrtf(x * x + y * y) / v_x;
        float t = 0.0f;
        arm_sqrt_f32(x * x + y * y, &t);
        t = t / v_x;
        float h = v_y * t - 0.5f * g * t * t;
        float dz = z - h;
        float abs_dz;
        arm_abs_f32(&dz, &abs_dz, 1);
        if (abs_dz < 0.00001f)
        {
            break;
        }

        // ���� dz ��������ŷ����¾�������µĸ����ǵı仯�������е�������
        pitch += asinf(dz / calc_distance(x, y, z));
    }

    // �������Ƶĸ�����ת��Ϊ�Ƕ���
    pitch = (pitch * 180.f / PI); // ����Ϊ��������Ϊ��

    return pitch;
}
/**
 * �������yaw��pitch
 *
 * @param x ������x����
 * @param y ������y����
 * @param z ������z����
 * @return ����õ���Ŀ��ǣ��ԽǶ��Ʊ�ʾ��
 */
void Auto_aim(float x, float y, float z, float *yaw, float *pitch, float *distance)
{
    *distance = calc_distance(x, y, z);
    *yaw = -calc_yaw(x, y, z);
    //*pitch = pitchTrajectoryCompensation(sqrtf(x*x+y*y), z, bullet_v);
    *pitch = calc_pitch(x, y, z);
}