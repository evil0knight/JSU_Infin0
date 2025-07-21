
#define FRAMENONE             (3)
#define RESAMPLEDIST          (0.02f)                       // ���ߵȾ���������

#include "zf_common_headfile.h"
#define ISLAND_H_4//ISLAND_H_1�����ȿ�Դ�е�Բ������,ISLAND_H_2��Bվ�Ĵ��룬ISLAND_H_3��100Ԫ����


#if defined(ISLAND_H_1)
typedef enum        // ģʽѡ��
{
    CIRCLE_NONE,
    CIRCLE_LEFT_BEGIN,
    CIRCLE_RIGHT_BEGIN,
    CIRCLE_LEFT_IN,
    CIRCLE_LEFT_RUNNING,
    CIRCLE_LEFT_OUT,
    CIRCLE_LEFT_END,
    CIRCLE_RIGHT_IN,
    CIRCLE_RIGHT_RUNNING,
    CIRCLE_RIGHT_OUT,
    CIRCLE_RIGHT_END,
}island_mode_choice;
extern island_mode_choice circle_type;
void CheckCircle(uint8 *l_border, uint8 *r_border,
                 uint16 total_num_l, uint16 total_num_r,
                 uint16 *dir_l, uint16 *dir_r,
                 uint16(*points_l)[2], uint16(*points_r)[2],
                 uint8* hightest,
                 uint16 *l_index, uint16 *r_index,
                 int monotonicity_l, int monotonicity_r);
void RunCircle(uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
                 uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2],
                 uint8* highest, uint16 *l_index, uint16 *r_index,
                 int monotonicity_l, int monotonicity_r);
/*typedef enum        // ģʽѡ��
{
    CIRCLE_NONE,
    CIRCLE_LEFT_BEGIN,
    CIRCLE_RIGHT_BEGIN,
    CIRCLE_LEFT_IN,
    CIRCLE_LEFT_RUNNING,
    CIRCLE_LEFT_OUT,
    CIRCLE_LEFT_END,
    CIRCLE_RIGHT_IN,
    CIRCLE_RIGHT_RUNNING,
    CIRCLE_RIGHT_OUT,
    CIRCLE_RIGHT_END,
}island_mode_choice;
extern island_mode_choice circle_type;
typedef enum track_type_e {
    TRACK_LEFT = 0,
    TRACK_RIGHT,
} track_type_e;
extern track_type_e track_type;
*/
#elif defined(ISLAND_H_2)


void around_fill_1(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
                 uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2],
                 uint8* hightest,uint16 *l_index,uint16 *r_index, int monotonicity_l, int monotonicity_r);

#elif defined(ISLAND_H_3)

#endif
