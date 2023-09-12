/**
 * @file vision_task.c
 * @author yuanluochen
 * @brief ��������yaw��pitch��roll����ԽǶȸ���λ���Ӿ����������Ӿ��������ݣ�����hal������ԭ���ȫ˫������ͨ��֧�ֲ����ر�ã�
 *        Ϊ����������⽫�������ݷ������봮�ڷ��ͷ��룬���ӳ����ڷ���ʱ��
 * @version 0.1
 * @date 2023-03-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "vision_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "shoot_task.h"
#include "gimbal_behaviour.h"
//�Ӿ���������ṹ���ʼ��
static void vision_send_task_init(vision_send_t* init);
//�Ӿ������ʼ��
static void vision_task_init(vision_control_t* init);
//�Ӿ������������ݸ���
static void vision_send_task_feedback_update(vision_send_t* update);
//�Ӿ��������ݸ���
static void vision_task_feedback_update(vision_control_t* update);
//���ݱ���
static void vision_tx_encode(uint8_t* buf, float yaw, float pitch, float roll, uint8_t mode_switch);
//����yaw��pitch������
static void vision_analysis_date(vision_control_t* vision_set);
//��������
static void send_message_to_vision(UART_HandleTypeDef* send_message_usart, DMA_HandleTypeDef* send_message_dma, uint8_t* send_message, uint16_t send_message_size);

/**
 * @brief �����Ӿ�ԭʼ�������ݣ�����ԭʼ���ݣ��ж��Ƿ�Ҫ���з���
 * 
 * @param shoot_judge �Ӿ��ṹ��
 * @param vision_begin_add_yaw_angle ��λ���Ӿ�yuw��ԭʼ���ӽǶ�
 * @param vision_begin_add_pitch_angle ��λ���Ӿ�pitch��ԭʼ���ӽǶ�
 */
static void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle);

/**
 * @brief ��λ�����ݷ���
 * 
 * @param vision_send ��λ�����ݷ��ͽṹ��
 */
static void vision_send_msg(vision_send_t* vision_send);




//�Ӿ�����ṹ��
vision_control_t vision_control = { 0 };
//�Ӿ���������ṹ��
vision_send_t vision_send = { 0 };

//δ���յ��Ӿ����ݱ�־λ����λΪ1 ��δ����
bool_t not_rx_vision_data_flag = 1;

    //�Ӿ���������ṹ���ʼ��
        //���͵�ǰλ�˴��ݸ���λ���Ӿ�
        vision_send_msg(&vision_send);
        //ϵͳ��ʱ

void vision_task_init(vision_control_t* init)
{
    // ��ȡ�����Ǿ��Խ�ָ��                                                                                                                                                                                                                                                                                                                                                           init->vision_angle_point = get_INS_angle_point();
    init->vision_angle_point = get_INS_angle_point();
	
    // ��ȡ��λ���Ӿ�ָ��
    init->vision_rxfifo = get_vision_rxfifo_point();
    //��ʼ������ģʽΪֹͣϮ��
    init->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;

    vision_task_feedback_update(init);
 
}
void vision_task(void const* pvParameters)
{
    // ��ʱ�ȴ����ȴ���λ���������ݳɹ�
    vTaskDelay(VISION_TASK_INIT_TIME);
    // �Ӿ������ʼ��
    vision_task_init(&vision_control);
    while (1)
    {
        // �ȴ�Ԥװ����ϣ��Լ���̨���̳�ʼ����ϣ��ж������Ƿ����
//        if (shoot_control_vision_task() && gimbal_control_vision_task())
//        {
            // ��������
            vision_task_feedback_update(&vision_control);
            // ������λ������,����yaw��pitch������,�Լ��ж��Ƿ���
            vision_analysis_date(&vision_control);
//        }
//        else
//        {
//            //��������
//        }
        // ϵͳ��ʱ
        vTaskDelay(VISION_CONTROL_TIME_MS);
    }
}

static void vision_send_msg(vision_send_t* vision_send)
{
    //���ݸ���
    vision_send_task_feedback_update(vision_send);

    //���ô��ڷ�������,����
    vision_tx_encode(vision_send->send_message, vision_send->send_absolution_angle.yaw * RADIAN_TO_ANGLE,
                                                vision_send->send_absolution_angle.pitch * RADIAN_TO_ANGLE,
                                                vision_send->send_absolution_angle.roll * RADIAN_TO_ANGLE,
                                                ARMOURED_PLATE);
    //���ڷ���
    send_message_to_vision(vision_send->send_message_usart, vision_send->send_message_dma, vision_send->send_message, SERIAL_SEND_MESSAGE_SIZE);
    
}

static void send_message_to_vision(UART_HandleTypeDef* send_message_usart, DMA_HandleTypeDef* send_message_dma, 
                                  uint8_t* send_message, uint16_t send_message_size)
{
    //�ȴ���һ�����ݷ���
    while (HAL_DMA_GetState(send_message_dma) == HAL_DMA_STATE_BUSY)
    {
        static int count = 0;
        if (count ++ >= 1000)
        {
            break;
        }
    }
    //�ر�DMA
    __HAL_DMA_DISABLE(send_message_dma);

    //��������
    HAL_UART_Transmit_DMA(send_message_usart, send_message, send_message_size);
}

static void vision_send_task_init(vision_send_t* init)
{
    //��ȡ�����Ǿ��Խ�ָ��                                                                                                                                                                                                                                                                                                                                                           init->vision_angle_point = get_INS_angle_point();
    init->vision_angle_point = get_INS_angle_point();
    //��ʼ�����ʹ�������
    init->send_message_usart = &VISION_USART;
    //��ʼ������dma
    init->send_message_dma = &VISION_TX_DMA;

    // ���ݸ���
    vision_send_task_feedback_update(init);
}

static void vision_send_task_feedback_update(vision_send_t* update)
{
    // ��ȡԭʼ����
    update->absolution_angle.yaw = *(update->vision_angle_point + INS_YAW_ADDRESS_OFFSET);
    update->absolution_angle.pitch = -*(update->vision_angle_point + INS_PITCH_ADDRESS_OFFSET);
    update->absolution_angle.roll = *(update->vision_angle_point + INS_ROLL_ADDRESS_OFFSET);

    //���·�������,Ϊ�������ţ����ݼ�180
    update->send_absolution_angle.yaw = update->absolution_angle.yaw + SEND_MESSAGE_ERROR;
    update->send_absolution_angle.pitch = update->absolution_angle.pitch + SEND_MESSAGE_ERROR;
    update->send_absolution_angle.roll = update->absolution_angle.roll + SEND_MESSAGE_ERROR;
}
static void vision_task_feedback_update(vision_control_t* update)
{
    // ��ȡԭʼ���ݲ�ת��Ϊ�Ƕ���
    update->absolution_angle.yaw = *(update->vision_angle_point + INS_YAW_ADDRESS_OFFSET) * RADIAN_TO_ANGLE;
    update->absolution_angle.pitch = -*(update->vision_angle_point + INS_PITCH_ADDRESS_OFFSET) * RADIAN_TO_ANGLE;
    update->absolution_angle.roll = *(update->vision_angle_point + INS_ROLL_ADDRESS_OFFSET) * RADIAN_TO_ANGLE;

}
static void vision_tx_encode(uint8_t* buf, float yaw, float pitch, float roll, uint8_t mode_switch)
{
    //������ʼ
    date32_to_date8_t head1_temp = { 0 };
    date32_to_date8_t head2_temp = { 0 };
    //yaw����ֵת��
    date32_to_date8_t yaw_temp = { 0 };
    //pitch����ֵת��
    date32_to_date8_t pitch_temp = { 0 };
    //roll������ת��
    date32_to_date8_t roll_temp = { 0 };
    //ģʽת��
    date32_to_date8_t switch_temp = { 0 };
    //��ֹλ
    date32_to_date8_t end_temp = { 0 };

    //��ֵ����
    head1_temp.uint32_val = HEAD1_DATA;
    head2_temp.uint32_val = HEAD2_DATA;
    yaw_temp.uint32_val = yaw * DOUBLE_;
    pitch_temp.uint32_val = pitch * DOUBLE_;
    roll_temp.uint32_val = roll * DOUBLE_;  
    switch_temp.uint32_val = mode_switch;
    end_temp.uint32_val = 0x0A;//"/n"

    for (int i = 3; i >= 0; i--)
    {
        int j = -(i - 3);//����λ��ת��
        buf[HEAD1_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = head1_temp.uin8_value[i];
        buf[HEAD2_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = head2_temp.uin8_value[i];
        buf[YAW_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = yaw_temp.uin8_value[i];
        buf[PITCH_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = pitch_temp.uin8_value[i];
        buf[ROLL_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = roll_temp.uin8_value[i];
        buf[SWITCH_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = switch_temp.uin8_value[i];
        buf[END_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = end_temp.uin8_value[i];
    }
}

static void vision_analysis_date(vision_control_t *vision_set)
{

    // ��λ���Ӿ��汾
    static fp32 vision_gimbal_yaw = 0;   // yaw����Խ�
    static fp32 vision_gimbal_pitch = 0; // pitch����Խ�
    // δ���յ���λ����ʱ��
    static int32_t unrx_time = MAX_UNRX_TIME;

    // �жϵ�ǰ��̨ģʽΪ����ģʽ
        // ������ģʽ�����ýǶ�Ϊ��λ�����ýǶ�

        // �ж��Ƿ���յ���λ������
        if (vision_set->vision_rxfifo->rx_flag) // ʶ��Ŀ��
        {
            // ���յ����ݱ�־λΪ0
            not_rx_vision_data_flag = 0;

            unrx_time = 0;
            // ���յ���λ������
            // ���ձ�־λ ����
            vision_set->vision_rxfifo->rx_flag = 0;

            // ��ȡ��λ���Ӿ�����
            vision_gimbal_pitch = vision_set->vision_rxfifo->pitch_fifo;
            vision_gimbal_yaw = vision_set->vision_rxfifo->yaw_fifo;
        }

        // �ж���λ���Ӿ�ֹͣ����ָ��
        if (unrx_time >= MAX_UNRX_TIME)
        {
            // ��������
            unrx_time = 0;
            // ֹͣ����
            vision_set->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
            not_rx_vision_data_flag = 1;
        }
    }

    // ��ֵ����ֵ
    // �ж��Ƿ����ֵ����ֵ
    if (vision_gimbal_pitch == 0 && vision_gimbal_yaw == 0)
    {

        // δ��ֵ����Ϊ��ǰֵ
        vision_set->gimbal_vision_control.gimbal_pitch = vision_set->absolution_angle.pitch;
        vision_set->gimbal_vision_control.gimbal_yaw = vision_set->absolution_angle.yaw;
    }
    else
    {
        // �Ѹ�ֵ��������ֵ
        vision_set->gimbal_vision_control.gimbal_pitch = vision_gimbal_pitch;
        vision_set->gimbal_vision_control.gimbal_yaw = vision_gimbal_yaw;
    }
}


/**
 * @brief �����Ӿ�ԭʼ�������ݣ�����ԭʼ���ݣ��ж��Ƿ�Ҫ���з��䣬�ж�yaw��pitch�ĽǶȣ������һ����Χ�ڣ������ֵ���ӣ����ӵ�һ����ֵ���жϷ��䣬���yaw��pitch��Ƕȴ��ڸ÷�Χ�����������
 * 
 * @param shoot_judge �Ӿ��ṹ��
 * @param vision_begin_add_yaw_angle ��λ���Ӿ�yuw��ԭʼ���ӽǶ�
 * @param vision_begin_add_pitch_angle ��λ���Ӿ�pitch��ԭʼ���ӽǶ�
 */
static void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle)
{
    // �жϻ������
    static int attack_count = 0;
    // �ж�ֹͣ����Ĵ��� 
    static int stop_attack_count = 0;

    
    // ��λ�����ͽǶȵ�һ����λ�ڼ���ֵ����
    if (fabs(vision_begin_add_pitch_angle) <= ALLOW_ATTACK_ERROR && fabs(vision_begin_add_yaw_angle) <= ALLOW_ATTACK_ERROR)
    {
        // ֹͣ�������ֵ����
        stop_attack_count = 0;

        // �жϼ���ֵ�Ƿ񵽴��жϻ���ļ���ֵ
        if (attack_count >= JUDGE_ATTACK_COUNT)
        {
            // ����ɻ���Ĵ���
            // ���û���
            shoot_judge->shoot_vision_control.shoot_command = SHOOT_ATTACK;
        }
        else
        {
            // δ����ɻ���Ĵ���
            // ����ֵ����
            attack_count++;
        }
    }
    // ��λ������Ƕȴ��ڸ÷�Χ����ֵ����
    else if (fabs(vision_begin_add_pitch_angle) > ALLOW_ATTACK_ERROR || fabs(vision_begin_add_yaw_angle) > ALLOW_ATTACK_ERROR)
    {
        

        if (stop_attack_count >= JUDGE_STOP_ATTACK_COUNT)
        {
            //�ﵽֹͣ����ļ���
            // �жϻ������ֵ����
            attack_count = 0;
            //����ֹͣ����
            shoot_judge->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
        }
        else
        {
            // δ����ֹͣ����Ĵ���
            // ����ֵ����
            stop_attack_count ++;
        }

        
    }
}


bool_t judge_not_rx_vision_data(void)
{
    return not_rx_vision_data_flag;
}

// ��ȡ��λ����̨����
gimbal_vision_control_t* get_vision_gimbal_point(void)
{
    return &vision_control.gimbal_vision_control;
}

// ��ȡ��λ����������
shoot_vision_control_t* get_vision_shoot_point(void)
{
    return &vision_control.shoot_vision_control;
}
 