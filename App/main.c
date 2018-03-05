/*!
    ��־λϵͳ
    //emergency_flag
        down                    //����
        lost_line               //����
        out_control             //ʧ��

    //run_mode
        stop                    //ͣ��
        go                      //��ʼ
        stand                   //վ�Ų���
        statr                   //����
        normoallization         //��һ��

    //function_choose_flag
        gear                    //��λ
        ramp_on                 //�µ�����
        SDS_on                  //����ʾ��������
 */   
  

#include "common.h"  
#include "include.h"
void normoallize (void);
void stop_detect (void);
void speed_get (void);
///////////////////////////////����ʱ��
uint8 time_sequential; //ʱ��
uint16 real_time_ms;  //ʵʱʱ�� ��λms
uint16  real_time_s;   //ʵʱʱ��  ��λs

uint8 speed_out_count;
uint8 turn_out_count;
uint8 stand_out_count;



 void  main(void)
{ 

if(initial_all() == 1)//��ʼ������ֵΪ1˵����ʼ��ʧ��
    { //��ʼ��ʧ��
      for(;;)
      {
          buzz_run();
          pit_delay_us(PIT1,1000);
      }
    }
    buzz_mode_flag = 1;//������ʾ
    
   enable_irq (PIT0_IRQn); //�������ж�
    
   if(run_mode == normoallization_mode)//��һ��
   {
     normoallize(); //��һ����ѭ��
   }
   
  
   for(;;)//������
    {
      CCDuse();
      /*
#ifdef USEING_SD_CARD       //����sd��ʱ�����ô˳���
      if(buff_full_flag == 1)
      {
        buff_full_flag = 0;
        save_sd();
      }
#endif
      
#ifndef USEING_SD_CARD ///////////������sd��ʱ��ѡ������ʾ������Һ����     
      if(SDS_on_flag==1)
      {
      #ifndef USEING_BLUETOOTH_LOWER_COMPUTER    //run these code when enable lower computer
         //   WirelessDateSend();
      #endif
      }
      else
      {
        display_menu();
      }
#endif
      
     
     fault_treat (); 
      //���ϴ���
    }
 */
    }
 
}


/////////���ж�
void PIT0_IRQHandler(void) 
{ 
   
    PIT_Flag_Clear(PIT0); //���жϱ�־λ 10 
    
    pit_time_start(PIT1); ///////////////////////////ʱ�������ʼ 
    buzz_run();                  //������ϵͳ
                          
  
    if(time_sequential%2 == 0) 
    {   ////����  
      

  uart_rx_irq_dis(CH_UART);      //�ر��ж�
      asmple_all_channel();
#ifdef USEING_BLUETOOTH_LOWER_COMPUTER    //run these code when enable lower computer
  uart_rx_irq_en(CH_UART);      //���������ж�
#endif
          pro_ran_time = pit_time_get_us(PIT1);///////////////////////ʱ��������� 

    }
    /*else//////////////����
    {
            if((time_sequential==1) || time_sequential==5 || time_sequential==9 || time_sequential==13 || time_sequential==17)
            {///����
              
              uint8 sd_lode_buff[12];
              level_used_flag = 1;
              buff_num = 0;
            
              ////////////////////////////////////////////////////////��һ��
              left_level = (left_level_buff[0]+left_level_buff[1])/2;
              left_normol = (long)1000*(left_level)/left_level_max;
              
              right_level = (right_level_buff[0]+right_level_buff[1])/2;
              right_normol = (long)1000*(right_level)/right_level_max;
              
              middle_level = (middle_level_buff[0]+middle_level_buff[1])/2;
              middle_normol = (long)1000*(middle_level)/middle_level_max;
              
              //////////////////////////////////////////////////////���㷽�����
              if(run_mode == go_mode || run_mode == stand_mode)
              {
                  v_turn_inc = turn(left_normol,right_normol,gyro_y_ad,middle_normol) - v_turn_last;
              }
              else
              {
                v_turn_inc = 0;
                v_turn = 0;
                v_turn_last=0;
              }
              
              
                turn_out_count = 1;
              
               #ifdef USEING_SD_CARD   //����sd��ʱ�����ô˳���
                    sd_lode_buff[0] = left_normol;///////��ߵ�ֵ
                    sd_lode_buff[1] = left_normol>>8;
                    sd_lode_buff[2] = right_normol;/////////�ұߵ�ֵ
                    sd_lode_buff[3] = right_normol>>8;
                    sd_lode_buff[4] = middle_normol;//////////�м��ֵ
                    sd_lode_buff[5] = middle_normol>>8;
                    sd_lode_buff[6] = real_angle;////////////���
                    sd_lode_buff[7] = real_angle>>8;
                    sd_lode_buff[8] = (gyro_y_ad - gyro_turn_middle);////////////ת��
                    sd_lode_buff[9] = (gyro_y_ad - gyro_turn_middle)>>8;
                    sd_lode_buff[10] = forward_speed;////////////����
                    sd_lode_buff[11] = forward_speed>>8;
                    
                    lode_to_buff(sd_lode_buff,12 );
                    
               #endif
                    
                    
            }
            else if(time_sequential == 19)
            {///�ٶȻ�

              
              speed_get(); //����
              
              
              if(run_mode == go_mode)
              {
                ///////////////////////�ٶȸ�ֵƽ��
                if(speed_goal_give < speed_goal - 30)
                speed_goal_give += 30;
                else if(speed_goal_give > speed_goal + 30) //��ֵ��+-30����ʱƽ����ֵ
                  speed_goal_give -= 30;
                else  //����ֱ�Ӹ�ֵ
                  speed_goal_give = speed_goal;
                
                  v_speed_inc = speed_contral (speed_goal_give,forward_speed) - v_speed_last;
              }
              else
              {
                v_speed_inc = 0;
                v_speed = 0;
                v_speed_last = 0;
              }
              
              
              speed_out_count = 1;
            }
            
            fresh_gesture_data(&raw_acc_gyro);
            if(run_mode == go_mode || run_mode == stand_mode)
            {//ֱ����               

                v_stand_inc = stand(gesture.real_angle,gesture.gyro_x_std) - v_stand_last;
            }
            else
            {
                v_stand_inc = 0;
                v_stand = 0;
                v_stand_last = 0;
            }
                stand_out_count = 1;
          
            
    }           
      
    }
    
    ///////////���
    if(stop_flag == 1)
    {//ͣ��
        if(left_speed > 50)
        {
          left_moto_dir(-10*left_speed);
          right_moto_dir(-10*left_speed);
        }
        else if(left_speed > 10)
        {
          left_moto_dir(-500);
          right_moto_dir(-500);
        }
        else
        {
          left_moto_dir(0);
          right_moto_dir(0);
        }
    }
    else if(1 == 1 && (down_flag == 1 || lost_line_flag == 1 || out_control_flag == 1 ))
    {////���������ֹ���
        left_moto_dir(0);
        right_moto_dir(0);
    }
    else
    {//�������
      v_speed = v_speed_last + v_speed_inc*speed_out_count/20;//ƽ���˲�
       v_turn = v_turn_last + v_turn_inc*turn_out_count/4;
      v_stand = v_stand_last + v_stand_inc*stand_out_count/2;
        
      
        v_speed = 0;
        v_turn = 0;   
        left_out = v_stand - v_turn + v_speed;
        right_out = v_stand + v_turn + v_speed;
      
//        

//  left_out ++;
//   right_out --;
//   if(left_out == 1000)
//     left_out = -1000;
//   
//   if(right_out == -1000)
// right_out = 0;     
//      left_out = right_out;
       
        left_moto_dir(left_out); //���
        right_moto_dir(right_out);
        
        
        if(turn_out_count == 2)//ֱ��
          v_stand_last = v_stand;
        stand_out_count++;
        
        if(turn_out_count == 4)//ת��
          v_turn_last = v_turn;
        turn_out_count++;
        
        if(speed_out_count == 20)//�ٶ�
          v_speed_last = v_speed;
        speed_out_count++;
    }

    
    
    ///////////////ʱ�����  
    time_sequential++;
    if (time_sequential == 20)
      time_sequential = 0;
    ///////////////ʵʱʱ�����
    real_time_ms++;
    if (real_time_ms == 1000)
    {
      real_time_s++;
      real_time_ms = 0;
    }
    
    stop_detect();/////////���ͣ���ź�*/
 
     
}
////////////////////////���ͣ���ź�
void stop_detect (void)
{
  ///////////////////ң����ͣ������
      stop_signal_now = gpio_get(STOP_SIGNAL_PIN);
    if(stop_signal_last == 0 && stop_signal_now == 1)
    {
      if(run_mode == starting_mode)
      {
        run_mode = go_mode;
      }
      else if(run_mode == go_mode || run_mode == normoallization_mode)
      {
          stop_flag = 1;
      }
    }
    stop_signal_last = stop_signal_now; 
   /////////////////��ʱ���� 
    
    if(run_mode == starting_mode && real_time_s > 0 )
    {
      run_mode = go_mode;
    }
    
  //////////////�ű�ͣ��  
    stop_signal_mag_now = gpio_get(STOP_SIGNAL_mag_PIN);
    if(stop_signal_mag_now == 0 && stop_signal_mag_last == 1 && real_time_s > 5)
    {
     stop_flag = 1;
     buzz_mode_flag = 3;
    }
    
    stop_signal_mag_last = stop_signal_mag_now; 
}
