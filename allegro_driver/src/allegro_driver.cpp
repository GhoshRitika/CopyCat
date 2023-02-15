#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <termios.h>  //_getch
#include <string.h>
#include <pthread.h>
#include <BHand/BHand.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include "allegro_lib/canAPI.h"
#include "allegro_lib/rDeviceAllegroHandCANDef.h"
#include "allegro_lib/RockScissorsPaper.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
// #include "sensor_msgs/msg/jointstate.hpp"


#define PEAKCAN (1)

typedef char    TCHAR;
#define _T(X)   X
#define _tcsicmp(x, y)   strcmp(x, y)

using namespace std::chrono_literals;

const double delT = 0.003;
int CAN_Ch = 0;
bool ioThreadRun = false;
pthread_t        hThread;
int recvNum = 0;
int sendNum = 0;
double statTime = -1.0;

AllegroHand_DeviceMemory_t vars;

double curTime = 0.0;

/////////////////////////////////////////////////////////////////////////////////////////
// for BHand library
BHand* pBHand = NULL;
double q[MAX_DOF];
double q_des[MAX_DOF];
double tau_des[MAX_DOF];
double cur_des[MAX_DOF];

// USER HAND CONFIGURATION
const bool	RIGHT_HAND = false;
const int	HAND_VERSION = 4;

const double tau_cov_const_v4 = 1200.0;

// char Getch();
bool OpenCAN();
void CloseCAN();
int GetCANChannelIndex(const TCHAR* cname);
bool CreateBHandAlgorithm();
void DestroyBHandAlgorithm();
void ComputeTorque();

// CAN communication thread
static void* ioThreadProc(void* inst)
{
    int id;
    int len;
    unsigned char data[8];
    unsigned char data_return = 0;
    int i;

    while (ioThreadRun)
    {
        /* wait for the event */
        while (0 == get_message(CAN_Ch, &id, &len, data, FALSE))
        {
//            printf(">CAN(%d): ", CAN_Ch);
//            for(int nd=0; nd<len; nd++)
//                printf("%02x ", data[nd]);
//            printf("\n");

            switch (id)
            {
            case ID_RTR_HAND_INFO:
            {
                printf(">CAN(%d): AllegroHand hardware version: 0x%02x%02x\n", CAN_Ch, data[1], data[0]);
                printf("                      firmware version: 0x%02x%02x\n", data[3], data[2]);
                printf("                      hardware type: %d(%s)\n", data[4], (data[4] == 0 ? "right" : "left"));
                printf("                      temperature: %d (celsius)\n", data[5]);
                printf("                      status: 0x%02x\n", data[6]);
                printf("                      servo status: %s\n", (data[6] & 0x01 ? "ON" : "OFF"));
                printf("                      high temperature fault: %s\n", (data[6] & 0x02 ? "ON" : "OFF"));
                printf("                      internal communication fault: %s\n", (data[6] & 0x04 ? "ON" : "OFF"));
            }
                break;
            case ID_RTR_SERIAL:
            {
                printf(">CAN(%d): AllegroHand serial number: SAH0%d0 %c%c%c%c%c%c%c%c\n", CAN_Ch, HAND_VERSION
                       , data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
            }
                break;
            case ID_RTR_FINGER_POSE_1:
            case ID_RTR_FINGER_POSE_2:
            case ID_RTR_FINGER_POSE_3:
            case ID_RTR_FINGER_POSE_4:
            {
                int findex = (id & 0x00000007);

                vars.enc_actual[findex*4 + 0] = (short)(data[0] | (data[1] << 8));
                vars.enc_actual[findex*4 + 1] = (short)(data[2] | (data[3] << 8));
                vars.enc_actual[findex*4 + 2] = (short)(data[4] | (data[5] << 8));
                vars.enc_actual[findex*4 + 3] = (short)(data[6] | (data[7] << 8));
                data_return |= (0x01 << (findex));
                recvNum++;

//                printf(">CAN(%d): Encoder[%d] Count : %6d %6d %6d %6d\n"
//                    , CAN_Ch, findex
//                    , vars.enc_actual[findex*4 + 0], vars.enc_actual[findex*4 + 1]
//                    , vars.enc_actual[findex*4 + 2], vars.enc_actual[findex*4 + 3]);

                if (data_return == (0x01 | 0x02 | 0x04 | 0x08))
                {
                    // convert encoder count to joint angle
                    for (i=0; i<MAX_DOF; i++)
                    {
                        q[i] = (double)(vars.enc_actual[i])*(333.3/65536.0)*(3.141592/180.0);
                        // q[i] = (double)(vars.enc_actual[i]*enc_dir[i]-32768-enc_offset[i])*(333.3/65536.0)*(3.141592/180.0);
                    }

                    // print joint angles
//                    for (int i=0; i<4; i++)
//                    {
//                        printf(">CAN(%d): Joint[%d] Pos : %5.1f %5.1f %5.1f %5.1f\n"
//                            , CAN_Ch, i, q[i*4+0]*RAD2DEG, q[i*4+1]*RAD2DEG, q[i*4+2]*RAD2DEG, q[i*4+3]*RAD2DEG);
//                    }

                    // compute joint torque
                    ComputeTorque();

                    // convert desired torque to desired current and PWM count
                    for (int i=0; i<MAX_DOF; i++)
                    {
                        cur_des[i] = tau_des[i];
                        // cur_des[i] = tau_des[i] * motor_dir[i];
                        if (cur_des[i] > 1.0) cur_des[i] = 1.0;
                        else if (cur_des[i] < -1.0) cur_des[i] = -1.0;
                    }

                    // send torques
                    for (int i=0; i<4;i++)
                    {
                        vars.pwm_demand[i*4+0] = (short)(cur_des[i*4+0]*tau_cov_const_v4);
                        vars.pwm_demand[i*4+1] = (short)(cur_des[i*4+1]*tau_cov_const_v4);
                        vars.pwm_demand[i*4+2] = (short)(cur_des[i*4+2]*tau_cov_const_v4);
                        vars.pwm_demand[i*4+3] = (short)(cur_des[i*4+3]*tau_cov_const_v4);

                        command_set_torque(CAN_Ch, i, &vars.pwm_demand[4*i]);
                        //usleep(5);
                    }
                    sendNum++;
                    curTime += delT;
                    data_return = 0;
                }
            }
                break;
            case ID_RTR_IMU_DATA:
            {
                printf(">CAN(%d): AHRS Roll : 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
                printf("               Pitch: 0x%02x%02x\n", data[2], data[3]);
                printf("               Yaw  : 0x%02x%02x\n", data[4], data[5]);
            }
                break;
            case ID_RTR_TEMPERATURE_1:
            case ID_RTR_TEMPERATURE_2:
            case ID_RTR_TEMPERATURE_3:
            case ID_RTR_TEMPERATURE_4:
            {
                int sindex = (id & 0x00000007);
                int celsius = (int)(data[0]      ) |
                              (int)(data[1] << 8 ) |
                              (int)(data[2] << 16) |
                              (int)(data[3] << 24);
                printf(">CAN(%d): Temperature[%d]: %d (celsius)\n", CAN_Ch, sindex, celsius);
            }
                break;
            default:
                printf(">CAN(%d): unknown command %d, len %d\n", CAN_Ch, id, len);
                /*for(int nd=0; nd<len; nd++)
                    printf("%d \n ", data[nd]);*/
                //return;
            }
        }
    }
    return NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Compute control torque for each joint using BHand library
void ComputeTorque()
{
    if (!pBHand) return;
    pBHand->SetJointPosition(q); // tell BHand library the current joint positions
    pBHand->SetJointDesiredPosition(q_des);
    pBHand->UpdateControl(0);
    pBHand->GetJointTorque(tau_des);
}

/////////////////////////////////////////////////////////////////////////////////////////
// Open a CAN data channel
bool OpenCAN()
{
#if defined(PEAKCAN)
    CAN_Ch = GetCANChannelIndex(_T("USBBUS1"));
#elif defined(IXXATCAN)
    CAN_Ch = 1;
#elif defined(SOFTINGCAN)
    CAN_Ch = 1;
#else
    CAN_Ch = 1;
#endif
    printf(">CAN(%d): open\n", CAN_Ch);

    int ret = command_can_open(CAN_Ch);
    if(ret < 0)
    {
        printf("ERROR command_can_open !!! \n");
        return false;
    }

    // initialize CAN I/O thread
    ioThreadRun = true;
    /*int ioThread_error = */pthread_create(&hThread, NULL, ioThreadProc, 0);
    printf(">CAN: starts listening CAN frames\n");

    // query h/w information
    printf(">CAN: query system information\n");
    ret = request_hand_information(CAN_Ch);
    if(ret < 0)
    {
        printf("ERROR request_hand_information !!! \n");
        command_can_close(CAN_Ch);
        return false;
    }
    ret = request_hand_serial(CAN_Ch);
    if(ret < 0)
    {
        printf("ERROR request_hand_serial !!! \n");
        command_can_close(CAN_Ch);
        return false;
    }

    // set periodic communication parameters(period)
    printf(">CAN: Comm period set\n");
    short comm_period[3] = {3, 0, 0}; // millisecond {position, imu, temperature}
    ret = command_set_period(CAN_Ch, comm_period);
    if(ret < 0)
    {
        printf("ERROR command_set_period !!! \n");
        command_can_close(CAN_Ch);
        return false;
    }

    // servo on
    printf(">CAN: servo on\n");
    ret = command_servo_on(CAN_Ch);
    if(ret < 0)
    {
        printf("ERROR command_servo_on !!! \n");
        command_set_period(CAN_Ch, 0);
        command_can_close(CAN_Ch);
        return false;
    }

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Close CAN data channel
void CloseCAN()
{
    printf(">CAN: stop periodic communication\n");
    int ret = command_set_period(CAN_Ch, 0);
    if(ret < 0)
    {
        printf("ERROR command_can_stop !!! \n");
    }

    if (ioThreadRun)
    {
        printf(">CAN: stoped listening CAN frames\n");
        ioThreadRun = false;
        int status;
        pthread_join(hThread, (void **)&status);
        hThread = 0;
    }

    printf(">CAN(%d): close\n", CAN_Ch);
    ret = command_can_close(CAN_Ch);
    if(ret < 0) printf("ERROR command_can_close !!! \n");
}

/////////////////////////////////////////////////////////////////////////////////////////
// Load and create grasping algorithm
bool CreateBHandAlgorithm()
{
    if (RIGHT_HAND)
        pBHand = bhCreateRightHand();
    else
        pBHand = bhCreateLeftHand();

    if (!pBHand) return false;
    pBHand->SetMotionType(eMotionType_NONE);
    pBHand->SetTimeInterval(delT);
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Destroy grasping algorithm
void DestroyBHandAlgorithm()
{
    if (pBHand)
    {
#ifndef _DEBUG
        delete pBHand;
#endif
        pBHand = NULL;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////
// Get channel index for Peak CAN interface
int GetCANChannelIndex(const TCHAR* cname)
{
    if (!cname) return 0;

    if (!_tcsicmp(cname, _T("0")) || !_tcsicmp(cname, _T("PCAN_NONEBUS")) || !_tcsicmp(cname, _T("NONEBUS")))
        return 0;
    else if (!_tcsicmp(cname, _T("1")) || !_tcsicmp(cname, _T("PCAN_ISABUS1")) || !_tcsicmp(cname, _T("ISABUS1")))
        return 1;
    else if (!_tcsicmp(cname, _T("2")) || !_tcsicmp(cname, _T("PCAN_ISABUS2")) || !_tcsicmp(cname, _T("ISABUS2")))
        return 2;
    else if (!_tcsicmp(cname, _T("3")) || !_tcsicmp(cname, _T("PCAN_ISABUS3")) || !_tcsicmp(cname, _T("ISABUS3")))
        return 3;
    else if (!_tcsicmp(cname, _T("4")) || !_tcsicmp(cname, _T("PCAN_ISABUS4")) || !_tcsicmp(cname, _T("ISABUS4")))
        return 4;
    else if (!_tcsicmp(cname, _T("5")) || !_tcsicmp(cname, _T("PCAN_ISABUS5")) || !_tcsicmp(cname, _T("ISABUS5")))
        return 5;
    else if (!_tcsicmp(cname, _T("7")) || !_tcsicmp(cname, _T("PCAN_ISABUS6")) || !_tcsicmp(cname, _T("ISABUS6")))
        return 6;
    else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS7")) || !_tcsicmp(cname, _T("ISABUS7")))
        return 7;
    else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS8")) || !_tcsicmp(cname, _T("ISABUS8")))
        return 8;
    else if (!_tcsicmp(cname, _T("9")) || !_tcsicmp(cname, _T("PCAN_DNGBUS1")) || !_tcsicmp(cname, _T("DNGBUS1")))
        return 9;
    else if (!_tcsicmp(cname, _T("10")) || !_tcsicmp(cname, _T("PCAN_PCIBUS1")) || !_tcsicmp(cname, _T("PCIBUS1")))
        return 10;
    else if (!_tcsicmp(cname, _T("11")) || !_tcsicmp(cname, _T("PCAN_PCIBUS2")) || !_tcsicmp(cname, _T("PCIBUS2")))
        return 11;
    else if (!_tcsicmp(cname, _T("12")) || !_tcsicmp(cname, _T("PCAN_PCIBUS3")) || !_tcsicmp(cname, _T("PCIBUS3")))
        return 12;
    else if (!_tcsicmp(cname, _T("13")) || !_tcsicmp(cname, _T("PCAN_PCIBUS4")) || !_tcsicmp(cname, _T("PCIBUS4")))
        return 13;
    else if (!_tcsicmp(cname, _T("14")) || !_tcsicmp(cname, _T("PCAN_PCIBUS5")) || !_tcsicmp(cname, _T("PCIBUS5")))
        return 14;
    else if (!_tcsicmp(cname, _T("15")) || !_tcsicmp(cname, _T("PCAN_PCIBUS6")) || !_tcsicmp(cname, _T("PCIBUS6")))
        return 15;
    else if (!_tcsicmp(cname, _T("16")) || !_tcsicmp(cname, _T("PCAN_PCIBUS7")) || !_tcsicmp(cname, _T("PCIBUS7")))
        return 16;
    else if (!_tcsicmp(cname, _T("17")) || !_tcsicmp(cname, _T("PCAN_PCIBUS8")) || !_tcsicmp(cname, _T("PCIBUS8")))
        return 17;
    else if (!_tcsicmp(cname, _T("18")) || !_tcsicmp(cname, _T("PCAN_USBBUS1")) || !_tcsicmp(cname, _T("USBBUS1")))
        return 18;
    else if (!_tcsicmp(cname, _T("19")) || !_tcsicmp(cname, _T("PCAN_USBBUS2")) || !_tcsicmp(cname, _T("USBBUS2")))
        return 19;
    else if (!_tcsicmp(cname, _T("20")) || !_tcsicmp(cname, _T("PCAN_USBBUS3")) || !_tcsicmp(cname, _T("USBBUS3")))
        return 20;
    else if (!_tcsicmp(cname, _T("21")) || !_tcsicmp(cname, _T("PCAN_USBBUS4")) || !_tcsicmp(cname, _T("USBBUS4")))
        return 21;
    else if (!_tcsicmp(cname, _T("22")) || !_tcsicmp(cname, _T("PCAN_USBBUS5")) || !_tcsicmp(cname, _T("USBBUS5")))
        return 22;
    else if (!_tcsicmp(cname, _T("23")) || !_tcsicmp(cname, _T("PCAN_USBBUS6")) || !_tcsicmp(cname, _T("USBBUS6")))
        return 23;
    else if (!_tcsicmp(cname, _T("24")) || !_tcsicmp(cname, _T("PCAN_USBBUS7")) || !_tcsicmp(cname, _T("USBBUS7")))
        return 24;
    else if (!_tcsicmp(cname, _T("25")) || !_tcsicmp(cname, _T("PCAN_USBBUS8")) || !_tcsicmp(cname, _T("USBBUS8")))
        return 25;
    else if (!_tcsicmp(cname, _T("26")) || !_tcsicmp(cname, _T("PCAN_PCCBUS1")) || !_tcsicmp(cname, _T("PCCBUS1")))
        return 26;
    else if (!_tcsicmp(cname, _T("27")) || !_tcsicmp(cname, _T("PCAN_PCCBUS2")) || !_tcsicmp(cname, _T("PCCBUS2")))
        return 27;
    else
        return 0;
}

class AllegroDriver : public rclcpp::Node
{
  public:
    AllegroDriver()
    : Node("allegro_driver"), count_(0.0)
    {
    //   rock_server_= create_service<std_srvs::srv::Empty>("rock", std::bind(&AllegroDriver::rock_callback, this, std::placeholders::_1, std::placeholders::_2));
    //   paper_server_= create_service<std_srvs::srv::Empty>("paper", std::bind(&AllegroDriver::paper_callback, this, std::placeholders::_1, std::placeholders::_2));
    //   scissor_server_= create_service<std_srvs::srv::Empty>("scissor", std::bind(&AllegroDriver::scissor_callback, this, std::placeholders::_1, std::placeholders::_2));
    //   trial_server_= create_service<std_srvs::srv::Empty>("trial", std::bind(&AllegroDriver::trial_callback, this, std::placeholders::_1, std::placeholders::_2));
    //   joint_state_pub = create_publisher<sensor_msgs::msg::JointState>("~/joint_state", 10);
    joint_sub = create_subscription<std_msgs::msg::Float64MultiArray>("Joint_angles", 10, std::bind(&AllegroDriver::angle_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(300ms, std::bind(&AllegroDriver::timer_callback, this));

    }

  private:
    void timer_callback()
    { 
        // count_ += delT; 
        
        // if (input == '1'){
        //     MotionRock();
        // }
        // else if (input == '2'){
        //     MotionPaper();
        // }
        // else if(input == '3'){
        //     MotionScissors();
        // }
        // else if (input == 'y'){
        //     pBHand->SetMotionType(eMotionType_GRAVITY_COMP);
        //     sleep(5);
            
        //     std::copy(q, q+MAX_DOF, q_temp);
        //     // RCLCPP_INFO(this->get_logger(), "Copying: %lf %lf %lf %lf", q_temp[0],q_temp[1],q_temp[2],q_temp[3]);
        //     RCLCPP_INFO(this->get_logger(), "finger 0: %lf %lf %lf %lf", q[0],q[1],q[2],q[3]);
        //     RCLCPP_INFO(this->get_logger(), "finger 1: %lf %lf %lf %lf", q[4],q[5],q[6],q[7]);
        //     RCLCPP_INFO(this->get_logger(), "finger 2: %lf %lf %lf %lf", q[8],q[9],q[10],q[11]);
        //     RCLCPP_INFO(this->get_logger(), "finger 3: %lf %lf %lf %lf", q[12],q[13],q[14],q[15]);
        //     input = 'k';
        // }
        // else if (input == 'n'){
        //     std::copy(q_temp, q_temp+MAX_DOF, q_des);
        //     RCLCPP_INFO(this->get_logger(), "Publishing: %lf %lf %lf %lf", q_des[0],q_des[1],q_des[2],q_des[3]);
        //     pBHand->SetMotionType(eMotionType_JOINT_PD);
        //     // SetGainsRSP();
        //     // if (!pBHand) return;
        //     double kp[] = {
        //         500, 800, 900, 500,
        //         500, 800, 900, 500,
        //         500, 800, 900, 500,
        //         1000, 700, 600, 600
        //     };
        //     double kd[] = {
        //         25, 50, 55, 40,
        //         25, 50, 55, 40,
        //         25, 50, 55, 40,
        //         50, 50, 50, 40
        //     };
        //     pBHand->SetGainsEx(kp, kd);
        // }
        // RCLCPP_INFO(this->get_logger(), "Publishing: %c", input);
        // pBHand->SetMotionType(eMotionType_GRAVITY_COMP);
            std::copy(qnew, qnew+MAX_DOF, q_des);
            RCLCPP_INFO(this->get_logger(), "Publishing:");
            pBHand->SetMotionType(eMotionType_JOINT_PD);
            // SetGainsRSP();
            // if (!pBHand) return;
            double kp[] = {
                500, 800, 900, 500,
                500, 800, 900, 500,
                500, 800, 900, 500,
                1000, 700, 600, 600
            };
            double kd[] = {
                25, 50, 55, 40,
                25, 50, 55, 40,
                25, 50, 55, 40,
                50, 50, 50, 40
            };
            pBHand->SetGainsEx(kp, kd);
    }

    void angle_callback(const std_msgs::msg::Float64MultiArray &msg){
        // std::vector<double> qnew(msg.data);
        // *qnew= &msg.data;
        std::copy(msg.data.begin(), msg.data.end(), qnew);
    }
    // void rock_callback(std_srvs::srv::Empty::Request::SharedPtr,
    //                     std_srvs::srv::Empty::Response::SharedPtr)
    // {
    //     input = '1';
    // }
    // void paper_callback(std_srvs::srv::Empty::Request::SharedPtr,
    //                     std_srvs::srv::Empty::Response::SharedPtr)
    // {
    //     input = '2';
    // }
    // void scissor_callback(std_srvs::srv::Empty::Request::SharedPtr,
    //                     std_srvs::srv::Empty::Response::SharedPtr)
    // {
    //     input = '3';
    // }
    // void trial_callback(std_srvs::srv::Empty::Request::SharedPtr,
    //                     std_srvs::srv::Empty::Response::SharedPtr)
    // {
    //     if (prev != 'y')
    //     {
    //         input = 'y';
    //     }
    //     else{
    //         input = 'n';
    //     }
    //     prev = input;
    // }

    // void publishData() {
    // // current position, velocity and effort (torque) published
    // current_joint_state.header.stamp = tnow;
    // for (int i = 0; i < DOF_JOINTS; i++) {
    //     current_joint_state.position[i] = current_position_filtered[i];
    //     current_joint_state.velocity[i] = current_velocity_filtered[i];
    //     current_joint_state.effort[i] = desired_torque[i];
    // }
    // joint_state_pub.publish(current_joint_state);
    // }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sub;
    // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr rock_server_;
    // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr paper_server_;
    // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr scissor_server_;
    // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr trial_server_;
    size_t count_;
    char input='k';
    char prev='n';
    double qnew[MAX_DOF];
    // sensor_msgs::msg::JointState current_state;
};



int main(int argc, char * argv[])
{
  memset(&vars, 0, sizeof(vars));
  memset(q, 0, sizeof(q));
  memset(q_des, 0, sizeof(q_des));
  memset(tau_des, 0, sizeof(tau_des));
  memset(cur_des, 0, sizeof(cur_des));
  curTime = 0.0;
  if (CreateBHandAlgorithm() && OpenCAN()){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AllegroDriver>());
    rclcpp::shutdown();
  }
  
  CloseCAN();
  DestroyBHandAlgorithm();
  return 0;
}
