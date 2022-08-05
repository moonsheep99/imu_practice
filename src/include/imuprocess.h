#pragma once    //保证头文件只被编译一次
#include <sensor_msgs/Imu.h>
#include <deque>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Geometry>

#include "comman_lib.h"
#include "so3_math.h"
using namespace std;

struct Pose6D
{
    double offtime;
    boost::array<double, 3> acc;    //boost::array<double, 3> 数组
    boost::array<double, 3> gyr;
    boost::array<double, 3> vel;
    boost::array<double, 3> pos;
    boost::array<double, 3> rot;
};

class ImuProcess
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     //保证内存对齐（march -native就是内存不对齐）//申请内存时重写operator new
    ImuProcess();
    ~ImuProcess();

    void reset();
    void reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
    void set_gyr_cov();
    void set_acc_cov();
    void set_gyr_bias_cov();
    void set_acc_bias_cov();
    Eigen::Matrix<double, 12, 12> Q;
    void Process(const MeasureGroup &meas, StateGroup &state);             //imu处理的主要函数

    Eigen::Matrix<double, 3, 1> cov_acc;    //协方差的维数 和 随机变量的个数有关 （这里不是协方差矩阵，是协方差。多个这样的在一起才是协方差矩阵，其每个元素是向量之间的协方差）
    Eigen::Matrix<double, 3, 1> cov_gyr;    //陀螺仪协方差
    Eigen::Matrix<double, 3, 1> cov_bias_acc;
    Eigen::Matrix<double, 3, 1> cov_bias_gyr;
    Eigen::Matrix<double, 3, 1> mean_acc;   //平均加速度，用于中值离散积分
    Eigen::Matrix<double, 3, 1> mean_gyr;   
    Eigen::Matrix<double, 3, 1> angvel_last;
    Eigen::Matrix<double, 3, 1> acc_last;
    double start_timestamp;
    int init_process_nums = 1;
    bool is_first_frame = true;
    bool imu_need_init = true;

    void IMU_init(const MeasureGroup &meas, StateGroup &state, int &N);                                             //初始化imu函数
    StateGroup imu_preintegration(const StateGroup &state_in, std::deque< sensor_msgs::Imu::ConstPtr > &v_imu);     //预积分函数（前向传播里的）
    void IMU_propagation(const MeasureGroup &meas, StateGroup &state);                                              //前向传播函数
    sensor_msgs::ImuConstPtr last_imu_;     //用于更新指针所指  //ImuConstPtr是智能指针
    deque<sensor_msgs::ImuConstPtr> v_imu_;
    vector<Pose6D> IMUpose;                 //仅用于往回迭代做点云去畸变
};

//构造函数
ImuProcess::ImuProcess() :  is_first_frame(true), imu_need_init(true), start_timestamp(-1) 
{
    init_process_nums = 1;
    Q = Eigen::Matrix<double, 12, 12>::Zero();
    cov_acc = Eigen::Matrix<double, 3, 1>(0.1, 0.1, 0.1); //赋初始值
    cov_gyr = Eigen::Matrix<double, 3, 1>(0.1, 0.1, 0.1);
    cov_bias_acc = Eigen::Matrix<double, 3, 1>(0.1, 0.1, 0.1);
    cov_bias_gyr = Eigen::Matrix<double, 3, 1>(0.1, 0.1, 0.1);
    mean_acc = Eigen::Matrix<double, 3, 1>(0.1, 0.1, 0.1);
    mean_gyr = Eigen::Matrix<double, 3, 1>(0.1, 0.1, 0.1);
    angvel_last = Eigen::Matrix<double, 3, 1>::Zero();
    acc_last = Eigen::Matrix<double, 3, 1>::Zero();
    last_imu_.reset(new sensor_msgs::Imu());       //当调用reset（new xxx())重新赋值时，智能指针指向新的对象
}
//析构
ImuProcess::~ImuProcess() {}

//reset imu函数
void ImuProcess::reset()
{
    cout << " reset imu " << endl;
    mean_acc = Eigen::Matrix<double, 3, 1>(0.1, 0.1, 0.1);
    mean_gyr = Eigen::Matrix<double, 3, 1>(0.1, 0.1, 0.1);
    angvel_last = Eigen::Matrix<double, 3, 1>::Zero();
    acc_last = Eigen::Matrix<double, 3, 1>::Zero();
    start_timestamp = -1;
    init_process_nums = 1;
    IMUpose.clear();
    is_first_frame = true;
    last_imu_.reset(new sensor_msgs::Imu());
    imu_need_init = true;                       //重置时 需重新初始化
}

//避免异常值（vel）输入函数
bool check_state_vel(StateGroup &state)
{
    bool is_fail = false;
    for (int i = 0; i < 3; i++)
    {
        if(fabs(state.Vel(i)) > 10)
        {
            is_fail = true;
            state.Vel(i) = 0.0;
            //cout << "check the outlier: " << "state.vel(" << i << ") = "  << state.Vel(i) << endl;
        }
    }
    return is_fail;
}
//避免异常值（pos）输入函数
void check_state_pos(const StateGroup &state_in, StateGroup &state_out)
{
    if( (state_in.Pos - state_out.Pos).norm() > 1.0 )
    {
        state_out.Pos = state_in.Pos;       //别写反了，写反了就是 in在前面，但是in是const，没法改变
        //cout << "check the outlier: " << "state_in.pos = "  << state_in.Pos.transpose() << ";  state_in.pos = " << state_out.Pos.transpose() << endl;
    }
}

//imu初始化函数
void ImuProcess::IMU_init(const MeasureGroup &meas, StateGroup &state, int &N)
{
    Eigen::Matrix<double, 3, 1> current_acc, current_gyr;
    if(is_first_frame)
    {
        reset();
        N = 1;
        is_first_frame = false;
        const auto &imu_acc = meas.imu.front()->linear_acceleration;
        const auto &imu_gyr = meas.imu.front()->angular_velocity;
        mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_gyr << imu_gyr.x, imu_gyr.y, imu_gyr.z;
    }
    for (const auto &i : meas.imu)  //增强for循环 ———— 遍历deque容器
    {
        const auto &imu_acc = i->linear_acceleration;
        const auto &imu_gyr = i->angular_velocity;
        current_acc << imu_acc.x, imu_acc.y, imu_acc.z;              //函数形参用了cosnt，导到新变量里做
        current_gyr << imu_gyr.x, imu_gyr.y, imu_gyr.z;

        mean_acc = mean_acc + (current_acc - mean_acc)/N;
        mean_gyr = mean_gyr + (current_gyr - mean_gyr)/N;
        N++;
    }
    state.gyr_bias = mean_gyr;                      //陀螺仪偏置
    state.gravity = mean_acc / mean_acc.norm() * 9.805;   //初始重力
    state.Rot = Eigen::Matrix3d::Identity();        //初始旋转
    last_imu_ = meas.imu.back();     
}

std::mutex g_imu_premutex;
StateGroup ImuProcess::imu_preintegration(const StateGroup &state_in, std::deque< sensor_msgs::Imu::ConstPtr > &v_imu)
{
    std::unique_lock< std::mutex > lock( g_imu_premutex );

    StateGroup state_out = state_in;
    if(check_state_vel(state_out))          //check the outlier
    {
        //state_out.display(state_out, "state_out");
        //state_in.display(state_in, "state_in");
    }

    Eigen::Matrix<double,3, 1> acc_imu_( 0, 0, 0 ), angvel_avr_( 0, 0, 0 ), acc_avr_( 0, 0, 0 ), vel_imu_( 0, 0, 0 ), pos_imu_( 0, 0, 0 );  //创建一些中间变量 中间计算用
    Eigen::Matrix3d rot_imu_;   double dt = 0;      int if_first_imu_ = 1;
    vel_imu_ = state_out.Vel;
    pos_imu_ = state_out.Pos;
    rot_imu_ = state_out.Rot;

    for (std::deque<sensor_msgs::Imu::ConstPtr>::iterator it_imu = v_imu.begin(); it_imu != ( v_imu.end()-1 ); it_imu++)
    {
        sensor_msgs::Imu::ConstPtr head = *( it_imu );
        sensor_msgs::Imu::ConstPtr tail = *( it_imu + 1 );

        //计算ak，wk （中值）
        angvel_avr_ << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x), 0.5 * (head->angular_velocity.y + tail->angular_velocity.y), 
                       0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
        acc_avr_    << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x), 0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                       0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);
        angvel_avr_ = angvel_avr_ - state_out.gyr_bias;          //平均角速度 - 陀螺偏置
        acc_avr_    = acc_avr_    - state_out.acc_bias;          //平均加速度 - 加速度偏置（加速度偏置忽略 = 0）

        //计算dt
        if(tail->header.stamp.toSec() < state_out.last_update_time)
        {
            continue;
        }
        if(if_first_imu_)
        {
            if_first_imu_ = 0;  //清除标志位
            dt = tail->header.stamp.toSec() - state_out.last_update_time;   //自己写遇到的问题1：忘记对last_update_time进行更新，或许是抖动或不准的原因（每组第一帧的dt都是和0的时间）
        }
        else
        {
            dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
        }
        if( dt > 0.05 ) //防止非线性误差过大
        {
            dt = 0.05;
        }

        //计算P V Q
        Eigen::Matrix3d Exp_f = Exp(angvel_avr_, dt);
        rot_imu_ = rot_imu_ * Exp_f;
        //cout << "state_out.gravity = " << state_out.gravity << endl;
        //cout << "acc_imu_ before - gravity = " << acc_imu_ << endl;
        //cout << "rot_imu = " << rot_imu_ << "; " << "acc_avr = " << acc_avr_ << "; " << "rot*acc_avr = " << rot_imu_ * acc_avr_ << endl; 
        acc_imu_ = rot_imu_ * acc_avr_ - state_out.gravity;
        //cout << acc_imu_.z() << endl;
        pos_imu_ = pos_imu_ + vel_imu_ * dt + 0.5 * acc_imu_ * dt * dt;
        vel_imu_ = vel_imu_ + acc_imu_ * dt;
        
        //利用完毕————更新数据————把当前帧变为上一帧
        angvel_last = angvel_avr_;  //仅用于点云校正
        acc_last    = acc_imu_;
    }
    state_out.last_update_time = v_imu.back()->header.stamp.toSec();    //更新上一组最后一帧imu的时间
    state_out.Vel = vel_imu_;
    state_out.Pos = pos_imu_;
    state_out.Rot = rot_imu_;
    
    if(check_state_vel(state_out))
    {
        //state_out.display(state_out, "state_out");
        //state_in.display(state_in, "state_in");     //state_in 是const对象，const对象只能调用const函数 void 函数() const {} 要么！就加入static关键字————变成静态成员函数，
                                                    //静态成员函数 为类的全部而服务，不为某一个特定的对象服务（也就是说我不管你这对象是不是const，我都好使）
    }
    check_state_pos(state_in, state_out);
    return state_out;
}

void ImuProcess::IMU_propagation(const MeasureGroup &meas, StateGroup &state)
{
    //上一组的最后帧 给到 当前组的第一帧 （测量量）
    auto v_imu = meas.imu;          // 不能改const里面的，要换一个碗
    v_imu.push_front(last_imu_);    // 把上一组的imu尾帧测量 给到 当前组的第一帧（改变了这组imu的第一帧，但是后续的所有测量都没有变）
    const double &imu_end_time = v_imu.back()->header.stamp.toSec();

    state = imu_preintegration(state, v_imu);       //把state更新了
    
    last_imu_ = meas.imu.back();
    cout << "state is : " << "\nstate.Pos = " << state.Pos.transpose() << "\nstate.Vel = " << state.Vel.transpose() << "\nstate.Rot = \n" << state.Rot << endl;
}

//处理imu的主函数----------------------------------------------------------
void ImuProcess::Process(const MeasureGroup &meas, StateGroup &state)  //MeasureGroup如果在main中，就会出错————头文件重复包含（这里需要main，而main又包含imuprocess，所以单独做一个工具类）
{
    if(meas.imu.empty())
    {
        cout << "meas.imu = empty" << endl;
        return;
    }
    if(imu_need_init)
    {
        IMU_init(meas, state, init_process_nums);
        
        last_imu_ = meas.imu.back();
        imu_need_init = true;
        if(init_process_nums > 10)
        {
            imu_need_init = false;
            cout << "IMU init successed: \n" << "Gravity: " << state.gravity << "\n" << "gyr_bias: " << state.gyr_bias << endl;
        }
        return;
    }
    IMU_propagation(meas, state);
    last_imu_ = meas.imu.back();
}