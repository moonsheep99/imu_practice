#include <omp.h>    //并行环境变量
#include <mutex>    //线程锁
#include <math.h>
#include <thread>   //多线程
#include <fstream>  //文件输出
#include <csignal>  //进程信号处理，如ctrl+c
#include <unistd.h> //unix的std库。许多在Linux下开发的C程序都需要头文件unistd.h，但VC中没有个头文件， 所以用VC编译总是报错。

#include <ros/ros.h>
#include <Eigen/Core>   
#include "imuprocess.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Vector3.h>
#include "comman_lib.h"


#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)
using namespace std;

std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
double last_timestamp_imu = -0.01;
mutex mtx_buffer;   //定义互斥量
condition_variable sig_buffer;


MeasureGroup measurement;                   //创建对象 全局变量 -- 测量量（imu）
StateGroup g_state;                         //创建状态 全局变量 -- 系统状态（imu）//构造函数里自动赋了初置
shared_ptr<ImuProcess> p_imu(new ImuProcess());

nav_msgs::Path IMU_path;                    //IMU path 用于发布
ros::Publisher pub_pre_odometry;            //imu位姿先验的发布者
ros::Publisher pub_pre_path;                //

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    //cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));   //堆区开辟空间给msg_in，用msg指向他（简单来说就是拷贝，因为用const接收）
    double imu_timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();          //-----------------------lock
    if(imu_timestamp < last_timestamp_imu)
    {
        cout << "时间戳可能错了" << endl;
        imu_buffer.clear();
    }
    last_timestamp_imu = imu_timestamp; //时间戳传递
    //cout << "in cbk: msg = \n" << msg->linear_acceleration << endl;
    imu_buffer.push_back(msg);
    //cout << "push back to the buffer finished" << endl;
    mtx_buffer.unlock();        //-----------------------unlock
    sig_buffer.notify_all();
}

bool buffer_to_meas(MeasureGroup &meas) //把buffer数据拿到meas中
{
    if(imu_buffer.empty())
    {
        //cout << "imu_buffer = 0" << endl;
        return false;
    }
    meas.imu.clear();
    while (!imu_buffer.empty()) //若imu_buffer不为空，则把buffer中的东西转到meas中
    {
        //cout << "imu_buffer != 0" << endl;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front(); //deque容器只能对头尾进行操作
    }
    return true;
}

void publish_pre_imu(const StateGroup &state)
{
    cout << "Pos:\n" << state.Pos << endl;
    cout << "Rot:\n" << state.Rot << endl; 
    Eigen::Quaterniond q = Eigen::Quaterniond(state.Rot);
    
    nav_msgs::Odometry imu_pre_odometry;
    imu_pre_odometry.header.frame_id = "base_link";
    imu_pre_odometry.child_frame_id = "/body";
    imu_pre_odometry.header.stamp = ros::Time().now();
    imu_pre_odometry.pose.pose.position.x = state.Pos(0);
    imu_pre_odometry.pose.pose.position.y = state.Pos(1);
    imu_pre_odometry.pose.pose.position.z = state.Pos(2);
    imu_pre_odometry.pose.pose.orientation.w = q.w();
    imu_pre_odometry.pose.pose.orientation.x = q.x();
    imu_pre_odometry.pose.pose.orientation.y = q.y();
    imu_pre_odometry.pose.pose.orientation.z = q.z();
    pub_pre_odometry.publish(imu_pre_odometry); //odometry是一个有方向的箭头（pose在header.frame_id坐标系下，twist再child_frame_id坐标系下）

    geometry_msgs::PoseStamped imu_pre_path;
    imu_pre_path.header.stamp = ros::Time().now();
    imu_pre_path.header.frame_id = "base_link";
    imu_pre_path.pose.position.x = state.Pos(0);
    imu_pre_path.pose.position.y = state.Pos(1);
    imu_pre_path.pose.position.z = state.Pos(2);
    imu_pre_path.pose.orientation.x = q.x();
    imu_pre_path.pose.orientation.y = q.y();
    imu_pre_path.pose.orientation.z = q.z();
    imu_pre_path.pose.orientation.w = q.w();
    IMU_path.header.frame_id = "base_link";
    IMU_path.poses.push_back(imu_pre_path);
    pub_pre_path.publish(IMU_path);             //path是一条连续的路径
}

//主函数
int main(int argc, char** argv)
{
    ros::init(argc,argv,"imuprocess");      //初始化节点
    ros::NodeHandle nh;
    ros::Subscriber sub_imu = nh.subscribe( "/handsfree/imu",200000,imu_cbk);
    pub_pre_odometry = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    pub_pre_path = nh.advertise<nav_msgs::Path>("/Path_IMU", 100000);
    ros::Rate rate(5000);       //5000hz = 0.02s = 20 ms
    bool status = ros::ok();

    while(status)
    {
        ros::spinOnce();
        if(buffer_to_meas(measurement))    //把imu信息从缓存器转到meas变量中 （注意！ 是地址传递，虽然动形参，但是等于动实参）
        {
            p_imu->Process(measurement, g_state);
            publish_pre_imu(g_state);
        }
        status = ros::ok();
        rate.sleep();
    }
    
    return 0;
}