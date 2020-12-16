#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h> //atoi()
#include <pthread.h>
#include <string>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include <sensor_msgs/LaserScan.h>
#include <strstream>
#define N 3
#define RobotNumber 3
#define PI 3.1415926
//发送的功能原来在程序中是作为阻塞主线程的死循环，现在因为要实时发送机器人的位置，因此还是需要把这个功能变成子线程，所以要在基础的TCP/IP通信功能上进行修改，其他部分暂无问题，包括
//订阅机器人的位置和速度，接受通信信息，机器人巡航
using namespace std;

void extractFiguresFromStr2Vec(std::string str, std::vector<double> &vec);
void * recv_msg(void *arg);//接收消息函数声明
void * send_msg(void *arg);
std::vector<std::string> split(std::string str, std::string pattern);
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg);
int currPos;
struct myData
{
    float x;
    float y;
    float z;
    float w;

};
//void InitialNodes(struct myData n[10]);
static pthread_t send_thread;
bool signal_received;
std::string str;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{

    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/odom", 1000, chatterCallback);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //ros::Subscriber sub = nh.subscribe("/odo777 1, odoCallback);
    //通信模块start
    signal_received=false;
    int socket_fd;
    int port=6666;
    struct myData m[17]={{1.52,1.61,2,2},{0.516,1.53,2,2},{-0.506,1.59,2,2},{2.19,0.461,2,2},{1.79,0.491,2,2},
                         {0.576,0.4,1.5,1.5},{-0.636,0.481,2,2},{-1.72,0.441,2,2},{2.18,-0.611,2,2},{1.53,-0.59,2,2},
                         {0.566,-0.681,2,2},{-0.546,-0.681,2,2},{-1.75,-0.511,2,2},{1.64,-1.68,2,2},{0.566,-1.84,2,2},
                         {-0.676,-1.78,2,2},{-2.58,0.01,2,2}};
    //m[0]={0.5,0.5,2,2};
    //m[1]={-2,0.5,2,1};
    /*m[2]={0.1,0.1,0.1,0.1};
    m[3]={2,2,2,2};
    m[4]={1.5,1.5,1.5,1.5};
    m[5]={0.5,0.5,0.5,0.5};
    m[6]={0.3,0.3,0.3,0.3};
   m[7]={1,2,1,2};
    m[8]={2,1,2,1};
   m[9]={-1,-1,2,2};
*/

    //InitialNodes(m);
    if( port<1025 || port>65535 )//0~1024一般给系统使用，一共可以分配到65535
    {
        printf("端口号范围应为1025~65535");
        return -1;
    }

    //1 创建tcp通信socket
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(socket_fd == -1)
    {
        perror("socket failed!\n");
    }

    //2 连接服务器
    struct sockaddr_in server_addr = {0};//服务器的地址信息
    server_addr.sin_family = AF_INET;//IPv4协议
    server_addr.sin_port = htons(port);//服务器端口号
    server_addr.sin_addr.s_addr = inet_addr("192.168.1.111");//设置服务器IP
    int ret = connect(socket_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if(ret == -1)
    {
        perror("connect failed!\n");
    }
    else
    {
        printf("connect server successful!\n");
    }

    //开启接收线程
    pthread_t recv_thread;//存放线程id       recv_msg：线程执行的函数，将通信socket：new_socket_fd传递进去
    ret = pthread_create(&recv_thread, NULL, recv_msg,  (void*)&socket_fd);
    if(ret != 0)
    {
        printf("开启线程失败\n");
    }
    ret= pthread_create(&send_thread,NULL,send_msg,(void*)&socket_fd);
    if(ret != 0)
    {
        printf("开启发送线程失败\n");
    }



    //while(1){
//	std::cout<<"test message"<<mData.x[0]<<std::endl;
//	std::cout<<"test message"<<mData.x[1]<<std::endl;
//	std::cout<<"test message"<<mData.x[2]<<std::endl;
//	std::cout<<"test message"<<mData.x[3]<<std::endl;
// }




    //通信模块end

    //ros::NodeHandle nh;
    //ros::Subscriber sub=nh.subscribe("/odom",1000,chatterCallback);
    //ros::spin();
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    ros::param::set("/Robot_pos", "Start_Point");

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal Goal0;
    move_base_msgs::MoveBaseGoal Goal1;

    // set up the frame parameters
    Goal0.target_pose.header.frame_id = "map";
    Goal0.target_pose.header.stamp = ros::Time::now();
    Goal1.target_pose.header.frame_id = "map";
    Goal1.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    /*    pick_up_goal.target_pose.pose.position.x = 0.5;
        pick_up_goal.target_pose.pose.position.y = -0.5;
        pick_up_goal.target_pose.pose.position.z = 1.0;
        pick_up_goal.target_pose.pose.orientation.w = 1.0;
        drop_off_goal.target_pose.pose.position.x = -2;
        drop_off_goal.target_pose.pose.position.y = 0.5;
        //drop_off_goal.target_pose.pose.orientation.z = 0.99;
        drop_off_goal.target_pose.pose.orientation.w = 1.0;
    */
    while(1){

        if(signal_received==true)
        {
            sleep(3);
            Goal0.target_pose.pose.position.x = m[0].x;
            Goal0.target_pose.pose.position.y = m[0].y;
            Goal0.target_pose.pose.position.z = m[0].z;
            Goal0.target_pose.pose.orientation.w = m[0].w;
            // pick_up_goal.target_pose.pose.position.x = -0.5;
            // pick_up_goal.target_pose.pose.position.y = 1.7;
            // pick_up_goal.target_pose.pose.position.z = 0.2;
            // pick_up_goal.target_pose.pose.orientation.w = 1.0;
            Goal1.target_pose.pose.position.x = m[1].x;
            Goal1.target_pose.pose.position.y = m[1].y;
            Goal1.target_pose.pose.orientation.z = m[1].z;
            Goal1.target_pose.pose.orientation.w = m[1].w;

            // Send the goal 1
            ROS_INFO("Sending Goal0");
            ac.sendGoal(Goal0);
            ros::param::set("/Robot_pos", "Moving");

            // Wait an infinite time for the results
            ac.waitForResult();

            // Check if the robot reached its goal
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Hooray, the base moved to the pick up Goal0");
                ros::param::set("/Robot_pos", "position_1");
            }
            else {
                ROS_INFO("The base failed to move to the Goal0");
                return 0;
            }

            // wait for 5 seconds
            ros::param::set("/Robot_pos", "Moving");
            ros::Duration(5.0).sleep();
            // send the goal 2
            ROS_INFO("Sending Goal1");
            ac.sendGoal(Goal1);
            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Hooray, the base moved to the Goal1");
                ros::param::set("/Robot_pos", "position_2");
            }
            else {
                ROS_INFO("The base failed to move to the drop off goal");
                return 0;
            }
        }
        else
        {
            std::cout<<"waiting for the signal to arrive"<<std::endl;
        }

    }

    ros::param::set("/Robot_pos", "finish");
    //ros::waitForShutdown();
    //4 关闭通信socket
    pthread_join(send_thread,NULL);
    close(socket_fd);

    return 0;
}


void * send_msg(void *arg)
{
    std::ostrstream oss;
    string s;
    char buf[1024] = {0};
    strcpy(buf,"-1;3;1;1;3;14;1.1;1.2;1.5;2.1;3;5;7;10;");//0.5;-0.5;1.0;1.0;-2;0.5;1;7;8;9;10;11;-1;-2;-3;
    int *socket_fd=(int *)arg;

    while(1)
    {
        oss<<-1<<";"<<RobotNumber<<";"<<1<<";"<<1<<";"<<currPos<<";"<<14<<";"<<0<<";"<<0<<";"<<0<<";"
           <<0<<";"<<currPos<<";"<<currPos<<";"<<currPos<<";"<<currPos<<";"<<endl;
        s=oss.str();
        s.assign(buf,1024);

        //写入节点的处理语句
        sleep(2);
        write(*socket_fd,buf,strlen(buf));
        oss.str();
    }
    pthread_exit(NULL);
}
//接收线程所要执行的函数 接收消息
void * recv_msg(void *arg)
{
    char buf[1024]={0};
    int *socket_fd=(int *)arg;
    int flag;
    string myString(buf,1024);
    vector<string> v;
    string pattern_0="#";
    string pattern_1=";";
    vector < vector <string> > sub_v;
    vector<int> data_v;
    while(1)
    {
        flag=read(*socket_fd, buf, sizeof(buf));//阻塞，等待接收消息
        if(flag>0)
        {
            signal_received=true;
        }
        else
        {
            signal_received=false;
        }
        myString.assign(buf,1024);
        sub_v.clear();
        int i=0;
        while(i!=1023)
        {
            cout<<buf[i];
            i=i+1;
            //cout<<str.length()<<endl;
        }
        i=0;
        cout<<endl;
        //int sign=1;
        v=split(myString,pattern_0);
        sub_v.resize(v.size());
        data_v.resize(v.size());
        for(int i=0;i<=v.size()-1;i++ )
        {
            sub_v[i]=split(v[i],pattern_1);
            if(atoi(sub_v[i][0].c_str())==RobotNumber)
            {
                data_v.clear();
                for(int j=0;j<sub_v[i].size()-1;j++)
                {
                    data_v.push_back(atoi(sub_v[i][j].c_str()));
                }
            }
        }
        sleep(2);
        if(strncmp(buf, "exit", 4) == 0 || strcmp(buf, "") == 0)
        {
            //通知主线程。。。
            printf("GOODBYE\n");
            bzero(buf, 1024);
            pthread_cancel(send_thread);
            break;//退出
        }
    }

    pthread_exit(NULL);
}
std::vector<std::string> split(std::string str, std::string pattern)
{
    std::string::size_type pos;
    std::vector<std::string> result;

    str += pattern;//扩展字符串以方便操作
    int size = str.size();

    for (int i = 0; i<size; i++) {
        pos = str.find(pattern, i);
        if (pos<size) {
            std::string s = str.substr(i, pos - i);
            result.push_back(s);
            i = pos + pattern.size() - 1;
        }
    }
    return result;
}

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::ostrstream oss;
    //ROS_INFO("Seq: [%d]", msg->header.seq);
    //ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    //ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    /*robot_a.position_x=msg->pose.pose.position.x;
    robot_a.position_y=msg->pose.pose.position.y;
    robot_a.position_z=msg->pose.pose.position.z;
    robot_a.orientation_x=msg->pose.pose.orientation.x;
    robot_a.orientation_y=msg->pose.pose.orientation.y;
    robot_a.orientation_z=msg->pose.pose.orientation.z;
    robot_a.twist_linear_x=msg->twist.twist.linear.x;
    robot_a.twist_linear_y=msg->twist.twist.linear.y;
    robot_a.twist_linear_z=msg->twist.twist.linear.z;
    robot_a.twist_angular_x=msg->twist.twist.angular.x;
    robot_a.twist_angular_y=msg->twist.twist.angular.y;
    robot_a.twist_angular_z=msg->twist.twist.angular.z;
*/
    oss << msg->pose.pose.position.x <<";"<< msg->pose.pose.position.y<< ";"<<msg->pose.pose.position.z<<";"<<msg->pose.pose.orientation.x<<";"<<msg->pose.pose.orientation.y<<";"<<msg->pose.pose.orientation.z<<";"<<msg->twist.twist.linear.x<<";"<<msg->twist.twist.linear.y<<";"<<msg->twist.twist.linear.z<<";"<<msg->twist.twist.angular.x<<";"<<msg->twist.twist.angular.y<<";"<<msg->twist.twist.angular.z<<";";

    std::cout << oss.str() << std::endl;
    str=oss.str();

}
/*void initial(struct myData n[10])
{
    n[0]={0.5,0.5,2,2};
    n[1]={-2,0.5,2,1};
    n[2]={0.1,0.1,0.1,0.1};
    n[3]={2,2,2,2};
    n[4]={1.5,1.5,1.5,1.5};
    n[5]={0.5,0.5,0.5,0.5};
    n[6]={0.3,0.3,0.3,0.3};
    n[7]={1,2,1,2};
    n[8]={2,1,2,1};
    n[9]={-1,-1,2,2};
}*/
