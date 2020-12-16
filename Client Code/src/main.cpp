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
#include <math.h>
#include <string>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include <sensor_msgs/LaserScan.h>
#include <strstream>
#include <vector>
#define N 3
#define PI 3.1415926
//发送的功能原来在程序中是作为阻塞主线程的死循环，现在因为要实时发送机器人的位置，因此还是需要把这个功能变成子线程，所以要在基础的TCP/IP通信功能上进行修改，其他部分暂无问题，包括
//订阅机器人的位置和速度，接受通信信息，机器人巡航
using namespace std;

//函数
void * recv_msg(void *arg);//接收消息函数声明
void * send_msg(void *arg);
void * executePath(void *arg);
bool isLocalPathUpdated(vector<int>path);
vector<string> split(string str, string pattern);
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg);
float CalculateDistance(struct robot_msg RobotPos,struct TempGoal NextNode);
//结构体
struct myData	{ float x,y,z,w;};
struct robot_msg
{
    float position_x,position_y,position_z;
    float orientation_x,orientation_y,orientation_z;
    float twist_linear_x,twist_linear_y,twist_linear_z;
    float twist_angular_x,twist_angular_y,twist_angular_z;
};
struct BaseInfo
{
    int RobotNumber;
    bool OnlineFlag;
    int WorkingState;
    int currentNode;
    int FinalGoalNode;
};
struct NodeInfo
{
    vector<int>NodeNumber;
    vector<pair<float,float> > NodesPos;
    vector<pair<float,float> > NodesVel;
    vector<float> w;
};
struct TempGoal
{
    float x,y,z,w;
    int NodeNum;
};
enum{RequestInitial,RequireNewPath,Pending,ReachFinalGoal};
//变量

vector<int> Nodes;

static struct myData mData;
static struct BaseInfo myInfo;
static struct robot_msg RobotInfo;
static struct NodeInfo myNode;
static pthread_t send_thread,execute_thread;
string str;
bool positionUpdate=false;
bool PosIntialedInServer=false;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    memset(&RobotInfo,0,sizeof(struct robot_msg));
    memset(&myInfo,0,sizeof(struct BaseInfo));//RobotNumber=0,offline,silent,node=0,
    myInfo.RobotNumber=1;
    myInfo.OnlineFlag=true;
    myInfo.FinalGoalNode=14;

    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/odom", 1000, chatterCallback);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //ros::Subscriber sub = nh.subscribe("/odo777 1, odoCallback);
    //通信模块start

    int socket_fd;
    int port=7777;//7777

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
    server_addr.sin_addr.s_addr = inet_addr("192.168.1.109");//设置服务器IP
    int nNetTimeout=2000;
    setsockopt(socket_fd,SOL_SOCKET,SO_SNDTIMEO,(char *)&nNetTimeout,sizeof(int));//设置收发时限为2秒
    setsockopt(socket_fd,SOL_SOCKET,SO_RCVTIMEO,(char *)&nNetTimeout,sizeof(int));
    //Err=setsockopt(socket_fd,IPPROTO_TCP,TCP_NODEALAY,&chOpt,sizeof((char)));

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
    //pthread_join(recv_thread,NULL);

    pthread_join(send_thread,NULL);
    ret=pthread_create(&execute_thread,NULL,executePath,NULL);
    if(ret!=0)
    {
        printf("开启执行线程失败\n");
    }
    //pthread_join(execute_thread,NULL);
    ret= pthread_create(&send_thread,NULL,send_msg,(void*)&socket_fd);
    if(ret != 0)
    {
        printf("开启发送线程失败\n");
    }
    //ros::waitForShutdown();
    //4 关闭通信socket
    pthread_join(send_thread,NULL);
    //pthread_join(execute_thread,NULL);
    //pthread_join(recv_msg,NULL);
    close(socket_fd);

    return 0;
}

void * send_msg(void *arg)
{
    int  sendFlag;
    char buf[1024] = {0};
    char send_buf[1024]={0};
    int *socket_fd=(int *)arg;
    myInfo.currentNode=5;
    while(1)
    {
        memset(send_buf,0,1024);
        if(positionUpdate==false)//chattercallback里面的位置信息是否得到更新，避免发送的位置是错误位置
        {
            sleep(1);
            continue;
        }
        sleep(2);
        std::ostrstream oss;
        string s;
        int j;
        s.clear();
        //刚开始，机器人发坐标点给服务器，让服务器找到机器人的位置，并在服务器标注出节点，之后，机器人与服务器之间仅通过节点交流，虽然机器人坐标也发送，但是其实不需要
//最后发送的时候记得以#结尾，否则无法解包
        oss<<-1<<";"<<myInfo.RobotNumber<<";"<<myInfo.OnlineFlag<<";"<<myInfo.WorkingState<<";"<<myInfo.currentNode<<";"
           <<myInfo.FinalGoalNode<<";"<<1<<";"<<2<<";"
           <<3 <<";"<<4<<";"
           <<RobotInfo.position_x<<";"<<RobotInfo.position_y<<";"<<0<<";"
           <<0<<";"<<0<<";"<<0<<";"<<"#"<<ends;
        s=oss.str();
        for(j=0; j<s.length(); j++)
        {
            buf[j] = s[j];
        }
        buf[j]='\0';
        //strcpy(buf,"-1;0;1;1;2;7;15;1;2;3;4;1.1;1.2;0;0;0;0;");//0.5;-0.5;1.0;1.0;-2;0.5;1;7;8;9;10;11;-1;-2;-3;
        sendFlag=write(*socket_fd,buf,strlen(buf));
        if(sendFlag<=0)
            continue;
        delete   oss.str();
    }
    pthread_exit(NULL);
}
//接收线程所要执行的函数 接收消息
void * recv_msg(void *arg)
{
    //myInfo.WorkingState=RequireNewPath;
    //在接收端修改主要是把每个数据位都对应起来，改到每个读的数据正确配对即为结束
    char buf[1024]={0};
    int recFlag;
    int *socket_fd=(int *)arg;
    string myString(buf,1024);
    string wholeString;
    string pattern_0="#";
    string pattern_1="|";
    string pattern_2=" ";
    vector< vector <string> >RecData;
    vector<string>  subGroup;
    //bool PosIntialedInServer=false;
    while(1)
    {
        memset(buf,0,1024);
        myString.clear();
        wholeString.clear();
        subGroup.clear();

        recFlag=read(*socket_fd, buf, sizeof(buf));//阻塞，等待接收消息
        sleep(2);
        if(strncmp(buf, "exit", 4) == 0 || strcmp(buf, "") == 0)
        {
            //通知主线程。。。
            printf("GOODBYE\n");
            bzero(buf, 1024);
            //pthread_cancel(execute_thread);
            pthread_cancel(send_thread);
            pthread_cancel(execute_thread);
            break;//退出
        }
        else
        {
            if(recFlag<=0)
                continue;
            myString.assign(buf,1024);
            if(split(myString,pattern_0).size()>0)
                wholeString.assign(split(myString,pattern_0)[0]);//wholeString为用#分割的一串
            //for(int i =0;i<wholeString.size();i++)
            if(split(wholeString,pattern_1).size()>0)
                subGroup=split(wholeString,pattern_1);//subGroup为用|分割的组
            RecData.clear();
            RecData.resize(subGroup.size()-1);
            vector<pair<float,float> > tmpNodesPos;
            vector<pair<float,float> > tmpNodesVel;
            vector<int>tmpNodeNumber;
            vector<float>tmpw;

            tmpNodeNumber.clear();
            tmpNodesPos.clear();
            tmpNodesVel.clear();
            tmpw.clear();

            for(int i=0;i<RecData.size();i++)
            {
                RecData[i]=(split(subGroup[i],pattern_2));//RecData[i][j]为用空格分割的具体数据不过还没转化为数字，具体某一组某一位的string类数据
                for(int j=0;j<RecData[i].size();j++)
                {
                    //这部分当是最为严格的，关系的数据更新是否正确，必须保证读到的每个数据不多不少
                    if(i==0&&j==3)//if(PosIntialedInServer== false&&i==0&&j==2)
                    {
                        int sum;
                        sum=atoi(RecData[0][0].c_str())+1+atoi(RecData[0][1].c_str());
                        if(atoi(RecData[0][0].c_str())!=myInfo.RobotNumber&& atoi(RecData[0][3].c_str())!=sum)
                            break;
                        atoi(RecData[0][0].c_str());//RobotNumber
                        PosIntialedInServer=atoi(RecData[0][1].c_str());//是否收到机器人数据,收到数据以后把WorkingState跳转到RequestNewPath
                        //把initial的标志改为RequirePath
                        myInfo.WorkingState=RequireNewPath;
                        myInfo.currentNode=atoi(RecData[0][2].c_str());
                        myInfo.FinalGoalNode=14;//11到4,避免停留在通道中间，顺序是11 8 5 0 1 2 3 4 或者 11 12 13 14 9 6 3 4
                        cout<<"Succeed connecting to Server"<<endl;
                        sleep(3);
                    }
                    else if(i!=0&&j==5)
                    {
                        tmpNodeNumber.push_back(atoi(RecData[i][0].c_str()));//只有x,y坐标都有了才保存，这是节点的编号，机器人这里是看不出来的，只有发送当前位置的时候方便
                        tmpNodesPos.push_back(make_pair(atof(RecData[i][1].c_str()),atof((RecData[i][2]).c_str())));//节点坐标
                        tmpNodesVel.push_back(make_pair(atof(RecData[i][3].c_str()),atof((RecData[i][4]).c_str())));//到达节点的速度
                        tmpw.push_back(atof(RecData[i][5].c_str()));
                        if(i==RecData.size())
                        {
                            cout<<"Receive new data of nodes are";
                            for (int k=0;k<tmpNodeNumber.size();k++)
                            {
                                cout<<tmpNodeNumber.at(k)<<" ";
                            }
                            cout<<endl;
                        }

                    }
                    else
                    {
                        //cout<<"Data update fail!"<<endl;
                        continue;
                    }
                }
            }

            //这里要写机器人需要更新路径的判断标志
            if(myInfo.WorkingState==RequireNewPath&&!tmpNodeNumber.empty()&&PosIntialedInServer==true)
            {
                if(*tmpNodeNumber.begin()!=myInfo.currentNode)
                {
                    myInfo.WorkingState=RequireNewPath;
                }
                else
                {
                    myNode.NodesPos=tmpNodesPos;
                    myNode.NodesVel=tmpNodesVel;
                    myNode.NodeNumber=tmpNodeNumber;
                    myInfo.WorkingState=Pending;
                    cout<<"Path Updated"<<endl;
                }
            }
            //cout<<"Xpos"<<myNode.NodesPos[0].first<<""<<"Ypos"<<myNode.NodesPos[0].second<<""<<endl;
            //sleep(1);
        }
    }

    pthread_exit(NULL);
}

void * executePath(void *arg)
{
    myInfo.OnlineFlag = true;
    myInfo.WorkingState = RequestInitial;

    float distance, epsilon = 0.2;

    TempGoal goal;
    move_base_msgs::MoveBaseGoal Goal1,Goal2,Goal3,Goal4;

    while (1)
    {
        sleep(1);
        MoveBaseClient ac("move_base", true);

        ros::param::set("/Robot_pos", "Start_Point");

        // Wait 5 sec for move_base action server to come up

        while (!ac.waitForServer(ros::Duration(2.0))) {

            ROS_INFO("Waiting for the move_base action server to come up");
        }

        Goal1.target_pose.header.frame_id = "map";
        Goal1.target_pose.header.stamp = ros::Time::now();
        Goal2.target_pose.header.frame_id = "map";
        Goal2.target_pose.header.stamp = ros::Time::now();
        Goal3.target_pose.header.frame_id = "map";
        Goal3.target_pose.header.stamp = ros::Time::now();
        Goal4.target_pose.header.frame_id = "map";
        Goal4.target_pose.header.stamp = ros::Time::now();
        int i=0;

        if (myInfo.WorkingState == RequestInitial)
        {
            cout << "Initial request! Please start robot and send current position to server.";
            sleep(2);
        } else if (myInfo.WorkingState == RequireNewPath)//需要路径，锁定执行
        {
            cout << "Execute thread request New Path,robot awaiting at node "<<myInfo.currentNode << endl;
            sleep(5);
        }
        else if (myInfo.WorkingState == Pending)
        {
            if (myNode.NodeNumber.empty())
            {
                cout << "Inplace Awaiting!" << endl;
                sleep(1);
            }
            else
            {
                int n=myNode.NodeNumber.size();
                if(i==n)//如果无节点，则清空缓存，并接受新的节点
                {
                    if(PosIntialedInServer==false)
                        continue;
                    myInfo.WorkingState = RequireNewPath;
                    myNode.NodeNumber.clear();
                    continue;
                }

                //第一个目标
                cout << "Robot target to " << myNode.NodeNumber[i] << endl;
                goal.NodeNum = myNode.NodeNumber[i];
                goal.x = myNode.NodesPos[i].first;
                goal.y = myNode.NodesPos[i].second;
                goal.w = 3.0;
                Goal1.target_pose.pose.position.x = goal.x;
                Goal1.target_pose.pose.position.y = goal.y;
                Goal1.target_pose.pose.orientation.w = goal.w = 3.0;
                ROS_INFO("Sending Goal1");
                ac.sendGoal(Goal1);
                positionUpdate=false;//禁止发送当前的数据，直到到达
                ros::param::set("/Robot_pos", "Moving");
                distance=CalculateDistance(RobotInfo,goal);
                sleep(5);
                ac.cancelGoal();

                myInfo.currentNode=myNode.NodeNumber[i];
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Hooray, the base moved to the pick up Goal1");
                    ros::param::set("/Robot_pos", "position_1");
                }
                else
                {
                    ROS_INFO("The base failed to move to the Goal1");
                }
                cout << "Robot target to first node:" << myNode.NodeNumber[i] << endl;
                //第二个目标
                i++;
                if(i==n)//如果无节点，则清空缓存，并接受新的节点
                {
                    if(PosIntialedInServer==false)
                        continue;
                    myInfo.WorkingState = RequireNewPath;
                    myNode.NodeNumber.clear();
                    continue;
                }
                cout << "Robot target to " << myNode.NodeNumber[i] << endl;
                goal.NodeNum = myNode.NodeNumber[i];
                goal.x = myNode.NodesPos[i].first;
                goal.y = myNode.NodesPos[i].second;
                goal.w = 3.0;
                Goal2.target_pose.pose.position.x = goal.x;
                Goal2.target_pose.pose.position.y = goal.y;
                Goal2.target_pose.pose.orientation.w = goal.w = 3.0;
                ROS_INFO("Sending Goal2");
                ac.sendGoal(Goal2);
                ros::param::set("/Robot_pos", "Moving");
                distance=CalculateDistance(RobotInfo,goal);
                sleep(5);//给5秒时间执行这段路程
                ac.cancelGoal();

                myInfo.currentNode=myNode.NodeNumber[i];
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Hooray, the base moved to the pick up Goal2");
                    ros::param::set("/Robot_pos", "position_2");
                }
                else
                {
                    ROS_INFO("The base failed to move to the Goal2");
                }
                cout << "Robot target to second node:" << myNode.NodeNumber[i] << endl;
                //第三个目标
                i++;
                if(i==n)//如果无节点，则清空缓存，并接受新的节点
                {
                    if(PosIntialedInServer==false)
                        continue;
                    myInfo.WorkingState = RequireNewPath;
                    myNode.NodeNumber.clear();
                    continue;
                }
                cout << "Robot target to " << myNode.NodeNumber[i] << endl;
                goal.NodeNum = myNode.NodeNumber[i];
                goal.x = myNode.NodesPos[i].first;
                goal.y = myNode.NodesPos[i].second;
                goal.w = 3.0;
                Goal3.target_pose.pose.position.x = goal.x;
                Goal3.target_pose.pose.position.y = goal.y;
                Goal3.target_pose.pose.orientation.w = goal.w = 3.0;
                ROS_INFO("Sending Goal3");
                ac.sendGoal(Goal3);
                ros::param::set("/Robot_pos", "Moving");
                distance=CalculateDistance(RobotInfo,goal);
                sleep(5);
                ac.cancelGoal();

                myInfo.currentNode=myNode.NodeNumber[i];
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Hooray, the base moved to the pick up Goal3");
                    ros::param::set("/Robot_pos", "position_3");
                }
                else
                {
                    ROS_INFO("The base failed to move to the Goal3");
                }
                cout << "Robot target to third node:" << myNode.NodeNumber[i] << endl;
                //第四个目标
                i++;
                if(i==n)//如果无节点，则清空缓存，并接受新的节点
                {
                    if(PosIntialedInServer==false)
                        continue;
                    myInfo.WorkingState = RequireNewPath;
                    myNode.NodeNumber.clear();
                    continue;
                }
                cout << "Robot target to " << myNode.NodeNumber[i] << endl;
                goal.NodeNum = myNode.NodeNumber[i];
                goal.x = myNode.NodesPos[i].first;
                goal.y = myNode.NodesPos[i].second;
                goal.w = 3.0;
                Goal4.target_pose.pose.position.x = goal.x;
                Goal4.target_pose.pose.position.y = goal.y;
                Goal4.target_pose.pose.orientation.w = goal.w = 3.0;
                ROS_INFO("Sending Goal4");
                ac.sendGoal(Goal4);
                ros::param::set("/Robot_pos", "Moving");
                distance=CalculateDistance(RobotInfo,goal);
                sleep(5);
                ac.cancelGoal();

                myInfo.currentNode=myNode.NodeNumber[i];
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Hooray, the base moved to the pick up Goal4");
                    ros::param::set("/Robot_pos", "position_4");
                }
                else
                {
                    ROS_INFO("The base failed to move to the Goal4");
                }
                cout << "Robot target to fourth node:" << myNode.NodeNumber[i] << endl;
                //4个节点巡航完毕
                cout << "Robot arrive Local Path,Require New path,Awaiting at " <<myNode.NodeNumber[i]<< endl;

            }
        }
        if(PosIntialedInServer==false)
            myInfo.WorkingState=RequestInitial;
        else
            myInfo.WorkingState = RequireNewPath;
        positionUpdate=true;
        //if(!myNode.NodeNumber.empty())
        //  myInfo.currentNode=*myNode.NodeNumber.end();

    }
}

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::ostrstream oss;
    RobotInfo.position_x=msg->pose.pose.position.x;
    RobotInfo.position_y=msg->pose.pose.position.y;
    RobotInfo.twist_linear_x=msg->twist.twist.linear.x;
    RobotInfo.twist_linear_y=msg->twist.twist.linear.y;
    positionUpdate=true;
    sleep(2);
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
    /*oss << msg->pose.pose.position.x <<";"<< msg->pose.pose.position.y<< ";"<<msg->pose.pose.position.z<<";"<<msg->pose.pose.orientation.x<<";"<<msg->pose.pose.orientation.y<<";"<<msg->pose.pose.orientation.z<<";"<<msg->twist.twist.linear.x<<";"<<msg->twist.twist.linear.y<<";"<<msg->twist.twist.linear.z<<";"<<msg->twist.twist.angular.x<<";"<<msg->twist.twist.angular.y<<";"<<msg->twist.twist.angular.z<<";";

    std::cout << oss.str() << std::endl;
    str=oss.str();
*/
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

float CalculateDistance(struct robot_msg RobotPos,struct TempGoal NextNode)
{
    float distance;
    distance=sqrt(pow((RobotPos.position_x-NextNode.x),2)+pow((RobotPos.position_y-NextNode.y),2));
    return distance;
}




