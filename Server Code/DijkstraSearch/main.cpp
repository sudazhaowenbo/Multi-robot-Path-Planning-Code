
#include "DijkstraSearch.h"
#include "EditDataFromClients.h"
#include "Server.h"
#include "NodeAllocation.h"
#include <unistd.h>
#include <pthread.h>
#include <set>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>
#include <algorithm>

using namespace std;
typedef multimap<int, int> multim;//第一个int是节点号，第二个int是机器人号
typedef pair<pair<int, int>, int > doublePair;
#define AmountOfNodes 3

void * recv_msg(void *arg);//接收消息函数声明
void * recv_msg_1(void *arg);
void * send_msg(void *arg);//发送消息函数声明
void * send_msg_1(void *arg);//发送消息函数声明
void * search_path(void *arg);
void * displayImg(void *arg);

static pthread_t send_thread,SearchThread,send_thread2,display_thread;
char received_buf[1024]={0};
//char received_buf[2][1024]={0};
char send_buf[3][1024]={0};
char received_buf1[1024]={0};
bool mapGenerated;
bool receivedFlag[3]={0};
Mat  DisplayTheMap;
pthread_mutex_t mutex_send[2],mutex_rec[2];

enum {Initial,RequestPath,Pending,ReachFinalGoal};

//现在从机器人处接收来的数据已经能被路径搜索的线程接受并处理了，但要小心处理，包括全局变量的初始化，一定要按照到EditData那个类的形式修改，另外长度也要有所保证，否则数组容易出问题;
//下一就是要让机器人动起来，改变当前的路径点，然后动态去规划他，这一步要把地图处理成设定的N个点，然后每个点要记录位置信息，
int main(int argc, char *argv[])
{
    memset(received_buf1,1,1024);
    memset(received_buf,1,1024);
    //memset(send_buf,0,1024);
    //memset(send_buf2,0,1024);

    //strcpy(received_buf,"-1;2;0;0;3;10;1.1;1.2;1.5;2.1;3;5;7;10;");
    //strcpy(received_buf1,"-1;2;0;0;3;10;1.1;1.2;1.5;2.1;3;5;7;10;");
    mapGenerated=false;
    int ret,ret2;
    //img2Nodes();
    Server server1(6666),server2(7777);//server1 6666
    //Mat img = imread("/home/zwb/CLionProjects/NodeGenerator/map.pgm");
    //Robot ro(0,img);
    //ro.generateGraph();
    server1.InitialServer();
    server2.InitialServer();
    pthread_mutex_init(&mutex_send[0], NULL);
    pthread_mutex_init(&mutex_rec[0], NULL);
    pthread_mutex_init(&mutex_send[1], NULL);
    pthread_mutex_init(&mutex_rec[1], NULL);
    pthread_create(&SearchThread,NULL,&search_path,NULL);
    pthread_create(&display_thread,NULL,&displayImg,NULL);//创建显示线程
    //开启接收线程
    pthread_t recv_thread,recv_thread2;//存放线程id       recv_msg：线程执行的函数，将通信socket：new_socket_fd传递进去
    ret = pthread_create(&recv_thread, NULL, recv_msg, (void *) &server1.new_socket_fd);
    ret2= pthread_create(&recv_thread2,NULL,recv_msg_1,(void *) &server2.new_socket_fd);
    //开启发送线程
    ret = pthread_create(&send_thread, NULL, send_msg, (void *) &server1.new_socket_fd);
    ret2= pthread_create(&send_thread2,NULL,send_msg_1,(void *) &server2.new_socket_fd);//改这句

    if(server1.socket_fd==-1||server1.port<1025||server1.port>65535||server1.ret==-1||server1.new_socket_fd==-1||ret!=0)
    {
        cout<<"Failed to creat server1,program exit"<<endl;
        return 0;
    }
    else {
        pthread_join(send_thread, NULL);
    }

    if(server2.socket_fd==-1||server2.port<1025||server2.port>65535||server2.ret==-1||server2.new_socket_fd==-1||ret!=0)
    {
        cout<<"Failed to creat server1,program exit"<<endl;
        return 0;
    }
    else
        pthread_join(send_thread2,NULL);

    pthread_join(SearchThread,NULL);
    pthread_join(display_thread,NULL);//显示线程
    server1.ShutDownServer();
    server2.ShutDownServer();
    pthread_mutex_destroy(&mutex_rec[0]);
    pthread_mutex_destroy(&mutex_send[0]);
    pthread_mutex_destroy(&mutex_rec[1]);
    pthread_mutex_destroy(&mutex_send[1]);
    return 0;
}

void * search_path(void *arg)
{
    //坐标点转化为节点，有以下几种选择1.在接收函数中转化，然后把节点传给搜索函数（不太好）那么需要地图，然后坐标转为像素（需要自己算），像素再转为节点，在像素转节点这一步怕是无法实现，
    // 因为，根本不知道某个节点是第几个节点。
    //2.在搜索函数里面转化，分为两种，一种是把传进来的数据放到EditDataFromClients,本质上和第一种没什么区别，因为也都需要自己重新计算;
    //3.另一种把数据传到robot类里面，这个类里面已经有了,但是在计算邻近节点的时候，如果每次都算的话计算量太大，因此，考虑只在第一次的时候计算邻近节点，之后
    //服务器与机器人通信只通过节点通信，机器人还是发位置坐标给服务器，服务器只管发，不管你执行，机器人如果错了，把错的信息发给服务器即可
    //
    //Mat img = imread("/home/zwb/CLionProjects/NodeGenerator/map.pgm");
    Mat img = imread("/home/zwb/CLionProjects/NodeGenerator/map.pgm");
    EditDataFromClients Editor[3];
    Robot robot[3]={Robot(65535,img),Robot(65535,img),Robot(65535,img)};//初始化位置为65535,如果不更新初始位置，不可进行路径搜索
    NodeAllocation nodeAlloc;
    DisplayTheMap=robot[0].generateGraph().clone();
    /*for(int i=0;i<3;i++)
    {
        robot[i].generateGraph();
    }*/
    mapGenerated=true;
    int start = 0, goal = 13, currentPos,tmpPos[2],RobotNumber;
    tmpPos[0]=-1;
    tmpPos[1]=-1;
    char buf[1024]={0}; memset(buf,0,1024);
    set<int> RobotSet;

    for(int i=0;i<3;i++)
    {
        robot[i].SetGoal(goal-i);
    }

    while (1)
    {
        for(int m=0;m<=1;m++)
        {
            //先检验数据正确与否，若不正确则不执行下面的语句
        }
        //pthread_mutex_lock(&mutex_rec[0]);
        Editor[0].GetData(received_buf);
        //pthread_mutex_unlock(&mutex_rec[0]);
        if (Editor[0].validData==false)
        {
            cout<<"Data from Robot 0 wrong,please enter again"<<endl;
            receivedFlag[0]=false;
            continue;
        }
        else
        {
            receivedFlag[0]=true;

            //节点搜索算法
            if (Editor[0].isRobotOnline() )//Editor代表robot0,&& Editor[0].PrintRobotNumber() == 0
            {
                robot[0].isRobotOnline=true;
                //根据机器人发送的情况规划，如果机器人自己知道在第几个节点，则进入RequestPath，如果不知道则进入Initial
                if (Editor[0].PrintWorkingState() == Initial)//在线状态0
                {
                    tmpPos[0]=65535;
                    tmpPos[0] = robot[0].PrintNearestNode(Editor[0].PrintRobotPosX(), Editor[0].PrintRobotPosY());
                    robot[0].SetCurrentPos(tmpPos[0]);
                }
                else if (Editor[0].PrintWorkingState() == RequestPath)//在线状态1
                {
                    robot[0].LocalPaths.clear();
                    robot[0].SetGoal(Editor[0].PrintRobotFinalGoal());
                    tmpPos[0] = Editor[0].PrintRobotCurrentNode();//currentPos由Robot类里面算出最近的节点，再赋值
                    if(tmpPos[0]>robot[0].PrintMaxNode())      //加入一个位置重计算功能，当机器人的位置越出地图最大节点数目时，认为某些地方出错，则机器人的位置由服务器重新计算
                        tmpPos[0]=robot[0].PrintNearestNode(Editor[0].PrintRobotPosX(),Editor[0].PrintRobotPosY());
                    //currentPos =robot0.PrintNearestNode(Editor.x,Editor.y);
                    robot[0].SetCurrentPos(tmpPos[0]);
                    robot[0].OtherS.clear();
                    robot[0].clearAllEdges();//清空搜索图的占用信息
                    //先清楚其他机器人的占用状态（因为可能已经更新了），然后压入其他机器人新的状态，包括机器人占用的边（用于在搜索时禁用）和
                    //第一轮搜索时，机器人并不知道其他机器人的行进方向，但是当有机器人占用了某个方向以后便需要将这个信息加入到搜索图中，而不再把机器人的占用点作
                    //临时节点,因此只能把其他机器人占用的节点作为临时障碍物考虑，
                    //第二轮搜索时，
                    //或者认为，每个被占用的节点都是有方向的，如果占用不移动，则占用所有方向，如果占用
                    //这些认为是离线的机器人可以不用
                    //robot0.UpdateOccupiedNodesOfOtherRobots(robot1.LocalS);
                    //robot0.UpdateOccupiedNodesOfOtherRobots(robot2.LocalS);
                    //robot0.UpdateOccupiedNodesOfOtherRobots(robot3.LocalS);
                    //这些是其他机器人占用的逆边
                    for (int m = 0; m < 3; m++) {
                        if (robot[m].LocalPaths.empty() || m == 0)
                            continue;
                        else {
                            robot[0].UpdateInverseEdges(robot[m].LocalPaths);
                        }
                    }
                    //搜索路径
                    robot[0].DijkstraSP(tmpPos[0]);
                    if(robot[0].LocalPaths.empty())
                        robot[0].LocalPaths.push_back(tmpPos[0]);
                    //robot0.PrintLocalPath();
                    //这里加一句转换的代码，把每个LocalPaths里面的元素转为x,y坐标
                }
                else if (Editor[0].PrintWorkingState() == Pending)//在线状态2
                {
                    //机器人正在行进，跳过
                    cout << "Robot is pending" << endl;
                }
                else if (Editor[0].PrintWorkingState() == ReachFinalGoal)//在线状态3
                {
                    //机器人到达全局路径终点
                    cout << "Robot has arrived the final goal" << endl;
                }
                else
                {
                    cout << "Something gets wrong" << endl;
                }
            }
            else
            {
                robot[0].isRobotOnline=0;
                //等等写离线情况
            }
        }

        //
        //pthread_mutex_lock(&mutex_rec[1]);
        Editor[1].GetData(received_buf1);//处理数据的时候不要接收
        //pthread_mutex_unlock(&mutex_rec[1]);
        if(Editor[1].validData==false)
        {
            cout<<"Data from Robot 1 wrong,please enter again"<<endl;
            receivedFlag[1]=false;
            continue;
        }
        else
        {
            receivedFlag[1]=true;
            if (Editor[1].isRobotOnline() ) //&& Editor[1].PrintRobotNumber() == 1
            {
                robot[1].isRobotOnline=true;
                if (Editor[1].PrintWorkingState() == Initial)//在线状态0
                {
                    //先让当前位置无效化，再计算，最后在发送的时候判断位置是否有效，有效才发
                    tmpPos[1]=65535;
                    tmpPos[1] = robot[1].PrintNearestNode(Editor[1].PrintRobotPosX(), Editor[1].PrintRobotPosY());
                    robot[1].SetCurrentPos(tmpPos[1]);
                }
                else if (Editor[1].PrintWorkingState() == RequestPath)//在线状态1
                {
                    //pthread_mutex_lock(&mutex_rec);
                    robot[1].LocalPaths.clear();
                    robot[1].SetGoal(Editor[1].PrintRobotFinalGoal());
                    tmpPos[1] = Editor[1].PrintRobotCurrentNode();
                    if(tmpPos[1]>robot[1].PrintMaxNode())      //加入一个位置重计算功能，当机器人的位置越出地图最大节点数目时，认为某些地方出错，则机器人的位置由服务器重新计算
                        tmpPos[1]=robot[1].PrintNearestNode(Editor[1].PrintRobotPosX(),Editor[1].PrintRobotPosY());
                    robot[1].SetCurrentPos(tmpPos[1]);
                    robot[1].OtherS.clear();
                    robot[1].clearAllEdges();
                    /*
                    robot[1].UpdateOccupiedNodesOfOtherRobots(robot[0].LocalS);
                    robot[1].UpdateOccupiedNodesOfOtherRobots(robot[2].LocalS);
                    robot[1].UpdateOccupiedNodesOfOtherRobots(robot[3].LocalS);
                    robot[1].DijkstraSP(currentPos);
                    robot[1].PrintLocalPath();
                     */
                    //这些是其他机器人占用的逆边
                    for (int m = 0; m < 3; m++)
                    {
                        if (robot[m].LocalPaths.empty() || m == 1)
                            continue;
                        else
                        {
                            robot[1].UpdateInverseEdges(robot[m].LocalPaths);
                        }
                    }

                    //robot[1].UpdateInverseEdges(robot[0].LocalPaths);
                    //robot[1].UpdateInverseEdges(robot[2].LocalPaths);
                    //robot[1].UpdateInverseEdges(robot[3].LocalPaths);
                    //搜索路径
                    robot[1].DijkstraSP(tmpPos[1]);
                    if(robot[1].LocalPaths.empty())
                        robot[1].LocalPaths.push_back(tmpPos[1]);

                } else if (Editor[1].PrintWorkingState() == Pending)//在线状态2
                {
                    //机器人正在行进，跳过
                    cout << "Robot is pending" << endl;
                } else if (Editor[1].PrintWorkingState() == ReachFinalGoal)//在线状态3
                {
                    //机器人到达全局路径终点
                    cout << "Robot has arrived the final goal" << endl;
                } else {
                    cout << "Something gets wrong" << endl;
                }
            } else {
                robot[1].isRobotOnline=0;
                //等等写离线情况
            }
            sleep(2);
        }


        //节点分配算法
        //这里还有问题，就是算出来的节点与机器人号不匹配，根源在于没把机器人号这个信息加进去
        //把robot的编号存入到nodeAlloc里面，处理好后再输出带编号的节点
        //不管有没有收到机器人的数据，都要把存储的节点压入进行重分配
        vector<int> Path[2];
        for(auto i:robot[0].LocalPaths)
            Path[0].push_back(i);
        for(auto i:robot[1].LocalPaths)
            Path[1].push_back(i);
        nodeAlloc.OriginalNodes.clear();
        for (int i = 0; i < 2; i++) {
            if(robot[i].isRobotOnline==true)//如果机器人连接，则把接下去一段路径压入，如果离线，则把当前位置压入
            {
                if (Editor[i].PrintWorkingState()==0)
                {
                    vector<int> cur;
                    cur.push_back(robot[i].PrintCurrentPos());
                    nodeAlloc.OriginalNodes.push_back(cur);
                }
                else if(Editor[i].PrintWorkingState()==1||Editor[i].PrintWorkingState()==2)
                {
                    nodeAlloc.OriginalNodes.push_back(robot[i].LocalPaths);
                }
            }
            else
            {
                vector<int> cur;
                cur.push_back(robot[i].PrintCurrentPos());
                nodeAlloc.OriginalNodes.push_back(cur);
            }

        }
        sleep(1);
        //到这里为止原则上不应该再出现诸如Edittor,robot等名称，会引起混乱，如果后面需要使用，需要给其他变量重新赋值

        if(nodeAlloc.OriginalNodes.size()>=2)//两台机器人的位置都连上了才继续执行后面的内容，否则直接返回
            nodeAlloc.alloc();//若无节点则没必要执行发送命令了,原则上宁可少发几次也要每次发的都对
        else
            continue;
        //每个机器人搜索到的节点都压入mp里面去筛选，包括正在执行的和准备发送的

        //节点发送算法
        //把机器人类的每个对象按数组的形式计入，然后N个机器人循环更新数据发送（要根据机器人的当前状况，也就是收到的Editor的标志位该机器人是否需要新的节点序列
        //若是不需要直接continue,需要则发送

        for (int j = 0; j <nodeAlloc.AdjustedNodes.size() ; j++)//nodeAlloc.AdjustedNodes.size()
        {
            string SendString;
            ostringstream oss;
            oss.str("");
            SendString.clear();

            if(Editor[j].subString.size()<=5)//再次确保数据是有效的
            {
                continue;
            }
            else if(nodeAlloc.AdjustedNodes[j].size()==0)//确保经过重分配后节点机器人仍有节点
            {
                continue;
            }
            else if(Editor[j].PrintWorkingState()==Initial)//状态0
            {
                //如果当前位置计算的不对，不能发送
                if(tmpPos[j]>=robot[j].PrintMaxNode()||tmpPos[j]<0)
                    continue;
                int sum=Editor[j].PrintRobotNumber()+1+tmpPos[j];
                oss << SendString << Editor[j].PrintRobotNumber() << " " << true << " "<<tmpPos[j] << " "<<sum<<"|"<<"#";
                SendString = oss.str();
                memset(send_buf[j],0,1024);
                SendString.copy(send_buf[j], SendString.length(), 0);
                send_buf[j][SendString.length()] = '\0';
                //输出初始化阶段发送的信息
                cout<<"Robot"<<Editor[j].PrintRobotNumber()<<"got initialed in server and its node is "
                    <<tmpPos[j]<<" with pos X and pos Y at"<<Editor[j].PrintRobotPosX()<<"and"<<
                    Editor[j].PrintRobotPosY()<<endl;
            }
            else if(Editor[j].PrintWorkingState() == RequestPath)//状态1
            {
                oss << SendString << Editor[j].PrintRobotNumber() << " " << true << " " << "|";

                for (int i = 0; i < nodeAlloc.AdjustedNodes[j].size(); i++)//
                {
                    float x_map, y_map;
                    float vx = 10.5, vy = 10.7, w = 10.9;
                    int node=nodeAlloc.AdjustedNodes[j].at(i);
                    x_map = robot[j].Num2Pos.find(node)->second.first;//改了这两句主要为了找到对应的坐标，看会不会出错
                    y_map = robot[j].Num2Pos.find(node)->second.second;
                    oss << SendString << nodeAlloc.AdjustedNodes[j].at(i) << " " << x_map << " " << y_map << " "
                        << vx << " " << vy << " " << w << " " << "|";
                }
                //pthread_mutex_unlock(&mutex_rec);
                oss << SendString << "#";
                SendString = oss.str();
                memset(send_buf[j],0,1024);
                SendString.copy(send_buf[j], SendString.length(), 0);
                send_buf[j][SendString.length()] = '\0';
                cout << "robot" << oss.str() << endl;
            }
            else
            {
                continue;
            }
        }
    }
}

void * recv_msg(void *arg)
{
    int recFlag;
    int *socket_fd = (int *)arg;//通信的socket
    struct timeval tmOut;
    tmOut.tv_sec=0;
    tmOut.tv_usec=0;
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(*socket_fd,&fds);
    int nRet;
    char tmp[2];
    memset(tmp,0,sizeof(tmp));
    char testbuf[1024]={0};
    while(1)
    {
        char buf[1024] = {0};
        sleep(1);

        /*
        nRet=select(FD_SETSIZE,&fds,NULL,NULL,&tmOut);
        if(nRet==0)//表示没数据可以读
        {
            sleep(1);
            continue;
        }
        */
        sleep(1);
        recFlag=read(*socket_fd, buf, sizeof(buf));//阻塞，等待接收消息
        sleep(1);
        if(recFlag<=0)
            continue;
        pthread_mutex_lock(&mutex_rec[0]);
        strcpy(received_buf,buf);
        pthread_mutex_unlock(&mutex_rec[0]);
        printf("receive msg:%s\n", buf);

        if(strncmp(buf, "exit", 4) == 0 )
        {
            //通知主线程。。。
            printf("GOODBYE\n");
            bzero(buf, 1024);
            pthread_cancel(SearchThread);
            pthread_cancel(send_thread);
            pthread_cancel(display_thread);//退出显示线程
            break;//退出
        }

    }
    return NULL;
}
void * recv_msg_1(void *arg)
{
    int recFlag;
    int *socket_fd = (int *)arg;//通信的socket
    struct timeval tmOut;
    tmOut.tv_sec=0;
    tmOut.tv_usec=0;
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(*socket_fd,&fds);
    int nRet;
    char tmp[2];
    memset(tmp,0,sizeof(tmp));
    while(1)
    {
        char buf[1024] = {0};
        sleep(1);
        /*
        nRet=select(FD_SETSIZE,&fds,NULL,NULL,&tmOut);
        if(nRet==0)//表示没数据可以读
            continue;
        */

        //pthread_mutex_lock(&mutex_rec);
        sleep(1);
        recFlag=read(*socket_fd, buf, sizeof(buf));//阻塞，等待接收消息
        sleep(1);
        if(recFlag<=0)
            continue;
        pthread_mutex_lock(&mutex_rec[1]);
        strcpy(received_buf1,buf);
        pthread_mutex_unlock(&mutex_rec[1]);
        printf("receive msg:%s\n", buf);
        if(strncmp(buf, "exit", 4) == 0 )
        {
            //通知主线程。。。
            printf("GOODBYE\n");
            bzero(buf, 1024);
            pthread_cancel(send_thread);
            pthread_cancel(SearchThread);
            pthread_cancel(display_thread);//退出显示线程
            pthread_cancel(send_thread2);//退出发送线程
            break;//退出
        }

    }
    return NULL;
}
void * send_msg(void *arg)
{
    char buf[1024] = {0};
    memset(send_buf[0],0,1024);
    int writeFlag;
    //strcpy(buf,"0.5;-0.5;1.0;1.0;-2;0.5;1;7;8;9;10;11;-1;-2;-3;");

    int *socket_fd=(int *)arg;
    while(1)
    {
        //strcpy(send_buf[0],"1 1|5 0.1 0.1 1.1 1.1 3|11 0.2 0.2 2.2 2.2 3|#");
        //写入节点的处理语句
        strcpy(buf,send_buf[0]);

        sleep(1);
        pthread_mutex_lock(&mutex_send[0]);
        writeFlag=write(*socket_fd,buf,strlen(buf));
        pthread_mutex_unlock(&mutex_send[0]);
        sleep(1);
        int i=0;
        cout<<"send msg"<<endl;
        if(writeFlag<=0)
            continue;
        cout<<"0号机器人发送";
        while(i!=1023)
        {
            cout<<buf[i];
            i=i+1;
            //cout<<str.length()<<endl;
        }
        i=0;

        sleep(2);
        /*
        if(strcmp(buf, "exit") == 0 || strcmp(buf, "") == 0)
        {
            break;//退出
        }
        */
    }

    pthread_exit(NULL);
}
void * send_msg_1(void *arg)
{
    pthread_mutex_t mutex_send1;
    char buf[1024] = {0};
    memset(send_buf[1],0,1024);
    //strcpy(send_buf2,"-1;2;1;1;0;13;1.1;1.2;1.5;2.1;3;5;7;10;");
    int writeFlag;
    //strcpy(send_buf2,"3;|;1;0.1;0.1;1.1;1.1;3;| 2;0.2;0.2;2.2;2.2;3;| #");
    //strcpy(buf,"0.5;-0.5;1.0;1.0;-2;0.5;1;7;8;9;10;11;-1;-2;-3;");
    int *socket_fd=(int *)arg;

    while(1)
    {
        //先试着把要发的信息自己算出来看能不能被执行
        //strcpy(send_buf[1],"0 1|1 0.1 0.1 1.1 1.1 3|2 0.2 0.2 2.2 2.2 3|#");
        //写入节点的处理语句
        strcpy(buf,send_buf[1]);

        sleep(1);
        pthread_mutex_lock(&mutex_send[1]);
        writeFlag=write(*socket_fd,buf,strlen(buf));
        pthread_mutex_unlock(&mutex_send[1]);
        if(writeFlag<=0)
            continue;
        sleep(1);
        cout<<"1号机器人发送:";
        int i=0;
        while(i!=1023)
        {
            cout<<buf[i];
            i=i+1;
            //cout<<str.length()<<endl;
        }
        i=0;

        sleep(2);
        /*
        if(strcmp(buf, "exit") == 0 || strcmp(buf, "") == 0)
        {
            break;//退出
        }
        */
    }

    pthread_exit(NULL);
}
void * displayImg(void *arg)
{
    while(1)
    {
        if(mapGenerated&&!DisplayTheMap.empty())
        {
            namedWindow("节点标记图", WINDOW_NORMAL);
            imshow("节点标记图", DisplayTheMap);
            waitKey(0);
        }
        else
        {
            sleep(2);
        }
    }


}
