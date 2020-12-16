//
// Created by zwb on 2020/5/28.
//

#include "EditDataFromClients.h"
EditDataFromClients::EditDataFromClients()
{
    buf[1024]={0};
}
EditDataFromClients::~EditDataFromClients(){}

void EditDataFromClients::GetData(char buf_[]) //给类中的变量buf赋值，用于字符串处理操作
{
    memset(buf,0,1024);
    strcpy(buf,buf_);
    validData=false;
    SubString();
    //DataValid();
    //ConvertSTof();

}
bool EditDataFromClients::DataValid() {
    bool flag=false;

    if(ClientData.size()<10)
        return false;
    else if(checkHeadFlag()== false)
    {
        return false;
    }
    else
        return true;
}
bool EditDataFromClients::SubString()//返回N串以;分割的字串，每串都是string类型,同时s也被赋值
{
    //std::size_t pos1,pos2;
    vector<string> ss;
    string myString;
    //string s,s_;
    string pattern=";",pattern0="#";
    myString.clear();
    myString.assign(buf,1024);
    /*s_=myString.substr(myString.find_first_of("@")+1,myString.size());
    pos1=s_.find_first_of("@")-2;
    pos2=myString.find_first_of("#")-1;
    if(pos1<100)
    {
        s=myString.substr(1,pos1);

    } else
    {
        cout<<"Data invalid"<<endl;
        subString.clear();
        return false;
    }
     */
    ss.clear();
    ss=split(myString,pattern0);
    subString.clear();
    if(ss.size()>=2)
    subString=split(ss.at(1),pattern);
    else{
        cout<<"Data invalid"<<endl;
        subString.clear();
        return false;
    }
    ClientData.clear();

    for(int i=0;i<subString.size()-1;i++)
    {
        ClientData.push_back(atof(subString[i].data()));
    }

    validData=true;

    cout<<"data valid"<<endl;

    return true;
}
vector<string> EditDataFromClients::split(string str, string pattern)
{
    std::string::size_type pos;
    std::vector<std::string> result;

    str += pattern;//扩展字符串以方便操作
    int size = str.size();

    for (int i = 0; i < size; i++) {
        pos = str.find(pattern, i);
        if (pos < size) {
            std::string s = str.substr(i, pos - i);
            result.push_back(s);
            i = pos + pattern.size() - 1;
        }
    }
    return result;
}

vector<float> EditDataFromClients::ConvertSTof()
{
    /*ClientData.clear();
    //subString.pop_back();
    for(int i=0;i<subString.size()-1;i++)//
    {
        ClientData.push_back(atof(subString[i].data()));
        if(i==10)
            break;

    }
    //sleep(1);
     */
};
bool EditDataFromClients::checkHeadFlag() {
    if(ClientData.empty())
        return false;
    else if(ClientData.at(0)==-1)
        return true;
    else
        return false;

}
int EditDataFromClients::PrintRobotNumber()
{
    if(ClientData.size()<2)
        std::cout<<"Data wrong,please enter Robot Number!"<<endl;
    else{
        return ClientData.at(1);
    }
}

bool EditDataFromClients::isRobotOnline()
{
    if(ClientData.size()<3)
        std::cout<<"Data wrong,please enter Robot Condition!"<<endl;
    else {
        return ClientData.at(2);
    }
}

int EditDataFromClients::PrintWorkingState()
{
    if(ClientData.size()<4)
        std::cout<<"Data wrong,please enter Robot State!"<<endl;
    else {
        return ClientData.at(3);
    }
}

int EditDataFromClients::PrintRobotCurrentNode()
{
    if(ClientData.size()<5)
        std::cout<<"Data wrong,please enter Robot Position!"<<endl;
    else {
        return ClientData.at(4);
    }
}
float EditDataFromClients::PrintRobotPosX()
{
    if(ClientData.size()<11)
        std::cout<<"Data wrong,please enter Robot Position X!"<<endl;
    else {
        return ClientData.at(10);
    }
}
float EditDataFromClients::PrintRobotPosY()
{
    if(ClientData.size()<12)
        std::cout<<"Data wrong,please enter Robot Position Y!"<<endl;
    else {
        return ClientData.at(11);
    }
}

int EditDataFromClients::PrintRobotFinalGoal()
{
    if(ClientData.size()<6)
        std::cout<<"Data wrong,please enter Robot Goal!"<<endl;
    else {
        return ClientData.at(5);
    }
}

vector<float>EditDataFromClients::PrintRobotVelocity()
{
    if(ClientData.size()<10)
        cout<<"Data wrong,please enter Robot Velocity and orientation!"<<endl;
    else
    {    vector<float>v;
        v.clear();
        float X_Velocity,Y_Velocity,Z_Velocity,Orientation;

        X_Velocity=ClientData.at(6);
        Y_Velocity=ClientData.at(7);
        Z_Velocity=ClientData.at(8);
        Orientation=ClientData.at(9);
        v.push_back(X_Velocity);
        v.push_back(Y_Velocity);
        v.push_back(Z_Velocity);
        v.push_back(Orientation);
        return v;
    }

}

vector<int>EditDataFromClients::PrintLocalPath()
{
    //这部分要考虑怎么处理局部路径，包括占用的和释放的,一种办法是通过正负号来判断，节点从1开始,正号代表被该机器人占用,负号表示节点已释放
    vector<int>NodesOfLocal;
    NodesOfLocal.clear();
    employedLocalPath.clear();
    releasedLocalPath.clear();
    for(int i=10;i<ClientData.size();i++)
    {
        NodesOfLocal.push_back(ClientData.at(i));
        if(ClientData.at(i)>0)
            employedLocalPath.push_back(ClientData.at(i));
        else
            releasedLocalPath.push_back(ClientData.at(i));
    }
    return NodesOfLocal;

}

vector<int>EditDataFromClients::PrintEmployedPath()
{

    return employedLocalPath;
}

vector<int>EditDataFromClients::PrintReleasedPath()
{
    return releasedLocalPath;
}

