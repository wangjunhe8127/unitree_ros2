#include "unitree/common/dds/dds_easy_model.hpp"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <unitree/ros2_idl/QtCommand_.hpp>
#include <unitree/ros2_idl/String_.hpp>
#include <unitree/ros2_idl/Odometry_.hpp>
#include <unitree/ros2_idl/QtNode_.hpp>
#include <unitree/ros2_idl/QtEdge_.hpp>

#include <stdio.h>
#include <termio.h>
#include <cmath>
#include <ctime>

#define COMMANDTOPIC "/qt_command"
#define NOTICETOPIC "/qt_notice"
#define ODOMTOPIC "/lio_sam_ros2/mapping/re_location_odometry"
#define ADDNODETOPIC "/qt_add_node"
#define ADDEDGETOPIC "/qt_add_edge"

using namespace unitree::robot;
using namespace std;

class slamDemo
{
    private:
    int index = 0;
    const nav_msgs::msg::dds_::Odometry_* currentOdom;
    u_int16_t node_name = 0;

    //pub
    ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtCommand_> pubQtCommand = ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtCommand_>(new ChannelPublisher<unitree_interfaces::msg::dds_::QtCommand_>(COMMANDTOPIC));
    ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtNode_> pubQtNode = ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtNode_>(new ChannelPublisher<unitree_interfaces::msg::dds_::QtNode_>(ADDNODETOPIC));
    ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtEdge_> pubQtEdge = ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtEdge_>(new ChannelPublisher<unitree_interfaces::msg::dds_::QtEdge_>(ADDEDGETOPIC));
    //sub
    ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subQtNotice = ChannelSubscriberPtr<std_msgs::msg::dds_::String_>(new ChannelSubscriber<std_msgs::msg::dds_::String_>(NOTICETOPIC));
    ChannelSubscriberPtr<nav_msgs::msg::dds_::Odometry_> subOdommetry = ChannelSubscriberPtr<nav_msgs::msg::dds_::Odometry_>(new ChannelSubscriber<nav_msgs::msg::dds_::Odometry_>(ODOMTOPIC));

    public:
    slamDemo();

    void qtNoticeHandler(const void* message);
    void odometryHandler(const void* message);

    void keyExecute();
    unsigned char keyDetection();

    void startMapping();
    void endMapping();
    void startRelocation();
    void initPose();
    void startNavigation();
    void defaultNavigation();
    void addNodeAndEdge();
    void addEdge(u_int16_t edge_name, u_int16_t start_node, u_int16_t end_node);
    void closeAllNode();
    void deleteAllNode();
    void deleteAllEdge();
    void pauseNavigation();
    void recoverNavigation();
};

slamDemo::slamDemo(){
    ChannelFactory::Instance()->Init(0,"eth0");//‘eth0’：网段为123的网卡名称

    //pub
    pubQtCommand->InitChannel();
    pubQtNode->InitChannel();
    pubQtEdge->InitChannel();

    //sub
    subQtNotice->InitChannel(std::bind(&slamDemo::qtNoticeHandler,this,std::placeholders::_1), 10);
    subOdommetry->InitChannel(std::bind(&slamDemo::odometryHandler,this,std::placeholders::_1), 1);

    cout<<"***********************  Unitree SLAM Demo ***********************\n";
    cout<<"------------------         q    w    e         -------------------\n";
    cout<<"------------------         a    s    d         -------------------\n";
    cout<<"------------------         z    x    c         -------------------\n";
    cout<<"------------------------------------------------------------------\n";
    cout<<"------------------ q: Close ROS node           -------------------\n"; 
    cout<<"------------------ w: Start mapping            -------------------\n";
    cout<<"------------------ e: End mapping              -------------------\n";
    cout<<"------------------ a: Start navigation         -------------------\n";
    cout<<"------------------ s: Pause navigation         -------------------\n";
    cout<<"------------------ d: Recover navigation       -------------------\n";
    cout<<"------------------ z: Relocation and init pose -------------------\n";
    cout<<"------------------ x: Add node and edge        -------------------\n";
    cout<<"------------------ c: Delete all node and edge -------------------\n";
    cout<<"--------------- Press Ctrl + C to exit the program ---------------\n";
    cout<<"------------------------------------------------------------------"<<endl;
}


void slamDemo::odometryHandler(const void* message){
    currentOdom = (const nav_msgs::msg::dds_::Odometry_*)message;
}

void slamDemo::qtNoticeHandler(const void* message){
    int index_,begin_,end_,feedback_,arrive_;
    const std_msgs::msg::dds_::String_* seq = (const std_msgs::msg::dds_::String_*)message; 
    string str_ ,notice_;

    begin_ = seq->data().find("index:",0);//指令识别码
	end_ = seq->data().find(";",begin_);
	str_ = seq->data().substr(begin_ + 6, end_ - begin_ - 6);
	index_ = atoi(str_.c_str());

    begin_ = seq->data().find("notice:",0);//提示消息
    end_ = seq->data().find(";",begin_);
    notice_ = seq->data().substr(begin_ + 7, end_ - begin_ - 7);

    if(index_ <= 10000){//命令执行反馈
        begin_ = seq->data().find("feedback:",0);
	    end_ = seq->data().find(";",begin_);
        str_ = seq->data().substr(begin_ + 9, end_ - begin_ - 9);
	    feedback_ = atoi(str_.c_str());
        if(feedback_==0 || feedback_==-1)cout<<"\033[1;31m"<<"Command execution failed with index = "<<index_<<"."<<"\033[0m";
    }
    else if(index_ == 10001){//导航反馈
        begin_ = seq->data().find("arrive:",0);
	    end_ = seq->data().find(";",begin_);
        str_ = seq->data().substr(begin_ + 7, end_ - begin_ - 7);
	    arrive_ = atoi(str_.c_str());
        cout<<" I arrived node "<<arrive_<<". ";
    }
    cout<<notice_<<endl;
}

void slamDemo::startMapping(){
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.command_() = 3;  //3为开启建图命令
    send_msg.attribute_() = 2; //该值为1时开启xt16雷达节点，为2时开启mid360雷达节点，请根据实际使用的雷达型号确认数值
    send_msg.seq_().data() = "index:"+to_string(index)+";";
    pubQtCommand->Write(send_msg);
}

void slamDemo::endMapping(){
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.command_() = 4;  //4为结束建图命令
    send_msg.attribute_() = 0;
    send_msg.seq_().data() = "index:"+to_string(index)+";";
    send_msg.floor_index_().push_back(0);//楼层编号信息，请给固定值0
    send_msg.pcdmap_index_().push_back(0);//地图编号信息，请给固定值0
    pubQtCommand->Write(send_msg);
}

void slamDemo::startRelocation(){
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:"+to_string(index)+";";
    send_msg.command_() = 6;  //6为开启重定位
    send_msg.attribute_() = 1; //雷达型号1为xt16雷达，2为mid360雷达，请根据实际使用的雷达型号进行赋值
    pubQtCommand->Write(send_msg);
}

void slamDemo::initPose(){
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:"+to_string(index)+";";
    send_msg.command_() = 7;  //7为定位初始化指令
    send_msg.quaternion_x_() = 0; //初始旋转姿态四元数
    send_msg.quaternion_y_() = 0;
    send_msg.quaternion_z_() = 0;
    send_msg.quaternion_w_() = 1;
    send_msg.translation_x_() = 0;//初始位置XYZ坐标
    send_msg.translation_y_() = 0;
    send_msg.translation_z_() = 0;
    pubQtCommand->Write(send_msg);
}

void slamDemo::startNavigation(){
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:"+to_string(index)+";";
    send_msg.command_() = 8;  //8为开启导航节点命令
    pubQtCommand->Write(send_msg);
}

void slamDemo::defaultNavigation(){
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:"+to_string(index)+";";
    send_msg.command_() = 10;  //10 循环默认点导航命令
    pubQtCommand->Write(send_msg);
}

void slamDemo::closeAllNode(){
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:"+to_string(index)+";";
    send_msg.command_() = 99;  //99关闭所有节点命令
    pubQtCommand->Write(send_msg);
}

void slamDemo::addNodeAndEdge(){
    index++;
    unitree_interfaces::msg::dds_::QtNode_ send_msg;
    float position_x,position_y,position_z,yaw,siny_cosp,cosy_cosp;
    time_t timeNow = time(0);
    if(timeNow - currentOdom->header().stamp().sec() > 1){
       cout<<"\033[1;31m"<<"Odometry timestamp dose not match."<<"\033[0m"<<endl;
       return ;

    }
    siny_cosp = 2 * (currentOdom->pose().pose().orientation().w() * currentOdom->pose().pose().orientation().z() + currentOdom->pose().pose().orientation().x() * currentOdom->pose().pose().orientation().y()); 
    cosy_cosp = 1 - 2 * (currentOdom->pose().pose().orientation().y() * currentOdom->pose().pose().orientation().y() + currentOdom->pose().pose().orientation().z() * currentOdom->pose().pose().orientation().z());

    position_x = currentOdom->pose().pose().position().x();
    position_y = currentOdom->pose().pose().position().y();
    position_z = currentOdom->pose().pose().position().z();
    yaw=std::atan2(siny_cosp, cosy_cosp);
 
    node_name++;
    send_msg.seq_().data() = "index:"+to_string(index)+";";
    send_msg.node_().node_name_().push_back(node_name);
    send_msg.node_().node_position_x_().push_back(position_x); //点X坐标信息
    send_msg.node_().node_position_y_().push_back(position_y); //点Y坐标信息
    send_msg.node_().node_position_z_().push_back(position_z);  //点Z坐标信息
    send_msg.node_().node_yaw_().push_back(yaw);  //点yaw角信息

    send_msg.node_().node_attribute_().push_back(0); //未开放属性，请赋值为0，注意：不可为空！！！不可为空！！！不可为空！！！
    send_msg.node_().undefined_().push_back(0);
    send_msg.node_().node_state_2_().push_back(0);
    send_msg.node_().node_state_3_().push_back(0);
    pubQtNode->Write(send_msg);
    cout<<"Add Node.   Name: "<<node_name<<"  X:"<<position_x<<"  Y:"<<position_y<<"  Z:"<<position_z<<"  Yaw:"<<yaw<<endl;
    if(node_name >= 2)addEdge(node_name - 1, node_name - 1, node_name);//顺序连接点
}

void slamDemo::addEdge(u_int16_t edge_name, u_int16_t start_node, u_int16_t end_node){
    index++;
    unitree_interfaces::msg::dds_::QtEdge_ send_msg;

    send_msg.seq_().data() = "index:"+to_string(index)+";";
    send_msg.edge_().edge_name_().push_back(edge_name);
    send_msg.edge_().start_node_name_().push_back(start_node); //边的起始点名称
    send_msg.edge_().end_node_name_().push_back(end_node); //边的终止点名称
    send_msg.edge_().dog_speed_().push_back(0.5); //狗走这条边时的速度（0-1），建议赋值0.5

    send_msg.edge_().dog_stats_().push_back(0);  //未开放属性，请赋值为0，注意：不可为空！！！不可为空！！！不可为空！！！
    send_msg.edge_().edge_length_().push_back(0);  
    send_msg.edge_().dog_back_stats_().push_back(0); 
    send_msg.edge_().edge_state_().push_back(0);  
    send_msg.edge_().edge_state_1_().push_back(0);
    send_msg.edge_().edge_state_2_().push_back(0);
    send_msg.edge_().edge_state_3_().push_back(0);
    send_msg.edge_().edge_state_4_().push_back(0);

    pubQtEdge->Write(send_msg);
    cout<<"Add Edge.   Name: "<<edge_name<<"  Start node:"<<start_node<<"  End node:"<<end_node<<endl;
    cout<<"--------------------------------------------------------------------------"<<endl;
}

void slamDemo::deleteAllNode(){
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:"+to_string(index)+";";
    send_msg.command_() = 1;  //1为删除指令
    send_msg.attribute_() = 1; //1为点
    send_msg.node_edge_name_().push_back(999);

    pubQtCommand->Write(send_msg);
}

void slamDemo::deleteAllEdge(){
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:"+to_string(index)+";";
    send_msg.command_() = 1;  //1为删除指令
    send_msg.attribute_() = 2; //2为边
    send_msg.node_edge_name_().push_back(999);
    pubQtCommand->Write(send_msg);  
}

void slamDemo::pauseNavigation(){
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:"+to_string(index)+";";
    send_msg.command_() = 13;  //13为暂停导航命令
    pubQtCommand->Write(send_msg);
}

void slamDemo::recoverNavigation(){
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:"+to_string(index)+";";
    send_msg.command_() = 14;  //14为恢复导航命令
    pubQtCommand->Write(send_msg);
}

//按键检测
unsigned char slamDemo::keyDetection(){
    termios tms_old, tms_new;
    tcgetattr(0, &tms_old);
    tms_new = tms_old;
    tms_new.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSANOW, &tms_new);
    unsigned char ch = getchar();
    tcsetattr(0, TCSANOW, &tms_old);
    cout<<"\033[1;32m"<<"Key "<<ch<<" pressed."<<"\033[0m"<<endl;
    return ch;
}

//执行
void slamDemo::keyExecute(){
    unsigned char currentKey;
    while(true){
        currentKey = keyDetection();
        switch (currentKey)
        {
        case 'q'://关闭所有节点
            closeAllNode();
            break;
        case 'w'://开始建图（默认清空点/边信息）
            deleteAllNode();
            deleteAllEdge();
            startMapping();
            node_name=0;
            break;
        case 'e'://结束建图
            endMapping();
            break;
        case 'a'://开始循环导航(默认点)
            startRelocation();
            startNavigation();
            initPose();
            defaultNavigation();
            break;
        case 's'://暂停导航
            pauseNavigation();
            break;
        case 'd'://恢复导航
            recoverNavigation();
            break;  
        case 'z'://开启定位和导航，为采集点/边信息作准备。（默认清空点/边信息）
            deleteAllNode();
            deleteAllEdge();
            startRelocation();
            startNavigation();
            initPose();
            break;
        case 'x'://采集点边信息
            addNodeAndEdge();
            break;
        case 'c'://清空点边信息
            deleteAllNode();
            deleteAllEdge();
            node_name=0;
            break;
        default:
            break;
        }
    }
}

int main()
{
    slamDemo slamTest;
    slamTest.keyExecute();
    return 0;
}