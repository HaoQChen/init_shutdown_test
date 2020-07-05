#include "ros/ros.h"
#include <signal.h>


void MySigintHandler(int sig)
{
    //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
    ROS_INFO("shutting down!");
    ros::shutdown();
}


int main(int argc, char** argv){
//ros::init()
    /**
      * The ros::init() function needs to see argc and argv so that it can perform
      * any ROS arguments and name remapping that were provided at the command line. For programmatic
      * remappings you can use a different version of init() which takes remappings
      * directly, but for most command-line programs, passing argc and argv is the easiest
      * way to do it.  The third argument to init() is the name of the node.
      *
      * You must call one of the versions of ros::init() before using any other
      * part of the ROS system.
    */
    ros::init(argc, argv, "ist_node");
    //初始化节点名字必须在最前面，如果ROS系统中出现重名，则之前的节点会被自动关闭
    //如果想要多个重名节点而不报错，可以在init中添加ros::init_options::AnonymousName参数
    //该参数会在原有节点名字的后面添加一些随机数来使每个节点独一无二
    //ros::init(argc, argv, "my_node_name", ros::init_options::AnonymousName);

    //如果是多机系统，可以使用另外一个版本的init函数，函数定义见下面的地址
    //http://docs.ros.org/api/roscpp/html/namespaceros.html#a61a193529a9aad90ddace7724c7fc759
    //master_url默认是http://localhost:11311，多机连接可以看下面这个教程
    //https://blog.csdn.net/lisfaf/article/details/90444541
    //std::map<std::string, std::string> remappings;
    //remappings["__master"] = master_url;
    //remappings["__hostname"] = host_url;
    //ros::init(remappings, "ist_node");

//rosmaster
    if (!ros::master::check()){
        printf("There is no master, please run roscore first\n");
        return -1;
    }
    // ros::master::check函数需要在ros::init之后调用

//ros::NodeHandle
    /**
      * NodeHandle is the main access point to communications with the ROS system.
      * The first NodeHandle constructed will fully initialize this node, and the last
      * NodeHandle destructed will call ros::shutdown() to close down the node.
    */
    ros::NodeHandle h_node;
    //获取节点的句柄,init是初始化节点，这个是Starting the node
    //如果不想通过对象的生命周期来管理节点的开始和结束，你可以通过ros::start()和ros::shutdown() 来自己管理节点。
    
    ros::Rate loop_rate(1);
    //loop once per second
    //Cannot use before the first NodeHandle has been created or ros::start() has been called.

//shut down
    signal(SIGINT, MySigintHandler);
    //覆盖原来的Ctrl+C中断函数，原来的只会调用ros::shutdown()
    //为你关闭节点相关的subscriptions, publications, service calls, and service servers，退出进程
    //注意，不是所有的退出都会有SIGINT信号，比如调用rosnode kill来杀节点就不会有这个信号，但ros::ok()会变false

//run status
    int sec = 0;
    while(ros::ok() && sec++ < 5){
        loop_rate.sleep();
        ROS_INFO("ROS is ok!");
        ros::spinOnce();
    }
    //ros::ok()返回false，代表可能发生了以下事件
        //1.SIGINT被触发(Ctrl-C)调用了ros::shutdown()
        //2.被另一同名节点踢出 ROS 网络
        //3.ros::shutdown()被程序的另一部分调用
        //4.节点中的所有ros::NodeHandles 都已经被销毁
    //ros::isShuttingDown():一旦ros::shutdown()被调用（注意是刚开始调用，而不是调用完毕），就返回true
    //一般建议用ros::ok()，特殊情况可以用ros::isShuttingDown()

    ROS_INFO("Node exit");
    printf("Process exit\n");
    return 0;
}