#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <string>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

void callback(const std_msgs::Bool::ConstPtr& msg, tcp::socket& socket)
{
    bool data = msg->data;
    // 메시지를 문자열로 직렬화
    std::ostringstream oss;
    oss << std::boolalpha << data << '\n';
    std::string message = oss.str();
    std::cout << "Received: " << message;

    // 데이터 전송
    boost::asio::write(socket, boost::asio::buffer(message));
}

int main(int argc, char** argv)
{
    // ROS 노드 초기화
    ros::init(argc, argv, "tcp_platoon_publisher");
    ros::NodeHandle nh;

    // TCP 클라이언트 설정
    boost::asio::io_service io_service;
    tcp::socket socket(io_service);
    tcp::resolver resolver(io_service);
    tcp::resolver::query query("192.168.0.83", "1300");  // 서버 주소와 포트 번호 지정 //cargo : 1200, platoon : 1300
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

    try
    {
        // 서버에 연결
        boost::asio::connect(socket, endpoint_iterator);

        // ROS 토픽 구독
        ros::Subscriber sub = nh.subscribe<std_msgs::Bool>(
            "/platoon_go", 10,
            [&socket](const std_msgs::Bool::ConstPtr& msg) {
                callback(msg, socket);
            }
        );

        // 데이터 전송 및 ROS 루프 실행
        ros::Rate rate(10);  // 10Hz 주기 설정
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }

        // 소켓 종료
        socket.close();
    }
    catch (const boost::system::system_error& e)
    {
        ROS_ERROR("TCP connection error: %s", e.what());
    }

    return 0;
}