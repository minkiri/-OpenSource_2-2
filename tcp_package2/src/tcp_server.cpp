#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <boost/asio.hpp>
#include <iostream>

using boost::asio::ip::tcp;

int main(int argc, char** argv)
{
    // ROS 노드 초기화
    ros::init(argc, argv, "tcp_platoon_server");
    ros::NodeHandle nh;

    // 토픽 발행자 생성
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("/platoon_go", 10);

    // TCP 서버 설정
    boost::asio::io_service io_service;
    tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), 1300));

    try
    {
        ROS_INFO("Waiting for TCP connection...");

        // 클라이언트 연결 대기
        tcp::socket socket(io_service);
        acceptor.accept(socket);

        ROS_INFO("TCP client connected!");

        while (ros::ok())
        {
            // 데이터 수신
            boost::asio::streambuf buffer;
            boost::asio::read_until(socket, buffer, '\n');
            std::istream input_stream(&buffer);
            std::string message;
            std::getline(input_stream, message);

            // 문자열을 bool 값으로 변환
            std_msgs::Bool msg;
            msg.data = (message == "true");
            std::cout<<message<<std::endl;

            // ROS 토픽에 데이터 발행
            pub.publish(msg);

            ros::spinOnce();
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