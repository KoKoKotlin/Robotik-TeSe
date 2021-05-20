#include <memory>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/laser_scan.hpp> //include für Datentyp sensor_msgs::msg::LaserScan

using std::placeholders::_1;

class LiDAR_Filter : public rclcpp::Node{
    private:
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr republisher_;       //erstellen des Publishers über den gefilterte Daten gepublished werden
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;   //erstellen des Subscribers über den rohe Daten angenommen werden

    public:
        LiDAR_Filter() : Node("LiDAR_FilterNode"){                                                                                                        //constructor
            subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&LiDAR_Filter::filtering, this, _1));             //Topicname muss ggf noch angepasst werden //Topic zuordnung beim Subscribers
            republisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("LiDAR_Filterdaten", 10);                                                  //Topicname muss ggf noch angepasst werden //Topic zuordnung beim Publisher
        }
    
    private:
        void filtering(const sensor_msgs::msg::LaserScan::SharedPtr msg) const{
            //TODO Filter Implementieren
            //
            //
            //
            //
        }
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LiDAR_Filter>());
  rclcpp::shutdown();
  return 0;
}