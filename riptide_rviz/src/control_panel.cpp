#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

class MissionPanel: public rviz_common::Panel {
Q_OBJECT public:
  MissionPanel( QWidget* parent = 0 );

  virtual void load( const rviz_common::Config& config );
  virtual void save( rviz_common::Config config ) const;
  
protected Q_SLOTS:
    //QT slots (function callbacks)
  void trigger_service(bool msg, std::string service_name);
  void set_dog_status(bool msg);
  void set_mode(int mode_id);
  void set_gait(int gait_id);
  void set_height(int height);
  void set_order_id(int order_id);
  void send_order();
  void set_wav_id();
  void play_wav();
  void set_volume(int vol);

protected:
  bool event(QEvent *event);

private:
  void discover_dogs_ns();

};