LINK="quadrotor::RGBD_gimbal_camera_link"
REFERENCE="quadrotor::base_link"

rosservice call /gazebo/get_link_state "{link_name: '${LINK}', reference_frame: '${REFERENCE}'}"

