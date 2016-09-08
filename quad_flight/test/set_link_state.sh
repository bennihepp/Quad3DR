LINK="quadrotor::RGBD_gimbal_camera_link"
#LINK="quadrotor::RGBD_gimbal_yaw_link"
REFERENCE="quadrotor::base_link"

X=0.10
Y=0.0
Z=-0.1

OX=0.0
OY=0.0
OZ=0.2
OW=-0.95

rosservice call /gazebo/set_link_state "{link_state: { link_name: '${LINK}', pose: { position: {x: ${X}, y: ${Y}, z: ${Z}}, orientation: {x: ${OX}, y: ${OY}, z: ${OZ}, w: ${OW}} }, reference_frame: '${REFERENCE}' } }"

