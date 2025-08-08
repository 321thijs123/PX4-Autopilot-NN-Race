# Gate Publisher
#
# Gates topic for drone racing, publishes the position of gates to race through
# Publisher: gate_publisher
# Subscriber: mc_nn_control

uint64 timestamp # [us] Time since system start

float32[4] x
float32[4] y
float32[4] z
float32[4] yaw
