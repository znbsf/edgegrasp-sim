# Atomic target identity and clock metadata for sequence orchestration.
# observation.header.stamp is the source timestamp; receive liveness remains
# a subscriber-local measurement and must not be inferred from this message.
string target_id
geometry_msgs/PointStamped observation
string clock_domain
uint64 clock_epoch
