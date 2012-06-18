sensor_msgs/PointCloud2   points #pcl::PointXYZRGBNormal cloud describing the ICR contact points expressed in the parent object's frame 
string                    phalange #name of the phalange for which the ICR were computed, e.g., "thdist"
uint32[]                  indices #array holding the parent object's point indicies for the region
uint32                    mp_index # index of the middle point of the contact region
geometry_msgs/Point       middle_point


