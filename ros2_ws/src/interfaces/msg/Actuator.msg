std_msgs/Header header      
# Single rudder vessel fields
float64 rudder
float64 propeller

# MAVYMINI control surfaces
float64 upper_rudder
float64 lower_rudder
float64 stern_fin_left
float64 stern_fin_right
float64 bow_fin_left
float64 bow_fin_right

# Covariance matrix (flattened)
# For single rudder: 2x2 matrix = 4 elements
# For MAVYMINI: 7x7 matrix = 49 elements (6 surfaces + 1 propeller)
float64[] covariance