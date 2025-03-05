int16 n_landmarks
int16 n_state
# state vector of size (n_state+2*n_landmarks, 1) 
float64[] mu 
 # covariance matrix in single row format of len (n_state+num_landmarks * 2)**2
float64[] sigma
