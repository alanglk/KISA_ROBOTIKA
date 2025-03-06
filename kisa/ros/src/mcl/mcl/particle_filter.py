import copy
import numpy as np
from mcl.resampler import Resampler


class ParticleFilter(object):
    """
    Notes:
        * State is (x, y, heading), where x and y are in meters and heading in radians
        * State space assumed limited size in each dimension, world is cyclic (hence leaving at x_max means entering at
        x_min)
        * propagation and measurement models are largely hardcoded (except for standard deviations.
    """

    def __init__(self,
                 number_of_particles,
                 limits,
                 process_noise,
                 measurement_noise):
        """
        Initialize the SIR particle filter.

        :param number_of_particles: Number of particles.
        :param limits: List with maximum and minimum values for x and y dimension: [xmin, xmax, ymin, ymax].
        :param process_noise: Process noise parameters (standard deviations): [std_forward, std_angular].
        :param measurement_noise: Measurement noise parameters (standard deviations): [std_range, std_angle].
        :param resampling_algorithm: Algorithm that must be used for core.
        """
        # Initialize particle filter base class
        if number_of_particles < 1:
            print("Warning: initializing particle filter with number of particles < 1: {}".format(number_of_particles))

        # Initialize filter settings
        self.n_particles = number_of_particles
        self.particles = []

        # State related settings
        self.state_dimension = 3  # x, y, theta
        self.x_min = limits[0]
        self.x_max = limits[1]
        self.y_min = limits[0]
        self.y_max = limits[1]

        # Set noise
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        
        self.resampler = Resampler()

    def needs_resampling_nepr(self):
        """
        Method that determines whether not a core step is needed for the current particle filter state estimate.
        The sampling importance core (SIR) scheme resamples every time step hence always return true.

        :return: Boolean indicating whether or not core is needed.
        
        """
        sum_weights_squared = 0
        for par in self.particles:
            sum_weights_squared += par[0] * par[0]
        # print("NEPR resampling? ", 1.0 / sum_weights_squared)
        return 1.0 / sum_weights_squared < self.n_particles/2.0
        
    def needs_resampling_mwr(self, th):
        """
        Override method that determines whether or not a core step is needed for the current particle filter state
        estimate. Resampling only occurs if the reciprocal of the maximum particle weight falls below the user-specified
        threshold. The reciprocal of the maximum weight is defined by P_N^2 in [1].

        [1] Martino, Luca, Victor Elvira, and Francisco Louzada. "Effective sample size for importance sampling based on
        discrepancy measures." Signal Processing 131 (2017): 386-401.

        :return: Boolean indicating whether or not core is needed.
        """
        max_weight = 0
        for par in self.particles:
            max_weight = max(max_weight, par[0])

        return 1.0 / max_weight < th

    def initialize_particle_uniform(self):
        """
        Initialize the particles uniformly over the world assuming a 3D state (x, y, heading). No arguments are required
        and function always succeeds hence no return value.
        """
        # Initialize particles with uniform weight distribution
        
        weight = 1.0 / self.n_particles
        return  [weight, [
                    np.random.uniform(self.x_min, self.x_max),
                    np.random.uniform(self.y_min, self.y_max),
                    np.random.uniform(-np.pi, np.pi)]
                ]
            

    def initialize_particles_uniform(self):
        """
        Initialize the particles uniformly over the world assuming a 3D state (x, y, heading). No arguments are required
        and function always succeeds hence no return value.
        """
        print("initializing particles inside: ", self.x_min, self.x_max, self.y_min, self.y_max)
        # Initialize particles with uniform weight distribution
        self.particles = []
        weight = 1.0 / self.n_particles
        for i in range(self.n_particles):
            # Add particle i
            self.particles.append(
                [weight, [
                    np.random.uniform(self.x_min, self.x_max, 1)[0],
                    np.random.uniform(self.y_min, self.y_max, 1)[0],
                    np.random.uniform(-np.pi, np.pi, 1),[0]]
                 ]
            )

    def initialize_particles_gaussian(self, mean_vector, standard_deviation_vector):
        """
        Initialize particle filter using a Gaussian distribution with dimension three: x, y, heading. Only standard
        deviations can be provided hence the covariances are all assumed zero.

        :param mean_vector: Mean of the Gaussian distribution used for initializing the particle states
        :param standard_deviation_vector: Standard deviations (one for each dimension)
        :return: Boolean indicating success
        """

        # Check input dimensions
        if len(mean_vector) != self.state_dimension or len(standard_deviation_vector) != self.state_dimension:
            print("Means and state deviation vectors have incorrect length in initialize_particles_gaussian()")
            return False

        # Initialize particles with uniform weight distribution
        self.particles = []
        weight = 1.0 / self.n_particles
        for i in range(self.n_particles):

            # Get state sample
            state_i = np.random.normal(mean_vector, standard_deviation_vector, self.state_dimension).tolist()

            # Add particle i
            self.particles.append([weight, state_i])



    def set_particles(self, particles):
        """
        Initialize the particle filter using the given set of particles.

        :param particles: Initial particle set: [[weight_1, [x1, y1, theta1]], ..., [weight_n, [xn, yn, thetan]]]
        """
        # Assumption: particle have correct format, set particles
        self.particles = copy.deepcopy(particles)
        self.n_particles = len(self.particles)

    def get_average_state(self):
        """
        Compute average state according to all weighted particles

        :return: Average x-position, y-position and orientation
        """
        
        # Compute sum of all weights
        sum_weights = 0.0
        for weighted_sample in self.particles:
            #print("*************gat average_state:", weighted_sample)
            sum_weights += weighted_sample[0]

        # Compute weighted average
        avg_x = 0.0
        avg_y = 0.0
        avg_theta = 0.0
        for weighted_sample in self.particles:
            avg_x += weighted_sample[0] / sum_weights * weighted_sample[1][0]
            avg_y += weighted_sample[0] / sum_weights * weighted_sample[1][1]
            avg_theta += weighted_sample[0] / sum_weights * weighted_sample[1][2]

        return [avg_x, avg_y, avg_theta]

    def get_max_weight(self):
        """
        Find maximum weight in particle filter.

        :return: Maximum particle weight
        """
        return max([weighted_sample[0] for weighted_sample in self.particles])


    def print_particles(self):
        """
        Print all particles: index, state and weight.
        """
        print("Particles:")
        for i in range(self.n_particles):
            print(" ({}): {} with w: {}".format(i+1, self.particles[i][1], self.particles[i][0]))

 
    def normalize_scaled_weights(self, weighted_samples):
        # Escalar los pesos para mejorar la estabilidad numérica
        # Utilizamos el máximo de los pesos para escalar, reduciendo el riesgo de underflow
        max_weight = max([weighted_sample[0] for weighted_sample in self.particles])

        if max_weight == 0:
           raise ValueError("Todos los pesos son cero, revisa el modelo de partículas.")
    
        scaled_samples = [[weighted_sample[0]/max_weight, weighted_sample[1]] for weighted_sample in weighted_samples]

    
        # Normalizar los pesos escalados para que sumen 1
        sum_scaled_weights = 0.0
        for scaled_sample in scaled_samples:
            sum_scaled_weights += scaled_sample[0]

        return [[scaled_sample[0] / sum_scaled_weights, scaled_sample[1]] for scaled_sample in scaled_samples]
    
        

    def normalize_weights(self, weighted_samples):
        """
        Normalize all particle weights.
        """

        # Compute sum weighted samples
        sum_weights = 0.0
        for weighted_sample in weighted_samples:
            sum_weights += weighted_sample[0]

        # Check if weights are non-zero
        if sum_weights < 1e-15:
            print("Weight normalization failed: sum of all weights is {} (weights will be reinitialized)".format(sum_weights))

            # Set uniform weights
            return [[1.0 / len(weighted_samples), weighted_sample[1]] for weighted_sample in weighted_samples]

        # Return normalized weights
        return [[weighted_sample[0] / sum_weights, weighted_sample[1]] for weighted_sample in weighted_samples]

    def propagate_sample(self, sample, forward_motion, angular_motion):
        """
        Propagate an individual sample with a simple motion model that assumes the robot rotates angular_motion rad and
        then moves forward_motion meters in the direction of its heading. Return the propagated sample (leave input
        unchanged).

        :param sample: Sample (unweighted particle) that must be propagated
        :param forward_motion: Forward motion in meters
        :param angular_motion: Angular motion in radians
        :return: propagated sample
        """
        # 1. rotate by given amount plus additive noise sample (index 1 is angular noise standard deviation)
        propagated_sample = copy.deepcopy(sample)
        propagated_sample[2] += np.random.normal(angular_motion, self.process_noise[1], 1)[0]

        # Compute forward motion by combining deterministic forward motion with additive zero mean Gaussian noise
        forward_displacement = np.random.normal(forward_motion, self.process_noise[0], 1)[0]

        # 2. move forward
        propagated_sample[0] += forward_displacement * np.cos(propagated_sample[2])
        propagated_sample[1] += forward_displacement * np.sin(propagated_sample[2])

        # Make sure we stay within cyclic world
        return propagated_sample

    def compute_likelihood(self, expected_scan, measured_scan):
        """
        Compute likelihood p(z|sample) for a specific measurement given sample state and landmarks.

        :param sample: Sample (unweighted particle) that must be propagated
        
        :return Likelihood
        """
        assert len(expected_scan) == len(measured_scan)

        # Initialize measurement likelihood
        likelihood_sample = 1.0
        n = len(measured_scan)
        # Loop over all landmarks for current particle
        for i in range(n):
            # Map difference true and expected distance measurement to probability
            p_z_given_distance = \
                np.exp(-(expected_scan[i]- measured_scan[i]) * (expected_scan[i]-measured_scan[i]) /
                       (2 * self.measurement_noise * self.measurement_noise))  
                          
        #     # Incorporate likelihoods current landmark
            likelihood_sample *= p_z_given_distance

        # Return importance weight based on all landmarks
        return likelihood_sample 