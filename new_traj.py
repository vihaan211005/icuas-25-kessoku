import numpy as np
import csv
import matplotlib.pyplot as plt

'''
a is taken as a polynomial of 5th degree (position of 7th degree)
Constraints : a(0) = a(tau) = 0
              a'(0) = a'(tau) = 0
              integral of a from 0 to tau = 1
              0 <= a(t) <= 1 for all t in [0, tau]

'''

def acceleration_poly(c, t, tau):
    return c * t**2 * (tau - t)**2

def velocity_poly(c, t, tau):
    return c * t**3/30 * (6* t**2 - 15*t*tau + 10*tau**2)

def yaw_poly(yaw_factor, T, t):
    return yaw_factor/6 * t**2 * (3*T - 2*t)

tau = 30/16
c = 30/tau**5

print(f"Optimal tau: {tau}")

distance_tau = tau**6/60

p0 = np.array([0, 0, 0, 0])  # Start point
pf = np.array([4, 3, 2, 1])  # End point
v_max = 1

remaining_dist = np.linalg.norm(pf[:3] - p0[:3]) - 2 * distance_tau
direction = (pf[:3] - p0[:3]) / np.linalg.norm(pf[:3] - p0[:3])
time_gap = remaining_dist * v_max
T = 2*tau + time_gap
yaw_factor = 6 * (pf[3] - p0[3]) / (T**3)

def plot_piecewise(t1 = tau, t2 = tau + time_gap, t3 = T, num_points=100):
    
    # Generate time values for each interval
    t_vals1 = np.linspace(0, t1, num_points)
    t_vals2 = np.linspace(t1, t2, num_points)
    t_vals3 = np.linspace(t2, t3, num_points)
    
    a_vals1 = acceleration_poly(c, t_vals1, tau)
    a_vals2 = [0 for _ in t_vals2]
    a_vals3 = -acceleration_poly(c, t_vals3 - tau - time_gap, tau)

    v_vals1 = velocity_poly(c, t_vals1, tau)
    v_vals2 = [1 for _ in t_vals2]
    v_vals3 = 1 - velocity_poly(c, t_vals3 - tau - time_gap, tau)

    yaw_vals1 = yaw_poly(yaw_factor, T, t_vals1)
    yaw_vals2 = yaw_poly(yaw_factor, T, t_vals2)
    yaw_vals3 = yaw_poly(yaw_factor, T, t_vals3)

    # Plot the function
    plt.figure(figsize=(8, 5))

    plt.plot(t_vals1, a_vals1, 'b', label = "Acceleration")
    plt.plot(t_vals2, a_vals2, 'b')
    plt.plot(t_vals3, a_vals3, 'b')
    plt.plot(t_vals1, v_vals1, 'r', label = "Velocity")
    plt.plot(t_vals2, v_vals2, 'r')
    plt.plot(t_vals3, v_vals3, 'r')
    plt.plot(t_vals1, yaw_vals1, 'g', label = "Yaw")
    plt.plot(t_vals2, yaw_vals2, 'g')
    plt.plot(t_vals3, yaw_vals3, 'g')

    plt.scatter([tau], [0], color='black', zorder=3, label=f'tau = {tau}')
    plt.scatter([T], [0], color='black', zorder=3, label=f'T = {T}')
    
    plt.xlabel('t')
    plt.title('Piecewise plot of acceleration, velocity, and yaw')
    plt.legend()
    plt.grid()
    plt.show()

'''
velocity at time t between 0 and tau is given by t^3/30 (6t^2 - 15 t tau + 10tau^2) 
distance travelled upto time t between 0 and tau is given by t^4/60 (2t^2 - 6 t tau + 5tau^2)
'''

def compute_trajectory_coefficients(p0, tau, flag):

    """Computes the polynomial coefficients for each spatial component."""
    if flag == 1 or flag == 3:
        coeffs_displacement = [0, 0, 0, 5 * tau**2/(60), -6*tau/(60), 2/(60)]
        if flag == 1:
            pf = [p0[i] + distance_tau * direction[i] for i in range(3)] + [yaw_poly(yaw_factor, T, tau)]
        else:
            pf = [p0[i] + distance_tau * direction[i] for i in range(3)] + [yaw_poly(yaw_factor, T, time_gap + 2*tau)]

    if flag == 2:
        coeffs_displacement = [v_max, 0, 0, 0, 0, 0]
        pf = [p0[i] + remaining_dist * direction[i] for i in range(3)] + [yaw_poly(yaw_factor, T, tau + time_gap)]
        
    coeffs_x = [p0[0]] + [c * direction[0] for c in coeffs_displacement]
    coeffs_y = [p0[1]] + [c * direction[1] for c in coeffs_displacement]
    coeffs_z = [p0[2]] + [c * direction[2] for c in coeffs_displacement]
    coeffs_yaw = [p0[3], 0, T/2 * yaw_factor, -yaw_factor/3] + [0]*3
    
    return pf, coeffs_x + coeffs_y + coeffs_z + coeffs_yaw

# Write to CSV file
csv_filename = "trajectory_coefficients.csv"
header = ["Duration"] + [f"x^{i}" for i in range(7)] + [f"y^{i}" for i in range(7)] + [f"z^{i}" for i in range(7)] + [f"yaw^{i}" for i in range(7)]

with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(header)
    pf, coeff = compute_trajectory_coefficients(p0, tau, 1)
    writer.writerow([tau] + coeff)
    pf, coeff = compute_trajectory_coefficients(pf, tau, 2)
    writer.writerow([time_gap] + coeff)
    pf, coeff = compute_trajectory_coefficients(pf, tau, 3)
    writer.writerow([tau] +  [coeff[0]] + list((-1) * np.array(coeff))[1:7] + [coeff[7]] + list((-1) * np.array(coeff))[8:14] + [coeff[14]] + list((-1) * np.array(coeff))[15:21] + coeff[21:])
    
print(f"Coefficients saved to {csv_filename}")
plot_piecewise()