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

tau = 30/16
c = 30/tau**5

print(f"Optimal tau: {tau}")

# Plot the optimized acceleration function
t_vals = np.linspace(0, tau, 1000)
a_vals = acceleration_poly(c, t_vals, tau)
plt.plot(t_vals, a_vals, label="Optimized Acceleration")
plt.xlabel("Time")
plt.ylabel("Acceleration")
plt.title("Optimal Acceleration Profile")
plt.legend()
plt.grid()
plt.show()

distance_tau = tau**6/60

p0 = np.array([0, 0, 0])  # Start point
pf = np.array([50, 40, 30])  # End point
v_max = 1

remaining_dist = np.linalg.norm(pf - p0) - 2 * distance_tau
direction = (pf - p0) / np.linalg.norm(pf - p0)
time_gap = remaining_dist * v_max

'''
velocity at time t between 0 and tau is given by t^3/30 (6t^2 - 15 t tau + 10tau^2) 
distance travelled upto time t between 0 and tau is given by t^4/60 (2t^2 - 6 t tau + 5tau^2)
'''

def compute_trajectory_coefficients(p0, tau, flag):

    """Computes the polynomial coefficients for each spatial component."""
    if flag == 1:
        coeffs_displacement = [0, 0, 0, 5 * tau**2/(60), -6*tau/(60), 2/(60)]
        pf = [p0[i] + distance_tau * direction[i] for i in range(3)]

    if flag == 2:
        coeffs_displacement = [v_max, 0, 0, 0, 0, 0]
        pf = [p0[i] + remaining_dist * direction[i] for i in range(3)]
        
    coeffs_x = [p0[0]] + [c * direction[0] for c in coeffs_displacement]
    coeffs_y = [p0[1]] + [c * direction[1] for c in coeffs_displacement]
    coeffs_z = [p0[2]] + [c * direction[2] for c in coeffs_displacement]
    
    return pf, coeffs_x + coeffs_y + coeffs_z

# Write to CSV file
csv_filename = "trajectory_coefficients.csv"
header = ["Duration"] + [f"x^{i}" for i in range(7)] + [f"y^{i}" for i in range(7)] + [f"z^{i}" for i in range(7)]

with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(header)
    pf, coeff = compute_trajectory_coefficients(p0, tau, 1)
    writer.writerow([tau] + coeff)
    pf, coeff = compute_trajectory_coefficients(pf, tau, 2)
    writer.writerow([time_gap] + coeff)
    pf, coeff = compute_trajectory_coefficients(pf, tau, 1)
    writer.writerow([tau] +  [coeff[0]] + list((-1) * np.array(coeff))[1:7] + [coeff[7]] + list((-1) * np.array(coeff))[8:14] + [coeff[14]] + list((-1) * np.array(coeff))[15:] )
    
print(f"Coefficients saved to {csv_filename}")
