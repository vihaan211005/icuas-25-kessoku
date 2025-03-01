import numpy as np
import csv
import matplotlib.pyplot as plt
import argparse

'''
a is taken as a polynomial of 5th degree (position of 7th degree)
Constraints : a(0) = a(tau) = 0
              a'(0) = a'(tau) = 0
              integral of a from 0 to tau = 1
              0 <= a(t) <= 1 for all t in [0, tau]

'''

v_max = 1
header = ["Duration"] + [f"x^{i}" for i in range(8)] + [f"y^{i}" for i in range(8)] + [f"z^{i}" for i in range(8)] + [f"yaw^{i}" for i in range(8)] 


def evaluate_polynomial(coeffs, t):
    result = np.zeros_like(t, dtype=float)
    for coeff in reversed(coeffs): # Process from highest degree to lowest
        result = result * t + coeff
    return result


def binomial_coeff(n, k):
    if k > n:
        return 0
    if k == 0 or k == n:
        return 1
    num, denom = 1, 1
    for i in range(k):
        num *= (n - i)
        denom *= (i + 1)
    return num // denom

def shift_polynomial(coeffs, tau):

    degree = len(coeffs) - 1
    new_coeffs = np.zeros(len(coeffs))

    for k in range(len(coeffs)):
        for j in range(k + 1):
            new_coeffs[j] += coeffs[k] * binomial_coeff(k, j) * (-tau) ** (k - j)

    return new_coeffs.tolist()



def yaw_poly(yaw_factor, T, t):
    return yaw_factor/6 * t**2 * (3*T - 2*t)

def case1(c, tau, dist, distance_tau, p0, pf):
    # def acceleration_poly(c, t, tau):
    #     return c * t**2 * (tau - t)**2
    # def velocity_poly(c, t, tau):
    #     return c * t**3/30 * (6* t**2 - 15*t*tau + 10*tau**2)
    # y^1 - 0
    # y^2 - 0
    # y^3 - 0
    # y^4 - (5ctau^2)/60
    # y^5 - -ctau/10
    # y^6 - c/30
    # y^7 - 0
    # print(f"Optimal tau: {tau}")

    c1_x=[]
    c2_x=[]
    c3_x=[]
    c1_y=[]
    c2_y=[]
    c3_y=[]
    c1_z=[]
    c2_z=[]
    c3_z=[]

    remaining_dist = dist - 2 * distance_tau
    time_gap = remaining_dist * v_max
    T = 2*tau + time_gap
    yaw_factor = 6 * (pf[3] - p0[3]) / (T**3)

    def compute_trajectory_coefficients(p0, tau, flag):
        nonlocal c1_x, c2_x, c3_x, c1_y, c2_y, c3_y, c1_z, c2_z, c3_z
        if flag == 1:
            coeffs_displacement = [0, 0, 0, (c*(tau**2))/12, ((-tau)*c)/10, c/30, 0]
            pf = [p0[i] + distance_tau * direction[i] for i in range(3)] + [yaw_poly(yaw_factor, T, tau)]
        elif flag==3:
            coeffs_displacement = [0, 0, 0, ((-c)*(tau**2))/12, ((-tau)*c)/10, (-c)/30, 0]
            pf = [p0[i] + distance_tau * direction[i] for i in range(3)] + [yaw_poly(yaw_factor, T, time_gap + 2*tau)]
        elif flag == 2:
            coeffs_displacement = [v_max, 0, 0, 0, 0, 0, 0]
            pf = [p0[i] + remaining_dist * direction[i] for i in range(3)] + [yaw_poly(yaw_factor, T, tau + time_gap)]
        
        if(flag == 3):
            p0[0] += distance_tau * direction[0]
            p0[1] += distance_tau * direction[1]
            p0[2] += distance_tau * direction[2]
        coeffs_x = [p0[0]] + [c * direction[0] for c in coeffs_displacement]
        coeffs_y = [p0[1]] + [c * direction[1] for c in coeffs_displacement]
        coeffs_z = [p0[2]] + [c * direction[2] for c in coeffs_displacement]
        coeffs_yaw = [p0[3], 0, T/2 * yaw_factor, -yaw_factor/3] + [0]*4

        if(flag == 1):
            c1_x = coeffs_x
            c1_y = coeffs_y
            c1_z = coeffs_z
        if(flag == 2):
            coeffs_x = shift_polynomial(coeffs_x, tau)
            coeffs_y = shift_polynomial(coeffs_y, tau)
            coeffs_z = shift_polynomial(coeffs_z, tau)
            c2_x = coeffs_x
            c2_y = coeffs_y
            c2_z = coeffs_z
        if(flag == 3):
            coeffs_x = shift_polynomial(coeffs_x, T)
            coeffs_y = shift_polynomial(coeffs_y, T)
            coeffs_z = shift_polynomial(coeffs_z, T)
            c3_x = coeffs_x
            c3_y = coeffs_y
            c3_z = coeffs_z        
        
        return pf, coeffs_x + coeffs_y + coeffs_z + coeffs_yaw
    
    with open(file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)
        pf, coeff = compute_trajectory_coefficients(p0, tau, 1)
        writer.writerow([tau] + coeff)
        pf, coeff = compute_trajectory_coefficients(pf, tau, 2)
        writer.writerow([time_gap] + coeff)
        pf, coeff = compute_trajectory_coefficients(pf, tau, 3)
        writer.writerow([tau] +  [coeff[0]] + list((-1) * np.array(coeff))[1:7] + [coeff[7]] + list((-1) * np.array(coeff))[8:14] + [coeff[14]] + list((-1) * np.array(coeff))[15:21] + coeff[21:])
    

    def plot_piecewise(t1 = tau, t2 = tau + time_gap, t3 = T, num_points=100):
        nonlocal c1_x, c2_x, c3_x, c1_y, c2_y, c3_y, c1_z, c2_z, c3_z
        # Generate time values for each interval
        t_vals1 = np.linspace(0, t1, num_points)
        t_vals2 = np.linspace(t1, t2, num_points)
        t_vals3 = np.linspace(t2, t3, num_points)

        yaw_vals1 = yaw_poly(yaw_factor, T, t_vals1)
        yaw_vals2 = yaw_poly(yaw_factor, T, t_vals2)
        yaw_vals3 = yaw_poly(yaw_factor, T, t_vals3)

        x_vals1 = evaluate_polynomial(c1_x, t_vals1)
        x_vals2 = evaluate_polynomial(c2_x, t_vals2)
        x_vals3 = evaluate_polynomial(c3_x, t_vals3)

        y_vals1 = evaluate_polynomial(c1_y, t_vals1)
        y_vals2 = evaluate_polynomial(c2_y, t_vals2)
        y_vals3 = evaluate_polynomial(c3_y, t_vals3)

        z_vals1 = evaluate_polynomial(c1_z, t_vals1)
        z_vals2 = evaluate_polynomial(c2_z, t_vals2)
        z_vals3 = evaluate_polynomial(c3_z, t_vals3)

        # Plot the function
        plt.figure(figsize=(8, 5))

        plt.plot(t_vals1, yaw_vals1, 'g', label = "Yaw")
        plt.plot(t_vals2, yaw_vals2, 'g')
        plt.plot(t_vals3, yaw_vals3, 'g')

        plt.plot(t_vals1, x_vals1, 'r', label = "x")
        plt.plot(t_vals2, x_vals2, 'r', label = "x")
        plt.plot(t_vals3, x_vals3, 'r', label = "x")

        plt.plot(t_vals1, y_vals1, 'g', label = "y")
        plt.plot(t_vals2, y_vals2, 'g', label = "y")
        plt.plot(t_vals3, y_vals3, 'g', label = "y")

        plt.plot(t_vals1, z_vals1, 'b', label = "z")
        plt.plot(t_vals2, z_vals2, 'b', label = "z")
        plt.plot(t_vals3, z_vals3, 'b', label = "z")

        plt.scatter([tau], [0], color='black', zorder=3, label=f'tau = {tau}')
        plt.scatter([T], [0], color='black', zorder=3, label=f'T = {T}')
        
        plt.xlabel('t')
        plt.title('Piecewise plot of acceleration, velocity, and yaw')
        plt.legend()
        plt.grid()
        plt.show()
    
    # plot_piecewise()

def case2(dist, direction, p0, pf):
    tau = (84/5)**(1/2)/5**(1/4)
    c = 840*dist/tau**7
    yaw_factor = 6 * (pf[3] - p0[3]) / (tau**3)

    c_x=[]
    c_y=[]
    c_z=[]

    def compute_trajectory_coefficients(p0):
        nonlocal c_x, c_y, c_z
        coeffs_displacement = [0, 0, 0, c*tau**3/24, -c*tau**2/10, c*tau/12, -c/42]
            
        coeffs_x = [p0[0]] + [c * direction[0] for c in coeffs_displacement]
        coeffs_y = [p0[1]] + [c * direction[1] for c in coeffs_displacement]
        coeffs_z = [p0[2]] + [c * direction[2] for c in coeffs_displacement]
        coeffs_yaw = [p0[3], 0, tau/2 * yaw_factor, -yaw_factor/3] + [0]*4
        
        c_x = coeffs_x
        c_y = coeffs_y
        c_z = coeffs_z

        return coeffs_x + coeffs_y + coeffs_z + coeffs_yaw

    with open(file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)
        coeff = compute_trajectory_coefficients(p0)
        writer.writerow([tau] + coeff)
    print(f"Coefficients saved to {file_path}")

    def plot(num_points = 100):
        nonlocal c_x, c_y, c_z
        # Generate time values for each interval
        t_vals = np.linspace(0, tau, num_points)

        yaw_vals = yaw_poly(yaw_factor, tau, t_vals)

        x_vals = evaluate_polynomial(c_x, t_vals)
        y_vals = evaluate_polynomial(c_y, t_vals)
        z_vals = evaluate_polynomial(c_z, t_vals)

        # Plot the function
        plt.figure(figsize=(8, 5))

        plt.plot(t_vals, yaw_vals, 'g', label = "Yaw")

        plt.plot(t_vals, x_vals, 'r', label = "x")
        plt.plot(t_vals, y_vals, 'b', label = "y")
        plt.plot(t_vals, z_vals, 'y', label = "z")

        plt.scatter([tau/2], [0], color='black', zorder=3, label=f'tau = {tau/2}')
        plt.scatter([tau], [0], color='black', zorder=3, label=f'T = {tau}')
        
        plt.xlabel('t')
        plt.title('Plot of acceleration, velocity, and yaw')
        plt.legend()
        plt.grid()
        plt.show()

    # plot()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Compute and save trajectory coefficients and plot trajectory.")
    
    parser.add_argument('--p0', type=float, nargs=4, default=[0, 0, 0, 0], help="Start point (x, y, z, yaw)")
    parser.add_argument('--pf', type=float, nargs=4, default=[4, 3, 2, 1], help="End point (x, y, z, yaw)")
    parser.add_argument('--file_path', type=str, default="trajectory_coefficients.csv", help="File path to save coefficients")
    
    args = parser.parse_args()
    
    p0 = np.array(args.p0)
    pf = np.array(args.pf)
    file_path = args.file_path

    tau = 30/16
    c = 30/tau**5
    distance_tau = c*(tau**6)/60
    dist = np.linalg.norm(pf[:3] - p0[:3])
    direction = (pf[:3] - p0[:3]) / dist

    if dist < 2*distance_tau:
        case2(dist, direction, p0, pf)
    else:
        case1(c, tau, dist, distance_tau, p0, pf)

