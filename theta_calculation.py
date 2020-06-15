import numpy as np
import random
import matplotlib.pyplot as plt
from scipy import integrate
from scipy.optimize import fsolve
#import sympy as sp
#import math
#import sympy

# initilizaion
#alpha = 7.84 * 10 ** (-7)
alpha = 3.51 * 10 ** (-7)
d1 = 1000.0
v = 28.0
#v = 18.26
eta = 0.1
#k = 41.0 / 100000.0
k = 32.2 / 100000.0
d2 = 30000.0
w_1 = 25.8 / 3600 # value of time
w_2 = 0.868 	  # oil price
gamma = 0.9

#arrival_rate = 0.1

def reward(sk):
	reward_catch_up = alpha * d1 * v**2 + eta * k * d2 - alpha * d1 * (d1 / (d1 / v - sk))**2
	reward_catch_up = reward_catch_up * w_2 + sk * w_1

	return reward_catch_up

def derivative_reward(sk):

	derivative_reward_catch_up = -2.0 * w_2 * alpha * d1**3 * (1.0 / ((d1 / v - sk) ** 3)) + w_1
	return derivative_reward_catch_up


def theta_c(arrival_rate, initial_value):

	def f1(x):	
		return np.exp(- arrival_rate * (1.0 - gamma) * x) * (derivative_reward(x) - arrival_rate * reward(x))

	def f(x):
		Z = float(x[0])
		theta = float(x[1])
		c = float(x[2])

		function_1 = reward(theta) + gamma * Z - Z

		function_2 = derivative_reward(c) - arrival_rate * reward(c) + arrival_rate * (1.0 - gamma) * (Z + reward(0.0))

		v2, err2 = integrate.quad(f1, c, theta)

		function_3 = np.exp(arrival_rate * (1.0 - gamma) * theta) * (v2 + (Z + reward(0.0)) * np.exp(- arrival_rate * (1.0 - gamma) * c)) - Z

		return [function_1, function_2, function_3]

	result = fsolve(f, initial_value)

	return result
