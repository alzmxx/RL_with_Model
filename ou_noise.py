# --------------------------------------
# Ornstein-Uhlenbeck Noise
# Author: Flood Sung
# Date: 2016.5.4
# Reference: https://github.com/rllab/rllab/blob/master/rllab/exploration_strategies/ou_strategy.py
# --------------------------------------

import numpy as np
import numpy.random as nr
import tensorflow as tf

class OUNoise:
    """docstring for OUNois e"""
    def __init__(self,action_dimension,mu=0., theta=0.15, sigma=0.3):
        self.action_dimension = action_dimension
        self.mu = tf.constant(mu)
        self.theta = tf.constant(theta)
        self.sigma = tf.constant(sigma)
        self.state = tf.multiply(tf.ones(self.action_dimension), self.mu)
        self.reset()

    def reset(self):
        self.state =tf.multiply(tf.ones(self.action_dimension), self.mu)

    def noise(self):
        x = self.state
        dx = tf.add(tf.multiply(self.theta, tf.sub(self.mu, x)), tf.multiply(self.sigma, tf.random_normal([tf.size(x)])))
        self.state = tf.add(x, dx)
        return self.state
'''
if __name__ == '__main__':
    ou = OUNoise(3)
    states = []
    for i in range(1000):
        states.append(ou.noise())
    import matplotlib.pyplot as plt

    plt.plot(states)
    plt.show()
'''
