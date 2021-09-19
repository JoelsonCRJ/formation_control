import numpy as np
import matplotlib.pyplot as plt

def norm(x):
    # normalise x to range [-1,1]
    nom = (x - x.min()) * 2.0
    denom = x.max() - x.min()
    return  nom/denom - 1.0

def sigmoid(x, k=0.1):
    # sigmoid function
    # use k to adjust the slope
    s = 1 / (1 + np.exp(-x / k)) 
    return s

# un-normalised data
x = np.linspace(0.0,0.2,100)
# normalise the data
x = norm(x) 

plt.plot(x, sigmoid(x))
plt.show()