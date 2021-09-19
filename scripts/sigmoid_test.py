import math 
import matplotlib.pyplot as plt
import numpy as np

# x_array = np.array([0.0, 0.07,0.08,gener0.15 ,0.16 ,0.17 ,0.20])
x_array=np.linspace(0.00, 0.20, num=100)
x_array=(1/0.20)*x_array
print(x_array)
def sigmoid_2(x,beta):
    y=np.zeros(x.size)
    for i in range(0,x.size):
        y[i]=1/((1+((x[i]/(1-x[i]))**-beta)))
    return y
def sigmoid(x):
    y=np.zeros(x.size)
    for i in range(0,x.size):
        y[i]=1/(1+math.exp(-x[i]))

    return y
y= sigmoid_2(x_array,2)


print(y)

plt.plot(x_array,y)
plt.show()