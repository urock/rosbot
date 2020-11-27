import matplotlib.pyplot as plt
import numpy as np

x = np.arange(0,2*np.pi,0.1)   # start,stop,step

# y = np.sin(x)*np.cos(x)

y = np.tanh(x)

for i in range(len(x)):
    print(x[i], y[i])

plt.plot(x,y)
plt.show()

