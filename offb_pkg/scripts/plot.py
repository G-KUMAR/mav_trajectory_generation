import numpy as np
import matplotlib.pyplot as plt



data = np.load('s_des2.npy')

r = data.shape[0]

t = [] 
angle = []

for i in xrange(0,r):
	t.append(i)

for i in xrange(0,r):
	angle.append(data[i,7])

plt.plot(t,angle)
plt.show()