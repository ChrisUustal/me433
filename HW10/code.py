from ulab import numpy as np  # to get access to ulab numpy functions
import time
# Declare an array with some made up data like
a = np.linspace(0, 2*np.pi, 1024)
b = np.zeros(1024)
for x in range(0, 1024):
    b[x] = np.sin(a[x]*2)
    b[x] += np.sin(a[x]*5)
    b[x] += np.sin(a[x]*13)

c = np.fft.fft(b)

for y in c[0]:
    print("(" + str(y) + ")")
    time.sleep(0.01)

# Want to know all the functions available in numpy? In REPL type np. and press Tab.
