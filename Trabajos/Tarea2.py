import numpy as np
import scipy.interpolate as spi
import matplotlib.pyplot as plt

# Generamos 10 puntos aleatorios en el plano XY
np.random.seed(5)  # Para poder reproducir los mismos puntos, fijamos la semilla
num_points = 10
xarr = np.random.rand(num_points) * 10  # Coordenadas X aleatorias
yarr = np.random.rand(num_points) * 10  # Coordenadas Y aleatorias
tarr = np.linspace(0, 9, num_points)  # Tiempos asociados a los puntos

# Calculamos una trayectoria que pase por todos los puntos
tnew = np.linspace(0, 9, 200)  # 200 instantes de tiempo para la trayectoria
xc = spi.splrep(tarr, xarr, s=0)  # Spline para X
yc = spi.splrep(tarr, yarr, s=0)  # Spline para Y

# Trayectoria
xnew = spi.splev(tnew, xc, der=0)  # Coordenadas X de la trayectoria
ynew = spi.splev(tnew, yc, der=0)  # Coordenadas Y de la trayectoria

# Visualización de la trayectoria
plt.figure(figsize=(10, 5))

# Trayectoria suave
plt.subplot(1, 2, 1)
plt.plot(xnew, ynew, label='Trayectoria suave')
plt.plot(xarr, yarr, 'o', label='Puntos aleatorios')
plt.title('Trayectoria en el plano XY')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()

# Velocidad en cada eje a lo largo del tiempo
plt.subplot(1, 2, 2)
xdot = spi.splev(tnew, xc, der=1)  # Velocidad en X
ydot = spi.splev(tnew, yc, der=1)  # Velocidad en Y
plt.plot(tnew, xdot, label='Velocidad en X')
plt.plot(tnew, ydot, label='Velocidad en Y')
plt.title('Velocidad en función del tiempo')
plt.xlabel('Tiempo')
plt.ylabel('Velocidad')
plt.legend()

plt.tight_layout()
plt.show()
