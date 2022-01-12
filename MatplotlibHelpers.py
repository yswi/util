# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt


### DIFFERNET DYPES OF PLOTS ###

def plot_3d_scatter(xdata, ydata, zdata):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    
    # Data for three-dimensional scattered points
    ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens');

    ax.set_xlim(-1, 1); ax.set_ylim(-1, 1); ax.set_zlim(-1, 1);

  
def plot_3d_line(xdata, ydata, zdata):
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    # Data for a three-dimensional line
    zline = np.linspace(0, 15, 1000)
    xline = np.sin(zline)
    yline = np.cos(zline)
    ax.plot3D(xline, yline, zline, 'gray')
    
def subplot_line():
    plt.figure(figsize = (10,20))
    plt.subplot(221)

    # equivalent but more general
    ax1 = plt.subplot(2, 2, 1)
    ax1.plot*

    # add a subplot with no frame
    ax2 = plt.subplot(222, frameon=False)
    
    
    
### DIFFERENT PLOT SETTINGS ###
def integer_xtick_grid(data):
    plt.plot(data)
    plt.grid(True) # grid on
    plt.xticks(np.arange(0, data.shape[0], step=1)) # Integer xticks
    plt.show()
