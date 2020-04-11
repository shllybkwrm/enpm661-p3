# ENPM661 Project 3 Phase 4
# Shelly Bagchi & Omololu Makinde

import math
import time
import numpy as np
import matplotlib.pyplot as plt


path = np.array( [[-4000, -3000], [-3899.902941083189, -2977.26473388777], [-3797.887325943388, -2965.9012926555806], [-3697.790267026577, -2943.1660265433507], [-3595.774651886776, -2931.8025853111612], [-3441.014229540926, -2777.042162965311], [-3286.2538071950758, -2622.281740619461], [-3186.1567482782643, -2599.546474507231], [-3084.141133138463, -2588.183033275042], [-2929.3807107926127, -2433.4226109291917], [-2774.6202884467625, -2278.6621885833415], [-2619.8598661009123, -2123.9017662374913], [-2465.099443755062, -1969.141343891641], [-2310.339021409212, -1814.380921545791], [-2210.2419624924005, -1791.6456554335612], [-2013.27330324077, -1887.065380448603], [-1911.2576881009684, -1875.701939216414], [-1811.1606291841567, -1852.9666731041846], [-1614.1919699325242, -1948.3863981192264], [-1417.2233106808917, -2043.8061231342683], [-1220.2546514292592, -2139.22584814931], [-1023.2859921776267, -2234.645573164352], [-826.3173329259942, -2330.0652981793937], [-629.3486736743616, -2425.4850231944356], [-432.3800144227294, -2520.9047482094775], [-378.06290568521183, -2608.0021949423535], [-333.85872542021366, -2700.6429201990622]] )

#plt.scatter(path[:,0], path[:,1])
#plt.show()

RPM_L=50
RPM_R=50

for point in path:
    r=76//2
    L=354//2
    t=0.0
    dt=0.1  # e.g. 10 steps
    Xn=coord[0]
    Yn=coord[1]
    Thetan=np.deg2rad(thetaIn)

    ### Convert RPM to rad/sec
    UL = action[0]*(1/60)*2*np.pi
    UR = action[1]*(1/60)*2*np.pi

# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes

    while t<1:
        t=t+dt
        Xs = Xn
        Ys = Yn

        # Note:  These are using the rotational velocities of the wheels in rad/sec
        Xn += 0.5 * r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5 * r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt

        # Check collisions at every point on path
        if inside_obstacle([[Xn,Yn]]):
            #print(">> Collides with obstacle!")
            #return [],[]
            # Make sure this new point isn't included (roll back changes)
            Xn = Xs
            Yn = Ys
            break

        # Plot here to get a curve rather than vector
        #ax.quiver(Xs, Ys, Xn-Xs, Yn-Ys, units='xy', scale=1, color='k', width=1, headwidth=1, headlength=0)
        ax.plot([Xs, Xn], [Ys, Yn], linewidth=0.25, color='k')

    ThetaDeg=np.rad2deg(Thetan) % 360