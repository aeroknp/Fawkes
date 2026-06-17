import matplotlib.pyplot as plt
import pandas as pd
import plotter

# Reading the data from the CSV File
df=pd.read_csv("data/t.csv")

# Creating the arrays
t=df["Time"].to_numpy()
x=df["X"].to_numpy()
y=df["Y"].to_numpy()
z=df["Z"].to_numpy()
vx=df["Vx"].to_numpy()
vy=df["Vy"].to_numpy()
vz=df["Vz"].to_numpy()
phi=df["Phi"].to_numpy()
theta=df["Theta"].to_numpy()
psi=df["Psi"].to_numpy()
p=df["P"].to_numpy()
q=df["Q"].to_numpy()
r=df["R"].to_numpy()
xtarget=df["X_Target"].to_numpy()
ytarget=df["Y_Target"].to_numpy()
ztarget=df["Z_Target"].to_numpy()

# Plotting the graphs
fig=plt.figure(figsize=(12,14))
ax=fig.add_subplot(111,projection="3d")
plotter.plot_3D_intercept(ax,x,y,z,xtarget,ytarget,ztarget)
plt.show()
plotter.animate_3D_intercept(x,y,z,xtarget,ytarget,ztarget,t)