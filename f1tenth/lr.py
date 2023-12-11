'''
import matplotlib.pyplot as plt 
import numpy as np
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from scipy import stats
#import pandas as pd

x=[[-60],[-50],[-40],[-30],[-20],[-10],[0],[10],[20],[30],[40]]
y=[[0],[0],[0],[0],[0],[0],[0],[0.2],[0.8],[1.8],[3.2]]
a=[[5]]
#x=[-1,-1,-1,-1,-1,-1,1,1,1,1,1,1]
#y=[1,2,3,4,5,6,1,2,3,4,5,6]

#model= linear_model.LinearRegression()
#model.fit(x,y)

pre_process= PolynomialFeatures(degree=10)
pr_model= LinearRegression()
pr_model.fit(x,y)
b= pr_model.predict(a)
plt.scatter(x, y)
#plt.scatter(a,b)
plt.plot(x,y)
plt.show()
'''
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
import numpy as np
import matplotlib.pyplot as plt 

# Sample data
X = np.array([1, 2, 3, 4, 5])
y = np.array([2, 3, 5, 7, 9])

# Transform the input data into polynomial features
poly = PolynomialFeatures(degree=2)
X_poly = poly.fit_transform(X.reshape(-1, 1))

# Fit a polynomial regression model
model = LinearRegression()
model.fit(X_poly, y)
plt.plot(X_poly,y)
plt.show()
