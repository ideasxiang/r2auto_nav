# Python program explaining
# numpy.size() method

# importing numpy
import numpy as np

# Making a random array
arr = np.array([[1, 2, 3, 4], [5, 6, 7, 8]])

# count the number of elements along the axis.
# Here rows and colums are being treated
# as elements

#gives no. of rows along x-axis
print(np.size(arr, 0))

#gives no. of columns along y-axis
print(np.size(arr, 1))
