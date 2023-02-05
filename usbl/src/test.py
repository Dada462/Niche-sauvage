import numpy as np
import pyproj
# a=np.array([[1,2,3],[4,5,6],[0,1,0],[10,0,12]])
# print(a[(a[:,2]!=0)])

# Define a projection with Proj4 notation, in this case an Icelandic grid 
isn2004=pyproj.CRS("+proj=lcc +lat_1=64.25 +lat_2=65.75 +lat_0=65 +lon_0=-19 +x_0=1700000 +y_0=300000 +no_defs +a=6378137 +rf=298.257222101 +to_meter=1") 
# Define some common projections using EPSG codes
wgs84=pyproj.CRS("EPSG:4326") # LatLon with WGS84 datum used by GPS units and Google Earth 
lam93=pyproj.CRS("EPSG:2154") # UK Ordnance Survey, 1936 datum 
UTM26N=pyproj.CRS("EPSG:32626") # UTM coords, zone 26N, WGS84 datum 
UTM27N=pyproj.CRS("EPSG:32627") # UTM coords, zone 27N, WGS84 datum 
UTM28N=pyproj.CRS("EPSG:32628") # ... you get the picture 
X,Y = pyproj.transform(wgs84, lam93, )
print(X,Y)