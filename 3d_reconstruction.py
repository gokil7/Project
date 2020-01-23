import copy
import pyvista as pv
import numpy as np
import open3d as o3d

import matplotlib.pyplot as plt
#from polylidar import extractPlanesAndPolygons, extractPolygons, Delaunator


if __name__ == "__main__":
    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud("gok.ply")
    #print(pcd)
    #print(np.asarray(pcd.points))
    #o3d.visualization.draw_geometries([pcd])

    #print("Downsample the point cloud with a voxel of 0.05")
    downpcd = pcd.voxel_down_sample( voxel_size = 0.05)
    o3d.visualization.Visualizer.create_window(downpcd)
    o3d.visualization.draw_geometries([downpcd])


    print("Recompute the normal of the downsampled point cloud")
    downpcd.estimate_normals(downpcd, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    o3d.visualization.draw_geometries([downpcd])

    print("Print a normal vector of the 0th point")
    print(downpcd.normals[0])
    print("Print the normal vectors of the first 10 points")
    print(np.asarray(downpcd.normals)[:10, :])
    print("")


    xyz = np.asarray(pcd.points)
    print('xyz')
    print(xyz)
    mesh = pv.StructuredGrid()
    # Set the coordinates from the numpy array
    mesh.points = xyz
    # set the dimensions
    mesh.dimensions = [29, 32, 500]
    print(mesh)
    # and then inspect it!
    #mesh.plot(show_edges=True, show_grid=True, cpos="xy")
    #mesh.save('gok.vtk')
