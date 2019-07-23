from open3d import *
import open3d


if __name__ == "__main__":

    print("Load a ply point cloud, print it, and render it")
    pcd = read_point_cloud("cloud.ply")

    open3d.draw_geometries([pcd])
