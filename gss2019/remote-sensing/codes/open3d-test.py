from open3d import *
import open3d
import networkx as nx
import numpy as np
from mayavi import mlab
import scipy.cluster.vq as vq
from sklearn.cluster import DBSCAN

#function for outlier removal to reduce noise from point cloud
def display_inlier_outlier(cloud, ind):
    inlier_cloud = select_down_sample(cloud, ind)
    outlier_cloud = select_down_sample(cloud, ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    open3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    return inlier_cloud

def genGraph(p, r_nn):
    N = len(p.points)
    tree = open3d.KDTreeFlann(p)
    pts = np.array(p.points)
    G = nx.Graph()
    for i in range(N): G.add_node(i, pos=pts[i])

    for i in range(N):
        [k, idxs, _] = tree.search_radius_vector_3d(pts[i], r_nn)
        elist = [(i, idx) for idx in idxs]
        G.add_edges_from(elist)
    return G


def drawGraph(p, G):
    graph = open3d.LineSet()
    graph.points = p_light.points
    graph.lines = open3d.Vector2iVector(G.edges)

    open3d.draw_geometries([graph])


if __name__ == "__main__":

    print("Load a ply point cloud, print it, and render it")
    pcd = read_point_cloud("data/ply/Group_4_Floor.ply")
    print(pcd)
    print(np.asarray(pcd.points))
    # draw_geometries([pcd])

    print("Downsample the point cloud with a voxel of 0.05")
    downpcd = voxel_down_sample(pcd, voxel_size = 0.5)
    # draw_geometries([downpcd])

    print("Recompute the normal of the downsampled point cloud")
    estimate_normals(downpcd, search_param = KDTreeSearchParamHybrid(
            radius = 0.1, max_nn = 30))
    # draw_geometries([downpcd])
    #
    # print("Print a normal vector of the 0th point")
    # print(downpcd.normals[0])
    # print("Print the normal vectors of the first 10 points")
    # print(np.asarray(downpcd.normals)[:10,:])
    # print("")

    # print("Load a polygon volume and use it to crop the original point cloud")
    # vol = read_selection_polygon_volume("../../TestData/Crop/cropped.json")
    # chair = vol.crop_point_cloud(pcd)
    # draw_geometries([chair])
    # print("")

    # print("Paint chair")
    # pcd.paint_uniform_color([1, 0.706, 0])
    # draw_geometries([pcd])
    # print("")


######################################################################################

    print("Downsample the point cloud with a voxel of 0.02")
    voxel_down_pcd = voxel_down_sample(pcd, voxel_size = 0.02)
    # draw_geometries([voxel_down_pcd])

    print("Every 5th points are selected")
    uni_down_pcd = uniform_down_sample(pcd, every_k_points = 5)
    # draw_geometries([uni_down_pcd])

    print("Statistical oulier removal")
    cl,ind = statistical_outlier_removal(voxel_down_pcd,
            nb_neighbors=100, std_ratio=0.1)
    inlier_cloud = display_inlier_outlier(voxel_down_pcd, ind)

    # print("Radius oulier removal")
    # cl,ind = radius_outlier_removal(voxel_down_pcd,
    #         nb_points=5, radius=0.05)
    # inlier_cloud = display_inlier_outlier(voxel_down_pcd, ind)

######################################################################################

    # pcd = open3d.read_point_cloud("data/arabette.ply")
    # pcd = read_point_cloud("data/ply/fused-rgba-2000.ply")
    r = .1
    p_light = open3d.voxel_down_sample(inlier_cloud, r)
    G = genGraph(p_light, 3 * r)

    A = nx.adjacency_matrix(G)
    D = np.diag(np.ravel(np.sum(A, axis=1)))
    L = D - A
    ev, U = np.linalg.eigh(L)

    k = 50
    means, labels = vq.kmeans2(U[:, 1:5], k)
    colors = np.random.uniform(0, 1, [k, 3])

    p_light.colors = open3d.Vector3dVector(colors[labels])
    open3d.draw_geometries([p_light])