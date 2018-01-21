
import networkx as nx
from math import sqrt, fabs
from sensors.pointsamplecam import PointSampleImage

#@profile
def extract_blobs_closest_points(this_robot, in_image, active_mask):
    """ Extracts blobs from the given image, each represented by the pixel
    closest to the robot. """

    out_image = PointSampleImage(in_image.calib_array, in_image.neighbour_array)

    G = nx.Graph()

    # First add all nodes, where each node consists of an index into
    # calib_array for one of the active pixels.
    for i in range(in_image.n_rows):
        G.add_node(i)

    # We will add edges between neighbouring pixels.  See
    # sensors/pointsamplecam for the definition of neighbouring.
    node_list = G.nodes()
    n = len(node_list)
    for i in range(n):
        if in_image.masks[i] & active_mask != 0:
            (ixi, iyi) = in_image.calib_array[i,0], in_image.calib_array[i,1]
            for j in in_image.neighbour_array[i]:
                if in_image.masks[j] & active_mask != 0:
                    G.add_edge(i, j)

    clusters = nx.connected_component_subgraphs(G, copy=False)
    n_clusters = 0
    for cluster in clusters:
        n_clusters += 1
        # Find the closest pixel to the robot in this cluster. 
        closest_i = None
        closest_distance = float('inf')
        for i in cluster.nodes():
            #(xr, yr) = in_image.calib_array[i,2], in_image.calib_array[i,3]
            #d = sqrt(xr*xr + yr*yr)

            # The pre-computed distance sqrt(xr*xr + yr*yr)
            d = in_image.calib_array[i,5]

            if d < closest_distance:
                closest_i = i
                closest_distance = d
        if closest_i != None:
            out_image.masks[closest_i] = in_image.masks[closest_i]

    return out_image
