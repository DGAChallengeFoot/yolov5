import cvxpy as cp
import numpy as np
import scipy as scp
from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_samples, silhouette_score
import scipy


def get_median(x1, x2):
    v1 = x1[0, 0:3] / np.linalg.norm(x1[:, 0:3])
    v2 = x2[0, 0:3] / np.linalg.norm(x2[:, 0:3])
    X1 = np.asarray(x1[:, 3:6]).reshape(-1)
    X2 = np.asarray(x2[:, 3:6]).reshape(-1)
    alpha = np.asarray(np.cross(v1, (X2 - X1))).reshape(-1)
    beta = np.asarray(np.cross(v1, v2)).reshape(-1)
    lambd = -np.dot(alpha, beta) / np.linalg.norm(beta) ** 2
    Xl2 = X2 + lambd * v2
    Xl1 = X1 + np.dot(v1, Xl2 - X1) * v1
    return (Xl2 + Xl1) / 2


def k_clustering_drone(directions):  # directions is a numpy Matrix
    points = []
    for i in range(0, directions.shape[0] - 1):
        for j in range(i + 1, directions.shape[0]):
            points.append(
                get_median(np.expand_dims(directions[i, :], axis=0), np.expand_dims(directions[j, :], axis=0)))

    kmeans_list = []
    silhouette_list = []
    for n_clusters in range(2, 6):  # on cherche entre 2 et 5 clusters   pas tres grave si un humain est repere 2 fois
        kmeans_list.append(KMeans(n_clusters=n_clusters, random_state=0))
        # The silhouette_score gives the average value for all the samples.
        # This gives a perspective into the density and separation of the formed
        # clusters
        silhouette_list.append(silhouette_score(points, kmeans_list[-1].fit_predict(points)))
        # print(KMeans(n_clusters=n_clusters, random_state=0).fit(points).inertia_)

    K_opti = np.argmax(silhouette_list)
    return kmeans_list[K_opti].cluster_centers_


def K_clustering_total(total_computed_clusters):  # directions is a numpy Matrix
    kmeans_list = []
    silhouette_list = []
    for n_clusters in range(2, 18):  # on cherche entre 2 et 18 clusters
        kmeans_list.append(KMeans(n_clusters=n_clusters, random_state=0))
        silhouette_list.append(
            silhouette_score(total_computed_clusters, kmeans_list[-1].fit_predict(total_computed_clusters)))

    K_opti = np.argmax(silhouette_list)
    return kmeans_list[K_opti].cluster_centers_


directions = np.random.rand(10, 6)
# k_clustering_drone(directions,2)
x1 = np.array([[1, 0, 0, 0, 1, 0]])
x2 = np.array([[0, 1, 0, 1, 0, 0]])

x = np.random.rand(1, 3)
A = np.concatenate((x1[:, 0:3], x2[:, 0:3]), axis=0)
b = np.concatenate((x1[:, 3:6], x2[:, 3:6]), axis=0)
A = A / np.linalg.norm(A, axis=1, keepdims=True)

xm = get_median(x1, x2)
print(xm)
clusters = k_clustering_drone(directions)
print(clusters)

#test sur un dataset 2D, on detecte les 15 clusters a detecter
'''
clusters=[]
for line in open('s1.txt','r'):
    print(line.rstrip('\n'))
    fields = line.rstrip('\n').split('    ')
    clusters.append([0,int(fields[1]),int(fields[2])])

print(clusters)
print(len(K_clustering_total(clusters)))
'''
