import cvxpy as cp
import numpy as np
from scipy.linalg import null_space
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
    for n_clusters in range(2, 10):  # on cherche entre 2 et 10 clusters
        kmeans_list.append(KMeans(n_clusters=n_clusters, random_state=0))
        silhouette_list.append(
            silhouette_score(total_computed_clusters, kmeans_list[-1].fit_predict(total_computed_clusters)))

    K_opti = np.argmax(silhouette_list)
    return kmeans_list[K_opti].cluster_centers_


def compute_orthonomal_basis_with_horizontal_u(w):  # w but be normalised
    if (w[0] == 0):
        u = np.array([1, 0, 0])
        v = np.cross(w, u)
    elif (w[1] == 0):
        u = np.array([0, 1, 0])
        v = np.cross(w, u)
    else:
        u = np.array(
            [w[1] / w[0] * np.sqrt(1 / (1 + w[1] ** 2 / w[0] ** 2)), -np.sqrt(1 / (1 + w[1] ** 2 / w[0] ** 2)), 0])
        v = np.cross(w, u)
    base = np.array([u, v, w])
    return base


def compute_direction(x_img, y_img, Cam_pos, Cam_dir):  # l'origine de l'image est en haut a gauche
    # rendu projectif inverse doc : https://towardsdatascience.com/inverse-projection-transformation-c866ccedef1c
    hfov = np.pi / 2
    res_x = 320.0
    res_y = 240.0
    s = res_x / res_y
    f = res_x / (np.tan(hfov / 2) * 2)  # distance focal en pixel
    K = np.array([[f, 0, res_x / 2], [0, f, res_y / 2], [0, 0, 1]])
    T = np.zeros((4, 4))
    T[0:3, 0:3] = np.identity(3)
    T[0:3, 3] = -Cam_pos
    R = np.identity(4)
    R[0:3, 0:3] = compute_orthonomal_basis_with_horizontal_u(Cam_dir / np.linalg.norm(Cam_dir))
    P = np.zeros((3, 4))
    P[0:3, 0:3] = np.identity(3)

    PI = np.dot(K, np.dot(P, np.dot(R, T)))  # mef au ratio s    voir doc
    PIinv = np.linalg.pinv(PI)
    ns = null_space(PI)
    x = np.array([x_img, y_img, 1])
    X = np.dot(PIinv, x)
    return (np.concatenate((X[0:3] / np.linalg.norm(X[0:3]), Cam_pos), axis=0))





'''
Cam_pos=np.array([1,2,3])
T = np.zeros((4,4))
T[0:4, 0:4] = np.identity(4)
T[0:3, 3] = -Cam_pos

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

'''

# test sur un dataset 2D, on detecte les 15 clusters a detecter
'''
clusters=[]
for line in open('s1.txt','r'):
    print(line.rstrip('\n'))
    fields = line.rstrip('\n').split('    ')
    clusters.append([0,int(fields[1]),int(fields[2])])

print(clusters)
print(len(K_clustering_total(clusters)))
'''
