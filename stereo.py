import math
import random
import numpy as np
import numpy.linalg as la
import scipy.ndimage as ndi
from scipy import signal

'''
 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 !!! NE MODIFIEZ PAS LE CODE EN DEHORS DES BLOCS TODO. !!!
 !!!  L'EVALUATEUR AUTOMATIQUE SERA TRES MECHANT AVEC  !!!
 !!!            VOUS SI VOUS LE FAITES !               !!!
 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
'''

# transformation coordonnées cartésiennes -> homogènes
pad = lambda x: np.hstack([x, np.ones((x.shape[0], 1))])

# suppression de la dernière coordonnée
unpad = lambda x: x[:,:-1]


def eight_points(pts1, pts2):
    """
    TODO4.1
       Eight Point Algorithm
       [I] pts1, points in image 1 (Nx2 matrix)
           pts2, points in image 2 (Nx2 matrix)           
       [O] F, the fundamental matrix (3x3 matrix)
    """    
    
    assert (pts1.shape[0] == pts2.shape[0]),\
        'Nombre différent de points en pts1 et pts2'
    
    F = None    
    
    # TODO-BLOC-DEBUT    
    # Normalisation des points
    def normalize(pts):
        centroid = np.mean(pts, axis=0)
        pts_centered = pts - centroid
        # Colleague's method
        ecart_type = np.sqrt(np.mean(np.sum(pts_centered**2, axis=1)))
        scale = np.sqrt(2) / ecart_type
        T = np.array([
            [scale, 0, -scale * centroid[0]],
            [0, scale, -scale * centroid[1]],
            [0,     0,                  1]
        ])
        pts_h = np.column_stack((pts, np.ones(pts.shape[0])))  # Homogeneous coordinates
        pts_norm = (T @ pts_h.T).T
        return pts_norm[:, :2], T  # Return only x,y coordinates and transformation matrix

    pts1_norm, T1 = normalize(pts1)
    pts2_norm, T2 = normalize(pts2)

    # Construction de la matrice A pour A*f = 0
    N = pts1.shape[0]
    A = np.zeros((N, 9))
    for i in range(N):
        x1, y1 = pts1_norm[i, 0], pts1_norm[i, 1]
        x2, y2 = pts2_norm[i, 0], pts2_norm[i, 1]
        A[i] = [x2*x1, x2*y1, x2, 
                y2*x1, y2*y1, y2, 
                x1,    y1,    1]

    # Résolution de Af = 0 par SVD
    U, S, Vt = np.linalg.svd(A)
    F_norm = Vt[-1].reshape(3, 3)

    # Forçage du rang 2 de F (annuler la plus petite valeur singulière)
    U_f, S_f, Vt_f = np.linalg.svd(F_norm)
    S_f[-1] = 0
    F_rank2 = U_f @ np.diag(S_f) @ Vt_f

    # Dénormalisation
    F = T2.T @ F_rank2 @ T1

    # TODO-BLOC-FIN

    return F


def ransac(keypoints1, keypoints2, matches, n_iters=500, threshold=1e-4):
    """
    TODO4.2
       RANSAC pour trouver une transformation projective robuste

       [I] keypoints1,  tableau M1 x 2, chaque rangée contient les coordonnées 
                        d'un point-clé (x_i,y_i) dans image1
           keypoints2,  tableau M2 x 2, chaque rangée contient les coordonnées 
                        d'un point-clé (x'_i,y'_i) dans image2
           matches,     tableau N x 2, chaque rangée représente une correspondance
                        [indice_dans_keypoints1, indice_dans_keypoints2]
           n_iters,     le nombre d'itérations à effectuer pour RANSAC
           threshold,   le seuil pour sélectionner des bonnes correspondances
       [O] F,           une estimation robuste de la matrice Fondamentale F
           goodmatches, tableau max_inliers x 2 contenant les indices des bonnes correspondances 
    """
    
    # Matrice Fondamentale
    F = None
    
    #indices des bonnes correspondances
    goodmatches = None
    
    # Initialisation du générateur de nombres aléatoires
    # fixé le seed pour pouvoir comparer le résultat retourné par 
    # cette fonction par rapport à la solution référence
    random.seed(131)
    
    # TODO-BLOC-DEBUT    
  
    pass
    # TODO-BLOC-FIN
                
    return F, goodmatches
        

def epipolar_match(im1, im2, F, pts1, W = 7):
    """
    TODO4.3
       Compute keypoints correspondences using Epipolar geometry
       [I] im1, image 1 (H1xW1x3 matrix)
           im2, image 2 (H2xW2x3 matrix)
           F, fundamental matrix from image 1 to image 2 (3x3 matrix)
           pts1, points in image 1 (Nx2 matrix)
       [O] pts2, points in image 2 (Nx2 matrix)
    """
    
    assert len(im1.shape) == 3 and len(im2.shape) == 3, '...'
    
    pts2 = None
    
    # TODO-BLOC-DEBUT    
    pass
    # TODO-BLOC-FIN
    
    return pts2


def estimate_camera_pose( F, principal_point, focal_distance, base_distance ):
    """
    TODO4.4
       Estimate the four possible camera poses
       [I] F, fundamental matrix from image 1 to image 2 (3x3 matrix)
           principal_point, camera's principal point coordinates (1x2 tuple)
           focal_distance, camera's x and y focal lengths (1x2 tuple)
           base_distance, distance betwenn the origins of the cameras (scalar)                      
       [O] K, camera's intrinsic parameters (3x3 matrix)
           Rt_list, camera's extrinsic parameters [R|t] (list of four 4x3 matrices)  
    """
    
    K       = None
    Rt_list = None

    # TODO-BLOC-DEBUT    
    pass
    # TODO-BLOC-FIN

    return K, Rt_list

    
def triangulate(P1, pts1, P2, pts2):
    """
    TODO4.5-1
       Triangulation
       [I] P1, camera projection matrix 1 (3x4 matrix)
           pts1, points in image 1 (Nx2 matrix)
           P2, camera projection matrix 2 (3x4 matrix)
           pts2, points in image 2 (Nx2 matrix)
       [O] pts3d, 3D points in space (Nx3 matrix)
    """    
    
    pts3d = None
    
    # TODO-BLOC-DEBUT    
    # Initialize pts3d as a numpy array with appropriate shape
    pts3d = np.zeros((len(pts1), 3))
    
    for i in range(len(pts1)):
        x, y = pts1[i]
        xp, yp = pts2[i]
        
        # Build the D matrix for SVD
        D = np.array([
            x * P1[2] - P1[0],
            y * P1[2] - P1[1],
            xp * P2[2] - P2[0],
            yp * P2[2] - P2[1]
        ])
        
        # Solve using SVD
        _, _, Vt = np.linalg.svd(D)
        X = Vt[-1]  # Last row of Vt
        
        # Convert from homogeneous to cartesian coordinates
        X = X / X[3]
        pts3d[i] = X[:3]
    # TODO-BLOC-FIN
    
    return pts3d


def check_chirality(K, Rt_list, pts1, pts2):
    """
    TODO4.5-2
       Chirality check
       [I] K, camera intrinsic matrix (3x3 matrix)
           Rt_list, camera's extrinsic parameters [R|t] (list of four 4x3 matrices)             
           pts1, points in image 1 (Nx2 matrix)
           pts2, points in image 2 (Nx2 matrix)
       [O] Rt, correct camera's extrinsic parameters [R|t] (4x3 matrices)             
           pts3d_list, 3D points in space (list of four Nx3 matrix)
    """    

    Rt = None
    pts3d_list  = None
    
    # TODO-BLOC-DEBUT    
    # Initialize variables
    Rt = None
    pts3d_list = []  # Initialize as empty list
    max_in_front = -1  # Initialize counter
    
    # Create projection matrix for first camera [I|0]
    P1 = K @ np.hstack((np.eye(3), np.zeros((3, 1))))
    
    for Rt_candidate in Rt_list:
        # Create projection matrix for second camera
        P2 = K @ Rt_candidate
        
        # Triangulate points
        pts3d = triangulate(P1, pts1, P2, pts2)
        pts3d_list.append(pts3d)
        
        # Convert to homogeneous coordinates
        X = np.hstack((pts3d, np.ones((len(pts3d), 1))))
        
        # Check chirality condition for first camera (all points should have Z > 0)
        in_front_cam1 = (X[:, 2] > 0)
        
        # Check chirality condition for second camera
        R = Rt_candidate[:, :3]
        t = Rt_candidate[:, 3]
        # Transform points to second camera's coordinate system
        X_cam2 = (R @ X[:, :3].T).T + t
        in_front_cam2 = (X_cam2[:, 2] > 0)
        
        # Count points in front of both cameras
        total_in_front = np.sum(in_front_cam1 & in_front_cam2)
        
        # Update best solution
        if total_in_front > max_in_front:
            max_in_front = total_in_front
            Rt = Rt_candidate
    # TODO-BLOC-FIN

    return Rt, pts3d_list

def compute_matching_homographies(F, principal_point, pts1, pts2):
    """
    TODO5
       Compute matching homography matrices     
       [I] F, fundamental matrix from image 1 to image 2 (3x3 matrix)
           principal_point, camera's principal point coordinates (1x2 tuple)
           pts1, points in image 1 (Nx2 matrix)
           pts2, points in image 2 (Nx2 matrix)
       [O] H1, homography transformation matrix for the first image (3x3 matrix)             
           H2, homography transformation matrix for the second image (3x3 matrix)             
    """    

    H1 = None
    H2 = None
    
    # TODO-BLOC-DEBUT    
    pass
    # TODO-BLOC-FIN

    return H1, H2


def compute_disparity(im1, im2, max_disparity, win_size):
    """
    TODO6.1
       Calcul de la carte de disparité
       [I] im1, rectified image 1 (HxWx3 matrix)
           im2, rectified image 2 (HxWx3 matrix)           
           max_disparity, maximum disparity to check (scalar)
           win_size, windows size for block matching (scalar > 0)
       [O] disp, disparity map associated with im1 (HxW matrix)
    """    
    assert im1.shape[0] == im2.shape[0] and \
           im1.shape[1] == im2.shape[1], 'les images doivent avoir des dimensions identiques'
    
    assert 0 < max_disparity and max_disparity < im2.shape[1], 'max_disparity < im1.shape[1]'
    
    disp = None   

    # TODO-BLOC-DEBUT     
    pass
    # TODO-BLOC-FIN
        
    return disp

       
def cross_disparity(im1, im2, disp1, max_disparity, win_size):
    """
    TODO6.2
       Validation de la carte de disparité
       [I] im1, rectified image 1 (HxWx3 matrix)
           im2, rectified image 2 (HxWx3 matrix)           
           disp1, left disparity matrix (HxW matrix)
           max_disparity, maximum disparity to check (scalar)
           win_size, windows size for block matching (scalar > 0)
       [O] disp2, disparity map associated with im2 (HxW matrix)
           dispc, coherent disparity map for im1 (HxW)
    """    
    assert im1.shape[0] == im2.shape[0] and \
           im1.shape[1] == im2.shape[1], 'les images doivent avoir des dimensions identiques'
    
    assert 0 < max_disparity and max_disparity < im2.shape[1], 'max_disparity < im1.shape[1]'
    
    disp2 = None
    dispc = None

    # TODO-BLOC-DEBUT     
    pass
    # TODO-BLOC-FIN
    
    return disp2, dispc


def fill_holes(dispc):    
    """
    TODO6.3
       Disparity holes filling 
       [I] dispc, coherent disparity map with holes (negative values) (HxW)
       [O] dispf, filled disparity map (HxW)
    """    

    dispf = None
        
    # TODO-BLOC-DEBUT     
    pass
    # TODO-BLOC-FIN

    return dispf

