from base import *

frame = cv2.imread('tmp/yeah1.png')
frame1 = frame[:720//2, :1280//2, :]
frame2 = frame[720//2:, :1280//2, :]
frame3 = frame[:720//2, 1280//2:, :]
frame4 = frame[720//2:, 1280//2:, :]

# cv2.imshow('yeah', frame)

K       = array([[1108.51, 0, 320],[0, 1108.51, 180],[0, 0, 1]])
K_inv   = inv(K)

h, w    = frame1.shape[:2]

points2D = array( meshgrid( linspace(0,w-1,w), linspace(0,h-1,h), 1) ).reshape(3,-1)

### Transformation matrix ###

t       = array([0,-1,-0.5]).reshape(3,1)
r,p,y   = 40,0,0
r,p,y   = deg2rad(r),deg2rad(p),deg2rad(y)

R_r     = array([[1,      0,      0],
                 [0, cos(r), sin(r)],
                 [0,-sin(r), cos(r)]]) 
R_p     = array([[cos(p), 0,-sin(p)],
                 [0,      1,      0],
                 [sin(p), 0, cos(p)]]) 
R_y     = array([[ cos(y), sin(y),0],
                 [-sin(y), cos(y),0],
                 [0,      0,      1]]) 

R       = R_r.T @ R_p.T @R_y.T

T       = block([[R,t],[0,0,0,1]])

#T = inv(T)

#############################

### Warping ####
frame1_warped = zeros((3000,3000), dtype=uint8)

points2D_warped = K @ ( ( T @ vstack( (K_inv @ points2D, ones(len(points2D[0])))) )[:3] )
points2D_warped = points2D_warped / points2D_warped[2]
print(points2D_warped)
frame1_gray = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)

for i in range(len(points2D[0])):

    u_,v_ = int(points2D_warped[0,i]),int(points2D_warped[1,i])
    u,v = int(points2D[0,i]),int(points2D[1,i])

    if 0 <= u_ < w and 0 <= v_ < h:
        frame1_warped[v_+1500,u_+1500] = frame1_gray[v,u]

cv2.imshow("raww",frame1) 
frame1_warped = cv2.resize(frame1_warped, (1500, 1500))
cv2.imshow("warped",frame1_warped)
cv2.waitKey(0)
cv2.destroyAllWindows()

################


