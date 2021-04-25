import numpy as np
import cv2
import time
from matplotlib import pyplot as plt
from draw_matches import draw_matches

#import saveImageBaxter
from saveImageBaxter2 import saveBaxterImage

#detemrine how many matches before we consider the match 'good' (40 seems about right for the text/icons for the objects)
MIN_MATCH_COUNT = 20
k = 0						# iterator for the 'for' loop

# Create list of the types of Keurig Cups Possible
img = ['Assets/Breakfast_Blend_Green_Mountain.png', 'Assets/Dark_Italian_Roast.png', 'Assets/Hot_Cocoa_Swiss_Miss.png', 'Assets/Colombian_Peaks.png', 'Assets/Donut_Shop.png', 'Assets/Newmans_Special_Blend.png', 'Assets/Hazelnut_Coffee.png']
#img1 = cv2.imread('box.png',0)          # queryImage

# Image to search for the objects in
#img2 = cv2.imread('Assets/Cups2.png',0)

# Read in the saved K-cups image
saveBaxterImage()
# Accessing file may take at least 1-2 seconds
time.sleep(2)
img2 = cv2.imread('BaxterCamPicture/imageCapture.png',0)

# Get user input for what to look for
imgtype = raw_input("Please enter coffee type: ")

# ISSUE: Unsure if the light glare will be an issue

# Search Function for finding place of what coffee type
def index_containing_substring(img, imgtype):
	for i, s in enumerate(img):
		if imgtype in s:
			return i
	return -1

#USEFUL SEARCH TOOLS
#filter(lambda x: imgtype in x, img)
#indices = [i for i, s in enumerate(img) if imgtype in s]
#print indices

i = index_containing_substring(img, imgtype)

img1 = cv2.imread(img[i],0)

# Initiate SIFT detector
sift = cv2.SIFT()
	
# Read image from list of images that contain the objects we want to search for
#img1 = cv2.imread(img[k],0)
	
# resize images, as they are likely to be too big (If we are doing specific detail in object, likely wont need resize for img1)
#img1 = cv2.resize(img1, (683, 545)) 					# resolution for 1280x800 version of picture for just icon/screen part
#img2 = cv2.resize(img2, (1024, 720)) 
	
# iterate
#k = k + 1
	
# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(img1,None)
kp2, des2 = sift.detectAndCompute(img2,None)

FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)

flann = cv2.FlannBasedMatcher(index_params, search_params)

matches = flann.knnMatch(des1,des2,k=2)

good=[]
# store all the good matches as per Lowe's ratio test. The lower the #*n.distance, the stricter the 'good' match has to be
for m,n in matches:
	if m.distance < 0.7*n.distance:
		good.append(m)

# Right now I have it so if enough good matches, draw rectangle around the matched item
# in the scene, (findHomography) when using Baxter, make it so if good matches, it found
# the state of coffee machine and does the next action required
if len(good)>MIN_MATCH_COUNT:
	print "Match found - %s: %d/%d" % (img[k-1], len(good), MIN_MATCH_COUNT)
		
	src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
	dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

	M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
	matchesMask = mask.ravel().tolist()

	h,w = img1.shape
	pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
	dst = cv2.perspectiveTransform(pts,M)
		
	# cv2.polylines returns nothing (opencv tutorial is wrong, it states to put:
	# img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.CV_AA)
	# But cv2.polylines returns nothing, making the image NULL, so JUST put:
	cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.CV_AA)
	#cv2.line(img2,(0,0),(511,511),(255,0,0),5)

else:
	print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
	matchesMask = None
	
#draw_params = dict(matchColor = (0,255,0), # draw matches in green color
				  # singlePointColor = None,
				   #matchesMask = matchesMask, # draw only inliers
				   #flags = 2)

#img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)

#plt.imshow(img2, 'gray'),plt.show()
	
#draw the macthes for the current found object
draw_matches(img1, kp1, img2, kp2, good)


# Show final result of the found objects in the final picture
#img2 = cv2.resize(img2, (1600, 900)) 

cv2.imshow("Image", img2)

# Slect the image (NOT TERMINAL) and press q to exit

