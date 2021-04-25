import numpy as np
import cv2
from matplotlib import pyplot as plt
from draw_matches import draw_matches

def recognizeImage(imgs, img2, matchCount):
	#detemrine how many matches before we consider the match 'good' (40 seems about right for the text/icons for the objects)
	MIN_MATCH_COUNT = matchCount
	k = 0						# iterator for the 'for' loop

	# NOTE: Fing just "pieces' of objects,when searching for VERY similar objects works best, FIND WHAT
	# IS DIFFERENT ABOUT THE OBJECTS AND SEARCH FOR THAT (Only if the objects we are searching 
	# for are similar, like in our Keurig screen example)

	# Create list of objects we are searching for in the image
	#img1 = ['Assets/beginclose1.png', 'Assets/brewingclose2.png', 'Assets/doneclose2.png', 'Assets/fillingclose1.png', 'Assets/preheatclose1.png', 'Assets/poweroffclose2.png', 'Assets/readyclose2.png', 'Assets/waterclose1.png', 'Assets/welcomeclose2.png']

	# Image to search for the objects in
	img2 = cv2.imread(img2, 0)

	# ISSUE: 3 of the states use the kueig logo, matching cant tell difference...

	for i in imgs:
		# Initiate SIFT detector
		sift = cv2.SIFT()
		
		# Read image from list of images that contain the objects we want to search for
		img1 = cv2.imread(imgs[k],0)
		
		# resize images, as they are likely to be too big (If we are doing specific detail in object, likely wont need resize for img1)
		#img1 = cv2.resize(img1, (683, 545)) 					# resolution for 1280x800 version of picture for just icon/screen part
		#img2 = cv2.resize(img2, (1280, 720)) 
		
		# iterate
		k = k + 1
		
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
			if m.distance < 0.65*n.distance:
				good.append(m)

		# Right now I have it so if enough good matches, draw rectangle around the matched item
		# in the scene, (findHomography) when using Baxter, make it so if good matches, it found
		# the state of coffee machine return 1 or return nothing otherwise
		if len(good)>MIN_MATCH_COUNT:
			print "\tMatch found - %s: %d/%d" % (imgs[k-1], len(good), MIN_MATCH_COUNT)
			
			src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
			dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

			M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
			matchesMask = mask.ravel().tolist()

			h,w = img1.shape
			pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
			dst = cv2.perspectiveTransform(pts,M)
			
			cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.CV_AA)
			#draw_matches(img1, kp1, img2, kp2, good)
			return 1

		else:
			print "\tNot enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
			matchesMask = None
			#draw_matches(img1, kp1, img2, kp2, good)
			
		#draw_matches(img1, kp1, img2, kp2, good)

	#cv2.imshow("Image", img2)

	# Slect the image (NOT TERMINAL) and press q to exit
	#while True:
		#if cv2.waitKey(1) & 0xFF == ord('q'):
			#break
