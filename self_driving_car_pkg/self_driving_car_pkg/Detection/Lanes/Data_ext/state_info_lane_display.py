import cv2
import numpy as np
from ....config import config
from ..utilities import Cord_Sort,findlaneCurvature

def EstimateNonMidMask(MidEdgeROi):
	Mid_Hull_Mask = np.zeros((MidEdgeROi.shape[0], MidEdgeROi.shape[1], 1), dtype=np.uint8)
	contours = cv2.findContours(MidEdgeROi,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]
	if contours: 
		hull_list = []
		contours = np.concatenate(contours)
		hull = cv2.convexHull(contours)
		hull_list.append(hull)
		Mid_Hull_Mask = cv2.drawContours(Mid_Hull_Mask, hull_list, 0, 255,-1)

	Non_Mid_Mask=cv2.bitwise_not(Mid_Hull_Mask)
	return Non_Mid_Mask

def LanePoints(MidLane,OuterLane,Offset_correction):

	Mid_cnts = cv2.findContours(MidLane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
	Outer_cnts = cv2.findContours(OuterLane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

	if(Mid_cnts and Outer_cnts):
		Mid_cnts_Rowsorted = Cord_Sort(Mid_cnts,"rows")
		Outer_cnts_Rowsorted = Cord_Sort(Outer_cnts,"rows")
		#print(Mid_cnts_Rowsorted)
		Mid_Rows = Mid_cnts_Rowsorted.shape[0]
		Outer_Rows = Outer_cnts_Rowsorted.shape[0]

		Mid_lowP = Mid_cnts_Rowsorted[Mid_Rows-1,:]
		Mid_highP = Mid_cnts_Rowsorted[0,:]
		Outer_lowP = Outer_cnts_Rowsorted[Outer_Rows-1,:]
		Outer_highP = Outer_cnts_Rowsorted[0,:]

		LanePoint_lower = ( int( (Mid_lowP[0] + Outer_lowP[0]  ) / 2 ) + Offset_correction, int( (Mid_lowP[1]  + Outer_lowP[1] ) / 2 ) )
		LanePoint_top   = ( int( (Mid_highP[0] + Outer_highP[0]) / 2 ) + Offset_correction, int( (Mid_highP[1] + Outer_highP[1]) / 2 ) )

		return LanePoint_lower,LanePoint_top
	else:
		return (0,0),(0,0)

def FetchInfoAndDisplay(Mid_lane_edge,Mid_lane,Outer_Lane,frame,Offset_correction):
	
	Traj_lowP,Traj_upP = LanePoints(Mid_lane,Outer_Lane,Offset_correction)

	PerpDist_LaneCentralStart_CarNose= -1000
	if(Traj_lowP!=(0,0)):
		PerpDist_LaneCentralStart_CarNose = Traj_lowP[0] - int(Mid_lane.shape[1]/2)
	curvature = findlaneCurvature(Traj_lowP[0],Traj_lowP[1],Traj_upP[0],Traj_upP[1])

	if config.Testing:
		# if (config.debugging and config.debugging_Lane and config.debugging_L_LaneInfoExtraction):
		# 	cv2.imshow("[FetchInfoAndDisplay] Mid_lane_edge",Mid_lane_edge)
		# 	cv2.imshow("[FetchInfoAndDisplay] Mid_lane ",Mid_lane)
		# else:
		# 	cv2.destroyWindow("[FetchInfoAndDisplay] Mid_lane_edge")
		# 	cv2.destroyWindow("[FetchInfoAndDisplay] Mid_lane ")

		Mid_lane_edge = cv2.bitwise_and(Mid_lane_edge,Mid_lane)

		# if (config.debugging and config.debugging_Lane and config.debugging_L_LaneInfoExtraction):
		# 	cv2.imshow("[FetchInfoAndDisplay] Trash Removed (Mid_lane_edge) ",Mid_lane_edge)
		# else:
		# 	cv2.destroyWindow("[FetchInfoAndDisplay] Trash Removed (Mid_lane_edge) ")

		Lanes_combined = cv2.bitwise_or(Outer_Lane,Mid_lane)

		# if (config.debugging and config.debugging_Lane and config.debugging_L_LaneInfoExtraction):
		# 	cv2.imshow("[FetchInfoAndDisplay] Lanes_combined",Lanes_combined)
		# else:
		# 	cv2.destroyWindow("[FetchInfoAndDisplay] Lanes_combined")
		
		
		ProjectedLane = np.zeros(Lanes_combined.shape,Lanes_combined.dtype)
		cnts = cv2.findContours(Lanes_combined,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]

		if cnts:
			cnts = np.concatenate(cnts)
			cnts = np.array(cnts)
			cv2.fillConvexPoly(ProjectedLane, cnts, 255)

			# if (config.debugging and config.debugging_Lane and config.debugging_L_LaneInfoExtraction):
			# 	cv2.imshow("[FetchInfoAndDisplay] ProjectedLane",ProjectedLane)
			# else:
			# 	cv2.destroyWindow("[FetchInfoAndDisplay] ProjectedLane")

			
		Mid_less_Mask = EstimateNonMidMask(Mid_lane_edge)

		# if (config.debugging and config.debugging_Lane and config.debugging_L_LaneInfoExtraction):
		# 	cv2.imshow("[FetchInfoAndDisplay] Mid_less_Mask ",Mid_less_Mask)
		# else:
		# 	cv2.destroyWindow("[FetchInfoAndDisplay] Mid_less_Mask ")

		
		ProjectedLane = cv2.bitwise_and(Mid_less_Mask,ProjectedLane)

		Lane_drawn_frame = frame

		Lane_drawn_frame[ProjectedLane==255] = Lane_drawn_frame[ProjectedLane==255] + (0,100,0)
		Lane_drawn_frame[Outer_Lane==255] = Lane_drawn_frame[Outer_Lane==255] + (0,0,100)# Outer Lane Coloured Red
		Lane_drawn_frame[Mid_lane==255] = Lane_drawn_frame[Mid_lane==255] + (100,0,0)# Mid Lane Coloured Blue

		Out_image = Lane_drawn_frame

		cv2.line(Out_image,(int(Out_image.shape[1]/2),Out_image.shape[0]),(int(Out_image.shape[1]/2),Out_image.shape[0]-int (Out_image.shape[0]/5)),(0,0,255),2)
		cv2.line(Out_image,Traj_lowP,Traj_upP,(255,0,0),2)

		if(Traj_lowP!=(0,0)):
			cv2.line(Out_image,Traj_lowP,(int(Out_image.shape[1]/2),Traj_lowP[1]),(255,255,0),2)# distance of car center with lane path

		# if (config.debugging and config.debugging_Lane):
		curvature_str="Curvature = " + f"{curvature:.2f}"
		PerpDist_ImgCen_CarNose_str="Distance = " + str(PerpDist_LaneCentralStart_CarNose)
		textSize_ratio = 0.5
		cv2.putText(Out_image,curvature_str,(10,30),cv2.FONT_HERSHEY_DUPLEX,textSize_ratio,(0,255,255),1)
		cv2.putText(Out_image,PerpDist_ImgCen_CarNose_str,(10,50),cv2.FONT_HERSHEY_DUPLEX,textSize_ratio,(0,255,255),1)

		# cv2.imshow('final', Out_image)
		# cv2.waitKey(1)

	return PerpDist_LaneCentralStart_CarNose,curvature, Out_image
