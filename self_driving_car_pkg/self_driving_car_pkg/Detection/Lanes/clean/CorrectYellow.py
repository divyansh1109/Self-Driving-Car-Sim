import cv2
import numpy as np
from ....config import config
from ..utilities import findLineParameter,findlaneCurvature,Distance_,Cord_Sort


def IsPathCrossingMid(Midlane,Mid_cnts,Outer_cnts):

	is_Ref_to_path_Left = 0
	Ref_To_Path_Image = np.zeros_like(Midlane)
	Midlane_copy = Midlane.copy()

	Mid_cnts_Rowsorted = Cord_Sort(Mid_cnts,"rows")
	Outer_cnts_Rowsorted = Cord_Sort(Outer_cnts,"rows")
	if (config.debugging_Lane and config.debugging and config.debugging_L_Cleaning):
		if not Mid_cnts:
			print("[Warning!!!] NO Midlane detected")
	Mid_Rows = Mid_cnts_Rowsorted.shape[0]
	Outer_Rows = Outer_cnts_Rowsorted.shape[0]

	Mid_lowP = Mid_cnts_Rowsorted[Mid_Rows-1,:]
	Outer_lowP = Outer_cnts_Rowsorted[Outer_Rows-1,:]

	Traj_lowP = ( int( (Mid_lowP[0] + Outer_lowP[0]  ) / 2 ) , int( (Mid_lowP[1]  + Outer_lowP[1] ) / 2 ) )
	
	cv2.line(Ref_To_Path_Image,Traj_lowP,(int(Ref_To_Path_Image.shape[1]/2),Ref_To_Path_Image.shape[0]),(255,255,0),2)# distance of car center with lane path
	cv2.line(Midlane_copy,tuple(Mid_lowP),(Mid_lowP[0],Midlane_copy.shape[0]-1),(255,255,0),2)# distance of car center with lane path

	is_Ref_to_path_Left = ( (int(Ref_To_Path_Image.shape[1]/2) - Traj_lowP[0]) > 0 )

	if( np.any( (cv2.bitwise_and(Ref_To_Path_Image,Midlane_copy) > 0) ) ):
		return True,is_Ref_to_path_Left
	else:
		return False,is_Ref_to_path_Left

def GetYellowInnerEdge(OuterLanes,MidLane,OuterLane_Points):
	
	# if (config.debugging_Lane and config.debugging and config.debugging_L_Cleaning):
	# 	cv2.imshow("[GetYellowInnerEdge] OuterLanes",OuterLanes)
	# else:
	# 	cv2.destroyWindow("[GetYellowInnerEdge] OuterLanes")

	Offset_correction = 0
	
	Outer_Lanes_ret= np.zeros(OuterLanes.shape,OuterLanes.dtype)
	
	Mid_cnts = cv2.findContours(MidLane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
	Outer_cnts = cv2.findContours(OuterLanes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

	if not Outer_cnts:
		NoOuterLane_before=True
	else:
		NoOuterLane_before=False

	Ref = (0,0)
	if(Mid_cnts):
		Ref = tuple(Mid_cnts[0][0][0])

	if  ( Mid_cnts and (len(OuterLane_Points)==2)):
		Point_a = OuterLane_Points[0]
		Point_b = OuterLane_Points[1]
		
		Closest_Index = 0
		if(Distance_(Point_a,Ref) <= Distance_(Point_b,Ref)):
			Closest_Index=0
		elif(len(Outer_cnts)>1):
			Closest_Index=1
		Outer_Lanes_ret = cv2.drawContours(Outer_Lanes_ret, Outer_cnts, Closest_Index, 255, 1)
		Outer_cnts_ret = [Outer_cnts[Closest_Index]]

		IsPathCrossing , IsCrossingLeft = IsPathCrossingMid(MidLane,Mid_cnts,Outer_cnts_ret)
		if(IsPathCrossing):
			if(config.debugging_Lane and config.debugging and config.debugging_L_Cleaning):
				print("[FindClosestLane] [(len(OuterLane_Points)==2)] Zeroing OuterLanes because LAnes are crossing")
			OuterLanes = np.zeros_like(OuterLanes)
		else:
			return Outer_Lanes_ret ,Outer_cnts_ret, Mid_cnts,0

		# if (config.debugging_Lane and config.debugging and config.debugging_L_Cleaning):
		# 	cv2.imshow("[GetYellowInnerEdge] OuterLanesaftr",OuterLanes)
		# else:
		# 	cv2.destroyWindow("[GetYellowInnerEdge] OuterLanesaftr")
		
	
	elif( Mid_cnts and np.any(OuterLanes>0) ):
		IsPathCrossing , IsCrossingLeft = IsPathCrossingMid(MidLane,Mid_cnts,Outer_cnts)
		if(IsPathCrossing):
			if (config.debugging_Lane and config.debugging and config.debugging_L_Cleaning):
				print("[FindClosestLane] [np.any(OuterLanes>0)] Zeroing OuterLanes because LAnes are crossing")
			OuterLanes = np.zeros_like(OuterLanes)
		else:
			if (config.debugging_Lane and config.debugging and config.debugging_L_Cleaning):
				print("[FindClosestLane] [np.any(OuterLanes>0)] Path are not crossing --> Ret as it is")
			return OuterLanes ,Outer_cnts, Mid_cnts,0		

	if( Mid_cnts and ( not np.any(OuterLanes>0) ) ):	
		if (config.debugging_Lane and config.debugging and config.debugging_L_Cleaning):
			print("[FindClosestLane] [OuterLanes is Empty] OuterLanes Not empty but points are empty")

		Mid_cnts_Rowsorted = Cord_Sort(Mid_cnts,"rows")
		Mid_Rows = Mid_cnts_Rowsorted.shape[0]
		Mid_lowP = Mid_cnts_Rowsorted[Mid_Rows-1,:]
		Mid_highP = Mid_cnts_Rowsorted[0,:]
		Mid_low_Col = Mid_lowP[0]
		
		DrawRight = False
 
		if NoOuterLane_before:
			if (config.debugging_Lane and config.debugging and config.debugging_L_Cleaning):
				print("[FindClosestLane] [OuterLanes is Empty] No OuterLanes were detected at all so can only rely on Midlane Info!!")
			if(Mid_low_Col < int(MidLane.shape[1]/2)): 
				DrawRight = True
		else:
			if (config.debugging_Lane and config.debugging and config.debugging_L_Cleaning):
				print("[FindClosestLane] IsPathCrossing = ",IsPathCrossing," IsCrossingLeft = ",IsCrossingLeft)
			if IsCrossingLeft: 
				DrawRight = True
		if (config.debugging_Lane and config.debugging and config.debugging_L_Cleaning):
			print("[FindClosestLane] [OuterLanes is Empty] DrawRight = ",DrawRight)

		if not DrawRight:
			low_Col=0
			high_Col=0
			Offset_correction = -20
		else:
			low_Col=(int(MidLane.shape[1])-1)
			high_Col=(int(MidLane.shape[1])-1)
			Offset_correction = 20

		Mid_lowP[1] = MidLane.shape[0]

		LanePoint_lower =  (low_Col , int( Mid_lowP[1] ) )
		LanePoint_top   =  (high_Col, int( Mid_highP[1]) )

		OuterLanes = cv2.line(OuterLanes,LanePoint_lower,LanePoint_top,255,1)	

		Outer_cnts = cv2.findContours(OuterLanes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

		return OuterLanes, Outer_cnts, Mid_cnts, Offset_correction
		
	else:
		return OuterLanes, Outer_cnts, Mid_cnts, Offset_correction