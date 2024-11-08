import cv2
import numpy as np
import time
from  ....config import config

from ..Morph_op import BwareaOpen,RetLargestContour_OuterLane,Ret_LowestEdgePoints

highest_val = 255

HLS=0
src=0
Hue_Low = 0
Lit_Low = 225
Sat_Low = 0#61  # for white we are not defining the higher value of hue
                    # as it is only the intensity of the illumination
                    # and lightness can used to segment out the white color
                    # but yellow is more color specific so we'll define for that

Hue_Low_Y = 30#30
Hue_High_Y = 33#40
Lit_Low_Y = 120#63
Sat_Low_Y = 0#81

def OnHueLowChange(val):
    global Hue_Low
    Hue_Low = val
    MaskExtract()
def OnLitLowChange(val):
    global Lit_Low
    Lit_Low = val
    MaskExtract()
def OnSatLowChange(val):
    global Sat_Low
    Sat_Low = val
    MaskExtract()

def OnHueLowChange_Y(val):
    global Hue_Low_Y
    Hue_Low_Y = val
    MaskExtract()
def OnHueHighChange_Y(val):
    global Hue_High_Y
    Hue_High_Y = val
    MaskExtract()	
def OnLitLowChange_Y(val):
    global Lit_Low_Y
    Lit_Low_Y = val
    MaskExtract()
def OnSatLowChange_Y(val):
    global Sat_Low_Y
    Sat_Low_Y = val
    MaskExtract()

def MaskExtract():
    mask   = clr_segment(HLS,(Hue_Low  ,Lit_Low   ,Sat_Low  ),(highest_val,highest_val,highest_val))
    mask_Y = clr_segment(HLS,(Hue_Low_Y,Lit_Low_Y ,Sat_Low_Y),(Hue_High_Y,highest_val,highest_val))#Combine 6ms
    mask_Y_ = mask_Y != 0
    dst_Y = src * (mask_Y_[:,:,None].astype(src.dtype))
    mask_ = mask != 0

    dst = src * (mask_[:,:,None].astype(src.dtype))
    if (config.debugging_Lane and config.debugging and config.debugging_L_ColorSeg):
        cv2.imshow('[Segment_Colour_final] mask',dst)
        cv2.imshow('[Segment_Colour_final] mask_Y',dst_Y)

def clr_segment(HSL,lower_range,upper_range):
    
    lower = np.array( [lower_range[0], lower_range[1], lower_range[2]] )
    upper = np.array( [upper_range[0], highest_val, highest_val])
    mask = cv2.inRange(HSL, lower, upper)
    
    kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3,3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
    return mask


def LaneROI(frame,mask,minArea):
    
    frame_Lane = cv2.bitwise_and(frame,frame,mask=mask)#Extracting only RGB from a specific region
    
    Lane_gray = cv2.cvtColor(frame_Lane,cv2.COLOR_BGR2GRAY) 
    
    Lane_gray_opened = BwareaOpen(Lane_gray,minArea) # Getting mask of only objects larger then minArea
    
    Lane_gray = cv2.bitwise_and(Lane_gray,Lane_gray_opened)# Getting the gray of that mask
    Lane_gray_Smoothed = cv2.GaussianBlur(Lane_gray,(11,11),1) # Smoothing out the edges for edge extraction later
   
    Lane_edge = cv2.Canny(Lane_gray_Smoothed,50,150, None, 3) # Extracting the Edge of Canny

    return Lane_edge,Lane_gray_opened

def OuterLaneROI(frame,mask,minArea):

    Outer_Points_list=[]

    frame_Lane = cv2.bitwise_and(frame,frame,mask=mask)
    Lane_gray = cv2.cvtColor(frame_Lane,cv2.COLOR_BGR2GRAY)
    Lane_gray_opened = BwareaOpen(Lane_gray,minArea)
    Lane_gray = cv2.bitwise_and(Lane_gray,Lane_gray_opened)
    Lane_gray_Smoothed = cv2.GaussianBlur(Lane_gray,(11,11),1)
    Lane_edge = cv2.Canny(Lane_gray_Smoothed,50,150, None, 3) 

    ROI_mask_Largest,Largest_found = RetLargestContour_OuterLane(Lane_gray_opened,minArea) # Extracting the largest Yellow object in frame

    if(Largest_found):
        
        Outer_edge_Largest = cv2.bitwise_and(Lane_edge,ROI_mask_Largest)
        
        Lane_TwoEdges, Outer_Points_list = Ret_LowestEdgePoints(ROI_mask_Largest)
        Lane_edge = Outer_edge_Largest
    else:
        Lane_TwoEdges = np.zeros(Lane_gray.shape,Lane_gray.dtype)

    return Lane_edge,Lane_TwoEdges,Outer_Points_list

def Segment_Colour(frame,minArea):
    
    global HLS,src

    src = frame.copy()
    
    HLS = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)

    mask   = clr_segment(HLS, (Hue_Low, Lit_Low, Sat_Low), (highest_val, highest_val, highest_val))
    mask_Y = clr_segment(HLS, (Hue_Low_Y, Lit_Low_Y, Sat_Low_Y), (Hue_High_Y, highest_val, highest_val))
    
    Outer_edge_ROI,OuterLane_SidesSeperated,Outer_Points_list = OuterLaneROI(frame,mask_Y,minArea+500)

    Mid_edge_ROI,Mid_ROI_mask = LaneROI(frame,mask,minArea)


    # if (config.debugging_Lane and config.debugging and config.debugging_L_ColorSeg):
        
    #     if not config.clr_seg_dbg_created:            
    #         config.clr_seg_dbg_created = True        
    #         cv2.namedWindow("[Segment_Colour_final] mask")
    #         cv2.namedWindow("[Segment_Colour_final] mask_Y")

    #         cv2.createTrackbar("Hue_L","[Segment_Colour_final] mask",Hue_Low,highest_val,OnHueLowChange)
    #         cv2.createTrackbar("Lit_L","[Segment_Colour_final] mask",Lit_Low,highest_val,OnLitLowChange)
    #         cv2.createTrackbar("Sat_L","[Segment_Colour_final] mask",Sat_Low,highest_val,OnSatLowChange)

    #         cv2.createTrackbar("Hue_L","[Segment_Colour_final] mask_Y",Hue_Low_Y,highest_val,OnHueLowChange_Y)
    #         cv2.createTrackbar("Hue_H","[Segment_Colour_final] mask_Y",Hue_High_Y,highest_val,OnHueHighChange_Y)
    #         cv2.createTrackbar("Lit_L","[Segment_Colour_final] mask_Y",Lit_Low_Y,highest_val,OnLitLowChange_Y)
    #         cv2.createTrackbar("Sat_L","[Segment_Colour_final] mask_Y",Sat_Low_Y,highest_val,OnSatLowChange_Y)

    #         cv2.imshow('[Segment_Colour_final] mask',mask)
    #         cv2.imshow('[Segment_Colour_final] mask_Y',mask_Y)
    #     cv2.imshow('Mid_edge_ROI',Mid_edge_ROI)
    #     cv2.imshow('Outer_edge_ROI',Outer_edge_ROI)
    #     cv2.imshow('OuterLane_Side_Seperated',OuterLane_SidesSeperated)
    # else:
    #     if config.clr_seg_dbg_created:
    #         cv2.destroyWindow('[Segment_Colour_final] mask')
    #         cv2.destroyWindow('[Segment_Colour_final] mask_Y')
    #     cv2.destroyWindow('Mid_edge_ROI')
    #     cv2.destroyWindow('Outer_edge_ROI')
    #     cv2.destroyWindow('OuterLane_Side_Seperated')

    return Mid_edge_ROI,Mid_ROI_mask,Outer_edge_ROI,OuterLane_SidesSeperated,Outer_Points_list