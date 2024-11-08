#!/usr/bin/env python3

from ...config import config
import cv2

from .Seg.color_seg import Segment_Colour
from .Est.est import Estimate_MidLane
from .clean.CorrectYellow import GetYellowInnerEdge
from .clean.ExtendLanes_RefineMidLaneEdge import ExtendShortLane
from .Data_ext.state_info_lane_display import FetchInfoAndDisplay


def detect_lanes(img):
        
        img_cropped = img[config.CropHeight_resized:,:]

        Mid_edge_ROI,Mid_ROI_mask,Outer_edge_ROI,OuterLane_TwoSide,OuterLane_Points = Segment_Colour(img_cropped,config.minArea_resized)

        Estimated_midlane = Estimate_MidLane(Mid_edge_ROI,config.MaxDist_resized)

        OuterLane_OneSide,Outer_cnts_oneSide,Mid_cnts,Offset_correction = GetYellowInnerEdge(OuterLane_TwoSide,Estimated_midlane,OuterLane_Points)
        Estimated_midlane,OuterLane_OneSide = ExtendShortLane(Estimated_midlane,Mid_cnts,Outer_cnts_oneSide,OuterLane_OneSide)
        
        Distance , Curvature = FetchInfoAndDisplay(Mid_edge_ROI,Estimated_midlane,OuterLane_OneSide,img_cropped,Offset_correction,img)
        
        # img_copy = img.copy()
        # img_copy[config.CropHeight_resized:,:] = out_image

        # cv2.imshow('final', img_copy)
        # cv2.waitKey(1)


        return Distance,Curvature
       