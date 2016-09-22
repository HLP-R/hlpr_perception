# Features for a single object cluster

Header header

geometry_msgs/Transform transform

BasicFeatures basicInfo
OrientedBoundingBox obb

bool setColorHist            # Flag to state if hs_hist was computed and set (default=true)
ColorHist hs_hist

bool setShapeHist            # Flag to state if shape_hist was computed and set (default=true)
ShapeHist shape_hist

bool setViewpointHist        # Flag to state if view_hist was computed and set (default=true)
ViewpointHist view_hist

bool setOtherFeatures        # Flag to state if other features were computed and set (default=false)
OtherFeatures other
