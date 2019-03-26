uint32 mnId
uint32 mClientId
uint32 command

uint64[] CovisibleList
uint32 Parent
uint64[] LoopEdgeList
float32[16] Twc
float32[16] Tcw
float32[3] Ow

uint32 nNextId
uint32 mnFrameId

float64 mTimeStamp
uint32 mnGridCols
uint32 mnGridRows
float64 mfGridElementWidthInv
float64 mfGridElementHeightInv

uint64 mnTrackReferenceForFrame
uint64 mnFuseTargetForKF

uint64 mnBALocalForKF
uint64 mnBAFixedForKF
uint64 mnBAGlobalForKF

uint64 mnLoopQuery
uint32 mnLoopWords
float32 mLoopScore
uint64 mnRelocQuery
int32 mnRelocWords
float32 mRelocScore

int32 mnScaleLevels
float32 mfScaleFactor
float32 mfLogScaleFactor
float32[] mvScaleFactors
float32[] mvLevelSigma2
float32[] mvInvLevelSigma2

float32 fx
float32 fy
float32 cx
float32 cy
float32 invfx
float32 invfy
float32 mbf
float32 mb
float32 mThDepth

int32 mnMinX
int32 mnMinY
int32 mnMaxX
int32 mnMaxY

float32[] mvuRight
float32[] mvDepth

string mDescriptors
string mData

uint64[] mvpMapPoints