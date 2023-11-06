import numpy as np
import scipy.signal
import quaternion
import math
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

def point_to_line_distance(point, line_start, line_end):
    line_direction = line_end - line_start
    line_length = np.linalg.norm(line_direction)
    line_direction = line_direction / line_length

    v = point - line_start
    t = np.dot(v, line_direction)
    t = max(0, min(t, line_length))

    projected_point = line_start + t * line_direction
    return projected_point

# function for setting ROI
def setCadaverArea(mpos, trocarPos, cadaverDirectionFg):
    # parameter for setting ROI
    heightOffset = -0.05  # height offset (m) 
    ROI_Height = 0.30  # height of ROI (m)
    ROI_FrontDepth = 0.20  # depth of ROI (front side) (m)
    ROI_BackDepth = 0.10  # depth of ROI (back side) (m)
    ROI_Width = 0.4  # width of ROI (ratio of length l in Fig.8)
    waistOffsetRatio = 0.05 # offset of ROI base point to cannula position (m) 
    # Reorder cannula and body marker positions according to left and right nephrectomy
    if cadaverDirectionFg == True:
        mpos = mpos.sort_values('Z')
        trocarPos = trocarPos.sort_values('Z')
    else:
        mpos = mpos.sort_values('Z', ascending=False)
        trocarPos = trocarPos.sort_values('Z', ascending=False)
    mpos.reset_index(inplace=True, drop=True)
    trocarPos.reset_index(inplace=True, drop=True)
    # applying y offset
    mpos.at[0, 'Y'] = mpos.at[1, 'Y'] + heightOffset
    mpos.at[2, 'Y'] = mpos.at[1, 'Y'] + heightOffset
    # calculate line L
    line = mpos.loc[2, :] - mpos.loc[0, :]
    # calculate cross point of line L and cannule position
    pt = point_to_line_distance(trocarPos.loc[2, :], mpos.loc[0, :], mpos.loc[2, :])
    # Determination of ROI endpoints
    mpos.loc[0, :] = pt - ROI_Width * line  # left (shoulder) side of ROI
    mpos.loc[2, :] = pt + waistOffsetRatio * line # right (foot) side of ROI
    # Calculate outer product vector with Y-axis
    tmpCross = np.cross(np.array([0, 1, 0]), line.to_numpy())
    tmpCross = tmpCross / np.linalg.norm(tmpCross)
    # calculate vertex of ROI
    recVertex = np.empty((8, 3))
    if cadaverDirectionFg == True:
        recVertex[0] = mpos.loc[0, :] + ROI_FrontDepth * tmpCross
        recVertex[1] = mpos.loc[0, :] - ROI_BackDepth * tmpCross
        recVertex[2] = mpos.loc[2, :] + ROI_FrontDepth * tmpCross
        recVertex[3] = mpos.loc[2, :] - ROI_BackDepth * tmpCross
    else:
        recVertex[0] = mpos.loc[0, :] - ROI_FrontDepth * tmpCross
        recVertex[1] = mpos.loc[0, :] + ROI_BackDepth * tmpCross
        recVertex[2] = mpos.loc[2, :] - ROI_FrontDepth * tmpCross
        recVertex[3] = mpos.loc[2, :] + ROI_BackDepth * tmpCross
    for i in range(4, 8):
        recVertex[i] = recVertex[i - 4]
        recVertex[i][1] -= ROI_Height
    # Generation of outer product vectors in the inner direction for each rectangular face
    crossVec = np.empty((6, 3))
    baseVec = np.empty((2, 3))
    crossVec[0] = np.cross(recVertex[1] - recVertex[0], recVertex[2] - recVertex[0])
    crossVec[1] = np.cross(recVertex[4] - recVertex[0], recVertex[1] - recVertex[0])
    crossVec[2] = np.cross(recVertex[2] - recVertex[0], recVertex[4] - recVertex[0])
    if cadaverDirectionFg == True:
        crossVec[0] = -1 * crossVec[0]
        crossVec[1] = -1 * crossVec[1]
        crossVec[2] = -1 * crossVec[2]
    crossVec[3] = -1 * crossVec[2]
    crossVec[4] = -1 * crossVec[1]
    crossVec[5] = -1 * crossVec[0]
    # Define base point
    baseVec[0] = recVertex[0]
    baseVec[1] = recVertex[7]
    return baseVec, crossVec, recVertex

# quaternion spherical linear completion function
def quatSlerp(quatFrame):
    modeFg=-1
    qStart=np.quaternion(1,0,0,0)
    qEnd=np.quaternion(1,0,0,0)
    result=quatFrame.to_numpy()

    for i in range(len(result)):        
        if modeFg<0 and np.isnan(result[i]).any()==True:
            modeFg=i
            if modeFg!=0:
                qStart=np.quaternion(result[i-1,3],result[i-1,0],result[i-1,1],result[i-1,2])
        if modeFg>=0:
            if np.isnan(result[i]).any()==False or i==len(result)-1:
                loopNum=0
                qEnd=np.quaternion(result[i,3],result[i,0],result[i,1],result[i,2])
                if modeFg==0:
                    qStart=qEnd
                if i==len(result)-1:
                    qEnd=qStart
                    loopNum=1
                modeFg=i-modeFg
                delta=1/(modeFg+1)
                for j in range(modeFg+loopNum):
                    tmpQuat=quaternion.slerp_evaluate(qStart,qEnd,(j+1)*delta)
                    result[i-modeFg+j,0]=tmpQuat.x
                    result[i-modeFg+j,1]=tmpQuat.y
                    result[i-modeFg+j,2]=tmpQuat.z
                    result[i-modeFg+j,3]=tmpQuat.w
                modeFg=-1
    return result

def convertQuatToZYXeulerAngles(quat):
    q = quaternion.as_quat_array(quat)
    for i in range(len(q)):
        matrix = quaternion.as_rotation_matrix(q[i])
        r11, r12, r13 = matrix[0]
        r21, r22, r23 = matrix[1]
        r31, r32, r33 = matrix[2]
        theta1 = np.arctan2(r21,r11)
        theta2 = np.arctan2(-r31 * np.cos(theta1), r11)
        theta3 = np.arctan2(r32 , r33)
        theta1 = theta1 * 180 / np.pi +180
        theta2 = theta2 * 180 / np.pi -180
        theta3 = theta3 * 180 / np.pi -180      
        quat[i] = [0, theta3, theta2, theta1]
    return quat

def getContinuousAngle(ang):
    th = math.pi
    for i in range(len(ang)):
        if i > 0:
            while ang[i] - ang[i-1] > th:
                ang[i] -= math.pi*2
            while ang[i] - ang[i-1] < -th:
                ang[i] += math.pi*2
    return ang

def applyAngleLimitation(ang):
    for i in range(len(ang)):
        while ang[i] > math.pi:
            ang[i]-=2*math.pi
        while ang[i] < -math.pi:
            ang[i]+=2*math.pi
    return ang

# function to check missing data
def checkForcepsMissing(forcepsData):
    npMissingFg=forcepsData.loc[:,'Position'].isnull().any(axis=1).astype('int8').to_numpy()
    npPos=forcepsData.loc[:,'Position'].to_numpy()
    npQuat=forcepsData.loc[:,'Rotation'].to_numpy()
    zeros=np.zeros(3)
    # If the position does not change in the next frame, it is assumed to be a missing value.
    for i in range(len(npPos)-1):
        if np.all(npPos[i+1]-npPos[i]==zeros):
            npMissingFg[i]=1
            npPos[i]=np.nan
            npQuat[i]=np.nan
    else:
        if np.all(npPos[len(npPos)-1]-npPos[len(npPos)-2]==zeros):
            npMissingFg[len(npPos)-1]=1
            npPos[i]=np.nan
            npQuat[i]=np.nan
    npMissingFg=npMissingFg.reshape(len(npMissingFg),1)
    return np.concatenate([npPos,npQuat,npMissingFg],axis=1)

# 2-order kalman filter (3-axis)
def posVelAccFilter(x,v,a, P, R, Q=0., dt=1.0):
    kf = KalmanFilter(dim_x=9, dim_z=3)
    kf.x = np.array([x[0],x[1],x[2],v[0],v[1],v[2],a[0],a[1],a[2]])
    kf.F = np.array([[1.,0.,0.,dt,0.,0.,0.5*dt*dt,0.,0.],
                     [0.,1.,0.,0.,dt,0.,0.,0.5*dt*dt,0.],
                     [0.,0.,1.,0.,0.,dt,0.,0.,0.5*dt*dt],
                     [0.,0.,0.,1.,0.,0.,dt,0.,0.],
                     [0.,0.,0.,0.,1.,0.,0.,dt,0.],
                     [0.,0.,0.,0.,0.,1.,0.,0.,dt],
                     [0.,0.,0.,0.,0.,0.,1.,0.,0.],
                     [0.,0.,0.,0.,0.,0.,0.,1.,0.],
                     [0.,0.,0.,0.,0.,0.,0.,0.,1.]])
    kf.H = np.array([[1.,0.,0.,0.,0.,0.,0.,0.,0.],
                     [0.,1.,0.,0.,0.,0.,0.,0.,0.],
                     [0.,0.,1.,0.,0.,0.,0.,0.,0.]])
    kf.R *= R
    if np.isscalar(P):
        kf.P *= P
    else:
        kf.P[:] = P
    if np.isscalar(Q):
        kf.Q = Q_discrete_white_noise(dim=3, dt=dt, var=Q,block_size=3,order_by_dim=False)
    else:
        kf.Q[:] = Q
    return kf

def setOutlier(pos,quat,v,missingFg,start,stop):
    for i in range(start,stop):
        pos[i]=np.nan
        quat[i]=np.nan
        v[i]=np.nan
        missingFg[i]=5
    return pos,quat,v,missingFg

# Function to exclude the long term and long distance interpolated data.
# missingFrameTH：threshold for number of missing frame (-)
# missingDisTH：threshold for interpolated distance (m)
def longTermLongDistanceInterpolationExclude(forcepsData,missingFrameTH,missingDisTH):
    npMissingFg=forcepsData.loc[:,('missingFg','bool')].to_numpy()
    npUsingFg=forcepsData.loc[:,('usingFg','bool')].to_numpy()
    npPos=forcepsData.loc[:,'Position'].to_numpy()
    npQuat=forcepsData.loc[:,'Rotation'].to_numpy()
    v=forcepsData.loc[:,('Velocity','3axis')].to_numpy()
    missingCount=0 
    processFg=False 
    missingPathLen=0

    for i in range(len(npPos)):
        if npMissingFg[i]>0:
            if i>0:
                missingPathLen+=np.linalg.norm(npPos[i]-npPos[i-1])
            missingCount+=1
        if npMissingFg[i]==0 or i==len(npPos)-1:
            if missingCount>0:
                missingPathLen+=np.linalg.norm(npPos[i]-npPos[i-1])
                if missingPathLen>missingDisTH:
                    processFg=True
                if missingCount>missingFrameTH:
                    processFg=True
                if processFg==True:
                    processFg=False
                    npPos,npQuat,v,npMissingFg=setOutlier(npPos,npQuat,v,npMissingFg,start=i-missingCount,stop=i)
            missingCount=0
            missingPathLen=0
    npMissingFg=npMissingFg.reshape(len(npMissingFg),1)
    npUsingFg=npUsingFg.reshape(len(npMissingFg),1)
    v=v.reshape(len(v),1)
    return np.concatenate([npPos,npQuat,v,npMissingFg,npUsingFg],axis=1)

# Function of outlier removal using kalman filter
# R: variances of observation noise (m^2)
# Q: variances of system noise (m^2)
def excOutlinerUsingKF(forcepsData,R,Q,thRatio):
    npMissingFg=forcepsData.loc[:,('missingFg','bool')].to_numpy()
    npUsingFg=forcepsData.loc[:,('usingFg','bool')].to_numpy()
    npPos=forcepsData.loc[:,'Position'].to_numpy()
    npQuat=forcepsData.loc[:,'Rotation'].to_numpy()

    missingTH=600
    xs=np.full((len(npPos),3),np.nan)
    outliner=np.full((len(npPos),3),np.nan)
    stdP=np.full((len(npPos),3),np.nan)
    x0=(0.,0.,0.)
    v0=(0.,0.,0.)
    a0=(0.,0.,0.)
    P = np.diag([500.,500.,500.,50.,50.,50.,5.,5.,5.])
    kf = posVelAccFilter(x0,v0,a0,R=R, P=P, Q=Q, dt=1/120)
    count=0

    boxin=np.full((len(npPos),3),np.nan)
    boxout=np.full((len(npPos),3),np.nan)

    # Applying the filter
    for i,z in enumerate(npPos):
        kf.predict()
        std=np.array([np.sqrt(kf.P[0,0]),np.sqrt(kf.P[1,1]),np.sqrt(kf.P[2,2])])
        # if not missing value and in the ROI 
        if not np.isnan(z).any() and npUsingFg[i]==True:
            boxin[i]=npPos[i]
            if np.linalg.norm(kf.x[0:3]-z)<np.linalg.norm(thRatio*std):
                kf.update(z)
            else:
                outliner[i]=z
                npMissingFg[i]=4
                npPos[i]=np.nan
                npQuat[i]=np.nan
            count=0
        else:
            boxout[i]=npPos[i]
            if count>missingTH:
                kf.P=P
                kf.x=(0.,0.,0.,0.,0.,0.,0.,0.,0.)
            count+=1
        xs[i]=kf.x[0:3]
        stdP[i]=std
    npMissingFg=npMissingFg.reshape(len(npMissingFg),1)
    npUsingFg=npUsingFg.reshape(len(npMissingFg),1)

    return np.concatenate([npPos,npQuat,npMissingFg,npUsingFg],axis=1)

# Outlier removal using ROI
def outlierDitectionUsingBoxout(forcepsData,usingTH,missTH):
    npMissingFg=forcepsData.loc[:,('missingFg','bool')].to_numpy()
    npUsingFg=forcepsData.loc[:,('usingFg','bool')].to_numpy()
    npPos=forcepsData.loc[:,'Position'].to_numpy()
    npQuat=forcepsData.loc[:,'Rotation'].to_numpy()
    boxinMissingFg=np.copy(npMissingFg)
    boxinPos=np.copy(npPos)
    boxinQuat=np.copy(npQuat)
    count=0
    missCount=0

    for i in range(len(npUsingFg)):
        if npUsingFg[i]==False:
            if npMissingFg[i]==0:
                boxinMissingFg[i]=3
                boxinPos[i]=np.nan
                boxinQuat[i]=np.nan
                count+=1
                missCount=0
            else:
                missCount+=1
                if missCount>missTH:
                    missCount=0
                    if count>usingTH:
                        for j in range(count):
                            boxinMissingFg[i-j-1]=npMissingFg[i-j-1]
                            boxinPos[i-j-1]=npPos[i-j-1]
                            boxinQuat[i-j-1]=npQuat[i-j-1]
                    count=0
        else:
            if count>usingTH:
                for j in range(count):
                    boxinMissingFg[i-j-1]=npMissingFg[i-j-1]
                    boxinPos[i-j-1]=npPos[i-j-1]
                    boxinQuat[i-j-1]=npQuat[i-j-1]
            count=0
            missCount=0
    boxinMissingFg=boxinMissingFg.reshape(len(npMissingFg),1)
    npUsingFg=npUsingFg.reshape(len(npUsingFg),1)
    return np.concatenate([boxinPos,boxinQuat,boxinMissingFg,npUsingFg],axis=1)

# interpolation and smoothing
def doInterpolation(forcepsData):
    if not forcepsData.isnull().all().all():
        # liner interpolation
        forcepsData.loc[:,('Position','X')]=forcepsData.loc[:,('Position','X')].interpolate(limit_direction='both',method='linear',limit=None)
        forcepsData.loc[:,('Position','Y')]=forcepsData.loc[:,('Position','Y')].interpolate(limit_direction='both',method='linear',limit=None)
        forcepsData.loc[:,('Position','Z')]=forcepsData.loc[:,('Position','Z')].interpolate(limit_direction='both',method='linear',limit=None)
        # quarternion slerp
        forcepsData.loc[:,'Rotation']=quatSlerp(forcepsData.loc[:,'Rotation'])
        # smoothing
        forcepsData.loc[:,'Position']=scipy.signal.savgol_filter(forcepsData.loc[:,'Position'].to_numpy(), window_length=121, polyorder=3, axis=0, mode='nearest')
        forcepsData.loc[:,('Velocity', '3axis')] = 120*np.linalg.norm(scipy.signal.savgol_filter(forcepsData.loc[:,'Position'].to_numpy(), window_length=121, polyorder=3, axis=0, deriv=1, mode='nearest'),axis=1)
        forcepsData.sort_index(level=0, axis=1, inplace=True)
        # Removing the interpolated data which is long term or long distance.
        forcepsData.loc[:]=longTermLongDistanceInterpolationExclude(forcepsData,missingFrameTH=600,missingDisTH=0.1)
    return forcepsData

# Check tip position is in the ROI
def checkForcepsUsing(position,baseVec,crossVec):
    npPos=position.to_numpy()
    npUsingFg=np.full(len(npPos),1)

    for i in range(len(npPos)):
        if np.isnan(npPos[i]).any()==False:
            for j in range(3):
                if np.dot(npPos[i]-baseVec[0],crossVec[j])<0:
                    npUsingFg[i]=0
            for j in range(3):
                if np.dot(npPos[i]-baseVec[1],crossVec[j+3])<0:
                    npUsingFg[i]=0
        else:
            npUsingFg[i]=0
    return npUsingFg


# outlier removal based on partial measurement success rate
# wside: moving window size
# capRatioTH: threshold for partial measurement success rate
def outlierDitectionUsingMrate(forcepsData,wsize,capRatioTH):
    npMissingFg=forcepsData.loc[:,('missingFg','bool')].to_numpy()
    npPos=forcepsData.loc[:,'Position'].to_numpy()
    npQuat=forcepsData.loc[:,'Rotation'].to_numpy()

    pdCapturedFg=forcepsData.loc[:,('missingFg','bool')].copy()
    pdCapturedFg=pdCapturedFg.mask(pdCapturedFg>0,1).astype('bool')
    pdCapturedFg=~pdCapturedFg
    pdCaptured=pdCapturedFg.astype('int8')
    npwsum=pdCaptured.rolling(window=wsize,center=True).sum().to_numpy()
    thNum=capRatioTH*wsize

    for i in range(len(npwsum)):
        if npwsum[i]!=np.nan and npMissingFg[i]==0:
            if npwsum[i]<thNum:
                npMissingFg[i]=2
                npPos[i]=np.nan
                npQuat[i]=np.nan

    npMissingFg=npMissingFg.reshape(len(npMissingFg),1)
    return np.concatenate([npPos,npQuat,npMissingFg],axis=1)