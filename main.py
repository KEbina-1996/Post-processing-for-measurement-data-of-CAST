import pandas as pd
import numpy as np
import dataProcessFunc
import time
from concurrent.futures import ProcessPoolExecutor

# data folder
dataFolder = 'mocap'
# save folder
saveFolder = 'calc'
# reading list of the target file
targetFile = pd.read_csv('targetFile.csv', header=0, encoding="shift-jis")
# List of surgical instrument used in the task
forcepsList = ['KARL-STORZ', 'LIGASHURE','OLYMPUS', 'SUCTION', 'HEM-O-LOK', 'NONOMURA']

# main process
def mainProcess(infoFile):
    print("Reading file:", infoFile["filename"])

    # reading target file
    df = pd.read_csv(dataFolder+"/"+infoFile["filename"], header=[0, 1, 3, 4])
    # reading cannula positions
    trocarPos = pd.DataFrame(np.array([[infoFile["trocar1_x"], infoFile["trocar1_y"], infoFile["trocar1_z"]],\
                          [infoFile["trocar2_x"], infoFile["trocar2_y"], infoFile["trocar2_z"]],\
                          [infoFile["trocar3_x"], infoFile["trocar3_y"], infoFile["trocar3_z"]]]),
                           columns=['X', 'Y', 'Z'])
    # reading cadaver body position (shoulder, Anterior Superior Iliac Spine, and knee)
    cadaverPos = pd.DataFrame(np.array([[infoFile["cadaver1_x"], infoFile["cadaver1_y"], infoFile["cadaver1_z"]],\
                                        [infoFile["cadaver2_x"], infoFile["cadaver2_y"], infoFile["cadaver2_z"]],\
                                        [infoFile["cadaver3_x"], infoFile["cadaver3_y"], infoFile["cadaver3_z"]]]),\
                                        columns=['X', 'Y', 'Z'])
    # set the ROI
    baseVec, crossVec,recVertex = dataProcessFunc.setCadaverArea(cadaverPos, trocarPos, infoFile["cadaverVec"])

    # Modifying file header
    df.rename(columns={'Type': 'Time', 'Name': '','Unnamed: 1_level_2': '', 'Time (Seconds)': ''}, inplace=True)
    df = df.loc[:, ['Time', 'Rigid Body', 'Rigid Body Marker']]
    df.sort_index(level=0, axis=1, inplace=True)

    # Checking missing value
    print("Check missing value:", infoFile["filename"])
    features = []
    with ProcessPoolExecutor(max_workers=7) as ppe:
        for forceps in forcepsList:
            features.append(ppe.submit(dataProcessFunc.checkForcepsMissing,df.loc[:, ('Rigid Body', forceps)]))
    for itr, forceps in enumerate(forcepsList):
        df.loc[:, ('Rigid Body', forceps, 'missingFg', 'bool')] = np.empty(len(df['Rigid Body', forceps, 'Position']))
        df.sort_index(level=0, axis=1, inplace=True)
        df.loc[:, ('Rigid Body', forceps)]=features[itr].result()

    # Outlier ditection using measurement rate in moving window
    print("Outlier removal using measurement rate in moving window", infoFile["filename"])
    features = []
    with ProcessPoolExecutor(max_workers=7) as ppe:
        for forceps in forcepsList:
            features.append(ppe.submit(dataProcessFunc.outlierDitectionUsingMrate,df.loc[:, ('Rigid Body', forceps)],wsize=3600,capRatioTH=0.25))
    for itr, forceps in enumerate(forcepsList):
        df.loc[:, ('Rigid Body', forceps)] = features[itr].result()

    # Checking if the tip of the surgical instrument is inside the ROI
    print("Check the tip position is inside the ROI (stage1):", infoFile["filename"])
    features = []
    with ProcessPoolExecutor(max_workers=7) as ppe:
        for forceps in forcepsList:
            features.append(
                ppe.submit(dataProcessFunc.checkForcepsUsing, df.loc[:, ('Rigid Body', forceps, 'Position')], baseVec,crossVec))
    for itr, forceps in enumerate(forcepsList):
        df.loc[:, ('Rigid Body', forceps, 'usingFg', 'bool')] = np.empty(
            len(df['Rigid Body', forceps, 'missingFg', 'bool']))
        df.sort_index(level=0, axis=1, inplace=True)
        df.loc[:, ('Rigid Body', forceps, 'usingFg', 'bool')] = features[itr].result()

    # Outlier removal based on ROI
    print("Outlier removal using box in/out", infoFile["filename"])
    features = []
    with ProcessPoolExecutor(max_workers=7) as ppe:
        for forceps in forcepsList:
            features.append(
                ppe.submit(dataProcessFunc.outlierDitectionUsingBoxout, df.loc[:, ('Rigid Body', forceps)], usingTH=600, missTH=12))
    for itr, forceps in enumerate(forcepsList):
        df.loc[:, ('Rigid Body', forceps)] = features[itr].result()

    # Outlier detection using kalman filter
    print("Outlier ditection using KF:", infoFile["filename"])
    features = []
    with ProcessPoolExecutor(max_workers=7) as ppe:
        for forceps in forcepsList:
            features.append(
                ppe.submit(dataProcessFunc.excOutlinerUsingKF, df.loc[:, ('Rigid Body', forceps)], R=0.0001, Q=0.01, thRatio=5))
    for itr, forceps in enumerate(forcepsList):
        df.loc[:, ('Rigid Body', forceps)] = features[itr].result()

    # Interpolation and smoothing process
    print("Interpolation", infoFile["filename"])
    features = []
    with ProcessPoolExecutor(max_workers=7) as ppe:
        for forceps in forcepsList:
            features.append(ppe.submit(dataProcessFunc.doInterpolation, df.loc[:, ('Rigid Body', forceps)]))
    for itr, forceps in enumerate(forcepsList):
        df.loc[:, ('Rigid Body', forceps, 'Velocity', '3axis')] = np.empty(
            len(df['Rigid Body', forceps, 'Position']))  # 速度配列確保
        df.sort_index(level=0, axis=1, inplace=True)
        df.loc[:, ('Rigid Body', forceps)] = features[itr].result().to_numpy()

    # Checking if the tip of the surgical instrument is inside the ROI
    print("Check the tip position is inside the ROI (stage2):", infoFile["filename"])
    features = []
    with ProcessPoolExecutor(max_workers=7) as ppe:
        for forceps in forcepsList:
            features.append(
                ppe.submit(dataProcessFunc.checkForcepsUsing, df.loc[:, ('Rigid Body', forceps, 'Position')], baseVec,
                           crossVec))
    for itr, forceps in enumerate(forcepsList):
        df.loc[:, ('Rigid Body', forceps, 'usingFg', 'bool')] = features[itr].result()

    # export csv data
    print("Exporting csvfile:", "Calc_" + infoFile["filename"])
    df[['Time', 'Rigid Body']].to_csv(saveFolder + "/Calc_" + infoFile["filename"])

if __name__ == '__main__':
    print("Starting analysis...")
    start = time.time()
    features = []
    rowIdx = []

    # Conduct processing using multi processing 
    with ProcessPoolExecutor(max_workers=4) as ppe:
        for i, info in enumerate(targetFile.to_dict(orient='records')):
            # Process only files with execution flags
            if info["processFg"] == 1:
                features.append(ppe.submit(mainProcess, info))
                rowIdx.append(i)

    # Complete 
    print("Complete all analysis")
    print("elapsed_time:{0}".format(time.time() - start) + "(sec)")