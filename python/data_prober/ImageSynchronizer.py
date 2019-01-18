import numpy as np

from Metadata import Metadata
from collections import OrderedDict
from statistics import median

class StereoPairStamp:
    
    def __init__(self, index, leftStamp, rightStamp):
        self.index = index
        self.leftStamp = leftStamp
        self.rightStamp = rightStamp

    def __repr__(self):
        return "StereoPairStamp()"

    def __str__(self):
        return str(self.index) + " " + str(self.leftStamp) + " " + str(self.rightStamp) + "\n"

class ImageSynchronizer:

    dataRootFolder = ""

    dataFrontLeft  = Metadata()
    dataFrontRight = Metadata()
    dataRearLeft   = Metadata()
    dataRearRight  = Metadata()
    dataNavLeft    = Metadata()
    dataNavRight   = Metadata()

    rawFrontStamps = OrderedDict()
    rawRearStamps  = OrderedDict()
    rawNavStamps   = OrderedDict()

    iSynchedFrontStamps = OrderedDict()
    iSynchedRearStamps  = OrderedDict()
    iSynchedNavStamps   = OrderedDict()

    eSynchedFrontStamps = OrderedDict()
    eSynchedRearStamps  = OrderedDict()
    eSynchedNavStamps   = OrderedDict()

    def __init__(self, dataRootFolder="./"):

        self.dataRootFolder = dataRootFolder

        # loading front cam metadata
        self.dataFrontLeft.parse_metadata( self.dataRootFolder + "front_cam/left/left_dataformat.txt",   "front_cam/left/left_all_metadata.txt")
        self.dataFrontRight.parse_metadata(self.dataRootFolder + "front_cam/right/right_dataformat.txt", "front_cam/right/right_all_metadata.txt")
        for i in range(len(self.dataFrontLeft.timestamp)):
            self.rawFrontStamps[self.dataFrontLeft.index[i]] = StereoPairStamp(self.dataFrontLeft.index[i], self.dataFrontLeft.timestamp[i], self.dataFrontRight.timestamp[i])

        # loading rear cam metadata
        self.dataRearLeft.parse_metadata( self.dataRootFolder + "rear_cam/left/left_dataformat.txt",   "rear_cam/left/left_all_metadata.txt")
        self.dataRearRight.parse_metadata(self.dataRootFolder + "rear_cam/right/right_dataformat.txt", "rear_cam/right/right_all_metadata.txt")
        for i in range(len(self.dataRearLeft.timestamp)):
            self.rawRearStamps[self.dataRearLeft.index[i]] = StereoPairStamp(self.dataRearLeft.index[i], self.dataRearLeft.timestamp[i], self.dataRearRight.timestamp[i])

        # loading nav cam metadata
        self.dataNavLeft.parse_metadata( self.dataRootFolder + "nav_cam/left/left_dataformat.txt",   "nav_cam/left/left_all_metadata.txt")
        self.dataNavRight.parse_metadata(self.dataRootFolder + "nav_cam/right/right_dataformat.txt", "nav_cam/right/right_all_metadata.txt")
        for i in range(len(self.dataNavLeft.timestamp)):
            self.rawNavStamps[self.dataNavLeft.index[i]] = StereoPairStamp(self.dataNavLeft.index[i], self.dataNavLeft.timestamp[i], self.dataNavRight.timestamp[i])

    def get_isynched_front_pair_stamps(self):
        res = np.empty([len(self.iSynchedFrontStamps.values()),1])
        i = 0
        for value in self.iSynchedFrontStamps.values():
            res[i] = value.leftStamp
            i += 1
        return res

    def get_isynched_rear_pair_stamps(self):
        res = np.empty([len(self.iSynchedRearStamps.values()),1])
        i = 0
        for value in self.iSynchedRearStamps.values():
            res[i] = value.leftStamp
            i += 1
        return res

    def get_isynched_nav_pair_stamps(self):
        res = np.empty([len(self.iSynchedNavStamps.values()),1])
        i = 0
        for value in self.iSynchedNavStamps.values():
            res[i] = value.leftStamp
            i += 1
        return res

    def get_esynched_front_pair_stamps(self):
        res = np.empty([len(self.eSynchedFrontStamps.values()),1])
        i = 0
        for value in self.eSynchedFrontStamps.values():
            res[i] = value.leftStamp
            i += 1
        return res

    def get_esynched_rear_pair_stamps(self):
        res = np.empty([len(self.eSynchedRearStamps.values()),1])
        i = 0
        for value in self.eSynchedRearStamps.values():
            res[i] = value.leftStamp
            i += 1
        return res

    def get_esynched_nav_pair_stamps(self):
        res = np.empty([len(self.eSynchedNavStamps.values()),1])
        i = 0
        for value in self.eSynchedNavStamps.values():
            res[i] = value.leftStamp
            i += 1
        return res

    # getting indexes
    def get_esynched_front_pair_indexes(self):
        res = np.empty([len(self.eSynchedFrontStamps.values()),1])
        i = 0
        for value in self.eSynchedFrontStamps.values():
            res[i] = value.index
            i += 1
        return res

    def get_esynched_rear_pair_indexes(self):
        res = np.empty([len(self.eSynchedRearStamps.values()),1])
        i = 0
        for value in self.eSynchedRearStamps.values():
            res[i] = value.index
            i += 1
        return res

    def get_esynched_nav_pair_indexes(self):
        res = np.empty([len(self.eSynchedNavStamps.values()),1])
        i = 0
        for value in self.eSynchedNavStamps.values():
            res[i] = value.index
            i += 1
        return res
    # remove unsynchronized stereo pairs tolerance is expressed in milliseconds
    def intrinsic_synchro(self, tolerance = 0):

        print("Stereo pairs intrisic synchronization :")

        print("Front cams :\n")
        self.iSynchedFrontStamps = self.rawFrontStamps
        toBeRemoved = []
        for pair in self.iSynchedFrontStamps.values():
            if abs(pair.leftStamp - pair.rightStamp) > 1000*tolerance:
                toBeRemoved.append(pair.index)
        for index in toBeRemoved:
            print("    Removing " + str(index))
            del self.iSynchedFrontStamps[index]

        print("Rear cams :\n")
        self.iSynchedRearStamps = self.rawRearStamps
        toBeRemoved = []
        for pair in self.iSynchedRearStamps.values():
            if abs(pair.leftStamp - pair.rightStamp) > 1000*tolerance:
                toBeRemoved.append(pair.index)
        for index in toBeRemoved:
            print("    Removing " + str(index))
            del self.iSynchedRearStamps[index]

        print("Nav cams :\n")
        self.iSynchedNavStamps = self.rawNavStamps
        toBeRemoved = []
        for pair in self.iSynchedNavStamps.values():
            if abs(pair.leftStamp - pair.rightStamp) > 1000*tolerance:
                toBeRemoved.append(pair.index)
        for index in toBeRemoved:
            print("    Removing " + str(index))
            del self.iSynchedNavStamps[index]


    # remove stereo pairs to keep sync between stereo benches, tolerance is expressed in milliseconds
    def extrinsic_synchro(self, tolerance = 75):

        def are_synched(stamps, tol):
            m = median(stamps)
            if abs(stamps[0] - m) < tol and abs(stamps[1] - m) < tol and abs(stamps[2] - m) < tol:
                return True
            else:
                return False

        print("Stereo benches extrinsic synchronization :")
        tolerance = 1000*tolerance

        iFront = list(self.iSynchedFrontStamps.values());
        iRear  = list(self.iSynchedRearStamps.values());
        iNav   = list(self.iSynchedNavStamps.values());

        while len(iFront) > 0 and len(iRear) > 0 and len(iNav) > 0:
        # for i in range(10):
            # print(median([iFront[0].leftStamp, iRear[0].leftStamp, iNav[0].leftStamp]), [iFront[0].leftStamp, iRear[0].leftStamp, iNav[0].leftStamp])
            if are_synched([iFront[0].leftStamp, iRear[0].leftStamp, iNav[0].leftStamp], tolerance):
                self.eSynchedFrontStamps[iFront[0].leftStamp] = iFront[0]
                self.eSynchedRearStamps [iRear[0].leftStamp]  = iRear[0]
                self.eSynchedNavStamps  [iNav[0].leftStamp]   = iNav[0]
                del(iFront[0])
                del(iRear[0])
                del(iNav[0])
            else:
                class SyncInfo:

                    def __init__(self, distToMedian, name, tol):
                        self.stamp = distToMedian
                        self.name = name
                        self.toKeep = True
                        if abs(self.stamp) <= tol:
                            self.synched = True
                        else:
                            self.synched = False

                    def __repr__(self):
                        return "SyncInfo()"

                    def __str__(self):
                        return str(self.stamp) + " " + self.name + " synched=" + str(self.synched) + " toKeep=" + str(self.toKeep)

                m = median([iFront[0].leftStamp, iRear[0].leftStamp, iNav[0].leftStamp])
                syncInfo = []
                syncInfo.append(SyncInfo(iFront[0].leftStamp - m, 'iFront', tolerance))
                syncInfo.append(SyncInfo(iRear[0].leftStamp  - m, 'iRear',  tolerance))
                syncInfo.append(SyncInfo(iNav[0].leftStamp   - m, 'iNav',   tolerance))

                # Sort syncInfo by stamp values
                syncInfo.sort(key=lambda x: x.stamp)

                if syncInfo[0].synched:
                    if syncInfo[2].synched:
                        raise Exception("Fatal error extrinsic_synchro")
                    else:
                        syncInfo[0].toKeep = False
                        syncInfo[1].toKeep = False
                else:
                    if syncInfo[2].synched:
                        syncInfo[0].toKeep = False
                    else:
                        syncInfo[0].toKeep = False
                        syncInfo[1].toKeep = False
                        syncInfo[2].toKeep = False

                for inf in syncInfo:
                    if not inf.toKeep:
                        cam = list(inf.name)
                        cam[0] = " " 
                        print("    Removing " + str(eval(inf.name)[0].index) + "".join(cam))
                        del(eval(inf.name)[0])

    def export_extrinsic_synchronized_index_file(self, filename):

        front = list(self.eSynchedFrontStamps.values())
        rear  = list(self.eSynchedRearStamps.values())
        nav   = list(self.eSynchedNavStamps.values())
        
        exportFile = open(filename, "w")
        for i in range(len(self.eSynchedFrontStamps)):
            exportFile.write(str(front[i].index) + " " + str(rear[i].index) + " " + str(nav[i].index) + "\n")
        exportFile.close()



