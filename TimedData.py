import numpy as np
import Quaternion
import time
from __builtin__ import classmethod
import math
import Utils

class TimedData:
    # Data numpy array
    d = 0
    # Number of columns of d / Number of data properties (e.g. 8 for t(1), pos(3), quat(4))
    Nc = 0
    # Index of last rows of d that has an entry
    last = 0
    timeID = 0
    
    def __init__(self, Nc):
        self.Nc = Nc; 
        self.last = -1;
        self.d = np.empty([10,Nc]);
    
    def initEmptyFromTimes(self, times): #TESTED
        self.last = np.shape(times)[0]-1
        self.d = np.zeros([np.shape(times)[0], self.Nc])
        self.d[:self.end(),self.timeID] = times;
    
    def cropTimes(self, t0, t1):
        indices, = np.nonzero(np.logical_and(self.getTime() >= t0,self.getTime() <= t1))
        self.d = np.take(self.D(), indices, axis=0)
        self.last = np.size(indices)-1
    
    def setColumnToSine(self, colID, amplitude, frequency, phaseShift):
        self.setCol(amplitude * np.sin(2 * np.pi * frequency * self.getTime() + phaseShift), colID)
    
    def setCol(self, data, colID): #TESTED
        if(np.shape(data)[0] == self.end()):
            if(colID < self.Nc):
                self.d[:self.end(),colID] = data;
            else:
                print('WARNING: Did not set column! No column with that ID!')
        else:
            print('WARNING: Did not set column! Wrong data size!');
    
    def setCols(self, data, colIDs): #TESTED
        if(np.shape(data)[0] == self.end()):
            if(np.all(colIDs) < self.Nc):
                self.d[:self.end(),colIDs] = data;
            else:
                print('WARNING: Did not set columns! No column with that ID!')
        else:
            print('WARNING: Did not set columns! Wrong data size!');
    
    def setRow(self, data, rowID): #TESTED
        if(np.shape(data)[0] == self.Nc):
            if(rowID < self.end()):
                self.d[rowID,:] = data;
            else:
                print('WARNING: Did not set row! No row with that ID!')
        else:
            print('WARNING: Did not set row! Wrong data size!');
            
    def setBlock(self, data, rowID, colID):
        if(np.shape(data)[0] <= (self.end()-rowID) ):
            if(np.shape(data)[1] <= (self.Nc-colID) ): 
                self.d[rowID:rowID+np.shape(data)[0],colID:colID+np.shape(data)[1]] = data;
            else:
                print('WARNING: Did not set block! Columns exceed matrix size!')
        else:
            print('WARNING: Did not set block! Rows exceed matrix size!');
    
    # These getter are used to access the "meaningful" data
    def D(self): #TESTED
        return self.d[:self.end(),];
    
    def col(self, colID): #TESTED
        # Check colID validity
        if (colID > (np.shape(self.d)[1]-1) ):
            print('WARNING: You requested an invalid colID = '+str(colID) + '. Returning Zeros!');
            return np.zeros([self.end()]); 
        # Return data
        return self.d[0:self.end(), colID];
    
    def cols(self, colIDs): #TESTED
        # Check colIDs validity
        if (max(colIDs) > (np.shape(self.d)[1]-1) ):
            print('WARNING: You requested an invalid colIDs = '+str(colIDs) + '. Returning Zeros!');
            return np.zeros([self.end()]); 
        # Return data
        return self.d[0:self.end(), colIDs];

    def row(self, rowID): #TESTED
        # Check rowID validity
        if(rowID > self.last):
            print('WARNING: You requested an invalid rowID = '+str(rowID) + '. Returning Zeros!');
            return np.zeros([self.Nc]); 
        # Return data
        return self.d[rowID,:];     
   
    def getTime(self):
        return self.col(self.timeID)
   
    def getLastTime(self):
        return self.col(self.timeID)[-1]
    
    def getFirstTime(self):
        return self.col(self.timeID)[0]
   
    # Append an entry, resize array
    def append(self): #TESTED
        if np.shape(self.d)[0] == self.end():
            self.d = np.resize(self.d, (2*np.shape(self.d)[0], self.Nc));   
        self.last += 1;

    def end(self): #TESTED
        return (self.last + 1);
    
    def length(self): #TESTED
        return (self.last + 1);
    
    # Math functions
    def computeNormOfColumns(self, colID, normID):
        self.setCol(Utils.norm(self.cols(colID)), normID)
        
    def computeDerivativeOfColumn(self, dataID, derivativeID, a=1, b=0): #TESTED
        dp = self.col(dataID)[a+b:self.end()]-self.col(dataID)[0:self.end()-(a+b)]
        dt = self.getTime()[a+b:self.end()]-self.getTime()[0:self.end()-(a+b)]
        self.d[0:a,derivativeID].fill(0)
        self.d[self.end()-b:self.end(),derivativeID].fill(0)
        self.d[a:self.end()-b,derivativeID] = np.divide(dp,dt);

    def computeVectorNDerivative(self, inputIDs, outputIDs, a=1, b=0): #TESTED
        if(len(inputIDs)==len(outputIDs)):
            dt = self.getTime()[a+b:self.end()]-self.getTime()[0:self.end()-(a+b)]
            for i in xrange(0,len(inputIDs)):
                dp = self.col(inputIDs[i])[a+b:self.end()]-self.col(inputIDs[i])[0:self.end()-(a+b)]
                self.d[0:a,outputIDs[i]].fill(0)
                self.d[self.end()-b:self.end(),outputIDs[i]].fill(0)
                self.d[a:self.end()-b,outputIDs[i]] = np.divide(dp,dt);
        else:
            print('WARNING: Compute N derivative failed! ColIDs did not match in size.')
    
    def computeVelocitiesInBodyFrameFromPostionInWorldFrame(self, posIDs, outputIDs, qIDs, a=1, b=0):
        self.computeVectorNDerivative(posIDs, outputIDs, a, b)
        self.setCols(Quaternion.q_rotate(self.cols(qIDs), self.cols(outputIDs)),outputIDs);
    
    def computeRotationalRateFromAttitude(self, attitudeID, rotationalrateID, a=1, b=0):
        dv = Quaternion.q_boxMinus(self.d[a+b:self.end(),attitudeID:attitudeID+4],self.d[0:self.end()-(a+b),attitudeID:attitudeID+4])
        dt = self.getTime()[a+b:self.end()]-self.getTime()[0:self.end()-(a+b)]
        for i in np.arange(0,3):
            self.d[0:a,rotationalrateID+i].fill(0)
            self.d[self.end()-b:self.end(),rotationalrateID+i].fill(0)
            self.d[a:self.end()-b,rotationalrateID+i] = np.divide(dv[:,i],dt);
        
    def interpolateColumns(self, other, colIDs, otherColIDs=None): #TESTED
        # Allow interpolating in to other columns / default is into same column
        if otherColIDs is None:
            otherColIDs = colIDs
        # ColIDs must match in size to be interpolated
        if(len(colIDs)==len(otherColIDs)):
            for i in xrange(0,len(colIDs)):
                self.interpolateColumn(other, colIDs[i], otherColIDs[i])
        else:
            print('WARNING: Interpolation failed! ColIDs did not match in size.')

    def interpolateColumn(self, other, colID, otherColID=None): #TESTED
        # NOTE: Values outside of the timerange of self are set to the first rsp. last value (no extrapolation)
        # Allow interpolating in to other columns / default is into same column
        if otherColID is None:
            otherColID = colID
        # When times are increasing interpolate using built in numpy interpolation
        if(np.all(np.diff(other.getTime()) > 0)):
            other.setCol(np.interp(other.getTime(), self.getTime(), self.col(colID)),otherColID)
        else:
            print('WARNING: Interpolation failed! Time values must be increasing.')
    
    def interpolateQuaternion(self, other, colID, otherColID=None):
        # All quaternions before self start are set to the first entry
        if otherColID is None:
            otherColID = colID
        counterOut = 0;
        counter = 0;
        while other.getTime()[counterOut] <= self.getTime()[counter]:
            other.d[counterOut,otherColID:otherColID+4] = self.d[counter,colID:colID+4]
            counterOut += 1
            
        # Interpolation
        while(counterOut < other.length()):
            while(self.getTime()[counter] < other.getTime()[counterOut] and counter < self.last):
                counter += 1
            counter
            if (counter != self.last):
                d = (other.getTime()[counterOut]-self.getTime()[counter-1])/(self.getTime()[counter]-self.getTime()[counter-1]);
                other.d[counterOut,otherColID:otherColID+4] = Quaternion.q_slerp(self.d[counter-1,colID:colID+4], self.d[counter,colID:colID+4], d)   
            else:
                # Set quaternion equal to last quaternion constant extrapolation
                other.d[counterOut,otherColID:otherColID+4] = self.d[counter,colID:colID+4];
            counterOut +=1
        
    def getTimeOffset(self,colID, other, otherColID=None):
        paddingWithRMS = False
        weightingWithOverlapLength = False
        if otherColID is None:
            otherColID = colID
        # Make timing calculation
        dtIn = other.getLastTime()-other.getFirstTime()
        dt = self.getLastTime()-self.getFirstTime()
        timeIncrement = min(dt/(self.length()-1), dtIn/(other.length()-1))
        N = math.floor(dt/timeIncrement)+1
        NIn = math.floor(dtIn/timeIncrement)+1
        # Interpolate of trajectories
        td1 = TimedData(2);
        td2 = TimedData(2);
        if paddingWithRMS:
            td1.initEmptyFromTimes(self.getFirstTime()+np.arange(-(NIn-1),N+(NIn-1))*timeIncrement)
        else:
            td1.initEmptyFromTimes(self.getFirstTime()+np.arange(N)*timeIncrement)
        td2.initEmptyFromTimes(other.getFirstTime()+np.arange(NIn)*timeIncrement)
        self.interpolateColumn(td1, colID, 1)
        other.interpolateColumn(td2, otherColID, 1)
        # Padding with RMS
        if paddingWithRMS:
            RMS = np.sqrt(np.mean(np.square(self.col(1))))
            td1.setBlock(np.ones([NIn-1,1])*RMS,0,1)
            td1.setBlock(np.ones([NIn-1,1])*RMS,N+(NIn-1),1)        
        # Calc COnvolution
        if paddingWithRMS:
            conv = np.correlate(td1.D()[:,1], td2.D()[:,1], 'valid')
        else:
            conv = np.correlate(td1.D()[:,1], td2.D()[:,1], 'full')
        if weightingWithOverlapLength:
            w = np.minimum(np.minimum(np.arange(1,N+NIn,1),np.ones(N+NIn-1)*NIn),np.arange(N+NIn-1,0,-1))
            conv = conv / w
        # Deviation of the maximum convolution from the middle of the convolution vector  
        n = np.argmax(conv) - NIn + 1
        return timeIncrement*n + self.getFirstTime() - other.getFirstTime()
    
    def applyTimeOffset(self,to):
        self.setCol(self.col(0) + to,0)
        
    def applyBodyTransform(self, posID, attID, translation, rotation):
        newTranslation = self.cols(np.arange(posID,posID+3)) \
                         + Quaternion.q_rotate(Quaternion.q_inverse(self.cols(np.arange(attID,attID+4))),
                                               np.kron(np.ones([self.length(),1]),translation))
        newRotation = Quaternion.q_mult(np.kron(np.ones([self.length(),1]),rotation),
                                        self.cols(np.arange(attID,attID+4)))
        for i in np.arange(0,3):
            self.setCol(newTranslation[:,i],posID+i)
        for i in np.arange(0,4):
            self.setCol(newRotation[:,i],attID+i)
    
    def applyBodyTransformToTwist(self, velID, rorID, translation, rotation):
        newVel = Quaternion.q_rotate(np.kron(np.ones([self.length(),1]),rotation),
                                     self.cols(np.arange(velID,velID+3)) \
                                     + np.cross(self.cols(np.arange(rorID,rorID+3)),np.kron(np.ones([self.length(),1]),translation)))
        newRor = Quaternion.q_rotate(np.kron(np.ones([self.length(),1]),rotation),
                                     self.cols(np.arange(rorID,rorID+3)))
        for i in np.arange(0,3):
            self.setCol(newVel[:,i],velID+i)
            self.setCol(newRor[:,i],rorID+i)
        
    def applyInertialTransform(self, posID, attID, translation, rotation):
        newTranslation = np.kron(np.ones([self.length(),1]),translation) \
                         + Quaternion.q_rotate(np.kron(np.ones([self.length(),1]),Quaternion.q_inverse(rotation)),
                                               self.cols(np.arange(posID,posID+3)))
        newRotation = Quaternion.q_mult(self.cols(np.arange(attID,attID+4)),
                                        np.kron(np.ones([self.length(),1]),rotation))
        for i in np.arange(0,3):
            self.setCol(newTranslation[:,i],posID+i)
        for i in np.arange(0,4):
            self.setCol(newRotation[:,i],attID+i)
        
    def invertRotation(self, attID):
        newRotation = Quaternion.q_inverse(self.cols(np.arange(attID,attID+4)))
        for i in np.arange(0,4):
            self.setCol(newRotation[:,i],attID+i)
        
    def invertTransform(self, posID, attID):
        newTranslation = -Quaternion.q_rotate(self.cols(np.arange(attID,attID+4)),self.cols(np.arange(posID,posID+3)))
        for i in np.arange(0,3):
            self.setCol(newTranslation[:,i],posID+i)
        self.invertRotation(attID)
    
    def calibrateBodyTransform(self, velID1, rorID1, other, velID2, rorID2):
        # Make timing calculation
        dt1 = self.getLastTime()-self.getFirstTime()
        dt2 = other.getLastTime()-other.getFirstTime()
        first = max(self.getFirstTime(),other.getFirstTime())
        last = min(self.getLastTime(),other.getLastTime())
        timeIncrement = min(dt1/(self.length()-1), dt2/(other.length()-1))
        td1 = TimedData(7);
        td2 = TimedData(7);
        td1.initEmptyFromTimes(np.arange(first,last,timeIncrement))
        td2.initEmptyFromTimes(np.arange(first,last,timeIncrement))
        self.interpolateColumns(td1, [velID1+0,velID1+1,velID1+2,rorID1+0,rorID1+1,rorID1+2], [1,2,3,4,5,6])
        other.interpolateColumns(td2, [velID2+0,velID2+1,velID2+2,rorID2+0,rorID2+1,rorID2+2], [1,2,3,4,5,6])
        AA = np.zeros([4,4])
        # TODO: Speed up by removing for loop
        for i in np.arange(0,td1.length()):
            q1 = np.zeros(4)
            q2 = np.zeros(4)
            q1[1:4] = td1.D()[i,4:7];
            q2[1:4] = td2.D()[i,4:7];
            A = Quaternion.q_Rmat(q1)-Quaternion.q_Lmat(q2)
            AA += A.T.dot(A)
        w, v = np.linalg.eigh(AA)
        rotation = v[:,0]
        
        # Find transformation
        A = np.zeros([3*td1.length(),3])
        b = np.zeros([3*td1.length()])
        for i in np.arange(0,td1.length()):
            A[3*i:3*i+3,] = Utils.skew(td1.D()[i,4:7])
            b[3*i:3*i+3] = Quaternion.q_rotate(Quaternion.q_inverse(rotation), td2.D()[i,1:4])-td1.D()[i,1:4]
        translation = np.linalg.lstsq(A, b)[0]
        
        return translation, rotation
        
    def calibrateInertialTransform(self, posID1, attID1, other, posID2, attID2, B_r_BC, q_CB, calIDs=[0]):        
        # Make timing calculation 
        dt1 = self.getLastTime()-self.getFirstTime()
        dt2 = other.getLastTime()-other.getFirstTime()
        first = max(self.getFirstTime(),other.getFirstTime())
        last = min(self.getLastTime(),other.getLastTime())
        timeIncrement = min(dt1/(self.length()-1), dt2/(other.length()-1))
        td1 = TimedData(8);
        td2 = TimedData(8);
        td1.initEmptyFromTimes(np.arange(first,last,timeIncrement))
        td2.initEmptyFromTimes(np.arange(first,last,timeIncrement))
        self.interpolateColumns(td1, [posID1+0,posID1+1,posID1+2], [1,2,3])
        other.interpolateColumns(td2, [posID2+0,posID2+1,posID2+2], [1,2,3])
        self.interpolateQuaternion(td1, attID1, 4)
        other.interpolateQuaternion(td2, attID2, 4)
        
        newIDs = np.arange(0,len(calIDs))
        q_CB_vec = np.kron(np.ones([len(calIDs),1]),q_CB)
        q_JC_vec = Quaternion.q_inverse(td2.D()[newIDs,4:8])
        q_BI_vec = td1.D()[newIDs,4:8]
        B_r_BC_vec = np.kron(np.ones([len(calIDs),1]),B_r_BC)
        J_r_JC = td2.D()[newIDs,1:4];
        J_r_BC = Quaternion.q_rotate(Quaternion.q_mult(q_JC_vec,q_CB_vec), B_r_BC_vec)
        J_r_IB = Quaternion.q_rotate(Quaternion.q_mult(q_JC_vec,Quaternion.q_mult(q_CB_vec,q_BI_vec)),td1.D()[newIDs,1:4])
        
        rotation = Quaternion.q_mean(Quaternion.q_inverse(Quaternion.q_mult(Quaternion.q_mult(q_JC_vec,q_CB_vec),q_BI_vec)))
        translation = np.mean(J_r_JC-J_r_BC-J_r_IB, axis=0)
        return translation.flatten(), rotation.flatten()
    
    def computeSigmaBounds(self, dataID, covID, plusID, minusID, factor):
        if(type(dataID) is int):
            dataID = [dataID]
            covID = [covID]
            plusID = [plusID]
            minusID = [minusID]
        for i in xrange(0,len(dataID)):
            self.setCol(self.col(dataID[i])+factor*self.col(covID[i])**(1./2),plusID[i])
            self.setCol(self.col(dataID[i])-factor*self.col(covID[i])**(1./2),minusID[i])
            
    def quaternionToYpr(self, attIDs, yprIDs):
        self.d[0:self.end(),yprIDs] = Quaternion.q_toYpr(self.cols(attIDs))
            
    def quaternionToYprCov(self, attIDs, attCovIDs, yprCovIDs):
        J = Quaternion.q_toYprJac(self.cols(attIDs))
        
        # Do the multiplication by hand, improve if possible
        for i in xrange(0,self.length()):
            self.d[i,yprCovIDs] = np.resize(np.dot(np.dot(np.resize(J[i,:],(3,3)),np.resize(self.cols(attCovIDs)[i,:],(3,3))),np.resize(J[i,:],(3,3)).T),(9))
            
    def applyBodyTransformToAttCov(self, attCovIDs, rotation):
        R = np.resize(Quaternion.q_toRotMat(rotation),(3,3))
        # Do the multiplication by hand, improve if possible
        for i in xrange(0,self.length()):
            self.d[i,attCovIDs] = np.resize(np.dot(np.dot(R,np.resize(self.d[i,attCovIDs],(3,3))),R.T),(9))
    
    def computeLeutiScore(self, posID1, attID1, velID1, other, posID2, attID2, distances, spacings, start):
        output = np.empty((len(distances),6))
        outputFull = []
        startIndex = np.nonzero(self.getTime() >= start)[0][0]
        
        for j in np.arange(len(distances)):
            tracks = [[startIndex, 0.0]]
            lastAddedStart = 0.0
            posErrors = []
            attErrors = []
        
            other_interp = TimedData(8)
            other_interp.initEmptyFromTimes(self.getTime())
            other.interpolateColumns(other_interp, posID2, [1,2,3])
            other.interpolateQuaternion(other_interp, attID2[0], 4)
            
            it = startIndex
            while it+1<self.last:
                # Check for new seeds
                if(lastAddedStart>spacings[j]):
                    tracks.append([it, 0.0])
                    lastAddedStart = 0.0
                # Add travelled distance
                d = np.asscalar(Utils.norm(self.d[it,velID1]*(self.d[it+1,0]-self.d[it,0])))
                lastAddedStart += d
                tracks = [[x[0], x[1]+d] for x in tracks]
                # Check if travelled distance large enough
                while len(tracks) > 0 and tracks[0][1] > distances[j]:
                    pos1 = Quaternion.q_rotate(self.d[tracks[0][0],attID1], self.d[it+1,posID1]-self.d[tracks[0][0],posID1])
                    pos2 = Quaternion.q_rotate(other_interp.d[tracks[0][0],[4,5,6,7]], other_interp.d[it+1,[1,2,3]]-other_interp.d[tracks[0][0],[1,2,3]])
                    att1 = Quaternion.q_mult(self.d[it+1,attID1], Quaternion.q_inverse(self.d[tracks[0][0],attID1]))
                    att2 = Quaternion.q_mult(other_interp.d[it+1,[4,5,6,7]], Quaternion.q_inverse(other_interp.d[tracks[0][0],[4,5,6,7]]))
                    posErrors.append(np.asscalar(np.sum((pos2-pos1)**2,axis=-1)))
                    attErrors.append(np.asscalar(np.sum((Quaternion.q_boxMinus(att1,att2))**2,axis=-1)))
                    tracks.pop(0)
                it += 1
            
            N = len(posErrors)
            posErrorRMS = (np.sum(posErrors,axis=-1)/N)**(0.5)
            posErrorMedian = np.median(posErrors)**(0.5)
            posErrorStd = np.std(np.array(posErrors)**(0.5))
            attErrorRMS = (np.sum(attErrors,axis=-1)/N)**(0.5)
            attErrorMedian = np.median(attErrors)**(0.5)
            attErrorStd = np.std(np.array(attErrors)**(0.5))
            print('Evaluated ' + str(N) + ' segments with a travelled distance of ' + str(distances[j]) \
                   + ':\n  posRMS of ' + str(posErrorRMS) + ' (Median: ' + str(posErrorMedian) + ', Std: ' + str(posErrorStd) + ')' \
                   + '\n  attRMS of ' + str(attErrorRMS) + ' (Median: ' + str(attErrorMedian) + ', Std: ' + str(attErrorStd) + ')')
            output[j,:] = [posErrorRMS, posErrorMedian, posErrorStd, attErrorRMS, attErrorMedian, attErrorStd]
            outputFull.append(np.array(posErrors)**(0.5))
        return outputFull


