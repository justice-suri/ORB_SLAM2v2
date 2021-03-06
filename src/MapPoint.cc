/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>
#include <stdlib.h>
#include <time.h>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0),mbTrackInView(false), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    //struct timespec ts;
    //clock_gettime(CLOCK_MONOTONIC, &ts);
    //srand((time_t)ts.tv_nsec);
    //UID = rand() + pMap->GetAllMapPoints().size();
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    UID=mnId=nNextId++;
}

MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0),mbTrackInView(false), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    UID = -2;
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    //mnId=nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;
    if(pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
}

void MapPoint::SetObservation(){
    mpRefKF = mObservations.begin()->first;
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
            if(pKF->mvuRight[idx]>=0)
                nObs-=2;
            else
                nObs--;

            mObservations.erase(pKF);

            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            /*cout << "pKF : " << pKF->mnId << "\t";
            cout << "UID : " << UID << "\t";
            cout << "nObs:" << nObs << "\t";
            cout << "mObservations size : " << mObservations.size() << "\t";
            if(mpRefKF == NULL){
                cout << "mpRefKF : NULL" << endl;
            }else{
                cout << "mpRefKF : " << mpRefKF->mnId << endl;
            }*/

            // If only 2 observations or less, discard point
            if(nObs<=2 || mpRefKF == NULL)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }
    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    unique_lock<mutex> lock(mGlobalMutex);
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    if(mbTest)
        cout << "[MapPoint] Retrieve all observed descriptors" << endl;
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        if(mbTest)
            cout << "[MapPoint] pKF = " << pKF->mnId << " mDescriptors.rows = " << pKF->mDescriptors.rows << " size_t = " << mit->second << endl;
        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;
    if(mbTest)
        cout << "Compute distances between them" << endl;
    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }
    if(mbTest)
        cout << "Take the descriptor with least median distance to the rest" << endl;
    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }
    if(mpRefKF == NULL){
        for(map<KeyFrame*,size_t>::iterator mit = observations.begin(), mend = observations.end(); mit!=mend; mit++){
            if(mbTest){
            if(mit->first == NULL){
                cout << "[mpRefKF = NULL]: observations : NULL" << endl;
                cout << "UID : " << UID << endl;
                cout << "nNextId : " << nNextId << endl;
            }
            else
            {
                cout << "[mpRefKF = NULL]: observations : " << mit->first->mnId << endl;
                cout << "UID : " << UID << endl;
                cout << "nNextId : " << nNextId << endl;
                cout << "isBad : " << mbBad << endl;
            }
            }
            
        }
    }

    if(observations.empty())
        return;
    
    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        //cout << "Owi : " << Owi << endl;
        cv::Mat normali = mWorldPos - Owi;
        //cout << "normali : " << normali << endl;
        normal = normal + normali/cv::norm(normali);
        n++;
    }
    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    //cout << "dist : " << dist << endl;
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    //cout << "level : " << level << endl;
    //cout << "mvScaleFactors : " << pRefKF->mvScaleFactors.size() << endl;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    //cout << "levelScaleFactor = " << levelScaleFactor << endl;
    const int nLevels = pRefKF->mnScaleLevels;
    //cout << "nLevels : " << nLevels << endl;
    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::GetVisible(){
    return mnVisible;
}

int MapPoint::GetmnFound(){
    return mnFound;
}

void MapPoint::getMap(Map* pMap){
    mpMap = pMap;
}

MapPoint::MapPoint(ServerMapPoint *smp, Map* pMap):
    mnId(smp->mnId), UID(smp->GetUID()), nObs(smp->nObs), mnTrackReferenceForFrame(smp->mnTrackReferenceForFrame),
    mnLastFrameSeen(smp->mnLastFrameSeen), mnBALocalForKF(smp->mnBALocalForKF), mnFuseCandidateForKF(smp->mnFuseCandidateForKF), mnLoopPointForKF(smp->mnLoopPointForKF), mnCorrectedByKF(smp->mnCorrectedByKF),
    mnCorrectedReference(smp->mnCorrectedReference), mnBAGlobalForKF(smp->mnBAGlobalForKF),mnVisible(smp->mnVisible), mnFound(smp->mnFound), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(smp->mfMinDistance), mfMaxDistance(smp->mfMaxDistance), mpMap(pMap),
    mnFirstKFid(smp->mnFirstKFid), mnFirstFrame(smp->mnFirstFrame)
{
    mWorldPos = smp->mWorldPos.clone();
    mDescriptor = smp->mDescriptor.clone();
    mPosGBA = smp->mPosGBA.clone();
}

MapPoint::MapPoint():
    nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0),mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0)
{}
template<class Archive>
void MapPoint::serialize(Archive &ar, const unsigned int version)
{
    ar & UID & mnId & nNextId & mnFirstKFid & mnFirstFrame & nObs;
    if(mbTest)
        cout << "Tracking related vars" << endl;
    // Tracking related vars
    ar & mTrackProjX;
    ar & mTrackProjY;
    ar & mTrackProjXR;
    ar & mbTrackInView;
    ar & mnTrackScaleLevel;
    ar & mTrackViewCos;
    ar & mnTrackReferenceForFrame;
    ar & mnLastFrameSeen;
    if(mbTest)
        cout << "Local Mapping related vars" << endl;
    // Local Mapping related vars
    ar & mnBALocalForKF & mnFuseCandidateForKF;
    if(mbTest)
        cout << "LoopClising related vars" << endl;
    // Loop Closing related vars
    if(mbTest)
        cout << "don't save the mutex" << endl;
    ar & mnLoopPointForKF & mnCorrectedByKF & mnCorrectedReference & mPosGBA & mnBAGlobalForKF;
    // don't save the mutex
    ar & mWorldPos;
    ar & mObservations;
    ar & mNormalVector;
    ar & mDescriptor;
    ar & mpRefKF;
    ar & mnVisible & mnFound;
    ar & mbBad & mpReplaced;
    ar & mfMinDistance & mfMaxDistance;
    ar & mpMap;
    // don't save the mutex
}
template void MapPoint::serialize(boost::archive::binary_iarchive&, const unsigned int);
template void MapPoint::serialize(boost::archive::binary_oarchive&, const unsigned int);


} //namespace ORB_SLAM
