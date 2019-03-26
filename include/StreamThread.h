#ifndef STREAMTHREAD_H
#define STREAMTHREAD_H

#include "BoostArchiver.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Map.h"

#include <queue>

namespace ORB_SLAM2{

class Map;

class SendClassToServer
{
public:
    SendClassToServer(){}
    SendClassToServer(ros::Publisher _data_pub, KeyFrame *_pKF, Map *pMap);
    SendClassToServer(ros::NodeHandle _nh, int cid, Map *pMap);
    void SetPublisher(ros::Publisher _data_pub);
    void SetKeyFrame(KeyFrame *_pKF);
    void SetMapPoint(MapPoint *_pMP);
    void EraseKeyFrame(KeyFrame *_pKF);
    void EraseMapPoint(MapPoint *_pMP);
    void UpdateKeyFrame(KeyFrame *_pKF);
    void UpdateMapPoint(MapPoint *_pMP);
    void RunKeyFrame(int command);
    void RunMapPoint(int command);
    void RunMapPoint(MapPoint* _pMP, int command);
    void Run();
    int GetClientId();

    void ActivateConnectionToServer();

    queue<KeyFrame*> mvpKF;
    queue<MapPoint*> mvpMP;
    queue<KeyFrame*> mvpUKF;
    queue<MapPoint*> mvpUMP;
    queue<KeyFrame*> mvpDKF;
    queue<MapPoint*> mvpDMP;

    void RequestFinish();
    bool isFinished();

private:
    KeyFrame *pKF;
    MapPoint *pMP;
    Map *mpMap;

    ros::NodeHandle n;
    ros::Publisher data_pub;
    ros::Publisher kf_data_pub;
    ros::Publisher mp_data_pub;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    int ClientId;
};

}

#endif 