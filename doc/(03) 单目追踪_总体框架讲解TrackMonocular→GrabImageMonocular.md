# 单目追踪_总体框架讲解TrackMonocular→GrabImageMonocular

### 一、前言

前面我们使用深度图像调试，并且进行了简单的讲解。但是深度图涉及的东西没有单目图像多，为了大家学习到更多的东西，接下来使用我们使用单目图像进行讲解。根据前面的博客，运行单目摄像头的代码从 ./Examples/Monocular/mono_tum.cc 中的 main 函数开始，其中的主要核心代码如下

```cpp
	//循环,读取图像进行追踪
	for(int ni=0; ni<nImages; ni++)
		//读取图像获得像素
		im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
		//根据输入的图像，进行单目追踪
        SLAM.TrackMonocular(im,tframe);
    // 停止所有线程
    SLAM.Shutdown();

```

### 二、TrackMonocular（单目追踪用函数）

在 src\System.cc 文件中，我们可以看到 TrackMonocular 函数实现的具体过程。在讲解该函数时，我们先回顾一下上一节我们讲解的 ORB_SLAM2::System 构造函数，其可以看到如下代码：

```cpp
	//追踪器，负责追踪的一些相关操作
    mpTracker = new Tracking(this,mpVocabulary,mpFrameDrawer,mpMapDrawer,mpMap,mpKeyFrameDatabase,strSettingsFile,mSensor);
    //局部建图器,负责局部地图的构建			
    mpLocalMapper = new LocalMapping(mpMap,mSensor==MONOCULAR);	
    //闭环器,闭环检测以及闭环操作
    mpLoopCloser = new LoopClosing(mpMap,mpKeyFrameDatabase,mpVocabulary,mSensor!=MONOCULAR);		
```

其上的类对象mpTracker、mpLocalMapper、mpLoopCloser都是十分重要的，是整个系统最最核心的3个类对象。回忆起我们上一章节讲解过的内容，那么我们再来看看 TrackMonocular 这个函数：

```cpp
//同理，输入为单目图像时的追踪器接口
cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)//图像+时间戳
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }
	
    //检查运行模式是否改变
    // Check mode change
    {
        // 独占锁，主要是为了mbActivateLocalizationMode和mbDeactivateLocalizationMode不会发生混乱
        unique_lock<mutex> lock(mMutexMode);
        // mbActivateLocalizationMode为true会关闭局部地图线程
        if(mbActivateLocalizationMode)
        {
            //调用局部建图器的请求停止函数
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }
			//运行到这里的时候，局部建图部分就真正地停止了
            //告知追踪器，现在 只有追踪工作
            // 局部地图关闭以后，只进行追踪的线程，只计算相机的位姿，没有对局部地图进行更新
            // 设置mbOnlyTracking为真
            mpTracker->InformOnlyTracking(true);// 定位时，只跟踪
            // 关闭线程可以使得别的线程得到更多的资源
            mbActivateLocalizationMode = false;// 防止重复执行
        }
        // 如果mbDeactivateLocalizationMode是true，局部地图线程就被释放, 关键帧从局部地图中删除.
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    //获取相机位姿的估计结果
     //用矩阵Tcw来保存估计的相机 位姿，运动追踪器的GrabImageMonocular函数才是真正进行运动估计的函数
    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);
    
	//给运动追踪状态上锁
    unique_lock<mutex> lock2(mMutexState);
    //获取运动追踪状态
    mTrackingState = mpTracker->mState;
    //获取当前帧追踪到的地图点向量指针
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    //获取当前帧追踪到的关键帧特征点向量的指针
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    
	//返回获得的相机运动估计
    return Tcw;
}
```

其上的操作总体来说还是比较简单的，主要流程如下:

	1、判断传感器的类型是否为单目模式，如果不是，则表示设置错误，函数直接返回
	
	2、上锁 模式锁(mMutexMode):
		(1)如果目前需要激活定位模式，则请求停止局部建图，并且等待局部建图线程停止，设置为仅追踪模式。
		(2)如果目前需要取消定位模式，则通知局部建图可以工作了，关闭仅追踪模式
		
	3、上锁 复位锁(mMutexReset): 检查是否存在复位请求，如果有，则进行复位操作
	
	4、核心部分: 根据输入的图像获得相机位姿态（其中包含了特征提取匹配，地图初始化，关键帧查询等操作）
	
	5、进行数据更新，如追踪状态、当前帧的地图点、当前帧矫正之后的关键点等。
其上的核心部分为 根据输入的图像获得相机位姿态，也就是函数 GrabImageMonocular()。

### 三、GrabImageMonocular（真正进行运动估计的函数）

```cpp
	 cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);
```

其主要进行了操作:

	1、如果输入的图像不为灰度图，则转换为灰度图。
	
	2、根据是否为第一帧或者或者是否进行初始化，使用不同的参数(提取的特征点数目)进行Frame类的创建
	
	3、Track(); 进行追踪
其代码注释如下（Tracking.cc）

```cpp
cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im,const double &timestamp)	
    //单目图像 + 时间戳
{
    mImGray = im;

    // Step 1 ：将彩色图像转为灰度图像
    //若图片是3、4通道的，还需要转化成灰度图
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    // Step 2 ：构造Frame
    //判断当前是否进行了初始化，如果当前为第一帧，则 mState==NO_IMAGES_YET，表示没有进行初始化
    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET) //没有成功初始化的前一个状态就是NO_IMAGES_YET
        mCurrentFrame = Frame(
            mImGray,
            timestamp,
            mpIniORBextractor,      //初始化ORB特征点提取器会提取2倍的指定特征点数目
            mpORBVocabulary,
            mK,
            mDistCoef,
            mbf,
            mThDepth);
    else
        mCurrentFrame = Frame(
            mImGray,
            timestamp,
            mpORBextractorLeft,     //正常运行的时的ORB特征点提取器，提取指定数目特征点
            mpORBVocabulary,
            mK,
            mDistCoef,
            mbf,
            mThDepth);

    // Step 3 ：跟踪
    Track();

    //返回当前帧的位姿
    return mCurrentFrame.mTcw.clone();
}

```

### 四、结语

从 GrabImageMonocular 函数中，我们可以看到其最核心的部分应该存在于 Track() 函数之中，但是其上的 Frame 创建也是十分重要的，其中做了很多追踪需要的预备工作，如图像金字塔、特征提取，关键点矫正、特征点均匀分布等操作，其也是我们接下来学习的主要对象。