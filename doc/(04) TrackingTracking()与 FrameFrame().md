# (04) Tracking::Tracking()与 Frame::Frame()

### 一、前言

上一节我们说到，Frame 的构建是比较重要的一个环节，但是再讲解其之前，我们再补充一点内容，那就是 src\Tracking.cc 中 Tracking 的构造函数，该类对象被是在 src\System.cc 的初始化函数中被创建的，如下；

```cpp
//追踪器，负责追踪的一些相关操作
mpTracker = new Tracking(this,mpVocabulary,mpFrameDrawer,mpMapDrawer,mpMap,mpKeyFrameDatabase,strSettingsFile,mSensor);
```
### 二、Tracking 构造函数（Tracking.cc）

Tracking 的构造函数主要进行了如下操作；

	1、根据配置文件(如"Examples/RGB-D/TUM1.yaml")中获得相机参数:
		(1)内参矩阵，矫正系数，帧率等信息
		(2)如果为双目，则还需要获得 Camera.bf参数
	
	2、根据配置文件获得特征提取的相关参数:
		(1)每帧图像提取关键点总数目
		(2)金字塔层数与缩放尺度
		(3)提取fast特征点的相关参数
	
	3、根据特征提取的相关配置创建特征提取类ORBextractor对象
		(1)所有传感器都是需要创建左目特征提取器 mpORBextractorLeft
		(2)如果为双目则需要创建右目特征提取器 mpORBextractorRight
		(3)如果为单目则需要额外创建一个初始化特征提取器 mpIniORBextractor
其上提到的特征提取器，是比较重要的一部分，我们再下一个章节进行讲解，关于 Tracking::Tracking 构造函数的注释代码如下:

```cpp
///构造函数
Tracking::Tracking(
    System *pSys,                       //系统实例
    ORBVocabulary* pVoc,                //BOW字典
    FrameDrawer *pFrameDrawer,          //帧绘制器
    MapDrawer *pMapDrawer,              //地图点绘制器
    Map *pMap,                          //地图句柄
    KeyFrameDatabase* pKFDB,            //关键帧产生的词袋数据库
    const string &strSettingPath,       //配置文件路径
    const int sensor):                  //传感器类型
        mState(NO_IMAGES_YET),                              //当前系统还没有准备好
        mSensor(sensor),                                
        mbOnlyTracking(false),                              //处于SLAM模式
        mbVO(false),                                        //当处于纯跟踪模式的时候，这个变量表示了当前跟踪状态的好坏
        mpORBVocabulary(pVoc),          
        mpKeyFrameDB(pKFDB), 
        mpInitializer(static_cast<Initializer*>(NULL)),     //暂时给地图初始化器设置为空指针
        mpSystem(pSys), 
        mpViewer(NULL),                                     //注意可视化的查看器是可选的，因为ORB-SLAM2最后是被编译成为一个库，所以对方人拿过来用的时候也应该有权力说我不要可视化界面（何况可视化界面也要占用不少的CPU资源）
        mpFrameDrawer(pFrameDrawer),
        mpMapDrawer(pMapDrawer), 
        mpMap(pMap), 
        mnLastRelocFrameId(0)                               //恢复为0,没有进行这个过程的时候的默认值
{
    // Load camera parameters from settings file
    // Step 1 从配置文件中加载相机参数
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    //     |fx  0   cx|
    // K = |0   fy  cy|
    //     |0   0   1 |
    //构造相机内参矩阵
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    // 图像矫正系数
    // [k1 k2 p1 p2 k3]
    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    //有些相机的畸变系数中会没有k3项
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    // 双目摄像头baseline * fx 50
    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    //输出
    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;

    // 1:RGB 0:BGR
    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    // Step 2 加载ORB特征点有关的参数,并新建特征点提取器

    // 每一帧提取的特征点数 1000
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    // 图像建立金字塔时的变化尺度 1.2
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    // 尺度金字塔的层数 8
    int nLevels = fSettings["ORBextractor.nLevels"];
    // 提取fast特征点的默认阈值 20
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    // 如果默认阈值提取不出足够fast特征点，则使用最小阈值 8
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    // tracking过程都会用到mpORBextractorLeft作为特征点提取器
    mpORBextractorLeft = new ORBextractor(
        nFeatures,      //参数的含义还是看上面的注释吧
        fScaleFactor,
        nLevels,
        fIniThFAST,
        fMinThFAST);

    // 如果是双目，tracking过程中还会用用到mpORBextractorRight作为右目特征点提取器
    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    // 在单目初始化的时候，会用mpIniORBextractor来作为特征点提取器
    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        // 判断一个3D点远/近的阈值 mbf * 35 / fx
        //ThDepth其实就是表示基线长度的多少倍
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        // 深度相机disparity转化为depth时的因子
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

}

```

mpIniORBextractor 与 mpORBextractorLeft、mpORBextractorRight 的不同，主要在于 _nfeatures 参数的不同，也就是特征提取的数目不一样。

### 三、Frame 构造函数（Frame.cc）-在GrabImageMonocular中，负责预处理图像

我们已经对 Tracking 的构造函数进行了简单的补充，那么我们回到之前的代码src\Tracking.cc 中的 GrabImageMonocular() 函数，上一节我们说到其会根据输入的图像进行 Frame 的创建，如下:

```cpp
// Step 2 ：构造Frame
//判断当前是否进行了初始化，如果当前为第一帧，则 mState==NO_IMAGES_YET，表示没有进行初始化
if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET) //没有成功初始化的前一个状态就是NO_IMAGES_YET
    mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
else
    mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
```
其上我们可以看到，当前是否初始化创建出来的 Frame 对象是一样的，其主要区别为传入的特征提取器为 mpIniORBextractor 或者 mpORBextractorLeft。 该两个类对象都是再 Tracking::Tracking() 中被穿甲来的。我们暂且不去理会。我们先来看看 Frame 的构造函数。其主要进行了如下操作:

	1、根据特征提取器，获得图像金字塔的相关参数
	2、进行特征提取 → ExtractORB(0,imGray);
	3、关键点畸变矫正 → UndistortKeyPoints();
	4、如果为初始化帧，则还需要将特征点分配到网络之中。 
其主要代码注释如下:

```cpp
Frame::Frame(const cv::Mat &imGray, 		//灰度化后的彩色图像
             const double &timeStamp, 		//时间戳
             ORBextractor* extractor,		//ORB特征点提取器的句柄
             ORBVocabulary* voc, 			//ORB字典的句柄
             cv::Mat &K, 					//相机的内参数矩阵
             cv::Mat &distCoef, 			//相机的去畸变参数
             const float &bf, 				//baseline*f
             const float &thDepth)			//区分远近点的深度阈值
    :mpORBvocabulary(voc),
	mpORBextractorLeft(extractor),
	mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
	//因为单目图像没有这个右侧图像的定义，所以这里的右图像特征点提取器的句柄为空
   	mTimeStamp(timeStamp), mK(K.clone()), 
	mDistCoef(distCoef.clone()), mbf(bf), 
	mThDepth(thDepth)
{
    // Frame ID
	// Step 1 获取帧数ID
    mnId=nNextId++;

    // Step 2 计算图像金字塔的参数 
    // Scale Level Info
	//获取图像金字塔的层数
    mnScaleLevels = mpORBextractorLeft->GetLevels();
	//获取每层的缩放因子
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
	//计算每层缩放因子的自然对数
    mfLogScaleFactor = log(mfScaleFactor);
	//获取各层图像的缩放因子
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
	//获取各层图像的缩放因子的倒数
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
	//获取sigma^2
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
	//获取sigma^2的倒数
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
	// Step 3 对这个单目图像进行提取特征点, 第一个参数0-左图， 1-右图
    ExtractORB(0,imGray);

	//求出特征点的个数
    N = mvKeys.size();

	//如果没有能够成功提取出特征点，那么就直接返回了
    if(mvKeys.empty())
        return;

    // Step 4 用OpenCV的矫正函数、内参对提取到的特征点进行矫正 
    UndistortKeyPoints();

    // Set no stereo information
	// 由于单目相机无法直接获得立体信息，所以这里要给右图像对应点和深度赋值-1表示没有相关信息
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);


    //初始化存储地图点句柄的vector
    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
	// 记录地图点是否为外点，初始化均为外点false        
   	//开始认为默认的地图点均为inlier
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
	//  Step 5 计算去畸变后图像边界，将特征点分配到网格中。这个过程一般是在第一帧或者是相机标定参数发生变化之后进行
    /* 判断是否需要进行进行特殊初始化,这个过程一般是在第一帧或者是重定位之后进行.主要操作有:
     *      - 计算未校正图像的边界 Frame::ComputeImageBounds() 
     *      - 计算一个像素列相当于几个（<1）图像网格列
     *      - 给相机的内参数赋值
     *      - 标志复位
     */ 
    if(mbInitialComputations)
    {
		// 计算去畸变后图像的边界
        ComputeImageBounds(imGray);

		// 表示一个图像像素相当于多少个图像网格列（宽）
        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
		// 表示一个图像像素相当于多少个图像网格行（高）
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

		//给类的静态成员变量复制
        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
		// 猜测是因为这种除法计算需要的时间略长，所以这里直接存储了这个中间计算结果
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

		//特殊的初始化过程完成，标志复位
        mbInitialComputations=false;
    }

    //计算 basline
    mb = mbf/fx;

	// 将特征点分配到图像网格中 
    AssignFeaturesToGrid();
}

```

### 四、结语

通过前面的介绍，大家对于追踪线程有了大概了解，其中最重要最复杂的函数是 Tracking::GrabImageMonocular() 中调用的 Track() 函数，但是在他的前面的进行了 Frame 对象的创建，我们说其也是非常重要的，主要是因为他的构造函数之中做了图像金字塔，以及ORB特征提取等操作。这是追踪过程中必不可少的预备工作。那么从下篇博客开始，我们将介绍金字塔以及ORB特征的相关知识。