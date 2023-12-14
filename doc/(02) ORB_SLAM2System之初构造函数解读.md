# ORB_SLAM2::System之初构造函数解读

### 一、前言

通过上一篇博客，我们对 ./Examples/RGB-D/rgbd_tum.cc 进行了简单解读，其中的代码还是十分简单的，并且我们也知道了，其中主要类对象来自于如下代码:

```cpp
    //初始化ORB-SLAM2系统
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
```

其中 ORB_SLAM2::System 这个类，是非常重要的。并且其中的函数：

```cpp
    cv::Mat TrackRGBD(const cv::Mat &im,                //彩色图像
                      const cv::Mat &depthmap,          //深度图像
                      const double &timestamp);  
```

是我们后续当中核心的核心，所以我们必须去了解 ORB_SLAM2::System 的实现以及构建过程。

### 二、代码注释

首先我们找到工程下面的 src/System.cc 文件，并且找到其中的构建函数 System::System()，其代码注释如下:

```cpp
//系统的构造函数，将会启动其他的线程
System::System(const string &strVocFile,					//词典文件路径
			   const string &strSettingsFile,				//配置文件路径
			   const eSensor sensor,						//传感器类型
               const bool bUseViewer):						//是否使用可视化界面
					 mSensor(sensor), 							//初始化传感器类型
					 mpViewer(static_cast<Viewer*>(NULL)),		//空。。。对象指针？  TODO 
					 mbReset(false),							//无复位标志
					 mbActivateLocalizationMode(false),			//没有这个模式转换标志
        			 mbDeactivateLocalizationMode(false)		//没有这个模式转换标志
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    // 输出当前传感器类型
    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), 	//将配置文件名转换成为字符串
    						   cv::FileStorage::READ);		//只读
    //如果打开失败，就输出调试信息
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       //然后退出
       exit(-1);
    }

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    //建立一个新的ORB字典
    mpVocabulary = new ORBVocabulary();
    //获取字典加载状态
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    //如果加载失败，就输出调试信息
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        //然后退出
        exit(-1);
    }
    //否则则说明加载成功
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    //这里的帧绘制器和地图绘制器将会被可视化的Viewer所使用
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //在本主进程中初始化追踪线程
    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this,						//现在还不是很明白为什么这里还需要一个this指针  TODO  
    						 mpVocabulary,				//字典
    						 mpFrameDrawer, 			//帧绘制器
    						 mpMapDrawer,				//地图绘制器
                             mpMap, 					//地图
                             mpKeyFrameDatabase, 		//关键帧地图
                             strSettingsFile, 			//设置文件路径
                             mSensor);					//传感器类型iomanip

    //初始化局部建图线程并运行
    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, 				//指定使iomanip
    								 mSensor==MONOCULAR);	// TODO 为什么这个要设置成为MONOCULAR？？？
    //运行这个局部建图线程
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,	//这个线程会调用的函数
    							 mpLocalMapper);				//这个调用函数的参数

    //Initialize the Loop Closing thread and launchiomanip
    mpLoopCloser = new LoopClosing(mpMap, 						//地图
    							   mpKeyFrameDatabase, 			//关键帧数据库
    							   mpVocabulary, 				//ORB字典
    							   mSensor!=MONOCULAR);			//当前的传感器是否是单目
    //创建回环检测线程
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run,	//线程的主函数
    							mpLoopCloser);					//该函数的参数

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
    	//如果指定了，程序的运行过程中需要运行可视化部分
    	//新建viewer
        mpViewer = new Viewer(this, 			//又是这个
        					  mpFrameDrawer,	//帧绘制器
        					  mpMapDrawer,		//地图绘制器
        					  mpTracker,		//追踪器
        					  strSettingsFile);	//配置文件的访问路径
        //新建viewer线程
        mptViewer = new thread(&Viewer::Run, mpViewer);
        //给运动追踪器设置其查看器
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    //设置进程间的指针
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

```

### 三、源码讲解

在前面的博客中，我已经给出了注释过的源码链接，其中可以看到十分详细的注释的内容。通过上面的代码，我们可以看到有几个核心部分:

#### (1)mpVocabulary（读入字典）

```cpp
    //建立一个新的ORB字典
    mpVocabulary = new ORBVocabulary();
    //获取字典加载状态
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
   	//Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
```

这里的 strVocFile 就是命令行传入的参数 Vocabulary/ORBvoc.txt， 这个呢是属于比较大的一块，主要和特征匹配相关，所以暂时就不进行详细讲解了，后续会专属章节进行讲解。 Vocabulary/ORBvoc.txt 是离线训练而来的文件，主要流程如下:

```
	首先图像提取ORB 特征点，将描述子通过 k-means 进行聚类，根据设定的树的分支数和深度，
	从叶子节点开始聚类一直到根节点，最后得到一个非常大的 vocabulary tree
	
	1、遍历所有的训练图像，对每幅图像提取ORB特征点。
	2、设定vocabulary tree的分支数K和深度L。将特征点的每个描述子用 K-means聚类，变成K个集合，
	作为vocabulary tree 的第1层级，然后对每个集合重复该聚类操作，就得到了vocabulary tree的第2层级，
	继续迭代最后得到满足条件的vocabulary tree，它的规模通常比较大，比如ORB-	SLAM2使用的离线字典就有108万+ 个节点。
	3、离根节点最远的一层节点称为叶子或者单词 Word。根据每个Word 在训练集中的相关	
	程度给定一个权重weight，训练集里出现的次数越多，说明辨别力越差，给与的权重越低。
```

上面只是描述了一下 离线训练 vocabulary tree（也称为字典），具体讲解我们后续再进行。其主要的作用是: 特征匹配 \color{red}{特征匹配}特征匹配，关键帧辨别 \color{red}{关键帧辨别}关键帧辨别。

#### (2)mpTracker（主线程-初始化）

```cpp
    //Create Drawers. These are used by the Viewer
    //这里的帧绘制器和地图绘制器将会被可视化的Viewer所使用
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this,						//现在还不是很明白为什么这里还需要一个this指针  TODO  
    						 mpVocabulary,				//字典
    						 mpFrameDrawer, 			//帧绘制器
    						 mpMapDrawer,				//地图绘制器
                             mpMap, 					//地图
                             mpKeyFrameDatabase, 		//关键帧地图
                             strSettingsFile, 			//设置文件路径
                             mSensor);					//传感器类型iomanip
```

其上主要初始化了追踪线程（或者说主线程），同时我们可以看到其初始化参数中包含了 mpFrameDrawer ， mpMapDrawer 。这两个对象主要是负责对帧与地图的绘画。

#### (3)mptLocalMapping，mptLoopClosing（局部建图-闭环线程）

```cpp
    //初始化局部建图线程并运行
    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, 				//指定使iomanip
    								 mSensor==MONOCULAR);	// 判断mSensor是不是MONOCULAR
    //运行这个局部建图线程
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,	//这个线程会调用的函数
    							 mpLocalMapper);				//这个调用函数的参数

    //Initialize the Loop Closing thread and launchiomanip
    mpLoopCloser = new LoopClosing(mpMap, 						//地图
    							   mpKeyFrameDatabase, 			//关键帧数据库
    							   mpVocabulary, 				//ORB字典
    							   mSensor!=MONOCULAR);			//当前的传感器是否是单目
    //创建回环检测线程
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run,	//线程的主函数
    							mpLoopCloser);					//该函数的参数
```

其上我们可以知道，其主要运行了两个线程ORB_SLAM2::LocalMapping::Run 以及 ORB_SLAM2::LoopClosing::Run，分别是局部建图以及闭环线程。这里已经启动了，这两个线程是后续我们需要重点讲解的大块，所以这里也不做详细的介绍了

#### (4)mptViewer（可视化线程）

```cpp
   	//如果指定了，程序的运行过程中需要运行可视化部分
   	//新建viewer
       mpViewer = new Viewer(this, 			//又是这个
       					  mpFrameDrawer,	//帧绘制器
       					  mpMapDrawer,		//地图绘制器
       					  mpTracker,		//追踪器
       					  strSettingsFile);	//配置文件的访问路径
       //新建viewer线程
       mptViewer = new thread(&Viewer::Run, mpViewer);
       //给运动追踪器设置其查看器
       mpTracker->SetViewer(mpViewer);
```

这里主要创建了一个可视化的线程，可视化的线程与追踪主线程是息息相关的。可视化的操作，主要根据追踪线程传递的信息来执行界面的绘画。

### 四、总结

针对于 ORB_SLAM2::System 的构造函数主要流程如下:

	1、加载ORB词汇表，构建Vocabulary，以及关键帧数据集库
	2、初始化追踪主线程，但是未运行
	3、初始化局部建图，回环闭合线程，且运行
	4、创建可视化线程，并且与追踪主线程关联起来。
