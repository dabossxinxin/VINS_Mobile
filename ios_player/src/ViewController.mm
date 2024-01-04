
// AR模式：左下角显示器显示里程计轨迹以及路标点，主显示器显示增强现实景象
// VINS模式：左下角显示器显示增强现实景象，主显示器显示里程计轨迹及路标点
#include "utility.hpp"
#include <opencv2/imgproc.hpp>
#include "utility/producer.h"

#import "ViewController.h"
#import "CameraUtils.h"

@interface ViewController ()
@property (weak, nonatomic) IBOutlet UILabel *featureMarkLabel;
@property (weak, nonatomic) IBOutlet UILabel *informationLabel;
@property (weak, nonatomic) IBOutlet UITextView *informationLogView;
@end

@implementation ViewController

bool start_record = false;
bool start_playback = false;
bool start_playback_vins = false;
bool PUBLISH_WITH_AMQP = true;

unsigned long imageDataIndex = 0;
unsigned long imageDataReadIndex = 0;
unsigned long imuDataIndex = 0;
unsigned long imuDataReadIndex = 0;
unsigned long vinsDataIndex = 0;
unsigned long vinsDataReadIndex = 0;

NSMutableData *imuDataBuf = [[NSMutableData alloc] init];
NSMutableData *vinsDataBuf = [[NSMutableData alloc] init];

NSData *imuReader;
NSData *vinsReader;

IMG_DATA imgData;
IMU_MSG imuData;
KEYFRAME_DATA vinsData;

// false: VINS trajectory is the main view, AR image is in left bottom
// true: AR image is the main view, VINS is in left bottom
bool ui_main = false;
bool box_in_AR = false;
bool box_in_trajectory = false;
bool start_show = false;            // If initialized finished, start show is true

UIActivityIndicatorView *loadingStatusIndicator;        // 内容加载状态图标

@synthesize resultImageView;        // 用于显示VINS轨迹以及AR信息
@synthesize videoCamera;            // 用于采集IPHONE后摄像头图像

int loopOldIndex = -1;      // 闭环检测线程检测到的闭环帧ID

float x_view_last = -999;   // 上一时刻里程计坐标x
float y_view_last = -999;   // 上一时刻里程计坐标y
float z_view_last = -999;   // 上一时刻里程计坐标z
float total_odom = 0;       // 里程计轨迹长度

VINS vins;
FeatureTracker  featuretracker;
RabbitMQ_producer imuProducer;
RabbitMQ_producer imgProducer;

//RMQConnection   *imuConnector;
//RMQConnection   *imgConnector;
//RMQExchange     *imuExchange;
//RMQExchange     *imgExchange;
NSString        *amqpStatus;

std::queue<ImgConstPtr> img_msg_buf;         // 存储图像特征信息
std::queue<ImuConstPtr> imu_msg_buf;         // 存储IMU信息
std::queue<IMU_MSG_LOCAL> local_imu_msg_buf; // 存储IMU信息

int waiting_lists = 0;      // 等待被处理的测量值个数
int frame_cnt = 0;

std::mutex m_buf;           // 用于锁定imu和image数据的互斥锁
std::condition_variable con;

NSTimeInterval current_time = -1;
NSTimeInterval lateast_imu_time = -1;

// MotionManager for read imu data
CMMotionManager *motionManager;

// Segment the trajectory using color when re-initialize
int segmentation_index = 0;

// Lock the solved VINS data feedback to featuretracker
std::mutex m_depth_feedback;

// Lock the IMU data feedback to featuretracker
std::mutex m_imu_feedback;

// Solved VINS feature feedback to featuretracker
list<IMG_MSG_LOCAL> solved_features;

// Solved VINS status feedback to featuretracker
VINS_RESULT solved_vins;

/******************************* Loop Closure ******************************/

// Raw image data buffer for extracting FAST feature
std::queue<std::pair<cv::Mat, double>> loopImageBuf;
std::mutex loopImageBufMutex;

// Detect loop
LoopClosure *loop_closure;

// Keyframe database
KeyFrameDatabase keyframe_database;

// Control the loop detection frequency
int keyframe_freq = 0;

// Index the keyframe
int global_frame_cnt = 0;

// Record the checked loop frame
int loop_check_cnt = 0;

bool isVocFileLoaded = false;       // 词袋文件是否成功加载完毕

// Indicate the loop frame index
int old_index = -1;

// drift
Eigen::Vector3d loop_correct_t = Eigen::Vector3d(0, 0, 0);
Eigen::Matrix3d loop_correct_r = Eigen::Matrix3d::Identity();

/******************************* Loop Closure ******************************/

// MARK: Unity Camera Mode Switching
// Ground truth from UI switch property "self.switchUIAREnabled"

// Implied, updated by updateCameraMode()
bool imuPredictEnabled = false;

// Implied, updated by updateCameraMode()
bool imageCacheEnabled = true;

std::mutex loginInfoMutex;              // 登陆信息互斥锁
std::mutex imuPublishMsgMutex;          // 待传输的IMU信息互斥锁
std::mutex imgPublishMsgMutex;          // 待传输的图像信息互斥锁

std::condition_variable cvImuPulish;    // IMU信息传输条件变量
std::condition_variable cvImgPulish;    // IMG信息传输条件变量

std::queue<std::pair<Vector6d, double>>  imuPublishMsg;  // 待RabbitMQ传输的IMU信息
std::queue<std::pair<cv::Mat, double>>   imgPublishMsg;  // 待RabbitMQ传输的图像信息

std::mutex vinsResultPoolMutex;
std::queue<IMG_DATA_CACHE> imageCachePool;
std::queue<VINS_DATA_CACHE> vinsResultPool;

// MARK: ViewController Methods
// 入口函数
- (void)viewDidLoad
{
    [super viewDidLoad];
    
    [self.informationLogView setEditable:NO];
    [self.informationLogView setScrollEnabled:YES];
    [self.informationLogView setScrollsToTop:YES];
    
    // 加载登陆界面中传过来的登陆信息
    [[NSNotificationCenter defaultCenter] addObserver:self
                                             selector:@selector(notificationHandler:)
                                                 name:@"loginInfo"
                                               object:nil];
    
    // IPHONE相机设置
    self.videoCamera = [[CvVideoCamera alloc] initWithParentView:resultImageView];
    self.videoCamera.delegate = self;
    self.videoCamera.defaultAVCaptureDevicePosition = AVCaptureDevicePositionBack;
    self.videoCamera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationPortrait;
    self.videoCamera.defaultAVCaptureSessionPreset = AVCaptureSessionPreset640x480;
    self.videoCamera.defaultFPS = 20;
    //[CameraUtils setExposureOffset: -1.0f];
    isCapturing = NO;
    
    didCameraThread=[[NSThread alloc]initWithTarget:self
                                           selector:@selector(didCameraThreadWork)
                                             object:nil];
    [didCameraThread setName:@"didCameraThread"];
    [didCameraThread start];
    
    // IPHONE端设置软件界面UI
    UIPanGestureRecognizer *resultPanGestureRecognizer = [[UIPanGestureRecognizer alloc]
                                                          initWithTarget:self
                                                          action:@selector(handlePan:)];
    resultPanGestureRecognizer.minimumNumberOfTouches = 1;
    resultPanGestureRecognizer.maximumNumberOfTouches = 2;
    [self.resultImageView addGestureRecognizer:resultPanGestureRecognizer];
    
    UIPinchGestureRecognizer *resultPinchGestureRecognizer = [[UIPinchGestureRecognizer alloc]
                                                              initWithTarget:self
                                                              action:@selector(handlePinch:)];
    [self.resultImageView addGestureRecognizer:resultPinchGestureRecognizer];
    
    UITapGestureRecognizer *resultTapGestureRecognizer = [[UITapGestureRecognizer alloc]
                                                          initWithTarget:self
                                                          action:@selector(handleTap:)];
    [self.resultImageView addGestureRecognizer:resultTapGestureRecognizer];
    
    UILongPressGestureRecognizer *resultLongPressGestureRecognizer = [[UILongPressGestureRecognizer alloc]
                                                                      initWithTarget:self
                                                                      action:@selector(handleLongPress:)];
    [self.resultImageView addGestureRecognizer:resultLongPressGestureRecognizer];
    
    self.loopButton.layer.zPosition = 1;
    self.reinitButton.layer.zPosition = 1;
    
    loadingStatusIndicator = [[UIActivityIndicatorView alloc] initWithActivityIndicatorStyle:UIActivityIndicatorViewStyleWhiteLarge];
    loadingStatusIndicator.center = CGPointMake(self.resultImageView.frame.size.width * 0.5+
                                                self.resultImageView.frame.origin.x,
                                                self.resultImageView.frame.size.height * 0.5+
                                                self.resultImageView.frame.origin.y);
    loadingStatusIndicator.color = [UIColor darkGrayColor];
    [loadingStatusIndicator startAnimating];
    [self.view addSubview:loadingStatusIndicator];
    
    if (!feature_tracker)
        feature_tracker = new FeatureTracker();
    vins.setIMUModel();
    
    // 初始化所有线程：前端处理，后端闭环以及数据保存线程
    self->condition=[[NSCondition alloc] init];
    mainThread=[[NSThread alloc]initWithTarget:self selector:@selector(mainThreadWork) object:nil];
    [mainThread setName:@"mainThread"];
    
    if(LOOP_CLOSURE)
    {
        loopDetectThread = [[NSThread alloc]initWithTarget:self selector:@selector(loopDetectThreadWork) object:nil];
        [loopDetectThread setName:@"loopDetectThread"];
        [loopDetectThread start];
        
        loopRefineThread=[[NSThread alloc]initWithTarget:self selector:@selector(loopRefineThreadWork) object:nil];
        [loopRefineThread setName:@"loopRefineThread"];
        [loopRefineThread start];
    }
    
    // 判断当前设备类型以及系统版本以设置不同的参数
    bool deviceCheck = setGlobalParam(deviceName());
    if(!deviceCheck)
    {
        UIAlertController *alertDevice = [UIAlertController alertControllerWithTitle:@"Unsupported Device"
                                                                             message:@"Supported devices are: iPhone12"
                                                                      preferredStyle:UIAlertControllerStyleAlert];
        
        UIAlertAction *okAction = [UIAlertAction actionWithTitle:@"OK"
                                                           style:UIAlertActionStyleDefault
                                                         handler:^(UIAlertAction * _Nonnull action) {}];
        
        [alertDevice addAction:okAction];
        
        dispatch_async(dispatch_get_main_queue(), ^ {
            [self presentViewController:alertDevice animated:YES completion:nil];
        });
    }
    vins.setExtrinsic();
    vins.setIMUModel();
    featuretracker.vins_pnp.setExtrinsic();
    featuretracker.vins_pnp.setIMUModel();
    
    // 检查IOS版本
    bool versionCheck = iosVersion();
    if(!versionCheck)
    {
        UIAlertController *alertVersion = [UIAlertController alertControllerWithTitle:@"Warn"
                                                                              message:@"Please upgrade your iOS version!"
                                                                       preferredStyle:UIAlertControllerStyleAlert];
        UIAlertAction *cancelAction = [UIAlertAction actionWithTitle:@"Cancel"
                                                               style:UIAlertActionStyleCancel
                                                             handler:^(UIAlertAction * action){exit(0);}];
        UIAlertAction *okAction = [UIAlertAction actionWithTitle:@"OK"
                                                           style:UIAlertActionStyleDefault
                                                         handler:^(UIAlertAction * action){exit(0);}];
        [alertVersion addAction:cancelAction];
        [alertVersion addAction:okAction];
        dispatch_async(dispatch_get_main_queue(), ^ {
            [self presentViewController:alertVersion animated:YES completion:nil];
        });
    }
    
    connectAMQPThread = [[NSThread alloc] initWithTarget:self
                                                selector:@selector(connectAMQPThreadWork)
                                                  object:nil];
    [connectAMQPThread setName:@"connectAMQPThread"];
    [connectAMQPThread start];
    
    sendImuThread = [[NSThread alloc] initWithTarget:self
                                            selector:@selector(sendImuThreadWork)
                                              object:nil];
    [sendImuThread setName:@"sendImuThread"];
    [sendImuThread start];
    
    sendImgThread = [[NSThread alloc] initWithTarget:self
                                            selector:@selector(sendImgThreadWork)
                                              object:nil];
    [sendImgThread setName:@"sendImgThread"];
    [sendImgThread start];
    
    // 硬件设备以及版本检测通过，开始VINS
    if(versionCheck && deviceCheck)
    {
        motionManager = [[CMMotionManager alloc] init];
        [self imuStartUpdate];
        
        isCapturing = YES;
        [mainThread start];
        
        frameSize = cv::Size(videoCamera.imageWidth,
                             videoCamera.imageHeight);
    }
}

/// 从软件中获取RabbitMQ服务连接信息并连接imuConnector和imgConnector
-(void) connectAMQPThreadWork
{
    while (![[NSThread currentThread] isCancelled])
    {
        // 获取连接信息
        amqpStatus = [NSString stringWithFormat:@"amqp: waiting"];
        while(self.username == nil || self.password == nil || self.hostname == nil)
        {
            [self performSelectorOnMainThread:@selector(showLogInfo:)
                                   withObject:@"Wait AMQP Login Info From Main Thread\n"
                                waitUntilDone:NO];
        }
        [self performSelectorOnMainThread:@selector(showLogInfo:)
                               withObject: @"AMQP Parameters Load Success!\n"
                            waitUntilDone:NO];
        
        NSString *uri = [NSString stringWithFormat:
                         @"amqp://%@:%@@%@:5672", self.username,self.password,self.hostname];
        [self performSelectorOnMainThread:@selector(showLogInfo:)
                               withObject:uri
                            waitUntilDone:NO];
        amqpStatus = [NSString stringWithFormat:@"amqp: start"];
        
        std::string username = std::string([self.username UTF8String]);
        std::string password = std::string([self.password UTF8String]);
        std::string hostname = std::string([self.hostname UTF8String]);
        
        imuProducer.setProducerName("imu_producer");
        imuProducer.setQueue("imu_queue", "imu");
        imuProducer.setExchange("imu_exchange", "direct");
        imuProducer.setHostName(hostname, "5672");
        imuProducer.credential(username, password);
        
        imgProducer.setProducerName("img_producer");
        imgProducer.setQueue("img_queue", "img");
        imgProducer.setExchange("img_exchange", "direct");
        imgProducer.setHostName(hostname, "5672");
        imgProducer.credential(username, password);
        
        imuProducer.run();
        imgProducer.run();
        
        amqpStatus = [NSString stringWithFormat:@"amqp: finish"];
        
        [[NSThread currentThread] cancel];
    }
}

-(void) notificationHandler:(NSNotification*) notification
{
    NSDictionary *dicts = [notification userInfo];
    self.username = [dicts objectForKey:@"username"];
    self.password = [dicts objectForKey:@"password"];
    self.hostname = [dicts objectForKey:@"hostname"];
}

/// OpenCV内部接收摄像头采集到的图像并进行处理的接口
/// - Parameter image: OpenCV调用摄像头采集的图像数据
- (void)processImage:(cv::Mat&)image
{
    if(isCapturing == YES)
    {
        std::shared_ptr<IMG_MSG> img_msg(new IMG_MSG());
        
        float lowPart = image.at<float>(0,0);
        float highPart = image.at<float>(0,1);
        float Group[2] = {lowPart,highPart};
        double* timeNowDecode = (double*)Group;
        img_msg->header = *timeNowDecode;
        
        if(lateast_imu_time <= 0)
        {
            cv::cvtColor(image, image, cv::COLOR_BGRA2RGB);
            cv::flip(image, image, -1);
            return;
        }
        
        BOOL isNeedRotation = image.size() != frameSize;
                
        cv::Mat inputImageFrame = image;
        prevTime = mach_absolute_time();
        
        // 1、图像直方图均衡化，降低光照影像
        cv::Mat grayImage;
        cv::Mat equalizeImage;
        cv::cvtColor(inputImageFrame, grayImage, cv::COLOR_RGBA2GRAY);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(3);
        clahe->apply(grayImage, equalizeImage);
        
        // 2、光流跟踪图像中的特征点
        TS(time_feature);
        m_depth_feedback.lock();
        featuretracker.solved_features = solved_features;
        featuretracker.solved_vins = solved_vins;
        m_depth_feedback.unlock();
        
        m_imu_feedback.lock();
        featuretracker.imu_msgs = getImuMeasurements(img_msg->header);
        m_imu_feedback.unlock();
        
        std::vector<Point2f> good_pts;
        std::vector<double> track_len;
        bool vins_normal = (vins.solver_flag == VINS::NON_LINEAR);
        
        Eigen::Vector3d PnpPos;
        Eigen::Matrix3d PnpRot;
        
        featuretracker.readImage(equalizeImage,frame_cnt, good_pts,track_len,
                                 img_msg->header, PnpPos, PnpRot, vins_normal);
        TE(time_feature);
            
        // 3、将直方图均衡化后的图像以及时间戳发送到服务器
        if (PUBLISH_WITH_AMQP && imgProducer.isOpen())
        {
            std::pair<cv::Mat, double> imageData;
            imageData.first = equalizeImage;
            imageData.second = img_msg->header;
            
            std::unique_lock<std::mutex> publishLock(imgPublishMsgMutex);
            imgPublishMsg.push(imageData);
            publishLock.unlock();
            cvImgPulish.notify_one();
        }
        
        // 4、将提取到的图像特征数据放进buffer中
        if(featuretracker.img_cnt == 0)
        {
            img_msg->point_clouds = featuretracker.image_msg;
            
            m_buf.lock();
            img_msg_buf.push(img_msg);
            m_buf.unlock();
            con.notify_one();
            
            if(imageCacheEnabled)
            {
                IMG_DATA_CACHE imgDataCache;
                imgDataCache.header = img_msg->header;
                imgDataCache.equalizeImage = equalizeImage;
                imageCachePool.push(imgDataCache);
            }
            
            if(LOOP_CLOSURE)
            {
                std::unique_lock<std::mutex> lock(loopImageBufMutex);
                loopImageBuf.push(std::make_pair(grayImage.clone(), img_msg->header));
                if(loopImageBuf.size() > WINDOW_SIZE)
                    loopImageBuf.pop();
                lock.unlock();
            }
        }
        
        featuretracker.img_cnt = (featuretracker.img_cnt + 1) % FREQ;
        for (int it = 0; it < good_pts.size(); ++it)
        {
            cv::circle(image, good_pts[it], 0, cv::Scalar(255 * (1 - track_len[it]), 0, 255 * track_len[it]), 7); //BGR
        }
        
        cv::Mat         latestImg;
        Eigen::Vector3f latestPos;
        Eigen::Matrix3f latestRot;
        
        TS(visualize);
        if(imageCacheEnabled)
        {
            //use aligned vins and image
            if(!vinsResultPool.empty() && !imageCachePool.empty())
            {
                std::unique_lock<std::mutex> vinsResultPoolLock(vinsResultPoolMutex);
                while(vinsResultPool.size() > 1)
                {
                    vinsResultPool.pop();
                }
                vinsResultPoolLock.unlock();
                
                while(!imageCachePool.empty() && imageCachePool.front().header < vinsResultPool.front().header)
                {
                    imageCachePool.pop();
                }
                if(!vinsResultPool.empty() && !imageCachePool.empty())
                {
                    latestImg = imageCachePool.front().equalizeImage;
                    latestPos = vinsResultPool.front().P;
                    latestRot = vinsResultPool.front().R;
                }
            }
            else if(!imageCachePool.empty())
            {
                if(imageCachePool.size() > 10)
                    imageCachePool.pop();
            }
        }
        
        // 绘制AR界面 || 还未正常采集图像 || 系统还未初始化成功
        if(ui_main || start_show == false || vins.solver_flag != VINS::NON_LINEAR)
        {
            if(vins.solver_flag == VINS::NON_LINEAR && start_show)
            {
                vins.drawresult.startInit = true;
                vins.drawresult.drawAR(vins.imageAI, vins.correct_point_cloud, latestPos, latestRot);
                
                cv::Mat latestImgBGR;
                if (!latestImg.empty())
                {
                    cv::cvtColor(latestImg, latestImgBGR, cv::COLOR_GRAY2BGR);
                    cv::Mat mask;
                    cv::cvtColor(vins.imageAI, mask, cv::COLOR_RGB2GRAY);
                
                    vins.imageAI.copyTo(latestImgBGR, mask);
                    cv::cvtColor(latestImgBGR, image, cv::COLOR_BGRA2BGR);
                }
            }
            
            // 系统还未正常采集图像 || 系统还未初始化成功
            if(vins.solver_flag != VINS::NON_LINEAR || !start_show)
                cv::cvtColor(image, image, cv::COLOR_RGBA2BGR);
        }
        // Draw VINS
        else
        {
            if(vins.solver_flag == VINS::NON_LINEAR)
            {
                vins.drawresult.pose.clear();
                vins.drawresult.pose = keyframe_database.refine_path;
                vins.drawresult.segment_indexs = keyframe_database.segment_indexs;
                vins.drawresult.Reprojection(vins.image_show, vins.correct_point_cloud,
                                             vins.correct_Rs, vins.correct_Ps, box_in_trajectory);
            }
            
            int originWidth = image.cols;
            int originHeight = image.rows;
            cv::Mat downOriginImage;
            
            cv::resize(image, downOriginImage, cv::Size(originWidth*0.25, originHeight*0.25));
            cv::cvtColor(downOriginImage, downOriginImage, cv::COLOR_BGRA2RGB);
            
            int imageRoiX = image.cols - downOriginImage.cols - 1;
            int imageRoiY = image.rows - downOriginImage.rows - 1;
            int imageRoiWidth = downOriginImage.cols;
            int imageRoiHeight = downOriginImage.rows;
            
            NSLog(@"imageWidth: %d", image.cols);
            NSLog(@"imageHeight: %d", image.rows);
            NSLog(@"imageRoiX: %d", imageRoiX);
            NSLog(@"imageRoiY: %d", imageRoiY);
            NSLog(@"imageRoiWidth: %d",imageRoiWidth);
            NSLog(@"imageRoiHeight: %d", imageRoiHeight);
            NSLog(@"showImageWidth: %d",vins.image_show.cols);
            NSLog(@"showImageheight: %d", vins.image_show.rows);
            
            if (vins.image_show.size != image.size)
            {
                vins.image_show = vins.image_show.t();
            }

            cv::Mat imageRoi;
            imageRoi = vins.image_show(cv::Rect(imageRoiX,imageRoiY, imageRoiWidth,imageRoiHeight));
            downOriginImage.copyTo(imageRoi);
            
            cv::cvtColor(vins.image_show, image, cv::COLOR_BGRA2BGR);
        }
        TE(visualize);
        
        [self performSelectorOnMainThread:@selector(showResultImageInfo:)
                               withObject:MatToUIImage(image)
                            waitUntilDone:YES];
    }
}

/// 获取时间戳对齐的图像数据以及惯导数据
std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> getMeasurements()
{
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;
    while (true)
    {
        if (imu_msg_buf.empty() || img_msg_buf.empty())
            return measurements;
        
        if (!(imu_msg_buf.back()->header > img_msg_buf.front()->header))
        {
            NSLog(@"wait for imu, only should happen at the beginning");
            return measurements;
        }
        
        if (!(imu_msg_buf.front()->header < img_msg_buf.front()->header))
        {
            NSLog(@"throw img, only should happen at the beginning");
            img_msg_buf.pop();
            continue;
        }
        ImgConstPtr img_msg = img_msg_buf.front();
        img_msg_buf.pop();
        
        std::vector<ImuConstPtr> IMUs;
        while (imu_msg_buf.front()->header <= img_msg->header)
        {
            IMUs.emplace_back(imu_msg_buf.front());
            imu_msg_buf.pop();
        }
        //NSLog(@"IMU_buf = %d",IMUs.size());
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

std::vector<IMU_MSG_LOCAL> getImuMeasurements(double header)
{
    vector<IMU_MSG_LOCAL> imu_measurements;
    static double last_header = -1;
    if(last_header < 0 || local_imu_msg_buf.empty())
    {
        last_header = header;
        return imu_measurements;
    }
    
    while(!local_imu_msg_buf.empty() && local_imu_msg_buf.front().header <= last_header)
        local_imu_msg_buf.pop();
    while(!local_imu_msg_buf.empty() && local_imu_msg_buf.front().header <= header)
    {
        imu_measurements.emplace_back(local_imu_msg_buf.front());
        local_imu_msg_buf.pop();
    }
    last_header = header;
    return imu_measurements;
}

void send_imu(const ImuConstPtr &imu_msg)
{
    NSTimeInterval t = imu_msg->header;
    if (current_time < 0)
        current_time = t;
    double dt = (t - current_time);
    current_time = t;
    
    double ba[]{0.0, 0.0, 0.0};
    double bg[]{0.0, 0.0, 0.0};
    
    double dx = imu_msg->acc.x() - ba[0];
    double dy = imu_msg->acc.y() - ba[1];
    double dz = imu_msg->acc.z() - ba[2];
    
    double rx = imu_msg->gyr.x() - bg[0];
    double ry = imu_msg->gyr.y() - bg[1];
    double rz = imu_msg->gyr.z() - bg[2];
    
    vins.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
}

/*
 VINS thread: this thread tightly fuses the visual measurements and imu data and solves pose, velocity, IMU bias, 3D feature for all frame in WINNDOW
 If the newest frame is keyframe, then push it into keyframe database
 */
-(void)mainThreadWork
{
    [self->condition lock];
    while (![[NSThread currentThread] isCancelled])
    {
        [self process];
        [NSThread sleepForTimeInterval:0.01];
    }
    [self->condition unlock];
    
}

-(void)didCameraThreadWork
{
    while(![[NSThread currentThread] isCancelled])
    {
        [self.videoCamera start];
    }
}

-(void)sendImuThreadWork
{
    while(![[NSThread currentThread] isCancelled])
    {
        [self sendImuMsgWithAMQP];
    }
}

-(void)sendImgThreadWork
{
    while(![[NSThread currentThread] isCancelled])
    {
        [self sendImgMsgWithAMQP];
    }
}


/// RabbitMQ发送移动端采集的惯导数据
-(void)sendImuMsgWithAMQP
{
    std::unique_lock<std::mutex> lock(imuPublishMsgMutex);
    cvImuPulish.wait(lock, [&]{
        while(!imuPublishMsg.empty())
        {
            imuProducer.send(imuPublishMsg.front());
            imuPublishMsg.pop();
        }
        return true;
    });
    lock.unlock();
}

/// RaabitMQ发送移动端采集的图像数据
-(void)sendImgMsgWithAMQP
{
    std::unique_lock<std::mutex> lock(imgPublishMsgMutex);
    cvImgPulish.wait(lock,[&]{
        while(!imgPublishMsg.empty())
        {
            imgProducer.send(imgPublishMsg.front(), ".jpg", 80);
            imgPublishMsg.pop();
        }
        return true;
    });
    lock.unlock();
}

int kf_global_index;
bool start_global_optimization = false;
-(void)process
{
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;
    std::unique_lock<std::mutex> lk(m_buf);
    con.wait(lk, [&] {
        return (measurements = getMeasurements()).size() != 0;
    });
    lk.unlock();
    
    waiting_lists = measurements.size();
    
    for(auto &measurement : measurements)
    {
        for(auto &imu_msg : measurement.first)
        {
            send_imu(imu_msg);
        }
        
        auto& img_msg = measurement.second;
        std::map<int, Vector3d> image = img_msg->point_clouds;
        double header = img_msg->header;
        
        TS(process_image);
        vins.processImage(image,header,waiting_lists);
        TE(process_image);
        
        double time_now = [[NSProcessInfo processInfo] systemUptime];
        double time_vins = vins.Headers[WINDOW_SIZE];
        NSLog(@"vins delay %lf", time_now - time_vins);
        
        //update feature position for front-end
        if(vins.solver_flag == vins.NON_LINEAR)
        {
            m_depth_feedback.lock();
            solved_vins.header = vins.Headers[WINDOW_SIZE - 1];
            solved_vins.Ba = vins.Bas[WINDOW_SIZE - 1];
            solved_vins.Bg = vins.Bgs[WINDOW_SIZE - 1];
            solved_vins.P = vins.correct_Ps[WINDOW_SIZE-1].cast<double>();
            solved_vins.R = vins.correct_Rs[WINDOW_SIZE-1].cast<double>();
            solved_vins.V = vins.Vs[WINDOW_SIZE - 1];
            Vector3d R_ypr = Utility::R2ypr(solved_vins.R);
            solved_features.clear();
            for (auto &it_per_id : vins.f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;
                if (it_per_id.solve_flag != 1)
                    continue;
                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                IMG_MSG_LOCAL tmp_feature;
                tmp_feature.id = it_per_id.feature_id;
                tmp_feature.position = vins.r_drift * vins.Rs[imu_i] * (vins.ric * pts_i + vins.tic) + vins.r_drift * vins.Ps[imu_i] + vins.t_drift;
                tmp_feature.track_num = (int)it_per_id.feature_per_frame.size();
                solved_features.emplace_back(tmp_feature);
            }
            m_depth_feedback.unlock();
        }
        
        if(imageCacheEnabled)
        {
            //add state into vins buff for alignwith image
            if(vins.solver_flag == VINS::NON_LINEAR && start_show)
            {
                VINS_DATA_CACHE vinsDataCache;
                vinsDataCache.header = vins.Headers[WINDOW_SIZE-1];
                vinsDataCache.P = vins.correct_Ps[WINDOW_SIZE-1];
                vinsDataCache.R = vins.correct_Rs[WINDOW_SIZE-1];
                
                std::unique_lock<std::mutex> vinsResultPoolLock(vinsResultPoolMutex);
                vinsResultPool.push(vinsDataCache);
                vinsResultPoolLock.unlock();
            }
            else if(vins.failure_occur == true)
            {
                vins.drawresult.change_color = true;
                vins.drawresult.indexs.push_back(vins.drawresult.pose.size());
                segmentation_index++;
                keyframe_database.max_seg_index++;
                keyframe_database.cur_seg_index = keyframe_database.max_seg_index;
                
                while(!vinsResultPool.empty())
                    vinsResultPool.pop();
            }
        }

        // start build keyframe database for loop closure
        if(LOOP_CLOSURE)
        {
            static bool first_frame = true;
            if(vins.solver_flag != vins.NON_LINEAR)
                first_frame = true;
            
            if(vins.marginalization_flag == vins.MARGIN_OLD && vins.solver_flag == vins.NON_LINEAR && !loopImageBuf.empty())
            {
                first_frame = false;
                if(!first_frame && keyframe_freq % LOOP_FREQ == 0)
                {
                    keyframe_freq = 0;
                    Vector3d T_w_i = vins.Ps[WINDOW_SIZE - 2];
                    Matrix3d R_w_i = vins.Rs[WINDOW_SIZE - 2];
                    
                    std::unique_lock<std::mutex> loopLock(loopImageBufMutex);
                    while(!loopImageBuf.empty() && loopImageBuf.front().second < vins.Headers[WINDOW_SIZE - 2])
                    {
                        loopImageBuf.pop();
                    }
                    
                    if(vins.Headers[WINDOW_SIZE - 2] == loopImageBuf.front().second)
                    {
                        const char *pattern_file = [[[NSBundle bundleForClass:[self class]] pathForResource:@"brief_pattern" ofType:@"yml"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
                        KeyFrame* keyframe = new KeyFrame(vins.Headers[WINDOW_SIZE - 2], global_frame_cnt, T_w_i, R_w_i, loopImageBuf.front().first, pattern_file, keyframe_database.cur_seg_index);
                        keyframe->setExtrinsic(vins.tic, vins.ric);
                        
                        // we still need save the measurement to the keyframe(not database) for add connection with
                        // looped pose and save the pointcloud to the keyframe for reprojection search correspondance
                        keyframe->buildKeyFrameFeatures(vins);
                        keyframe_database.add(keyframe);
                        
                        global_frame_cnt++;
                    }
                    loopLock.unlock();
                }
                else
                {
                    first_frame = false;
                }
                // update loop info
                for (int i = 0; i < WINDOW_SIZE; i++)
                {
                    if(vins.Headers[i] == vins.front_pose.header)
                    {
                        KeyFrame* cur_kf = keyframe_database.getKeyframe(vins.front_pose.cur_index);
                        if (abs(vins.front_pose.relative_yaw) > 30.0 || vins.front_pose.relative_t.norm() > 10.0)
                        {
                            printf("Wrong loop\n");
                            cur_kf->removeLoop();
                            break;
                        }
                        cur_kf->updateLoopConnection(vins.front_pose.relative_t,
                                                     vins.front_pose.relative_q,
                                                     vins.front_pose.relative_yaw);
                        break;
                    }
                }
                /*
                 ** update the keyframe pose when this frame slides out the window and optimize loop graph
                 */
                int search_cnt = 0;
                for(int i = 0; i < keyframe_database.size(); i++)
                {
                    search_cnt++;
                    KeyFrame* kf = keyframe_database.getLastKeyframe(i);
                    if(kf->header == vins.Headers[0])
                    {
                        kf->updateOriginPose(vins.Ps[0], vins.Rs[0]);
                        //update edge
                        // if loop happens in this frame, update pose graph;
                        if (kf->has_loop)
                        {
                            kf_global_index = kf->global_index;
                            start_global_optimization = true;
                        }
                        break;
                    }
                    else
                    {
                        if(search_cnt > 2 * WINDOW_SIZE)
                            break;
                    }
                }
                keyframe_freq++;
            }
        }
        waiting_lists--;
        
        [self performSelectorOnMainThread:@selector(showFeatureStaticInfo) withObject:nil waitUntilDone:YES];
    }
}


/// VINS加载词袋文件并开始闭环检测
-(void)loopDetectThreadWork
{
    // 加载词袋文件
    if(LOOP_CLOSURE && loop_closure == NULL)
    {
        NSLog(@"Loop Thread Start Load VocFile\n");
        TS(load_voc);
        const char *voc_file = [[[NSBundle bundleForClass:[self class]] pathForResource:@"brief_k10L6" ofType:@"bin"]
                                cStringUsingEncoding:[NSString defaultCStringEncoding]];
        loop_closure = new LoopClosure(voc_file, COL, ROW);
        TE(load_voc);
        NSLog(@"Loop Thread Load VocFile Success\n");
        
        isVocFileLoaded = true;
    }
    
    // 开始进行闭环检测
    while(![[NSThread currentThread] isCancelled] )
    {
        if(!LOOP_CLOSURE)
        {
            [NSThread sleepForTimeInterval:0.5];
            continue;
        }
        
        bool loop_succ = false;
        if (loop_check_cnt < global_frame_cnt)
        {
            KeyFrame* cur_kf = keyframe_database.getLastKeyframe();
            //assert(loop_check_cnt == cur_kf->global_index);
            loop_check_cnt++;
            cur_kf->check_loop = 1;
            
            cv::Mat current_image;
            current_image = cur_kf->image;
            
            std::vector<cv::Point2f> measurements_old;
            std::vector<cv::Point2f> measurements_old_norm;
            std::vector<cv::Point2f> measurements_cur;
            std::vector<int> features_id;
            std::vector<cv::Point2f> measurements_cur_origin = cur_kf->measurements;
            
            vector<cv::Point2f> cur_pts;
            vector<cv::Point2f> old_pts;
            cur_kf->extractBrief(current_image);
            printf("loop extract %d feature\n", cur_kf->keypoints.size());
            loop_succ = loop_closure->startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index);
            if(loop_succ)
            {
                KeyFrame* old_kf = keyframe_database.getKeyframe(old_index);
                if (old_kf == NULL)
                {
                    printf("NO such %dth frame in keyframe_database\n", old_index);
                    assert(false);
                }
                printf("loop succ with %drd image\n", old_index);
                assert(old_index!=-1);
                
                Vector3d T_w_i_old;
                Matrix3d R_w_i_old;
                
                old_kf->getPose(T_w_i_old, R_w_i_old);
                cur_kf->findConnectionWithOldFrame(old_kf, cur_pts, old_pts,
                                                   measurements_old, measurements_old_norm);
                measurements_cur = cur_kf->measurements;
                features_id = cur_kf->features_id;
                
                if(measurements_old_norm.size()>MIN_LOOP_NUM)
                {
                    Quaterniond Q_loop_old(R_w_i_old);
                    RetriveData retrive_data;
                    retrive_data.cur_index = cur_kf->global_index;
                    retrive_data.header = cur_kf->header;
                    retrive_data.P_old = T_w_i_old;
                    retrive_data.Q_old = Q_loop_old;
                    retrive_data.use = true;
                    retrive_data.measurements = measurements_old_norm;
                    retrive_data.features_ids = features_id;
                    vins.retrive_pose_data = (retrive_data);
                    
                    //cout << "old pose " << T_w_i_old.transpose() << endl;
                    //cout << "refinded pose " << T_w_i_refine.transpose() << endl;
                    // add loop edge in current frame
                    cur_kf->detectLoop(old_index);
                    keyframe_database.addLoop(old_index);
                    old_kf->is_looped = 1;
                    loopOldIndex = old_index;
                }
            }
            cur_kf->image.release();
        }
        
        if(loop_succ)
            [NSThread sleepForTimeInterval:2.0];
        [NSThread sleepForTimeInterval:0.05];
    }
}


/// VINS检测到闭环后进行闭环校正
-(void)loopRefineThreadWork{
    while (![[NSThread currentThread] isCancelled])
    {
        if(start_global_optimization)
        {
            start_global_optimization = false;
            TS(loop_thread);
            keyframe_database.optimize4DoFLoopPoseGraph(kf_global_index,
                                                        loop_correct_t,
                                                        loop_correct_r);
            vins.t_drift = loop_correct_t;
            vins.r_drift = loop_correct_r;
            TE(loop_thread);
            [NSThread sleepForTimeInterval:1.17];
        }
        [NSThread sleepForTimeInterval:0.03];
    }
}

// Z^
// |   /Y
// |  /
// | /
// |/--------->X
// IMU数据插值获取陀螺仪和加速度计的数据
int imu_prepare = 0;
bool imuDataFinished = false;
bool vinsDataFinished = false;
std::vector<IMU_MSG> gyro_buf;
std::shared_ptr<IMU_MSG> cur_acc(new IMU_MSG());


/// 移动端采集惯导数据并进行加速度计与陀螺仪时间戳对齐
- (void)imuStartUpdate
{
    if (!motionManager.accelerometerAvailable)
        NSLog(@"we don't have device accelemeter.");
    if (!motionManager.gyroAvailable)
        NSLog(@"we don't have device gyrometer.");
    
    motionManager.gyroUpdateInterval = 0.01;
    motionManager.accelerometerUpdateInterval = 0.01;
    
    [motionManager startDeviceMotionUpdates];
    
    [motionManager startAccelerometerUpdatesToQueue:[NSOperationQueue currentQueue]
                                        withHandler:^(CMAccelerometerData *latestAcc, NSError *error) {
        if(imu_prepare < 10) imu_prepare++;
        std::shared_ptr<IMU_MSG> acc_msg(new IMU_MSG());
        acc_msg->header = latestAcc.timestamp;
        acc_msg->acc << -latestAcc.acceleration.x * GRAVITY,
                        -latestAcc.acceleration.y * GRAVITY,
                        -latestAcc.acceleration.z * GRAVITY;
        cur_acc = acc_msg;
    }];
    
    [motionManager startGyroUpdatesToQueue:[NSOperationQueue currentQueue]
                               withHandler:^(CMGyroData *latestGyro, NSError *error) {
        if(latestGyro.timestamp <= 0) return;
        if(imu_prepare < 10) return;
        
        IMU_MSG gyro_msg;
        gyro_msg.header = latestGyro.timestamp;
        gyro_msg.gyr << latestGyro.rotationRate.x,
                        latestGyro.rotationRate.y,
                        latestGyro.rotationRate.z;
        
        if(gyro_buf.empty())
        {
            gyro_buf.emplace_back(gyro_msg);
            gyro_buf.emplace_back(gyro_msg);
            return;
        }
        else
        {
            gyro_buf[0] = gyro_buf[1];
            gyro_buf[1] = gyro_msg;
        }
        
        std::shared_ptr<IMU_MSG> imu_msg(new IMU_MSG());
        if(cur_acc->header >= gyro_buf[0].header && cur_acc->header < gyro_buf[1].header)
        {
            imu_msg->header = cur_acc->header;
            imu_msg->acc    = cur_acc->acc;
            imu_msg->gyr    = gyro_buf[0].gyr + (cur_acc->header - gyro_buf[0].header)*
            (gyro_buf[1].gyr - gyro_buf[0].gyr)/(gyro_buf[1].header - gyro_buf[0].header);
            
            if (PUBLISH_WITH_AMQP && imuProducer.isOpen())
            {
                Vector6d imuData = Vector6d::Zero();
                imuData.block(0, 0, 3, 1) = imu_msg->acc;
                imuData.block(3, 0, 3, 1) = imu_msg->gyr;
                std::pair<Vector6d, double> imuMsg(std::make_pair(imuData, imu_msg->header));
                
                {
                    std::unique_lock<std::mutex> publishLock(imuPublishMsgMutex);
                    imuPublishMsg.push(imuMsg);
                }
                cvImuPulish.notify_one();
            }
        }
        else
        {
            [self performSelectorOnMainThread:@selector(showLogInfo:)
                                   withObject:@"Imu Interplation Error, Check it.\n"
                                waitUntilDone:NO];
            return;
        }
        
        lateast_imu_time = imu_msg->header;
        
        //img_msg callback
        {
            IMU_MSG_LOCAL imu_msg_local;
            imu_msg_local.header = imu_msg->header;
            imu_msg_local.acc = imu_msg->acc;
            imu_msg_local.gyr = imu_msg->gyr;
            
            m_imu_feedback.lock();
            local_imu_msg_buf.push(imu_msg_local);
            m_imu_feedback.unlock();
        }
        
        // 将IMU数据加入到指定buf中
        m_buf.lock();
        imu_msg_buf.push(imu_msg);
        m_buf.unlock();
        con.notify_one();
    }];
}

- (void)showFeatureStaticInfo
{
    NSString *stringView;
    static bool finish_init = false;
    
    // VINS系统进入初始化状态跟踪系统是否初始化成功
    if(vins.solver_flag != vins.NON_LINEAR)
    {
        finish_init = false;
        switch (vins.init_status) {
            case vins.FAIL_IMU:
                stringView = [NSString stringWithFormat:@"Initialize Status: FAIL_IMU, Please Move Your Iphone\n"];
                break;
            case vins.FAIL_PARALLAX:
                stringView = [NSString stringWithFormat:@"Initialize Status: FAIL_PARA, Please Move Your Iphone\n"];
                break;
            case vins.FAIL_RELATIVE:
                stringView = [NSString stringWithFormat:@"Initialize Status: FAIL_RELA, Please Move Your Iphone\n"];
                break;
            case vins.FAIL_SFM:
                stringView = [NSString stringWithFormat:@"Initialize Status: FAIL_SFM, Please Move Your Iphone\n"];
                break;
            case vins.FAIL_PNP:
                stringView = [NSString stringWithFormat:@"Initialize Status: FAIL_PNP, Please Move Your Iphone\n"];
                break;
            case vins.FAIL_ALIGN:
                stringView = [NSString stringWithFormat:@"Initialize Status: FAIL_ALIGN, Please Move Your Iphone\n"];
                break;
            case vins.FAIL_CHECK:
                stringView = [NSString stringWithFormat:@"Initialize Status: FAIL_COST, Please Move Your Iphone\n"];
                break;
            case vins.SUCC:
                stringView = [NSString stringWithFormat:@"Initialize Status: SUCCESS, Now Start VINS\n"];
                break;
            default:
                break;
        }
        [self showLogInfo:stringView];
        stringView = [NSString stringWithFormat:@"fail: %d", vins.fail_times];
        //[_failTimesLabel setText:stringView];
        stringView = [NSString stringWithFormat:@"parallax: %d", vins.parallax_num_view];
        //[_parallaxLabel setText:stringView];
        stringView = [NSString stringWithFormat:@"initializing: %d%%", vins.initProgress];
        [_featureMarkLabel setText:stringView];
        
        [_featureMarkLabel setHidden:NO];
        [loadingStatusIndicator setHidden:NO];
    }
    // VINS系统已经初始化完毕进入非线性优化状态
    else
    {
        if(finish_init == false)
        {
            [_featureMarkLabel setHidden:YES];
            [loadingStatusIndicator setHidden:YES];
            
            start_show = true;
            finish_init = true;
        }
        
        float x_view = (float)vins.correct_Ps[frame_cnt][0];
        float y_view = (float)vins.correct_Ps[frame_cnt][1];
        float z_view = (float)vins.correct_Ps[frame_cnt][2];
        if(x_view_last == -999)
        {
            x_view_last = x_view;
            y_view_last = y_view;
            z_view_last = z_view;
        }
        total_odom += sqrt(pow((x_view - x_view_last), 2) +
                           pow((y_view - y_view_last), 2) +
                           pow((z_view - z_view_last), 2));
        x_view_last = x_view;
        y_view_last = y_view;
        z_view_last = z_view;
        
        NSString *length = [NSString stringWithFormat:@" LENGTH: %.2f\n",total_odom];
        NSString *position = [NSString stringWithFormat:@" POS: [%.4f,%.4f,%.4f]\n",x_view,y_view,z_view];
        stringView = [NSString stringWithFormat:@"----------------\n"];
        stringView = [stringView stringByAppendingString:position];
        stringView = [stringView stringByAppendingString:length];
        stringView = [stringView stringByAppendingString:@"----------------\n"];
        [self showLogInfo:stringView];
    }
    stringView = [NSString stringWithFormat:@"buffer: %d",waiting_lists];
    //[_bufferNumsLabel setText:stringView];
    
    stringView = [NSString stringWithFormat:@"feature: %d",vins.feature_num];
    //[_featureNumsLabel setText:stringView];
    
    if(loopOldIndex != -1)
    {
        stringView = [NSString stringWithFormat:@"loop id: %d",loopOldIndex];
        //[_loopIndexLabel setText:stringView];
    }
}

- (void)showLogInfo:(NSString*)info
{
    dispatch_async(dispatch_get_main_queue(), ^(void) {
        NSString* logInfo = [self.informationLogView.text stringByAppendingString:info];
        [self.informationLogView setText:logInfo];
        [self.informationLogView scrollRangeToVisible:NSMakeRange(logInfo.length, 0)];
    });
}

- (void)showRawImageInfo:(UIImage*)image
{
    [self.rawImageView setImage:image];
}

- (void)showResultImageInfo:(UIImage*)image
{
    [self.resultImageView setImage:image];
}

-(IBAction)switchUI:(UISegmentedControl *)sender
{
    switch (_switchUI.selectedSegmentIndex)
    {
        case 0:
            self.switchUIAREnabled = YES;
            
            NSLog(@"Show AR.");
            ui_main = true;
            box_in_AR= true;
            imageCacheEnabled = true;
            break;
        case 1:
            self.switchUIAREnabled = NO;
            
            NSLog(@"Show VINS.");
            ui_main = false;
            if (box_in_AR)
                box_in_trajectory = true;
            imageCacheEnabled = true;
            break;
        default:
            break;
    }
}

- (IBAction)fovSliderValueChanged:(id)sender {
    //self.fovLabel.text = [[NSNumber numberWithFloat:self.fovSlider.value] stringValue];
}

- (void) handlePan:(UIPanGestureRecognizer*) recognizer
{
    if(ui_main and 0)
        return;
    
    if (!ui_main)
    {
        CGPoint translation = [recognizer translationInView:self.view];
        CGFloat velocityX = [recognizer velocityInView:self.view].x;
        CGFloat velocityY = [recognizer velocityInView:self.view].y;
        //recognizer.view.center = CGPointMake(recognizer.view.center.x + translation.x,
        static CGFloat vx_last = 0;
        static CGFloat vy_last = 0;
        
        CGFloat vx_smooth = 0.5*velocityX + 0.5*vx_last;
        CGFloat vy_smooth = 0.5*velocityY + 0.5*vy_last;
        vx_last = vx_smooth;
        vy_last = vy_smooth;
        if(recognizer.numberOfTouches == 2)
        {
            vins.drawresult.Y0 += vx_smooth/100.0;
            vins.drawresult.X0 += vy_smooth/100.0;
        }
        else
        {
            vins.drawresult.theta += vy_smooth/100.0;
            vins.drawresult.theta = fmod(vins.drawresult.theta, 360.0);
            vins.drawresult.phy += vx_smooth/100.0;
            vins.drawresult.phy = fmod(vins.drawresult.phy, 360.0);
        }
        
        vins.drawresult.change_view_manualy = true;
    }
    else
    {
        CGPoint translation = [recognizer translationInView:self.view];
        CGFloat velocityX = [recognizer velocityInView:self.view].x;
        CGFloat velocityY = [recognizer velocityInView:self.view].y;
        //CGFloat translationX =
        //CGFloat translationY = [recognizer translationInView:self.view].y;
        //NSLog(@"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!pipikk test x: %f y: %f", translationX, translationY);
        //NSLog(@"!!!!!!!!!!!!!!!!!!!!!!%f  %f", resultImageView.frame.size.height, resultImageView.frame.size.width);
        CGPoint point = [recognizer locationInView:self.view];
        //NSLog(@"X Location: %f", point.x);
        //NSLog(@"Y Location: %f",point.y);
        
        //recognizer.view.center = CGPointMake(recognizer.view.center.x + translation.x,
        static CGFloat vx_lastAR = 0;
        static CGFloat vy_lastAR = 0;
        
        CGFloat vx_smooth = 0.5*velocityX + 0.5*vx_lastAR;
        CGFloat vy_smooth = 0.5*velocityY + 0.5*vy_lastAR;
        vx_lastAR = vx_smooth;
        vy_lastAR = vy_smooth;
        if(recognizer.numberOfTouches == 2)
        {
            vins.drawresult.Y0AR += vx_smooth/100.0;
            vins.drawresult.X0AR += vy_smooth/100.0;
            
            vins.drawresult.locationXT2 = point.x * 640.0 / resultImageView.frame.size.width;
            vins.drawresult.locationYT2 = point.y * 480.0 / resultImageView.frame.size.height;
            
            vins.drawresult.finger_s = 0;
            vins.drawresult.finger_p = 0;
            if ((vins.drawresult.finger_d ++) > 7)
                vins.drawresult.finger_state = 2;
        }
        else
        {
            vins.drawresult.thetaAR += vy_smooth/100.0;
            //vins.drawresult.thetaAR = fmod(vins.drawresult.thetaAR, 360.0);
            vins.drawresult.phyAR += vx_smooth/100.0;
            //vins.drawresult.phyAR = fmod(vins.drawresult.phyAR, 360.0);
            
            vins.drawresult.locationX = point.x * 640.0 / resultImageView.frame.size.width;
            vins.drawresult.locationY = point.y * 480.0 / resultImageView.frame.size.height;
            
            vins.drawresult.finger_d = 0;
            vins.drawresult.finger_p = 0;
            if ((vins.drawresult.finger_s ++) > 7)
                vins.drawresult.finger_state = 1;
        }
    }
}

- (void) handlePinch:(UIPinchGestureRecognizer*) recognizer
{
    if(ui_main and 0)
        return;
    
    if (!ui_main)
    {
        vins.drawresult.change_view_manualy = true;
        if(vins.drawresult.radius > 5 || recognizer.velocity < 0)
            vins.drawresult.radius -= recognizer.velocity * 0.5;
        else
        {
            vins.drawresult.Fx += recognizer.velocity * 15;
            if(vins.drawresult.Fx < 50)
                vins.drawresult.Fx = 50;
            vins.drawresult.Fy += recognizer.velocity * 15;
            if(vins.drawresult.Fy < 50)
                vins.drawresult.Fy = 50;
        }
    }
    else{
        
        vins.drawresult.finger_s = 0;
        vins.drawresult.finger_d = 0;
        if ((vins.drawresult.finger_p ++) > 7)
            vins.drawresult.finger_state = 3;
        
        CGPoint point = [recognizer locationInView:self.view];
        vins.drawresult.locationXP = point.x * 640.0 / resultImageView.frame.size.width;
        vins.drawresult.locationYP = point.y * 480.0 / resultImageView.frame.size.height;
        
        //NSLog(@"pipikk_radius: %f velocity: ", vins.drawresult.radiusAR, recognizer.velocity);
        
        //if(vins.drawresult.radiusAR > 5 || recognizer.velocity < 0)
        //{
        vins.drawresult.radiusAR -= recognizer.velocity * 0.5;
        //}
    }
    
}

- (void) handleTap:(UITapGestureRecognizer*) recognizer
{
    if (!ui_main)
    {
        
    }
    else{
        
        /*vins.drawresult.finger_s = 0;
         vins.drawresult.finger_d = 0;
         if ((vins.drawresult.finger_p ++) > 7)
         vins.drawresult.finger_state = 3;*/
        
        CGPoint point = [recognizer locationInView:self.view];
        vins.drawresult.locationTapX = point.x * 640.0 / resultImageView.frame.size.width;
        vins.drawresult.locationTapY = point.y * 480.0 / resultImageView.frame.size.height;
        
        vins.drawresult.tapFlag = true;
        
    }
    
}

- (void) handleLongPress:(UILongPressGestureRecognizer*) recognizer
{
    if (!ui_main)
    {
        
    }
    {
        CGPoint point = [recognizer locationInView:self.view];
        vins.drawresult.locationLongPressX = point.x * 640.0 / resultImageView.frame.size.width;
        vins.drawresult.locationLongPressY = point.y * 480.0 / resultImageView.frame.size.height;
        vins.drawresult.longPressFlag = true;
    }
}

- (IBAction)loopButtonPressed:(id)sender
{
    if(LOOP_CLOSURE)
    {
        LOOP_CLOSURE = false;
        [_loopButton setTitle:@"ENLOOP" forState:UIControlStateNormal];
    }
    else
    {
        LOOP_CLOSURE = true;
        [_loopButton setTitle:@"UNLOOP" forState:UIControlStateNormal];
    }
}

- (IBAction)reinitButtonPressed:(id)sender
{
    vins.drawresult.planeInit = false;
    vins.failure_hand = true;
    vins.drawresult.change_color = true;
    vins.drawresult.indexs.emplace_back(vins.drawresult.pose.size());
    
    segmentation_index++;
    keyframe_database.max_seg_index++;
    keyframe_database.cur_seg_index = keyframe_database.max_seg_index;
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
}

- (void)tapSaveImageToIphone:(UIImage*)image
{
    UIImageWriteToSavedPhotosAlbum(image, self, @selector(image:didFinishSavingWithError:contextInfo:), nil);
}

- (void)image:(UIImage *)image didFinishSavingWithError:(NSError *)error contextInfo:(void *)contextInfo{
    
    if (error == nil) {
        NSLog(@"Save Access");
    }else{
        NSLog(@"Save Failed");
    }
}

- (void)checkDirectoryPath:(unsigned long)index withObject:(NSString*)directoryPath
{
    //delete already exist directory first time
    NSError *error;
    if (index == 0 && [[NSFileManager defaultManager] fileExistsAtPath:directoryPath])	//Does directory exist?
    {
        if (![[NSFileManager defaultManager] removeItemAtPath:directoryPath error:&error])	//Delete it
        {
            NSLog(@"Delete directory error: %@", error);
        }
    }
    
    //creat file directory if it does not exist
    if (![[NSFileManager defaultManager] fileExistsAtPath:directoryPath])
    {
        NSLog(@"directory does not exist");
        if (![[NSFileManager defaultManager] createDirectoryAtPath:directoryPath
                                       withIntermediateDirectories:NO
                                                        attributes:nil
                                                             error:&error])
        {
            NSLog(@"Create directory error: %@", error);
        }
    }
}

- (void)recordImu
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [paths objectAtIndex:0];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:@"IMU"]; //Add the file name
    
    [imuDataBuf writeToFile:filePath atomically:YES];
    //[msgData writeToFile:filePath atomically:YES];
}

- (void)recordVins
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [paths objectAtIndex:0];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:@"VINS"]; //Add the file name
    
    [vinsDataBuf writeToFile:filePath atomically:YES];
    //[msgData writeToFile:filePath atomically:YES];
}

- (void)recordImageTime:(IMG_DATA&)image_data
{
    double time = image_data.header;
    NSData *msgData = [NSData dataWithBytes:&time length:sizeof(time)];
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE_TIME"];; //Get the docs directory
    
    [self checkDirectoryPath:imageDataIndex withObject:documentsPath];
    
    NSString *filename = [NSString stringWithFormat:@"%lu", imageDataIndex];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name
    
    [msgData writeToFile:filePath atomically:YES];
}

- (void)recordImage:(IMG_DATA&)image_data
{
    NSData *msgData = UIImagePNGRepresentation(image_data.image);
    //NSData *msgData = [NSData dataWithBytes:&image_data length:sizeof(image_data)];
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE"];; //Get the docs directory
    
    [self checkDirectoryPath:imageDataIndex withObject:documentsPath];
    
    NSString *filename = [NSString stringWithFormat:@"%lu", imageDataIndex];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name
    
    [msgData writeToFile:filePath atomically:YES];
}

-(bool)readImageTime:(unsigned long)index
{
    bool file_exist;
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE_TIME"]; //Get the docs directory
    NSString *filename = [NSString stringWithFormat:@"%lu", index];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name
    
    //check file exists
    if ([[NSFileManager defaultManager] fileExistsAtPath:filePath])
    {
        NSData *file1 = [[NSData alloc] initWithContentsOfFile:filePath];
        if (file1)
        {
            double time;
            [file1 getBytes:&time length:sizeof(time)];
            imgData.header = time;
        }
        file_exist = true;
    }
    else
    {
        file_exist = false;
        //NSLog(@"File does not exist");
    }
    return file_exist;
}

-(bool)readImage:(unsigned long)index
{
    bool file_exist;
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE"]; //Get the docs directory
    NSString *filename = [NSString stringWithFormat:@"%lu", index];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name
    
    //check file exists
    if ([[NSFileManager defaultManager] fileExistsAtPath:filePath])
    {
        NSData *pngData = [NSData dataWithContentsOfFile:filePath];
        imgData.image = [UIImage imageWithData:pngData];
        file_exist = true;
    }
    else
    {
        file_exist = false;
        //NSLog(@"File does not exist");
    }
    return file_exist;
}

- (void)viewDidAppear:(BOOL)animated
{
    [super viewDidAppear:animated];
}

- (void)viewDidDisappear:(BOOL)animated
{
    [super viewDidDisappear:animated];
    if (isCapturing)
    {
        [videoCamera stop];
    }
    [mainThread cancel];
    [drawThread cancel];
#ifdef LOOP_CLOSURE
    [loopDetectThread cancel];
    [loopRefineThread cancel];
#endif
    
    imuProducer.flush_and_exit();
    imgProducer.flush_and_exit();
    //[imuConnector close];
    //[imgConnector close];
    
    [sendImuThread cancel];
    [sendImgThread cancel];
    [didCameraThread cancel];
    
    [[NSNotificationCenter defaultCenter] removeObserver:self
                                                    name:@"loginInfo"
                                                  object:nil];
}

-(void)viewDidUnload{
    [motionManager stopGyroUpdates];
    [motionManager stopAccelerometerUpdates];
    [motionManager stopDeviceMotionUpdates];
    [motionManager stopMagnetometerUpdates];
    [super viewDidUnload];
}

- (void)dealloc
{
    videoCamera.delegate = nil;
}

DeviceType deviceName()
{
    struct utsname systemInfo;
    uname(&systemInfo);
    
    NSString *device = [NSString stringWithCString:systemInfo.machine
                                          encoding:NSUTF8StringEncoding];
    DeviceType device_type;
    if(([device compare:@"iPhone9,1"] == NSOrderedSame) ||
       ([device compare:@"iPhone9,3"] == NSOrderedSame))
    {
        NSLog(@"Device iPhone7");
        device_type = iPhone7;
    }
    else if(([device compare:@"iPhone9,2"] == NSOrderedSame) ||
            ([device compare:@"iPhone9,4"] == NSOrderedSame))
    {
        NSLog(@"Device iPhone7 plus");
        device_type = iPhone7P;
    }
    else if(([device compare:@"iPhone8,2"] == NSOrderedSame))
    {
        NSLog(@"Device iPhone6s plus");
        device_type = iPhone6sP;
    }
    else if(([device compare:@"iPhone8,1"] == NSOrderedSame))
    {
        NSLog(@"Device iPhone6s");
        device_type = iPhone6s;
    }
    else if(([device compare:@"iPad6,3"] == NSOrderedSame)||
            ([device compare:@"iPad6,4"] == NSOrderedSame))
    {
        NSLog(@"Device iPad pro 9.7");
        device_type = iPadPro97;
    }
    else if(([device compare:@"iPad6,7"] == NSOrderedSame)||
            ([device compare:@"iPad6,8"] == NSOrderedSame))
    {
        NSLog(@"Device iPad pro 12.9");
        device_type = iPadPro129;
    }
    else if([device compare:@"iPhone13,2"] == NSOrderedSame)
    {
        NSLog(@"Device iPhone12");
        device_type = iPhone12;
    }
    else
    {
        NSLog(@"Device undefine");
        device_type = unDefine;
    }
    return device_type;
}


/// 检查移动端IOS版本信息是否满足本程序的要求
bool iosVersion()
{
    NSComparisonResult order = [[UIDevice currentDevice].systemVersion
                                compare: @"10.2.1"
                                options: NSNumericSearch];
    
    if (order == NSOrderedSame || order == NSOrderedDescending)
    {
        NSLog(@"system version >= 10.2.1");
        return true;
    }
    else
    {
        NSLog(@"system version < 10.2.1");
        return false;
    }
}
@end
