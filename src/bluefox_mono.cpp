#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <dynamic_reconfigure/server.h>
#include <bluefox_mono_ros/DynamicConfConfig.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <Helper.h>
#include <unistd.h>

using namespace std;
using namespace cv;
using namespace mvIMPACT::acquire;

#ifdef MALLOC_TRACE
#   include <mcheck.h>
#endif  // MALLOC_TRACE

#define PRESS_A_KEY_AND_RETURN          \
    cout << "Press a key..." << endl;   \
    getchar(); \
    return 0;


//-----------------------------------------------------------------------------
void callback(bluefox_mono_ros::DynamicConfConfig &config, uint32_t level, Device* pDev)
//-----------------------------------------------------------------------------
{

  mvIMPACT::acquire::GenICam::AcquisitionControl ac( pDev );
  mvIMPACT::acquire::ImageDestination id( pDev );
  mvIMPACT::acquire::ImageProcessing ip( pDev );
  mvIMPACT::acquire::GenICam::AnalogControl anctrl( pDev );

  conditionalSetEnumPropertyByString(ac.exposureAuto, config.autoExposure);
  if(ac.exposureAuto.readS()=="Off")
      conditionalSetProperty(ac.exposureTime, config.exposureTime);
  else
  {

    if(config.autoExpLowerLimit < config.autoExpUpperLimit)
    {
      ac.mvExposureAutoLowerLimit.write( (float)config.autoExpLowerLimit  );
      ac.mvExposureAutoUpperLimit.write( (float)config.autoExpUpperLimit  );
    }
    else

      ROS_WARN_STREAM("autoExpLowerLimit MUST BE MINOR than autoExpUpperLimit!!, request will be skipped!");

  }
  conditionalSetEnumPropertyByString(id.scalerMode, config.scalerMode);
  conditionalSetProperty(id.imageWidth, config.scaledWidth);
  conditionalSetProperty(id.imageHeight, config.scaledHeight);
  conditionalSetEnumPropertyByString(ip.gainOffsetKneeEnable, config.gainKneeEnable?"On":"Off");
  conditionalSetProperty(ip.gainOffsetKneeMasterOffset_pc, config.gainPerc);
  conditionalSetEnumPropertyByString(ac.mvAcquisitionFrameRateEnable,  config.manualFps);
  conditionalSetProperty(ac.acquisitionFrameRate, config.fpsValue);
  conditionalSetEnumPropertyByString(anctrl.balanceWhiteAuto, config.Wb_setting );
  conditionalSetProperty(anctrl.blackLevel, config.blk_level );
  conditionalSetEnumPropertyByString(ip.LUTEnable, config.lutGamma);
  conditionalSetEnumPropertyByString(ip.colorTwistEnable, config.colorCorrectionsEnable);
  if(ip.colorTwistEnable.readS() == "On"){
    conditionalSetEnumPropertyByString(ip.colorTwistInputCorrectionMatrixEnable, "On");
    conditionalSetEnumPropertyByString(ip.colorTwistInputCorrectionMatrixMode, "DeviceSpecific");
    conditionalSetEnumPropertyByString(ip.colorTwistOutputCorrectionMatrixEnable, "On");
    ip.setSaturation(( config.Saturation / 100 ) );
    const unsigned int LUTCnt = ip.getLUTParameterCount();

    for( unsigned int i = 0; i < LUTCnt; i++ )
    {
       mvIMPACT::acquire::LUTParameters& lpm = ip.getLUTParameter( i );
        lpm.gamma.write(config.gammaGain);
    }

  }
  else
  {
    conditionalSetEnumPropertyByString(ip.colorTwistInputCorrectionMatrixEnable, "Off");
    conditionalSetEnumPropertyByString(ip.colorTwistInputCorrectionMatrixMode, "DeviceSpecific");
    conditionalSetEnumPropertyByString(ip.colorTwistOutputCorrectionMatrixEnable, "Off");
  }

}

//-----------------------------------------------------------------------------
void initializeRosParameters(Device* pDev, const ros::NodeHandle& n)
//-----------------------------------------------------------------------------
{

  mvIMPACT::acquire::GenICam::AcquisitionControl ac( pDev );
  mvIMPACT::acquire::ImageDestination id( pDev );
  mvIMPACT::acquire::ImageProcessing ip( pDev );
  mvIMPACT::acquire::GenICam::AnalogControl anctrl( pDev );


  double exp_time = ac.exposureTime.read();
  string scaler_Mode = id.scalerMode.readS();
  int defaultScalerWidth = 640;
  int defaultScalerHeigth = 480;
  bool gainOffsetKneeEnable = ip.gainOffsetKneeEnable.read();
  double gainOffsetKneeMasterOffset_pc = ip.gainOffsetKneeMasterOffset_pc.read();
  string manual_fps = ac.mvAcquisitionFrameRateEnable.readS();
  int fps_value = ac.acquisitionFrameRate.read();
  string white_balance_init = anctrl.balanceWhiteAuto.readS();
  double black_level_init = anctrl.blackLevel.read();
  string lut_enable_init = ip.LUTEnable.readS();
  string colorTwistMat_init = ip.colorTwistEnable.readS();
  int saturation_level_init = 100;
  string auto_exposure_auto_init = ac.exposureAuto.readS();
  int auto_exposure_lowerLimit = 10;
  int auto_exposure_upperLimit = 20000;
  double gamma_gain_init = 1.0;



  n.getParam("exposureTime", exp_time);
  n.getParam("scalerMode", scaler_Mode);
  n.getParam("scaledWidth", defaultScalerWidth);
  n.getParam("scaledHeight", defaultScalerHeigth);
  n.getParam("gainKneeEnable",gainOffsetKneeEnable);
  n.getParam("gainPerc",gainOffsetKneeMasterOffset_pc);
  n.getParam("manualFps",manual_fps);
  n.getParam("fpsValue",fps_value);
  n.getParam("Wb_setting", white_balance_init);
  n.getParam("blackLevel", black_level_init);
  n.getParam("lutGamma", lut_enable_init);
  n.getParam("colorCorrectionsEnable", colorTwistMat_init);
  n.getParam("Saturation", saturation_level_init);
  n.getParam("autoExposure", auto_exposure_auto_init);
  n.getParam("autoExpLowerLimit", auto_exposure_lowerLimit);
  n.getParam("autoExpUpperLimit", auto_exposure_upperLimit);
  n.getParam("gammaGain", gamma_gain_init);


}

//-----------------------------------------------------------------------------
void sendImageToRos( const Request* pRequest, Mat img, cv_bridge::CvImage* bridge_image, int count,
  camera_info_manager::CameraInfoManager* camera_info_mgr, image_transport::CameraPublisher* image_publisher)
//-----------------------------------------------------------------------------
{
    const int wid = pRequest->imageWidth.read();
    const int heig = pRequest->imageHeight.read();
    const int pitch = pRequest->imageLinePitch.read();
    const int nChannels = pRequest->imageChannelCount.read();

    //ros
    /* -- fill in information in cv_bridge image -- */
    bridge_image->header.seq   = count;
    bridge_image->header.stamp = ros::Time::now();

    /* -- fill in information in CameraInfo -- */
    sensor_msgs::CameraInfo cam_info = camera_info_mgr->getCameraInfo();
    cam_info.header.stamp = bridge_image->header.stamp;
    cam_info.width  = wid;
    cam_info.height = heig;

    // create image
    if (nChannels == 1)
    {
        cv::Mat camImgWrapped(heig, wid, CV_8UC1, pRequest->imageData.read(), pitch);
        img = camImgWrapped.clone();
        //imshow("bluefox3_wow", img );
        //waitKey(1);
        bridge_image->encoding = sensor_msgs::image_encodings::MONO8;
    }
    else if (nChannels == 3)
    {
        cv::Mat camImgWrapped(heig, wid, CV_8UC3, pRequest->imageData.read(), pitch);
        img = camImgWrapped.clone();
        //imshow("bluefox3_wow3", img );
        //waitKey(1);
        bridge_image->encoding = sensor_msgs::image_encodings::BGR8;
    }
    else if (nChannels == 4)
    {
        cv::Mat camImgWrapped(heig, wid, CV_8UC4, pRequest->imageData.read(), pitch);
        img = camImgWrapped.clone();
        //imshow("bluefox3_wow", img );
        //waitKey(1);
        bridge_image->encoding = sensor_msgs::image_encodings::BGRA8;
    }
    else
    {
    //errore and stop
    }

    bridge_image->image = img;
    /* -- Convert image to a message and publish it -- */
		sensor_msgs::Image image_message;
		bridge_image->toImageMsg(image_message);
		image_publisher->publish(image_message,cam_info);

    ros::spinOnce();

																// unlock the buffer to let the driver know that you no longer need this
}


//-----------------------------------------------------------------------------
// return value: 0: stop because of user input
//               1: stop for acquisition problem (e.g. camera disconnected)
bool liveLoop(Device* pDev)
//-----------------------------------------------------------------------------
{

    //create img Mat for opencv
    cv::Mat image_placeholder;

    /**************** Ros setup *************************************/

    ros::NodeHandle n("~");
    image_transport::ImageTransport it(n);
    cv_bridge::CvImage bridge_image;
    camera_info_manager::CameraInfoManager camera_info_mgr(n,"Bluefox_mono");
		image_transport::CameraPublisher image_publisher(it.advertiseCamera("image_raw",100));

    //setting up dynamic reconfigure ROS
		dynamic_reconfigure::Server<bluefox_mono_ros::DynamicConfConfig> server;
		dynamic_reconfigure::Server<bluefox_mono_ros::DynamicConfConfig>::CallbackType f;
		f = boost::bind(&callback, _1, _2, pDev);
		server.setCallback(f);;

    //setting up RosParameters
    initializeRosParameters(pDev, n);

    /**************** End Ros setup *********************************/
    ROS_INFO_STREAM(" == " << __FUNCTION__ << " - establish access to the statistic properties....");
    //cout << " == " << __FUNCTION__ << " - establish access to the statistic properties...." << endl;
    // establish access to the statistic properties
    Statistics statistics( pDev );
    ROS_INFO_STREAM(" == " << __FUNCTION__ << " - create an interface to the device found....");
    //cout << " == " << __FUNCTION__ << " - create an interface to the device found...." << endl;
    // create an interface to the device found
    FunctionInterface fi( pDev );

    SystemSettings ss( pDev );
    // Pre-fill the capture queue with ALL buffers currently available. In case the acquisition engine is operated
    // manually, buffers can only be queued when they have been queued before the acquisition engine is started as well.
    // Even though there can be more than 1, for this sample we will work with the default capture queue

    int requestResult = DMR_NO_ERROR;
    int requestCount = 0;


    while( ( requestResult = fi.imageRequestSingle() ) == DMR_NO_ERROR )
    {
        ++requestCount;
    }

    if( requestResult != DEV_NO_FREE_REQUEST_AVAILABLE )
    {
        ROS_ERROR_STREAM("Last result: " << requestResult << "(" << ImpactAcquireException::getErrorCodeAsString( requestResult ) << "), ");
        //cout << "Last result: " << requestResult << "(" << ImpactAcquireException::getErrorCodeAsString( requestResult ) << "), ";
    }
    ROS_INFO_STREAM(requestCount << " buffers requested");
    //cout << requestCount << " buffers requested";

    if( ss.requestCount.hasMaxValue() )
    {
      ROS_INFO_STREAM("max request count: " << ss.requestCount.getMaxValue());
        //cout << ", max request count: " << ss.requestCount.getMaxValue();
    }

    ROS_INFO("\nPress <<ENTER>> to end manually the application!!");

    manuallyStartAcquisitionIfNeeded( pDev, fi );
    // run thread loop
    const Request* pRequest = 0;
    const unsigned int timeout_ms = 8000;   // USB 1.1 on an embedded system needs a large timeout for the first image
    int requestNr = -1;
    unsigned int cnt = 0;
    bool boError = false;
    bool rosStop = false;
    while( !boError && !rosStop)
    {
        // wait for results from the default capture queue
        requestNr = fi.imageRequestWaitFor( timeout_ms );
        if( fi.isRequestNrValid( requestNr ) )
        {
            pRequest = fi.getRequest( requestNr );
            if( pRequest->isOK() )
            {
                ++cnt;
                // send current image to Ros by the function sendImageToRos
                sendImageToRos( pRequest, image_placeholder, &bridge_image, cnt, &camera_info_mgr, &image_publisher);

                // here we can display some statistical information every 100th image
                if( cnt % 100 == 0 )
                {

                    ROS_INFO("Info From %s :\n->Image count: %d \n->%s : %s \n->%s : %s \n->%s : %s \n->Image dimension: %d x %d , format: %s , line pitch : %d\n", (pDev->serial.read()).c_str(),
                       cnt, (statistics.framesPerSecond.name()).c_str(), statistics.framesPerSecond.readS().c_str(), (statistics.errorCount.name()).c_str(), statistics.errorCount.readS().c_str(),
                        (statistics.captureTime_s.name()).c_str(), statistics.captureTime_s.readS().c_str(), pRequest->imageWidth.read(), pRequest->imageHeight.read(), pRequest->imagePixelFormat.readS().c_str(),
                          pRequest->imageLinePitch.read() );

                }
            }
            else
            {
                ROS_ERROR_STREAM("Liveloop -> Error: request not OK, result : " << pRequest->requestResult << endl);
                //cout << "*** Error: request not OK, result: " << pRequest->requestResult << endl;
                boError = true;
            }

            // this image has been displayed thus the buffer is no longer needed...
            fi.imageRequestUnlock( requestNr );
            // send a new image request into the capture queue
            fi.imageRequestSingle();
        }
        else
        {
            // If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide
            // additional information under TDMR_ERROR in the interface reference
            ROS_ERROR( "Liveloop -> imageRequestWaitFor failed ( %d, %s ), timeout value too small?",
              requestNr, ImpactAcquireException::getErrorCodeAsString( requestNr ).c_str() );

            boError = true;
        }

        if( waitForInput( 0, STDOUT_FILENO ) != 0 )
        {
            ROS_INFO("Liveloop -> finished by user request to stop\n");
            break;
        }

        if(!n.ok()){

          cout << "Liveloop -> finished by ctrl-c node ended" << endl;
          rosStop = true;
          // free resources
          fi.imageRequestReset( 0, 0 );
          return rosStop;

        }

    }
    if( boError )
    {
          ROS_ERROR_STREAM("== "<< __FUNCTION__ << " finished by error - " << endl);
        //cout << " == " << __FUNCTION__ << " finished by error - " << endl;
    }


    manuallyStopAcquisitionIfNeeded( pDev, fi );

    cout << "Liveloop -> free resources...." << endl;
    // free resources
    fi.imageRequestReset( 0, 0 );
    return boError;
}


//-----------------------------------------------------------------------------
bool configureDevice( Device* pDev )
//-----------------------------------------------------------------------------
{

  string settingName;
  int width = 1280;
  int height = 1024;
  /** 'PixelFormat' defines a dictionary. Valid values are:
  [17301512]: BayerGR8
  [17825804]: BayerGR10
  [17825808]: BayerGR12
  [17825838]: BayerGR16
  [35127316]: RGB8Packed
  [35127317]: BGR8Packed
  [35651607]: BGRA8Packed
  [35651613]: BGR10V2Packed
  [34603039]: YUV422Packed
  [34603058]: YUV422_YUYVPacked
  [35127328]: YUV444Packed
  [0]: RGB8
  [1]: BGR8
  [2]: BGRa8
  [3]: RGB10p32
  [6]: YUV422_8_UYVY
  [7]: YUV422_8
  [8]: YUV8_UYV
  [34340894]: YUV411_8_UYYVYY
  [17563735]: BayerGR12p **/
  string pixelFormat="BGR8Packed";
  string acquisitionMode= "Continuous";
  /*gamma_LUT_init set the gain of gamma of LUT!!! */
  double gamma_LUT_init = 1.0;

  bool alldone = true;

    try
    {
        // Restore the factory default first in order to make sure nothing is incorrectly configured
        GenICam::UserSetControl usc( pDev );
        conditionalSetEnumPropertyByString( usc.userSetSelector, "Default" );
        const TDMR_ERROR result = static_cast<TDMR_ERROR>( usc.userSetLoad.call() );
        if( result != DMR_NO_ERROR )
        {
            ROS_ERROR("An error occurred while restoring the factory default for device %s \n->(error code: %s).\n", (pDev->serial.read()).c_str(),
              ImpactAcquireException::getErrorCodeAsString( result ).c_str());
        }
        // Auto exposure or an open shutter will not be helpful for this example thus switch it off if possible.
        mvIMPACT::acquire::GenICam::AcquisitionControl ac( pDev );
        conditionalSetEnumPropertyByString( ac.exposureMode, "Timed" );
        /**'ExposureAuto' defines a dictionary. Valid values are:
        [0]: Off
        [1]: Continuous **/
        conditionalSetEnumPropertyByString( ac.exposureAuto, "Off" );

        // Auto gain will not be helpful for this example either thus switch it off if possible.
        mvIMPACT::acquire::GenICam::AnalogControl anctrl( pDev );
        if( anctrl.gainSelector.isValid() )
        {
            // There might be more than a single 'Gain' as a 'GainSelector' is present. Iterate over all
            // 'Gain's that can be configured and switch off every 'Auto' feature detected.
            vector<string> validGainSelectorValues;
            anctrl.gainSelector.getTranslationDictStrings( validGainSelectorValues );
            const vector<string>::size_type cnt = validGainSelectorValues.size();
            for( vector<string>::size_type i = 0; i < cnt; i++ )
            {
                //conditionalSetEnumPropertyByString( ac.gainSelector, validGainSelectorValues[i] );
                conditionalSetEnumPropertyByString( anctrl.gainAuto, "Off" );
            }
        }
        else
        {
            // There is just a single 'Gain' turn off the 'Auto' feature if supported.
            conditionalSetEnumPropertyByString( anctrl.gainAuto, "Off" );
        }

        mvIMPACT::acquire::GenICam::ImageFormatControl ifc( pDev );

        if( width > 0 )
        {
            ifc.width.write( width );
        }
        if( height > 0 )
        {
            ifc.height.write( height );
        }
        if( !pixelFormat.empty() )
        {
            //ifc.pixelFormat.writeS( pixelFormat );
            conditionalSetEnumPropertyByString(ifc.pixelFormat, pixelFormat);
        }
        if( !acquisitionMode.empty() )
        {
            ac.acquisitionMode.writeS( acquisitionMode );
        }
        acquisitionMode = ac.acquisitionMode.readS();

        //calculate offest if the original width = 1280 and height=1024 is different
        int offsetX = ( (1280 - width) / 2 );
        int offsetY = ( (1024 - height) / 2 );
        if(offsetX>0 && ifc.offsetX.isValid() && ifc.offsetX.isWriteable())
        {
          ifc.offsetX.write(offsetX);
          //cout << "offsetX set  to: " << offsetX << endl;
        }
        if(offsetY>0  && ifc.offsetY.isValid() && ifc.offsetY.isWriteable())
        {
          ifc.offsetY.write(offsetY);
          //cout << "offsetY set  to: " << offsetY << endl;
        }


        /*---------------setup some settings for FrameRate---------------*/
        //displayPropertyData( ac.mvAcquisitionFrameRateEnable );
        conditionalSetEnumPropertyByString(ac.mvAcquisitionFrameRateLimitMode, "mvDeviceLinkThroughput");
        conditionalSetEnumPropertyByString(ac.mvAcquisitionFrameRateEnable, "Off");

        ROS_INFO_STREAM("Device set up to " << ifc.pixelFormat.readS() << " " << ifc.width.read() << "x" << ifc.height.read() <<" fps:" <<ac.acquisitionFrameRate.read() << endl);
        //cout << "Device set up to " << ifc.pixelFormat.readS() << " " << ifc.width.read() << "x" << ifc.height.read() <<" fps:" <<ac.acquisitionFrameRate.read() << endl;

        // Chunk mode is needed in order to get back all the information needed to properly check
        // if an image has been taken using the desired parameters.
        GenICam::ChunkDataControl cdc( pDev );
        cdc.chunkModeActive.write( bTrue );
        //  if We want to act fast, thus if e.g. Bayer-images arrive in the system do NOT convert them on the fly as depending
        // on the device speed the host system might be too slow deal with the amount of data
        ImageProcessing ip( pDev );


        /****** Settings for Black and Saturation ********/
        /**'BlackLevelSelector' defines a dictionary. Valid values are:
        [0]: All
        [1]: DigitalAll
        **/
        conditionalSetEnumPropertyByString(anctrl.blackLevelSelector, "Once" );

        conditionalSetProperty(anctrl.blackLevel, 0.00);

        /****** Settings for whiteBalance ********/

        /** 'BalanceWhiteAuto' defines a dictionary. Valid values are:
        [0]: Off
        [1]: Once
        [2]: Continuous
        **/
        conditionalSetEnumPropertyByString(anctrl.balanceWhiteAuto, "Once" );


        //  if We want to act fast, thus if e.g. Bayer-images arrive in the system do NOT convert them on the fly as depending
        // on the device speed the host system might be too slow deal with the amount of data
        // -> SO IF YOU WANT SPEED UNCOMMENT THOSE LINES OF CODE AND COMMENT THE 2 LINES ACTIVE ON THE OTHER BLOCKS IN THIS SECTION :
        //  ...(ip.colorProcessing, "Auto") AND ....(ip.whiteBalance, "User1")!!!!

        /**conditionalSetEnumPropertyByString(ip.colorProcessing, "Raw");
         if( ip.tapSortEnable.isValid() )
        {
            ip.tapSortEnable.write( bFalse );
        } **/



        // note: those 2 parameters works only if pixelformat is BayerGR*

        /** 'ColorProcessing' defines a dictionary. Valid values are:

        [0]: Auto
        [1]: Raw
        [2]: ColorBayer
        [3]: ColorBayerToMono
        [4]: RawToPlanes
        **/
        //conditionalSetEnumPropertyByString(ip.colorProcessing, "Auto");

        /** 'WhiteBalance' defines a dictionary. Valid values are:
        [0]: TungstenLamp
        [1]: HalogenLamp
        [2]: FluorescentLamp
        [3]: DayLight
        [4]: PhotoFlash
        [5]: BlueSky
        [6]: User1
        [7]: User2
        [8]: User3
        [9]: User4 **/
        /**if( ip.whiteBalance.isValid() && ip.whiteBalance.isWriteable() )
            conditionalSetEnumPropertyByString(ip.whiteBalance, "User1");
        **/


        /****** Settings for colorTwist ********/

        /** 'ColorTwistEnable' defines a dictionary. Valid values are:
        [0]: Off
        [1]: On **/
        if( ip.colorTwistEnable.isValid() && ip.colorTwistEnable.isWriteable() )
            conditionalSetEnumPropertyByString(ip.colorTwistEnable, "Off");


        /** 'ColorTwistInputCorrectionMatrixEnable' defines a dictionary. Valid values are:
        [0]: Off
        [1]: On **/
        if( ip.colorTwistInputCorrectionMatrixEnable.isValid() && ip.colorTwistInputCorrectionMatrixEnable.isWriteable() )
            conditionalSetEnumPropertyByString(ip.colorTwistInputCorrectionMatrixEnable, "Off");


        /** use the function below to see the settings available
        displayPropertyData( ip.colorTwistInputCorrectionMatrixMode ); **/
        if( ip.colorTwistInputCorrectionMatrixMode.isValid() && ip.colorTwistInputCorrectionMatrixMode.isWriteable() )
            conditionalSetEnumPropertyByString(ip.colorTwistInputCorrectionMatrixMode, "DeviceSpecific");



        /**'ColorTwistOutputCorrectionMatrixEnable' defines a dictionary. Valid values are:
        [0]: Off
        [1]: On **/
        if( ip.colorTwistOutputCorrectionMatrixEnable.isValid() && ip.colorTwistOutputCorrectionMatrixEnable.isWriteable() )
            conditionalSetEnumPropertyByString(ip.colorTwistOutputCorrectionMatrixEnable, "Off");


        /**'ColorTwistOutputCorrectionMatrixMode' defines a dictionary. Valid values are:
        [0]: User
        [1]: XYZToAdobeRGB_D50
        [2]: XYZTosRGB_D50
        [3]: XYZToWideGamutRGB_D50
        [4]: XYZToAdobeRGB_D65
        [5]: XYZTosRGB_D65 **/
        if( ip.colorTwistOutputCorrectionMatrixMode.isValid() && ip.colorTwistOutputCorrectionMatrixMode.isWriteable() )
            conditionalSetEnumPropertyByString( ip.colorTwistOutputCorrectionMatrixMode, "XYZToAdobeRGB_D50" );


        /****** Settings for LUT Settings ********/

        /** 'LUTEnable' defines a dictionary. Valid values are:
        [1]: On
        [0]: Off **/
        if( ip.LUTEnable.isValid() && ip.LUTEnable.isWriteable() )
        {
            conditionalSetEnumPropertyByString(ip.LUTEnable, "On");


          /**'LUTImplementation' defines a dictionary. Valid values are:
          [1]: Software **/
          if( ip.LUTImplementation.isValid() && ip.LUTImplementation.isWriteable() )
            conditionalSetEnumPropertyByString( ip.LUTImplementation, "Software");



          /** 'LUTMappingSoftware' defines a dictionary. Valid values are:
          Note--> values must match with the bit of pixelformat!!!!!!
          [524296]: 8To8
          [655370]: 10To10
          [786444]: 12To12
          [917518]: 14To14
          [1048592]: 16To16 **/
          if( ip.LUTMappingSoftware.isValid() && ip.LUTMappingSoftware.isWriteable() )
            conditionalSetEnumPropertyByString(ip.LUTMappingSoftware, "8To8");


          /** 'LUTMode' defines a dictionary. Valid values are:
          [0]: Interpolated
          [1]: Gamma
          [2]: Direct **/
          if( ip.LUTMode.isValid() && ip.LUTMode.isWriteable() )
            conditionalSetEnumPropertyByString(ip.LUTMode, "LUTmGamma");

          if( ip.LUTEnable.isValid() && ip.LUTEnable.isWriteable() )
          {
            const unsigned int LUTCnt = ip.getLUTParameterCount();

            for( unsigned int i = 0; i < LUTCnt; i++ )
            {
               mvIMPACT::acquire::LUTParameters& lpm = ip.getLUTParameter( i );
                lpm.gamma.write(gamma_LUT_init);
            }

            /**for( unsigned int i = 0; i < LUTCnt; i++ )
            {
               mvIMPACT::acquire::LUTParameters& lpm = ip.getLUTParameter( i );
               cout<< "gamma :" << lpm.gamma.read() << endl;
               cout <<"gammaMode :" << lpm.gammaMode.readS() << endl;
            } **/
          }
        }
      }

      catch( const ImpactAcquireException& e )

      {

          // This e.g. might happen if the same device is already opened in another process...
          ROS_ERROR_STREAM("== " << __FUNCTION__ << "An error occurred while configuring the device " << pDev->serial.read()
                    << "(error code: " << e.getErrorCodeAsString() << ")." << endl);
          /**cout << "== " << __FUNCTION__ << "An error occurred while configuring the device " << pDev->serial.read()
                    << "(error code: " << e.getErrorCodeAsString() << ")." << endl;**/

        alldone = false;
      }


      return alldone;

  }

//-----------------------------------------------------------------------------
int main( int argc, char* argv[] )
//-----------------------------------------------------------------------------
{
    ROS_INFO("++ starting application....");
    //cout << " ++ starting application...." << endl;

    //Init ros
    ros::init(argc,argv, "bluefox_camera");

    // settings base for init device
    string deviceSerial = "";
    int defaultRequestCount = -1;


    DeviceManager devMgr ;
    Device* pDev = 0;
    bool bGoOn( true );
    while( bGoOn )
    {
        if( !deviceSerial.empty() )
        {
            pDev = devMgr.getDeviceBySerial( deviceSerial );
            if( pDev )
            {
                // if this device offers the 'GenICam' interface switch it on, as this will
                // allow are better control over GenICam compliant devices
                conditionalSetProperty( pDev->interfaceLayout, dilGenICam );
                // if this device offers a user defined acquisition start/stop behaviour
                // enable iapps\CameraDescriptions\t as this allows finer control about the streaming behaviour
                conditionalSetProperty( pDev->acquisitionStartStopBehaviour, assbUser );
            }
        }
        if( !pDev )
        {
            // this will automatically set the interface layout etc. to the values from the branch above
            pDev = getDeviceFromUserInput( devMgr );
        }
        if( pDev )
        {
            deviceSerial = pDev->serial.read();
            ROS_INFO_STREAM("Initialising device: " << pDev->serial.read() << ". This might take some time..." << endl);
            ROS_INFO_STREAM("Using interface layout '" << pDev->interfaceLayout.readS() << "'." << endl);
            /**cout << "Initialising device: " << pDev->serial.read() << ". This might take some time..." << endl
                 << "Using interface layout '" << pDev->interfaceLayout.readS() << "'." << endl;**/
            try
            {
                if( defaultRequestCount > 0 )
                {
                    ROS_INFO_STREAM("Setting default request count to " << defaultRequestCount << endl;
                    pDev->defaultRequestCount.write( defaultRequestCount));
                    /**cout << "Setting default request count to " << defaultRequestCount << endl;
                    pDev->defaultRequestCount.write( defaultRequestCount );**/
                }
                pDev->open();

                // try to configure the device
                bGoOn = configureDevice( pDev);

                // start the execution of the 'live' loop.
                ROS_INFO("starting live loop");
                //cout << "starting live loop" << endl;
                if( liveLoop( pDev ) )  // stop for user input
                {
                    bGoOn = false;
                }
                ROS_INFO("finished live loop");
                //cout << "finished live loop" << endl;
                pDev->close();
            }
            catch( const ImpactAcquireException& e )
            {
                // this e.g. might happen if the same device is already opened in another process...
                ROS_ERROR_STREAM("==" << __FUNCTION__ << " - An error occurred while opening the device " << pDev->serial.read()
                     << "(error code: " << e.getErrorCode() << ", " << e.getErrorCodeAsString() << "). Press any key to end the application..." << endl);
                /**cout << "==" << __FUNCTION__ << " - An error occurred while opening the device " << pDev->serial.read()
                     << "(error code: " << e.getErrorCode() << ", " << e.getErrorCodeAsString() << "). Press any key to end the application..." << endl;**/
            }
        }
        else
        {
            ROS_ERROR("Unable to get device!\n");
            //cout << "Unable to get device!";
            bGoOn = false;
        }

        if( waitForInput( 0, STDOUT_FILENO ) != 0 )
        {
            break;
        }
    }
    ROS_INFO(" -- ending application....\n");
    //cout << " -- ending application...." << endl;
    return 0;
}
