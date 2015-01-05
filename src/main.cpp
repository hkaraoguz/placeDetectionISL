#include "bubble/bubbleprocess.h"
#include "imageprocess/imageprocess.h"
#include "database/databasemanager.h"
#include "Utility.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <QDir>
#include <QDebug>

namespace enc = sensor_msgs::image_encodings;

double compareHistHK( InputArray _H1, InputArray _H2, int method );

double compareHKCHISQR(cv::Mat input1, cv::Mat input2);


// TODO Temporal Window ve basepointleri DB ye kaydedecegiz


/*class UninformativeFrame
{
public:
    UninformativeFrame();
    int sat_mean;
    int sat_var;
    int frame_no;


};
UninformativeFrame*/
TemporalWindow::TemporalWindow()
{
    startPoint = 0;
    endPoint = 0;
    tau_w = 0;
    tau_n = 0;
    id = -1;

}

ros::Timer timer;
PlaceDetector detector;
DatabaseManager dbmanager;

void timerCallback(const ros::TimerEvent& event)
{
    detector.shouldProcess = false;
    detector.processImage();



}

void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{

    if(detector.shouldProcess)
    {

        detector.currentImage = cv_bridge::toCvCopy(original_image, enc::BGR8)->image;


    }

    //cv::imshow("win",image);

    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    //cv::waitKey(3);


}

double compareHKCHISQR(Mat input1, Mat input2)
{
    double res = -1;

    if(input1.rows != input2.rows)
    {
        qDebug()<<"Comparison failed due to col size mismatch";
        return res;
    }
    double summ  = 0;
    for(int i = 0; i < input1.rows; i++)
    {
        float in1 = input1.at<float>(i,0);
        float in2 = input2.at<float>(i,0);

        double mul = (in1-in2)*(in1-in2);

        double ss =  in1+in2;
        summ += mul/ss;
        qDebug()<<i<<mul<<ss<<summ;

    }

    return summ;
}

void writeInvariant(cv::Mat inv, int count)
{
    QString pathh = QDir::homePath();
    pathh.append("/invariants_").append(QString::number(count)).append(".txt");
    QFile file(pathh);

    if(file.open(QFile::WriteOnly))
    {
        QTextStream str(&file);

        for(int i = 0; i < inv.rows; i++)
        {
            str<<inv.at<float>(i,0)<<"\n";

        }

        file.close();
    }

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "placeDetectionISL");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    image_transport::ImageTransport it(nh);
    image_transport::TransportHints hints("compressed");

    detector.tau_w = 1;
    detector.tau_n = 1;
    detector.tau_p = 20;
    std::string camera_topic = "";
    int img_width = 640;
    int img_height = 480;
    detector.focalLengthPixels = 525;

    detector.satLower = 30;
    detector.satUpper = 230;
    detector.valLower = 30;
    detector.valUpper = 230;

    detector.noHarmonics = 10;
    detector.image_counter = 1;
    detector.shouldProcess = true;

    pnh.getParam("tau_w",detector.tau_w);
    pnh.getParam("tau_n",detector.tau_n);
    pnh.getParam("tau_p",detector.tau_p);
    pnh.getParam("tau_inv",detector.tau_inv);
    pnh.getParam("camera_topic",camera_topic);
    pnh.getParam("image_width",img_width);
    pnh.getParam("image_height",img_height);
    pnh.getParam("focal_length_pixels",detector.focalLengthPixels);
    pnh.getParam("tau_val_mean",detector.tau_val_mean);
    pnh.getParam("tau_val_var",detector.tau_val_var);


   // dbmanager.openDB("/home/hakan/Development/ISL/Datasets/Own/deneme/db1.db");

    /*Place place = DatabaseManager::getPlace(1);

    qDebug()<<place.meanInvariant.at<float>(500,0)<<place.memberIds.at<int>(50,0);*/

    // This should be done before starting the process
    bubbleProcess::calculateImagePanAngles(detector.focalLengthPixels,img_width,img_height);
    bubbleProcess::calculateImageTiltAngles(detector.focalLengthPixels,img_width,img_height);

    QString basepath = QDir::homePath();
    basepath.append("/visual_filters");

    QString path(basepath);

    path.append("/filtre0.txt");
    qDebug()<<path;

    ImageProcess::readFilter(path,29,false,false,false);

    path.clear();
    path = basepath;

    path.append("/filtre6.txt");
    qDebug()<<path;

    ImageProcess::readFilter(path,29,false,false,false);

    path.clear();
    path = basepath;

    path.append("/filtre12.txt");
    qDebug()<<path;


    ImageProcess::readFilter(path,29,false,false,false);

    path.clear();
    path = basepath;

    path.append("/filtre18.txt");
    qDebug()<<path;


    ImageProcess::readFilter(path,29,false,false,false);

    path.clear();
    path = basepath;


    path.append("/filtre36.txt");
    qDebug()<<path;

    ImageProcess::readFilter(path,29,false,false,false);


    image_transport::Subscriber imageSub = it.subscribe(camera_topic.data(), 1, imageCallback,hints);



    //  timer = nh.createTimer(ros::Duration(0.25), timerCallback);

    //ros::Publisher pub = nh.advertise<std_msgs::String>("hello",5);

    // Create a ROS subscriber for the input point cloud
    //ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

    // QDir dirr("/home/hakan/Development/ISL/Datasets/Own/jaguar/tour1");

    /*dirr.setSorting(QDir::LocaleAware);

    QStringList filter("*.jpg");*/

    /*   QStringList list = dirr.entryList(filter,QDir::Files);

    qSort(list.begin(), list.end(), qGreater<QString>());*/

    //list.sort();

    //  qDebug()<<list;

    ros::Rate loop(50);

    for(int i = 951; i <= 952; i++)
    {

        QString path("/home/hakan/Development/ISL/Datasets/Own/jaguar/tour1/");

        path.append("rgb_").append(QString::number(i)).append(".jpg");

        detector.currentImage = imread(path.toStdString().data(),CV_LOAD_IMAGE_COLOR);
        detector.shouldProcess = true;
        detector.processImage();


    }

    qDebug()<<"New Place";
    detector.currentPlace->calculateMeanInvariant();

    // qDebug()<<"Current place mean invariant: "<<currentPlace->meanInvariant.rows<<currentPlace->meanInvariant.cols<<currentPlace->meanInvariant.at<float>(50,0);

    if(detector.currentPlace->memberIds.rows >= detector.tau_p){
        dbmanager.insertPlace(*detector.currentPlace);
        detector.detectedPlaces.push_back(*detector.currentPlace);
        detector.placeID++;
    }


    delete detector.currentPlace;
    detector.currentPlace = 0;


    //timer.start();

    while(ros::ok())
    {


        ros::spinOnce();

        loop.sleep();






        /*  if(detector.shouldProcess)
        {
            detector.shouldProcess = false;
            //     qDebug()<<"We are here";
            detector.processImage();

        }*/



    }


    //  timer.stop();

    dbmanager.closeDB();

    ros::shutdown();
    return 0;

}

void PlaceDetector::processImage()
{
    if(!currentImage.empty())
    {
        /*  int satLower =  30;

        int satUpper =  230;

        int valLower =  30;

        int valUpper = 230;*/


        timer.stop();

        //  Mat img;

        //  cv::cvtColor(currentImage,img,CV_BGR2HSV);

        Mat hueChannel= ImageProcess::generateChannelImage(currentImage,0,satLower,satUpper,valLower,valUpper);
        vector<bubblePoint> hueBubble = bubbleProcess::convertGrayImage2Bub(hueChannel,focalLengthPixels,180);
        vector<bubblePoint> reducedHueBubble = bubbleProcess::reduceBubble(hueBubble);



        /************************** Perform filtering and obtain resulting mat images ***********************/
        //  Mat satChannel= ImageProcess::generateChannelImage(img,1,satLower,satUpper,valLower,valUpper);
        Mat valChannel= ImageProcess::generateChannelImage(currentImage,2,satLower,satUpper,valLower,valUpper);
        /*****************************************************************************************************/

        /*************************** Convert images to bubbles ***********************************************/

        //  vector<bubblePoint> satBubble = bubbleProcess::convertGrayImage2Bub(satChannel,focalLengthPixels,255);
        vector<bubblePoint> valBubble = bubbleProcess::convertGrayImage2Bub(valChannel,focalLengthPixels,255);

        /*****************************************************************************************************/


        /***************** Reduce the bubbles ********************************************************/
        //   vector<bubblePoint> reducedSatBubble = bubbleProcess::reduceBubble(satBubble);
        vector<bubblePoint> reducedValBubble = bubbleProcess::reduceBubble(valBubble);


        // Calculate statistics
        //  bubbleStatistics statsHue =  bubbleProcess::calculateBubbleStatistics(reducedHueBubble,180);
        // bubbleStatistics statsSat =  bubbleProcess::calculateBubbleStatistics(reducedSatBubble,255);
        bubbleStatistics statsVal =  bubbleProcess::calculateBubbleStatistics(reducedValBubble,255);

        qDebug()<<"Bubble statistics: "<<statsVal.mean<<statsVal.variance;
        currentBasePoint.avgVal = statsVal.mean;
        currentBasePoint.varVal = statsVal.variance;
        currentBasePoint.id = image_counter;
        currentBasePoint.status = 0;

        /*********************** WE CHECK FOR THE UNINFORMATIVENESS OF THE FRAME   *************************/
        if(statsVal.mean <= this->tau_val_mean || statsVal.variance <= this->tau_val_var)
        {

            //   qDebug()<<"This image is uninformative"<<image_counter;

            currentBasePoint.status = 1;

            this->shouldProcess = true;


            // If we don't have an initialized window then initialize
            if(!this->tempwin)
            {
                this->tempwin = new TemporalWindow();
                this->tempwin->tau_n = this->tau_n;
                this->tempwin->tau_w = this->tau_w;
                this->tempwin->startPoint = image_counter;
                this->tempwin->endPoint = image_counter;
                this->tempwin->id = twindow_counter;

                this->tempwin->members.push_back(currentBasePoint);

            }
            else
            {

                this->tempwin->endPoint = image_counter;

                this->tempwin->members.push_back(currentBasePoint);

            }


            dbmanager.insertBasePoint(currentBasePoint);
            //  previousBasePoint = currentBasePoint;

            image_counter++;

            timer.start();

            return;


        }
        /***********************************  IF THE FRAME IS INFORMATIVE *************************************************/
        else
        {
            Mat totalInvariants;

            DFCoefficients dfcoeff = bubbleProcess::calculateDFCoefficients(reducedHueBubble,noHarmonics,noHarmonics);
            Mat hueInvariants = bubbleProcess::calculateInvariantsMat(dfcoeff,noHarmonics, noHarmonics);

            totalInvariants = hueInvariants.clone();


            cv::Mat logTotal;

            // qDebug()<<hueInvariants.rows<<hueInvariants.cols<<hueInvariants.at<float>(0,10);

            Mat grayImage;

            cv::cvtColor(currentImage,grayImage,CV_BGR2GRAY);

            std::vector<Mat> sonuc = ImageProcess::applyFilters(grayImage);

            for(uint j = 0; j < sonuc.size(); j++)
            {
                vector<bubblePoint> imgBubble = bubbleProcess::convertGrayImage2Bub(sonuc[j],focalLengthPixels,255);

                vector<bubblePoint> resred = bubbleProcess::reduceBubble(imgBubble);

                DFCoefficients dfcoeff =  bubbleProcess::calculateDFCoefficients(resred,noHarmonics,noHarmonics);

                Mat invariants=  bubbleProcess::calculateInvariantsMat(dfcoeff,noHarmonics,noHarmonics);

                cv::hconcat(totalInvariants, invariants, totalInvariants);


            }

            // TOTAL INVARIANTS 1 X N vector

            //  qDebug()<<totalInvariants.at<float>(0,64);

            cv::log(totalInvariants,logTotal);
            logTotal = logTotal/25;
            cv::transpose(logTotal,logTotal);



            //   qDebug()<<logTotal.rows<<logTotal.cols<<logTotal.at<float>(10,0);

            // We don't have a previous base point
            if(previousBasePoint.id == 0)
            {
                currentBasePoint.id = image_counter;
                currentBasePoint.invariants = logTotal;
                previousBasePoint = currentBasePoint;


                currentPlace->members.push_back(currentBasePoint);

                dbmanager.insertBasePoint(currentBasePoint);

            }
            else
            {

                currentBasePoint.id = image_counter;
                currentBasePoint.invariants = logTotal;

                double result = compareHistHK(currentBasePoint.invariants,previousBasePoint.invariants, CV_COMP_CHISQR);

                double result2= compareHKCHISQR(currentBasePoint.invariants,previousBasePoint.invariants);

                qDebug()<<"Invariant diff between "<<currentBasePoint.id<<previousBasePoint.id<<"is"<<result<<result2;//previousBasePoint.invariants.rows<<currentBasePoint.invariants.rows;

                qDebug()<<currentBasePoint.invariants.at<float>(1,0)<<currentBasePoint.invariants.at<float>(2,0)<<previousBasePoint.invariants.at<float>(1,0)<<previousBasePoint.invariants.at<float>(2,0);

                writeInvariant(previousBasePoint.invariants,previousBasePoint.id);
                ///////////////////////////// IF THE FRAMES ARE COHERENT ///////////////////////////////////////////////////////////////////////////////////////////////////////
                if(result <= tau_inv)
                {

                    dbmanager.insertBasePoint(currentBasePoint);

                    /// If we have a temporal window
                    if(tempwin)
                    {
                        // Temporal window will extend, we are still looking for the next incoming frames
                        if(tempwin->checkExtensionStatus(currentBasePoint.id))
                        {

                            tempwin->cohMembers.push_back(currentBasePoint);


                            basepointReservoir.push_back(currentBasePoint);


                        }
                        // Temporal window will not extend anymore, we should check whether it is really a temporal window or not
                        else
                        {

                            // This is a valid temporal window
                            if(tempwin->endPoint - tempwin->startPoint >= tau_w)
                            {
                                qDebug()<<"New Place";
                                currentPlace->calculateMeanInvariant();

                                qDebug()<<"Current place mean invariant: "<<currentPlace->meanInvariant.rows<<currentPlace->meanInvariant.cols<<currentPlace->meanInvariant.at<float>(50,0);

                                if(currentPlace->memberIds.rows >= tau_p){
                                    dbmanager.insertPlace(*currentPlace);
                                    this->detectedPlaces.push_back(*currentPlace);
                                    this->placeID++;
                                }


                                delete currentPlace;
                                currentPlace = 0;
                                // this->placeID++;

                                /*    cv::Mat result = DatabaseManager::getPlaceMeanInvariant(this->placeID-1);

                                qDebug()<<"Previous place mean invariant: "<<result.rows<<result.cols<<result.at<float>(50,0);

                                result = DatabaseManager::getPlaceMemberIds(this->placeID-1);

                                for(int k = 0; k< result.rows; k++){

                                    qDebug()<<"Previous place members: "<<result.rows<<result.cols<<result.at<unsigned short>(k,0);
                                }*/

                                currentPlace = new Place(this->placeID);

                                //   currentPlace->


                                basepointReservoir.push_back(currentBasePoint);

                                currentPlace->members = basepointReservoir;
                                basepointReservoir.clear();

                                dbmanager.insertTemporalWindow(*tempwin);

                                delete tempwin;
                                tempwin = 0;
                                this->twindow_counter++;
                                // A new place will be created. Current place will be published




                            }
                            // This is just a noisy temporal window. We should add the coherent basepoints to the current place
                            else
                            {
                                basepointReservoir.push_back(currentBasePoint);

                                delete tempwin;
                                tempwin = 0;

                                std::vector<BasePoint> AB;
                                AB.reserve( currentPlace->members.size() + basepointReservoir.size() ); // preallocate memory
                                AB.insert( AB.end(), currentPlace->members.begin(), currentPlace->members.end() );
                                AB.insert( AB.end(), basepointReservoir.begin(), basepointReservoir.end() );
                                currentPlace->members.clear();
                                currentPlace->members = AB;

                                basepointReservoir.clear();



                            }



                        }



                    }
                    else
                    {
                        currentPlace->members.push_back(currentBasePoint);

                    }



                }
                ///////////////////////// IF THE FRAMES ARE INCOHERENT /////////////////////////////////////
                else
                {
                    currentBasePoint.status = 2;
                    dbmanager.insertBasePoint(currentBasePoint);
                    // If we don't have a temporal window create one
                    if(!tempwin)
                    {
                        tempwin = new TemporalWindow();
                        this->tempwin->tau_n = this->tau_n;
                        this->tempwin->tau_w = this->tau_w;
                        this->tempwin->startPoint = image_counter;
                        this->tempwin->endPoint = image_counter;
                        this->tempwin->id = twindow_counter;

                        this->tempwin->members.push_back(currentBasePoint);

                    }
                    // add the basepoint to the temporal window
                    else
                    {
                        // Temporal window will extend, we are still looking for the next incoming frames
                        if(tempwin->checkExtensionStatus(currentBasePoint.id))
                        {

                            this->tempwin->endPoint = image_counter;

                            this->tempwin->members.push_back(currentBasePoint);

                            basepointReservoir.clear();


                        }
                        else
                        {
                            // This is a valid temporal window
                            if(tempwin->endPoint - tempwin->startPoint >= tau_w)
                            {
                                qDebug()<<"New Place";
                                currentPlace->calculateMeanInvariant();

                                qDebug()<<"Current place mean invariant: "<<currentPlace->meanInvariant.rows<<currentPlace->meanInvariant.cols<<currentPlace->meanInvariant.at<float>(50,0);

                                if(currentPlace->memberIds.rows >= tau_p){
                                    dbmanager.insertPlace(*currentPlace);
                                    this->detectedPlaces.push_back(*currentPlace);
                                    this->placeID++;
                                }


                                delete currentPlace;
                                currentPlace = 0;
                                // this->placeID++;

                                //  cv::Mat result = DatabaseManager::getPlaceMeanInvariant(this->placeID-1);

                                //  qDebug()<<"Previous place mean invariant: "<<result.rows<<result.cols<<result.at<float>(50,0);

                                //  result = DatabaseManager::getPlaceMemberIds(this->placeID-1);

                                /*    for(int k = 0; k< result.rows; k++){

                                    qDebug()<<"Previous place members: "<<result.rows<<result.cols<<result.at<unsigned short>(k,0);
                                }*/

                                currentPlace = new Place(this->placeID);

                                //   currentPlace->


                                //basepointReservoir.push_back(currentBasePoint);

                                currentPlace->members = basepointReservoir;
                                basepointReservoir.clear();

                                dbmanager.insertTemporalWindow(*tempwin);

                                delete tempwin;
                                tempwin = 0;
                                this->twindow_counter++;
                                // A new place will be created. Current place will be published




                            }
                            // This is just a noisy temporal window. We should add the coherent basepoints to the current place
                            else
                            {
                                //  basepointReservoir.push_back(currentBasePoint);

                                delete tempwin;
                                tempwin = 0;

                                std::vector<BasePoint> AB;
                                AB.reserve( currentPlace->members.size() + basepointReservoir.size() ); // preallocate memory
                                AB.insert( AB.end(), currentPlace->members.begin(), currentPlace->members.end() );
                                AB.insert( AB.end(), basepointReservoir.begin(), basepointReservoir.end() );
                                currentPlace->members.clear();
                                currentPlace->members = AB;

                                basepointReservoir.clear();



                            }


                            tempwin = new TemporalWindow();
                            this->tempwin->tau_n = this->tau_n;
                            this->tempwin->tau_w = this->tau_w;
                            this->tempwin->startPoint = image_counter;
                            this->tempwin->endPoint = image_counter;
                            this->tempwin->id = twindow_counter;

                            this->tempwin->members.push_back(currentBasePoint);



                        }


                        /*     this->tempwin->endPoint = image_counter;

                          this->tempwin->members.push_back(currentBasePoint);

                          basepointReservoir.clear();*/

                    }



                }


                previousBasePoint = currentBasePoint;

                //////////////////////////////////////////////////////////////////////////////////////////////////
            }

            // DatabaseManager::insertInvariants(HUE_TYPE,frameNumber,invariants);
            //   qDebug()<<"Image Counter: "<<image_counter;
            image_counter++;

            //this->shouldProcess = true;

        }



    }

    this->shouldProcess = true;

    timer.start();



}
PlaceDetector::PlaceDetector()
{

    this->tempwin = 0;
    this->currentBasePoint.id = 0;
    this->previousBasePoint.id = 0;
    this->placeID = 1;
    currentPlace = new Place(this->placeID);
    this->twindow_counter = 1;



}


bool TemporalWindow::checkExtensionStatus(uint currentID)
{
    if(currentID - this->endPoint <= tau_n)
    {

        return true;

    }

    return false;



}

double compareHistHK( InputArray _H1, InputArray _H2, int method )
{
    Mat H1 = _H1.getMat(), H2 = _H2.getMat();
    const Mat* arrays[] = {&H1, &H2, 0};
    Mat planes[2];
    NAryMatIterator it(arrays, planes);
    double result = 0;
    int j, len = (int)it.size;

    CV_Assert( H1.type() == H2.type() && H1.depth() == CV_32F );

    double s1 = 0, s2 = 0, s11 = 0, s12 = 0, s22 = 0;

    CV_Assert( it.planes[0].isContinuous() && it.planes[1].isContinuous() );

    for( size_t i = 0; i < it.nplanes; i++, ++it )
    {
        const float* h1 = (const float*)it.planes[0].data;
        const float* h2 = (const float*)it.planes[1].data;
        len = it.planes[0].rows*it.planes[0].cols*H1.channels();


        for( j = 0; j < len; j++ )
        {
            double a = h1[j] - h2[j];
            double b =  h1[j] + h2[j];
            if( fabs(b) > DBL_EPSILON )
                result += a*a/b;
        }

    }

    return result;

}
