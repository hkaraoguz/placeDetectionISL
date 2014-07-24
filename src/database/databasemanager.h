#ifndef DATABASEMANAGER_H
#define DATABASEMANAGER_H

#include "bubbleprocess.h"
#include <opencv2/opencv.hpp>
#include <QObject>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlError>
#include <QFile>
#include <QDir>

//#define DB_PATH "/home/hakan/Development/ISL/Datasets/ImageClef2012/training3v2.db"
//#define INVARIANTS_DB_PATH "/home/hakan/Development/ISL/Datasets/ImageClef2012/invariants.db"

#define LASER_TYPE 55
#define HUE_TYPE 56
#define SAT_TYPE 57
#define VAL_TYPE 58

class Place;
class TemporalWindow;
class BasePoint;

class DatabaseManager : public QObject
{
    Q_OBJECT
public:
    explicit DatabaseManager(QObject *parent = 0);
  //  ~DatabaseManager();

   static bool openDB(QString filePath);

   static void closeDB();

   static bool deleteDB();

   static bool isOpen();

    // Type = 55:Lazer, 56:Hue, XX:Filter_Number, Number: the current bubble number-> obtained from frame number
   static bool insertRowtoBubble(int type, int number, bubblePoint row);

   static bool insertRowtoBubble(int type, int number, int pan, int tilt, double val);

   static bool insertBubble(int type,int number, std::vector<bubblePoint> bubble);

   static bool insertInvariants(int type,int number, std::vector< std::vector<float> > invariants);

   static bool insertBubbleStatistics(int type, int number, bubbleStatistics stats);

   static std::vector<bubblePoint> readBubble(int type, int number);

   static void determinePlaceLabels(QString filePath);

   static bool insertTemporalWindow(const TemporalWindow& twindow);

   static bool insertBasePoint(const BasePoint& basepoint);




   static bool insertPlace(const Place& place);

   static cv::Mat getPlaceMeanInvariant(int id);

   static Place getPlace(int id);

   static cv::Mat getPlaceMemberIds(int id);

   static QByteArray mat2ByteArray(const cv::Mat &image);

   static cv::Mat byteArray2Mat(const QByteArray & byteArray);

   QSqlError lastError();

private:
 //  QSqlDatabase db;

    
signals:
    
public slots:
    
};

#endif // DATABASEMANAGER_H
