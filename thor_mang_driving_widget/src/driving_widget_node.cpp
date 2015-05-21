#include <ros/ros.h>
#include <QtGui/QApplication>
#include "driving_widget.h"

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
      ros::init( argc, argv, "driving_widget", ros::init_options::AnonymousName );
    }
    QApplication a(argc, argv);
    DrivingWidget w;
    w.show();

    return a.exec();
}
